#include "dave_gz_model_plugins/HeaveAugmentPlugin.hh"
#include "dave_gz_model_plugins/heave_augment_core.hh"

// Gazebo includes
#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

// Standard library includes
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

GZ_ADD_PLUGIN(
  dave_gz_model_plugins::HeaveAugmentPlugin, gz::sim::System,
  dave_gz_model_plugins::HeaveAugmentPlugin::ISystemConfigure,
  dave_gz_model_plugins::HeaveAugmentPlugin::ISystemPreUpdate)

namespace dave_gz_model_plugins
{
namespace core = heave_augment;

struct HeaveAugmentPlugin::PrivateData
{
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};

  // Must mirror the Hydrodynamics plugin's zW/zWabsW (both render from the
  // same rig.hydrodynamics values, biofouling multipliers included).
  double zW{0.0};
  double zWabsW{0.0};

  // <ascent_drag_relief>
  double retainFraction{1.0};  // 1.0 = exact no-op
  double minAscentSpeed{0.01};

  // <entry_momentum>
  double peakSpeed{0.0};  // <= 0 = exact no-op
  double riseTime{20.0};
  double decayTime{25.0};
  double servoGain{1000.0};
  double maxForce{60.0};
  std::optional<core::EntryTrigger> trigger;

  // Entry-servo suppression, driven over gz-transport by the sim_ready_gate's
  // pre-mission physics-liveness probe: the probe's commanded sink from the
  // surface float (>= trigger_depth, >= trigger_speed) is indistinguishable
  // from a real leg entry, and a 60 N servo firing against the probe's ~2-4 N
  // restore buoyancy would drag the hull metres deep. While suppressed the
  // trigger is not stepped and no entry force is applied; the ascent relief
  // stays live (it is stateless). Written from a transport thread, read in
  // PreUpdate — hence the atomic.
  gz::transport::Node node;
  std::atomic<bool> entrySuppressed{false};
  bool prevSuppressed{false};

  // The documented no-op contract: retain_fraction 1.0 / peak_speed <= 0.
  bool ReliefOn() const { return retainFraction < 1.0; }
  bool EntryOn() const { return peakSpeed > 0.0; }
};

HeaveAugmentPlugin::HeaveAugmentPlugin() : dataPtr(std::make_unique<PrivateData>()) {}

HeaveAugmentPlugin::~HeaveAugmentPlugin() = default;

/////////////////////////////////////////////////
void HeaveAugmentPlugin::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & /*_eventMgr*/)
{
  auto model = gz::sim::Model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "HeaveAugmentPlugin must be attached to a model entity" << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();
  const auto linkName = sdfClone->Get<std::string>("link_name", "base_link").first;
  this->dataPtr->linkEntity = model.LinkByName(_ecm, linkName);
  if (this->dataPtr->linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "HeaveAugmentPlugin: link [" << linkName << "] not found" << std::endl;
    return;
  }
  gz::sim::Link(this->dataPtr->linkEntity).EnableVelocityChecks(_ecm, true);

  this->dataPtr->zW = sdfClone->Get<double>("zW", this->dataPtr->zW).first;
  this->dataPtr->zWabsW = sdfClone->Get<double>("zWabsW", this->dataPtr->zWabsW).first;

  if (sdfClone->HasElement("ascent_drag_relief"))
  {
    auto elem = sdfClone->GetElement("ascent_drag_relief");
    this->dataPtr->retainFraction =
      elem->Get<double>("retain_fraction", this->dataPtr->retainFraction).first;
    this->dataPtr->minAscentSpeed =
      elem->Get<double>("min_ascent_speed", this->dataPtr->minAscentSpeed).first;
  }

  core::EntryTriggerParams triggerParams;
  if (sdfClone->HasElement("entry_momentum"))
  {
    auto elem = sdfClone->GetElement("entry_momentum");
    this->dataPtr->peakSpeed = elem->Get<double>("peak_speed", this->dataPtr->peakSpeed).first;
    this->dataPtr->riseTime = elem->Get<double>("rise_time", this->dataPtr->riseTime).first;
    this->dataPtr->decayTime = elem->Get<double>("decay_time", this->dataPtr->decayTime).first;
    triggerParams.triggerDepth =
      elem->Get<double>("trigger_depth", triggerParams.triggerDepth).first;
    triggerParams.triggerSpeed =
      elem->Get<double>("trigger_speed", triggerParams.triggerSpeed).first;
    triggerParams.triggerHold = elem->Get<double>("trigger_hold", triggerParams.triggerHold).first;
    triggerParams.rearmSpeed = elem->Get<double>("rearm_speed", triggerParams.rearmSpeed).first;
    triggerParams.rearmHold = elem->Get<double>("rearm_hold", triggerParams.rearmHold).first;
    this->dataPtr->servoGain = elem->Get<double>("servo_gain", this->dataPtr->servoGain).first;
    this->dataPtr->maxForce = elem->Get<double>("max_force", this->dataPtr->maxForce).first;
  }
  triggerParams.activeDuration =
    core::entryActiveDuration(this->dataPtr->riseTime, this->dataPtr->decayTime);
  this->dataPtr->trigger.emplace(triggerParams);

  // Suppress control topic, named from the live model (no SDF/jinja knob, so
  // the rendered-vs-canonical SDF parity is untouched). Latching semantics
  // are the publisher's problem; a lost message fails safe — the probe's
  // restore phase times out and the run retries instead of recording garbage.
  const auto suppressTopic =
    "/model/" + model.Name(_ecm) + "/heave_augment/entry_suppress";
  std::function<void(const gz::msgs::Boolean &)> onSuppress =
    [data = this->dataPtr.get()](const gz::msgs::Boolean & _msg)
  { data->entrySuppressed.store(_msg.data()); };
  if (!this->dataPtr->node.Subscribe(suppressTopic, onSuppress))
  {
    gzerr << "HeaveAugmentPlugin: failed to subscribe [" << suppressTopic << "]" << std::endl;
  }

  const bool reliefOn = this->dataPtr->ReliefOn();
  const bool entryOn = this->dataPtr->EntryOn();
  gzmsg << "HeaveAugmentPlugin on [" << linkName << "]: ascent relief "
        << (reliefOn ? "retain_fraction=" + std::to_string(this->dataPtr->retainFraction)
                     : std::string("off"))
        << ", entry momentum "
        << (entryOn ? "peak_speed=" + std::to_string(this->dataPtr->peakSpeed)
                    : std::string("off"))
        << std::endl;
}

/////////////////////////////////////////////////
void HeaveAugmentPlugin::PreUpdate(
  const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm)
{
  if (_info.paused || this->dataPtr->linkEntity == gz::sim::kNullEntity)
  {
    return;
  }
  const bool reliefOn = this->dataPtr->ReliefOn();
  const bool entryOn = this->dataPtr->EntryOn();
  if (!reliefOn && !entryOn)
  {
    return;
  }

  gz::sim::Link link(this->dataPtr->linkEntity);
  const auto pose = link.WorldPose(_ecm);
  const auto velocity = link.WorldLinearVelocity(_ecm);
  if (!pose || !velocity)
  {
    return;
  }

  const double t = std::chrono::duration<double>(_info.simTime).count();
  const double depth = -pose->Pos().Z();
  const double dDown = -velocity->Z();

  // While suppressed (sim_ready_gate liveness probe) the trigger is frozen,
  // not stepped: the probe's sink must neither fire the servo nor count
  // toward the debounce. On the release edge, drop any partial hold window
  // so the first post-probe poll can't satisfy the 2 s debounce instantly.
  const bool suppressed = this->dataPtr->entrySuppressed.load();
  if (this->dataPtr->prevSuppressed && !suppressed)
  {
    this->dataPtr->trigger->ResetHold();
  }
  this->dataPtr->prevSuppressed = suppressed;

  // At most one feature contributes per tick: the entry servo only pushes
  // while descending, the relief only acts while ascending. With the
  // entry feature off, the trigger state machine is dead weight — skip it.
  const bool entryActive =
    entryOn && !suppressed && this->dataPtr->trigger->Step(t, depth, dDown);
  if (entryActive)
  {
    const double dRef = core::entryRefSpeed(
      this->dataPtr->trigger->TimeSinceFire(t), this->dataPtr->peakSpeed, this->dataPtr->riseTime,
      this->dataPtr->decayTime);
    const double force =
      core::entryServoForce(dRef, dDown, this->dataPtr->servoGain, this->dataPtr->maxForce);
    if (force > 0.0)
    {
      // Pure world-down force at the CoM: injects momentum without a
      // spurious pitch moment (pitch stays owned by the calibrated trim).
      link.AddWorldForce(_ecm, gz::math::Vector3d(0.0, 0.0, -force));
    }
    return;
  }

  if (reliefOn && velocity->Z() > this->dataPtr->minAscentSpeed)
  {
    // Body heave rate: the same axis the Hydrodynamics zW row damps.
    const double w = pose->Rot().RotateVectorReverse(*velocity).Z();
    const double fBody = core::reliefForceBodyZ(
      w, this->dataPtr->zW, this->dataPtr->zWabsW, this->dataPtr->retainFraction);
    if (fBody != 0.0)
    {
      // Same application point (link origin) and axis (body z) as the
      // Hydrodynamics wrench, so net ascent heave drag is exactly
      // retain_fraction * D(w) at every attitude.
      const auto fWorld = pose->Rot().RotateVector(gz::math::Vector3d(0.0, 0.0, fBody));
      link.AddWorldWrench(_ecm, fWorld, gz::math::Vector3d::Zero);
    }
  }
}

}  // namespace dave_gz_model_plugins
