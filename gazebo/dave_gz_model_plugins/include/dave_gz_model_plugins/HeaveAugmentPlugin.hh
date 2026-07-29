#ifndef DAVE_GZ_MODEL_PLUGINS__HEAVEAUGMENTPLUGIN_HH_
#define DAVE_GZ_MODEL_PLUGINS__HEAVEAUGMENTPLUGIN_HH_

// HeaveAugmentPlugin: two heave-fidelity corrections calibrated to the
// 2026-06-24 lake test (nominal dives 2/4), applied to the same link the
// upstream Hydrodynamics plugin damps:
//
//  * <ascent_drag_relief>: cancels a fraction of the symmetric heave drag
//    while the vehicle ascends (the descent-anchored zW/zWabsW fit
//    over-damps ascent by ~35 %). retain_fraction = 1.0 is an exact no-op.
//  * <entry_momentum>: a one-sided velocity servo that reproduces the real
//    descent-entry plunge (a decaying hump peaking at <peak_speed>), fired
//    once per descent leg. peak_speed <= 0 is an exact no-op.
//
// No ROS surface: SDF-parameterized, timed by sim time. One gz-transport
// control: /model/<model>/heave_augment/entry_suppress (gz.msgs.Boolean)
// freezes the entry trigger while true — the sim_ready_gate's pre-mission
// physics-liveness probe sinks the hull on purpose and must not fire the
// leg-entry servo doing it.

#include <gz/sim/System.hh>
#include <sdf/sdf.hh>

#include <memory>

namespace dave_gz_model_plugins
{
class HeaveAugmentPlugin : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate
{
public:
  HeaveAugmentPlugin();
  ~HeaveAugmentPlugin() override;

  void Configure(
    const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm) override;

private:
  struct PrivateData;
  std::unique_ptr<PrivateData> dataPtr;
};
}  // namespace dave_gz_model_plugins

#endif  // DAVE_GZ_MODEL_PLUGINS__HEAVEAUGMENTPLUGIN_HH_
