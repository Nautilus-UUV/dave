#ifndef DAVE_GZ_MODEL_PLUGINS__HEAVE_AUGMENT_CORE_HH_
#define DAVE_GZ_MODEL_PLUGINS__HEAVE_AUGMENT_CORE_HH_

// Pure logic for HeaveAugmentPlugin (no Gazebo dependencies) so the force
// laws and the entry-trigger state machine are unit-testable with gtest.
//
// Conventions: `w` is the body-frame heave rate (positive along body +z, the
// same axis the Hydrodynamics zW/zWabsW row acts on); `dDown` is the
// world-frame downward speed (positive while descending); `depth` is metres
// below the surface (positive down). Drag coefficients are the SDF values,
// i.e. negative (zW = -52.2, zWabsW = -436.7 nominal).

#include <algorithm>
#include <cmath>

namespace dave_gz_model_plugins
{
namespace heave_augment
{

/// Heave damping force along body z as applied by the upstream
/// gz Hydrodynamics diagonal: D(w) = zW*w + zWabsW*|w|*w.
/// With negative coefficients the result opposes w.
inline double heaveDrag(double w, double zW, double zWabsW)
{
  return zW * w + zWabsW * std::abs(w) * w;
}

/// Body-z corrective force that cancels the fraction (1 - retainFraction) of
/// the heave drag, leaving a net ascent drag of retainFraction * D(w).
/// retainFraction = 1.0 is the exact no-op (returns 0 for every w).
inline double reliefForceBodyZ(double w, double zW, double zWabsW, double retainFraction)
{
  return -(1.0 - retainFraction) * heaveDrag(w, zW, zWabsW);
}

/// Reference downward-speed hump for the entry transient: smoothstep rise to
/// `peak` over `riseS`, then exponential decay with time constant `decayS`.
/// `t` is seconds since the trigger fired.
inline double entryRefSpeed(double t, double peak, double riseS, double decayS)
{
  if (t <= 0.0 || peak <= 0.0)
  {
    return 0.0;
  }
  if (t <= riseS)
  {
    const double x = t / riseS;
    return peak * (3.0 * x * x - 2.0 * x * x * x);
  }
  return peak * std::exp(-(t - riseS) / decayS);
}

/// One-sided velocity servo: pushes the vehicle down toward the reference
/// speed, never brakes it, and saturates at maxForce.
inline double entryServoForce(double dRef, double dDown, double gain, double maxForce)
{
  return std::clamp(gain * (dRef - dDown), 0.0, maxForce);
}

/// Seconds the entry transient stays active after firing: by
/// riseS + 5 * decayS the reference hump has decayed below 1 % of peak.
inline double entryActiveDuration(double riseS, double decayS)
{
  return riseS + 5.0 * decayS;
}

/// Debounce for "condition continuously true for holdS seconds"; any false
/// condition resets the timer.
class HoldTimer
{
public:
  /// Advance with the current condition and sim time [s]; returns true once
  /// the condition has held for holdS.
  bool Sustained(bool condition, double t, double holdS)
  {
    if (!condition)
    {
      valid_ = false;
      return false;
    }
    if (!valid_)
    {
      valid_ = true;
      since_ = t;
    }
    return t - since_ >= holdS;
  }

  void Reset() { valid_ = false; }

private:
  bool valid_{false};
  double since_{0.0};
};

struct EntryTriggerParams
{
  /// Depth [m] the vehicle must reach before the transient may fire
  /// (filters the surface float during the initial pump-out).
  double triggerDepth{0.3};
  /// Downward speed [m/s] that must be sustained to count as a descent.
  double triggerSpeed{0.03};
  /// Seconds triggerSpeed/triggerDepth must hold continuously before firing
  /// (filters wave bobbing).
  double triggerHold{2.0};
  /// Below this downward speed [m/s] the descent counts as ended.
  double rearmSpeed{0.01};
  /// Seconds the descent must stay ended before re-arming (leg apex).
  double rearmHold{5.0};
  /// Seconds the transient stays active after firing; see
  /// entryActiveDuration() (default: the nominal 20 s rise / 25 s decay).
  double activeDuration{entryActiveDuration(20.0, 25.0)};
};

/// Fires the entry transient once per descent leg:
/// ARMED -> ACTIVE on a sustained descent below triggerDepth,
/// ACTIVE -> SPENT once the reference hump is exhausted,
/// SPENT -> ARMED after the descent has demonstrably ended (rearmHold).
class EntryTrigger
{
public:
  explicit EntryTrigger(const EntryTriggerParams & params) : params_(params) {}

  /// Advance the state machine; call once per physics step with the current
  /// sim time [s]. Returns true while the transient is ACTIVE.
  bool Step(double t, double depth, double dDown)
  {
    switch (state_)
    {
      case State::kArmed:
        if (hold_.Sustained(
              depth >= params_.triggerDepth && dDown >= params_.triggerSpeed, t,
              params_.triggerHold))
        {
          state_ = State::kActive;
          tFire_ = t;
          hold_.Reset();
        }
        break;
      case State::kActive:
        if (t - tFire_ > params_.activeDuration)
        {
          state_ = State::kSpent;
        }
        break;
      case State::kSpent:
        if (hold_.Sustained(dDown < params_.rearmSpeed, t, params_.rearmHold))
        {
          state_ = State::kArmed;
          hold_.Reset();
        }
        break;
    }
    return state_ == State::kActive;
  }

  bool Active() const { return state_ == State::kActive; }

  /// Seconds since the transient fired (only meaningful while ACTIVE).
  double TimeSinceFire(double t) const { return t - tFire_; }

  /// Drop any partially-accumulated debounce window (fire or re-arm).
  /// Callers that pause Step() for a while (e.g. the sim_ready_gate's
  /// pre-mission liveness probe suppressing the entry servo) use this on
  /// resume so a stale hold window can't satisfy the debounce instantly.
  void ResetHold() { hold_.Reset(); }

private:
  enum class State
  {
    kArmed,
    kActive,
    kSpent,
  };

  EntryTriggerParams params_;
  State state_{State::kArmed};
  // Shared by the ARMED (fire) and SPENT (re-arm) debounces: the states
  // are mutually exclusive and the timer resets on every transition.
  HoldTimer hold_;
  double tFire_{0.0};
};

}  // namespace heave_augment
}  // namespace dave_gz_model_plugins

#endif  // DAVE_GZ_MODEL_PLUGINS__HEAVE_AUGMENT_CORE_HH_
