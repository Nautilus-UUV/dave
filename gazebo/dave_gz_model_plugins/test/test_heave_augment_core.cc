#include <gtest/gtest.h>

#include "dave_gz_model_plugins/heave_augment_core.hh"

#include <cmath>

namespace core = dave_gz_model_plugins::heave_augment;

// Lake-fitted nominal coefficients (SDF convention: negative).
constexpr double kZW = -52.2;
constexpr double kZWabsW = -436.7;

TEST(HeaveDrag, OpposesMotionAndIsOdd)
{
  // Ascending (body w < 0 for the roll-pi mounted vehicle): drag positive
  // along body z (i.e. opposing the motion).
  EXPECT_GT(core::heaveDrag(-0.07, kZW, kZWabsW), 0.0);
  EXPECT_LT(core::heaveDrag(0.07, kZW, kZWabsW), 0.0);
  EXPECT_DOUBLE_EQ(core::heaveDrag(0.0, kZW, kZWabsW), 0.0);
  EXPECT_DOUBLE_EQ(
    core::heaveDrag(-0.13, kZW, kZWabsW), -core::heaveDrag(0.13, kZW, kZWabsW));
}

TEST(ReliefForce, NoOpAtFullRetainFraction)
{
  for (double w : {-0.2, -0.05, 0.0, 0.05, 0.2})
  {
    EXPECT_DOUBLE_EQ(core::reliefForceBodyZ(w, kZW, kZWabsW, 1.0), 0.0);
  }
}

TEST(ReliefForce, CancelsExactlyOneMinusKOfDrag)
{
  const double k = 0.66;
  for (double w : {-0.113, -0.07, -0.02, 0.05})
  {
    const double drag = core::heaveDrag(w, kZW, kZWabsW);
    const double relief = core::reliefForceBodyZ(w, kZW, kZWabsW, k);
    // Relief opposes the drag with magnitude (1 - k)|D|; the net damping
    // left after Hydrodynamics applies D is exactly k * D.
    EXPECT_NEAR(relief, -(1.0 - k) * drag, 1e-12);
    EXPECT_NEAR(drag + relief, k * drag, 1e-12);
  }
}

TEST(EntryRefSpeed, HumpShape)
{
  const double peak = 0.205, rise = 20.0, decay = 25.0;
  EXPECT_DOUBLE_EQ(core::entryRefSpeed(0.0, peak, rise, decay), 0.0);
  EXPECT_DOUBLE_EQ(core::entryRefSpeed(rise, peak, rise, decay), peak);
  // Monotone rise.
  double prev = 0.0;
  for (double t = 1.0; t <= rise; t += 1.0)
  {
    const double v = core::entryRefSpeed(t, peak, rise, decay);
    EXPECT_GT(v, prev);
    prev = v;
  }
  // Exponential decay after the peak.
  EXPECT_NEAR(core::entryRefSpeed(rise + decay, peak, rise, decay), peak * std::exp(-1.0), 1e-12);
  // Spent (< 1 % of peak) by rise + 5 * decay.
  EXPECT_LT(core::entryRefSpeed(rise + 5.0 * decay, peak, rise, decay), 0.01 * peak);
  // peak <= 0 disables.
  EXPECT_DOUBLE_EQ(core::entryRefSpeed(10.0, 0.0, rise, decay), 0.0);
}

TEST(EntryServoForce, OneSidedAndCapped)
{
  const double gain = 1000.0, maxForce = 60.0;
  // Vehicle slower than reference: push down, proportional.
  EXPECT_NEAR(core::entryServoForce(0.20, 0.18, gain, maxForce), 20.0, 1e-9);
  // Vehicle at/above reference: never brake.
  EXPECT_DOUBLE_EQ(core::entryServoForce(0.20, 0.20, gain, maxForce), 0.0);
  EXPECT_DOUBLE_EQ(core::entryServoForce(0.10, 0.25, gain, maxForce), 0.0);
  // Saturation.
  EXPECT_DOUBLE_EQ(core::entryServoForce(0.25, 0.0, gain, maxForce), maxForce);
}

class EntryTriggerTest : public ::testing::Test
{
protected:
  core::EntryTriggerParams params_;  // defaults: 0.3 m / 0.03 m/s / 2 s hold,
                                     // rearm 0.01 m/s / 5 s, active 145 s
};

TEST_F(EntryTriggerTest, NoFireDuringSurfaceFloat)
{
  core::EntryTrigger trigger(params_);
  // 75 s of wave bob at the surface: shallow AND slow, with brief spikes of
  // one condition but never both sustained.
  for (double t = 0.0; t < 75.0; t += 0.1)
  {
    const double depth = 0.15 + 0.1 * std::sin(t);          // < 0.3 m mostly
    const double dDown = 0.05 * std::sin(3.0 * t);          // alternating sign
    EXPECT_FALSE(trigger.Step(t, depth, dDown));
  }
  EXPECT_FALSE(trigger.Active());
}

TEST_F(EntryTriggerTest, FiresOncePerLegAndRearms)
{
  core::EntryTrigger trigger(params_);
  double t = 0.0;
  const double dt = 0.1;

  // Descent begins: sustained 0.05 m/s at 0.5 m depth -> fires after 2 s.
  bool fired = false;
  double tFire = 0.0;
  for (; t < 10.0; t += dt)
  {
    if (trigger.Step(t, 0.5 + 0.05 * t, 0.05) && !fired)
    {
      fired = true;
      tFire = t;
    }
  }
  ASSERT_TRUE(fired);
  EXPECT_NEAR(tFire, 2.0, 2.0 * dt);
  EXPECT_TRUE(trigger.Active());

  // Stays ACTIVE through the hump, then SPENT after activeDuration; brief
  // slowdowns mid-descent must not re-arm it.
  for (; t < 200.0; t += dt)
  {
    const double dDown = (std::fmod(t, 30.0) < 1.0) ? 0.005 : 0.12;  // 1 s dips
    trigger.Step(t, 5.0, dDown);
  }
  EXPECT_FALSE(trigger.Active());  // SPENT, not re-fired (dips < rearmHold)

  // Leg apex: descent ends for > rearmHold, then the next leg fires again.
  for (; t < 210.0; t += dt)
  {
    EXPECT_FALSE(trigger.Step(t, 12.0, 0.0));  // holding/ascending
  }
  fired = false;
  for (; t < 220.0; t += dt)
  {
    if (trigger.Step(t, 12.0, 0.06))
    {
      fired = true;
      break;
    }
  }
  EXPECT_TRUE(fired);  // re-armed and fired on the second descent leg
}

TEST_F(EntryTriggerTest, ResetHoldDropsPartialDebounce)
{
  core::EntryTrigger trigger(params_);
  const double dt = 0.1;
  // 1.5 s of qualifying descent: under the 2 s hold, must not fire.
  double t = 0.0;
  for (; t < 1.5; t += dt)
  {
    EXPECT_FALSE(trigger.Step(t, 0.5, 0.05));
  }
  // Plugin suppression semantics: Step() pauses while the liveness probe
  // sinks the hull, then ResetHold() on release. Without the reset, the
  // stale window (since_ = 0) would satisfy the 2 s debounce on the very
  // first resumed Step at t = 20.
  trigger.ResetHold();
  t = 20.0;
  EXPECT_FALSE(trigger.Step(t, 0.6, 0.05));  // fresh window starts here
  bool fired = false;
  double tFire = 0.0;
  for (t += dt; t < 30.0; t += dt)
  {
    if (trigger.Step(t, 0.6, 0.05))
    {
      fired = true;
      tFire = t;
      break;
    }
  }
  ASSERT_TRUE(fired);
  EXPECT_NEAR(tFire, 22.0, 2.0 * dt);  // full debounce re-served after reset
}

TEST_F(EntryTriggerTest, DeepSpawnFiresOnlyOnCommandedDescent)
{
  core::EntryTrigger trigger(params_);
  double t = 0.0;
  const double dt = 0.1;
  // Spawned at 5 m slightly buoyant: deep but rising -> never fires.
  for (; t < 30.0; t += dt)
  {
    EXPECT_FALSE(trigger.Step(t, 5.0 - 0.02 * t, -0.02));
  }
  // Commanded descent begins.
  bool fired = false;
  for (; t < 40.0; t += dt)
  {
    if (trigger.Step(t, 5.0, 0.08))
    {
      fired = true;
      break;
    }
  }
  EXPECT_TRUE(fired);
}
