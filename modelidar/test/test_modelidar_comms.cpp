// Serial protocol (firmware/README.md, "Protocole série") and wheel conversions.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "modelidar/modelidar_comms.hpp"
#include "modelidar/wheel.hpp"

using modelidar::BoardState;
using modelidar::Gains;

TEST(ParseState, ExampleOfTheReadme)
{
  BoardState s;
  ASSERT_TRUE(modelidar::parse_state("!0700,0600 L:+600,+420 R:+42.857,+210 C:USB F:-", s));
  EXPECT_EQ(s.duty_left, 700);
  EXPECT_EQ(s.duty_right, 600);
  EXPECT_DOUBLE_EQ(s.left.velocity, 600.0);
  EXPECT_EQ(s.left.position, 420);
  EXPECT_DOUBLE_EQ(s.right.velocity, 42.857);
  EXPECT_EQ(s.right.position, 210);
  EXPECT_EQ(s.owner, "USB");
  EXPECT_EQ(s.fault, "-");
}

TEST(ParseState, NegativeValuesAndFault)
{
  BoardState s;
  ASSERT_TRUE(modelidar::parse_state("!0300,0500 L:-12.5,-100000 R:+0,-3 C:RC F:STALL", s));
  EXPECT_DOUBLE_EQ(s.left.velocity, -12.5);
  EXPECT_EQ(s.left.position, -100000);
  EXPECT_EQ(s.right.position, -3);
  EXPECT_EQ(s.owner, "RC");
  EXPECT_EQ(s.fault, "STALL");
}

TEST(ParseState, RejectsOtherLinesAndKeepsOutput)
{
  BoardState s;
  s.owner = "unchanged";
  EXPECT_FALSE(modelidar::parse_state("", s));
  EXPECT_FALSE(modelidar::parse_state("MCU started (reset: watchdog)", s));
  EXPECT_FALSE(modelidar::parse_state("K:0.05,0.5,0.05,0.2", s));
  // truncated, or followed by garbage
  EXPECT_FALSE(modelidar::parse_state("!0700,0600 L:+600,+420 R:+42.857,+210 C:USB", s));
  EXPECT_FALSE(modelidar::parse_state("!0700,0600 L:+600,+420 R:+42.857,+210 C:USB F:- x", s));
  EXPECT_EQ(s.owner, "unchanged");
}

TEST(ParseGains, AnswerOfTheBoard)
{
  Gains g;
  ASSERT_TRUE(modelidar::parse_gains("K:0.05,0.5,0.05,0.2", g));
  EXPECT_DOUBLE_EQ(g.kp, 0.05);
  EXPECT_DOUBLE_EQ(g.ki, 0.5);
  EXPECT_DOUBLE_EQ(g.kff, 0.05);
  EXPECT_DOUBLE_EQ(g.dmin, 0.2);
  EXPECT_FALSE(modelidar::parse_gains("K:0.05,0.5,0.05", g));
  EXPECT_FALSE(modelidar::parse_gains("!0700,0600 L:+600,+420 R:+42.857,+210 C:USB F:-", g));
}

TEST(SameGains, ToleratesFloatRounding)
{
  const Gains asked{0.05, 0.5, 0.05, 0.2};
  EXPECT_TRUE(modelidar::same_gains({0.0500000007, 0.5, 0.0499999993, 0.2}, asked));
  EXPECT_FALSE(modelidar::same_gains({0.06, 0.5, 0.05, 0.2}, asked));
}

TEST(VelocityFrame, MilliradiansRoundedAndClamped)
{
  EXPECT_EQ(modelidar::velocity_frame(1.2345, -0.5), "V1235,-500\r");
  EXPECT_EQ(modelidar::velocity_frame(0.0, 0.0), "V0,0\r");
  EXPECT_EQ(modelidar::velocity_frame(1000.0, -1000.0), "V50000,-50000\r");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(modelidar::velocity_frame(nan, inf), "V0,0\r");
}

TEST(Wheel, TicksToRadians)
{
  modelidar::Wheel w;
  w.setup("left_wheel_joint", 544);
  EXPECT_EQ(w.position_name, "left_wheel_joint/position");
  EXPECT_EQ(w.velocity_name, "left_wheel_joint/velocity");
  w.zero(0);
  EXPECT_NEAR(w.position(544), 2.0 * M_PI, 1e-12);
  EXPECT_NEAR(w.velocity(272.0), M_PI, 1e-12);
}

TEST(Wheel, ZeroOnConfigure)
{
  modelidar::Wheel w;
  w.setup("j", 544);
  w.zero(1000);  // the firmware was already counting
  EXPECT_DOUBLE_EQ(w.position(1000), 0.0);
  EXPECT_NEAR(w.position(1272), M_PI, 1e-12);
}

TEST(Wheel, PositionContinuousAcrossFirmwareRestart)
{
  modelidar::Wheel w;
  w.setup("j", 544);
  w.zero(0);
  const double before = w.position(5440);  // 10 turns
  w.firmware_restarted();                  // the firmware counts from 0 again
  EXPECT_DOUBLE_EQ(w.position(0), before);
  EXPECT_NEAR(w.position(544), before + 2.0 * M_PI, 1e-12);
}
