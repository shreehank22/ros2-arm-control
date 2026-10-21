#include <gtest/gtest.h>
#include <string>

#include "arm_control/control_state.hpp"

// nextState(has_measured, has_desired, measured_fresh, desired_fresh)

TEST(NextStateTest, NoMeasurementIsWaiting)
{
    EXPECT_EQ(nextState(false, false, false, false), State::WAITING);
    // a missing measurement wins over everything else
    EXPECT_EQ(nextState(false, true, true, true), State::WAITING);
}

TEST(NextStateTest, NoDesiredStateIsFallback)
{
    EXPECT_EQ(nextState(true, false, true, false), State::FALLBACK);
}

TEST(NextStateTest, StaleMeasurementIsFallback)
{
    EXPECT_EQ(nextState(true, true, false, true), State::FALLBACK);
}

TEST(NextStateTest, StaleDesiredStateIsFallback)
{
    EXPECT_EQ(nextState(true, true, true, false), State::FALLBACK);
}

TEST(NextStateTest, FreshMeasurementAndDesiredStateIsTracking)
{
    EXPECT_EQ(nextState(true, true, true, true), State::TRACKING);
}

TEST(StateNameTest, ReturnsNameForEachState)
{
    EXPECT_EQ(std::string(stateName(State::WAITING)), "WAITING");
    EXPECT_EQ(std::string(stateName(State::FALLBACK)), "FALLBACK");
    EXPECT_EQ(std::string(stateName(State::TRACKING)), "TRACKING");
}
