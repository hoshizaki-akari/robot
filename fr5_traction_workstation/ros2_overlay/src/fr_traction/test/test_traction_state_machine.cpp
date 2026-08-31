#include "fr_traction/traction_state_machine.hpp"

#include "gtest/gtest.h"

namespace fr_traction
{

TEST(TractionStateMachine, AcceptsEveryNominalEdge)
{
  StateMachine machine;
  for (const auto next : {TractionState::READY, TractionState::MANUAL_SETUP,
      TractionState::PRETENSION, TractionState::CALIBRATING,
      TractionState::DIRECTION_LOCKED, TractionState::TRACTION,
      TractionState::RELEASING, TractionState::COMPLETED, TractionState::READY})
  {
    EXPECT_TRUE(machine.transition(next));
  }
}

TEST(TractionStateMachine, RejectsIllegalEdgesWithoutChangingState)
{
  StateMachine machine;
  EXPECT_FALSE(machine.transition(TractionState::TRACTION));
  EXPECT_EQ(machine.state(), TractionState::INITIALIZING);
  EXPECT_TRUE(machine.transition(TractionState::READY));
  for (const auto illegal : {TractionState::INITIALIZING, TractionState::PRETENSION,
      TractionState::CALIBRATING, TractionState::DIRECTION_LOCKED,
      TractionState::TRACTION, TractionState::RELEASING, TractionState::COMPLETED})
  {
    EXPECT_FALSE(machine.transition(illegal));
    EXPECT_EQ(machine.state(), TractionState::READY);
  }
  EXPECT_TRUE(machine.transition(TractionState::MANUAL_SETUP));
  EXPECT_TRUE(can_transition(TractionState::MANUAL_SETUP, TractionState::CALIBRATING));
  EXPECT_TRUE(machine.transition(TractionState::FAULT));
  EXPECT_FALSE(machine.transition(TractionState::TRACTION));
  EXPECT_TRUE(machine.transition(TractionState::READY));
}

TEST(TractionStateMachine, EmergencyStopIsAvailableFromAnyOperationalState)
{
  StateMachine machine;
  EXPECT_TRUE(machine.transition(TractionState::EMERGENCY_STOP));
  EXPECT_EQ(machine.state(), TractionState::EMERGENCY_STOP);
  EXPECT_TRUE(machine.transition(TractionState::READY));
  EXPECT_TRUE(machine.transition(TractionState::MANUAL_SETUP));
  EXPECT_TRUE(machine.transition(TractionState::PRETENSION));
  EXPECT_TRUE(machine.transition(TractionState::EMERGENCY_STOP));
}

}  // namespace fr_traction
