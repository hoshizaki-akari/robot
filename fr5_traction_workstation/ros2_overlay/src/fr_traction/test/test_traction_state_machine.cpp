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
  for (const auto illegal : {TractionState::INITIALIZING, TractionState::CALIBRATING,
      TractionState::TRACTION, TractionState::COMPLETED})
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

TEST(TractionStateMachine, CompletedRunReturnsToDirectionLockedIdleState)
{
  StateMachine machine;
  for (const auto next : {TractionState::READY, TractionState::MANUAL_SETUP,
      TractionState::CALIBRATING, TractionState::DIRECTION_LOCKED,
      TractionState::TRACTION, TractionState::RELEASING, TractionState::COMPLETED})
  {
    EXPECT_TRUE(machine.transition(next));
  }
  EXPECT_TRUE(machine.transition(TractionState::DIRECTION_LOCKED));
  EXPECT_TRUE(machine.transition(TractionState::TRACTION));
  EXPECT_TRUE(can_transition(TractionState::DIRECTION_LOCKED, TractionState::MANUAL_SETUP));
}

TEST(TractionStateMachine, CalibrationCanReturnToManualSetupForOperatorRetry)
{
  StateMachine machine;
  EXPECT_TRUE(machine.transition(TractionState::READY));
  EXPECT_TRUE(machine.transition(TractionState::MANUAL_SETUP));
  EXPECT_TRUE(machine.transition(TractionState::CALIBRATING));
  EXPECT_TRUE(machine.transition(TractionState::MANUAL_SETUP));
  EXPECT_EQ(machine.state(), TractionState::MANUAL_SETUP);
}

TEST(TractionStateMachine, SupportsAssistedDragAndPositionHoldFlows)
{
  StateMachine drag;
  EXPECT_TRUE(drag.transition(TractionState::READY));
  EXPECT_TRUE(drag.transition(TractionState::DRAGGING));
  EXPECT_TRUE(drag.transition(TractionState::COMPLETED));
  EXPECT_TRUE(drag.transition(TractionState::READY));

  StateMachine position;
  for (const auto next : {TractionState::READY, TractionState::MANUAL_SETUP,
      TractionState::CALIBRATING, TractionState::DIRECTION_LOCKED,
      TractionState::TRACTION, TractionState::POSITION_HOLD,
      TractionState::TRACTION, TractionState::POSITION_HOLD,
      TractionState::COMPLETED, TractionState::DIRECTION_LOCKED})
  {
    EXPECT_TRUE(position.transition(next));
  }
}

TEST(TractionStateMachine, AutomaticDirectionAndBoundedSearchNeedNoManualCalibration)
{
  StateMachine machine;
  for (const auto next : {TractionState::READY, TractionState::PRETENSION,
      TractionState::DIRECTION_LOCKED, TractionState::TRACTION,
      TractionState::COMPLETED, TractionState::READY, TractionState::DIRECTION_LOCKED,
      TractionState::TRACTION, TractionState::POSITION_HOLD})
  {
    ASSERT_TRUE(machine.transition(next));
  }
  EXPECT_TRUE(machine.transition(TractionState::FAULT));
}

TEST(TractionStateMachine, UnloadingCanInterruptTractionDragOrPositionHold)
{
  for (const auto state : {TractionState::READY, TractionState::TRACTION,
      TractionState::DRAGGING, TractionState::POSITION_HOLD})
  {
    EXPECT_TRUE(can_transition(state, TractionState::RELEASING));
  }
  EXPECT_FALSE(can_transition(TractionState::FAULT, TractionState::RELEASING));
  EXPECT_FALSE(can_transition(TractionState::EMERGENCY_STOP, TractionState::RELEASING));
  EXPECT_TRUE(can_transition(TractionState::PRETENSION, TractionState::COMPLETED));
}

}  // namespace fr_traction
