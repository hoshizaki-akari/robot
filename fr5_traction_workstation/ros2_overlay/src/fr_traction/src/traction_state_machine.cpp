#include "fr_traction/traction_state_machine.hpp"

namespace fr_traction
{

const char * state_name(TractionState state)
{
  switch (state) {
    case TractionState::INITIALIZING: return "INITIALIZING";
    case TractionState::READY: return "READY";
    case TractionState::MANUAL_SETUP: return "MANUAL_SETUP";
    case TractionState::PRETENSION: return "PRETENSION";
    case TractionState::CALIBRATING: return "CALIBRATING";
    case TractionState::DIRECTION_LOCKED: return "DIRECTION_LOCKED";
    case TractionState::TRACTION: return "TRACTION";
    case TractionState::RELEASING: return "RELEASING";
    case TractionState::COMPLETED: return "COMPLETED";
    case TractionState::FAULT: return "FAULT";
    case TractionState::EMERGENCY_STOP: return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

bool is_motion_state(TractionState state)
{
  return state == TractionState::PRETENSION || state == TractionState::TRACTION ||
         state == TractionState::RELEASING;
}

bool can_transition(TractionState from, TractionState to)
{
  if (to == TractionState::EMERGENCY_STOP && from != TractionState::EMERGENCY_STOP) {
    return true;
  }
  if ((is_motion_state(from) || from == TractionState::MANUAL_SETUP ||
    from == TractionState::CALIBRATING || from == TractionState::DIRECTION_LOCKED) &&
    to == TractionState::FAULT)
  {
    return true;
  }
  if ((from == TractionState::FAULT || from == TractionState::EMERGENCY_STOP) &&
    to == TractionState::READY)
  {
    return true;
  }
  switch (from) {
    case TractionState::INITIALIZING: return to == TractionState::READY;
    case TractionState::READY: return to == TractionState::MANUAL_SETUP;
    // Manual teach-pendant setup can lock a direction directly from the
    // measured force; the legacy automatic pretension path remains available.
    case TractionState::MANUAL_SETUP:
      return to == TractionState::PRETENSION || to == TractionState::CALIBRATING;
    case TractionState::PRETENSION: return to == TractionState::CALIBRATING;
    case TractionState::CALIBRATING: return to == TractionState::DIRECTION_LOCKED;
    case TractionState::DIRECTION_LOCKED:
      return to == TractionState::TRACTION || to == TractionState::MANUAL_SETUP;
    case TractionState::TRACTION: return to == TractionState::RELEASING;
    case TractionState::RELEASING: return to == TractionState::COMPLETED;
    case TractionState::COMPLETED:
      return to == TractionState::READY || to == TractionState::DIRECTION_LOCKED;
    case TractionState::FAULT:
    case TractionState::EMERGENCY_STOP: return false;
  }
  return false;
}

bool StateMachine::transition(TractionState next)
{
  if (!can_transition(state_, next)) {return false;}
  state_ = next;
  return true;
}

}  // namespace fr_traction
