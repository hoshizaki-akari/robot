#ifndef FR_TRACTION__TRACTION_STATE_MACHINE_HPP_
#define FR_TRACTION__TRACTION_STATE_MACHINE_HPP_

#include <cstdint>
#include <string>

namespace fr_traction
{

enum class TractionState : uint8_t
{
  INITIALIZING = 0,
  READY = 1,
  MANUAL_SETUP = 2,
  PRETENSION = 3,
  CALIBRATING = 4,
  DIRECTION_LOCKED = 5,
  TRACTION = 6,
  RELEASING = 7,
  COMPLETED = 8,
  FAULT = 9,
  EMERGENCY_STOP = 10
};

const char * state_name(TractionState state);
bool is_motion_state(TractionState state);
bool can_transition(TractionState from, TractionState to);

class StateMachine
{
public:
  TractionState state() const {return state_;}
  bool transition(TractionState next);

private:
  TractionState state_ = TractionState::INITIALIZING;
};

}  // namespace fr_traction

#endif  // FR_TRACTION__TRACTION_STATE_MACHINE_HPP_
