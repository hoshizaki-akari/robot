#pragma once

// TEST-ONLY SDK double.  Keep the production transport include unchanged and
// put this directory first on the offline test include path.
#include <aubo_driver/AuboRobotMetaType.h>

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace aubo_v4_test_sdk
{
  struct State
  {
    std::mutex mutex;
    std::condition_variable changed;
    int constructor_count {0};
    int destructor_count {0};
    int login_count {0};
    int logout_count {0};
    int ik_call_count {0};
    int ik_completed_count {0};
    int line_move_count {0};
    int line_move_completed_count {0};
    int stop_count {0};
    int diagnosis_count {0};
    double profile_speed {0.0}, profile_acceleration {0.0};
    bool bad_profile_readback {false};
    bool block_profile {false}, profile_entered {false}, release_profile {false};
    int follow_move_count {0};
    std::array<double, 6> last_follow_target {};
    aubo_robot_namespace::JointVelcAccParam joint_velocity {}, joint_acceleration {};
    std::size_t login_index {0};
    std::vector < int > login_results {aubo_robot_namespace::ErrnoSucc};
    int ik_result {aubo_robot_namespace::ErrnoSucc};
    int line_move_result {aubo_robot_namespace::ErrnoSucc};
    std::vector < int > stop_results {aubo_robot_namespace::ErrnoSucc};
    std::array < double, aubo_robot_namespace::ARM_DOF > ik_delta {};
    bool ik_nan {false};
    bool block_ik {false};
    bool ik_entered {false};
    bool release_ik {false};
    bool block_line_move {false};
    bool line_move_entered {false};
    bool release_line_move {false};
    bool block_first_stop {false};
    bool first_stop_entered {false};
    bool release_first_stop {false};
    bool real_robot {true};
    aubo_robot_namespace::RobotWorkMode work_mode {aubo_robot_namespace::RobotModeReal};
    int diagnosis_result {aubo_robot_namespace::ErrnoSucc};
    int state_result {aubo_robot_namespace::ErrnoSucc};
    bool block_state {false}, state_entered {false}, release_state {false};
    aubo_robot_namespace::RobotDiagnosis diagnosis {};
    aubo_robot_namespace::RobotState robot_state {aubo_robot_namespace::RobotStopped};
    RealTimeRoadPointCallback waypoint_callback {nullptr};
    void * waypoint_context {nullptr};
    std::vector < std::string > calls;
    std::size_t event_sequence {0};

    void notify(const std::string & call)
    {
      calls.push_back(call);
      ++event_sequence;
      changed.notify_all();
    }

    void reset()
    {
      std::lock_guard < std::mutex > lock(mutex);
      constructor_count = 0;
      destructor_count = 0;
      login_count = 0;
      logout_count = 0;
      ik_call_count = 0;
      ik_completed_count = 0;
      line_move_count = 0;
      line_move_completed_count = 0;
      stop_count = 0;
      diagnosis_count = 0;
      profile_speed = profile_acceleration = 0.0;
      bad_profile_readback = false;
      block_profile = profile_entered = release_profile = false;
      follow_move_count = 0;
      last_follow_target.fill(0.0);
      joint_velocity = {};
      joint_acceleration = {};
      login_index = 0;
      login_results = {aubo_robot_namespace::ErrnoSucc};
      ik_result = aubo_robot_namespace::ErrnoSucc;
      line_move_result = aubo_robot_namespace::ErrnoSucc;
      stop_results = {aubo_robot_namespace::ErrnoSucc};
      ik_delta.fill(0.0);
      ik_nan = false;
      block_ik = false;
      ik_entered = false;
      release_ik = false;
      block_line_move = false;
      line_move_entered = false;
      release_line_move = false;
      block_first_stop = false;
      first_stop_entered = false;
      release_first_stop = false;
      real_robot = true;
      work_mode = aubo_robot_namespace::RobotModeReal;
      diagnosis_result = aubo_robot_namespace::ErrnoSucc;
      state_result = aubo_robot_namespace::ErrnoSucc;
      block_state = state_entered = release_state = false;
      diagnosis = {};
      diagnosis.armPowerStatus = true;
      robot_state = aubo_robot_namespace::RobotStopped;
      waypoint_callback = nullptr;
      waypoint_context = nullptr;
      calls.clear();
      event_sequence = 0;
    }
  };

  inline State & state()
  {
    static State value;
    return value;
  }

  inline void reset()
  {
    state().reset();
  }

  inline bool wait_for_event(std::size_t & seen, std::chrono::milliseconds timeout)
  {
    State & s = state();
    std::unique_lock < std::mutex > lock(s.mutex);
    const bool changed = s.changed.wait_for(
      lock, timeout, [&s, &seen]() {
      return s.event_sequence != seen;
    });
    seen = s.event_sequence;
    return changed;
  }

  inline bool wait_for_ik(std::chrono::milliseconds timeout)
  {
    State & s = state();
    std::unique_lock < std::mutex > lock(s.mutex);
    return s.changed.wait_for(lock, timeout, [&s]() {return s.ik_entered;});
  }

  inline bool wait_for_line_move(std::chrono::milliseconds timeout)
  {
    State & s = state();
    std::unique_lock < std::mutex > lock(s.mutex);
    return s.changed.wait_for(lock, timeout, [&s]() {return s.line_move_entered;});
  }

  inline bool wait_for_stop_count(int count, std::chrono::milliseconds timeout)
  {
    State & s = state();
    std::unique_lock < std::mutex > lock(s.mutex);
    return s.changed.wait_for(lock, timeout, [&s, count]() {return s.stop_count >= count;});
  }

  inline bool wait_for_ik_completed(int count, std::chrono::milliseconds timeout)
  {
    State & s = state();
    std::unique_lock < std::mutex > lock(s.mutex);
    return s.changed.wait_for(
      lock, timeout, [&s, count]() {
      return s.ik_completed_count >= count;
    });
  }

  inline bool wait_for_line_move_completed(int count, std::chrono::milliseconds timeout)
  {
    State & s = state();
    std::unique_lock < std::mutex > lock(s.mutex);
    return s.changed.wait_for(
      lock, timeout, [&s, count]() {
      return s.line_move_completed_count >= count;
    });
  }

  inline void release_ik()
  {
    State & s = state();
    std::lock_guard < std::mutex > lock(s.mutex);
    s.release_ik = true;
    s.block_ik = false;
    s.changed.notify_all();
  }

  inline void release_line_move()
  {
    State & s = state();
    std::lock_guard < std::mutex > lock(s.mutex);
    s.release_line_move = true;
    s.block_line_move = false;
    s.changed.notify_all();
  }

  inline void release_first_stop()
  {
    State & s = state();
    std::lock_guard < std::mutex > lock(s.mutex);
    s.release_first_stop = true;
    s.block_first_stop = false;
    s.changed.notify_all();
  }

  inline std::vector < std::string > call_order()
  {
    State & s = state();
    std::lock_guard < std::mutex > lock(s.mutex);
    return s.calls;
  }

  inline void emit_waypoint(const aubo_robot_namespace::wayPoint_S & point)
  {
    State & s = state();
    RealTimeRoadPointCallback callback;
    void * context;
    {
      std::lock_guard < std::mutex > lock(s.mutex);
      callback = s.waypoint_callback;
      context = s.waypoint_context;
    }
    if (callback != nullptr) {
      callback(&point, context);
    }
  }

  inline void set_login_results(const std::vector < int > & values)
  {
    State & s = state();
    std::lock_guard < std::mutex > lock(s.mutex);
    s.login_results = values;
  }

  inline void set_stop_results(const std::vector < int > & values)
  {
    State & s = state();
    std::lock_guard < std::mutex > lock(s.mutex);
    s.stop_results = values;
  }

  class ServiceInterface
  {
public:
    ServiceInterface()
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      ++s.constructor_count;
      s.notify("construct");
    }

    ~ServiceInterface()
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      ++s.destructor_count;
      s.notify("destruct");
    }

    int robotServiceLogin(const char *, int, const char *, const char *)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      ++s.login_count;
      const std::size_t index = s.login_index++;
      const int result = index < s.login_results.size() ?
        s.login_results[index] : aubo_robot_namespace::ErrnoSucc;
      s.notify("login");
      return result;
    }

    int robotServiceLogout()
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      ++s.logout_count;
      s.notify("logout");
      return aubo_robot_namespace::ErrnoSucc;
    }

    int robotServiceGetIsRealRobotExist(bool & value)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      value = s.real_robot;
      s.notify("real_robot");
      return aubo_robot_namespace::ErrnoSucc;
    }

    int robotServiceGetRobotWorkMode(aubo_robot_namespace::RobotWorkMode & mode)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      mode = s.work_mode;
      s.notify("work_mode");
      return aubo_robot_namespace::ErrnoSucc;
    }

    int robotServiceInitGlobalMoveProfile()
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      s.notify("init_profile");
      return aubo_robot_namespace::ErrnoSucc;
    }

    int robotServiceSetGlobalMoveEndMaxLineAcc(double value)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      s.notify("set_acc");
      s.profile_acceleration = value;
      return aubo_robot_namespace::ErrnoSucc;
    }

    int robotServiceSetGlobalMoveEndMaxLineVelc(double value)
    {
      State & s = state();
      std::unique_lock < std::mutex > lock(s.mutex);
      s.profile_entered = true;
      s.notify("set_vel");
      s.changed.wait(lock, [&s]() {return !s.block_profile || s.release_profile;});
      s.profile_speed = value;
      return aubo_robot_namespace::ErrnoSucc;
    }

    void robotServiceGetGlobalMoveEndMaxLineVelc(double & value)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      value = s.bad_profile_readback ? 0.05 : s.profile_speed;
      s.notify("get_vel");
    }

    void robotServiceGetGlobalMoveEndMaxLineAcc(double & value)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      value = s.profile_acceleration;
      s.notify("get_acc");
    }

    int robotServiceSetGlobalMoveJointMaxVelc(const aubo_robot_namespace::JointVelcAccParam & value)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      s.joint_velocity = value;
      return 0;
    }

    int robotServiceSetGlobalMoveJointMaxAcc(const aubo_robot_namespace::JointVelcAccParam & value)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      s.joint_acceleration = value;
      return 0;
    }

    void robotServiceGetGlobalMoveJointMaxVelc(aubo_robot_namespace::JointVelcAccParam & value)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      value = s.joint_velocity;
    }

    void robotServiceGetGlobalMoveJointMaxAcc(aubo_robot_namespace::JointVelcAccParam & value)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      value = s.joint_acceleration;
    }

    int robotServiceSetNoArrivalAhead() {return 0;}
    int robotServiceSetArrivalAheadTimeMode(double) {return 0;}

    int robotServiceFollowModeJointMove(double * target)
    {
      State & s = state();
      std::unique_lock < std::mutex > lock(s.mutex);
      ++s.follow_move_count;
      for (unsigned i = 0; i < 6; ++i) {s.last_follow_target[i] = target[i];}
      s.line_move_entered = true;
      s.notify("follow_move_enter");
      s.changed.wait(lock, [&s]() {return !s.block_line_move || s.release_line_move;});
      s.notify("follow_move_exit");
      return s.line_move_result;
    }

    int robotServiceRegisterRealTimeRoadPointCallback(
      const RealTimeRoadPointCallback callback, void * context)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      s.waypoint_callback = callback;
      s.waypoint_context = context;
      s.notify(callback == nullptr ? "unregister_waypoint" : "register_waypoint");
      return aubo_robot_namespace::ErrnoSucc;
    }

    int robotServiceSetRealTimeRoadPointPush(bool enabled)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      s.notify(enabled ? "enable_waypoint" : "disable_waypoint");
      return aubo_robot_namespace::ErrnoSucc;
    }

    int robotServiceRobotIk(
      const double * seed, const aubo_robot_namespace::Pos &,
      const aubo_robot_namespace::Ori &, aubo_robot_namespace::wayPoint_S & target)
    {
      State & s = state();
      std::unique_lock < std::mutex > lock(s.mutex);
      ++s.ik_call_count;
      s.ik_entered = true;
      s.notify("ik_enter");
      s.changed.wait(lock, [&s]() {return !s.block_ik || s.release_ik;});
      const int result = s.ik_result;
      const auto delta = s.ik_delta;
      const bool nan = s.ik_nan;
      for (std::size_t i = 0; i < aubo_robot_namespace::ARM_DOF; ++i) {
        target.jointpos[i] = nan ? std::numeric_limits < double >
          ::quiet_NaN() : seed[i] + delta[i];
      }
      ++s.ik_completed_count;
      s.notify("ik_exit");
      return result;
    }

    int robotServiceLineMove(aubo_robot_namespace::wayPoint_S &, bool)
    {
      State & s = state();
      std::unique_lock < std::mutex > lock(s.mutex);
      ++s.line_move_count;
      s.line_move_entered = true;
      s.notify("line_move_enter");
      s.changed.wait(lock, [&s]() {return !s.block_line_move || s.release_line_move;});
      const int result = s.line_move_result;
      ++s.line_move_completed_count;
      s.notify("line_move_exit");
      return result;
    }

    int robotMoveFastStop()
    {
      State & s = state();
      std::unique_lock < std::mutex > lock(s.mutex);
      ++s.stop_count;
      const int number = s.stop_count;
      s.notify(number == 1 ? "stop_first" : "stop_final");
      if (number == 1) {
        s.first_stop_entered = true;
        s.changed.notify_all();
        s.changed.wait(lock, [&s]() {return !s.block_first_stop || s.release_first_stop;});
      }
      const std::size_t index = static_cast < std::size_t > (number - 1);
      const int result = index < s.stop_results.size() ?
        s.stop_results[index] : aubo_robot_namespace::ErrnoSucc;
      s.notify(number == 1 ? "stop_first_exit" : "stop_final_exit");
      return result;
    }

    int robotServiceGetRobotDiagnosisInfo(aubo_robot_namespace::RobotDiagnosis & value)
    {
      State & s = state();
      std::lock_guard < std::mutex > lock(s.mutex);
      value = s.diagnosis;
      ++s.diagnosis_count;
      s.notify("diagnosis");
      return s.diagnosis_result;
    }

    int robotServiceGetRobotCurrentState(aubo_robot_namespace::RobotState & value)
    {
      State & s = state();
      std::unique_lock < std::mutex > lock(s.mutex);
      s.state_entered = true;
      s.notify("robot_state_enter");
      s.changed.wait(lock, [&s]() {return !s.block_state || s.release_state;});
      value = s.robot_state;
      s.notify("robot_state");
      return s.state_result;
    }
  };
}  // namespace aubo_v4_test_sdk

// The legacy SDK declares this class at global scope.  The using-declaration
// preserves the production lookup while keeping all test state namespaced.
using aubo_v4_test_sdk::ServiceInterface;
