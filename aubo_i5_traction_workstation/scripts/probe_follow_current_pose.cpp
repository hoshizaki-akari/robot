// Explicit hardware compatibility check. Never used by offline tests or web startup.
// The only motion target is the fresh, stationary, measured current joint pose.
#include <aubo_driver/serviceinterface.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include "aubo_v4_driver/state_validation.hpp"
#include "aubo_v4_driver/stepwise_scheduler.hpp"

namespace v = aubo_robot_namespace;
using Clock = std::chrono::steady_clock;
struct Observation
{
  std::mutex mutex;
  v::wayPoint_S point{};
  Clock::time_point at{};
  unsigned count{0};
};
void observe(const v::wayPoint_S * point, void * context)
{
  if (!point || !context) {return;}
  auto & sample = *static_cast<Observation *>(context);
  std::lock_guard<std::mutex> guard(sample.mutex);
  sample.point = *point;
  sample.at = Clock::now();
  ++sample.count;
}
const char * required(const char * name)
{
  const char * value = std::getenv(name);
  if (!value || !*value) {throw std::runtime_error("missing process environment");}
  return value;
}
int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  const bool read_only = std::string(argv[1]) == "--read-only-state";
  if (!read_only && std::string(argv[1]) != "--execute-current-pose-only") {return 2;}
  const int lock = open("debug/live_workstation.lock", O_CREAT | O_RDWR, 0600);
  if (lock < 0 || flock(lock, LOCK_EX | LOCK_NB) != 0) {
    std::cerr << "Another workstation owns the robot; no SDK connection created.\n";
    return 2;
  }
  ServiceInterface send, read, stop;
  Observation sample;
  bool logged_send = false, logged_read = false, logged_stop = false, dispatched = false;
  std::array<double, 6> target{};
  int result = 1;
  try {
    const char * host = required("AUBO_V4_HOST");
    const char * user = required("AUBO_V4_USERNAME");
    const char * password = required("AUBO_V4_PASSWORD");
    const int port = std::stoi(required("AUBO_V4_PORT"));
    logged_send = send.robotServiceLogin(host, port, user, password) == 0;
    logged_read = read.robotServiceLogin(host, port, user, password) == 0;
    logged_stop = stop.robotServiceLogin(host, port, user, password) == 0;
    if (!logged_send || !logged_read || !logged_stop) {throw std::runtime_error("login failed");}
    bool real = false;
    v::RobotWorkMode mode = v::RobotModeSimulator;
    if (read.robotServiceGetIsRealRobotExist(real) != 0 || !real ||
      read.robotServiceGetRobotWorkMode(mode) != 0 || mode != v::RobotModeReal)
    {
      throw std::runtime_error("real controller not confirmed");
    }
    v::RobotDiagnosis diagnosis{};
    if (read.robotServiceGetRobotDiagnosisInfo(diagnosis) != 0 || !diagnosis.armPowerStatus ||
      diagnosis.armCanbusStatus || diagnosis.softEmergency || diagnosis.remoteEmergency ||
      diagnosis.robotCollision || diagnosis.jointCollisionDetect || diagnosis.encoderErrorStatus ||
      diagnosis.encoderLinesError || diagnosis.jointErrorStatus ||
      diagnosis.singularityOverSpeedAlarm ||
      diagnosis.robotCurrentAlarm || diagnosis.toolIoError)
    {
      throw std::runtime_error("diagnosis unhealthy");
    }
    if (read.robotServiceRegisterRealTimeRoadPointCallback(observe, &sample) != 0 ||
      read.robotServiceSetRealTimeRoadPointPush(true) != 0) {throw std::runtime_error("no push");}
    bool have_previous = false;
    unsigned quiet = 0;
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    unsigned sequence = 0;
    while (Clock::now() < deadline && quiet < 3) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      std::lock_guard<std::mutex> guard(sample.mutex);
      if (sample.count == sequence || Clock::now() - sample.at > std::chrono::milliseconds(100)) {
        continue;
      }
      sequence = sample.count;
      bool unchanged = have_previous;
      for (unsigned i = 0; i < 6; ++i) {
        if (!std::isfinite(sample.point.jointpos[i])) {throw std::runtime_error("invalid joint");}
        unchanged = unchanged && std::abs(sample.point.jointpos[i] - target[i]) < 1e-6;
        target[i] = sample.point.jointpos[i];
      }
      have_previous = true;
      quiet = unchanged ? quiet + 1 : 0;
    }
    v::RobotState state = v::RobotRunning;
    if (quiet < 3 || read.robotServiceGetRobotCurrentState(state) != 0 ||
      state != v::RobotStopped)
    {
      throw std::runtime_error("stationary current pose not confirmed");
    }
    if (read_only) {
      v::wayPoint_S calculated{}, observed{};
      {
        std::lock_guard<std::mutex> guard(sample.mutex);
        observed = sample.point;
      }
      const int fk_result = read.robotServiceRobotFk(target.data(), 6, calculated);
      double squared = 0.0;
      for (unsigned i = 0; i < 3; ++i) {
        const double delta = calculated.cartPos.positionVector[i] -
          observed.cartPos.positionVector[i];
        squared += delta * delta;
      }
      const double angle = aubo_v4_driver::quaternion_distance_rad(
        {calculated.orientation.x, calculated.orientation.y, calculated.orientation.z,
          calculated.orientation.w},
        {observed.orientation.x, observed.orientation.y, observed.orientation.z,
          observed.orientation.w});
      const double error = std::sqrt(squared);
      std::cout << "read_only_fk_result=" << fk_result << " position_error_m=" << error <<
        " orientation_error_rad=" << angle << '\n';
      if (fk_result != 0 || !std::isfinite(error) || error > 0.001 || angle > 0.01) {
        throw std::runtime_error("FK does not match measured pose");
      }
      std::cout << "read_only_stopped_state=1 distinct_quiet_frames=" << quiet << '\n';
      result = 0;
    } else {
      v::JointVelcAccParam velocity{}, acceleration{}, got_velocity{}, got_acceleration{};
      std::fill_n(velocity.jointPara, 6, 0.02);
      std::fill_n(acceleration.jointPara, 6, 0.05);
      if (send.robotServiceInitGlobalMoveProfile() != 0 ||
        send.robotServiceSetGlobalMoveJointMaxVelc(velocity) != 0 ||
        send.robotServiceSetGlobalMoveJointMaxAcc(acceleration) != 0 ||
        send.robotServiceSetArrivalAheadTimeMode(0.05) != 0)
      {
        throw std::runtime_error("follow profile rejected");
      }
      send.robotServiceGetGlobalMoveJointMaxVelc(got_velocity);
      send.robotServiceGetGlobalMoveJointMaxAcc(got_acceleration);
      for (unsigned i = 0; i < 6; ++i) {
        if (std::abs(got_velocity.jointPara[i] - 0.02) > 1e-9 ||
          std::abs(got_acceleration.jointPara[i] - 0.05) > 1e-9)
        {
          throw std::runtime_error("joint profile readback mismatch");
        }
      }
      // Profile setup must not turn an old pose into a new motion target.
      {
        std::lock_guard<std::mutex> guard(sample.mutex);
        if (Clock::now() - sample.at > std::chrono::milliseconds(100)) {
          throw std::runtime_error("pose expired during setup");
        }
        for (unsigned i = 0; i < 6; ++i) {
          if (std::abs(sample.point.jointpos[i] - target[i]) > 1e-6) {
            throw std::runtime_error("pose changed during setup");
          }
        }
      }
      std::atomic<bool> done{false}, timed_out{false};
      std::thread watchdog([&]() {
          const auto until = Clock::now() + std::chrono::seconds(2);
          while (!done && Clock::now() < until) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
          if (!done) {timed_out = true; stop.robotMoveFastStop();}
        });
      dispatched = true;
      const auto began = Clock::now();
      int code = -1;
      try {
        code = send.robotServiceFollowModeJointMove(target.data());
      } catch (...) {
        done = true; watchdog.join(); throw;
      }
      done = true;
      watchdog.join();
      std::cout << "follow_current_pose_result=" << code << " elapsed_s=" <<
        std::chrono::duration<double>(Clock::now() - began).count() << '\n';
      result = code == 0 && !timed_out ? 0 : 1;
    }
  } catch (const std::exception & error) {
    std::cerr << "probe_failed=" << error.what() << '\n';
  }
  if (dispatched) {
    const int urgent = stop.robotMoveFastStop();
    const int ordered = send.robotMoveFastStop();
    std::cout << "stop_results=" << urgent << ',' << ordered << '\n';
    if (urgent || ordered) {result = 1;}
    unsigned sequence = 0, stopped_frames = 0;
    const auto until = Clock::now() + std::chrono::seconds(3);
    double maximum_joint_delta = 0.0;
    while (Clock::now() < until && stopped_frames < 3) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      v::RobotState state = v::RobotRunning;
      const bool stopped = read.robotServiceGetRobotCurrentState(state) == 0 &&
        state == v::RobotStopped;
      std::lock_guard<std::mutex> guard(sample.mutex);
      if (sample.count == sequence || Clock::now() - sample.at > std::chrono::milliseconds(100)) {
        continue;
      }
      sequence = sample.count;
      bool quiet = stopped;
      for (unsigned i = 0; i < 6; ++i) {
        const double delta = std::abs(sample.point.jointpos[i] - target[i]);
        maximum_joint_delta = std::max(maximum_joint_delta, delta);
        quiet = quiet && std::isfinite(delta) && delta < 1e-4;
      }
      stopped_frames = quiet ? stopped_frames + 1 : 0;
    }
    std::cout << "stationary_post_stop_frames=" << stopped_frames <<
      " maximum_joint_delta_rad=" << maximum_joint_delta << '\n';
    if (stopped_frames < 3 || maximum_joint_delta >= 1e-4) {result = 1;}
  }
  if (logged_read) {
    read.robotServiceRegisterRealTimeRoadPointCallback(nullptr, nullptr);
    read.robotServiceLogout();
  }
  if (logged_send) {send.robotServiceLogout();}
  if (logged_stop) {stop.robotServiceLogout();}
  close(lock);
  std::cout << "ACK is NOT proof of moving-target following or physical stop.\n";
  return result;
}
