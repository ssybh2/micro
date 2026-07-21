#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "custom_msgs/msg/read_djirc.hpp"
#include "custom_msgs/msg/read_dm_motor.hpp"
#include "custom_msgs/msg/write_dm_motor_mit_control.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

double clamp_value(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

double sign_or_throw(const double value, const std::string & name)
{
  if (std::abs(std::abs(value) - 1.0) > 1.0e-9) {
    throw std::runtime_error(name + " must be exactly +1 or -1");
  }
  return value;
}

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

bool normalize_quaternion(Quaternion & q)
{
  const double norm =
    std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

  if (!std::isfinite(norm) || norm < 1.0e-10) {
    return false;
  }

  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  return true;
}

double quaternion_dot(const Quaternion & a, const Quaternion & b)
{
  return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

Quaternion relative_quaternion(
  const Quaternion & reference,
  Quaternion current)
{
  if (quaternion_dot(reference, current) < 0.0) {
    current.w = -current.w;
    current.x = -current.x;
    current.y = -current.y;
    current.z = -current.z;
  }

  // conjugate(reference) * current
  Quaternion result;
  result.w =
    reference.w * current.w +
    reference.x * current.x +
    reference.y * current.y +
    reference.z * current.z;

  result.x =
    reference.w * current.x -
    reference.x * current.w -
    reference.y * current.z +
    reference.z * current.y;

  result.y =
    reference.w * current.y +
    reference.x * current.z -
    reference.y * current.w -
    reference.z * current.x;

  result.z =
    reference.w * current.z -
    reference.x * current.y +
    reference.y * current.x -
    reference.z * current.w;

  normalize_quaternion(result);
  return result;
}

double quaternion_roll(const Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.x + q.y * q.z),
    1.0 - 2.0 * (q.x * q.x + q.y * q.y));
}

class FirstOrderLowPass
{
public:
  FirstOrderLowPass() = default;

  FirstOrderLowPass(const double cutoff_hz, const double dt)
  {
    configure(cutoff_hz, dt);
  }

  void configure(const double cutoff_hz, const double dt)
  {
    cutoff_hz_ = cutoff_hz;
    dt_ = dt;
    initialized_ = false;
    value_ = 0.0;
  }

  void reset(const double value = 0.0)
  {
    value_ = value;
    initialized_ = true;
  }

  double update(const double input)
  {
    if (!initialized_) {
      reset(input);
      return input;
    }

    if (cutoff_hz_ <= 0.0) {
      value_ = input;
      return value_;
    }

    const double tau = 1.0 / (2.0 * kPi * cutoff_hz_);
    const double alpha = dt_ / (tau + dt_);
    value_ += alpha * (input - value_);
    return value_;
  }

private:
  double cutoff_hz_{0.0};
  double dt_{0.003};
  double value_{0.0};
  bool initialized_{false};
};

class WrappedAngleUnwrapper
{
public:
  void configure(const double half_range)
  {
    half_range_ = std::max(half_range, 1.0e-6);
    period_ = 2.0 * half_range_;
  }

  void reset(const double raw_angle)
  {
    last_raw_ = raw_angle;
    unwrapped_ = 0.0;
    initialized_ = true;
  }

  double update(const double raw_angle)
  {
    if (!initialized_) {
      reset(raw_angle);
      return 0.0;
    }

    double delta = raw_angle - last_raw_;

    while (delta > half_range_) {
      delta -= period_;
    }
    while (delta < -half_range_) {
      delta += period_;
    }

    unwrapped_ += delta;
    last_raw_ = raw_angle;
    return unwrapped_;
  }

private:
  double half_range_{kPi};
  double period_{2.0 * kPi};
  double last_raw_{0.0};
  double unwrapped_{0.0};
  bool initialized_{false};
};

struct ImuSample
{
  Quaternion orientation{};
  double gyro_x{0.0};
  rclcpp::Time received_time{0, 0, RCL_ROS_TIME};
  bool received{false};
};

struct RcSample
{
  bool online{false};
  std::uint8_t right_switch{0};
  double left_y{0.0};
  rclcpp::Time received_time{0, 0, RCL_ROS_TIME};
  bool received{false};
};

struct MotorSample
{
  bool online{false};
  bool disabled{false};
  bool enabled{false};
  bool overvoltage{false};
  bool undervoltage{false};
  bool overcurrent{false};
  bool mos_overtemperature{false};
  bool rotor_overtemperature{false};
  bool communication_lost{false};
  bool overload{false};
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  rclcpp::Time received_time{0, 0, RCL_ROS_TIME};
  bool received{false};

  bool has_fault() const
  {
    return
      overvoltage ||
      undervoltage ||
      overcurrent ||
      mos_overtemperature ||
      rotor_overtemperature ||
      communication_lost ||
      overload;
  }
};

}  // namespace


class MicroLqrController : public rclcpp::Node
{
public:
  MicroLqrController()
  : Node("micro_lqr_controller")
  {
    declare_and_load_parameters();
    configure_filters();

    const auto qos =
      rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability_volatile();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        imu_.orientation = {
          msg->orientation.w,
          msg->orientation.x,
          msg->orientation.y,
          msg->orientation.z};
        imu_.gyro_x = msg->angular_velocity.x;
        imu_.received_time = now();
        imu_.received = true;
      });

    rc_sub_ = create_subscription<custom_msgs::msg::ReadDJIRC>(
      rc_topic_,
      qos,
      [this](const custom_msgs::msg::ReadDJIRC::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        rc_.online = msg->online != 0U;
        rc_.right_switch = msg->right_switch;
        rc_.left_y = msg->left_y;
        rc_.received_time = now();
        rc_.received = true;
      });

    left_motor_sub_ =
      create_subscription<custom_msgs::msg::ReadDmMotor>(
      left_motor_read_topic_,
      qos,
      [this](const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        copy_motor_message(*msg, left_motor_);
      });

    right_motor_sub_ =
      create_subscription<custom_msgs::msg::ReadDmMotor>(
      right_motor_read_topic_,
      qos,
      [this](const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        copy_motor_message(*msg, right_motor_);
      });

    left_motor_pub_ =
      create_publisher<custom_msgs::msg::WriteDmMotorMITControl>(
      left_motor_write_topic_, qos);

    right_motor_pub_ =
      create_publisher<custom_msgs::msg::WriteDmMotorMITControl>(
      right_motor_write_topic_, qos);

    debug_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
      debug_topic_, qos);

    const auto timer_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(control_period_s_));

    control_timer_ = create_wall_timer(
      timer_period,
      std::bind(&MicroLqrController::control_step, this));

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(
        &MicroLqrController::on_parameter_change,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "micro_lqr_controller started at %.3f Hz; dry_run=%s",
      1.0 / control_period_s_,
      dry_run_ ? "true" : "false");

    RCLCPP_INFO(
      get_logger(),
      "RC right switch: calibrate=%d, arm=%d, disable=%d",
      calibrate_switch_value_,
      arm_switch_value_,
      disable_switch_value_);
  }

  ~MicroLqrController() override
  {
    publish_motor_commands(false, 0.0, 0.0);
  }

private:
  void declare_and_load_parameters()
  {
    imu_topic_ = declare_parameter<std::string>(
      "imu_topic", "/ecat/sn2031674/app1/read");
    rc_topic_ = declare_parameter<std::string>(
      "rc_topic", "/ecat/sn2031674/app2/read");
    left_motor_read_topic_ = declare_parameter<std::string>(
      "left_motor_read_topic", "/ecat/sn2031674/app3/read");
    left_motor_write_topic_ = declare_parameter<std::string>(
      "left_motor_write_topic", "/ecat/sn2031674/app3/write");
    right_motor_read_topic_ = declare_parameter<std::string>(
      "right_motor_read_topic", "/ecat/sn2031674/app4/read");
    right_motor_write_topic_ = declare_parameter<std::string>(
      "right_motor_write_topic", "/ecat/sn2031674/app4/write");
    debug_topic_ = declare_parameter<std::string>(
      "debug_topic", "/micro_lqr/debug");

    control_period_s_ = declare_parameter<double>(
      "control_period_s", 0.003);

    dry_run_ = declare_parameter<bool>("dry_run", true);
    torque_limit_ = declare_parameter<double>("torque_limit", 0.05);
    hard_torque_limit_ = declare_parameter<double>(
      "hard_torque_limit", 0.45);

    arm_max_tilt_rad_ =
      declare_parameter<double>("arm_max_tilt_deg", 8.0) * kPi / 180.0;
    fall_cutoff_rad_ =
      declare_parameter<double>("fall_cutoff_deg", 25.0) * kPi / 180.0;

    imu_timeout_s_ = declare_parameter<double>("imu_timeout_s", 0.05);
    motor_timeout_s_ = declare_parameter<double>("motor_timeout_s", 0.05);
    rc_timeout_s_ = declare_parameter<double>("rc_timeout_s", 0.20);

    calibrate_switch_value_ = declare_parameter<int>(
      "calibrate_switch_value", 1);
    arm_switch_value_ = declare_parameter<int>(
      "arm_switch_value", 3);
    disable_switch_value_ = declare_parameter<int>(
      "disable_switch_value", 2);

    enable_velocity_command_ = declare_parameter<bool>(
      "enable_velocity_command", false);
    max_target_velocity_ = declare_parameter<double>(
      "max_target_velocity", 0.30);
    rc_deadband_ = declare_parameter<double>("rc_deadband", 0.08);

    k_x_ = declare_parameter<double>("k_x", 1.88271299656);
    k_x_dot_ = declare_parameter<double>("k_x_dot", 1.32543605302);
    k_pitch_ = declare_parameter<double>("k_pitch", 3.38842472039);
    k_pitch_rate_ = declare_parameter<double>(
      "k_pitch_rate", 0.323694284635);

    output_gain_sign_ = sign_or_throw(
      declare_parameter<double>("output_gain_sign", 1.0),
      "output_gain_sign");
    left_motor_sign_ = sign_or_throw(
      declare_parameter<double>("left_motor_sign", -1.0),
      "left_motor_sign");
    right_motor_sign_ = sign_or_throw(
      declare_parameter<double>("right_motor_sign", 1.0),
      "right_motor_sign");

    left_encoder_sign_ = sign_or_throw(
      declare_parameter<double>("left_encoder_sign", 1.0),
      "left_encoder_sign");
    right_encoder_sign_ = sign_or_throw(
      declare_parameter<double>("right_encoder_sign", -1.0),
      "right_encoder_sign");

    imu_angle_sign_ = sign_or_throw(
      declare_parameter<double>("imu_angle_sign", 1.0),
      "imu_angle_sign");
    imu_rate_sign_ = sign_or_throw(
      declare_parameter<double>("imu_rate_sign", 1.0),
      "imu_rate_sign");

    pitch_position_compensation_sign_ = sign_or_throw(
      declare_parameter<double>(
        "pitch_position_compensation_sign", 1.0),
      "pitch_position_compensation_sign");

    pitch_rate_compensation_sign_ = sign_or_throw(
      declare_parameter<double>(
        "pitch_rate_compensation_sign", 1.0),
      "pitch_rate_compensation_sign");

    wheel_radius_m_ = declare_parameter<double>(
      "wheel_radius_m", 0.030);

    motor_position_wrap_half_range_ = declare_parameter<double>(
      "motor_position_wrap_half_range", kPi);

    pitch_filter_hz_ = declare_parameter<double>(
      "pitch_filter_hz", 40.0);
    pitch_rate_filter_hz_ = declare_parameter<double>(
      "pitch_rate_filter_hz", 45.0);
    wheel_velocity_filter_hz_ = declare_parameter<double>(
      "wheel_velocity_filter_hz", 30.0);

    auto_calibrate_on_first_valid_data_ = declare_parameter<bool>(
      "auto_calibrate_on_first_valid_data", false);

    validate_limits();

    left_unwrapper_.configure(motor_position_wrap_half_range_);
    right_unwrapper_.configure(motor_position_wrap_half_range_);
  }

  void validate_limits()
  {
    if (control_period_s_ <= 0.0) {
      throw std::runtime_error("control_period_s must be positive");
    }
    if (hard_torque_limit_ <= 0.0) {
      throw std::runtime_error("hard_torque_limit must be positive");
    }
    if (torque_limit_ <= 0.0 || torque_limit_ > hard_torque_limit_) {
      throw std::runtime_error(
        "torque_limit must be positive and <= hard_torque_limit");
    }
    if (wheel_radius_m_ <= 0.0) {
      throw std::runtime_error("wheel_radius_m must be positive");
    }
  }

  void configure_filters()
  {
    pitch_filter_.configure(pitch_filter_hz_, control_period_s_);
    pitch_rate_filter_.configure(
      pitch_rate_filter_hz_, control_period_s_);
    wheel_velocity_filter_.configure(
      wheel_velocity_filter_hz_, control_period_s_);
  }

  void copy_motor_message(
    const custom_msgs::msg::ReadDmMotor & msg,
    MotorSample & sample)
  {
    sample.online = msg.online != 0U;
    sample.disabled = msg.disabled != 0U;
    sample.enabled = msg.enabled != 0U;
    sample.overvoltage = msg.overvoltage != 0U;
    sample.undervoltage = msg.undervoltage != 0U;
    sample.overcurrent = msg.overcurrent != 0U;
    sample.mos_overtemperature = msg.mos_overtemperature != 0U;
    sample.rotor_overtemperature = msg.rotor_overtemperature != 0U;
    sample.communication_lost = msg.communication_lost != 0U;
    sample.overload = msg.overload != 0U;
    sample.position = msg.position;
    sample.velocity = msg.velocity;
    sample.torque = msg.torque;
    sample.received_time = now();
    sample.received = true;
  }

  bool is_fresh(
    const bool received,
    const rclcpp::Time & stamp,
    const double timeout_s,
    const rclcpp::Time & current_time) const
  {
    return received && (current_time - stamp).seconds() <= timeout_s;
  }

  bool all_input_data_valid(const rclcpp::Time & current_time) const
  {
    return
      is_fresh(
        imu_.received, imu_.received_time,
        imu_timeout_s_, current_time) &&
      is_fresh(
        rc_.received, rc_.received_time,
        rc_timeout_s_, current_time) &&
      is_fresh(
        left_motor_.received, left_motor_.received_time,
        motor_timeout_s_, current_time) &&
      is_fresh(
        right_motor_.received, right_motor_.received_time,
        motor_timeout_s_, current_time) &&
      rc_.online &&
      left_motor_.online &&
      right_motor_.online &&
      !left_motor_.has_fault() &&
      !right_motor_.has_fault();
  }

  bool calculate_raw_pitch(double & pitch) const
  {
    if (!imu_zero_valid_) {
      return false;
    }

    Quaternion current = imu_.orientation;
    if (!normalize_quaternion(current)) {
      return false;
    }

    const Quaternion relative =
      relative_quaternion(imu_zero_, current);

    pitch = imu_angle_sign_ * quaternion_roll(relative);
    return std::isfinite(pitch);
  }

  void calibrate_zero()
  {
    Quaternion current = imu_.orientation;

    if (!normalize_quaternion(current)) {
      RCLCPP_ERROR(
        get_logger(),
        "Calibration rejected: invalid IMU quaternion");
      return;
    }

    imu_zero_ = current;
    imu_zero_valid_ = true;

    left_unwrapper_.reset(left_motor_.position);
    right_unwrapper_.reset(right_motor_.position);

    pitch_filter_.reset(0.0);
    pitch_rate_filter_.reset(0.0);
    wheel_velocity_filter_.reset(0.0);

    target_position_m_ = 0.0;
    target_velocity_mps_ = 0.0;
    calibrated_ = true;
    armed_ = false;
    arm_transition_required_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Zero calibrated: IMU attitude and wheel positions stored");
  }

  void disarm(const char * reason, const bool require_switch_cycle)
  {
    if (armed_) {
      RCLCPP_WARN(get_logger(), "LQR disarmed: %s", reason);
    }

    armed_ = false;
    if (require_switch_cycle) {
      arm_transition_required_ = true;
    }

    publish_motor_commands(false, 0.0, 0.0);
  }

  void attempt_arm()
  {
    double pitch = 0.0;

    if (!calibrated_ || !calculate_raw_pitch(pitch)) {
      RCLCPP_WARN(
        get_logger(),
        "Arm rejected: calibrate zero first");
      return;
    }

    if (std::abs(pitch) > arm_max_tilt_rad_) {
      RCLCPP_WARN(
        get_logger(),
        "Arm rejected: |pitch|=%.2f deg exceeds %.2f deg",
        std::abs(pitch) * 180.0 / kPi,
        arm_max_tilt_rad_ * 180.0 / kPi);
      return;
    }

    // Use the current position as the LQR position reference.
    const double left_angle =
      left_encoder_sign_ * left_unwrapper_.update(
      left_motor_.position);
    const double right_angle =
      right_encoder_sign_ * right_unwrapper_.update(
      right_motor_.position);
    const double mean_relative_angle =
      0.5 * (left_angle + right_angle);

    target_position_m_ = wheel_radius_m_ * (
      mean_relative_angle +
      pitch_position_compensation_sign_ * pitch);

    target_velocity_mps_ = 0.0;
    armed_ = true;
    arm_transition_required_ = false;

    RCLCPP_INFO(
      get_logger(),
      "LQR armed; target_position=%.5f m; dry_run=%s",
      target_position_m_,
      dry_run_ ? "true" : "false");
  }

  double process_rc_velocity_command() const
  {
    if (!enable_velocity_command_) {
      return 0.0;
    }

    double command = clamp_value(rc_.left_y, -1.0, 1.0);

    if (std::abs(command) <= rc_deadband_) {
      return 0.0;
    }

    const double scaled =
      (std::abs(command) - rc_deadband_) /
      std::max(1.0 - rc_deadband_, 1.0e-6);

    return std::copysign(
      scaled * max_target_velocity_, command);
  }

  void control_step()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const rclcpp::Time current_time = now();

    if (!all_input_data_valid(current_time)) {
      disarm("input timeout, RC offline, motor offline, or motor fault", true);
      last_switch_value_ = 255;
      publish_debug(
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0);
      return;
    }

    const int switch_value = static_cast<int>(rc_.right_switch);
    const bool switch_changed = switch_value != last_switch_value_;

    if (
      auto_calibrate_on_first_valid_data_ &&
      !calibrated_)
    {
      calibrate_zero();
    }

    if (
      switch_changed &&
      switch_value == calibrate_switch_value_)
    {
      disarm("calibration switch selected", false);
      calibrate_zero();
    }

    if (switch_value != arm_switch_value_) {
      disarm("RC switch is not in arm position", false);
      arm_transition_required_ = false;
      last_switch_value_ = switch_value;

      publish_debug(
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0);
      return;
    }

    if (switch_changed) {
      arm_transition_required_ = false;
      attempt_arm();
    } else if (
      !armed_ &&
      !arm_transition_required_)
    {
      // Allows first arm after startup if the switch was already moved
      // from another valid position into arm.
      attempt_arm();
    }

    last_switch_value_ = switch_value;

    if (!armed_) {
      publish_motor_commands(false, 0.0, 0.0);
      return;
    }

    double pitch_raw = 0.0;
    if (!calculate_raw_pitch(pitch_raw)) {
      disarm("invalid IMU quaternion", true);
      return;
    }

    const double pitch_rate_raw =
      imu_rate_sign_ * imu_.gyro_x;

    const double pitch =
      pitch_filter_.update(pitch_raw);
    const double pitch_rate =
      pitch_rate_filter_.update(pitch_rate_raw);

    if (std::abs(pitch) > fall_cutoff_rad_) {
      disarm("fall angle exceeded", true);
      return;
    }

    const double left_angle =
      left_encoder_sign_ * left_unwrapper_.update(
      left_motor_.position);
    const double right_angle =
      right_encoder_sign_ * right_unwrapper_.update(
      right_motor_.position);

    const double mean_relative_angle =
      0.5 * (left_angle + right_angle);

    const double mean_relative_rate_raw =
      0.5 * (
      left_encoder_sign_ * left_motor_.velocity +
      right_encoder_sign_ * right_motor_.velocity);

    const double mean_relative_rate =
      wheel_velocity_filter_.update(
      mean_relative_rate_raw);

    const double position_m = wheel_radius_m_ * (
      mean_relative_angle +
      pitch_position_compensation_sign_ * pitch);

    const double velocity_mps = wheel_radius_m_ * (
      mean_relative_rate +
      pitch_rate_compensation_sign_ * pitch_rate);

    target_velocity_mps_ = process_rc_velocity_command();
    target_position_m_ +=
      target_velocity_mps_ * control_period_s_;

    const double position_error =
      position_m - target_position_m_;
    const double velocity_error =
      velocity_mps - target_velocity_mps_;
    const double pitch_error = pitch;
    const double pitch_rate_error = pitch_rate;

    const double lqr_raw = -(
      k_x_ * position_error +
      k_x_dot_ * velocity_error +
      k_pitch_ * pitch_error +
      k_pitch_rate_ * pitch_rate_error);

    const double common_torque_unsaturated =
      output_gain_sign_ * lqr_raw;

    const double common_torque = clamp_value(
      common_torque_unsaturated,
      -torque_limit_,
      torque_limit_);

    const double left_command =
      left_motor_sign_ * common_torque;
    const double right_command =
      right_motor_sign_ * common_torque;

    if (dry_run_) {
      publish_motor_commands(false, 0.0, 0.0);
    } else {
      publish_motor_commands(
        true,
        left_command,
        right_command);
    }

    publish_debug(
      position_m,
      velocity_mps,
      pitch,
      pitch_rate,
      target_position_m_,
      target_velocity_mps_,
      lqr_raw,
      common_torque);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "armed=%d dry=%d x=%+.4f v=%+.4f pitch=%+.2fdeg "
      "rate=%+.3f u=%+.4f L=%+.4f R=%+.4f",
      armed_ ? 1 : 0,
      dry_run_ ? 1 : 0,
      position_m,
      velocity_mps,
      pitch * 180.0 / kPi,
      pitch_rate,
      common_torque,
      left_command,
      right_command);
  }

  void publish_motor_commands(
    const bool enable,
    const double left_torque,
    const double right_torque)
  {
    custom_msgs::msg::WriteDmMotorMITControl left_message;
    custom_msgs::msg::WriteDmMotorMITControl right_message;

    left_message.enable = enable ? 1U : 0U;
    left_message.p_des = 0.0F;
    left_message.v_des = 0.0F;
    left_message.kp = 0.0F;
    left_message.kd = 0.0F;
    left_message.torque = static_cast<float>(
      clamp_value(
        left_torque,
        -hard_torque_limit_,
        hard_torque_limit_));

    right_message.enable = enable ? 1U : 0U;
    right_message.p_des = 0.0F;
    right_message.v_des = 0.0F;
    right_message.kp = 0.0F;
    right_message.kd = 0.0F;
    right_message.torque = static_cast<float>(
      clamp_value(
        right_torque,
        -hard_torque_limit_,
        hard_torque_limit_));

    left_motor_pub_->publish(left_message);
    right_motor_pub_->publish(right_message);
  }

  void publish_debug(
    const double position,
    const double velocity,
    const double pitch,
    const double pitch_rate,
    const double target_position,
    const double target_velocity,
    const double lqr_raw,
    const double common_torque)
  {
    std_msgs::msg::Float64MultiArray message;
    message.data = {
      position,
      velocity,
      pitch,
      pitch_rate,
      target_position,
      target_velocity,
      lqr_raw,
      common_torque,
      armed_ ? 1.0 : 0.0,
      dry_run_ ? 1.0 : 0.0,
      output_gain_sign_,
      torque_limit_};

    debug_pub_->publish(message);
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool force_disarm = false;

    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();

      try {
        if (name == "dry_run") {
          dry_run_ = parameter.as_bool();
          force_disarm = true;
        } else if (name == "torque_limit") {
          const double value = parameter.as_double();
          if (value <= 0.0 || value > hard_torque_limit_) {
            throw std::runtime_error(
              "torque_limit must be >0 and <= hard_torque_limit");
          }
          torque_limit_ = value;
          force_disarm = true;
        } else if (name == "output_gain_sign") {
          output_gain_sign_ = sign_or_throw(
            parameter.as_double(), name);
          force_disarm = true;
        } else if (name == "left_motor_sign") {
          left_motor_sign_ = sign_or_throw(
            parameter.as_double(), name);
          force_disarm = true;
        } else if (name == "right_motor_sign") {
          right_motor_sign_ = sign_or_throw(
            parameter.as_double(), name);
          force_disarm = true;
        } else if (name == "left_encoder_sign") {
          left_encoder_sign_ = sign_or_throw(
            parameter.as_double(), name);
          force_disarm = true;
        } else if (name == "right_encoder_sign") {
          right_encoder_sign_ = sign_or_throw(
            parameter.as_double(), name);
          force_disarm = true;
        } else if (name == "imu_angle_sign") {
          imu_angle_sign_ = sign_or_throw(
            parameter.as_double(), name);
          force_disarm = true;
        } else if (name == "imu_rate_sign") {
          imu_rate_sign_ = sign_or_throw(
            parameter.as_double(), name);
          force_disarm = true;
        } else if (name == "k_x") {
          k_x_ = parameter.as_double();
          force_disarm = true;
        } else if (name == "k_x_dot") {
          k_x_dot_ = parameter.as_double();
          force_disarm = true;
        } else if (name == "k_pitch") {
          k_pitch_ = parameter.as_double();
          force_disarm = true;
        } else if (name == "k_pitch_rate") {
          k_pitch_rate_ = parameter.as_double();
          force_disarm = true;
        } else if (name == "enable_velocity_command") {
          enable_velocity_command_ = parameter.as_bool();
          force_disarm = true;
        } else if (name == "max_target_velocity") {
          max_target_velocity_ = parameter.as_double();
        }
      } catch (const std::exception & exception) {
        result.successful = false;
        result.reason = exception.what();
        return result;
      }
    }

    if (force_disarm) {
      armed_ = false;
      arm_transition_required_ = true;
      publish_motor_commands(false, 0.0, 0.0);
      RCLCPP_WARN(
        get_logger(),
        "Safety-sensitive parameter changed; move RC switch away "
        "from ARM and back to ARM");
    }

    return result;
  }

  std::mutex data_mutex_;

  ImuSample imu_;
  RcSample rc_;
  MotorSample left_motor_;
  MotorSample right_motor_;

  Quaternion imu_zero_{};
  bool imu_zero_valid_{false};
  bool calibrated_{false};
  bool armed_{false};
  bool arm_transition_required_{true};
  int last_switch_value_{255};

  WrappedAngleUnwrapper left_unwrapper_;
  WrappedAngleUnwrapper right_unwrapper_;
  FirstOrderLowPass pitch_filter_;
  FirstOrderLowPass pitch_rate_filter_;
  FirstOrderLowPass wheel_velocity_filter_;

  double target_position_m_{0.0};
  double target_velocity_mps_{0.0};

  std::string imu_topic_;
  std::string rc_topic_;
  std::string left_motor_read_topic_;
  std::string left_motor_write_topic_;
  std::string right_motor_read_topic_;
  std::string right_motor_write_topic_;
  std::string debug_topic_;

  double control_period_s_{0.003};
  bool dry_run_{true};
  double torque_limit_{0.05};
  double hard_torque_limit_{0.45};
  double arm_max_tilt_rad_{8.0 * kPi / 180.0};
  double fall_cutoff_rad_{25.0 * kPi / 180.0};
  double imu_timeout_s_{0.05};
  double motor_timeout_s_{0.05};
  double rc_timeout_s_{0.20};

  int calibrate_switch_value_{1};
  int arm_switch_value_{3};
  int disable_switch_value_{2};

  bool enable_velocity_command_{false};
  double max_target_velocity_{0.30};
  double rc_deadband_{0.08};

  double k_x_{1.88271299656};
  double k_x_dot_{1.32543605302};
  double k_pitch_{3.38842472039};
  double k_pitch_rate_{0.323694284635};

  double output_gain_sign_{1.0};
  double left_motor_sign_{-1.0};
  double right_motor_sign_{1.0};
  double left_encoder_sign_{1.0};
  double right_encoder_sign_{-1.0};
  double imu_angle_sign_{1.0};
  double imu_rate_sign_{1.0};
  double pitch_position_compensation_sign_{1.0};
  double pitch_rate_compensation_sign_{1.0};

  double wheel_radius_m_{0.030};
  double motor_position_wrap_half_range_{kPi};
  double pitch_filter_hz_{40.0};
  double pitch_rate_filter_hz_{45.0};
  double wheel_velocity_filter_hz_{30.0};
  bool auto_calibrate_on_first_valid_data_{false};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<custom_msgs::msg::ReadDJIRC>::SharedPtr rc_sub_;
  rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr
    left_motor_sub_;
  rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr
    right_motor_sub_;

  rclcpp::Publisher<
    custom_msgs::msg::WriteDmMotorMITControl>::SharedPtr
    left_motor_pub_;
  rclcpp::Publisher<
    custom_msgs::msg::WriteDmMotorMITControl>::SharedPtr
    right_motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    debug_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<MicroLqrController>());
  } catch (const std::exception & exception) {
    std::cerr << "micro_lqr_controller fatal error: "
              << exception.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
