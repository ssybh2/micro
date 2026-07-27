#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
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

double apply_continuous_deadband(const double value, const double deadband)
{
  const double magnitude = std::abs(value);
  if (magnitude <= deadband) {
    return 0.0;
  }
  return std::copysign(magnitude - deadband, value);
}

double shape_unit_stick(const double value, const double deadband)
{
  const double clamped = clamp_value(value, -1.0, 1.0);
  return apply_continuous_deadband(clamped, deadband) /
         std::max(1.0 - deadband, 1.0e-6);
}

double sign_or_throw(const double value, const std::string & name)
{
  if (!std::isfinite(value) || std::abs(std::abs(value) - 1.0) > 1.0e-9) {
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
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
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

Quaternion relative_quaternion(const Quaternion & reference, Quaternion current)
{
  if (quaternion_dot(reference, current) < 0.0) {
    current.w = -current.w;
    current.x = -current.x;
    current.y = -current.y;
    current.z = -current.z;
  }

  Quaternion result;
  result.w = reference.w * current.w + reference.x * current.x +
             reference.y * current.y + reference.z * current.z;
  result.x = reference.w * current.x - reference.x * current.w -
             reference.y * current.z + reference.z * current.y;
  result.y = reference.w * current.y + reference.x * current.z -
             reference.y * current.w - reference.z * current.x;
  result.z = reference.w * current.z - reference.x * current.y +
             reference.y * current.x - reference.z * current.w;
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
  void configure(const double cutoff_hz, const double nominal_dt)
  {
    cutoff_hz_ = cutoff_hz;
    nominal_dt_ = nominal_dt;
    initialized_ = false;
    value_ = 0.0;
  }

  void reset(const double value = 0.0)
  {
    value_ = value;
    initialized_ = true;
  }

  double update(const double input, const double actual_dt)
  {
    if (!initialized_) {
      reset(input);
      return input;
    }
    if (cutoff_hz_ <= 0.0) {
      value_ = input;
      return value_;
    }
    double dt = nominal_dt_;
    if (std::isfinite(actual_dt) && actual_dt > 1.0e-6) {
      dt = clamp_value(actual_dt, 1.0e-6, 0.050);
    }
    const double tau = 1.0 / (2.0 * kPi * cutoff_hz_);
    const double alpha = dt / (tau + dt);
    value_ += alpha * (input - value_);
    return value_;
  }

  double value() const {return value_;}

private:
  double cutoff_hz_{0.0};
  double nominal_dt_{0.003};
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
    while (delta > half_range_) {delta -= period_;}
    while (delta < -half_range_) {delta += period_;}
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
  double gyro_z{0.0};
  rclcpp::Time received_time{0, 0, RCL_ROS_TIME};
  bool received{false};
};

struct RcSample
{
  bool online{false};
  std::uint8_t right_switch{0};
  double right_y{0.0};
  double left_x{0.0};
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
  std::uint64_t sequence{0};
  bool received{false};

  bool has_fault() const
  {
    return overvoltage || undervoltage || overcurrent || mos_overtemperature ||
           rotor_overtemperature || communication_lost || overload;
  }
};

struct DebugSample
{
  double position{0.0};
  double velocity{0.0};
  double pitch{0.0};
  double pitch_rate{0.0};
  double target_position{0.0};
  double target_velocity{0.0};
  double model_total_unsaturated{0.0};
  double per_wheel_common_torque{0.0};
  double pitch_raw{0.0};
  double pitch_rate_raw{0.0};
  double u_x{0.0};
  double u_x_dot{0.0};
  double u_pitch{0.0};
  double u_pitch_rate{0.0};
  double total_torque_after_limit{0.0};
  double saturation_flag{0.0};
  double actual_dt{0.0};
  double imu_age_s{0.0};
  double left_motor_age_s{0.0};
  double right_motor_age_s{0.0};
  double left_position_raw{0.0};
  double right_position_raw{0.0};
  double left_velocity_raw{0.0};
  double right_velocity_raw{0.0};
  double velocity_from_position{0.0};
  double position_error{0.0};
  double velocity_error{0.0};
  double velocity_motor_based{0.0};
  double velocity_mismatch{0.0};
  double cascade_mode_flag{1.0};
  double pitch_setpoint_raw{0.0};
  double pitch_setpoint_limited{0.0};
  double pitch_setpoint_command{0.0};
  double outer_velocity_filtered{0.0};
  double position_integral{0.0};
  double attitude_error{0.0};
  double outer_saturation_flag{0.0};
  double yaw_enabled_flag{0.0};
  double yaw_rate_raw{0.0};
  double yaw_rate_filtered{0.0};
  double yaw_rate_for_control{0.0};
  double yaw_rate_error{0.0};
  double yaw_acceleration_filtered{0.0};
  double yaw_differential_raw{0.0};
  double yaw_differential_command{0.0};
  double yaw_available_headroom{0.0};
  double yaw_limited_flag{0.0};
  double left_physical_torque{0.0};
  double right_physical_torque{0.0};
  double left_motor_command{0.0};
  double right_motor_command{0.0};
  double rc_right_y_raw{0.0};
  double rc_left_x_raw{0.0};
  double rc_forward_shaped{0.0};
  double rc_yaw_shaped{0.0};
  double rc_velocity_command{0.0};
  double rc_yaw_rate_command{0.0};
  double rc_velocity_slew_limited_flag{0.0};
  double rc_yaw_slew_limited_flag{0.0};

  // Appended schema: old indices 0..72 remain unchanged.
  double velocity_from_position_raw{0.0};       // 73
  double velocity_from_position_filtered{0.0};  // 74
  double manual_trim{0.0};                      // 75
  double auto_trim{0.0};                        // 76
  double total_trim{0.0};                       // 77
  double full_pitch_reference{0.0};             // 78
  double auto_trim_learning_flag{0.0};          // 79
  double encoder_pair_updated_flag{0.0};        // 80
  double stiction_compensation_total{0.0};       // 81
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

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        imu_.orientation = {msg->orientation.w, msg->orientation.x,
          msg->orientation.y, msg->orientation.z};
        imu_.gyro_x = msg->angular_velocity.x;
        imu_.gyro_z = msg->angular_velocity.z;
        imu_.received_time = now();
        imu_.received = true;
      });
    rc_sub_ = create_subscription<custom_msgs::msg::ReadDJIRC>(
      rc_topic_, qos, [this](const custom_msgs::msg::ReadDJIRC::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        rc_.online = msg->online != 0U;
        rc_.right_switch = msg->right_switch;
        rc_.right_y = msg->right_y;
        rc_.left_x = msg->left_x;
        rc_.received_time = now();
        rc_.received = true;
      });
    left_motor_sub_ = create_subscription<custom_msgs::msg::ReadDmMotor>(
      left_motor_read_topic_, qos, [this](const custom_msgs::msg::ReadDmMotor::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        copy_motor_message(*msg, left_motor_);
      });
    right_motor_sub_ = create_subscription<custom_msgs::msg::ReadDmMotor>(
      right_motor_read_topic_, qos, [this](const custom_msgs::msg::ReadDmMotor::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        copy_motor_message(*msg, right_motor_);
      });

    left_motor_pub_ = create_publisher<custom_msgs::msg::WriteDmMotorMITControl>(
      left_motor_write_topic_, qos);
    right_motor_pub_ = create_publisher<custom_msgs::msg::WriteDmMotorMITControl>(
      right_motor_write_topic_, qos);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(debug_topic_, qos);

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(control_period_s_));
    control_timer_ = create_wall_timer(
      timer_period, std::bind(&MicroLqrController::control_step, this));

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&MicroLqrController::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "controller started at %.3f Hz; mode=%s; dry_run=%s; independent position velocity LPF=%.1fHz",
      1.0 / control_period_s_, control_mode_.c_str(), dry_run_ ? "true" : "false",
      position_velocity_filter_hz_);
    RCLCPP_INFO(
      get_logger(),
      "trim: manual=%+.3fdeg auto=%s gain=%.4frad/(m*s) limit=%.2fdeg dwell=%.2fs",
      manual_trim_rad_ * 180.0 / kPi, auto_trim_enable_ ? "true" : "false",
      auto_trim_gain_rad_per_m_s_, auto_trim_limit_rad_ * 180.0 / kPi,
      auto_trim_dwell_s_);
  }

  ~MicroLqrController() override
  {
    publish_motor_commands(false, 0.0, 0.0);
  }

private:
  void declare_and_load_parameters()
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/ecat/sn2031674/app1/read");
    rc_topic_ = declare_parameter<std::string>("rc_topic", "/ecat/sn2031674/app2/read");
    left_motor_read_topic_ = declare_parameter<std::string>(
      "left_motor_read_topic", "/ecat/sn2031674/app3/read");
    left_motor_write_topic_ = declare_parameter<std::string>(
      "left_motor_write_topic", "/ecat/sn2031674/app3/write");
    right_motor_read_topic_ = declare_parameter<std::string>(
      "right_motor_read_topic", "/ecat/sn2031674/app4/read");
    right_motor_write_topic_ = declare_parameter<std::string>(
      "right_motor_write_topic", "/ecat/sn2031674/app4/write");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/micro_lqr/debug");

    control_mode_ = declare_parameter<std::string>("control_mode", "cascade");
    control_period_s_ = declare_parameter<double>("control_period_s", 0.003);
    dry_run_ = declare_parameter<bool>("dry_run", true);
    per_wheel_torque_limit_ = declare_parameter<double>("torque_limit", 0.05);
    hard_per_wheel_torque_limit_ = declare_parameter<double>("hard_torque_limit", 0.45);
    lqr_gain_scale_ = declare_parameter<double>("lqr_gain_scale", 1.0);

    arm_max_tilt_rad_ = declare_parameter<double>("arm_max_tilt_deg", 3.0) * kPi / 180.0;
    arm_max_pitch_rate_rad_s_ = declare_parameter<double>("arm_max_pitch_rate_rad_s", 0.30);
    fall_cutoff_rad_ = declare_parameter<double>("fall_cutoff_deg", 25.0) * kPi / 180.0;
    imu_timeout_s_ = declare_parameter<double>("imu_timeout_s", 0.05);
    motor_timeout_s_ = declare_parameter<double>("motor_timeout_s", 0.05);
    rc_timeout_s_ = declare_parameter<double>("rc_timeout_s", 0.20);

    calibrate_switch_value_ = declare_parameter<int>("calibrate_switch_value", 1);
    arm_switch_value_ = declare_parameter<int>("arm_switch_value", 3);
    disable_switch_value_ = declare_parameter<int>("disable_switch_value", 2);

    enable_velocity_command_ = declare_parameter<bool>("enable_velocity_command", true);
    max_target_velocity_ = declare_parameter<double>("max_target_velocity", 0.20);
    enable_yaw_rate_command_ = declare_parameter<bool>("enable_yaw_rate_command", true);
    max_target_yaw_rate_rad_s_ = declare_parameter<double>("max_target_yaw_rate_rad_s", 0.80);
    rc_deadband_ = declare_parameter<double>("rc_deadband", 0.08);
    rc_forward_sign_ = sign_or_throw(
      declare_parameter<double>("rc.forward_sign", 1.0), "rc.forward_sign");
    rc_yaw_sign_ = sign_or_throw(
      declare_parameter<double>("rc.yaw_sign", 1.0), "rc.yaw_sign");
    rc_velocity_slew_rate_mps2_ = declare_parameter<double>(
      "rc.velocity_slew_rate_mps2", 0.60);
    rc_yaw_rate_slew_rate_rad_s2_ = declare_parameter<double>(
      "rc.yaw_rate_slew_rate_rad_s2", 3.0);

    output_gain_sign_ = sign_or_throw(
      declare_parameter<double>("output_gain_sign", 1.0), "output_gain_sign");
    left_motor_sign_ = sign_or_throw(
      declare_parameter<double>("left_motor_sign", -1.0), "left_motor_sign");
    right_motor_sign_ = sign_or_throw(
      declare_parameter<double>("right_motor_sign", 1.0), "right_motor_sign");
    left_encoder_sign_ = sign_or_throw(
      declare_parameter<double>("left_encoder_sign", 1.0), "left_encoder_sign");
    right_encoder_sign_ = sign_or_throw(
      declare_parameter<double>("right_encoder_sign", -1.0), "right_encoder_sign");
    imu_angle_sign_ = sign_or_throw(
      declare_parameter<double>("imu_angle_sign", 1.0), "imu_angle_sign");
    imu_rate_sign_ = sign_or_throw(
      declare_parameter<double>("imu_rate_sign", 1.0), "imu_rate_sign");
    pitch_position_compensation_sign_ = sign_or_throw(
      declare_parameter<double>("pitch_position_compensation_sign", 1.0),
      "pitch_position_compensation_sign");
    pitch_rate_compensation_sign_ = sign_or_throw(
      declare_parameter<double>("pitch_rate_compensation_sign", 1.0),
      "pitch_rate_compensation_sign");

    yaw_enable_ = declare_parameter<bool>("yaw.enable", true);
    yaw_imu_rate_sign_ = sign_or_throw(
      declare_parameter<double>("yaw.imu_rate_sign", 1.0), "yaw.imu_rate_sign");
    yaw_output_sign_ = sign_or_throw(
      declare_parameter<double>("yaw.output_sign", 1.0), "yaw.output_sign");
    yaw_rate_kp_per_wheel_nm_per_rad_s_ = declare_parameter<double>(
      "yaw.rate_kp_per_wheel_nm_per_rad_s", 0.015);
    yaw_rate_kd_per_wheel_nm_per_rad_s2_ = declare_parameter<double>(
      "yaw.rate_kd_per_wheel_nm_per_rad_s2", 0.0);
    yaw_rate_deadband_rad_s_ = declare_parameter<double>("yaw.rate_deadband_rad_s", 0.02);
    yaw_rate_filter_hz_ = declare_parameter<double>("yaw.rate_filter_hz", 15.0);
    yaw_acceleration_filter_hz_ = declare_parameter<double>(
      "yaw.acceleration_filter_hz", 8.0);
    yaw_differential_torque_limit_nm_ = declare_parameter<double>(
      "yaw.differential_torque_limit_nm", 0.025);
    yaw_differential_slew_rate_nm_s_ = declare_parameter<double>(
      "yaw.differential_slew_rate_nm_s", 0.50);

    wheel_radius_m_ = declare_parameter<double>("model.wheel_radius_m", 0.030);

    // Retain direct-LQR manual fallback parameter names used by the repository.
    lqr_use_manual_gain_ = declare_parameter<bool>("lqr.use_manual_gain", true);
    lqr_manual_k_pitch_ = declare_parameter<double>("lqr.manual_k_pitch", -2.0);
    lqr_manual_k_pitch_rate_ = declare_parameter<double>("lqr.manual_k_pitch_rate", -0.10);
    lqr_manual_k_position_ = declare_parameter<double>("lqr.manual_k_position", -0.55);
    lqr_manual_k_velocity_ = declare_parameter<double>("lqr.manual_k_velocity", -0.13);

    cascade_attitude_k_pitch_ = declare_parameter<double>("cascade.attitude_k_pitch", 0.80);
    cascade_attitude_k_pitch_rate_ = declare_parameter<double>(
      "cascade.attitude_k_pitch_rate", 0.04);
    cascade_position_kp_rad_per_m_ = declare_parameter<double>(
      "cascade.position_kp_rad_per_m", 0.05);
    cascade_velocity_kd_rad_per_mps_ = declare_parameter<double>(
      "cascade.velocity_kd_rad_per_mps", 0.05);
    cascade_position_ki_rad_per_m_s_ = declare_parameter<double>(
      "cascade.position_ki_rad_per_m_s", 0.0);
    cascade_enable_integral_ = declare_parameter<bool>("cascade.enable_integral", false);
    cascade_integral_limit_m_s_ = declare_parameter<double>(
      "cascade.integral_limit_m_s", 0.20);
    cascade_position_to_pitch_sign_ = sign_or_throw(
      declare_parameter<double>("cascade.position_to_pitch_sign", -1.0),
      "cascade.position_to_pitch_sign");
    cascade_pitch_limit_rad_ = declare_parameter<double>(
      "cascade.pitch_limit_deg", 2.0) * kPi / 180.0;
    cascade_pitch_slew_rate_rad_s_ = declare_parameter<double>(
      "cascade.pitch_slew_rate_deg_s", 5.0) * kPi / 180.0;
    cascade_position_error_limit_m_ = declare_parameter<double>(
      "cascade.position_error_limit_m", 0.50);
    cascade_velocity_error_limit_mps_ = declare_parameter<double>(
      "cascade.velocity_error_limit_mps", 0.80);
    cascade_outer_velocity_filter_hz_ = declare_parameter<double>(
      "cascade.outer_velocity_filter_hz", 5.0);
    cascade_arm_bumpless_ = declare_parameter<bool>("cascade.arm_bumpless", true);

    manual_trim_rad_ = declare_parameter<double>("cascade.pitch_trim_deg", 0.0) * kPi / 180.0;
    auto_trim_enable_ = declare_parameter<bool>("cascade.auto_trim.enable", true);
    auto_trim_gain_rad_per_m_s_ = declare_parameter<double>(
      "cascade.auto_trim.gain_rad_per_m_s", 0.020);
    auto_trim_limit_rad_ = declare_parameter<double>(
      "cascade.auto_trim.limit_deg", 4.0) * kPi / 180.0;
    auto_trim_position_deadband_m_ = declare_parameter<double>(
      "cascade.auto_trim.position_deadband_m", 0.005);
    auto_trim_velocity_limit_mps_ = declare_parameter<double>(
      "cascade.auto_trim.velocity_limit_mps", 0.025);
    auto_trim_pitch_rate_limit_rad_s_ = declare_parameter<double>(
      "cascade.auto_trim.pitch_rate_limit_rad_s", 0.10);
    auto_trim_safe_pitch_rad_ = declare_parameter<double>(
      "cascade.auto_trim.safe_pitch_deg", 6.0) * kPi / 180.0;
    auto_trim_max_position_error_m_ = declare_parameter<double>(
      "cascade.auto_trim.max_position_error_m", 0.15);
    auto_trim_dwell_s_ = declare_parameter<double>("cascade.auto_trim.dwell_s", 0.50);
    auto_trim_max_rate_rad_s_ = declare_parameter<double>(
      "cascade.auto_trim.max_rate_deg_s", 0.10) * kPi / 180.0;
    auto_trim_reset_on_calibrate_ = declare_parameter<bool>(
      "cascade.auto_trim.reset_on_calibrate", true);
    (void)declare_parameter<bool>("cascade.auto_trim.reset", false);

    motor_position_wrap_half_range_ = declare_parameter<double>(
      "motor_position_wrap_half_range", kPi);
    pitch_filter_hz_ = declare_parameter<double>("pitch_filter_hz", 40.0);
    pitch_rate_filter_hz_ = declare_parameter<double>("pitch_rate_filter_hz", 25.0);
    wheel_velocity_filter_hz_ = declare_parameter<double>("wheel_velocity_filter_hz", 30.0);
    position_velocity_filter_hz_ = declare_parameter<double>(
      "position_velocity_filter_hz", 12.0);
    velocity_blend_ = declare_parameter<double>("velocity_blend", 0.6);
    velocity_mismatch_warn_mps_ = declare_parameter<double>(
      "velocity_mismatch_warn_mps", 0.50);
    auto_calibrate_on_first_valid_data_ = declare_parameter<bool>(
      "auto_calibrate_on_first_valid_data", false);

    stiction_enable_ = declare_parameter<bool>("stiction.enable", false);
    stiction_velocity_limit_mps_ = declare_parameter<double>(
      "stiction.velocity_limit_mps", 0.015);
    stiction_pitch_rate_limit_rad_s_ = declare_parameter<double>(
      "stiction.pitch_rate_limit_rad_s", 0.08);
    stiction_position_error_m_ = declare_parameter<double>(
      "stiction.position_error_m", 0.008);
    stiction_command_min_total_nm_ = declare_parameter<double>(
      "stiction.command_min_total_nm", 0.002);
    stiction_compensation_each_nm_ = declare_parameter<double>(
      "stiction.compensation_each_nm", 0.005);

    validate_parameters();
    left_unwrapper_.configure(motor_position_wrap_half_range_);
    right_unwrapper_.configure(motor_position_wrap_half_range_);
  }

  void validate_parameters() const
  {
    if (control_mode_ != "cascade" && control_mode_ != "lqr") {
      throw std::runtime_error("control_mode must be cascade or lqr");
    }
    if (!std::isfinite(control_period_s_) || control_period_s_ <= 0.0) {
      throw std::runtime_error("control_period_s must be positive");
    }
    if (per_wheel_torque_limit_ <= 0.0 ||
      per_wheel_torque_limit_ > hard_per_wheel_torque_limit_)
    {
      throw std::runtime_error("torque_limit must be positive and <= hard_torque_limit");
    }
    if (wheel_radius_m_ <= 0.0 || cascade_attitude_k_pitch_ <= 0.0 ||
      cascade_attitude_k_pitch_rate_ < 0.0 || cascade_position_kp_rad_per_m_ < 0.0 ||
      cascade_velocity_kd_rad_per_mps_ < 0.0 || cascade_position_ki_rad_per_m_s_ < 0.0)
    {
      throw std::runtime_error("invalid controller gain or wheel radius");
    }
    if (velocity_blend_ < 0.0 || velocity_blend_ > 1.0 ||
      position_velocity_filter_hz_ < 0.0)
    {
      throw std::runtime_error("velocity_blend must be [0,1] and LPF cutoff non-negative");
    }
    if (auto_trim_gain_rad_per_m_s_ < 0.0 || auto_trim_limit_rad_ < 0.0 ||
      auto_trim_dwell_s_ < 0.0 || auto_trim_max_rate_rad_s_ < 0.0)
    {
      throw std::runtime_error("auto trim parameters must be non-negative");
    }
  }

  void configure_filters()
  {
    pitch_filter_.configure(pitch_filter_hz_, control_period_s_);
    pitch_rate_filter_.configure(pitch_rate_filter_hz_, control_period_s_);
    wheel_velocity_filter_.configure(wheel_velocity_filter_hz_, control_period_s_);
    position_velocity_filter_.configure(position_velocity_filter_hz_, control_period_s_);
    outer_velocity_filter_.configure(cascade_outer_velocity_filter_hz_, control_period_s_);
    yaw_rate_filter_.configure(yaw_rate_filter_hz_, control_period_s_);
    yaw_acceleration_filter_.configure(yaw_acceleration_filter_hz_, control_period_s_);
  }

  void copy_motor_message(
    const custom_msgs::msg::ReadDmMotor & msg, MotorSample & sample)
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
    ++sample.sequence;
    sample.received = true;
  }

  bool is_fresh(
    const bool received, const rclcpp::Time & stamp, const double timeout_s,
    const rclcpp::Time & current_time) const
  {
    return received && (current_time - stamp).seconds() <= timeout_s;
  }

  bool all_input_data_valid(const rclcpp::Time & current_time) const
  {
    return is_fresh(imu_.received, imu_.received_time, imu_timeout_s_, current_time) &&
           is_fresh(rc_.received, rc_.received_time, rc_timeout_s_, current_time) &&
           is_fresh(left_motor_.received, left_motor_.received_time, motor_timeout_s_, current_time) &&
           is_fresh(right_motor_.received, right_motor_.received_time, motor_timeout_s_, current_time) &&
           rc_.online && left_motor_.online && right_motor_.online &&
           !left_motor_.has_fault() && !right_motor_.has_fault();
  }

  bool calculate_raw_pitch(double & pitch) const
  {
    if (!imu_zero_valid_) {return false;}
    Quaternion current = imu_.orientation;
    if (!normalize_quaternion(current)) {return false;}
    const Quaternion relative = relative_quaternion(imu_zero_, current);
    pitch = imu_angle_sign_ * quaternion_roll(relative);
    return std::isfinite(pitch);
  }

  void reset_controller_states()
  {
    target_position_m_ = 0.0;
    target_velocity_mps_ = 0.0;
    target_yaw_rate_rad_s_ = 0.0;
    commanded_target_velocity_mps_ = 0.0;
    commanded_target_yaw_rate_rad_s_ = 0.0;
    last_position_m_ = 0.0;
    position_fd_initialized_ = false;
    encoder_fd_time_initialized_ = false;
    control_time_initialized_ = false;
    cascade_pitch_setpoint_rad_ = 0.0;
    cascade_position_integral_m_s_ = 0.0;
    auto_trim_stationary_time_s_ = 0.0;
    auto_trim_learning_flag_ = false;
    yaw_rate_initialized_ = false;
    last_yaw_rate_filtered_rad_s_ = 0.0;
    yaw_differential_command_nm_ = 0.0;
    velocity_from_position_raw_mps_ = 0.0;
    velocity_from_position_filtered_mps_ = 0.0;
  }

  void calibrate_zero()
  {
    Quaternion current = imu_.orientation;
    if (!normalize_quaternion(current)) {
      RCLCPP_ERROR(get_logger(), "Calibration rejected: invalid IMU quaternion");
      return;
    }
    // 当前四元数对应的未归零IMU角度。
    // 你的机器人将IMU roll轴作为车身pitch使用。
    const double stored_abs_pitch_rad =
      imu_angle_sign_ * quaternion_roll(current);

    imu_zero_ = current;
    imu_zero_valid_ = true;
    left_unwrapper_.reset(left_motor_.position);
    right_unwrapper_.reset(right_motor_.position);
    pitch_filter_.reset(0.0);
    pitch_rate_filter_.reset(0.0);
    wheel_velocity_filter_.reset(0.0);
    position_velocity_filter_.reset(0.0);
    outer_velocity_filter_.reset(0.0);
    yaw_rate_filter_.reset(0.0);
    yaw_acceleration_filter_.reset(0.0);
    reset_controller_states();
    if (auto_trim_reset_on_calibrate_) {
      auto_trim_rad_ = 0.0;
    }
    calibrated_ = true;
    armed_ = false;
    arm_transition_required_ = true;
    RCLCPP_INFO(
      get_logger(),
      "IMU ZERO STORED: switch=%u abs_pitch=%+.3fdeg "
      "gyro_x=%+.4frad/s "
      "q_zero=[w=%.8f x=%.8f y=%.8f z=%.8f] "
      "auto_trim=%+.3fdeg",
      static_cast<unsigned int>(rc_.right_switch),
      stored_abs_pitch_rad * 180.0 / kPi,
      imu_rate_sign_ * imu_.gyro_x,
      imu_zero_.w,
      imu_zero_.x,
      imu_zero_.y,
      imu_zero_.z,
      auto_trim_rad_ * 180.0 / kPi);
  }

  void disarm(const char * reason, const bool require_switch_cycle)
  {
    if (armed_) {RCLCPP_WARN(get_logger(), "controller disarmed: %s", reason);}
    armed_ = false;
    auto_trim_stationary_time_s_ = 0.0;
    auto_trim_learning_flag_ = false;
    cascade_position_integral_m_s_ = 0.0;
    yaw_differential_command_nm_ = 0.0;
    if (require_switch_cycle) {arm_transition_required_ = true;}
    publish_motor_commands(false, 0.0, 0.0);
  }

  void attempt_arm()
  {
    double pitch = 0.0;
    if (!calibrated_ || !calculate_raw_pitch(pitch)) {
      RCLCPP_WARN(get_logger(), "Arm rejected: calibrate zero first");
      arm_transition_required_ = true;
      return;
    }
    const double pitch_rate_raw = imu_rate_sign_ * imu_.gyro_x;
    const double yaw_rate_raw = yaw_imu_rate_sign_ * imu_.gyro_z;

    Quaternion arm_current_orientation = imu_.orientation;
    if (!normalize_quaternion(arm_current_orientation)) {
      RCLCPP_WARN(get_logger(), "Arm rejected: invalid current IMU quaternion");
      arm_transition_required_ = true;
      return;
    }

    const double stored_abs_pitch_rad =
      imu_angle_sign_ * quaternion_roll(imu_zero_);
    const double current_abs_pitch_rad =
      imu_angle_sign_ * quaternion_roll(arm_current_orientation);

    RCLCPP_INFO(
      get_logger(),
      "ARM IMU CHECK: stored_abs=%+.3fdeg current_abs=%+.3fdeg "
      "relative=%+.3fdeg rate=%+.4frad/s abs_difference=%+.3fdeg",
      stored_abs_pitch_rad * 180.0 / kPi,
      current_abs_pitch_rad * 180.0 / kPi,
      pitch * 180.0 / kPi,
      pitch_rate_raw,
      (current_abs_pitch_rad - stored_abs_pitch_rad) * 180.0 / kPi);

    RCLCPP_INFO(
      get_logger(),
      "ARM IMU QUAT: q_zero=[%.8f %.8f %.8f %.8f] "
      "q_now=[%.8f %.8f %.8f %.8f]",
      imu_zero_.w,
      imu_zero_.x,
      imu_zero_.y,
      imu_zero_.z,
      arm_current_orientation.w,
      arm_current_orientation.x,
      arm_current_orientation.y,
      arm_current_orientation.z);

    if (std::abs(pitch) > arm_max_tilt_rad_ ||
      std::abs(pitch_rate_raw) > arm_max_pitch_rate_rad_s_)
    {
      RCLCPP_WARN(
        get_logger(), "Arm rejected: pitch=%+.2fdeg rate=%+.3frad/s",
        pitch * 180.0 / kPi, pitch_rate_raw);
      arm_transition_required_ = true;
      return;
    }

    const double left_angle = left_encoder_sign_ * left_unwrapper_.update(left_motor_.position);
    const double right_angle = right_encoder_sign_ * right_unwrapper_.update(right_motor_.position);
    const double mean_relative_angle = 0.5 * (left_angle + right_angle);
    const double mean_relative_rate_raw = 0.5 * (
      left_encoder_sign_ * left_motor_.velocity +
      right_encoder_sign_ * right_motor_.velocity);
    target_position_m_ = wheel_radius_m_ * (
      mean_relative_angle + pitch_position_compensation_sign_ * pitch);

    pitch_filter_.reset(pitch);
    pitch_rate_filter_.reset(pitch_rate_raw);
    wheel_velocity_filter_.reset(mean_relative_rate_raw);
    const double initial_velocity_mps = wheel_radius_m_ * (
      mean_relative_rate_raw + pitch_rate_compensation_sign_ * pitch_rate_raw);
    position_velocity_filter_.reset(initial_velocity_mps);
    outer_velocity_filter_.reset(0.0);
    yaw_rate_filter_.reset(yaw_rate_raw);
    yaw_acceleration_filter_.reset(0.0);

    last_position_m_ = target_position_m_;
    velocity_from_position_raw_mps_ = initial_velocity_mps;
    velocity_from_position_filtered_mps_ = initial_velocity_mps;
    last_left_fd_sequence_ = left_motor_.sequence;
    last_right_fd_sequence_ = right_motor_.sequence;
    last_encoder_fd_time_ = std::chrono::steady_clock::now();
    position_fd_initialized_ = true;
    encoder_fd_time_initialized_ = true;

    const double total_trim = manual_trim_rad_ + auto_trim_rad_;
    cascade_pitch_setpoint_rad_ = cascade_arm_bumpless_ ?
      clamp_value(pitch - total_trim, -cascade_pitch_limit_rad_, cascade_pitch_limit_rad_) : 0.0;
    cascade_position_integral_m_s_ = 0.0;
    auto_trim_stationary_time_s_ = 0.0;
    auto_trim_learning_flag_ = false;
    target_velocity_mps_ = 0.0;
    target_yaw_rate_rad_s_ = 0.0;
    commanded_target_velocity_mps_ = 0.0;
    commanded_target_yaw_rate_rad_s_ = 0.0;
    yaw_rate_initialized_ = true;
    last_yaw_rate_filtered_rad_s_ = yaw_rate_raw;
    yaw_differential_command_nm_ = 0.0;
    control_time_initialized_ = false;
    armed_ = true;
    arm_transition_required_ = false;

    RCLCPP_INFO(
      get_logger(),
      "%s armed; x0=%.5fm pitch=%+.3fdeg trim=%+.3fdeg correction0=%+.3fdeg",
      control_mode_.c_str(), target_position_m_, pitch * 180.0 / kPi,
      total_trim * 180.0 / kPi, cascade_pitch_setpoint_rad_ * 180.0 / kPi);
  }

  double process_rc_velocity_command(const double dt, DebugSample & debug)
  {
    const double raw = clamp_value(rc_.right_y, -1.0, 1.0);
    const double shaped = rc_forward_sign_ * shape_unit_stick(raw, rc_deadband_);
    const double requested = enable_velocity_command_ ? shaped * max_target_velocity_ : 0.0;
    const double max_step = rc_velocity_slew_rate_mps2_ * dt;
    commanded_target_velocity_mps_ += clamp_value(
      requested - commanded_target_velocity_mps_, -max_step, max_step);
    debug.rc_right_y_raw = raw;
    debug.rc_forward_shaped = shaped;
    debug.rc_velocity_command = commanded_target_velocity_mps_;
    debug.rc_velocity_slew_limited_flag =
      std::abs(requested - commanded_target_velocity_mps_) > 1.0e-9 ? 1.0 : 0.0;
    return commanded_target_velocity_mps_;
  }

  double process_rc_yaw_rate_command(const double dt, DebugSample & debug)
  {
    const double raw = clamp_value(rc_.left_x, -1.0, 1.0);
    const double shaped = rc_yaw_sign_ * shape_unit_stick(raw, rc_deadband_);
    const double requested = enable_yaw_rate_command_ ? shaped * max_target_yaw_rate_rad_s_ : 0.0;
    const double max_step = rc_yaw_rate_slew_rate_rad_s2_ * dt;
    commanded_target_yaw_rate_rad_s_ += clamp_value(
      requested - commanded_target_yaw_rate_rad_s_, -max_step, max_step);
    debug.rc_left_x_raw = raw;
    debug.rc_yaw_shaped = shaped;
    debug.rc_yaw_rate_command = commanded_target_yaw_rate_rad_s_;
    debug.rc_yaw_slew_limited_flag =
      std::abs(requested - commanded_target_yaw_rate_rad_s_) > 1.0e-9 ? 1.0 : 0.0;
    return commanded_target_yaw_rate_rad_s_;
  }

  double calculate_cascade_torque(
    const double pitch, const double pitch_rate, const double position_error,
    const double velocity_error, const double dt, DebugSample & debug)
  {
    const double outer_velocity = outer_velocity_filter_.update(velocity_error, dt);
    const double limited_position_error = clamp_value(
      position_error, -cascade_position_error_limit_m_, cascade_position_error_limit_m_);
    const double limited_velocity_error = clamp_value(
      outer_velocity, -cascade_velocity_error_limit_mps_, cascade_velocity_error_limit_mps_);

    if (cascade_enable_integral_) {
      cascade_position_integral_m_s_ = clamp_value(
        cascade_position_integral_m_s_ + limited_position_error * dt,
        -cascade_integral_limit_m_s_, cascade_integral_limit_m_s_);
    } else {
      cascade_position_integral_m_s_ = 0.0;
    }

    const double outer_command =
      cascade_position_kp_rad_per_m_ * limited_position_error +
      cascade_velocity_kd_rad_per_mps_ * limited_velocity_error +
      cascade_position_ki_rad_per_m_s_ * cascade_position_integral_m_s_;
    const double pitch_setpoint_raw = cascade_position_to_pitch_sign_ * outer_command;
    const double pitch_setpoint_limited = clamp_value(
      pitch_setpoint_raw, -cascade_pitch_limit_rad_, cascade_pitch_limit_rad_);
    const bool outer_saturated =
      std::abs(pitch_setpoint_raw - pitch_setpoint_limited) > 1.0e-12;

    const double max_step = cascade_pitch_slew_rate_rad_s_ * dt;
    cascade_pitch_setpoint_rad_ += clamp_value(
      pitch_setpoint_limited - cascade_pitch_setpoint_rad_, -max_step, max_step);

    if (cascade_enable_integral_ && outer_saturated) {
      cascade_position_integral_m_s_ = clamp_value(
        cascade_position_integral_m_s_ - limited_position_error * dt,
        -cascade_integral_limit_m_s_, cascade_integral_limit_m_s_);
    }

    const double total_trim = manual_trim_rad_ + auto_trim_rad_;
    const double pitch_reference = total_trim + cascade_pitch_setpoint_rad_;
    const double attitude_error = pitch - pitch_reference;
    const double u_pitch_rate = cascade_attitude_k_pitch_rate_ * pitch_rate;
    const double u_total = cascade_attitude_k_pitch_ * attitude_error + u_pitch_rate;

    debug.u_pitch = cascade_attitude_k_pitch_ * pitch;
    debug.u_pitch_rate = u_pitch_rate;
    debug.u_x = -cascade_attitude_k_pitch_ * cascade_position_to_pitch_sign_ *
      cascade_position_kp_rad_per_m_ * limited_position_error;
    debug.u_x_dot = -cascade_attitude_k_pitch_ * cascade_position_to_pitch_sign_ *
      cascade_velocity_kd_rad_per_mps_ * limited_velocity_error;
    debug.pitch_setpoint_raw = pitch_setpoint_raw;
    debug.pitch_setpoint_limited = pitch_setpoint_limited;
    debug.pitch_setpoint_command = pitch_reference;
    debug.outer_velocity_filtered = outer_velocity;
    debug.position_integral = cascade_position_integral_m_s_;
    debug.attitude_error = attitude_error;
    debug.outer_saturation_flag = outer_saturated ? 1.0 : 0.0;
    debug.manual_trim = manual_trim_rad_;
    debug.auto_trim = auto_trim_rad_;
    debug.total_trim = total_trim;
    debug.full_pitch_reference = pitch_reference;
    return u_total;
  }

  double calculate_lqr_torque(
    const double pitch, const double pitch_rate,
    const double position_error, const double velocity_error,
    DebugSample & debug) const
  {
    if (!lqr_use_manual_gain_) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "This replacement package supports the repository manual LQR fallback only");
    }
    const double raw = -lqr_gain_scale_ * (
      lqr_manual_k_pitch_ * pitch +
      lqr_manual_k_pitch_rate_ * pitch_rate +
      lqr_manual_k_position_ * position_error +
      lqr_manual_k_velocity_ * velocity_error);
    debug.u_pitch = -lqr_gain_scale_ * lqr_manual_k_pitch_ * pitch;
    debug.u_pitch_rate = -lqr_gain_scale_ * lqr_manual_k_pitch_rate_ * pitch_rate;
    debug.u_x = -lqr_gain_scale_ * lqr_manual_k_position_ * position_error;
    debug.u_x_dot = -lqr_gain_scale_ * lqr_manual_k_velocity_ * velocity_error;
    debug.pitch_setpoint_command = manual_trim_rad_ + auto_trim_rad_;
    debug.manual_trim = manual_trim_rad_;
    debug.auto_trim = auto_trim_rad_;
    debug.total_trim = manual_trim_rad_ + auto_trim_rad_;
    debug.full_pitch_reference = debug.total_trim;
    return raw;
  }

  double calculate_yaw_differential_torque(
    const double yaw_rate_raw, const double target_yaw_rate,
    const double common_torque, const double dt, DebugSample & debug)
  {
    const double yaw_rate_filtered = yaw_rate_filter_.update(yaw_rate_raw, dt);
    double yaw_acceleration_raw = 0.0;
    if (yaw_rate_initialized_) {
      yaw_acceleration_raw =
        (yaw_rate_filtered - last_yaw_rate_filtered_rad_s_) / std::max(dt, 1.0e-6);
    }
    last_yaw_rate_filtered_rad_s_ = yaw_rate_filtered;
    yaw_rate_initialized_ = true;
    const double yaw_acceleration_filtered =
      yaw_acceleration_filter_.update(yaw_acceleration_raw, dt);

    const bool command_active = std::abs(target_yaw_rate) > 1.0e-12;
    const double rate_for_control = command_active ? yaw_rate_filtered :
      apply_continuous_deadband(yaw_rate_filtered, yaw_rate_deadband_rad_s_);
    const double error = target_yaw_rate - rate_for_control;
    const double raw = yaw_enable_ ? yaw_output_sign_ * (
      yaw_rate_kp_per_wheel_nm_per_rad_s_ * error -
      yaw_rate_kd_per_wheel_nm_per_rad_s2_ * yaw_acceleration_filtered) : 0.0;

    const double headroom = std::max(0.0, per_wheel_torque_limit_ - std::abs(common_torque));
    const double allowed = std::min(yaw_differential_torque_limit_nm_, headroom);
    const double target = clamp_value(raw, -allowed, allowed);
    if (!yaw_enable_ || (!command_active && std::abs(rate_for_control) <= 1.0e-12)) {
      yaw_differential_command_nm_ = 0.0;
    } else {
      const double max_step = yaw_differential_slew_rate_nm_s_ * dt;
      yaw_differential_command_nm_ += clamp_value(
        target - yaw_differential_command_nm_, -max_step, max_step);
      yaw_differential_command_nm_ = clamp_value(
        yaw_differential_command_nm_, -allowed, allowed);
    }

    debug.yaw_enabled_flag = yaw_enable_ ? 1.0 : 0.0;
    debug.yaw_rate_raw = yaw_rate_raw;
    debug.yaw_rate_filtered = yaw_rate_filtered;
    debug.yaw_rate_for_control = rate_for_control;
    debug.yaw_rate_error = error;
    debug.yaw_acceleration_filtered = yaw_acceleration_filtered;
    debug.yaw_differential_raw = raw;
    debug.yaw_differential_command = yaw_differential_command_nm_;
    debug.yaw_available_headroom = headroom;
    debug.yaw_limited_flag =
      std::abs(raw - yaw_differential_command_nm_) > 1.0e-9 ? 1.0 : 0.0;
    return yaw_differential_command_nm_;
  }

  void update_auto_trim(
    const double position_error, const double velocity_mps,
    const double pitch, const double pitch_rate, const double target_velocity,
    const bool outer_saturated, const bool torque_saturated, const double dt)
  {
    auto_trim_learning_flag_ = false;
    if (!auto_trim_enable_ || control_mode_ != "cascade") {
      auto_trim_stationary_time_s_ = 0.0;
      return;
    }

    const bool safe_to_learn =
      std::abs(target_velocity) < 0.005 &&
      std::abs(commanded_target_yaw_rate_rad_s_) < 0.01 &&
      std::abs(velocity_mps) < auto_trim_velocity_limit_mps_ &&
      std::abs(pitch_rate) < auto_trim_pitch_rate_limit_rad_s_ &&
      std::abs(pitch) < auto_trim_safe_pitch_rad_ &&
      std::abs(position_error) < auto_trim_max_position_error_m_ &&
      !outer_saturated && !torque_saturated;

    if (!safe_to_learn) {
      auto_trim_stationary_time_s_ = 0.0;
      return;
    }

    auto_trim_stationary_time_s_ += dt;
    if (auto_trim_stationary_time_s_ < auto_trim_dwell_s_) {return;}

    const double effective_error = apply_continuous_deadband(
      position_error, auto_trim_position_deadband_m_);
    if (std::abs(effective_error) <= 1.0e-12) {return;}

    double trim_rate = cascade_position_to_pitch_sign_ *
      auto_trim_gain_rad_per_m_s_ * effective_error;
    trim_rate = clamp_value(trim_rate, -auto_trim_max_rate_rad_s_, auto_trim_max_rate_rad_s_);
    auto_trim_rad_ = clamp_value(
      auto_trim_rad_ + trim_rate * dt, -auto_trim_limit_rad_, auto_trim_limit_rad_);
    auto_trim_learning_flag_ = true;
  }

  double apply_stiction_compensation(
    const double signed_total_torque, const double position_error,
    const double velocity, const double pitch_rate, const double target_velocity,
    DebugSample & debug) const
  {
    if (!stiction_enable_ || std::abs(target_velocity) > 0.005 ||
      std::abs(velocity) > stiction_velocity_limit_mps_ ||
      std::abs(pitch_rate) > stiction_pitch_rate_limit_rad_s_ ||
      std::abs(position_error) < stiction_position_error_m_ ||
      std::abs(signed_total_torque) < stiction_command_min_total_nm_)
    {
      debug.stiction_compensation_total = 0.0;
      return signed_total_torque;
    }
    const double compensation = std::copysign(
      2.0 * stiction_compensation_each_nm_, signed_total_torque);
    debug.stiction_compensation_total = compensation;
    return signed_total_torque + compensation;
  }

  void control_step()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    double actual_dt = control_period_s_;
    if (control_time_initialized_) {
      actual_dt = std::chrono::duration<double>(
        steady_now - last_control_steady_time_).count();
    }
    last_control_steady_time_ = steady_now;
    control_time_initialized_ = true;
    if (!std::isfinite(actual_dt) || actual_dt <= 0.0) {actual_dt = control_period_s_;}
    actual_dt = clamp_value(actual_dt, 0.0005, 0.020);

    std::lock_guard<std::mutex> lock(data_mutex_);
    const rclcpp::Time current_time = now();
    DebugSample debug;
    debug.actual_dt = actual_dt;
    debug.cascade_mode_flag = control_mode_ == "cascade" ? 1.0 : 0.0;

    if (!all_input_data_valid(current_time)) {
      disarm("input timeout, RC offline, motor offline, or motor fault", true);
      last_switch_value_ = 255;
      publish_debug(debug);
      return;
    }

    debug.imu_age_s = std::max(0.0, (current_time - imu_.received_time).seconds());
    debug.left_motor_age_s = std::max(0.0, (current_time - left_motor_.received_time).seconds());
    debug.right_motor_age_s = std::max(0.0, (current_time - right_motor_.received_time).seconds());
    debug.left_position_raw = left_motor_.position;
    debug.right_position_raw = right_motor_.position;
    debug.left_velocity_raw = left_motor_.velocity;
    debug.right_velocity_raw = right_motor_.velocity;

    const int switch_value = static_cast<int>(rc_.right_switch);
    const bool switch_changed = switch_value != last_switch_value_;

    if (switch_changed) {
      RCLCPP_INFO(
        get_logger(),
        "RC RIGHT SWITCH: %d -> %d",
        last_switch_value_,
        switch_value);
    }

    if (auto_calibrate_on_first_valid_data_ && !calibrated_) {calibrate_zero();}
    if (switch_changed && switch_value == calibrate_switch_value_) {calibrate_zero();}
    if (switch_value == disable_switch_value_) {
      disarm("disable switch", false);
      arm_transition_required_ = false;
    } else if (switch_changed && switch_value == arm_switch_value_) {
      if (!arm_transition_required_ || switch_changed) {attempt_arm();}
    }
    last_switch_value_ = switch_value;

    if (!armed_) {
      publish_motor_commands(false, 0.0, 0.0);

      if (imu_zero_valid_) {
        Quaternion current_orientation = imu_.orientation;
        double relative_pitch_rad = 0.0;

        if (normalize_quaternion(current_orientation) &&
          calculate_raw_pitch(relative_pitch_rad))
        {
          const double stored_abs_pitch_rad =
            imu_angle_sign_ * quaternion_roll(imu_zero_);
          const double current_abs_pitch_rad =
            imu_angle_sign_ * quaternion_roll(current_orientation);
          const double pitch_rate_raw =
            imu_rate_sign_ * imu_.gyro_x;

          RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            200,
            "IMU HOLD: switch=%d stored_abs=%+.3fdeg "
            "current_abs=%+.3fdeg relative=%+.3fdeg "
            "rate=%+.4frad/s auto_trim=%+.3fdeg",
            switch_value,
            stored_abs_pitch_rad * 180.0 / kPi,
            current_abs_pitch_rad * 180.0 / kPi,
            relative_pitch_rad * 180.0 / kPi,
            pitch_rate_raw,
            auto_trim_rad_ * 180.0 / kPi);
        }
      }

      debug.manual_trim = manual_trim_rad_;
      debug.auto_trim = auto_trim_rad_;
      debug.total_trim = manual_trim_rad_ + auto_trim_rad_;
      publish_debug(debug);
      return;
    }

    double pitch_raw = 0.0;
    if (!calculate_raw_pitch(pitch_raw)) {
      disarm("invalid pitch quaternion", true);
      publish_debug(debug);
      return;
    }
    const double pitch_rate_raw = imu_rate_sign_ * imu_.gyro_x;
    const double yaw_rate_raw = yaw_imu_rate_sign_ * imu_.gyro_z;
    debug.pitch_raw = pitch_raw;
    debug.pitch_rate_raw = pitch_rate_raw;
    if (std::abs(pitch_raw) > fall_cutoff_rad_) {
      disarm("raw fall angle exceeded", true);
      publish_debug(debug);
      return;
    }

    const double pitch = pitch_filter_.update(pitch_raw, actual_dt);
    const double pitch_rate = pitch_rate_filter_.update(pitch_rate_raw, actual_dt);
    debug.pitch = pitch;
    debug.pitch_rate = pitch_rate;
    if (std::abs(pitch) > fall_cutoff_rad_) {
      disarm("filtered fall angle exceeded", true);
      publish_debug(debug);
      return;
    }

    const double left_angle = left_encoder_sign_ * left_unwrapper_.update(left_motor_.position);
    const double right_angle = right_encoder_sign_ * right_unwrapper_.update(right_motor_.position);
    const double mean_relative_angle = 0.5 * (left_angle + right_angle);
    const double mean_relative_rate_raw = 0.5 * (
      left_encoder_sign_ * left_motor_.velocity +
      right_encoder_sign_ * right_motor_.velocity);
    const double mean_relative_rate = wheel_velocity_filter_.update(
      mean_relative_rate_raw, actual_dt);

    const double position_m = wheel_radius_m_ * (
      mean_relative_angle + pitch_position_compensation_sign_ * pitch);
    const double velocity_motor_based = wheel_radius_m_ * (
      mean_relative_rate + pitch_rate_compensation_sign_ * pitch_rate);

    const bool new_encoder_pair =
      left_motor_.sequence != last_left_fd_sequence_ &&
      right_motor_.sequence != last_right_fd_sequence_;
    debug.encoder_pair_updated_flag = new_encoder_pair ? 1.0 : 0.0;
    if (new_encoder_pair) {
      const auto encoder_now = steady_now;
      if (position_fd_initialized_ && encoder_fd_time_initialized_) {
        const double encoder_dt = std::chrono::duration<double>(
          encoder_now - last_encoder_fd_time_).count();
        if (std::isfinite(encoder_dt) && encoder_dt > 1.0e-5 && encoder_dt < 0.10) {
          velocity_from_position_raw_mps_ =
            (position_m - last_position_m_) / encoder_dt;
          velocity_from_position_filtered_mps_ = position_velocity_filter_.update(
            velocity_from_position_raw_mps_, encoder_dt);
        }
      } else {
        velocity_from_position_raw_mps_ = velocity_motor_based;
        position_velocity_filter_.reset(velocity_motor_based);
        velocity_from_position_filtered_mps_ = velocity_motor_based;
        position_fd_initialized_ = true;
        encoder_fd_time_initialized_ = true;
      }
      last_position_m_ = position_m;
      last_encoder_fd_time_ = encoder_now;
      last_left_fd_sequence_ = left_motor_.sequence;
      last_right_fd_sequence_ = right_motor_.sequence;
    }

    const double velocity_mps =
      velocity_blend_ * velocity_motor_based +
      (1.0 - velocity_blend_) * velocity_from_position_filtered_mps_;

    target_velocity_mps_ = process_rc_velocity_command(actual_dt, debug);
    target_yaw_rate_rad_s_ = process_rc_yaw_rate_command(actual_dt, debug);
    target_position_m_ += target_velocity_mps_ * actual_dt;
    const double position_error = position_m - target_position_m_;
    const double velocity_error = velocity_mps - target_velocity_mps_;

    double u_total_unsaturated = 0.0;
    if (control_mode_ == "cascade") {
      u_total_unsaturated = calculate_cascade_torque(
        pitch, pitch_rate, position_error, velocity_error, actual_dt, debug);
    } else {
      u_total_unsaturated = calculate_lqr_torque(
        pitch, pitch_rate, position_error, velocity_error, debug);
    }

    if (control_mode_ == "cascade") {
      Quaternion current_orientation = imu_.orientation;

      if (normalize_quaternion(current_orientation)) {
        const double stored_abs_pitch_rad =
          imu_angle_sign_ * quaternion_roll(imu_zero_);
        const double current_abs_pitch_rad =
          imu_angle_sign_ * quaternion_roll(current_orientation);
        const double total_trim =
          manual_trim_rad_ + auto_trim_rad_;

        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          200,
          "IMU LIVE: stored_abs=%+.3fdeg current_abs=%+.3fdeg "
          "relative_raw=%+.3fdeg relative_filt=%+.3fdeg "
          "rate=%+.4frad/s trim=%+.3fdeg correction=%+.3fdeg "
          "reference=%+.3fdeg ex=%+.4fm v=%+.4fm/s",
          stored_abs_pitch_rad * 180.0 / kPi,
          current_abs_pitch_rad * 180.0 / kPi,
          pitch_raw * 180.0 / kPi,
          pitch * 180.0 / kPi,
          pitch_rate,
          total_trim * 180.0 / kPi,
          cascade_pitch_setpoint_rad_ * 180.0 / kPi,
          debug.full_pitch_reference * 180.0 / kPi,
          position_error,
          velocity_mps);
      }
    }

    const double max_total_torque = 2.0 * per_wheel_torque_limit_;
    double signed_total_torque = output_gain_sign_ * u_total_unsaturated;
    signed_total_torque = apply_stiction_compensation(
      signed_total_torque, position_error, velocity_mps, pitch_rate,
      target_velocity_mps_, debug);
    const double total_torque_after_limit = clamp_value(
      signed_total_torque, -max_total_torque, max_total_torque);
    const bool saturated =
      std::abs(signed_total_torque - total_torque_after_limit) > 1.0e-9;
    const double common_torque = 0.5 * total_torque_after_limit;
    const double yaw_diff = calculate_yaw_differential_torque(
      yaw_rate_raw, target_yaw_rate_rad_s_, common_torque, actual_dt, debug);

    const double left_physical = clamp_value(
      common_torque + yaw_diff, -per_wheel_torque_limit_, per_wheel_torque_limit_);
    const double right_physical = clamp_value(
      common_torque - yaw_diff, -per_wheel_torque_limit_, per_wheel_torque_limit_);
    const double left_command = left_motor_sign_ * left_physical;
    const double right_command = right_motor_sign_ * right_physical;

    if (dry_run_) {
      publish_motor_commands(false, 0.0, 0.0);
    } else {
      publish_motor_commands(true, left_command, right_command);
    }

    update_auto_trim(
      position_error, velocity_mps, pitch, pitch_rate, target_velocity_mps_,
      debug.outer_saturation_flag > 0.5, saturated, actual_dt);

    debug.position = position_m;
    debug.velocity = velocity_mps;
    debug.target_position = target_position_m_;
    debug.target_velocity = target_velocity_mps_;
    debug.model_total_unsaturated = u_total_unsaturated;
    debug.per_wheel_common_torque = common_torque;
    debug.total_torque_after_limit = total_torque_after_limit;
    debug.saturation_flag = saturated ? 1.0 : 0.0;
    debug.velocity_from_position = velocity_from_position_filtered_mps_;
    debug.velocity_from_position_raw = velocity_from_position_raw_mps_;
    debug.velocity_from_position_filtered = velocity_from_position_filtered_mps_;
    debug.position_error = position_error;
    debug.velocity_error = velocity_error;
    debug.velocity_motor_based = velocity_motor_based;
    debug.velocity_mismatch = velocity_motor_based - velocity_from_position_filtered_mps_;
    debug.left_physical_torque = left_physical;
    debug.right_physical_torque = right_physical;
    debug.left_motor_command = left_command;
    debug.right_motor_command = right_command;
    debug.auto_trim = auto_trim_rad_;
    debug.total_trim = manual_trim_rad_ + auto_trim_rad_;
    debug.auto_trim_learning_flag = auto_trim_learning_flag_ ? 1.0 : 0.0;
    publish_debug(debug);

    if (std::abs(debug.velocity_mismatch) > velocity_mismatch_warn_mps_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "velocity mismatch: motor=%+.3f filtered_position_fd=%+.3f raw_fd=%+.3f",
        velocity_motor_based, velocity_from_position_filtered_mps_,
        velocity_from_position_raw_mps_);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 100,
      "pitch=%+.2fdeg ref=%+.2fdeg rate=%+.3f x=%+.4f ex=%+.4f "
      "v=%+.4f vfd=%+.4f vf=%+.4f trim=%+.2fdeg learn=%d uT=%+.4f common=%+.4f",
      pitch * 180.0 / kPi, debug.full_pitch_reference * 180.0 / kPi,
      pitch_rate, position_m * 100.0, position_error * 100, velocity_mps,
      velocity_from_position_filtered_mps_, debug.outer_velocity_filtered,
      debug.total_trim * 180.0 / kPi, auto_trim_learning_flag_ ? 1 : 0,
      u_total_unsaturated, common_torque);
  }

  void publish_motor_commands(
    const bool enable, const double left_torque, const double right_torque)
  {
    custom_msgs::msg::WriteDmMotorMITControl left_message;
    custom_msgs::msg::WriteDmMotorMITControl right_message;
    left_message.enable = enable ? 1U : 0U;
    left_message.p_des = 0.0F;
    left_message.v_des = 0.0F;
    left_message.kp = 0.0F;
    left_message.kd = 0.0F;
    left_message.torque = static_cast<float>(clamp_value(
      left_torque, -hard_per_wheel_torque_limit_, hard_per_wheel_torque_limit_));
    right_message.enable = enable ? 1U : 0U;
    right_message.p_des = 0.0F;
    right_message.v_des = 0.0F;
    right_message.kp = 0.0F;
    right_message.kd = 0.0F;
    right_message.torque = static_cast<float>(clamp_value(
      right_torque, -hard_per_wheel_torque_limit_, hard_per_wheel_torque_limit_));
    left_motor_pub_->publish(left_message);
    right_motor_pub_->publish(right_message);
  }

  void publish_debug(const DebugSample & s)
  {
    const double equivalent_k0 = -cascade_attitude_k_pitch_;
    const double equivalent_k1 = -cascade_attitude_k_pitch_rate_;
    const double equivalent_k2 = cascade_attitude_k_pitch_ *
      cascade_position_to_pitch_sign_ * cascade_position_kp_rad_per_m_;
    const double equivalent_k3 = cascade_attitude_k_pitch_ *
      cascade_position_to_pitch_sign_ * cascade_velocity_kd_rad_per_mps_;

    std_msgs::msg::Float64MultiArray message;
    message.data = {
      s.position, s.velocity, s.pitch, s.pitch_rate, s.target_position, s.target_velocity,
      s.model_total_unsaturated, s.per_wheel_common_torque, armed_ ? 1.0 : 0.0,
      dry_run_ ? 1.0 : 0.0, output_gain_sign_, per_wheel_torque_limit_,
      s.pitch_raw, s.pitch_rate_raw, s.u_x, s.u_x_dot, s.u_pitch, s.u_pitch_rate,
      s.total_torque_after_limit, s.saturation_flag, s.actual_dt, s.imu_age_s,
      s.left_motor_age_s, s.right_motor_age_s, s.left_position_raw, s.right_position_raw,
      s.left_velocity_raw, s.right_velocity_raw, s.velocity_from_position,
      s.position_error, s.velocity_error, equivalent_k0, equivalent_k1, equivalent_k2,
      equivalent_k3, s.velocity_motor_based, s.velocity_mismatch, 4.0, 0.0,
      control_mode_ == "cascade" ? 1.0 : lqr_gain_scale_, 0.0,
      s.cascade_mode_flag, s.pitch_setpoint_raw, s.pitch_setpoint_limited,
      s.pitch_setpoint_command, s.outer_velocity_filtered, s.position_integral,
      s.attitude_error, s.outer_saturation_flag, cascade_pitch_limit_rad_,
      cascade_pitch_slew_rate_rad_s_, s.yaw_enabled_flag, s.yaw_rate_raw,
      s.yaw_rate_filtered, s.yaw_rate_for_control, s.yaw_rate_error,
      s.yaw_acceleration_filtered, s.yaw_differential_raw, s.yaw_differential_command,
      s.yaw_available_headroom, s.yaw_limited_flag, s.left_physical_torque,
      s.right_physical_torque, s.left_motor_command, s.right_motor_command,
      s.rc_right_y_raw, s.rc_left_x_raw, s.rc_forward_shaped, s.rc_yaw_shaped,
      s.rc_velocity_command, s.rc_yaw_rate_command, s.rc_velocity_slew_limited_flag,
      s.rc_yaw_slew_limited_flag,
      s.velocity_from_position_raw, s.velocity_from_position_filtered,
      s.manual_trim, s.auto_trim, s.total_trim, s.full_pitch_reference,
      s.auto_trim_learning_flag, s.encoder_pair_updated_flag,
      s.stiction_compensation_total
    };
    debug_pub_->publish(message);
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();
      if (name == "dry_run") {
        dry_run_ = parameter.as_bool();
      } else if (name == "cascade.pitch_trim_deg") {
        manual_trim_rad_ = parameter.as_double() * kPi / 180.0;
      } else if (name == "cascade.auto_trim.enable") {
        auto_trim_enable_ = parameter.as_bool();
      } else if (name == "cascade.auto_trim.reset") {
        if (parameter.as_bool()) {auto_trim_rad_ = 0.0;}
      } else {
        result.successful = false;
        result.reason = name + " requires YAML edit and node restart";
        return result;
      }
    }
    armed_ = false;
    arm_transition_required_ = true;
    publish_motor_commands(false, 0.0, 0.0);
    return result;
  }

  std::mutex data_mutex_;
  ImuSample imu_;
  RcSample rc_;
  MotorSample left_motor_;
  MotorSample right_motor_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<custom_msgs::msg::ReadDJIRC>::SharedPtr rc_sub_;
  rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr left_motor_sub_;
  rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr right_motor_sub_;
  rclcpp::Publisher<custom_msgs::msg::WriteDmMotorMITControl>::SharedPtr left_motor_pub_;
  rclcpp::Publisher<custom_msgs::msg::WriteDmMotorMITControl>::SharedPtr right_motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::string imu_topic_, rc_topic_, left_motor_read_topic_, left_motor_write_topic_;
  std::string right_motor_read_topic_, right_motor_write_topic_, debug_topic_;
  std::string control_mode_{"cascade"};
  double control_period_s_{0.003};
  bool dry_run_{true};
  double per_wheel_torque_limit_{0.05};
  double hard_per_wheel_torque_limit_{0.45};
  double lqr_gain_scale_{1.0};
  double arm_max_tilt_rad_{3.0 * kPi / 180.0};
  double arm_max_pitch_rate_rad_s_{0.30};
  double fall_cutoff_rad_{25.0 * kPi / 180.0};
  double imu_timeout_s_{0.05}, motor_timeout_s_{0.05}, rc_timeout_s_{0.20};
  int calibrate_switch_value_{1}, arm_switch_value_{3}, disable_switch_value_{2};
  bool enable_velocity_command_{true}, enable_yaw_rate_command_{true};
  double max_target_velocity_{0.20}, max_target_yaw_rate_rad_s_{0.80};
  double rc_deadband_{0.08}, rc_forward_sign_{1.0}, rc_yaw_sign_{1.0};
  double rc_velocity_slew_rate_mps2_{0.60}, rc_yaw_rate_slew_rate_rad_s2_{3.0};
  double output_gain_sign_{1.0}, left_motor_sign_{-1.0}, right_motor_sign_{1.0};
  double left_encoder_sign_{1.0}, right_encoder_sign_{-1.0};
  double imu_angle_sign_{1.0}, imu_rate_sign_{1.0};
  double pitch_position_compensation_sign_{1.0}, pitch_rate_compensation_sign_{1.0};

  bool yaw_enable_{true};
  double yaw_imu_rate_sign_{1.0}, yaw_output_sign_{1.0};
  double yaw_rate_kp_per_wheel_nm_per_rad_s_{0.015};
  double yaw_rate_kd_per_wheel_nm_per_rad_s2_{0.0};
  double yaw_rate_deadband_rad_s_{0.02}, yaw_rate_filter_hz_{15.0};
  double yaw_acceleration_filter_hz_{8.0};
  double yaw_differential_torque_limit_nm_{0.025};
  double yaw_differential_slew_rate_nm_s_{0.50};

  double wheel_radius_m_{0.030};
  bool lqr_use_manual_gain_{true};
  double lqr_manual_k_pitch_{-2.0}, lqr_manual_k_pitch_rate_{-0.10};
  double lqr_manual_k_position_{-0.55}, lqr_manual_k_velocity_{-0.13};

  double cascade_attitude_k_pitch_{0.80};
  double cascade_attitude_k_pitch_rate_{0.04};
  double cascade_position_kp_rad_per_m_{0.05};
  double cascade_velocity_kd_rad_per_mps_{0.05};
  double cascade_position_ki_rad_per_m_s_{0.0};
  bool cascade_enable_integral_{false};
  double cascade_integral_limit_m_s_{0.20};
  double cascade_position_to_pitch_sign_{-1.0};
  double cascade_pitch_limit_rad_{2.0 * kPi / 180.0};
  double cascade_pitch_slew_rate_rad_s_{5.0 * kPi / 180.0};
  double cascade_position_error_limit_m_{0.50};
  double cascade_velocity_error_limit_mps_{0.80};
  double cascade_outer_velocity_filter_hz_{5.0};
  bool cascade_arm_bumpless_{true};

  double manual_trim_rad_{0.0}, auto_trim_rad_{0.0};
  bool auto_trim_enable_{true}, auto_trim_reset_on_calibrate_{true};
  double auto_trim_gain_rad_per_m_s_{0.020};
  double auto_trim_limit_rad_{4.0 * kPi / 180.0};
  double auto_trim_position_deadband_m_{0.005};
  double auto_trim_velocity_limit_mps_{0.025};
  double auto_trim_pitch_rate_limit_rad_s_{0.10};
  double auto_trim_safe_pitch_rad_{6.0 * kPi / 180.0};
  double auto_trim_max_position_error_m_{0.15};
  double auto_trim_dwell_s_{0.50};
  double auto_trim_max_rate_rad_s_{0.10 * kPi / 180.0};

  double motor_position_wrap_half_range_{kPi};
  double pitch_filter_hz_{40.0}, pitch_rate_filter_hz_{25.0};
  double wheel_velocity_filter_hz_{30.0}, position_velocity_filter_hz_{12.0};
  double velocity_blend_{0.6}, velocity_mismatch_warn_mps_{0.50};
  bool auto_calibrate_on_first_valid_data_{false};

  bool stiction_enable_{false};
  double stiction_velocity_limit_mps_{0.015};
  double stiction_pitch_rate_limit_rad_s_{0.08};
  double stiction_position_error_m_{0.008};
  double stiction_command_min_total_nm_{0.002};
  double stiction_compensation_each_nm_{0.005};

  Quaternion imu_zero_{};
  bool imu_zero_valid_{false}, calibrated_{false}, armed_{false};
  bool arm_transition_required_{false};
  int last_switch_value_{255};
  WrappedAngleUnwrapper left_unwrapper_, right_unwrapper_;
  FirstOrderLowPass pitch_filter_, pitch_rate_filter_, wheel_velocity_filter_;
  FirstOrderLowPass position_velocity_filter_, outer_velocity_filter_;
  FirstOrderLowPass yaw_rate_filter_, yaw_acceleration_filter_;

  double target_position_m_{0.0}, target_velocity_mps_{0.0}, target_yaw_rate_rad_s_{0.0};
  double commanded_target_velocity_mps_{0.0}, commanded_target_yaw_rate_rad_s_{0.0};
  double last_position_m_{0.0};
  double velocity_from_position_raw_mps_{0.0};
  double velocity_from_position_filtered_mps_{0.0};
  bool position_fd_initialized_{false}, encoder_fd_time_initialized_{false};
  std::uint64_t last_left_fd_sequence_{0}, last_right_fd_sequence_{0};
  std::chrono::steady_clock::time_point last_encoder_fd_time_{};
  bool control_time_initialized_{false};
  std::chrono::steady_clock::time_point last_control_steady_time_{};

  double cascade_pitch_setpoint_rad_{0.0};
  double cascade_position_integral_m_s_{0.0};
  double auto_trim_stationary_time_s_{0.0};
  bool auto_trim_learning_flag_{false};
  bool yaw_rate_initialized_{false};
  double last_yaw_rate_filtered_rad_s_{0.0};
  double yaw_differential_command_nm_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MicroLqrController>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("micro_lqr_controller"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
