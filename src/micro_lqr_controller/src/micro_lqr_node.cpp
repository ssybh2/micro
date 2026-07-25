#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>

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

using Matrix4 = Eigen::Matrix<double, 4, 4>;
using Vector4 = Eigen::Matrix<double, 4, 1>;
using RowVector4 = Eigen::Matrix<double, 1, 4>;
using Vector1 = Eigen::Matrix<double, 1, 1>;

// Plant state order: [pitch, pitch_rate, position, velocity].
// Positive model input is TOTAL axle torque.

double clamp_value(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
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
  result.w =
    reference.w * current.w + reference.x * current.x +
    reference.y * current.y + reference.z * current.z;
  result.x =
    reference.w * current.x - reference.x * current.w -
    reference.y * current.z + reference.z * current.y;
  result.y =
    reference.w * current.y + reference.x * current.z -
    reference.y * current.w - reference.z * current.x;
  result.z =
    reference.w * current.z - reference.x * current.y +
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
      dt = clamp_value(actual_dt, 1.0e-6, 0.020);
    }
    const double tau = 1.0 / (2.0 * kPi * cutoff_hz_);
    const double alpha = dt / (tau + dt);
    value_ += alpha * (input - value_);
    return value_;
  }

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
    return overvoltage || undervoltage || overcurrent || mos_overtemperature ||
           rotor_overtemperature || communication_lost || overload;
  }
};

struct LqrDesignResult
{
  Matrix4 a_continuous{Matrix4::Zero()};
  Vector4 b_continuous{Vector4::Zero()};
  Matrix4 a_discrete{Matrix4::Zero()};
  Vector4 b_discrete{Vector4::Zero()};
  RowVector4 k{RowVector4::Zero()};
  int controllability_rank{0};
  double max_closed_loop_pole_abs{0.0};
  int dare_iterations{0};
};

double max_closed_loop_pole_abs(
  const Matrix4 & a_discrete,
  const Vector4 & b_discrete,
  const RowVector4 & k,
  const double gain_scale)
{
  const Matrix4 a_closed_loop = a_discrete - b_discrete * (gain_scale * k);
  const Eigen::EigenSolver<Matrix4> eigen_solver(a_closed_loop, false);
  double max_abs = 0.0;
  for (int i = 0; i < 4; ++i) {
    max_abs = std::max(max_abs, std::abs(eigen_solver.eigenvalues()(i)));
  }
  return max_abs;
}

LqrDesignResult design_discrete_lqr(
  const double body_mass,
  const double non_pitch_mass,
  const double wheel_inertia_each,
  const double com_height,
  const double body_pitch_inertia,
  const double wheel_radius,
  const double gravity,
  const double sample_time,
  const Vector4 & q_diagonal,
  const double r_total_torque,
  const bool use_course_legacy_b4,
  const int max_iterations,
  const double tolerance)
{
  if (
    body_mass <= 0.0 || non_pitch_mass < 0.0 || wheel_inertia_each < 0.0 ||
    com_height <= 0.0 || body_pitch_inertia <= 0.0 || wheel_radius <= 0.0 ||
    gravity <= 0.0 || sample_time <= 0.0 || r_total_torque <= 0.0)
  {
    throw std::runtime_error("invalid physical or LQR parameter");
  }
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(q_diagonal(i)) || q_diagonal(i) < 0.0) {
      throw std::runtime_error("all Q diagonal values must be finite and >= 0");
    }
  }

  const double m = body_mass;
  const double m_cart =
    non_pitch_mass + 2.0 * wheel_inertia_each / (wheel_radius * wheel_radius);
  const double h = com_height;
  const double i_body = body_pitch_inertia;
  const double denominator =
    (m_cart + m) * (m * h * h + i_body) - m * m * h * h;
  if (!std::isfinite(denominator) || denominator <= 1.0e-12) {
    throw std::runtime_error("model denominator D is non-positive; check mass, h and inertia");
  }

  LqrDesignResult result;
  result.a_continuous <<
    0.0, 1.0, 0.0, 0.0,
    ((m_cart + m) * m * gravity * h) / denominator, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
    -(m * m * gravity * h * h) / denominator, 0.0, 0.0, 0.0;

  double b4 = (m * h * h + i_body) / (denominator * wheel_radius);
  if (use_course_legacy_b4) {
    b4 =
      1.0 / ((m_cart + m) * wheel_radius) -
      (m * m * h * h) /
      ((m_cart + m) * denominator * wheel_radius);
  }
  result.b_continuous <<
    0.0,
    -(m * h) / (denominator * wheel_radius),
    0.0,
    b4;

  Eigen::Matrix<double, 5, 5> augmented = Eigen::Matrix<double, 5, 5>::Zero();
  augmented.block<4, 4>(0, 0) = result.a_continuous;
  augmented.block<4, 1>(0, 4) = result.b_continuous;
  const Eigen::Matrix<double, 5, 5> exp_augmented =
    (augmented * sample_time).exp();
  result.a_discrete = exp_augmented.block<4, 4>(0, 0);
  result.b_discrete = exp_augmented.block<4, 1>(0, 4);

  Eigen::Matrix<double, 4, 4> controllability;
  controllability.col(0) = result.b_discrete;
  controllability.col(1) = result.a_discrete * result.b_discrete;
  controllability.col(2) = result.a_discrete * result.a_discrete * result.b_discrete;
  controllability.col(3) =
    result.a_discrete * result.a_discrete * result.a_discrete * result.b_discrete;
  result.controllability_rank = controllability.fullPivLu().rank();
  if (result.controllability_rank != 4) {
    throw std::runtime_error("discrete model is not fully controllable");
  }

  const Matrix4 q = q_diagonal.asDiagonal();
  const Vector1 r = Vector1::Constant(r_total_torque);
  Matrix4 p = q;
  bool converged = false;
  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    const Vector1 s = r + result.b_discrete.transpose() * p * result.b_discrete;
    if (!std::isfinite(s(0, 0)) || std::abs(s(0, 0)) < 1.0e-15) {
      throw std::runtime_error("DARE scalar denominator became invalid");
    }
    const RowVector4 k =
      s.inverse() * result.b_discrete.transpose() * p * result.a_discrete;
    const Matrix4 p_next =
      result.a_discrete.transpose() * p * result.a_discrete -
      result.a_discrete.transpose() * p * result.b_discrete * k + q;
    result.dare_iterations = iteration + 1;
    if ((p_next - p).norm() < tolerance) {
      p = p_next;
      converged = true;
      break;
    }
    p = p_next;
  }
  if (!converged) {
    throw std::runtime_error("DARE iteration did not converge");
  }

  const Vector1 s = r + result.b_discrete.transpose() * p * result.b_discrete;
  result.k = s.inverse() * result.b_discrete.transpose() * p * result.a_discrete;
  result.max_closed_loop_pole_abs =
    max_closed_loop_pole_abs(result.a_discrete, result.b_discrete, result.k, 1.0);
  if (!std::isfinite(result.max_closed_loop_pole_abs) ||
      result.max_closed_loop_pole_abs >= 1.0)
  {
    throw std::runtime_error("designed discrete LQR is not asymptotically stable");
  }
  return result;
}

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

  // Cascade-specific extension, appended after the legacy schema.
  double cascade_mode_flag{0.0};
  double pitch_setpoint_raw{0.0};
  double pitch_setpoint_limited{0.0};
  double pitch_setpoint_command{0.0};
  double outer_velocity_filtered{0.0};
  double position_integral{0.0};
  double attitude_error{0.0};
  double outer_saturation_flag{0.0};
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
    design_controller_and_check_local_stability();

    const auto qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        imu_.orientation = {
          msg->orientation.w, msg->orientation.x,
          msg->orientation.y, msg->orientation.z};
        imu_.gyro_x = msg->angular_velocity.x;
        imu_.received_time = now();
        imu_.received = true;
      });

    rc_sub_ = create_subscription<custom_msgs::msg::ReadDJIRC>(
      rc_topic_, qos,
      [this](const custom_msgs::msg::ReadDJIRC::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        rc_.online = msg->online != 0U;
        rc_.right_switch = msg->right_switch;
        rc_.left_y = msg->left_y;
        rc_.received_time = now();
        rc_.received = true;
      });

    left_motor_sub_ = create_subscription<custom_msgs::msg::ReadDmMotor>(
      left_motor_read_topic_, qos,
      [this](const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        copy_motor_message(*msg, left_motor_);
      });

    right_motor_sub_ = create_subscription<custom_msgs::msg::ReadDmMotor>(
      right_motor_read_topic_, qos,
      [this](const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
      {
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
      "controller started at %.3f Hz; mode=%s; dry_run=%s; state=[pitch,pitch_rate,x,x_dot]",
      1.0 / control_period_s_, control_mode_.c_str(), dry_run_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(), "RC right switch: calibrate=%d, arm=%d, disable=%d",
      calibrate_switch_value_, arm_switch_value_, disable_switch_value_);
  }

  ~MicroLqrController() override
  {
    publish_motor_commands(false, 0.0, 0.0);
  }

private:
  bool cascade_mode() const
  {
    return control_mode_ == "cascade";
  }

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
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/micro_lqr/debug");

    control_mode_ = declare_parameter<std::string>("control_mode", "cascade");
    control_period_s_ = declare_parameter<double>("control_period_s", 0.003);
    dry_run_ = declare_parameter<bool>("dry_run", true);
    per_wheel_torque_limit_ = declare_parameter<double>("torque_limit", 0.05);
    hard_per_wheel_torque_limit_ = declare_parameter<double>("hard_torque_limit", 0.45);
    lqr_gain_scale_ = declare_parameter<double>("lqr_gain_scale", 1.0);

    arm_max_tilt_rad_ =
      declare_parameter<double>("arm_max_tilt_deg", 3.0) * kPi / 180.0;
    arm_max_pitch_rate_rad_s_ =
      declare_parameter<double>("arm_max_pitch_rate_rad_s", 0.30);
    fall_cutoff_rad_ =
      declare_parameter<double>("fall_cutoff_deg", 25.0) * kPi / 180.0;
    imu_timeout_s_ = declare_parameter<double>("imu_timeout_s", 0.05);
    motor_timeout_s_ = declare_parameter<double>("motor_timeout_s", 0.05);
    rc_timeout_s_ = declare_parameter<double>("rc_timeout_s", 0.20);

    calibrate_switch_value_ = declare_parameter<int>("calibrate_switch_value", 1);
    arm_switch_value_ = declare_parameter<int>("arm_switch_value", 3);
    disable_switch_value_ = declare_parameter<int>("disable_switch_value", 2);

    enable_velocity_command_ = declare_parameter<bool>("enable_velocity_command", false);
    max_target_velocity_ = declare_parameter<double>("max_target_velocity", 0.30);
    rc_deadband_ = declare_parameter<double>("rc_deadband", 0.08);

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

    wheel_radius_m_ = declare_parameter<double>("model.wheel_radius_m", 0.030);
    body_mass_kg_ = declare_parameter<double>("model.body_mass_kg", 2.54);
    non_pitch_mass_kg_ = declare_parameter<double>("model.non_pitch_mass_kg", 0.26);
    wheel_inertia_each_kg_m2_ =
      declare_parameter<double>("model.wheel_inertia_each_kg_m2", 0.0);
    com_height_m_ = declare_parameter<double>("model.com_height_m", 0.120);
    body_pitch_inertia_kg_m2_ =
      declare_parameter<double>("model.body_pitch_inertia_kg_m2", 0.036576);
    gravity_mps2_ = declare_parameter<double>("model.gravity_mps2", 9.80665);
    use_course_legacy_b4_ =
      declare_parameter<bool>("model.use_course_legacy_b4", false);

    q_pitch_ = declare_parameter<double>("lqr.q_pitch", 1.0);
    q_pitch_rate_ = declare_parameter<double>("lqr.q_pitch_rate", 1.0);
    q_position_ = declare_parameter<double>("lqr.q_position", 1.0);
    q_velocity_ = declare_parameter<double>("lqr.q_velocity", 1.0);
    r_total_torque_ = declare_parameter<double>("lqr.r_total_torque", 10.0);
    dare_max_iterations_ = declare_parameter<int>("lqr.dare_max_iterations", 20000);
    dare_tolerance_ = declare_parameter<double>("lqr.dare_tolerance", 1.0e-12);
    use_manual_gain_ = declare_parameter<bool>("lqr.use_manual_gain", false);
    manual_k_pitch_ = declare_parameter<double>("lqr.manual_k_pitch", 0.0);
    manual_k_pitch_rate_ = declare_parameter<double>("lqr.manual_k_pitch_rate", 0.0);
    manual_k_position_ = declare_parameter<double>("lqr.manual_k_position", 0.0);
    manual_k_velocity_ = declare_parameter<double>("lqr.manual_k_velocity", 0.0);

    cascade_attitude_k_pitch_ =
      declare_parameter<double>("cascade.attitude_k_pitch", 0.80);
    cascade_attitude_k_pitch_rate_ =
      declare_parameter<double>("cascade.attitude_k_pitch_rate", 0.04);
    cascade_position_kp_rad_per_m_ =
      declare_parameter<double>("cascade.position_kp_rad_per_m", 0.05);
    cascade_velocity_kd_rad_per_mps_ =
      declare_parameter<double>("cascade.velocity_kd_rad_per_mps", 0.05);
    cascade_position_ki_rad_per_m_s_ =
      declare_parameter<double>("cascade.position_ki_rad_per_m_s", 0.0);
    cascade_enable_integral_ =
      declare_parameter<bool>("cascade.enable_integral", false);
    cascade_integral_limit_m_s_ =
      declare_parameter<double>("cascade.integral_limit_m_s", 0.20);
    cascade_pitch_limit_rad_ =
      declare_parameter<double>("cascade.pitch_limit_deg", 2.0) * kPi / 180.0;
    cascade_pitch_slew_rate_rad_s_ =
      declare_parameter<double>("cascade.pitch_slew_rate_deg_s", 5.0) * kPi / 180.0;
    cascade_position_error_limit_m_ =
      declare_parameter<double>("cascade.position_error_limit_m", 0.50);
    cascade_velocity_error_limit_mps_ =
      declare_parameter<double>("cascade.velocity_error_limit_mps", 0.80);
    cascade_outer_velocity_filter_hz_ =
      declare_parameter<double>("cascade.outer_velocity_filter_hz", 5.0);
    cascade_position_to_pitch_sign_ = sign_or_throw(
      declare_parameter<double>("cascade.position_to_pitch_sign", -1.0),
      "cascade.position_to_pitch_sign");

    motor_position_wrap_half_range_ = declare_parameter<double>(
      "motor_position_wrap_half_range", kPi);
    pitch_filter_hz_ = declare_parameter<double>("pitch_filter_hz", 40.0);
    pitch_rate_filter_hz_ = declare_parameter<double>("pitch_rate_filter_hz", 25.0);
    wheel_velocity_filter_hz_ =
      declare_parameter<double>("wheel_velocity_filter_hz", 30.0);
    velocity_blend_ = declare_parameter<double>("velocity_blend", 1.0);
    velocity_mismatch_warn_mps_ =
      declare_parameter<double>("velocity_mismatch_warn_mps", 0.50);
    auto_calibrate_on_first_valid_data_ =
      declare_parameter<bool>("auto_calibrate_on_first_valid_data", false);

    validate_parameters();
    left_unwrapper_.configure(motor_position_wrap_half_range_);
    right_unwrapper_.configure(motor_position_wrap_half_range_);
  }

  void validate_parameters() const
  {
    if (control_mode_ != "cascade" && control_mode_ != "lqr") {
      throw std::runtime_error("control_mode must be 'cascade' or 'lqr'");
    }
    if (!std::isfinite(control_period_s_) || control_period_s_ <= 0.0) {
      throw std::runtime_error("control_period_s must be positive");
    }
    if (
      !std::isfinite(per_wheel_torque_limit_) || per_wheel_torque_limit_ <= 0.0 ||
      !std::isfinite(hard_per_wheel_torque_limit_) || hard_per_wheel_torque_limit_ <= 0.0 ||
      per_wheel_torque_limit_ > hard_per_wheel_torque_limit_)
    {
      throw std::runtime_error("torque_limit must be positive and <= hard_torque_limit");
    }
    if (!std::isfinite(lqr_gain_scale_) || lqr_gain_scale_ <= 0.0) {
      throw std::runtime_error("lqr_gain_scale must be positive");
    }
    if (
      !std::isfinite(arm_max_tilt_rad_) || arm_max_tilt_rad_ <= 0.0 ||
      !std::isfinite(fall_cutoff_rad_) || fall_cutoff_rad_ <= arm_max_tilt_rad_)
    {
      throw std::runtime_error("fall_cutoff_deg must be greater than arm_max_tilt_deg");
    }
    if (!std::isfinite(velocity_blend_) || velocity_blend_ < 0.0 || velocity_blend_ > 1.0) {
      throw std::runtime_error("velocity_blend must be in [0,1]");
    }
    if (!std::isfinite(rc_deadband_) || rc_deadband_ < 0.0 || rc_deadband_ >= 1.0) {
      throw std::runtime_error("rc_deadband must be in [0,1)");
    }
    if (
      !std::isfinite(cascade_attitude_k_pitch_) || cascade_attitude_k_pitch_ <= 0.0 ||
      !std::isfinite(cascade_attitude_k_pitch_rate_) || cascade_attitude_k_pitch_rate_ < 0.0 ||
      !std::isfinite(cascade_position_kp_rad_per_m_) || cascade_position_kp_rad_per_m_ < 0.0 ||
      !std::isfinite(cascade_velocity_kd_rad_per_mps_) || cascade_velocity_kd_rad_per_mps_ < 0.0 ||
      !std::isfinite(cascade_position_ki_rad_per_m_s_) || cascade_position_ki_rad_per_m_s_ < 0.0)
    {
      throw std::runtime_error("cascade gains must be finite; Kp values positive and Kd/Ki non-negative");
    }
    if (
      !std::isfinite(cascade_pitch_limit_rad_) || cascade_pitch_limit_rad_ <= 0.0 ||
      !std::isfinite(cascade_pitch_slew_rate_rad_s_) || cascade_pitch_slew_rate_rad_s_ <= 0.0 ||
      !std::isfinite(cascade_position_error_limit_m_) || cascade_position_error_limit_m_ <= 0.0 ||
      !std::isfinite(cascade_velocity_error_limit_mps_) || cascade_velocity_error_limit_mps_ <= 0.0 ||
      !std::isfinite(cascade_integral_limit_m_s_) || cascade_integral_limit_m_s_ < 0.0 ||
      !std::isfinite(cascade_outer_velocity_filter_hz_) || cascade_outer_velocity_filter_hz_ < 0.0)
    {
      throw std::runtime_error("cascade limits/filter parameters are invalid");
    }
  }

  void configure_filters()
  {
    pitch_filter_.configure(pitch_filter_hz_, control_period_s_);
    pitch_rate_filter_.configure(pitch_rate_filter_hz_, control_period_s_);
    wheel_velocity_filter_.configure(wheel_velocity_filter_hz_, control_period_s_);
    outer_velocity_filter_.configure(cascade_outer_velocity_filter_hz_, control_period_s_);
  }

  RowVector4 cascade_equivalent_k() const
  {
    // Visualizer convention: u = -K*x. Runtime cascade (before nonlinear limits):
    // u = Ka*pitch + Kd*rate - Ka*sign*(Kx*x + Kv*v).
    RowVector4 k;
    k <<
      -cascade_attitude_k_pitch_,
      -cascade_attitude_k_pitch_rate_,
      cascade_attitude_k_pitch_ * cascade_position_to_pitch_sign_ *
        cascade_position_kp_rad_per_m_,
      cascade_attitude_k_pitch_ * cascade_position_to_pitch_sign_ *
        cascade_velocity_kd_rad_per_mps_;
    return k;
  }

  RowVector4 active_equivalent_k() const
  {
    return cascade_mode() ? cascade_equivalent_k() : lqr_design_.k;
  }

  double active_equivalent_gain_scale() const
  {
    return cascade_mode() ? 1.0 : lqr_gain_scale_;
  }

  void design_controller_and_check_local_stability()
  {
    const Vector4 q_diag(q_pitch_, q_pitch_rate_, q_position_, q_velocity_);
    lqr_design_ = design_discrete_lqr(
      body_mass_kg_, non_pitch_mass_kg_, wheel_inertia_each_kg_m2_,
      com_height_m_, body_pitch_inertia_kg_m2_, wheel_radius_m_, gravity_mps2_,
      control_period_s_, q_diag, r_total_torque_, use_course_legacy_b4_,
      dare_max_iterations_, dare_tolerance_);

    if (use_manual_gain_) {
      lqr_design_.k <<
        manual_k_pitch_, manual_k_pitch_rate_, manual_k_position_, manual_k_velocity_;
    }

    const RowVector4 active_k = active_equivalent_k();
    const double active_scale = active_equivalent_gain_scale();
    active_local_max_pole_abs_ = max_closed_loop_pole_abs(
      lqr_design_.a_discrete, lqr_design_.b_discrete, active_k, active_scale);
    if (!std::isfinite(active_local_max_pole_abs_) || active_local_max_pole_abs_ >= 1.0) {
      throw std::runtime_error(
              "active controller local linearization is unstable; adjust YAML before running");
    }

    if (cascade_mode()) {
      RCLCPP_INFO(
        get_logger(),
        "CASCADE inner: u=%.4f*(pitch-pitch_sp)+%.4f*pitch_rate; "
        "outer: pitch_sp=%+.0f*(%.4f*x_err+%.4f*v_err), limit=%.2fdeg, slew=%.2fdeg/s",
        cascade_attitude_k_pitch_, cascade_attitude_k_pitch_rate_,
        cascade_position_to_pitch_sign_, cascade_position_kp_rad_per_m_,
        cascade_velocity_kd_rad_per_mps_, cascade_pitch_limit_rad_ * 180.0 / kPi,
        cascade_pitch_slew_rate_rad_s_ * 180.0 / kPi);
      RCLCPP_INFO(
        get_logger(),
        "cascade local equivalent K=[%+.6f %+.6f %+.6f %+.6f], max|pole|=%.9f "
        "(limits, slew and filters are nonlinear and not included)",
        active_k(0), active_k(1), active_k(2), active_k(3), active_local_max_pole_abs_);
    } else {
      lqr_design_.max_closed_loop_pole_abs = active_local_max_pole_abs_;
      RCLCPP_INFO(
        get_logger(),
        "LQR K=[%+.9f %+.9f %+.9f %+.9f], scale=%.4f, controllability=%d, "
        "max|pole|=%.9f, DARE=%d",
        lqr_design_.k(0), lqr_design_.k(1), lqr_design_.k(2), lqr_design_.k(3),
        lqr_gain_scale_, lqr_design_.controllability_rank,
        lqr_design_.max_closed_loop_pole_abs, lqr_design_.dare_iterations);
    }
    RCLCPP_INFO(
      get_logger(),
      "model input is TOTAL axle torque; each wheel receives 0.5*u_total before motor sign");
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
    sample.received = true;
  }

  bool is_fresh(
    const bool received, const rclcpp::Time & stamp,
    const double timeout_s, const rclcpp::Time & current_time) const
  {
    return received && (current_time - stamp).seconds() <= timeout_s;
  }

  bool all_input_data_valid(const rclcpp::Time & current_time) const
  {
    return
      is_fresh(imu_.received, imu_.received_time, imu_timeout_s_, current_time) &&
      is_fresh(rc_.received, rc_.received_time, rc_timeout_s_, current_time) &&
      is_fresh(left_motor_.received, left_motor_.received_time, motor_timeout_s_, current_time) &&
      is_fresh(right_motor_.received, right_motor_.received_time, motor_timeout_s_, current_time) &&
      rc_.online && left_motor_.online && right_motor_.online &&
      !left_motor_.has_fault() && !right_motor_.has_fault();
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
    const Quaternion relative = relative_quaternion(imu_zero_, current);
    pitch = imu_angle_sign_ * quaternion_roll(relative);
    return std::isfinite(pitch);
  }

  void reset_controller_states()
  {
    target_position_m_ = 0.0;
    target_velocity_mps_ = 0.0;
    last_position_m_ = 0.0;
    position_fd_initialized_ = false;
    control_time_initialized_ = false;
    cascade_pitch_setpoint_rad_ = 0.0;
    cascade_position_integral_m_s_ = 0.0;
  }

  void calibrate_zero()
  {
    Quaternion current = imu_.orientation;
    if (!normalize_quaternion(current)) {
      RCLCPP_ERROR(get_logger(), "Calibration rejected: invalid IMU quaternion");
      return;
    }
    imu_zero_ = current;
    imu_zero_valid_ = true;
    left_unwrapper_.reset(left_motor_.position);
    right_unwrapper_.reset(right_motor_.position);
    pitch_filter_.reset(0.0);
    pitch_rate_filter_.reset(0.0);
    wheel_velocity_filter_.reset(0.0);
    outer_velocity_filter_.reset(0.0);
    reset_controller_states();
    calibrated_ = true;
    armed_ = false;
    arm_transition_required_ = true;
    RCLCPP_INFO(get_logger(), "Zero calibrated: IMU attitude and wheel positions stored");
  }

  void disarm(const char * reason, const bool require_switch_cycle)
  {
    if (armed_) {
      RCLCPP_WARN(get_logger(), "controller disarmed: %s", reason);
    }
    armed_ = false;
    position_fd_initialized_ = false;
    cascade_pitch_setpoint_rad_ = 0.0;
    cascade_position_integral_m_s_ = 0.0;
    if (require_switch_cycle) {
      arm_transition_required_ = true;
    }
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
    if (!std::isfinite(pitch_rate_raw)) {
      RCLCPP_WARN(get_logger(), "Arm rejected: invalid pitch rate");
      arm_transition_required_ = true;
      return;
    }
    if (std::abs(pitch) > arm_max_tilt_rad_) {
      RCLCPP_WARN(
        get_logger(), "Arm rejected: |pitch|=%.2f deg exceeds %.2f deg",
        std::abs(pitch) * 180.0 / kPi, arm_max_tilt_rad_ * 180.0 / kPi);
      arm_transition_required_ = true;
      return;
    }
    if (std::abs(pitch_rate_raw) > arm_max_pitch_rate_rad_s_) {
      RCLCPP_WARN(
        get_logger(), "Arm rejected: |pitch_rate|=%.3f rad/s exceeds %.3f rad/s",
        std::abs(pitch_rate_raw), arm_max_pitch_rate_rad_s_);
      arm_transition_required_ = true;
      return;
    }

    const double left_angle =
      left_encoder_sign_ * left_unwrapper_.update(left_motor_.position);
    const double right_angle =
      right_encoder_sign_ * right_unwrapper_.update(right_motor_.position);
    const double mean_relative_angle = 0.5 * (left_angle + right_angle);
    const double mean_relative_rate_raw = 0.5 * (
      left_encoder_sign_ * left_motor_.velocity +
      right_encoder_sign_ * right_motor_.velocity);

    target_position_m_ = wheel_radius_m_ * (
      mean_relative_angle + pitch_position_compensation_sign_ * pitch);
    target_velocity_mps_ = 0.0;
    pitch_filter_.reset(pitch);
    pitch_rate_filter_.reset(pitch_rate_raw);
    wheel_velocity_filter_.reset(mean_relative_rate_raw);
    outer_velocity_filter_.reset(0.0);
    last_position_m_ = target_position_m_;
    position_fd_initialized_ = true;
    control_time_initialized_ = false;
    cascade_pitch_setpoint_rad_ = 0.0;
    cascade_position_integral_m_s_ = 0.0;
    armed_ = true;
    arm_transition_required_ = false;
    RCLCPP_INFO(
      get_logger(),
      "%s armed; target_position=%.5f m; pitch=%+.3fdeg; rate=%+.3frad/s; dry_run=%s",
      cascade_mode() ? "CASCADE" : "LQR", target_position_m_, pitch * 180.0 / kPi,
      pitch_rate_raw, dry_run_ ? "true" : "false");
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
      (std::abs(command) - rc_deadband_) / std::max(1.0 - rc_deadband_, 1.0e-6);
    return std::copysign(scaled * max_target_velocity_, command);
  }

  double calculate_cascade_torque(
    const double pitch,
    const double pitch_rate,
    const double position_error,
    const double velocity_error,
    const double actual_dt,
    DebugSample & debug)
  {
    const double outer_velocity = outer_velocity_filter_.update(velocity_error, actual_dt);
    const double limited_position_error = clamp_value(
      position_error, -cascade_position_error_limit_m_, cascade_position_error_limit_m_);
    const double limited_velocity_error = clamp_value(
      outer_velocity, -cascade_velocity_error_limit_mps_, cascade_velocity_error_limit_mps_);

    if (cascade_enable_integral_) {
      cascade_position_integral_m_s_ += limited_position_error * actual_dt;
      cascade_position_integral_m_s_ = clamp_value(
        cascade_position_integral_m_s_,
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

    const double max_setpoint_step = cascade_pitch_slew_rate_rad_s_ * actual_dt;
    const double setpoint_delta = clamp_value(
      pitch_setpoint_limited - cascade_pitch_setpoint_rad_,
      -max_setpoint_step, max_setpoint_step);
    cascade_pitch_setpoint_rad_ += setpoint_delta;

    // Prevent integral wind-up when the pitch setpoint is limited.
    if (cascade_enable_integral_ && outer_saturated) {
      cascade_position_integral_m_s_ -= limited_position_error * actual_dt;
      cascade_position_integral_m_s_ = clamp_value(
        cascade_position_integral_m_s_,
        -cascade_integral_limit_m_s_, cascade_integral_limit_m_s_);
    }

    const double attitude_error = pitch - cascade_pitch_setpoint_rad_;
    const double u_pitch = cascade_attitude_k_pitch_ * pitch;
    const double u_pitch_rate = cascade_attitude_k_pitch_rate_ * pitch_rate;
    const double u_position_equivalent =
      -cascade_attitude_k_pitch_ * cascade_position_to_pitch_sign_ *
      cascade_position_kp_rad_per_m_ * limited_position_error;
    const double u_velocity_equivalent =
      -cascade_attitude_k_pitch_ * cascade_position_to_pitch_sign_ *
      cascade_velocity_kd_rad_per_mps_ * limited_velocity_error;
    const double u_total =
      cascade_attitude_k_pitch_ * attitude_error + u_pitch_rate;

    debug.u_pitch = u_pitch;
    debug.u_pitch_rate = u_pitch_rate;
    debug.u_x = u_position_equivalent;
    debug.u_x_dot = u_velocity_equivalent;
    debug.pitch_setpoint_raw = pitch_setpoint_raw;
    debug.pitch_setpoint_limited = pitch_setpoint_limited;
    debug.pitch_setpoint_command = cascade_pitch_setpoint_rad_;
    debug.outer_velocity_filtered = outer_velocity;
    debug.position_integral = cascade_position_integral_m_s_;
    debug.attitude_error = attitude_error;
    debug.outer_saturation_flag = outer_saturated ? 1.0 : 0.0;
    return u_total;
  }

  double calculate_lqr_torque(
    const double pitch,
    const double pitch_rate,
    const double position_error,
    const double velocity_error,
    DebugSample & debug) const
  {
    Vector4 state_error;
    state_error << pitch, pitch_rate, position_error, velocity_error;
    const double u_total = -lqr_gain_scale_ * (lqr_design_.k * state_error)(0);
    debug.u_pitch = -lqr_gain_scale_ * lqr_design_.k(0) * pitch;
    debug.u_pitch_rate = -lqr_gain_scale_ * lqr_design_.k(1) * pitch_rate;
    debug.u_x = -lqr_gain_scale_ * lqr_design_.k(2) * position_error;
    debug.u_x_dot = -lqr_gain_scale_ * lqr_design_.k(3) * velocity_error;
    return u_total;
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
    if (!std::isfinite(actual_dt) || actual_dt <= 0.0) {
      actual_dt = control_period_s_;
    }
    actual_dt = clamp_value(actual_dt, 0.0005, 0.020);

    std::lock_guard<std::mutex> lock(data_mutex_);
    const rclcpp::Time current_time = now();
    DebugSample debug;
    debug.actual_dt = actual_dt;
    debug.cascade_mode_flag = cascade_mode() ? 1.0 : 0.0;

    if (!all_input_data_valid(current_time)) {
      disarm("input timeout, RC offline, motor offline, or motor fault", true);
      last_switch_value_ = 255;
      publish_debug(debug);
      return;
    }

    debug.imu_age_s = std::max(0.0, (current_time - imu_.received_time).seconds());
    debug.left_motor_age_s =
      std::max(0.0, (current_time - left_motor_.received_time).seconds());
    debug.right_motor_age_s =
      std::max(0.0, (current_time - right_motor_.received_time).seconds());
    debug.left_position_raw = left_motor_.position;
    debug.right_position_raw = right_motor_.position;
    debug.left_velocity_raw = left_motor_.velocity;
    debug.right_velocity_raw = right_motor_.velocity;

    const int switch_value = static_cast<int>(rc_.right_switch);
    const bool switch_changed = switch_value != last_switch_value_;
    if (auto_calibrate_on_first_valid_data_ && !calibrated_) {
      calibrate_zero();
    }
    if (switch_changed && switch_value == calibrate_switch_value_) {
      disarm("calibration switch selected", false);
      calibrate_zero();
    }
    if (switch_value != arm_switch_value_) {
      disarm("RC switch is not in arm position", false);
      arm_transition_required_ = false;
      last_switch_value_ = switch_value;
      publish_debug(debug);
      return;
    }
    if (switch_changed) {
      if (arm_transition_required_) {
        RCLCPP_WARN(get_logger(), "Arm blocked: move RC switch away from ARM and back to ARM");
      } else {
        attempt_arm();
      }
    }
    last_switch_value_ = switch_value;
    if (!armed_) {
      publish_motor_commands(false, 0.0, 0.0);
      publish_debug(debug);
      return;
    }

    double pitch_raw = 0.0;
    if (!calculate_raw_pitch(pitch_raw)) {
      disarm("invalid IMU quaternion", true);
      publish_debug(debug);
      return;
    }
    const double pitch_rate_raw = imu_rate_sign_ * imu_.gyro_x;
    if (!std::isfinite(pitch_rate_raw)) {
      disarm("invalid IMU pitch rate", true);
      publish_debug(debug);
      return;
    }
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

    const double left_angle =
      left_encoder_sign_ * left_unwrapper_.update(left_motor_.position);
    const double right_angle =
      right_encoder_sign_ * right_unwrapper_.update(right_motor_.position);
    const double mean_relative_angle = 0.5 * (left_angle + right_angle);
    const double mean_relative_rate_raw = 0.5 * (
      left_encoder_sign_ * left_motor_.velocity +
      right_encoder_sign_ * right_motor_.velocity);
    const double mean_relative_rate =
      wheel_velocity_filter_.update(mean_relative_rate_raw, actual_dt);

    const double position_m = wheel_radius_m_ * (
      mean_relative_angle + pitch_position_compensation_sign_ * pitch);
    const double velocity_motor_based = wheel_radius_m_ * (
      mean_relative_rate + pitch_rate_compensation_sign_ * pitch_rate);

    double velocity_from_position = velocity_motor_based;
    if (position_fd_initialized_) {
      velocity_from_position =
        (position_m - last_position_m_) / std::max(actual_dt, 1.0e-6);
    }
    last_position_m_ = position_m;
    position_fd_initialized_ = true;
    const double velocity_mps =
      velocity_blend_ * velocity_motor_based +
      (1.0 - velocity_blend_) * velocity_from_position;

    target_velocity_mps_ = process_rc_velocity_command();
    target_position_m_ += target_velocity_mps_ * actual_dt;

    const double position_error = position_m - target_position_m_;
    const double velocity_error = velocity_mps - target_velocity_mps_;

    double u_total_unsaturated = 0.0;
    if (cascade_mode()) {
      u_total_unsaturated = calculate_cascade_torque(
        pitch, pitch_rate, position_error, velocity_error, actual_dt, debug);
    } else {
      u_total_unsaturated = calculate_lqr_torque(
        pitch, pitch_rate, position_error, velocity_error, debug);
    }

    const double max_total_torque = 2.0 * per_wheel_torque_limit_;
    const double signed_total_torque = output_gain_sign_ * u_total_unsaturated;
    const double total_torque_after_limit = clamp_value(
      signed_total_torque, -max_total_torque, max_total_torque);
    const bool saturated =
      std::abs(signed_total_torque - total_torque_after_limit) > 1.0e-9;
    const double per_wheel_common_torque = 0.5 * total_torque_after_limit;
    const double left_command = left_motor_sign_ * per_wheel_common_torque;
    const double right_command = right_motor_sign_ * per_wheel_common_torque;

    if (dry_run_) {
      publish_motor_commands(false, 0.0, 0.0);
    } else {
      publish_motor_commands(true, left_command, right_command);
    }

    debug.position = position_m;
    debug.velocity = velocity_mps;
    debug.target_position = target_position_m_;
    debug.target_velocity = target_velocity_mps_;
    debug.model_total_unsaturated = u_total_unsaturated;
    debug.per_wheel_common_torque = per_wheel_common_torque;
    debug.total_torque_after_limit = total_torque_after_limit;
    debug.saturation_flag = saturated ? 1.0 : 0.0;
    debug.velocity_from_position = velocity_from_position;
    debug.position_error = position_error;
    debug.velocity_error = velocity_error;
    debug.velocity_motor_based = velocity_motor_based;
    debug.velocity_mismatch = velocity_motor_based - velocity_from_position;
    publish_debug(debug);

    if (std::abs(debug.velocity_mismatch) > velocity_mismatch_warn_mps_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "velocity mismatch: motor=%+.3f m/s, position_fd=%+.3f m/s; check encoder signs/wrap",
        velocity_motor_based, velocity_from_position);
    }

    if (cascade_mode()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 100,
        "pitch=%+.2fdeg rate=%+.3f x=%+.4f ex=%+.4f v=%+.4f vf=%+.4f "
        "pitch_sp=%+.2fdeg uT=%+.4f perWheel=%+.4f sat=%d outer_sat=%d",
        pitch * 180.0 / kPi, pitch_rate, position_m, position_error,
        velocity_mps, debug.outer_velocity_filtered,
        debug.pitch_setpoint_command * 180.0 / kPi,
        u_total_unsaturated, per_wheel_common_torque, saturated ? 1 : 0,
        debug.outer_saturation_flag > 0.5 ? 1 : 0);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 100,
        "pitch=%+.2fdeg rate=%+.3f x=%+.4f v=%+.4f "
        "uT=%+.4f perWheel=%+.4f sat=%d dt=%.3fms imu=%.3fms",
        pitch * 180.0 / kPi, pitch_rate, position_m, velocity_mps,
        u_total_unsaturated, per_wheel_common_torque, saturated ? 1 : 0,
        actual_dt * 1000.0, debug.imu_age_s * 1000.0);
    }
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

  void publish_debug(const DebugSample & sample)
  {
    const RowVector4 equivalent_k = active_equivalent_k();
    std_msgs::msg::Float64MultiArray message;
    message.data = {
      sample.position,                         // 0
      sample.velocity,                         // 1
      sample.pitch,                            // 2
      sample.pitch_rate,                       // 3
      sample.target_position,                  // 4
      sample.target_velocity,                  // 5
      sample.model_total_unsaturated,           // 6
      sample.per_wheel_common_torque,           // 7
      armed_ ? 1.0 : 0.0,                      // 8
      dry_run_ ? 1.0 : 0.0,                    // 9
      output_gain_sign_,                        // 10
      per_wheel_torque_limit_,                  // 11
      sample.pitch_raw,                         // 12
      sample.pitch_rate_raw,                    // 13
      sample.u_x,                               // 14
      sample.u_x_dot,                           // 15
      sample.u_pitch,                           // 16
      sample.u_pitch_rate,                      // 17
      sample.total_torque_after_limit,           // 18
      sample.saturation_flag,                   // 19
      sample.actual_dt,                         // 20
      sample.imu_age_s,                         // 21
      sample.left_motor_age_s,                  // 22
      sample.right_motor_age_s,                 // 23
      sample.left_position_raw,                 // 24
      sample.right_position_raw,                // 25
      sample.left_velocity_raw,                 // 26
      sample.right_velocity_raw,                // 27
      sample.velocity_from_position,            // 28
      sample.position_error,                    // 29
      sample.velocity_error,                    // 30
      equivalent_k(0),                          // 31
      equivalent_k(1),                          // 32
      equivalent_k(2),                          // 33
      equivalent_k(3),                          // 34
      sample.velocity_motor_based,              // 35
      sample.velocity_mismatch,                 // 36
      static_cast<double>(lqr_design_.controllability_rank),  // 37
      active_local_max_pole_abs_,                // 38
      active_equivalent_gain_scale(),            // 39
      use_course_legacy_b4_ ? 1.0 : 0.0,         // 40
      sample.cascade_mode_flag,                  // 41
      sample.pitch_setpoint_raw,                 // 42
      sample.pitch_setpoint_limited,             // 43
      sample.pitch_setpoint_command,             // 44
      sample.outer_velocity_filtered,            // 45
      sample.position_integral,                  // 46
      sample.attitude_error,                     // 47
      sample.outer_saturation_flag,              // 48
      cascade_pitch_limit_rad_,                  // 49
      cascade_pitch_slew_rate_rad_s_             // 50
    };
    debug_pub_->publish(message);
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    const bool old_dry_run = dry_run_;
    const double old_torque_limit = per_wheel_torque_limit_;
    const double old_output_gain_sign = output_gain_sign_;
    const bool old_enable_velocity_command = enable_velocity_command_;
    const double old_max_target_velocity = max_target_velocity_;
    bool force_disarm = false;

    try {
      for (const auto & parameter : parameters) {
        const std::string & name = parameter.get_name();
        if (name == "dry_run") {
          dry_run_ = parameter.as_bool();
          force_disarm = true;
        } else if (name == "torque_limit") {
          per_wheel_torque_limit_ = parameter.as_double();
          force_disarm = true;
        } else if (name == "output_gain_sign") {
          output_gain_sign_ = sign_or_throw(parameter.as_double(), name);
          force_disarm = true;
        } else if (name == "enable_velocity_command") {
          enable_velocity_command_ = parameter.as_bool();
          force_disarm = true;
        } else if (name == "max_target_velocity") {
          max_target_velocity_ = parameter.as_double();
          force_disarm = true;
        } else {
          throw std::runtime_error(name + " requires node restart and YAML edit");
        }
      }
      validate_parameters();
    } catch (const std::exception & exception) {
      dry_run_ = old_dry_run;
      per_wheel_torque_limit_ = old_torque_limit;
      output_gain_sign_ = old_output_gain_sign;
      enable_velocity_command_ = old_enable_velocity_command;
      max_target_velocity_ = old_max_target_velocity;
      result.successful = false;
      result.reason = exception.what();
      return result;
    }

    if (force_disarm) {
      armed_ = false;
      arm_transition_required_ = true;
      cascade_pitch_setpoint_rad_ = 0.0;
      cascade_position_integral_m_s_ = 0.0;
      publish_motor_commands(false, 0.0, 0.0);
    }
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

  std::string imu_topic_;
  std::string rc_topic_;
  std::string left_motor_read_topic_;
  std::string left_motor_write_topic_;
  std::string right_motor_read_topic_;
  std::string right_motor_write_topic_;
  std::string debug_topic_;
  std::string control_mode_{"cascade"};

  double control_period_s_{0.003};
  bool dry_run_{true};
  double per_wheel_torque_limit_{0.05};
  double hard_per_wheel_torque_limit_{0.45};
  double lqr_gain_scale_{1.0};
  double arm_max_tilt_rad_{3.0 * kPi / 180.0};
  double arm_max_pitch_rate_rad_s_{0.30};
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
  double body_mass_kg_{2.54};
  double non_pitch_mass_kg_{0.26};
  double wheel_inertia_each_kg_m2_{0.0};
  double com_height_m_{0.120};
  double body_pitch_inertia_kg_m2_{0.036576};
  double gravity_mps2_{9.80665};
  bool use_course_legacy_b4_{false};

  double q_pitch_{1.0};
  double q_pitch_rate_{1.0};
  double q_position_{1.0};
  double q_velocity_{1.0};
  double r_total_torque_{10.0};
  int dare_max_iterations_{20000};
  double dare_tolerance_{1.0e-12};
  bool use_manual_gain_{false};
  double manual_k_pitch_{0.0};
  double manual_k_pitch_rate_{0.0};
  double manual_k_position_{0.0};
  double manual_k_velocity_{0.0};
  LqrDesignResult lqr_design_;
  double active_local_max_pole_abs_{0.0};

  double cascade_attitude_k_pitch_{0.80};
  double cascade_attitude_k_pitch_rate_{0.04};
  double cascade_position_kp_rad_per_m_{0.05};
  double cascade_velocity_kd_rad_per_mps_{0.05};
  double cascade_position_ki_rad_per_m_s_{0.0};
  bool cascade_enable_integral_{false};
  double cascade_integral_limit_m_s_{0.20};
  double cascade_pitch_limit_rad_{2.0 * kPi / 180.0};
  double cascade_pitch_slew_rate_rad_s_{5.0 * kPi / 180.0};
  double cascade_position_error_limit_m_{0.50};
  double cascade_velocity_error_limit_mps_{0.80};
  double cascade_outer_velocity_filter_hz_{5.0};
  double cascade_position_to_pitch_sign_{-1.0};

  double motor_position_wrap_half_range_{kPi};
  double pitch_filter_hz_{40.0};
  double pitch_rate_filter_hz_{25.0};
  double wheel_velocity_filter_hz_{30.0};
  double velocity_blend_{1.0};
  double velocity_mismatch_warn_mps_{0.50};
  bool auto_calibrate_on_first_valid_data_{false};

  Quaternion imu_zero_{};
  bool imu_zero_valid_{false};
  bool calibrated_{false};
  bool armed_{false};
  bool arm_transition_required_{false};
  int last_switch_value_{255};

  WrappedAngleUnwrapper left_unwrapper_;
  WrappedAngleUnwrapper right_unwrapper_;
  FirstOrderLowPass pitch_filter_;
  FirstOrderLowPass pitch_rate_filter_;
  FirstOrderLowPass wheel_velocity_filter_;
  FirstOrderLowPass outer_velocity_filter_;

  double target_position_m_{0.0};
  double target_velocity_mps_{0.0};
  double last_position_m_{0.0};
  bool position_fd_initialized_{false};
  bool control_time_initialized_{false};
  std::chrono::steady_clock::time_point last_control_steady_time_{};

  double cascade_pitch_setpoint_rad_{0.0};
  double cascade_position_integral_m_s_{0.0};
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
