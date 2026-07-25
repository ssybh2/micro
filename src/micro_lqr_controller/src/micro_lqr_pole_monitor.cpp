#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>

#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kSchemaVersion = 1.0;

using Matrix4 = Eigen::Matrix<double, 4, 4>;
using Vector4 = Eigen::Matrix<double, 4, 1>;
using RowVector4 = Eigen::Matrix<double, 1, 4>;
using Vector1 = Eigen::Matrix<double, 1, 1>;

struct LqrModel
{
  Matrix4 a_discrete{Matrix4::Zero()};
  Vector4 b_discrete{Vector4::Zero()};
  RowVector4 automatic_k{RowVector4::Zero()};
  int controllability_rank{0};
  int dare_iterations{0};
};

struct PoleAnalysis
{
  std::array<std::complex<double>, 4> poles{};
  std::array<double, 4> magnitudes{};
  std::array<double, 4> damping_ratios{};
  std::array<double, 4> damped_frequencies_hz{};
  double max_abs{std::numeric_limits<double>::infinity()};
  bool stable{false};
};

struct ControllerParameters
{
  double sample_time{0.003};
  double gain_scale{1.0};

  double body_mass{0.0};
  double non_pitch_mass{0.0};
  double wheel_inertia_each{0.0};
  double com_height{0.0};
  double body_pitch_inertia{0.0};
  double wheel_radius{0.0};
  double gravity{9.80665};
  bool use_course_legacy_b4{false};

  Vector4 q_diagonal{Vector4::Zero()};
  double r_total_torque{1.0};
  int dare_max_iterations{20000};
  double dare_tolerance{1.0e-12};

  bool use_manual_gain{false};
  RowVector4 manual_k{RowVector4::Zero()};
};

LqrModel build_discrete_model_and_lqr(const ControllerParameters & p)
{
  if (
    p.body_mass <= 0.0 || p.non_pitch_mass < 0.0 || p.wheel_inertia_each < 0.0 ||
    p.com_height <= 0.0 || p.body_pitch_inertia <= 0.0 || p.wheel_radius <= 0.0 ||
    p.gravity <= 0.0 || p.sample_time <= 0.0 || p.r_total_torque <= 0.0)
  {
    throw std::runtime_error("invalid physical or LQR parameter");
  }
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(p.q_diagonal(i)) || p.q_diagonal(i) < 0.0) {
      throw std::runtime_error("all Q diagonal values must be finite and >= 0");
    }
  }

  // This is intentionally the same model construction used by micro_lqr_node.cpp.
  const double m = p.body_mass;
  const double m_cart =
    p.non_pitch_mass + 2.0 * p.wheel_inertia_each / (p.wheel_radius * p.wheel_radius);
  const double h = p.com_height;
  const double i_body = p.body_pitch_inertia;
  const double denominator =
    (m_cart + m) * (m * h * h + i_body) - m * m * h * h;
  if (!std::isfinite(denominator) || denominator <= 1.0e-12) {
    throw std::runtime_error("model denominator D is non-positive");
  }

  Matrix4 a_continuous;
  a_continuous <<
    0.0, 1.0, 0.0, 0.0,
    ((m_cart + m) * m * p.gravity * h) / denominator, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
    -(m * m * p.gravity * h * h) / denominator, 0.0, 0.0, 0.0;

  double b4 = (m * h * h + i_body) / (denominator * p.wheel_radius);
  if (p.use_course_legacy_b4) {
    b4 =
      1.0 / ((m_cart + m) * p.wheel_radius) -
      (m * m * h * h) /
      ((m_cart + m) * denominator * p.wheel_radius);
  }

  Vector4 b_continuous;
  b_continuous <<
    0.0,
    -(m * h) / (denominator * p.wheel_radius),
    0.0,
    b4;

  Eigen::Matrix<double, 5, 5> augmented = Eigen::Matrix<double, 5, 5>::Zero();
  augmented.block<4, 4>(0, 0) = a_continuous;
  augmented.block<4, 1>(0, 4) = b_continuous;
  const Eigen::Matrix<double, 5, 5> exp_augmented =
    (augmented * p.sample_time).exp();

  LqrModel result;
  result.a_discrete = exp_augmented.block<4, 4>(0, 0);
  result.b_discrete = exp_augmented.block<4, 1>(0, 4);

  Eigen::Matrix<double, 4, 4> controllability;
  controllability.col(0) = result.b_discrete;
  controllability.col(1) = result.a_discrete * result.b_discrete;
  controllability.col(2) =
    result.a_discrete * result.a_discrete * result.b_discrete;
  controllability.col(3) =
    result.a_discrete * result.a_discrete * result.a_discrete * result.b_discrete;
  result.controllability_rank = controllability.fullPivLu().rank();
  if (result.controllability_rank != 4) {
    throw std::runtime_error("discrete model is not fully controllable");
  }

  const Matrix4 q = p.q_diagonal.asDiagonal();
  const Vector1 r = Vector1::Constant(p.r_total_torque);
  Matrix4 p_riccati = q;
  bool converged = false;

  for (int iteration = 0; iteration < p.dare_max_iterations; ++iteration) {
    const Vector1 s =
      r + result.b_discrete.transpose() * p_riccati * result.b_discrete;
    if (!std::isfinite(s(0, 0)) || std::abs(s(0, 0)) < 1.0e-15) {
      throw std::runtime_error("DARE scalar denominator became invalid");
    }

    const RowVector4 k =
      s.inverse() * result.b_discrete.transpose() * p_riccati * result.a_discrete;
    const Matrix4 p_next =
      result.a_discrete.transpose() * p_riccati * result.a_discrete -
      result.a_discrete.transpose() * p_riccati * result.b_discrete * k + q;

    result.dare_iterations = iteration + 1;
    if ((p_next - p_riccati).norm() < p.dare_tolerance) {
      p_riccati = p_next;
      converged = true;
      break;
    }
    p_riccati = p_next;
  }

  if (!converged) {
    throw std::runtime_error("DARE iteration did not converge");
  }

  const Vector1 s =
    r + result.b_discrete.transpose() * p_riccati * result.b_discrete;
  result.automatic_k =
    s.inverse() * result.b_discrete.transpose() * p_riccati * result.a_discrete;
  return result;
}

PoleAnalysis analyze_closed_loop(
  const Matrix4 & a_discrete,
  const Vector4 & b_discrete,
  const RowVector4 & k,
  const double gain_scale,
  const double sample_time)
{
  if (!std::isfinite(gain_scale) || gain_scale <= 0.0 || sample_time <= 0.0) {
    throw std::runtime_error("gain scale and sample time must be positive");
  }
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(k(i))) {
      throw std::runtime_error("feedback gain contains a non-finite value");
    }
  }

  const Matrix4 a_closed_loop =
    a_discrete - b_discrete * (gain_scale * k);
  const Eigen::EigenSolver<Matrix4> solver(a_closed_loop, false);

  std::array<std::complex<double>, 4> sorted_poles{};
  for (int i = 0; i < 4; ++i) {
    sorted_poles[static_cast<std::size_t>(i)] = solver.eigenvalues()(i);
  }
  std::sort(
    sorted_poles.begin(), sorted_poles.end(),
    [](const std::complex<double> & lhs, const std::complex<double> & rhs)
    {
      if (std::abs(lhs.imag() - rhs.imag()) > 1.0e-12) {
        return lhs.imag() < rhs.imag();
      }
      return lhs.real() < rhs.real();
    });

  PoleAnalysis analysis;
  analysis.poles = sorted_poles;
  analysis.max_abs = 0.0;

  for (std::size_t i = 0; i < analysis.poles.size(); ++i) {
    const std::complex<double> lambda = analysis.poles[i];
    const double magnitude = std::abs(lambda);
    const std::complex<double> continuous_pole = std::log(lambda) / sample_time;
    const double natural_frequency = std::abs(continuous_pole);

    analysis.magnitudes[i] = magnitude;
    analysis.damping_ratios[i] =
      natural_frequency > 1.0e-12 ?
      -continuous_pole.real() / natural_frequency :
      std::numeric_limits<double>::quiet_NaN();
    analysis.damped_frequencies_hz[i] =
      std::abs(continuous_pole.imag()) / (2.0 * kPi);
    analysis.max_abs = std::max(analysis.max_abs, magnitude);
  }

  analysis.stable = std::isfinite(analysis.max_abs) && analysis.max_abs < 1.0;
  return analysis;
}

class ParameterReader
{
public:
  explicit ParameterReader(
    const std::unordered_map<std::string, rcl_interfaces::msg::ParameterValue> & values)
  : values_(values)
  {
  }

  double get_double(const std::string & name) const
  {
    const auto & value = find(name);
    if (value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      return value.double_value;
    }
    if (value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(value.integer_value);
    }
    throw std::runtime_error(name + " is not numeric");
  }

  int get_int(const std::string & name) const
  {
    const auto & value = find(name);
    if (value.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      throw std::runtime_error(name + " is not an integer");
    }
    return static_cast<int>(value.integer_value);
  }

  bool get_bool(const std::string & name) const
  {
    const auto & value = find(name);
    if (value.type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      throw std::runtime_error(name + " is not a bool");
    }
    return value.bool_value;
  }

private:
  const rcl_interfaces::msg::ParameterValue & find(const std::string & name) const
  {
    const auto iterator = values_.find(name);
    if (iterator == values_.end()) {
      throw std::runtime_error("missing controller parameter: " + name);
    }
    if (iterator->second.type == rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET) {
      throw std::runtime_error("controller parameter is not set: " + name);
    }
    return iterator->second;
  }

  const std::unordered_map<std::string, rcl_interfaces::msg::ParameterValue> & values_;
};

}  // namespace

class MicroLqrPoleMonitor : public rclcpp::Node
{
public:
  MicroLqrPoleMonitor()
  : Node("micro_lqr_pole_monitor")
  {
    controller_node_name_ = declare_parameter<std::string>(
      "controller_node_name", "/micro_lqr_controller");
    pole_topic_ = declare_parameter<std::string>(
      "pole_topic", "/micro_lqr/poles");
    monitor_period_s_ = declare_parameter<double>("monitor_period_s", 0.5);

    if (controller_node_name_.empty() || controller_node_name_.front() != '/') {
      controller_node_name_ = "/" + controller_node_name_;
    }
    if (!std::isfinite(monitor_period_s_) || monitor_period_s_ <= 0.0) {
      throw std::runtime_error("monitor_period_s must be positive");
    }

    parameter_names_ = {
      "control_period_s",
      "lqr_gain_scale",
      "model.body_mass_kg",
      "model.non_pitch_mass_kg",
      "model.wheel_inertia_each_kg_m2",
      "model.com_height_m",
      "model.body_pitch_inertia_kg_m2",
      "model.wheel_radius_m",
      "model.gravity_mps2",
      "model.use_course_legacy_b4",
      "lqr.q_pitch",
      "lqr.q_pitch_rate",
      "lqr.q_position",
      "lqr.q_velocity",
      "lqr.r_total_torque",
      "lqr.dare_max_iterations",
      "lqr.dare_tolerance",
      "lqr.use_manual_gain",
      "lqr.manual_k_pitch",
      "lqr.manual_k_pitch_rate",
      "lqr.manual_k_position",
      "lqr.manual_k_velocity"
    };

    const std::string service_name = controller_node_name_ + "/get_parameters";
    parameter_client_ = create_client<rcl_interfaces::srv::GetParameters>(service_name);

    const auto pole_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pole_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      pole_topic_, pole_qos);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(monitor_period_s_));
    timer_ = create_wall_timer(period, std::bind(&MicroLqrPoleMonitor::request_update, this));

    RCLCPP_INFO(
      get_logger(),
      "pole monitor waiting for %s; publishing %s every %.3f s",
      service_name.c_str(), pole_topic_.c_str(), monitor_period_s_);
  }

private:
  void request_update()
  {
    if (request_in_flight_) {
      return;
    }
    if (!parameter_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "waiting for controller parameter service: %s/get_parameters",
        controller_node_name_.c_str());
      return;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = parameter_names_;
    request_in_flight_ = true;

    using SharedFuture = rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture;
    parameter_client_->async_send_request(
      request,
      [this](SharedFuture future)
      {
        request_in_flight_ = false;
        try {
          const auto response = future.get();
          if (response->values.size() != parameter_names_.size()) {
            throw std::runtime_error("parameter response length mismatch");
          }

          std::unordered_map<std::string, rcl_interfaces::msg::ParameterValue> values;
          for (std::size_t i = 0; i < parameter_names_.size(); ++i) {
            values.emplace(parameter_names_[i], response->values[i]);
          }
          update_from_parameters(ParameterReader(values));
        } catch (const std::exception & exception) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "closed-loop pole calculation failed: %s", exception.what());
        }
      });
  }

  void update_from_parameters(const ParameterReader & reader)
  {
    ControllerParameters p;
    p.sample_time = reader.get_double("control_period_s");
    p.gain_scale = reader.get_double("lqr_gain_scale");

    p.body_mass = reader.get_double("model.body_mass_kg");
    p.non_pitch_mass = reader.get_double("model.non_pitch_mass_kg");
    p.wheel_inertia_each = reader.get_double("model.wheel_inertia_each_kg_m2");
    p.com_height = reader.get_double("model.com_height_m");
    p.body_pitch_inertia = reader.get_double("model.body_pitch_inertia_kg_m2");
    p.wheel_radius = reader.get_double("model.wheel_radius_m");
    p.gravity = reader.get_double("model.gravity_mps2");
    p.use_course_legacy_b4 = reader.get_bool("model.use_course_legacy_b4");

    p.q_diagonal <<
      reader.get_double("lqr.q_pitch"),
      reader.get_double("lqr.q_pitch_rate"),
      reader.get_double("lqr.q_position"),
      reader.get_double("lqr.q_velocity");
    p.r_total_torque = reader.get_double("lqr.r_total_torque");
    p.dare_max_iterations = reader.get_int("lqr.dare_max_iterations");
    p.dare_tolerance = reader.get_double("lqr.dare_tolerance");

    p.use_manual_gain = reader.get_bool("lqr.use_manual_gain");
    p.manual_k <<
      reader.get_double("lqr.manual_k_pitch"),
      reader.get_double("lqr.manual_k_pitch_rate"),
      reader.get_double("lqr.manual_k_position"),
      reader.get_double("lqr.manual_k_velocity");

    const LqrModel model = build_discrete_model_and_lqr(p);
    const RowVector4 active_k = p.use_manual_gain ? p.manual_k : model.automatic_k;
    const PoleAnalysis analysis = analyze_closed_loop(
      model.a_discrete, model.b_discrete, active_k, p.gain_scale, p.sample_time);

    publish_analysis(p, model, active_k, analysis);

    const bool changed =
      !have_last_result_ ||
      std::abs(analysis.max_abs - last_max_abs_) > 1.0e-9 ||
      (active_k - last_k_).norm() > 1.0e-12 ||
      std::abs(p.gain_scale - last_gain_scale_) > 1.0e-12;
    if (changed) {
      RCLCPP_INFO(
        get_logger(),
        "%s K=[%+.6f %+.6f %+.6f %+.6f], scale=%.6f, max|pole|=%.9f, margin=%+.9f",
        analysis.stable ? "STABLE" : "UNSTABLE",
        active_k(0), active_k(1), active_k(2), active_k(3),
        p.gain_scale, analysis.max_abs, 1.0 - analysis.max_abs);
      have_last_result_ = true;
      last_max_abs_ = analysis.max_abs;
      last_gain_scale_ = p.gain_scale;
      last_k_ = active_k;
    }
  }

  void publish_analysis(
    const ControllerParameters & p,
    const LqrModel & model,
    const RowVector4 & active_k,
    const PoleAnalysis & analysis)
  {
    std_msgs::msg::Float64MultiArray message;
    message.data.reserve(31);

    message.data.push_back(kSchemaVersion);                         // 0
    message.data.push_back(analysis.stable ? 1.0 : 0.0);           // 1
    message.data.push_back(analysis.max_abs);                      // 2
    message.data.push_back(1.0 - analysis.max_abs);                // 3 radial margin
    message.data.push_back(p.gain_scale);                          // 4
    message.data.push_back(p.sample_time);                         // 5
    message.data.push_back(p.use_manual_gain ? 1.0 : 0.0);        // 6
    message.data.push_back(active_k(0));                           // 7
    message.data.push_back(active_k(1));                           // 8
    message.data.push_back(active_k(2));                           // 9
    message.data.push_back(active_k(3));                           // 10

    for (std::size_t i = 0; i < analysis.poles.size(); ++i) {
      message.data.push_back(analysis.poles[i].real());            // 11 + 5*i
      message.data.push_back(analysis.poles[i].imag());
      message.data.push_back(analysis.magnitudes[i]);
      message.data.push_back(analysis.damping_ratios[i]);
      message.data.push_back(analysis.damped_frequencies_hz[i]);
    }

    (void)model;
    pole_pub_->publish(message);
  }

  std::string controller_node_name_;
  std::string pole_topic_;
  double monitor_period_s_{0.5};
  std::vector<std::string> parameter_names_;

  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr parameter_client_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pole_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool request_in_flight_{false};

  bool have_last_result_{false};
  double last_max_abs_{0.0};
  double last_gain_scale_{0.0};
  RowVector4 last_k_{RowVector4::Zero()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MicroLqrPoleMonitor>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("micro_lqr_pole_monitor"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
