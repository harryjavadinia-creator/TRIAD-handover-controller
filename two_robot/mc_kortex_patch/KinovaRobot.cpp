#include "KinovaRobot.h"
#include <csignal>
#include <exception>
#include <Eigen/src/Core/Matrix.h>
#include <mc_rtc/DataStore.h>
#include <mc_rbdyn/ForceSensor.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
// the block of 329 and 737 is chqnged but the real version is preserved as comments
namespace mc_kinova {

namespace
{

// CALL_PHYSICAL_ROBOT_B_FIXED_Q_OVERRIDE_V2
// CALL_PHYSICAL_ROBOT_B_SYNCED_PRESENTATION_V3
//
// Physical Robot B follows the normal dual-controller output during the
// existing preposition. After that reference has moved and then remained
// stationary, the next reference-motion onset is the existing Pure-X
// presentation start. At that instant we capture the real Robot-B joints and
// synchronously interpolate them to the requested final configuration.
//
// Robot A, virtual Robot B, virtual object, Pure-X timing and the FSM are not
// modified.
constexpr double kCallPhysicalRobotBTargetDeg[7] = {
    7.60, 40.41, 166.25, 319.26, 17.70, 350.52, 80.22};

constexpr double kCallPureXDistanceM = 0.370;
constexpr double kCallPureXSpeedMPerS = 0.080;
constexpr double kCallPureXDecelerationDurationS = 0.850;
constexpr double kCallPureXConstantDurationS =
    (kCallPureXDistanceM
     - 0.5 * kCallPureXSpeedMPerS
               * kCallPureXDecelerationDurationS)
    / kCallPureXSpeedMPerS;
constexpr double kCallPureXTotalDurationS =
    kCallPureXConstantDurationS
    + kCallPureXDecelerationDurationS;

bool callPhysicalRobotBFixedQEnabled()
{
  static const bool enabled = []() {
    const char * value =
        std::getenv("CALL_PHYSICAL_ROBOT_B_FIXED_JOINTS");
    return value && value[0] == '1' && value[1] == '\0';
  }();
  return enabled;
}

double callWrapDegrees360(double value)
{
  value = std::fmod(value, 360.0);
  if(value < 0.0) { value += 360.0; }
  return value;
}

double callShortestDegreeError(double target, double current)
{
  return std::remainder(target - current, 360.0);
}

double callPureXProgress(double elapsed)
{
  if(elapsed <= 0.0) { return 0.0; }

  if(elapsed < kCallPureXConstantDurationS)
  {
    return std::min(
        1.0,
        kCallPureXSpeedMPerS * elapsed / kCallPureXDistanceM);
  }

  if(elapsed < kCallPureXTotalDurationS)
  {
    const double decelerationElapsed =
        elapsed - kCallPureXConstantDurationS;
    const double deceleration =
        kCallPureXSpeedMPerS
        / kCallPureXDecelerationDurationS;
    const double distance =
        kCallPureXSpeedMPerS * kCallPureXConstantDurationS
        + kCallPureXSpeedMPerS * decelerationElapsed
        - 0.5 * deceleration
                  * decelerationElapsed
                  * decelerationElapsed;
    return std::min(1.0, distance / kCallPureXDistanceM);
  }

  return 1.0;
}

// CALL_PHYSICAL_ROBOT_B_EXACT_EVENT_SYNC_V4
volatile std::sig_atomic_t g_callPhysicalRobotBPresentationStartV4 = 0;

void callPhysicalRobotBPresentationSignalV4(int)
{
  g_callPhysicalRobotBPresentationStartV4 = 1;
}


constexpr const char * kGripperCloseKey =
    "HandoverInterceptionController::gripperClose";
constexpr const char * kGripperOpenPercentKey =
    "HandoverInterceptionController::gripperOpenPercent";
constexpr const char * kGripperClosePercentKey =
    "HandoverInterceptionController::gripperClosePercent";
constexpr const char * kGripperMaxPercentKey =
    "HandoverInterceptionController::gripperMaxPercent";
constexpr const char * kGripperCommandEnabledKey =
    "HandoverInterceptionController::gripperCommandEnabled";
constexpr const char * kGripperMeasuredPercentKey =
    "HandoverInterceptionController::gripperMeasuredPercent";
constexpr const char * kGripperMeasuredVelocityPercentKey =
    "HandoverInterceptionController::gripperMeasuredVelocityPercent";
constexpr const char * kGripperFeedbackValidKey =
    "HandoverInterceptionController::gripperFeedbackValid";
constexpr const char * kGripperFeedbackSequenceKey =
    "HandoverInterceptionController::gripperFeedbackSequence";
constexpr const char * kGripperBridgeCommandSeenKey =
    "HandoverInterceptionController::gripperBridgeCommandSeen";
constexpr const char * kGripperBridgeOpenPercentKey =
    "HandoverInterceptionController::gripperBridgeOpenPercent";
constexpr const char * kGripperBridgeClosePercentKey =
    "HandoverInterceptionController::gripperBridgeClosePercent";
constexpr const char * kGripperBridgeMaxPercentKey =
    "HandoverInterceptionController::gripperBridgeMaxPercent";

// Temporary compatibility with the older controller name. V6.4 always
// prefers the HandoverInterceptionController keys and logs once if a legacy
// key is the only source available.
constexpr const char * kLegacyGripperCloseKey =
    "HandoverThesisController::gripperClose";
constexpr const char * kLegacyGripperMaxPercentKey =
    "HandoverThesisController::gripperMaxPercent";

constexpr const char * kPassiveWrenchSensorName = "EEForceSensor";
constexpr const char * kPassiveWrenchParentBody = "gen3_end_effector_link";

void ensurePassiveWrenchSensor(mc_rbdyn::Robot & robot,
                               const char * collectionName)
{
  if(robot.hasForceSensor(kPassiveWrenchSensorName)) { return; }

  robot.addForceSensor(mc_rbdyn::ForceSensor{
      kPassiveWrenchSensorName,
      kPassiveWrenchParentBody,
      sva::PTransformd::Identity()});

  mc_rtc::log::success(
      "[PhysicalWrenchBridge][{}] registered sensor={} parent={} collection={} passiveOnly=true",
      robot.name(), kPassiveWrenchSensorName, kPassiveWrenchParentBody,
      collectionName);
}
}

KinovaRobot::KinovaRobot(const std::string &name, const std::string &ip_address,
                         const std::string &username = "admin",
                         const std::string &password = "admin")
    : m_name(name), m_ip_address(ip_address), m_port(10000),
      m_port_real_time(10001), m_username(username), m_password(password),
      stop_controller(false) {
  m_dt = 0;
  t_plot = 0.0;
  m_control_id = 0;
  m_prev_control_id = 0;
  m_use_filtered_velocities = false;
  m_velocity_filter_ratio = 0.0;
  m_router = nullptr;
  m_router_real_time = nullptr;
  m_transport = nullptr;
  m_transport_real_time = nullptr;
  m_session_manager = nullptr;
  m_session_manager_real_time = nullptr;
  m_base = nullptr;
  m_base_cyclic = nullptr;
  m_device_manager = nullptr;
  m_actuator_config = nullptr;
  m_model_has_robotiq_joints = false;
  m_physical_gripper_present = false;
  m_gripper_motor_command = nullptr;
  gripper_position = 0.0f;
  gripper_velocity = 0.0f;
  m_state = k_api::BaseCyclic::Feedback();
  m_control_mode = k_api::ActuatorConfig::ControlMode::POSITION;
  m_control_mode_id = 0;
  m_prev_control_mode_id = 0;
  m_torque_control_type = mc_kinova::TorqueControlType::Default;
}

KinovaRobot::~KinovaRobot() {
  // CALL_MCKORTEX_SAFE_SHUTDOWN_V2_2
  //
  // A destructor must never allow a Kortex session timeout to escape.
  // CloseSession may throw when a robot has already released the session or
  // when teardown acknowledgement is late. Such an exception previously
  // called std::terminate after a successful --init-only preflight.
  const auto closeSessionSafely =
      [this](k_api::SessionManager * manager, const char * channel)
      {
        if(!manager) { return; }
        try
        {
          manager->CloseSession();
        }
        catch(const std::exception & exception)
        {
          mc_rtc::log::warning(
              "[mc_kortex][{}] ignoring {} CloseSession exception during "
              "shutdown: {}",
              m_name,
              channel,
              exception.what());
        }
        catch(...)
        {
          mc_rtc::log::warning(
              "[mc_kortex][{}] ignoring unknown {} CloseSession exception "
              "during shutdown",
              m_name,
              channel);
        }
      };

  const auto deactivateRouterSafely =
      [this](k_api::RouterClient * router, const char * channel)
      {
        if(!router) { return; }
        try
        {
          router->SetActivationStatus(false);
        }
        catch(const std::exception & exception)
        {
          mc_rtc::log::warning(
              "[mc_kortex][{}] ignoring {} router-deactivation exception "
              "during shutdown: {}",
              m_name,
              channel,
              exception.what());
        }
        catch(...)
        {
          mc_rtc::log::warning(
              "[mc_kortex][{}] ignoring unknown {} router-deactivation "
              "exception during shutdown",
              m_name,
              channel);
        }
      };

  const auto disconnectTcpSafely =
      [this](k_api::TransportClientTcp * transport)
      {
        if(!transport) { return; }
        try
        {
          transport->disconnect();
        }
        catch(const std::exception & exception)
        {
          mc_rtc::log::warning(
              "[mc_kortex][{}] ignoring TCP transport-disconnect exception "
              "during shutdown: {}",
              m_name,
              exception.what());
        }
        catch(...)
        {
          mc_rtc::log::warning(
              "[mc_kortex][{}] ignoring unknown TCP transport-disconnect "
              "exception during shutdown",
              m_name);
        }
      };

  const auto disconnectUdpSafely =
      [this](k_api::TransportClientUdp * transport)
      {
        if(!transport) { return; }
        try
        {
          transport->disconnect();
        }
        catch(const std::exception & exception)
        {
          mc_rtc::log::warning(
              "[mc_kortex][{}] ignoring UDP transport-disconnect exception "
              "during shutdown: {}",
              m_name,
              exception.what());
        }
        catch(...)
        {
          mc_rtc::log::warning(
              "[mc_kortex][{}] ignoring unknown UDP transport-disconnect "
              "exception during shutdown",
              m_name);
        }
      };

  closeSessionSafely(m_session_manager, "TCP");
  closeSessionSafely(m_session_manager_real_time, "UDP");

  deactivateRouterSafely(m_router, "TCP");
  disconnectTcpSafely(m_transport);
  deactivateRouterSafely(m_router_real_time, "UDP");
  disconnectUdpSafely(m_transport_real_time);

  // Destroy API clients after all best-effort network teardown.
  delete m_actuator_config;
  delete m_device_manager;
  delete m_session_manager_real_time;
  delete m_session_manager;
  delete m_base_cyclic;
  delete m_base;
  delete m_router_real_time;
  delete m_router;
  delete m_transport_real_time;
  delete m_transport;
}

// ==================== Getter ==================== //

std::vector<double> KinovaRobot::getJointPosition() {
  std::vector<double> q(m_actuator_count);
  for (auto actuator : m_state.actuators())
    q[jointIdFromCommandID(actuator.command_id())] = actuator.position();
  return q;
}

std::string KinovaRobot::getName(void) { return m_name; }

bool KinovaRobot::isPrimaryRobot(
    mc_control::MCGlobalController & gc) const
{
  return m_name == gc.controller().robot().name();
}

std::string KinovaRobot::scopedHandoverKey(const std::string & leaf) const
{
  return "HandoverInterceptionController::" + m_name + "::" + leaf;
}

std::string KinovaRobot::scopedRuntimeKey(const std::string & leaf) const
{
  return "mc_kortex::" + m_name + "::" + leaf;
}

std::string KinovaRobot::logKey(
    mc_control::MCGlobalController & gc,
    const std::string & base) const
{
  return isPrimaryRobot(gc) ? base : (m_name + "_" + base);
}

// ==================== Setter ==================== //

void KinovaRobot::setLowServoingMode() {
  // Ignore if already in low level servoing mode
  // if(m_servoing_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
  // return;

  auto servoingMode = k_api::Base::ServoingModeInformation();

  servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  m_base->SetServoingMode(servoingMode);
  m_servoing_mode = k_api::Base::ServoingMode::LOW_LEVEL_SERVOING;
}

void KinovaRobot::setSingleServoingMode() {
  // Ignore if already in "high" level servoing mode
  // if(m_servoing_mode == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
  // return;

  auto servoingMode = k_api::Base::ServoingModeInformation();

  servoingMode.set_servoing_mode(
      k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  m_base->SetServoingMode(servoingMode);
  m_servoing_mode = k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING;
}

void KinovaRobot::setCustomTorque(mc_rtc::Configuration &torque_config) {
  if (torque_config.has("friction_compensation")) {
    if (torque_config("friction_compensation").has("stiction")) {
      m_stiction_values = torque_config("friction_compensation")("stiction");
      if (not(m_stiction_values.size() == m_actuator_count))
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[MC_KORTEX] for {} robot, value for \"compensation_values\" key "
            "does not match actuators count.\nActuators count = ",
            m_name, m_actuator_count);
    } else {
      m_stiction_values = {3.0, 3.0, 3.0, 3.0, 1.25, 1.25, 1.25};
    }
    if (torque_config("friction_compensation").has("coulomb")) {
      m_friction_values = torque_config("friction_compensation")("coulomb");
      if (not(m_friction_values.size() == m_actuator_count))
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[MC_KORTEX] for {} robot, value for \"compensation_values\" key "
            "does not match actuators count.\nActuators count = ",
            m_name, m_actuator_count);
    } else {
      m_friction_values = {3.0, 3.0, 3.0, 3.0, 1.25, 1.25, 1.25};
    }
    if (torque_config("friction_compensation").has("viscous")) {
      m_viscous_values = torque_config("friction_compensation")("viscous");
      if (not(m_viscous_values.size() == m_actuator_count))
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[MC_KORTEX] for {} robot, value for \"compensation_values\" key "
            "does not match actuators count.\nActuators count = ",
            m_name, m_actuator_count);
    } else {
      m_viscous_values = {2.416, 2.416, 2.416, 2.416, 1.1, 1.1, 1.1};
    }

    if (torque_config("friction_compensation").has("velocity_threshold")) {
      m_friction_vel_threshold =
          torque_config("friction_compensation")("velocity_threshold");
    } else {
      m_friction_vel_threshold = 0.01;
    }

    if (torque_config("friction_compensation").has("acceleration_threshold")) {
      m_friction_accel_threshold =
          torque_config("friction_compensation")("acceleration_threshold");
    } else {
      m_friction_accel_threshold = 100;
    }

    if (torque_config.has("lambda")) {
      m_lambda = torque_config("lambda");
    } else {
      m_lambda = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    }
  } else {
    m_friction_vel_threshold = 0.01;
    m_friction_accel_threshold = 100;
    m_friction_values = {3.0, 3.0, 3.0, 3.0, 1.25, 1.25, 1.25};
    m_viscous_values = {2.416, 2.416, 2.416, 2.416, 1.1, 1.1, 1.1};
  }

  if (torque_config.has("integral_term")) {
    if (torque_config("integral_term").has("theta")) {
      m_integral_slow_theta = torque_config("integral_term")("theta");
    } else {
      m_integral_slow_theta = 0.1;
    }

    if (torque_config("integral_term").has("gain")) {
      m_integral_slow_gain = torque_config("integral_term")("gain");
    } else {
      m_integral_slow_gain = 1e-3;
    }
  } else {
    m_integral_slow_theta = 0.1;
    m_integral_slow_gain = 1e-3;
  }

  mc_rtc::log::info(
      "[mc_kortex] {} robot is using custom torque control with parameters:",
      m_name);
}

void KinovaRobot::setControlMode(std::string mode) {
  if (mode.compare("Position") == 0) {
    if (m_control_mode == k_api::ActuatorConfig::ControlMode::POSITION)
      return;

    // mc_rtc::log::info("[mc_kortex] Using position control");
    m_control_mode = k_api::ActuatorConfig::ControlMode::POSITION;
    m_control_mode_id++;
    return;
  }
  if (mode.compare("Velocity") == 0) {
    if (m_control_mode == k_api::ActuatorConfig::ControlMode::VELOCITY)
      return;

    // mc_rtc::log::info("[mc_kortex] Using velocity control");
    m_control_mode = k_api::ActuatorConfig::ControlMode::VELOCITY;
    m_control_mode_id++;
    return;
  }
  if (mode.compare("Torque") == 0) {
    switch (m_torque_control_type) {
    case mc_kinova::TorqueControlType::Default:
    case mc_kinova::TorqueControlType::Feedforward:
    case mc_kinova::TorqueControlType::Custom:
      if (m_control_mode == k_api::ActuatorConfig::ControlMode::CURRENT)
        return;

      // mc_rtc::log::info("[mc_kortex] Using torque control");
      initFiltersBuffers();
      m_control_mode = k_api::ActuatorConfig::ControlMode::CURRENT;
      m_control_mode_id++;
      return;
      break;
    }
  }
}

void KinovaRobot::setTorqueMode(std::string mode) {
  if (mode.compare("Default") == 0) {
    m_torque_control_type = mc_kinova::TorqueControlType::Default;
  } else if (mode.compare("Feedforward") == 0) {
    m_torque_control_type = mc_kinova::TorqueControlType::Feedforward;
  } else if (mode.compare("Custom") == 0) {
    m_torque_control_type = mc_kinova::TorqueControlType::Custom;
  } else {
    mc_rtc::log::error("[mc_kortex] Unknown torque control type: {}", mode);
  }
}

// ==================== Public functions ==================== //

void KinovaRobot::init(mc_control::MCGlobalController &gc,
                       mc_rtc::Configuration &kortexConfig,
                       bool enable_control) {
  mc_rtc::log::info("[mc_kortex] Initializing connection to the robot at {}:{}",
                    m_ip_address, m_port);

  auto error_callback = [](k_api::KError err) {
    mc_rtc::log::error("_________ callback error _________ {}", err.toString());
  };

  // Initiate connection
  m_transport = new k_api::TransportClientTcp();
  m_router = new k_api::RouterClient(m_transport, error_callback);
  m_transport->connect(m_ip_address, m_port);

  m_transport_real_time = new k_api::TransportClientUdp();
  m_router_real_time =
      new k_api::RouterClient(m_transport_real_time, error_callback);
  m_transport_real_time->connect(m_ip_address, m_port_real_time);

  // Set session data connection information
  auto createSessionInfo = k_api::Session::CreateSessionInfo();
  createSessionInfo.set_username(m_username);
  createSessionInfo.set_password(m_password);
  createSessionInfo.set_session_inactivity_timeout(60000);   // (milliseconds)
  createSessionInfo.set_connection_inactivity_timeout(2000); // (milliseconds)

  // Session manager service wrapper
  m_session_manager = new k_api::SessionManager(m_router);
  m_session_manager->CreateSession(createSessionInfo);
  m_session_manager_real_time = new k_api::SessionManager(m_router_real_time);
  m_session_manager_real_time->CreateSession(createSessionInfo);

  // Create services
  m_device_manager = new k_api::DeviceManager::DeviceManagerClient(m_router);
  m_actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(m_router);
  m_base = new k_api::Base::BaseClient(m_router);
  m_base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(m_router_real_time);

  // Read actuators count
  setSingleServoingMode();
  m_actuator_count = m_base->GetActuatorCount().count();

  auto & modelRobot = gc.controller().robots().robot(m_name);
  const auto & rjo = modelRobot.refJointOrder();
  const std::vector<std::string> gen3ArmJoints = {
      "gen3_joint_1", "gen3_joint_2", "gen3_joint_3", "gen3_joint_4",
      "gen3_joint_5", "gen3_joint_6", "gen3_joint_7"};
  const std::vector<std::string> plainArmJoints = {
      "joint_1", "joint_2", "joint_3", "joint_4",
      "joint_5", "joint_6", "joint_7"};
  const auto containsAll = [&rjo](const std::vector<std::string> & names)
  {
    return std::all_of(names.begin(), names.end(),
                       [&rjo](const std::string & name)
                       {
                         return std::find(rjo.begin(), rjo.end(), name) != rjo.end();
                       });
  };
  if(m_actuator_count != 7)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_kortex] unsupported Gen3 hardware actuator count: {}",
        m_actuator_count);
  }
  if(containsAll(gen3ArmJoints))
  {
    m_arm_joint_names = gen3ArmJoints;
  }
  else if(containsAll(plainArmJoints))
  {
    m_arm_joint_names = plainArmJoints;
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_kortex] robot={} is neither the gen3_joint_1..7 model nor the plain joint_1..7 Kinova model",
        m_name);
  }
  const std::vector<std::string> robotiqJoints = {
      "gen3_robotiq_85_left_knuckle_joint",
      "gen3_robotiq_85_right_knuckle_joint",
      "gen3_robotiq_85_left_inner_knuckle_joint",
      "gen3_robotiq_85_right_inner_knuckle_joint",
      "gen3_robotiq_85_left_finger_tip_joint",
      "gen3_robotiq_85_right_finger_tip_joint"};
  m_model_has_robotiq_joints = std::all_of(
      robotiqJoints.begin(), robotiqJoints.end(),
      [&rjo](const std::string & name)
      {
        return std::find(rjo.begin(), rjo.end(), name) != rjo.end();
      });

  m_physical_gripper_present = isPrimaryRobot(gc);
  if(kortexConfig.has(m_name))
  {
    const auto robotConfig = kortexConfig(m_name);
    if(robotConfig.has("physical_gripper"))
    {
      robotConfig("physical_gripper", m_physical_gripper_present);
    }
  }

  mc_rtc::log::info(
      "[mc_kortex] {} hardwareActuators={} modelReferenceJoints={} armJointConvention={} sharedRobotiqModel={} physicalGripper={}",
      m_name, m_actuator_count, rjo.size(),
      m_arm_joint_names.front().find("gen3_") == 0 ? "gen3_joint" : "joint",
      m_model_has_robotiq_joints, m_physical_gripper_present);

  m_filter_command.assign(m_actuator_count, 0.0);
  m_filter_command_w_gain.assign(m_actuator_count, 0.0);
  m_current_command.setZero(m_actuator_count);
  m_current_measurement.setZero(m_actuator_count);
  m_torque_from_current_measurement.setZero(m_actuator_count);
  m_torque_measure_corrected.assign(m_actuator_count, 0.0);
  m_tau_sensor.setZero(m_actuator_count);
  m_torque_error.assign(m_actuator_count, 0.0);
  m_prev_torque_error.assign(m_actuator_count, 0.0);
  m_integral_slow_filter.assign(m_actuator_count, 0.0);
  m_integral_slow_filter_w_gain.assign(m_actuator_count, 0.0);
  m_integral_slow_bound.assign(m_actuator_count, 0.0);
  m_friction_compensation_mode.assign(m_actuator_count, 0.0);
  m_current_friction_compensation.assign(m_actuator_count, 0.0);
  m_jac_transpose_f.assign(m_actuator_count, 0.0);
  m_offsets.assign(m_actuator_count, 0.0);
  tau_fric.setZero(m_actuator_count);
  m_lambda.assign(m_actuator_count, 0.0);
  m_integral_slow_theta = 1.0;
  m_integral_slow_gain = 1e-2;

  // A preflight connection is sensor-only: do not alter actuator control modes.
  if(enable_control)
  {
    auto controle_mode = k_api::ActuatorConfig::ControlModeInformation();
    controle_mode.set_control_mode(m_control_mode);
    for (int i = 0; i < m_actuator_count; i++)
      m_actuator_config->SetControlMode(controle_mode, i + 1);
  }

  m_allow_init_posture_gui =
      kortexConfig("allow_init_posture_gui", true);

  // Init pose if desired
  if (kortexConfig.has("init_posture")) {
    if (kortexConfig("init_posture").has("posture")) {
      m_init_posture = kortexConfig("init_posture")("posture");
      if (not(m_init_posture.size() == m_actuator_count))
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[MC_KORTEX] for {} robot, value for \"posture\" key does not "
            "match actuators count.\nActuators count = ",
            m_name, m_actuator_count);
    }

    if (enable_control && kortexConfig("init_posture")("on_startup", false)) {
      moveToInitPosition();
    }
  } else {
    m_init_posture.resize(m_actuator_count);

    auto joints_feedback = m_base->GetMeasuredJointAngles();
    for (size_t i = 0; i < m_actuator_count; i++) {
      auto joint_feedback = joints_feedback.joint_angles(i);
      m_init_posture[joint_feedback.joint_identifier() - 1] =
          joint_feedback.value();
    }
  }

  // if (gripper_enabled) {
  //   gripper_position = 0.0;
  //   m_base_command.mutable_interconnect()->mutable_command_id()->set_identifier(
  //       0);
  //   m_gripper_motor_command = m_base_command.mutable_interconnect()
  //                                 ->mutable_gripper_command()
  //                                 ->add_motor_cmd();
  //   m_gripper_motor_command->set_position(0.0);
  //   m_gripper_motor_command->set_velocity(0.0);
  //   m_gripper_motor_command->set_force(100.0);
  // }
  
  /*
   * Direct Robotiq command setup. The shared simulation/hardware RobotModule
   * retains all six Robotiq joints for geometry, but hardware still receives
   * one physical motor command through the Kortex interconnect.
   */
  if(enable_control && m_physical_gripper_present)
  {
    m_base_command.mutable_interconnect()->mutable_command_id()->set_identifier(0);
    m_gripper_motor_command = m_base_command.mutable_interconnect()
                                  ->mutable_gripper_command()
                                  ->add_motor_cmd();
    m_gripper_motor_command->set_position(0.0f);
    m_gripper_motor_command->set_velocity(0.0f);
    m_gripper_motor_command->set_force(30.0f);
    mc_rtc::log::warning("[mc_kortex][{}] direct Robotiq motor command initialized", m_name);
  }
  else if(enable_control)
  {
    mc_rtc::log::info("[mc_kortex][{}] plain-arm mode: no gripper command channel created", m_name);
  }

  // Initialize state
  updateState();
  updateSensors(gc);

  mc_rtc::log::info(
      "[mc_kortex gripper PREFLIGHT] feedbackValid={} measured={:.2f}% velocity={:.2f}%/s commandEnabled={} commandSeen={}",
      m_gripper_feedback_valid.load(),
      m_gripper_measured_percent.load(),
      m_gripper_measured_velocity_percent.load(),
      m_handover_gripper_command_enabled.load(),
      m_handover_gripper_command_valid.load());

  // Velocity filtering init
  m_use_filtered_velocities = kortexConfig.has("filter_velocity");
  if (m_use_filtered_velocities) {
    m_velocity_filter_ratio = kortexConfig("filter_velocity")("ratio", 0.0);
    mc_rtc::log::info(
        "[mc_kortex] Filtering velocities for {} robot with {} ratio", m_name,
        m_velocity_filter_ratio);
  }
  m_filtered_velocities.assign(m_actuator_count, 0.0);

  // Custom torque control init
  if (kortexConfig.has("torque_control")) {
    auto torqueConfig = kortexConfig("torque_control");
    if (!torqueConfig.has("mode"))
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_kortex] For {} robot, \"torque_control\" key found in config "
          "file but \"mode\" key is missing.",
          m_name);

    std::string controle_mode = torqueConfig("mode");
    if (controle_mode.compare("feedforward") == 0) {
      m_torque_control_type = mc_kinova::TorqueControlType::Feedforward;
      mc_rtc::log::info(
          "[mc_kortex] Using feedforward only for torque control");
    } else if (controle_mode.compare("custom") == 0) {
      m_torque_control_type = mc_kinova::TorqueControlType::Custom;
      setCustomTorque(torqueConfig);
      mc_rtc::log::info("[mc_kortex] Using custom control for torque control");
    } else {
      m_torque_control_type = mc_kinova::TorqueControlType::Default;
      mc_rtc::log::info(
          "[mc_kortex] Using Kinova's default control for torque control");
    }
  }

  if(enable_control)
  {
    auto & runtimeDs = gc.controller().datastore();
    runtimeDs.make_call(
        scopedRuntimeKey("set_friction_compensation_stiction"),
        [this](std::vector<double> v) { m_stiction_values = v; });
    runtimeDs.make_call(
        scopedRuntimeKey("set_friction_compensation_coulomb"),
        [this](std::vector<double> v) { m_friction_values = v; });
    runtimeDs.make_call(
        scopedRuntimeKey("set_friction_compensation_viscous"),
        [this](std::vector<double> v) { m_viscous_values = v; });
    runtimeDs.make_call(
        scopedRuntimeKey("set_integral_term_gain"),
        [this](double g) { m_integral_slow_gain = g; });

    // Preserve the historical single-robot callback names for Robot A only.
    if(isPrimaryRobot(gc))
    {
      runtimeDs.make_call(
          "set_kinova_friction_compensation_stiction",
          [this](std::vector<double> v) { m_stiction_values = v; });
      runtimeDs.make_call(
          "set_kinova_friction_compensation_coulomb",
          [this](std::vector<double> v) { m_friction_values = v; });
      runtimeDs.make_call(
          "set_kinova_friction_compensation_viscous",
          [this](std::vector<double> v) { m_viscous_values = v; });
      runtimeDs.make_call(
          "set_kinova_integral_term_gain",
          [this](double g) { m_integral_slow_gain = g; });
    }

    // Command-side buffers, callbacks and GUI elements are deliberately absent
    // from the headless no-motion preflight path.
    auto robot = &gc.robots().robot(m_name);
    Eigen::VectorXd tu = rbd::paramToVector(robot->mb(), robot->tu());
    for (int i = 0; i < m_actuator_count; i++)
    {
      m_integral_slow_bound[i] = 0.05 * tu[i]; // 5% of torque limit
      m_base_command.add_actuators()->set_position(
          m_state.actuators(i).position());
    }

    addGui(gc);
  }

  mc_rtc::log::success("[mc_kortex] Connected succesfuly to robot at {}:{}",
                       m_ip_address, m_port);
}

void KinovaRobot::addLogEntry(mc_control::MCGlobalController &gc) {
  auto & logger = gc.controller().logger();
  logger.addLogEntry(logKey(gc, "kortex_LoopPerf"),
                     [this]() { return m_dt; });
  if (m_torque_control_type == mc_kinova::TorqueControlType::Feedforward) {
    logger.addLogEntry(logKey(gc, "kortex_commanded_current"),
                       [this]() { return m_current_command; });
    logger.addLogEntry(logKey(gc, "kortex_current_measurement"),
                       [this]() { return m_torque_from_current_measurement; });
  }

  if (m_torque_control_type == mc_kinova::TorqueControlType::Custom) {
    logger.addLogEntry(logKey(gc, "tauInCorrected"),
                       [this]() { return m_torque_measure_corrected; });
    logger.addLogEntry(logKey(gc, "kortex_friction_velocity_threshold"),
                       [this]() { return m_friction_vel_threshold; });
    logger.addLogEntry(logKey(gc, "kortex_friction_acceleration_threshold"),
                       [this]() { return m_friction_accel_threshold; });
    logger.addLogEntry(logKey(gc, "kortex_friction_coulomb_values"),
                       [this]() { return m_friction_values; });
    logger.addLogEntry(logKey(gc, "kortex_friction_viscous_values"),
                       [this]() { return m_viscous_values; });
    logger.addLogEntry(logKey(gc, "kortex_friction_mode"),
                       [this]() { return m_friction_compensation_mode; });
    logger.addLogEntry(logKey(gc, "torque friction"),
                       [this]() { return tau_fric; });
    logger.addLogEntry(logKey(gc, "kortex_friction_current_compensation"),
                       [this]() { return m_current_friction_compensation; });
    logger.addLogEntry(logKey(gc, "kortex_torque_error"),
                       [this]() { return m_torque_error; });
    logger.addLogEntry(logKey(gc, "kortex_integral_value"),
                       [this]() { return m_integral_slow_filter; });
    logger.addLogEntry(logKey(gc, "kortex_integral_gains"),
                       [this]() { return m_integral_slow_gain; });
    logger.addLogEntry(logKey(gc, "kortex_integral_theta"),
                       [this]() { return m_integral_slow_theta; });
    logger.addLogEntry(logKey(gc, "kortex_integral_w_gain"),
                       [this]() { return m_integral_slow_filter_w_gain; });
    logger.addLogEntry(logKey(gc, "kortex_transfer_function"),
                       [this]() { return m_filter_command; });
    logger.addLogEntry(logKey(gc, "kortex_transfer_w_gain"),
                       [this]() { return m_filter_command_w_gain; });
    logger.addLogEntry(logKey(gc, "kortex_current_command"),
                       [this]() { return m_current_command; });
    logger.addLogEntry(logKey(gc, "kortex_current_measurement"),
                       [this]() { return m_torque_from_current_measurement; });
    logger.addLogEntry(logKey(gc, "kortex_jac_transpose_F"),
                       [this]() { return m_jac_transpose_f; });
    logger.addLogEntry(logKey(gc, "kortex_posture_task_offset"),
                       [this]() { return m_offsets; });
    logger.addLogEntry(logKey(gc, "kortex_lambda"),
                       [this]() { return m_lambda; });
  }
}

void KinovaRobot::removeLogEntry(mc_control::MCGlobalController &gc) {
  auto & logger = gc.controller().logger();
  const auto remove = [this, &gc, &logger](const std::string & key)
  {
    logger.removeLogEntry(logKey(gc, key));
  };

  // Remove exactly the entries that addLogEntry installed for this mode.
  remove("kortex_LoopPerf");

  if(m_torque_control_type == mc_kinova::TorqueControlType::Feedforward)
  {
    remove("kortex_commanded_current");
    remove("kortex_current_measurement");
  }

  if(m_torque_control_type == mc_kinova::TorqueControlType::Custom)
  {
    remove("tauInCorrected");
    remove("kortex_friction_velocity_threshold");
    remove("kortex_friction_acceleration_threshold");
    remove("kortex_friction_coulomb_values");
    remove("kortex_friction_viscous_values");
    remove("kortex_friction_mode");
    remove("torque friction");
    remove("kortex_friction_current_compensation");
    remove("kortex_torque_error");
    remove("kortex_integral_value");
    remove("kortex_integral_gains");
    remove("kortex_integral_theta");
    remove("kortex_integral_w_gain");
    remove("kortex_transfer_function");
    remove("kortex_transfer_w_gain");
    remove("kortex_current_command");
    remove("kortex_current_measurement");
    remove("kortex_jac_transpose_F");
    remove("kortex_posture_task_offset");
    remove("kortex_lambda");
  }
}

void KinovaRobot::cleanupControllerInterfaces(
    mc_control::MCGlobalController & gc)
{
  removeLogEntry(gc);
  removeGui(gc);
}

void KinovaRobot::updateState() {
  std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
  m_state = m_base_cyclic->RefreshFeedback();
}

void KinovaRobot::updateState(bool &running) {
  m_base_cyclic->RefreshFeedback_callback(
      [&, this](const Kinova::Api::Error &err,
                const k_api::BaseCyclic::Feedback data) {
        updateState(data);
        checkBaseFaultBanks(data.base().fault_bank_a(),
                            data.base().fault_bank_b());
        // checkActuatorsFaultBanks(data);
        if (err.error_code() != k_api::ErrorCodes::ERROR_NONE) {
          printError(err);
          running = false;
        }
      });
}

void KinovaRobot::updateState(const k_api::BaseCyclic::Feedback data) {
  std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
  m_state = data;
}

void KinovaRobot::torqueFrictionComputation(
    mc_rbdyn::Robot &robot, k_api::BaseCyclic::Feedback m_state_local,
    size_t joint_idx) {
      if (joint_idx >= m_friction_values.size() || joint_idx >= m_viscous_values.size()) {
        return;
      }
  double velocity = mc_rtc::constants::toRad(
      m_state_local.mutable_actuators(joint_idx)->velocity());
  double friction_torque = 0.0;
  auto qdd_r = m_command.alphaD[robot.jointIndexByName(m_arm_joint_names.at(joint_idx))][0];

  // Friction compensation logic
  if (velocity > m_friction_vel_threshold) {
    friction_torque =
        m_friction_values[joint_idx] + m_viscous_values[joint_idx] * velocity;
  } else if (velocity < -m_friction_vel_threshold) {
    friction_torque =
        -m_friction_values[joint_idx] + m_viscous_values[joint_idx] * velocity;
  } else {
    if (qdd_r > m_friction_accel_threshold) {
      friction_torque = m_friction_values[joint_idx];
    } else if (qdd_r < -m_friction_accel_threshold) {
      friction_torque = -m_friction_values[joint_idx];
    }
  }
  tau_fric[joint_idx] = friction_torque;
}

double
KinovaRobot::currentTorqueControlLaw(mc_rbdyn::Robot &robot,
                                     k_api::BaseCyclic::Feedback m_state_local,
                                     double joint_idx) {

  double velocity = mc_rtc::constants::toRad(
      m_state_local.mutable_actuators(joint_idx)->velocity());
  double torque_measured = m_state_local.mutable_actuators(joint_idx)->torque();

  double torque_constant = (joint_idx > 3) ? 0.076 : 0.11;
  auto filter_input = m_filter_input_buffer[joint_idx];
  auto filter_output = m_filter_output_buffer[joint_idx];
  double friction_torque = 0.0;

  auto qdd_r = m_command.alphaD[robot.jointIndexByName(m_arm_joint_names.at(joint_idx))][0];

  double tau_desired =
      m_command.jointTorque[robot.jointIndexByName(m_arm_joint_names.at(joint_idx))][0];

  double rotor_inertia =
      robot.mb().joint(robot.jointIndexByName(m_arm_joint_names.at(joint_idx))).rotorInertia();

  double rotor_inertia_torque = rotor_inertia * GEAR_RATIO * GEAR_RATIO * qdd_r;

  double torque_error = tau_desired + torque_measured - rotor_inertia_torque;

  m_prev_torque_error[joint_idx] = m_torque_error[joint_idx];
  m_torque_error[joint_idx] = torque_error;

  // Friction compensation logic
  if (velocity > m_friction_vel_threshold) {
    m_friction_compensation_mode[joint_idx] = 2;
    friction_torque =
        m_friction_values[joint_idx] + m_viscous_values[joint_idx] * velocity;
  } else if (velocity < -m_friction_vel_threshold) {
    m_friction_compensation_mode[joint_idx] = -2;
    friction_torque =
        -m_friction_values[joint_idx] + m_viscous_values[joint_idx] * velocity;
  } else {
    if (qdd_r > m_friction_accel_threshold) {
      m_friction_compensation_mode[joint_idx] = 1;
      friction_torque = m_stiction_values[joint_idx];
    } else if (qdd_r < -m_friction_accel_threshold) {
      m_friction_compensation_mode[joint_idx] = -1;
      friction_torque = -m_stiction_values[joint_idx];
    }
  }

  m_integral_slow_filter[joint_idx] =
      exp(-(1e-3 / m_integral_slow_theta)) *
          (m_integral_slow_filter_w_gain[joint_idx] / m_integral_slow_gain) +
      (1 - exp(-(1e-3 / m_integral_slow_theta))) * torque_error;

  double integral_w_gain =
      m_integral_slow_gain * m_integral_slow_filter[joint_idx];

  m_current_friction_compensation[joint_idx] = friction_torque;

  m_integral_slow_filter_w_gain[joint_idx] =
      std::max(-m_integral_slow_bound[joint_idx],
               std::min(integral_w_gain, m_integral_slow_bound[joint_idx]));

  // Filtered command calculation
  m_filter_command[joint_idx] =
      1.975063 * filter_output[0] - 0.9751799 * filter_output[1] +
      0.02017482 * (torque_error)-0.03697504 * filter_input[0] +
      0.01691718 * filter_input[1];

  // Update filter buffers
  m_filter_input_buffer[joint_idx].push_front(torque_error);
  m_filter_output_buffer[joint_idx].push_front(m_filter_command[joint_idx]);
  m_filter_command_w_gain[joint_idx] = m_filter_command[joint_idx];
  // Current calculation
  double current =
      (m_lambda[joint_idx] * m_filter_command[joint_idx] + tau_desired +
       m_integral_slow_filter_w_gain[joint_idx] + friction_torque) /
      (GEAR_RATIO * torque_constant);

  m_current_command[joint_idx] = current * torque_constant * GEAR_RATIO;

  return current;
}

bool KinovaRobot::sendCommand(mc_rbdyn::Robot &robot, bool &running) {
  bool return_value = true;
  k_api::BaseCyclic::Feedback m_state_local;
  {
    std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
    m_state_local = m_state;
  }

  std::unique_lock<std::mutex> lock(m_update_control_mutex);
  if (m_control_id == m_prev_control_id)
    return false;

  auto lambda_fct = [&, this](const Kinova::Api::Error &err,
                              const k_api::BaseCyclic::Feedback data) {
    updateState(data);
    checkBaseFaultBanks(data.base().fault_bank_a(), data.base().fault_bank_b());
    if (err.error_code() != k_api::ErrorCodes::ERROR_NONE) {
      printError(err);
      running = false;
    }
  };

  for (size_t i = 0; i < m_actuator_count; i++) {
    const std::string & armJointName = m_arm_joint_names.at(i);
    const auto armJointIndex = robot.jointIndexByName(armJointName);
    double kt = (i > 3) ? 0.076 : 0.11;
    if (m_control_mode == k_api::ActuatorConfig::ControlMode::POSITION) {
      if(callPhysicalRobotBFixedQEnabled()
         && m_name == "kinova"
         && i < 7)
      {
        constexpr double kDt = 0.001;

        static thread_local bool signalHandlerInstalled = false;
        static thread_local bool presentationStarted = false;
        static thread_local bool terminalLogged = false;
        static thread_local size_t presentationCycles = 0;
        static thread_local double physicalStartDeg[7] = {0.0};
        static thread_local double physicalDeltaDeg[7] = {0.0};
        static thread_local double synchronizedProgress = 0.0;

        if(i == 0)
        {
          if(!signalHandlerInstalled)
          {
            std::signal(
                SIGUSR2,
                callPhysicalRobotBPresentationSignalV4);
            signalHandlerInstalled = true;

            mc_rtc::log::warning(
                "[mc_kortex PHYSICAL-B EXACT-EVENT V4] armed; "
                "following normal Robot-B output until DualGiver START");
          }

          if(!presentationStarted
             && g_callPhysicalRobotBPresentationStartV4 != 0)
          {
            presentationStarted = true;
            presentationCycles = 0;
            synchronizedProgress = 0.0;

            for(size_t joint = 0; joint < 7; ++joint)
            {
              physicalStartDeg[joint] =
                  m_state_local.mutable_actuators(joint)->position();
              physicalDeltaDeg[joint] =
                  callShortestDegreeError(
                      kCallPhysicalRobotBTargetDeg[joint],
                      physicalStartDeg[joint]);
            }

            mc_rtc::log::warning(
                "[mc_kortex PHYSICAL-B EXACT-EVENT V4] "
                "DualGiver START received; captured measured real start "
                "and beginning synchronized {:.3f}s motion to "
                "[7.60,40.41,166.25,319.26,17.70,350.52,80.22]deg",
                kCallPureXTotalDurationS);
          }

          if(presentationStarted)
          {
            const double elapsed =
                static_cast<double>(presentationCycles) * kDt;
            synchronizedProgress = callPureXProgress(elapsed);
            ++presentationCycles;

            if(presentationCycles == 1
               || presentationCycles % 1000 == 0)
            {
              mc_rtc::log::warning(
                  "[mc_kortex PHYSICAL-B EXACT-EVENT V4] "
                  "progress={:.3f} elapsed={:.3f}/{:.3f}s",
                  synchronizedProgress,
                  elapsed,
                  kCallPureXTotalDurationS);
            }

            if(synchronizedProgress >= 1.0 && !terminalLogged)
            {
              terminalLogged = true;
              mc_rtc::log::success(
                  "[mc_kortex PHYSICAL-B EXACT-EVENT V4] "
                  "final target command reached and held");
            }
          }
        }

        if(!presentationStarted)
        {
          m_base_command.mutable_actuators(i)->set_position(
              radToJointPose(i, m_command.q[armJointIndex][0]));
        }
        else
        {
          const double command = callWrapDegrees360(
              physicalStartDeg[i]
              + synchronizedProgress * physicalDeltaDeg[i]);
          m_base_command.mutable_actuators(i)->set_position(command);
        }

        m_base_command.mutable_actuators(i)->set_current_motor(
            m_state_local.mutable_actuators(i)->current_motor());
      }
      else
      {
        m_base_command.mutable_actuators(i)->set_position(
            radToJointPose(i, m_command.q[armJointIndex][0]));
        m_base_command.mutable_actuators(i)->set_current_motor(
            m_state_local.mutable_actuators(i)->current_motor());
      }
      continue;
    } else {
      m_base_command.mutable_actuators(i)->set_position(
          m_state_local.mutable_actuators(i)->position());
    }

    torqueFrictionComputation(robot, m_state_local, i);

    auto qdd_r = m_command.alphaD[armJointIndex][0];

    double tau_desired =
        m_command.jointTorque[armJointIndex][0];

    double rotor_inertia =
        robot.mb().joint(armJointIndex).rotorInertia();

    double rotor_inertia_torque =
        rotor_inertia * GEAR_RATIO * GEAR_RATIO * qdd_r;

    m_torque_measure_corrected[i] =
        -m_state_local.mutable_actuators(i)->torque() + rotor_inertia_torque;

    switch (m_torque_control_type) {
    case mc_kinova::TorqueControlType::Default:
      m_base_command.mutable_actuators(i)->set_torque_joint(
          m_command.jointTorque[armJointIndex][0] -
          rotor_inertia_torque);
      break;
    case mc_kinova::TorqueControlType::Feedforward:
      m_base_command.mutable_actuators(i)->set_current_motor(
          m_command.jointTorque[armJointIndex][0] /
          (GEAR_RATIO * kt));
      m_current_command(i) =
          m_command.jointTorque[armJointIndex][0] /
          (GEAR_RATIO * kt);
      break;
    case mc_kinova::TorqueControlType::Custom:
      m_base_command.mutable_actuators(i)->set_current_motor(
          currentTorqueControlLaw(robot, m_state_local, i));
      break;
    default:
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_kortex] wrong torque control type when trying to send command");
      break;
    }
    // std::cout << m_base_command.mutable_actuators(i)->position() << " " <<
    // m_state_local.mutable_actuators(i)->position() << " | ";
  }

  /*
   * V6.4 physical Robotiq command through BaseCyclic interconnect.
   *
   * Hardware commands are fail-closed at the bridge level: unless the
   * HandoverInterceptionController explicitly publishes both a valid command
   * and gripperCommandEnabled=true, the bridge commands the currently measured
   * aperture with zero velocity. Missing feedback also suppresses motion.
   */
  if(m_gripper_motor_command)
  {
    ++m_gripper_command_sequence;

    double close_cmd = m_handover_gripper_close.load();
    double open_percent = m_handover_gripper_open_percent.load();
    double close_percent = m_handover_gripper_close_percent.load();
    double max_percent = m_handover_gripper_max_percent.load();
    const bool command_enabled =
        m_handover_gripper_command_enabled.load()
        && m_handover_gripper_command_valid.load();
    const bool feedback_valid = m_gripper_feedback_valid.load();

    close_cmd = std::max(0.0, std::min(1.0, close_cmd));
    open_percent = std::max(0.0, std::min(99.0, open_percent));
    close_percent = std::max(
        open_percent + 1e-3, std::min(100.0, close_percent));
    max_percent = std::max(
        close_percent, std::min(100.0, max_percent));

    const float measured_gripper_position = static_cast<float>(
        m_gripper_measured_percent.load());
    float gripper_target = measured_gripper_position;
    float gripper_velocity_target = 0.0f;

    if(command_enabled && feedback_valid)
    {
      const double calibrated_target =
          open_percent + close_cmd * (close_percent - open_percent);
      gripper_target = static_cast<float>(
          std::min(max_percent, calibrated_target));
      gripper_velocity_target =
          std::fabs(gripper_target - measured_gripper_position) * 2.2f;
      gripper_velocity_target =
          std::max(0.0f, std::min(60.0f, gripper_velocity_target));
      m_missing_gripper_feedback_warned = false;
    }
    else if(command_enabled && !feedback_valid && !m_missing_gripper_feedback_warned)
    {
      m_missing_gripper_feedback_warned = true;
      mc_rtc::log::error(
          "[mc_kortex gripper][{}] physical command suppressed because Robotiq feedback is invalid",
          m_name);
    }

    m_base_command.mutable_interconnect()
        ->mutable_command_id()
        ->set_identifier(static_cast<uint32_t>(m_gripper_command_sequence));

    m_gripper_motor_command->set_position(gripper_target);
    m_gripper_motor_command->set_velocity(gripper_velocity_target);
    m_gripper_motor_command->set_force(30.0f);

    if(m_gripper_command_sequence == 1 || m_gripper_command_sequence % 500 == 0)
    {
      mc_rtc::log::info(
          "[mc_kortex gripper BRIDGE][{}] iter={} enabled={} commandValid={} feedbackValid={} "
          "close={:.3f} calibration=[{:.2f},{:.1f}]% cap={:.1f}% target={:.1f}% measured={:.1f}% velocity={:.1f}%/s",
          m_name,
          m_gripper_command_sequence,
          m_handover_gripper_command_enabled.load(),
          m_handover_gripper_command_valid.load(),
          feedback_valid,
          close_cmd,
          open_percent,
          close_percent,
          max_percent,
          gripper_target,
          measured_gripper_position,
          gripper_velocity_target);
    }
  }

  // if (m_control_mode != k_api::ActuatorConfig::ControlMode::POSITION)
  // std::cout << std::endl;

  // ========================= Control mode has changed in mc_rtc, change it for
  // the robot ========================= //
  if (m_control_mode_id != m_prev_control_mode_id) {
    auto control_mode = k_api::ActuatorConfig::ControlModeInformation();
    control_mode.set_control_mode(m_control_mode);

    try {
      mc_rtc::log::info("[mc_kortex] Changing robot control mode to {} ",
                        m_control_mode);
      for (int i = 0; i < m_actuator_count; i++) {
        // printJointActiveControlLoop(i+1);
        m_actuator_config->SetControlMode(control_mode, i + 1);
        // printJointActiveControlLoop(i+1);
      }
      m_prev_control_mode_id = m_control_mode_id;
    } catch (k_api::KDetailedException &ex) {
      printException(ex);
      return_value = false;
      running = false;
    }
  }

  try {
    m_base_cyclic->Refresh_callback(m_base_command, lambda_fct, 0);
    return_value = true;
  } catch (k_api::KDetailedException &ex) {
    printException(ex);
    return_value = false;
    running = false;
  }

  m_prev_control_id = m_control_id;
  return return_value;
}

void KinovaRobot::updateSensors(mc_control::MCGlobalController &gc) {
  std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
  auto &robot = gc.controller().robots().robot(m_name);

  /*
   * Dual-safe gripper bridge datastore exchange.
   *
   * Every hardware instance owns a robot-scoped command/feedback namespace:
   *   HandoverInterceptionController::<robot>::<leaf>
   *
   * The primary Robot-A instance also accepts and republishes the historical
   * unscoped HandoverInterceptionController keys so the frozen/active Robot-A
   * controllers remain bit-for-bit compatible. Secondary robots never read
   * those unscoped commands, preventing Robot-A closure from actuating Robot B.
   */
  auto & ds = gc.controller().datastore();
  const bool primaryRobot = isPrimaryRobot(gc);

  const std::string scopedCloseKey = scopedHandoverKey("gripperClose");
  const std::string scopedOpenPercentKey =
      scopedHandoverKey("gripperOpenPercent");
  const std::string scopedClosePercentKey =
      scopedHandoverKey("gripperClosePercent");
  const std::string scopedMaxPercentKey =
      scopedHandoverKey("gripperMaxPercent");
  const std::string scopedCommandEnabledKey =
      scopedHandoverKey("gripperCommandEnabled");

  bool commandSeen = false;
  bool legacyCommandUsed = false;
  double closeCommand = 0.0;
  double openPercent = 0.87;
  double closePercent = 35.0;
  double maxPercent = 35.0;
  bool commandEnabled = false;

  try
  {
    if(ds.has(scopedCloseKey))
    {
      closeCommand = ds.get<double>(scopedCloseKey);
      commandSeen = true;
    }
    else if(primaryRobot && ds.has(kGripperCloseKey))
    {
      closeCommand = ds.get<double>(kGripperCloseKey);
      commandSeen = true;
    }
    else if(primaryRobot && ds.has(kLegacyGripperCloseKey))
    {
      closeCommand = ds.get<double>(kLegacyGripperCloseKey);
      commandSeen = true;
      legacyCommandUsed = true;
    }

    if(ds.has(scopedOpenPercentKey))
    {
      openPercent = ds.get<double>(scopedOpenPercentKey);
    }
    else if(primaryRobot && ds.has(kGripperOpenPercentKey))
    {
      openPercent = ds.get<double>(kGripperOpenPercentKey);
    }

    if(ds.has(scopedClosePercentKey))
    {
      closePercent = ds.get<double>(scopedClosePercentKey);
    }
    else if(primaryRobot && ds.has(kGripperClosePercentKey))
    {
      closePercent = ds.get<double>(kGripperClosePercentKey);
    }

    if(ds.has(scopedMaxPercentKey))
    {
      maxPercent = ds.get<double>(scopedMaxPercentKey);
    }
    else if(primaryRobot && ds.has(kGripperMaxPercentKey))
    {
      maxPercent = ds.get<double>(kGripperMaxPercentKey);
    }
    else if(primaryRobot && ds.has(kLegacyGripperMaxPercentKey))
    {
      maxPercent = ds.get<double>(kLegacyGripperMaxPercentKey);
      legacyCommandUsed = true;
    }

    if(ds.has(scopedCommandEnabledKey))
    {
      commandEnabled = ds.get<bool>(scopedCommandEnabledKey);
    }
    else if(primaryRobot && ds.has(kGripperCommandEnabledKey))
    {
      commandEnabled = ds.get<bool>(kGripperCommandEnabledKey);
    }
  }
  catch(const std::exception & e)
  {
    commandSeen = false;
    commandEnabled = false;
    mc_rtc::log::error(
        "[mc_kortex gripper][{}] datastore command rejected: {}",
        m_name, e.what());
  }
  catch(...)
  {
    commandSeen = false;
    commandEnabled = false;
    mc_rtc::log::error(
        "[mc_kortex gripper][{}] datastore command rejected by unknown exception",
        m_name);
  }

  closeCommand = std::max(0.0, std::min(1.0, closeCommand));
  openPercent = std::max(0.0, std::min(99.0, openPercent));
  closePercent = std::max(
      openPercent + 1e-3, std::min(100.0, closePercent));
  maxPercent = std::max(
      closePercent, std::min(100.0, maxPercent));
  m_handover_gripper_close.store(closeCommand);
  m_handover_gripper_open_percent.store(openPercent);
  m_handover_gripper_close_percent.store(closePercent);
  m_handover_gripper_max_percent.store(maxPercent);
  m_handover_gripper_command_valid.store(commandSeen);
  m_handover_gripper_command_enabled.store(commandEnabled);

  if(legacyCommandUsed)
  {
    if(!m_legacy_gripper_warning_logged)
    {
      m_legacy_gripper_warning_logged = true;
      mc_rtc::log::warning(
          "[mc_kortex gripper][{}] using legacy HandoverThesisController datastore key; migrate to HandoverInterceptionController keys",
          m_name);
    }
  }

  bool feedbackValid = false;
  double measuredPercent = m_gripper_measured_percent.load();
  double measuredVelocityPercent = 0.0;
  try
  {
    const auto & inter = m_state.interconnect();
    if(m_physical_gripper_present && inter.has_gripper_feedback() && inter.gripper_feedback().motor_size() > 0)
    {
      const auto & motor = inter.gripper_feedback().motor()[0];
      const double position = static_cast<double>(motor.position());
      const double velocity = static_cast<double>(motor.velocity());
      if(std::isfinite(position) && std::isfinite(velocity)
         && position >= -1.0 && position <= 101.0)
      {
        measuredPercent = std::max(0.0, std::min(100.0, position));
        measuredVelocityPercent = velocity;
        feedbackValid = true;
        gripper_position = static_cast<float>(measuredPercent);
        gripper_velocity = static_cast<float>(measuredVelocityPercent);
      }
    }
  }
  catch(...)
  {
    feedbackValid = false;
  }

  m_gripper_measured_percent.store(measuredPercent);
  m_gripper_measured_velocity_percent.store(measuredVelocityPercent);
  m_gripper_feedback_valid.store(feedbackValid);
  uint64_t feedbackSequence = m_gripper_feedback_sequence.load();
  if(feedbackValid)
  {
    feedbackSequence++;
    m_gripper_feedback_sequence.store(feedbackSequence);
  }

  auto publishDouble = [&ds](const std::string & key, double value)
  {
    if(ds.has(key)) { ds.assign<double>(key, value); }
    else { ds.make<double>(key, value); }
  };
  auto publishBool = [&ds](const std::string & key, bool value)
  {
    if(ds.has(key)) { ds.assign<bool>(key, value); }
    else { ds.make<bool>(key, value); }
  };
  auto publishUInt64 = [&ds](const std::string & key, uint64_t value)
  {
    if(ds.has(key)) { ds.assign<uint64_t>(key, value); }
    else { ds.make<uint64_t>(key, value); }
  };

  publishDouble(scopedHandoverKey("gripperMeasuredPercent"), measuredPercent);
  publishDouble(scopedHandoverKey("gripperMeasuredVelocityPercent"),
                measuredVelocityPercent);
  publishBool(scopedHandoverKey("gripperFeedbackValid"), feedbackValid);
  publishUInt64(scopedHandoverKey("gripperFeedbackSequence"), feedbackSequence);
  publishBool(scopedHandoverKey("gripperBridgeCommandSeen"), commandSeen);
  publishDouble(scopedHandoverKey("gripperBridgeOpenPercent"), openPercent);
  publishDouble(scopedHandoverKey("gripperBridgeClosePercent"), closePercent);
  publishDouble(scopedHandoverKey("gripperBridgeMaxPercent"), maxPercent);

  if(primaryRobot)
  {
    publishDouble(kGripperMeasuredPercentKey, measuredPercent);
    publishDouble(kGripperMeasuredVelocityPercentKey,
                  measuredVelocityPercent);
    publishBool(kGripperFeedbackValidKey, feedbackValid);
    publishUInt64(kGripperFeedbackSequenceKey, feedbackSequence);
    publishBool(kGripperBridgeCommandSeenKey, commandSeen);
    publishDouble(kGripperBridgeOpenPercentKey, openPercent);
    publishDouble(kGripperBridgeClosePercentKey, closePercent);
    publishDouble(kGripperBridgeMaxPercentKey, maxPercent);
  }

  const auto & rjo = robot.refJointOrder();

  // Continuous-joint offset must use the posture task belonging to this
  // robot. The historical unscoped callback is accepted only for Robot A.
  const std::string scopedPostureTaskKey =
      scopedRuntimeKey("getPostureTask");
  mc_tasks::PostureTaskPtr posture_task_pt = nullptr;
  if(gc.controller().datastore().has(scopedPostureTaskKey))
  {
    posture_task_pt = gc.controller().datastore().call<mc_tasks::PostureTaskPtr>(
        scopedPostureTaskKey);
  }
  else if(primaryRobot && gc.controller().datastore().has("getPostureTask"))
  {
    posture_task_pt = gc.controller().datastore().call<mc_tasks::PostureTaskPtr>(
        "getPostureTask");
  }
  m_offsets = computePostureTaskOffset(
      robot, posture_task_pt, !primaryRobot);

  // The mc_rtc state vector follows the complete RobotModule reference-joint
  // order. Seven entries come from physical arm encoders; the six Robotiq
  // model joints are reconstructed from the single physical motor feedback.
  std::vector<double> q(rjo.size(), 0.0);
  std::vector<double> qdot(rjo.size(), 0.0);
  std::vector<double> tau(rjo.size(), 0.0);
  std::map<std::string, sva::ForceVecd> wrenches;
  std::map<std::string, double> current;

  auto refIndex = [&rjo](const std::string & name) -> size_t
  {
    const auto it = std::find(rjo.begin(), rjo.end(), name);
    if(it == rjo.end())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_kortex] required model joint missing from reference order: {}", name);
    }
    return static_cast<size_t>(std::distance(rjo.begin(), it));
  };

  for (size_t i = 0; i < m_actuator_count; i++) {
    const std::string & jointName = m_arm_joint_names.at(i);
    const size_t ref = refIndex(jointName);
    double kt = (i > 3) ? 0.076 : 0.11;
    q[ref] = jointPoseToRad(i, m_state.mutable_actuators(i)->position()) +
             m_offsets[i];
    if (m_use_filtered_velocities) {
      m_filtered_velocities[i] =
          m_velocity_filter_ratio * m_filtered_velocities[i] +
          (1 - m_velocity_filter_ratio) *
              mc_rtc::constants::toRad(
                  m_state.mutable_actuators(i)->velocity());
      qdot[ref] = m_filtered_velocities[i];
    } else {
      qdot[ref] =
          mc_rtc::constants::toRad(m_state.mutable_actuators(i)->velocity());
    }
    tau[ref] = -m_state.mutable_actuators(i)->torque();
    m_tau_sensor(i) = tau[ref];
    m_current_measurement(i) = m_state.mutable_actuators(i)->current_motor();
    m_torque_from_current_measurement(i) =
        m_state.mutable_actuators(i)->current_motor() * kt * GEAR_RATIO;
    current[jointName] = m_current_measurement(i);
  }

  if(m_model_has_robotiq_joints)
  {
    const double openPercent = m_handover_gripper_open_percent.load();
    const double closePercent = m_handover_gripper_close_percent.load();
    const double span = std::max(1e-6, closePercent - openPercent);
    const double closure = std::max(
        0.0, std::min(1.0, (measuredPercent - openPercent) / span));
    const double closureRate = measuredVelocityPercent / span;
    constexpr double openQ = 0.0;
    constexpr double closeQ = 0.8;
    const double qGrip = openQ + closure * (closeQ - openQ);
    const double qdotGrip = closureRate * (closeQ - openQ);
    const std::vector<std::pair<std::string, double>> gripQ = {
        {"gen3_robotiq_85_left_knuckle_joint", qGrip},
        {"gen3_robotiq_85_right_knuckle_joint", -qGrip},
        {"gen3_robotiq_85_left_inner_knuckle_joint", qGrip},
        {"gen3_robotiq_85_right_inner_knuckle_joint", -qGrip},
        {"gen3_robotiq_85_left_finger_tip_joint", -qGrip},
        {"gen3_robotiq_85_right_finger_tip_joint", qGrip}};
    const std::vector<std::pair<std::string, double>> gripQdot = {
        {"gen3_robotiq_85_left_knuckle_joint", qdotGrip},
        {"gen3_robotiq_85_right_knuckle_joint", -qdotGrip},
        {"gen3_robotiq_85_left_inner_knuckle_joint", qdotGrip},
        {"gen3_robotiq_85_right_inner_knuckle_joint", -qdotGrip},
        {"gen3_robotiq_85_left_finger_tip_joint", -qdotGrip},
        {"gen3_robotiq_85_right_finger_tip_joint", qdotGrip}};
    for(const auto & item : gripQ) { q[refIndex(item.first)] = item.second; }
    for(const auto & item : gripQdot) { qdot[refIndex(item.first)] = item.second; }
  }

  gc.setEncoderValues(m_name, q);
  gc.setEncoderVelocities(m_name, qdot);
  gc.setJointTorques(m_name, tau);

  /*
   * V6.1.1 passive physical-wrench bridge.
   *
   * The validated V6.1 handover controller remains configured with
   * transfer.source=synthetic. These measurements are therefore diagnostic
   * only: they cannot authorize attachment, alter motion, or complete the
   * transfer state. The Kortex tool-frame convention and signs must be
   * calibrated before this signal is used by the force-transfer policy.
   */
  if(primaryRobot)
  {
    ensurePassiveWrenchSensor(robot, "control");
    auto & realRobot = gc.controller().realRobots().robot(m_name);
    ensurePassiveWrenchSensor(realRobot, "real");

  const auto & baseFeedback = m_state.base();
  const Eigen::Vector3d rawToolForce(
      static_cast<double>(baseFeedback.tool_external_wrench_force_x()),
      static_cast<double>(baseFeedback.tool_external_wrench_force_y()),
      static_cast<double>(baseFeedback.tool_external_wrench_force_z()));
  const Eigen::Vector3d rawToolCouple(
      static_cast<double>(baseFeedback.tool_external_wrench_torque_x()),
      static_cast<double>(baseFeedback.tool_external_wrench_torque_y()),
      static_cast<double>(baseFeedback.tool_external_wrench_torque_z()));

  if(rawToolForce.allFinite() && rawToolCouple.allFinite())
  {
    wrenches.emplace(
        kPassiveWrenchSensorName,
        sva::ForceVecd(rawToolCouple, rawToolForce));
    gc.setWrenches(m_name, wrenches);

    ++m_passive_wrench_samples;
    if(m_passive_wrench_samples == 1 || m_passive_wrench_samples % 1000 == 0)
    {
      mc_rtc::log::info(
          "[PhysicalWrenchBridge][{}] sample={} source=KortexBaseCyclic sensor={} frame=KortexToolRaw "
          "force=[{:.4f},{:.4f},{:.4f}]N couple=[{:.4f},{:.4f},{:.4f}]Nm "
          "forceNorm={:.4f}N coupleNorm={:.4f}Nm passiveOnly=true syntheticTransferUnchanged=true",
          m_name, m_passive_wrench_samples, kPassiveWrenchSensorName,
          rawToolForce.x(), rawToolForce.y(), rawToolForce.z(),
          rawToolCouple.x(), rawToolCouple.y(), rawToolCouple.z(),
          rawToolForce.norm(), rawToolCouple.norm());
    }
  }
  else
  {
    if(!m_invalid_wrench_warned)
    {
      m_invalid_wrench_warned = true;
      mc_rtc::log::error(
          "[PhysicalWrenchBridge][{}] non-finite Kortex external wrench rejected; passive bridge remains unavailable",
          m_name);
    }
  }

  }

  // mc_kortex reports motor-current names as joint_1 ... joint_7,
  // The active model may use gen3_joint_1..7 or joint_1..7.
  // Convert current map keys before passing them to mc_rtc.
  std::map<std::string, double> current_mcrtc;

  for(const auto & kv : current)
  {
    std::string name = kv.first;

    if(name.rfind("joint_", 0) == 0)
    {
      name = "gen3_" + name;
    }

    current_mcrtc[name] = kv.second;
  }

  // Disabled for custom gen3_2f85 fixed-gripper model bring-up.
  // Motor currents are optional sensor data and are not required for the first safe arm motion.
  // The custom env robot model does not accept these current-map joint keys here.
  // gc.setJointMotorCurrents(m_name, current_mcrtc);
  // gc.setJointMotorTemperatures(m_name,temp);

  // Store torque friction in a robot-specific channel. Robot A keeps the
  // historical key for compatibility with the validated single-robot stack.
  const std::string torqueFrictionKey =
      isPrimaryRobot(gc) ? "torque_fric" : scopedRuntimeKey("torque_fric");
  if (!gc.controller().datastore().has(torqueFrictionKey)) {
    gc.controller().datastore().make<Eigen::VectorXd>(torqueFrictionKey, tau_fric);
  } else {
    gc.controller().datastore().assign(torqueFrictionKey, tau_fric);
  }
}

void KinovaRobot::updateControl(mc_control::MCGlobalController &controller) {
  std::unique_lock<std::mutex> lock(m_update_control_mutex);
  auto &robot = controller.controller().robots().robot(m_name);
  m_command = robot.mbc();
  m_control_id++;
}

std::string KinovaRobot::controlLoopParamToString(
    k_api::ActuatorConfig::LoopSelection &loop_selected, int actuator_idx) {
  k_api::ActuatorConfig::ControlLoopParameters parameters =
      m_actuator_config->GetControlLoopParameters(loop_selected, actuator_idx);
  std::ostringstream ss;
  ss << "kAz = [";
  for (size_t i = 0; i < parameters.kaz_size() - 1; i++)
    ss << parameters.kaz(i) << ",";
  ss << parameters.kaz(parameters.kaz_size()) << "] kBz = [";
  for (size_t i = 0; i < parameters.kbz_size() - 1; i++)
    ss << parameters.kbz(i) << ",";
  ss << parameters.kbz(parameters.kbz_size()) << "]";

  return ss.str();
}

void KinovaRobot::checkBaseFaultBanks(uint32_t fault_bank_a,
                                      uint32_t fault_bank_b) {
  if (fault_bank_a != 0) {
    auto error_list = getBaseFaultList(fault_bank_a);
    std::ostringstream ss;
    ss << "[";
    std::copy(error_list.begin(), error_list.end() - 1,
              std::ostream_iterator<std::string>(ss, ", "));
    ss << error_list.back() << "]";
    mc_rtc::log::error_and_throw("[MC_KORTEX] Error in base fault bank A : {}",
                                 ss.str());
  }
  if (fault_bank_b != 0) {
    auto error_list = getBaseFaultList(fault_bank_b);
    std::ostringstream ss;
    ss << "[";
    std::copy(error_list.begin(), error_list.end() - 1,
              std::ostream_iterator<std::string>(ss, ", "));
    ss << error_list.back() << "]";
    mc_rtc::log::error_and_throw("[MC_KORTEX] Error in base fault bank B : {}",
                                 ss.str());
  }
}

void KinovaRobot::checkActuatorsFaultBanks(
    k_api::BaseCyclic::Feedback feedback) {
  for (size_t i = 0; i < m_actuator_count; i++) {
    if (feedback.mutable_actuators(i)->fault_bank_a() != 0) {
      auto error_list =
          getActuatorFaultList(feedback.mutable_actuators(i)->fault_bank_a());
      std::ostringstream ss;
      ss << "[";
      std::copy(error_list.begin(), error_list.end() - 1,
                std::ostream_iterator<std::string>(ss, ", "));
      ss << error_list.back() << "]";
      mc_rtc::log::error_and_throw(
          "[MC_KORTEX] Error in base fault bank A : {}", ss.str());
    }
    if (feedback.mutable_actuators(i)->fault_bank_b() != 0) {
      auto error_list =
          getActuatorFaultList(feedback.mutable_actuators(i)->fault_bank_b());
      std::ostringstream ss;
      ss << "[";
      std::copy(error_list.begin(), error_list.end() - 1,
                std::ostream_iterator<std::string>(ss, ", "));
      ss << error_list.back() << "]";
      mc_rtc::log::error_and_throw(
          "[MC_KORTEX] Error in base fault bank B : {}", ss.str());
    }
  }
}

std::vector<std::string> KinovaRobot::getBaseFaultList(uint32_t fault_bank) {
  std::vector<string> fault_list;
  if (fault_bank & (uint32_t)0x1)
    fault_list.push_back("FIRMWARE_UPDATE_FAILURE");
  if (fault_bank & (uint32_t)0x2)
    fault_list.push_back("EXTERNAL_COMMUNICATION_ERROR");
  if (fault_bank & (uint32_t)0x4)
    fault_list.push_back("MAXIMUM_AMBIENT_TEMPERATURE");
  if (fault_bank & (uint32_t)0x8)
    fault_list.push_back("MAXIMUM_CORE_TEMPERATURE");
  if (fault_bank & (uint32_t)0x10)
    fault_list.push_back("JOINT_FAULT");
  if (fault_bank & (uint32_t)0x20)
    fault_list.push_back("CYCLIC_DATA_JITTER");
  if (fault_bank & (uint32_t)0x40)
    fault_list.push_back("REACHED_MAXIMUM_EVENT_LOGS");
  if (fault_bank & (uint32_t)0x80)
    fault_list.push_back("NO_KINEMATICS_SUPPORT");
  if (fault_bank & (uint32_t)0x100)
    fault_list.push_back("ABOVE_MAXIMUM_DOF");
  if (fault_bank & (uint32_t)0x200)
    fault_list.push_back("NETWORK_ERROR");
  if (fault_bank & (uint32_t)0x400)
    fault_list.push_back("UNABLE_TO_REACH_POSE");
  if (fault_bank & (uint32_t)0x800)
    fault_list.push_back("JOINT_DETECTION_ERROR");
  if (fault_bank & (uint32_t)0x1000)
    fault_list.push_back("NETWORK_INITIALIZATION_ERROR");
  if (fault_bank & (uint32_t)0x2000)
    fault_list.push_back("MAXIMUM_CURRENT");
  if (fault_bank & (uint32_t)0x4000)
    fault_list.push_back("MAXIMUM_VOLTAGE");
  if (fault_bank & (uint32_t)0x8000)
    fault_list.push_back("MINIMUM_VOLTAGE");
  if (fault_bank & (uint32_t)0x10000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY");
  if (fault_bank & (uint32_t)0x20000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY");
  if (fault_bank & (uint32_t)0x40000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION");
  if (fault_bank & (uint32_t)0x80000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION");
  if (fault_bank & (uint32_t)0x100000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE");
  if (fault_bank & (uint32_t)0x200000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE");
  if (fault_bank & (uint32_t)0x400000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_PAYLOAD");
  if (fault_bank & (uint32_t)0x800000)
    fault_list.push_back("EMERGENCY_STOP_ACTIVATED");
  if (fault_bank & (uint32_t)0x1000000)
    fault_list.push_back("EMERGENCY_LINE_ACTIVATED");
  if (fault_bank & (uint32_t)0x2000000)
    fault_list.push_back("INRUSH_CURRENT_LIMITER_FAULT");
  if (fault_bank & (uint32_t)0x4000000)
    fault_list.push_back("NVRAM_CORRUPTED");
  if (fault_bank & (uint32_t)0x8000000)
    fault_list.push_back("INCOMPATIBLE_FIRMWARE_VERSION");
  if (fault_bank & (uint32_t)0x10000000)
    fault_list.push_back("POWERON_SELF_TEST_FAILURE");
  if (fault_bank & (uint32_t)0x20000000)
    fault_list.push_back("DISCRETE_INPUT_STUCK_ACTIVE");
  if (fault_bank & (uint32_t)0x40000000)
    fault_list.push_back("ARM_INTO_ILLEGAL_POSITION");
  return fault_list;
}

std::vector<std::string>
KinovaRobot::getActuatorFaultList(uint32_t fault_bank) {
  std::vector<string> fault_list;
  if (fault_bank & (uint32_t)0x1)
    fault_list.push_back("FOLLOWING_ERROR");
  if (fault_bank & (uint32_t)0x2)
    fault_list.push_back("MAXIMUM_VELOCITY");
  if (fault_bank & (uint32_t)0x4)
    fault_list.push_back("JOINT_LIMIT_HIGH");
  if (fault_bank & (uint32_t)0x8)
    fault_list.push_back("JOINT_LIMIT_LOW");
  if (fault_bank & (uint32_t)0x10)
    fault_list.push_back("STRAIN_GAUGE_MISMATCH");
  if (fault_bank & (uint32_t)0x20)
    fault_list.push_back("MAXIMUM_TORQUE");
  if (fault_bank & (uint32_t)0x40)
    fault_list.push_back("UNRELIABLE_ABSOLUTE_POSITION");
  if (fault_bank & (uint32_t)0x80)
    fault_list.push_back("MAGNETIC_POSITION");
  if (fault_bank & (uint32_t)0x100)
    fault_list.push_back("HALL_POSITION");
  if (fault_bank & (uint32_t)0x200)
    fault_list.push_back("HALL_SEQUENCE");
  if (fault_bank & (uint32_t)0x400)
    fault_list.push_back("INPUT_ENCODER_HALL_MISMATCH");
  if (fault_bank & (uint32_t)0x800)
    fault_list.push_back("INPUT_ENCODER_INDEX_MISMATCH");
  if (fault_bank & (uint32_t)0x1000)
    fault_list.push_back("INPUT_ENCODER_MAGNETIC_MISMATCH");
  if (fault_bank & (uint32_t)0x2000)
    fault_list.push_back("MAXIMUM_MOTOR_CURRENT");
  if (fault_bank & (uint32_t)0x4000)
    fault_list.push_back("MOTOR_CURRENT_MISMATCH");
  if (fault_bank & (uint32_t)0x8000)
    fault_list.push_back("MAXIMUM_VOLTAGE");
  if (fault_bank & (uint32_t)0x10000)
    fault_list.push_back("MINIMUM_VOLTAGE");
  if (fault_bank & (uint32_t)0x20000)
    fault_list.push_back("MAXIMUM_MOTOR_TEMPERATURE");
  if (fault_bank & (uint32_t)0x40000)
    fault_list.push_back("MAXIMUM_CORE_TEMPERATURE");
  if (fault_bank & (uint32_t)0x80000)
    fault_list.push_back("NON_VOLATILE_MEMORY_CORRUPTED");
  if (fault_bank & (uint32_t)0x100000)
    fault_list.push_back("MOTOR_DRIVER_FAULT");
  if (fault_bank & (uint32_t)0x200000)
    fault_list.push_back("EMERGENCY_LINE_ASSERTED");
  if (fault_bank & (uint32_t)0x400000)
    fault_list.push_back("COMMUNICATION_TICK_LOST");
  if (fault_bank & (uint32_t)0x800000)
    fault_list.push_back("WATCHDOG_TRIGGERED");
  if (fault_bank & (uint32_t)0x1000000)
    fault_list.push_back("UNRELIABLE_CAPACITIVE_SENSOR");
  if (fault_bank & (uint32_t)0x2000000)
    fault_list.push_back("UNEXPECTED_GEAR_RATIO");
  if (fault_bank & (uint32_t)0x4000000)
    fault_list.push_back("HALL_MAGNETIC_MISMATCH");
  return fault_list;
}

void KinovaRobot::controlThread(mc_control::MCGlobalController &controller,
                                std::mutex &startM,
                                std::condition_variable &startCV, bool &start,
                                bool &running) {
  {
    std::unique_lock<std::mutex> lock(startM);
    startCV.wait(lock, [&]() { return start; });
  }

  setLowServoingMode();

  int64_t now = 0;
  int64_t last = 0;
  int64_t dt = 0;

  bool return_status;

  try {

    while (not stop_controller) {
      now = GetTickUs();
      if (now - last < 1000)
        continue;
      dt = now - last;
      last = now;

      if (m_servoing_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) {
        sendCommand(controller.robots().robot(m_name), running);
        t_plot += 1e-3;
      } else {
        mc_rtc::log::info("high level servoing");
        // updateState(running);
      }
    }

    mc_rtc::log::warning("[MC_KORTEX] {} control loop killed", m_name);

    return_status = true;
  } catch (k_api::KDetailedException &ex) {
    std::cout << "Kortex error: " << ex.what() << std::endl;
    return_status = false;
  } catch (std::runtime_error &ex2) {
    std::cout << "Runtime error: " << ex2.what() << std::endl;
    return_status = false;
  }

  auto control_mode = k_api::ActuatorConfig::ControlModeInformation();
  control_mode.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);

  for (int i = 0; i < m_actuator_count; i++) {
    // printJointActiveControlLoop(i+1);
    m_actuator_config->SetControlMode(control_mode, i + 1);
    // printJointActiveControlLoop(i+1);
  }

  setSingleServoingMode();

}

void KinovaRobot::stopController() { stop_controller = true; }

void KinovaRobot::moveToHomePosition() {
  // Make sure the arm is in Single Level Servoing before executing an Action
  setSingleServoingMode();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Move arm to "Home" position
  mc_rtc::log::info("[mc_kortex] Moving the arm to a safe position");
  auto action_type = k_api::Base::RequestedActionType();
  action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
  auto action_list = m_base->ReadAllActions(action_type);
  auto action_handle = k_api::Base::ActionHandle();
  action_handle.set_identifier(0);
  for (auto action : action_list.action_list()) {
    if (action.name() == "Home") {
      action_handle = action.handle();
    }
  }

  if (action_handle.identifier() == 0) {
    mc_rtc::log::warning("[mc_kortex] Can't reach safe position, exiting");
  } else {
    bool action_finished = false;
    // Notify of any action topic event
    auto options = k_api::Common::NotificationOptions();
    auto notification_handle = m_base->OnNotificationActionTopic(
        check_for_end_or_abort(action_finished), options);

    m_base->ExecuteActionFromReference(action_handle);

    while (!action_finished) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    m_base->Unsubscribe(notification_handle);
  }
}

void KinovaRobot::moveToInitPosition() {
  // Make sure the arm is in Single Level Servoing before executing an Action
  setSingleServoingMode();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Create trajectory object
  k_api::Base::WaypointList wpts = k_api::Base::WaypointList();

  // Define joint poses
  auto jointPoses = std::vector<std::array<float, 7>>();
  std::array<float, 7> arr;
  for (size_t i = 0; i < min(m_actuator_count, 7); i++)
    arr[i] = radToJointPose(i, m_init_posture[i]);
  jointPoses.push_back(arr);

  // Add initial pose as a waypoint
  k_api::Base::Waypoint *wpt = wpts.add_waypoints();
  wpt->set_name("waypoint_0");
  k_api::Base::AngularWaypoint *ang = wpt->mutable_angular_waypoint();
  for (size_t i = 0; i < m_actuator_count; i++) {
    ang->add_angles(jointPoses.at(0).at(i));
  }

  // Connect to notification action topic
  std::promise<k_api::Base::ActionEvent> finish_promise_cart;
  auto finish_future_cart = finish_promise_cart.get_future();
  auto promise_notification_handle_cart = m_base->OnNotificationActionTopic(
      create_event_listener_by_promise(finish_promise_cart),
      k_api::Common::NotificationOptions());

  k_api::Base::WaypointValidationReport result;
  try {
    // Verify validity of waypoints
    auto validationResult = m_base->ValidateWaypointList(wpts);
    result = validationResult;
  } catch (k_api::KDetailedException &ex) {
    mc_rtc::log::error(
        "[mc_kortex] Error on waypoint list to reach initial position");
    printException(ex);
    return;
  }

  // Trajectory error report always exists and we need to make sure no elements
  // are found in order to validate the trajectory
  if (result.trajectory_error_report().trajectory_error_elements_size() == 0) {
    // Execute action
    try {
      mc_rtc::log::info("[mc_kortex] Moving the arm to initial position");
    } catch (k_api::KDetailedException &ex) {
      mc_rtc::log::error("[mc_kortex] Error when trying to execute trajectory "
                         "to reach initial position");
      printException(ex);
      return;
    }
    // Wait for future value from promise
    const auto ang_status =
        finish_future_cart.wait_for(std::chrono::seconds(100));
    m_base->Unsubscribe(promise_notification_handle_cart);
    if (ang_status != std::future_status::ready) {
      mc_rtc::log::warning("[mc_kortex] Timeout when trying to reach initial "
                           "position, try again");
    } else {
      const auto ang_promise_event = finish_future_cart.get();
      mc_rtc::log::success("[mc_kortex] Angular waypoint trajectory completed");
    }
  } else {
    mc_rtc::log::error(
        "[mc_kortex] Error found in trajectory to initial position");
    mc_rtc::log::error(result.trajectory_error_report().DebugString());
  }
}

void KinovaRobot::printState() {
  std::string serialized_data;
  google::protobuf::util::MessageToJsonString(m_state, &serialized_data);
  std::cout << serialized_data << std::endl;
}

void KinovaRobot::printJointActiveControlLoop(int joint_id) {
  uint32_t control_loop;
  std::vector<std::string> active_loops;
  control_loop =
      m_actuator_config->GetActivatedControlLoop(joint_id).control_loop();
  if (control_loop &
      k_api::ActuatorConfig::ControlLoopSelection::JOINT_POSITION)
    active_loops.push_back("JOINT_POSITION");
  if (control_loop & k_api::ActuatorConfig::ControlLoopSelection::JOINT_TORQUE)
    active_loops.push_back("JOINT_TORQUE");
  if (control_loop &
      k_api::ActuatorConfig::ControlLoopSelection::JOINT_TORQUE_HIGH_VELOCITY)
    active_loops.push_back("JOINT_TORQUE_HIGH_VELOCITY");
  if (control_loop &
      k_api::ActuatorConfig::ControlLoopSelection::JOINT_VELOCITY)
    active_loops.push_back("JOINT_VELOCITY");
  if (control_loop & k_api::ActuatorConfig::ControlLoopSelection::MOTOR_CURRENT)
    active_loops.push_back("MOTOR_CURRENT");
  if (control_loop &
      k_api::ActuatorConfig::ControlLoopSelection::MOTOR_POSITION)
    active_loops.push_back("MOTOR_POSITION");
  if (control_loop &
      k_api::ActuatorConfig::ControlLoopSelection::MOTOR_VELOCITY)
    active_loops.push_back("MOTOR_VELOCITY");

  std::ostringstream ss;
  ss << "[";
  std::copy(active_loops.begin(), active_loops.end() - 1,
            std::ostream_iterator<std::string>(ss, ", "));
  ss << active_loops.back() << "]";

  mc_rtc::log::info("[mc_kortex][Joint {}] Active control loops: {}", joint_id,
                    ss.str());
}

// ============================== Private methods ==============================
// //

void KinovaRobot::initFiltersBuffers() {
  m_filter_input_buffer.assign(m_actuator_count,
                               boost::circular_buffer<double>(2, 0.0));
  m_filter_output_buffer.assign(m_actuator_count,
                                boost::circular_buffer<double>(2, 0.0));
}

void KinovaRobot::addGui(mc_control::MCGlobalController &gc) {
  if(m_allow_init_posture_gui)
  {
    gc.controller().gui()->addElement(
        {"Kortex", m_name},
        mc_rtc::gui::Button("Move to initial position", [this]() {
          setSingleServoingMode();
          moveToInitPosition();
          setLowServoingMode();
        }));
  }

  gc.controller().gui()->addElement(
      {"Kortex", m_name},
      mc_rtc::gui::ArrayLabel("PostureTask offsets",
                              gc.controller().robots().robot(m_name).refJointOrder(),
                              [this]() { return m_offsets; }));

  if (m_torque_control_type == mc_kinova::TorqueControlType::Custom) {
    gc.controller().gui()->addElement(
        {"Kortex", m_name, "Friction"},
        mc_rtc::gui::ArrayInput(
            "Friction stiction values", gc.controller().robots().robot(m_name).refJointOrder(),
            [this]() { return m_stiction_values; },
            [this](const std::vector<double> &v) { m_stiction_values = v; }),
        mc_rtc::gui::ArrayInput(
            "Friction coulomb values", gc.controller().robots().robot(m_name).refJointOrder(),
            [this]() { return m_friction_values; },
            [this](const std::vector<double> &v) { m_friction_values = v; }),
        mc_rtc::gui::ArrayInput(
            "Friction viscous values", gc.controller().robots().robot(m_name).refJointOrder(),
            [this]() { return m_viscous_values; },
            [this](const std::vector<double> &v) { m_viscous_values = v; }),
        mc_rtc::gui::NumberInput(
            "Friction compensation velocity threshold",
            [this]() { return m_friction_vel_threshold; },
            [this](const double v) { m_friction_vel_threshold = v; }),
        mc_rtc::gui::NumberInput(
            "Friction compensation acceleration threshold",
            [this]() { return m_friction_accel_threshold; },
            [this](const double v) { m_friction_accel_threshold = v; }));

    gc.controller().gui()->addElement(
        {"Kortex", m_name, "Transfer function"},
        mc_rtc::gui::NumberInput(
            "Lambda", [this]() { return m_lambda[0]; },
            [this](const double v) { m_lambda.assign(v, m_actuator_count); }));

    gc.controller().gui()->addElement(
        {"Kortex", m_name, "Integral term"},
        mc_rtc::gui::NumberInput(
            "Integral time constant",
            [this]() { return m_integral_slow_theta; },
            [this](const double v) { m_integral_slow_theta = v; }),
        mc_rtc::gui::NumberInput(
            "Integral gain", [this]() { return m_integral_slow_gain; },
            [this](const double v) { m_integral_slow_gain = v; }),
        mc_rtc::gui::ArrayInput(
            "Integral bound", [this]() { return m_integral_slow_bound; },
            [this](const std::vector<double> v) {
              m_integral_slow_bound = v;
            }));
  }

  if (m_use_filtered_velocities) {
    gc.controller().gui()->addElement(
        {"Kortex", m_name},
        mc_rtc::gui::NumberSlider(
            "Velocity filtering ratio (decay):",
            [this]() { return m_velocity_filter_ratio; },
            [this](const double v) { m_velocity_filter_ratio = v; }, 0.0, 1.0));
  }
}

void KinovaRobot::removeGui(mc_control::MCGlobalController &gc) {
  gc.controller().gui()->removeCategory({"Kortex", m_name});
  mc_rtc::log::success("[mc_kortex] Removed GUI");
}

void KinovaRobot::addPlot(mc_control::MCGlobalController &gc) {
  for (int i = 0; i < m_actuator_count; i++) {
    gc.controller().gui()->addPlot(
        fmt::format("{} Joint {}", m_name, i),
        mc_rtc::gui::plot::X("t", [this]() { return t_plot; }),
        mc_rtc::gui::plot::Y(
            "Integral term", [this, i]() { return m_integral_slow_filter[i]; },
            mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y(
            "Transfert function", [this, i]() { return m_filter_command[i]; },
            mc_rtc::gui::Color::Blue));
  }
}

void KinovaRobot::removePlot(mc_control::MCGlobalController &gc) {
  for (int i = 0; i < m_actuator_count; i++) {
    gc.controller().gui()->removePlot(
        fmt::format("{} Joint {}", m_name, i));
  }
}

double KinovaRobot::jointPoseToRad(int joint_idx, double deg) {
  return mc_rtc::constants::toRad((deg < 180.0) ? deg : deg - 360);
}

double KinovaRobot::radToJointPose(int joint_idx, double rad) {
  return mc_rtc::constants::toDeg((rad > 0) ? rad : 2 * M_PI + rad);
}

std::vector<double>
KinovaRobot::computePostureTaskOffset(
    mc_rbdyn::Robot &robot,
    mc_tasks::PostureTaskPtr posture_task,
    bool use_model_reference_if_missing) {
  std::vector<double> offsets(m_actuator_count, 0.0);

  const bool posture_available =
      posture_task != nullptr && !posture_task->posture().empty();
  if(!posture_available && !use_model_reference_if_missing)
  {
    // Preserve the validated Robot-A behavior when no posture task exists.
    return offsets;
  }

  const auto & rjo = robot.refJointOrder();

  for(size_t i = 0; i < m_actuator_count; ++i)
  {
    const auto jointIndex = robot.jointIndexByName(m_arm_joint_names.at(i));
    double target_pose = robot.mbc().q[jointIndex][0];
    if(posture_available)
    {
      target_pose = posture_task->posture()[jointIndex][0];
    }
    const double q =
        jointPoseToRad(i, m_state.mutable_actuators(i)->position());
    if(i % 2 == 0)
    {
      if(q > target_pose + M_PI) { offsets[i] = -2 * M_PI; }
      else if(q < target_pose - M_PI) { offsets[i] = 2 * M_PI; }
    }
  }
  return offsets;
}

uint32_t KinovaRobot::jointIdFromCommandID(google::protobuf::uint32 cmd_id) {
  return (cmd_id >> 16) & 0x0000000F;
}

int64_t KinovaRobot::GetTickUs() {
  struct timespec start;
  clock_gettime(CLOCK_MONOTONIC, &start);

  return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

void KinovaRobot::printError(const k_api::Error &err) {
  mc_rtc::log::error("[mc_kortex] KError error_code: {}", err.error_code());
  mc_rtc::log::error("[mc_kortex] KError sub_code: {}", err.error_sub_code());
  mc_rtc::log::error("[mc_kortex] KError sub_string: {}",
                     err.error_sub_string());

  // Error codes by themselves are not very verbose if you don't see their
  // corresponding enum value You can use google::protobuf helpers to get the
  // string enum element for every error code and sub-code
  mc_rtc::log::error(
      "[mc_kortex] Error code string equivalent: {}",
      k_api::ErrorCodes_Name(k_api::ErrorCodes(err.error_code())));
  mc_rtc::log::error(
      "[mc_kortex] Error sub-code string equivalent: {}",
      k_api::SubErrorCodes_Name(k_api::SubErrorCodes(err.error_sub_code())));
}

void KinovaRobot::printException(k_api::KDetailedException &ex) {
  // You can print the error informations and error codes
  auto error_info = ex.getErrorInfo().getError();
  mc_rtc::log::error("[mc_kortex] KDetailedException detected : {}", ex.what());

  printError(error_info);
}

std::function<void(k_api::Base::ActionNotification)>
KinovaRobot::check_for_end_or_abort(bool &finished) {
  return [&finished](k_api::Base::ActionNotification notification) {
    mc_rtc::log::info(
        "[mc_kortex] EVENT : {}",
        k_api::Base::ActionEvent_Name(notification.action_event()));

    // The action is finished when we receive a END or ABORT event
    switch (notification.action_event()) {
    case k_api::Base::ActionEvent::ACTION_ABORT:
    case k_api::Base::ActionEvent::ACTION_END:
      finished = true;
      break;
    default:
      break;
    }
  };
}

std::function<void(k_api::Base::ActionNotification)>
KinovaRobot::create_event_listener_by_promise(
    std::promise<k_api::Base::ActionEvent> &finish_promise_cart) {
  return [&finish_promise_cart](k_api::Base::ActionNotification notification) {
    const auto action_event = notification.action_event();
    switch (action_event) {
    case k_api::Base::ActionEvent::ACTION_END:
    case k_api::Base::ActionEvent::ACTION_ABORT:
      finish_promise_cart.set_value(action_event);
      break;
    default:
      break;
    }
  };
}

std::string printVec(std::vector<double> vec)
{
  std::ostringstream s;
  s << "[";

  if(vec.empty())
  {
    s << "]";
    return s.str();
  }

  std::copy(vec.begin(),
            vec.end() - 1,
            std::ostream_iterator<double>(s, ", "));

  s << vec.back() << "]";
  return s.str();
}
} // namespace mc_kinova
