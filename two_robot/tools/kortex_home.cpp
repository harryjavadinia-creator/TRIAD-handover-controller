// kortex_home: send a Kinova Gen3 to one of its stored joint actions ("Home" by default) through the
// Kortex API and wait for the action to end. Used to return Robot A to its start posture between hardware
// runs, because the driver's own Kortex.init_posture.on_startup path is rejected by the robot firmware
// ("Time optimal splines are not supported") and the driver then crashes at control-loop start.
//
// Build: bash two_robot/tools/build_kortex_home.sh   (needs the Kortex API 2.6.0 that mc_kortex downloads)
// Run:   KORTEX_IP=192.168.1.10 KORTEX_USER=… KORTEX_PASS=… kortex_home [--dry-run] [action-name]
//        --dry-run prints the arm state, the joint angles and the stored actions without moving anything.
//        --status  only reads the arm state and the joint feedback twice, one second apart (no servoing-mode
//                  change, no fault clearing, no action): what is the arm doing right now.
// The tool clears an arm fault, switches the arm to single-level servoing (which also recovers an arm left in
// low-level servoing by a killed driver), executes the action and waits up to 40 s for ACTION_END.
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <chrono>
#include <cmath>
#include <thread>
#include <cstdlib>
#include <future>
#include <iostream>
#include <string>
namespace k_api = Kinova::Api;
static const char * env(const char * n) { const char * v = std::getenv(n); if(!v) { std::cerr << "missing env " << n << "\n"; std::exit(2); } return v; }
int main(int argc, char ** argv)
{
  std::string ip = env("KORTEX_IP"), user = env("KORTEX_USER"), pass = env("KORTEX_PASS");
  std::string action_name = "Home"; bool dry = false; bool status = false;
  for(int i = 1; i < argc; ++i) { std::string a = argv[i]; if(a == "--dry-run") dry = true; else if(a == "--status") status = true; else action_name = a; }
  auto error_callback = [](k_api::KError err) { std::cerr << "kortex error: " << err.toString() << "\n"; };
  auto transport = new k_api::TransportClientTcp();
  auto router = new k_api::RouterClient(transport, error_callback);
  if(!transport->connect(ip, 10000)) { std::cerr << "connect failed\n"; return 1; }
  auto session_manager = new k_api::SessionManager(router);
  k_api::Session::CreateSessionInfo info; info.set_username(user); info.set_password(pass);
  info.set_session_inactivity_timeout(60000); info.set_connection_inactivity_timeout(2000);
  session_manager->CreateSession(info);
  auto base = new k_api::Base::BaseClient(router);
  auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);
  auto print_joints = [&](const char * tag) {
    auto fb = base_cyclic->RefreshFeedback(); std::cout << tag << " joints(deg):";
    for(int i = 0; i < fb.actuators_size(); ++i) std::cout << " " << fb.actuators(i).position();
    std::cout << "  gripper=" << (fb.interconnect().gripper_feedback().motor_size() ? fb.interconnect().gripper_feedback().motor(0).position() : -1) << "%\n";
  };
  auto state = base->GetArmState();
  std::cout << "arm state: " << k_api::Common::ArmState_Name(state.active_state()) << "\n";
  print_joints("before");
  if(status) {
    auto fb0 = base_cyclic->RefreshFeedback(); std::this_thread::sleep_for(std::chrono::seconds(1));
    auto fb1 = base_cyclic->RefreshFeedback(); double maxd = 0; double maxv = 0;
    for(int i = 0; i < fb1.actuators_size() && i < fb0.actuators_size(); ++i) {
      double d = std::abs(fb1.actuators(i).position() - fb0.actuators(i).position()); if(d > 180) d = 360 - d; maxd = std::max(maxd, d);
      maxv = std::max(maxv, (double)std::abs(fb1.actuators(i).velocity()));
      if(fb1.actuators(i).fault_bank_a() || fb1.actuators(i).fault_bank_b() || fb1.actuators(i).warning_bank_a())
        std::cout << "actuator " << i + 1 << " fault_a=" << fb1.actuators(i).fault_bank_a() << " fault_b=" << fb1.actuators(i).fault_bank_b() << " warning_a=" << fb1.actuators(i).warning_bank_a() << "\n";
    }
    std::cout << "base fault_a=" << fb1.base().fault_bank_a() << " fault_b=" << fb1.base().fault_bank_b() << " active_state=" << k_api::Common::ArmState_Name(fb1.base().active_state()) << "\n";
    std::cout << "motion over 1 s: max joint change " << maxd << " deg, max |velocity| " << maxv << " deg/s -> " << (maxd > 0.2 || maxv > 1.0 ? "MOVING" : "still") << "\n";
    print_joints("now");
    session_manager->CloseSession(); router->SetActivationStatus(false); transport->disconnect();
    delete base_cyclic; delete base; delete session_manager; delete router; delete transport; return 0;
  }
  if(state.active_state() == k_api::Common::ARMSTATE_IN_FAULT) { std::cout << "arm in fault: clearing\n"; base->ClearFaults(); std::this_thread::sleep_for(std::chrono::seconds(1)); std::cout << "arm state now: " << k_api::Common::ArmState_Name(base->GetArmState().active_state()) << "\n"; }
  k_api::Base::ServoingModeInformation sm; sm.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING); base->SetServoingMode(sm);
  k_api::Base::RequestedActionType type; type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
  auto actions = base->ReadAllActions(type);
  k_api::Base::ActionHandle handle; handle.set_identifier(0);
  std::cout << "joint actions on robot:";
  for(const auto & a : actions.action_list()) { std::cout << " [" << a.name() << "]"; if(a.name() == action_name) handle = a.handle(); }
  std::cout << "\n";
  int rc = 0;
  if(handle.identifier() == 0) { std::cerr << "action '" << action_name << "' not found\n"; rc = 3; }
  else if(dry) { std::cout << "dry run: not executing\n"; }
  else {
    std::promise<k_api::Base::ActionEvent> done; auto fut = done.get_future();
    auto sub = base->OnNotificationActionTopic([&done](k_api::Base::ActionNotification n) {
        auto ev = n.action_event();
        if(ev == k_api::Base::ACTION_END || ev == k_api::Base::ACTION_ABORT) { try { done.set_value(ev); } catch(...) {} } },
      k_api::Common::NotificationOptions());
    std::cout << "executing '" << action_name << "'\n";
    base->ExecuteActionFromReference(handle);
    auto st = fut.wait_for(std::chrono::seconds(40));
    base->Unsubscribe(sub);
    if(st != std::future_status::ready) { std::cerr << "timeout waiting for action end\n"; rc = 4; }
    else { auto ev = fut.get(); std::cout << "action event: " << k_api::Base::ActionEvent_Name(ev) << "\n"; if(ev != k_api::Base::ACTION_END) rc = 5; }
    print_joints("after");
  }
  session_manager->CloseSession(); router->SetActivationStatus(false); transport->disconnect();
  delete base_cyclic; delete base; delete session_manager; delete router; delete transport;
  return rc;
}
