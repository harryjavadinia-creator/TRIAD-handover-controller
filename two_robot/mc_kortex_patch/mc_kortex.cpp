#include "mc_kortex.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace mc_kortex {

void *global_thread_init(
    mc_control::MCGlobalController::GlobalConfiguration &gconfig,
    bool start_control_threads) {
  auto kortexConfig = gconfig.config("Kortex");
  auto loop_data = new ControlLoopData();
  // Create mc_rtc's global controller
  loop_data->controller = new mc_control::MCGlobalController(gconfig);
  loop_data->kinova_threads = new std::vector<std::thread>();
  auto &controller = *loop_data->controller;
  if (controller.controller().timeStep < 0.001) {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_kortex] mc_rtc cannot run faster than 1kHz with mc_kortex");
  }
  size_t freq = std::ceil(1 / controller.controller().timeStep);
  mc_rtc::log::info("[mc_kortex] mc_rtc running at {}Hz", freq);
  auto &robots = controller.controller().robots();
  // Initialize all real robots
  for (size_t i = controller.realRobots().size(); i < robots.size(); ++i) {
    controller.realRobots().robotCopy(robots.robot(i), robots.robot(i).name());
  }

  // Initialize controlled kinova robot
  loop_data->kinovas = new std::vector<mc_kinova::KinovaRobotPtr>();
  auto &kinovas = *loop_data->kinovas;
  {
    std::vector<std::thread> kinova_init_threads;
    std::mutex kinova_init_mutex;
    std::condition_variable kinova_init_cv;
    bool kinovas_init_ready = false;
    for (auto &robot : robots) {
      if (robot.mb().nrDof() == 0) {
        continue;
      }
      if (kortexConfig.has(robot.name())) {
        std::string ip = kortexConfig(robot.name())("ip");
        std::string username = kortexConfig(robot.name())("username");
        std::string password = kortexConfig(robot.name())("password");
        const std::string robotName = robot.name();
        kinova_init_threads.emplace_back(
            [&, robotName, ip, username, password]() {
          {
            std::unique_lock<std::mutex> lock(kinova_init_mutex);
            kinova_init_cv.wait(
                lock, [&kinovas_init_ready]() { return kinovas_init_ready; });
          }
          auto kinova = std::unique_ptr<mc_kinova::KinovaRobot>(
              new mc_kinova::KinovaRobot(robotName, ip, username, password));
          std::unique_lock<std::mutex> lock(kinova_init_mutex);
          kinovas.emplace_back(std::move(kinova));
        });
      } else {
        mc_rtc::log::warning("The loaded controller uses an actuated robot "
                             "that is not configured and not ignored: {}",
                             robot.name());
      }
    }
    {
      std::lock_guard<std::mutex> lock(kinova_init_mutex);
      kinovas_init_ready = true;
    }
    kinova_init_cv.notify_all();
    for (auto &th : kinova_init_threads) {
      th.join();
    }
  }
  if(kinovas.empty())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_kortex] no configured actuated Kinova robot was loaded");
  }
  mc_rtc::log::info("[mc_kortex] initializing {} independent Kinova connection(s)",
                    kinovas.size());
  for (auto &kinova : kinovas) {
    kinova->init(controller, kortexConfig, start_control_threads);
  }

  // Initialize every mc_rtc model from its own physical Kinova encoder state.
  // The former single-vector initialization only initialized MainRobot and left
  // Robot B at its module zero posture, which made dual Cartesian hold unsafe.
  std::map<std::string, std::vector<double>> qInitByRobot;
  for(auto & kinova : kinovas)
  {
    const std::string name = kinova->getName();
    if(!controller.realRobots().hasRobot(name) || !robots.hasRobot(name))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_kortex] initialized hardware robot is missing from mc_rtc collections: {}",
          name);
    }
    const auto q = controller.realRobots().robot(name).encoderValues();
    const auto & rjo = robots.robot(name).refJointOrder();
    if(q.size() != rjo.size())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_kortex] per-robot initialization size mismatch robot={} encoderValues={} modelReferenceJoints={}",
          name, q.size(), rjo.size());
    }
    if(!std::all_of(q.begin(), q.end(),
                    [](double value) { return std::isfinite(value); }))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_kortex] per-robot initialization rejected non-finite state robot={}",
          name);
    }
    qInitByRobot[name] = q;
    mc_rtc::log::info("[mc_kortex] qInit[{}] = {}",
                      name, mc_kinova::printVec(q));
  }

  if(qInitByRobot.size() != kinovas.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_kortex] per-robot initialization map incomplete hardware={} mapped={}",
        kinovas.size(), qInitByRobot.size());
  }

  if(start_control_threads)
  {
    controller.init(qInitByRobot);
    controller.running = true;
    controller.controller().gui()->addElement(
        {"Kortex"}, mc_rtc::gui::Button("Stop controller", [&controller]() {
          controller.running = false;
        }));

    // Install per-robot logger entries sequentially. Logger registration is
    // not performed from the concurrent cyclic threads.
    for(auto & kinova : kinovas)
    {
      kinova->addLogEntry(controller);
    }

    // Start low-level control loops only for a normal run. The --init-only
    // preflight path deliberately stops before this point.
    static std::mutex startMutex;
    static std::condition_variable startCV;
    static bool startControl = false;
    for (auto &kinova : kinovas) {
      auto * kinovaPtr = kinova.get();
      loop_data->kinova_threads->emplace_back([&, kinovaPtr]() {
        kinovaPtr->controlThread(controller, startMutex, startCV, startControl,
                                 controller.running);
      });
    }
    {
      std::lock_guard<std::mutex> lock(startMutex);
      startControl = true;
    }
    startCV.notify_all();
    loop_data->control_threads_started = true;
  }
  else
  {
    controller.running = false;
    mc_rtc::log::success(
        "[mc_kortex] headless no-motion preflight PASS: independently validated {} physical robot state vector(s); full controller initialization, ROS services, GUI state publication, low-level servoing and cyclic command threads were not started",
        qInitByRobot.size());
  }

  return loop_data;
}

void run(void *data) {
  mc_rtc::log::info("[mc_kortex] Starting control loop");
  auto control_data = static_cast<mc_kortex::ControlLoopData *>(data);
  auto controller_ptr = control_data->controller;
  auto &controller = *controller_ptr;
  auto &kinovas = *control_data->kinovas;

  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  double now = 0;
  double last = ts.tv_sec * 1e6 + ts.tv_nsec * 1e-3;
  controller.controller().logger().addLogEntry(
      "perf_LoopDt", [&]() { return (now - last) / 1000; });

  while (controller.running) {
    clock_gettime(CLOCK_MONOTONIC, &ts);
    now = ts.tv_sec * 1e6 + ts.tv_nsec * 1e-3;
    if (now - last > controller.timestep() * 1e6) {
      // mc_rtc::log::info("[mc_kortex] Control loop elapsed time {}ms",
      // (now-last)*1e-3);
      for (auto &kinova : kinovas) {
        if (controller.controller().datastore().has("TorqueMode"))
          kinova->setTorqueMode(
              controller.controller().datastore().get<std::string>(
                  "TorqueMode"));
        if (controller.controller().datastore().has("ControlMode"))
          kinova->setControlMode(
              controller.controller().datastore().get<std::string>(
                  "ControlMode"));
        kinova->updateSensors(controller);
      }

      // Run the controller
      controller.run();

      for (auto &kinova : kinovas) {
        kinova->updateControl(controller);
      }

      last = now;
    }
  }

  shutdown(data);
}

void shutdown(void *data)
{
  if(!data) { return; }
  auto control_data = static_cast<mc_kortex::ControlLoopData *>(data);
  mc_rtc::log::info(
      "[mc_kortex] shutdown started controlThreadsStarted={}",
      control_data->control_threads_started);

  if(control_data->control_threads_started && control_data->kinovas)
  {
    for(auto & kinova : *control_data->kinovas)
    {
      kinova->stopController();
    }
  }

  if(control_data->kinova_threads)
  {
    for(auto & th : *control_data->kinova_threads)
    {
      if(th.joinable()) { th.join(); }
    }
  }

  if(control_data->control_threads_started && control_data->kinovas
     && control_data->controller)
  {
    for(auto & kinova : *control_data->kinovas)
    {
      kinova->cleanupControllerInterfaces(*control_data->controller);
    }
  }

  delete control_data->kinovas;
  delete control_data->kinova_threads;
  delete control_data->controller;
  delete control_data;
  mc_rtc::log::success("[mc_kortex] shutdown complete");
}

} // namespace mc_kortex
