// Copyright 2025 Hector Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Tests for controller chain switching: topological sorting and AUTO/FORCE_AUTO modes.
//
// N-structure topology:
//
//   position_tracking ──cmd──→ diff_drive ──cmd──┬──→ pid_left ──cmd──→ HW
//                                                └──→ pid_right ──cmd──→ HW
//   velocity_cmd ──cmd───────────────────────────┬──→ pid_left
//                                                └──→ pid_right
//   odom_publisher ──state──→ diff_drive
//
// diff_drive and velocity_cmd share PID reference interfaces — mutually exclusive.

#include <memory>
#include <regex>
#include <string>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "gmock/gmock.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "test_chainable_controller/test_chainable_controller.hpp"
#include "test_controller/test_controller.hpp"

class TestChainSwitching;

class TestableControllerManager : public controller_manager::ControllerManager
{
  friend TestChainSwitching;

  // Topological sorting
  FRIEND_TEST(TestChainSwitching, activate_chain_wrong_order);
  FRIEND_TEST(TestChainSwitching, deactivate_all_reactivate_wrong_order);
  FRIEND_TEST(TestChainSwitching, atomic_switch_diff_drive_to_velocity_cmd);
  FRIEND_TEST(TestChainSwitching, atomic_switch_velocity_cmd_to_diff_drive);
  FRIEND_TEST(TestChainSwitching, activate_independent_chains);
  // AUTO
  FRIEND_TEST(TestChainSwitching, auto_expands_full_chain);
  FRIEND_TEST(TestChainSwitching, auto_expands_mid_chain);
  FRIEND_TEST(TestChainSwitching, auto_skips_already_active);
  FRIEND_TEST(TestChainSwitching, auto_expands_state_providers);
  FRIEND_TEST(TestChainSwitching, auto_state_provider_skips_already_active);
  FRIEND_TEST(TestChainSwitching, auto_does_not_expand_state_consumers);
  FRIEND_TEST(TestChainSwitching, auto_with_explicit_deactivation);
  FRIEND_TEST(TestChainSwitching, auto_fails_on_conflict);
  FRIEND_TEST(TestChainSwitching, auto_fails_on_state_provider_conflict);
  FRIEND_TEST(TestChainSwitching, auto_rejects_impossible_combination);
  FRIEND_TEST(TestChainSwitching, auto_rejects_state_cmd_conflict);
  // FORCE_AUTO
  FRIEND_TEST(TestChainSwitching, force_auto_deactivates_conflict_and_upstream);
  FRIEND_TEST(TestChainSwitching, force_auto_reverse_n_switch);
  FRIEND_TEST(TestChainSwitching, force_auto_mid_chain_deactivates_sibling);
  FRIEND_TEST(TestChainSwitching, force_auto_sibling_without_upstream);
  FRIEND_TEST(TestChainSwitching, force_auto_deactivates_state_dependents);
  FRIEND_TEST(TestChainSwitching, force_auto_state_provider_conflict);
  FRIEND_TEST(TestChainSwitching, force_auto_expands_independent_chains);
  FRIEND_TEST(TestChainSwitching, force_auto_expands_state_providers);
  FRIEND_TEST(TestChainSwitching, force_auto_no_unnecessary_deactivation);
  FRIEND_TEST(TestChainSwitching, force_auto_odom_survives_when_chain_stays);
  FRIEND_TEST(TestChainSwitching, force_auto_rejects_impossible_combination);
  FRIEND_TEST(TestChainSwitching, force_auto_rejects_state_cmd_conflict);

public:
  TestableControllerManager(
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & node_namespace = "",
    const rclcpp::NodeOptions & node_options = controller_manager::get_cm_node_options())
  : controller_manager::ControllerManager(
      std::move(resource_manager), executor, manager_node_name, node_namespace, node_options)
  {
  }
};

class TestChainSwitching
: public ControllerManagerFixture<TestableControllerManager>,
  public testing::WithParamInterface<Strictness>
{
public:
  static constexpr char PID_LEFT[] = "pid_left_ctrl";
  static constexpr char PID_RIGHT[] = "pid_right_ctrl";
  static constexpr char DIFF_DRIVE[] = "diff_drive_ctrl";
  static constexpr char POSITION_TRACKING[] = "position_tracking_ctrl";
  static constexpr char VELOCITY_CMD[] = "velocity_cmd_ctrl";
  static constexpr char ODOM_PUBLISHER[] = "odom_publisher_ctrl";

  static constexpr int32_t AUTO =
    controller_manager_msgs::srv::SwitchController::Request::AUTO;
  static constexpr int32_t FORCE_AUTO =
    controller_manager_msgs::srv::SwitchController::Request::FORCE_AUTO;

  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    const std::regex velocity_pattern(R"(velocity\s*=\s*"-?[0-9]+(\.[0-9]+)?")");
    const std::string diffbot_urdf = std::regex_replace(
      ros2_control_test_assets::diffbot_urdf, velocity_pattern, R"(velocity="10000.0")");
    cm_ = std::make_shared<TestableControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(
        diffbot_urdf, rm_node_->get_node_clock_interface(),
        rm_node_->get_node_logging_interface(), true),
      executor_, TEST_CM_NAME);
    run_updater_ = false;
  }

  void SetupNStructureControllers()
  {
    pid_left = std::make_shared<test_chainable_controller::TestChainableController>();
    pid_left->set_command_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_left/velocity"}});
    pid_left->set_state_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_left/velocity"}});
    pid_left->set_reference_interface_names({"velocity"});

    pid_right = std::make_shared<test_chainable_controller::TestChainableController>();
    pid_right->set_command_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_right/velocity"}});
    pid_right->set_state_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_right/velocity"}});
    pid_right->set_reference_interface_names({"velocity"});

    diff_drive = std::make_shared<test_chainable_controller::TestChainableController>();
    diff_drive->set_command_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL,
       {std::string(PID_LEFT) + "/velocity", std::string(PID_RIGHT) + "/velocity"}});
    diff_drive->set_state_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL,
       {"wheel_left/velocity", "wheel_right/velocity"}});
    diff_drive->set_reference_interface_names({"vel_x", "vel_y"});
    diff_drive->set_exported_state_interface_names({"odom_x", "odom_y"});

    velocity_cmd = std::make_shared<test_controller::TestController>();
    velocity_cmd->set_command_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL,
       {std::string(PID_LEFT) + "/velocity", std::string(PID_RIGHT) + "/velocity"}});
    velocity_cmd->set_state_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL,
       {"wheel_left/velocity", "wheel_right/velocity"}});

    position_tracking = std::make_shared<test_controller::TestController>();
    position_tracking->set_command_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL,
       {std::string(DIFF_DRIVE) + "/vel_x", std::string(DIFF_DRIVE) + "/vel_y"}});

    odom_publisher = std::make_shared<test_controller::TestController>();
    odom_publisher->set_command_interface_configuration(
      {controller_interface::interface_configuration_type::NONE, {}});
    odom_publisher->set_state_interface_configuration(
      {controller_interface::interface_configuration_type::INDIVIDUAL,
       {std::string(DIFF_DRIVE) + "/odom_x", std::string(DIFF_DRIVE) + "/odom_y"}});
  }

  void AddAllControllers()
  {
    cm_->add_controller(
      pid_left, PID_LEFT, test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
    cm_->add_controller(
      pid_right, PID_RIGHT, test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
    cm_->add_controller(
      diff_drive, DIFF_DRIVE, test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
    cm_->add_controller(
      position_tracking, POSITION_TRACKING, test_controller::TEST_CONTROLLER_CLASS_NAME);
    cm_->add_controller(
      velocity_cmd, VELOCITY_CMD, test_controller::TEST_CONTROLLER_CLASS_NAME);
    cm_->add_controller(
      odom_publisher, ODOM_PUBLISHER, test_controller::TEST_CONTROLLER_CLASS_NAME);
  }

  void ConfigureAllControllers()
  {
    ControllerManagerRunner<TestableControllerManager> cm_runner(this);
    for (const auto & name :
         {PID_LEFT, PID_RIGHT, DIFF_DRIVE, POSITION_TRACKING, VELOCITY_CMD, ODOM_PUBLISHER})
    {
      ASSERT_EQ(controller_interface::return_type::OK, cm_->configure_controller(name))
        << "Failed to configure " << name;
    }
  }

  void ExpectState(const std::string & ctrl_name, uint8_t expected_state)
  {
    auto loaded = cm_->get_loaded_controllers();
    auto it = std::find_if(loaded.begin(), loaded.end(), [&](const auto & spec) {
      return spec.info.name == ctrl_name;
    });
    ASSERT_NE(it, loaded.end()) << "Controller " << ctrl_name << " not found";
    EXPECT_EQ(expected_state, it->c->get_lifecycle_state().id())
      << "Controller " << ctrl_name << " in wrong state";
  }

  void ExpectActive(const std::string & name)
  {
    ExpectState(name, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  void ExpectInactive(const std::string & name)
  {
    ExpectState(name, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  }

  void ActivateDiffDriveChainBottomUp()
  {
    switch_test_controllers({PID_LEFT, PID_RIGHT}, {}, STRICT);
    switch_test_controllers({DIFF_DRIVE}, {}, STRICT);
    switch_test_controllers({POSITION_TRACKING}, {}, STRICT);
    ExpectActive(PID_LEFT);
    ExpectActive(PID_RIGHT);
    ExpectActive(DIFF_DRIVE);
    ExpectActive(POSITION_TRACKING);
  }

  void ActivateVelocityCmdChainBottomUp()
  {
    switch_test_controllers({PID_LEFT, PID_RIGHT}, {}, STRICT);
    switch_test_controllers({VELOCITY_CMD}, {}, STRICT);
    ExpectActive(PID_LEFT);
    ExpectActive(PID_RIGHT);
    ExpectActive(VELOCITY_CMD);
  }

  void PrepareAllControllers()
  {
    SetupNStructureControllers();
    AddAllControllers();
    ConfigureAllControllers();
  }

  std::shared_ptr<test_chainable_controller::TestChainableController> pid_left;
  std::shared_ptr<test_chainable_controller::TestChainableController> pid_right;
  std::shared_ptr<test_chainable_controller::TestChainableController> diff_drive;
  std::shared_ptr<test_controller::TestController> position_tracking;
  std::shared_ptr<test_controller::TestController> velocity_cmd;
  std::shared_ptr<test_controller::TestController> odom_publisher;
};

// =============================================================================
// Topological sorting: activation/deactivation order should not matter
// =============================================================================

// Top-down order (wrong dependency order) should succeed via internal sorting.
TEST_P(TestChainSwitching, activate_chain_wrong_order)
{
  PrepareAllControllers();

  switch_test_controllers(
    {POSITION_TRACKING, DIFF_DRIVE, PID_LEFT, PID_RIGHT}, {}, GetParam().strictness);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
}

// Deactivate all, then re-activate in wrong order. Interfaces are cleaned up
// during deactivation, so re-activation must still work.
TEST_P(TestChainSwitching, deactivate_all_reactivate_wrong_order)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();

  switch_test_controllers(
    {}, {POSITION_TRACKING, DIFF_DRIVE, PID_LEFT, PID_RIGHT}, GetParam().strictness);
  ExpectInactive(PID_LEFT);
  ExpectInactive(PID_RIGHT);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);

  switch_test_controllers(
    {POSITION_TRACKING, DIFF_DRIVE, PID_LEFT, PID_RIGHT}, {}, GetParam().strictness);
  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
}

// Atomic switch from diff_drive path to velocity_cmd. PIDs stay active (shared).
TEST_P(TestChainSwitching, atomic_switch_diff_drive_to_velocity_cmd)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();

  switch_test_controllers(
    {VELOCITY_CMD}, {POSITION_TRACKING, DIFF_DRIVE}, GetParam().strictness);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
  ExpectActive(VELOCITY_CMD);
}

// Atomic switch from velocity_cmd back to diff_drive chain.
TEST_P(TestChainSwitching, atomic_switch_velocity_cmd_to_diff_drive)
{
  PrepareAllControllers();
  ActivateVelocityCmdChainBottomUp();

  switch_test_controllers(
    {DIFF_DRIVE, POSITION_TRACKING}, {VELOCITY_CMD}, GetParam().strictness);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectInactive(VELOCITY_CMD);
}

// Non-conflicting controllers activate together regardless of order.
TEST_P(TestChainSwitching, activate_independent_chains)
{
  PrepareAllControllers();

  switch_test_controllers(
    {ODOM_PUBLISHER, PID_LEFT, PID_RIGHT, DIFF_DRIVE, POSITION_TRACKING}, {},
    GetParam().strictness);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectActive(ODOM_PUBLISHER);
}

// =============================================================================
// AUTO: expand dependencies, fail on unresolved conflicts
// =============================================================================

// Top-level controller expands to activate the entire command chain.
TEST_P(TestChainSwitching, auto_expands_full_chain)
{
  PrepareAllControllers();

  switch_test_controllers({POSITION_TRACKING}, {}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
}

// Mid-chain controller expands only its downstream dependencies.
TEST_P(TestChainSwitching, auto_expands_mid_chain)
{
  PrepareAllControllers();

  switch_test_controllers({DIFF_DRIVE}, {}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
}

// Already-active dependencies are skipped, only missing ones are activated.
TEST_P(TestChainSwitching, auto_skips_already_active)
{
  PrepareAllControllers();
  switch_test_controllers({PID_LEFT, PID_RIGHT}, {}, STRICT);

  switch_test_controllers({POSITION_TRACKING}, {}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
}

// State-interface providers are expanded (odom_publisher needs diff_drive chain).
TEST_P(TestChainSwitching, auto_expands_state_providers)
{
  PrepareAllControllers();

  switch_test_controllers({ODOM_PUBLISHER}, {}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(ODOM_PUBLISHER);
  ExpectInactive(POSITION_TRACKING);
}

// State provider already active — only odom_publisher itself is activated.
TEST_P(TestChainSwitching, auto_state_provider_skips_already_active)
{
  PrepareAllControllers();
  switch_test_controllers({PID_LEFT, PID_RIGHT}, {}, STRICT);
  switch_test_controllers({DIFF_DRIVE}, {}, STRICT);

  switch_test_controllers({ODOM_PUBLISHER}, {}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(ODOM_PUBLISHER);
  ExpectInactive(POSITION_TRACKING);
}

// State-interface consumers are NOT pulled in. position_tracking does not need odom_publisher.
TEST_P(TestChainSwitching, auto_does_not_expand_state_consumers)
{
  PrepareAllControllers();

  switch_test_controllers({POSITION_TRACKING}, {}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectInactive(ODOM_PUBLISHER);
}

// Explicit deactivation list resolves conflicts that AUTO cannot resolve itself.
TEST_P(TestChainSwitching, auto_with_explicit_deactivation)
{
  PrepareAllControllers();
  ActivateVelocityCmdChainBottomUp();

  switch_test_controllers({POSITION_TRACKING}, {VELOCITY_CMD}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectInactive(VELOCITY_CMD);
}

// AUTO fails when expanded chain conflicts with active controller (command interfaces).
TEST_P(TestChainSwitching, auto_fails_on_conflict)
{
  PrepareAllControllers();
  ActivateVelocityCmdChainBottomUp();

  switch_test_controllers(
    {POSITION_TRACKING}, {}, AUTO,
    std::future_status::ready, controller_interface::return_type::ERROR);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(VELOCITY_CMD);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
}

// AUTO fails when state-provider expansion creates a conflict with active controller.
TEST_P(TestChainSwitching, auto_fails_on_state_provider_conflict)
{
  PrepareAllControllers();
  ActivateVelocityCmdChainBottomUp();

  switch_test_controllers(
    {ODOM_PUBLISHER}, {}, AUTO,
    std::future_status::ready, controller_interface::return_type::ERROR);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(VELOCITY_CMD);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(ODOM_PUBLISHER);
}

// Two controllers in the activation set claiming the same resource is always impossible.
TEST_P(TestChainSwitching, auto_rejects_impossible_combination)
{
  PrepareAllControllers();

  switch_test_controllers(
    {DIFF_DRIVE, VELOCITY_CMD}, {}, AUTO,
    std::future_status::ready, controller_interface::return_type::ERROR);

  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(VELOCITY_CMD);
}

// odom_publisher expands to diff_drive which conflicts with velocity_cmd — impossible.
TEST_P(TestChainSwitching, auto_rejects_state_cmd_conflict)
{
  PrepareAllControllers();

  switch_test_controllers(
    {ODOM_PUBLISHER, VELOCITY_CMD}, {}, AUTO,
    std::future_status::ready, controller_interface::return_type::ERROR);

  ExpectInactive(ODOM_PUBLISHER);
  ExpectInactive(VELOCITY_CMD);
}

// =============================================================================
// FORCE_AUTO: expand dependencies AND auto-deactivate conflicts
// =============================================================================

// Deactivates conflicting controller and its upstream dependents.
TEST_P(TestChainSwitching, force_auto_deactivates_conflict_and_upstream)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();

  switch_test_controllers({VELOCITY_CMD}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
  ExpectActive(VELOCITY_CMD);
}

// Reverse: deactivates velocity_cmd, activates diff_drive chain for position_tracking.
TEST_P(TestChainSwitching, force_auto_reverse_n_switch)
{
  PrepareAllControllers();
  ActivateVelocityCmdChainBottomUp();

  switch_test_controllers({POSITION_TRACKING}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectInactive(VELOCITY_CMD);
}

// Mid-chain activation deactivates sibling (no upstream to propagate).
TEST_P(TestChainSwitching, force_auto_mid_chain_deactivates_sibling)
{
  PrepareAllControllers();
  ActivateVelocityCmdChainBottomUp();

  switch_test_controllers({DIFF_DRIVE}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectInactive(VELOCITY_CMD);
  ExpectInactive(POSITION_TRACKING);
}

// Sibling without upstream children — only the conflicting controller is deactivated.
TEST_P(TestChainSwitching, force_auto_sibling_without_upstream)
{
  PrepareAllControllers();
  switch_test_controllers({PID_LEFT, PID_RIGHT}, {}, STRICT);
  switch_test_controllers({DIFF_DRIVE}, {}, STRICT);

  switch_test_controllers({VELOCITY_CMD}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectInactive(DIFF_DRIVE);
  ExpectActive(VELOCITY_CMD);
}

// Controllers reading exported state interfaces must be deactivated when the
// provider is deactivated (state interfaces become unavailable).
TEST_P(TestChainSwitching, force_auto_deactivates_state_dependents)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();
  switch_test_controllers({ODOM_PUBLISHER}, {}, STRICT);

  switch_test_controllers({VELOCITY_CMD}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
  ExpectInactive(ODOM_PUBLISHER);
  ExpectActive(VELOCITY_CMD);
}

// State-provider conflict: odom_publisher needs diff_drive which conflicts with velocity_cmd.
// FORCE_AUTO deactivates velocity_cmd and activates the provider chain.
TEST_P(TestChainSwitching, force_auto_state_provider_conflict)
{
  PrepareAllControllers();
  ActivateVelocityCmdChainBottomUp();

  switch_test_controllers({ODOM_PUBLISHER}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(ODOM_PUBLISHER);
  ExpectInactive(VELOCITY_CMD);
  ExpectInactive(POSITION_TRACKING);
}

// Independent chains expand and activate without conflict.
TEST_P(TestChainSwitching, force_auto_expands_independent_chains)
{
  PrepareAllControllers();

  switch_test_controllers({POSITION_TRACKING, ODOM_PUBLISHER}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectActive(ODOM_PUBLISHER);
}

// State-interface providers are expanded (same as AUTO).
TEST_P(TestChainSwitching, force_auto_expands_state_providers)
{
  PrepareAllControllers();

  switch_test_controllers({ODOM_PUBLISHER}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(ODOM_PUBLISHER);
  ExpectInactive(POSITION_TRACKING);
}

// State-only controller alongside active chain — no conflict, stays active.
TEST_P(TestChainSwitching, force_auto_no_unnecessary_deactivation)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();

  switch_test_controllers({ODOM_PUBLISHER}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectActive(ODOM_PUBLISHER);
}

// odom_publisher survives when its state provider (diff_drive) stays active.
TEST_P(TestChainSwitching, force_auto_odom_survives_when_chain_stays)
{
  PrepareAllControllers();
  switch_test_controllers({PID_LEFT, PID_RIGHT}, {}, STRICT);
  switch_test_controllers({DIFF_DRIVE}, {}, STRICT);
  switch_test_controllers({ODOM_PUBLISHER}, {}, STRICT);

  switch_test_controllers({POSITION_TRACKING}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectActive(ODOM_PUBLISHER);
}

// Two controllers in the activation set claiming the same resource is always impossible.
TEST_P(TestChainSwitching, force_auto_rejects_impossible_combination)
{
  PrepareAllControllers();

  switch_test_controllers(
    {DIFF_DRIVE, VELOCITY_CMD}, {}, FORCE_AUTO,
    std::future_status::ready, controller_interface::return_type::ERROR);

  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(VELOCITY_CMD);
}

// odom_publisher expands to diff_drive which conflicts with velocity_cmd — impossible.
TEST_P(TestChainSwitching, force_auto_rejects_state_cmd_conflict)
{
  PrepareAllControllers();

  switch_test_controllers(
    {ODOM_PUBLISHER, VELOCITY_CMD}, {}, FORCE_AUTO,
    std::future_status::ready, controller_interface::return_type::ERROR);

  ExpectInactive(ODOM_PUBLISHER);
  ExpectInactive(VELOCITY_CMD);
}

INSTANTIATE_TEST_SUITE_P(
  test_strict_best_effort, TestChainSwitching, testing::Values(strict, best_effort));
