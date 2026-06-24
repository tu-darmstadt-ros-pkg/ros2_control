// Copyright 2025 Team Hector
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
  FRIEND_TEST(
    TestChainSwitching, force_auto_does_not_deactivate_state_only_provider_of_conflict_candidate);
  FRIEND_TEST(
    TestChainSwitching,
    force_auto_explicit_deactivate_state_provider_propagates_to_dependents);
  FRIEND_TEST(
    TestChainSwitching,
    auto_explicit_deactivate_state_provider_requires_complete_stop_list);
  FRIEND_TEST(
    TestChainSwitching,
    auto_or_force_auto_rejects_dependency_also_explicitly_deactivated);
  FRIEND_TEST(
    TestChainSwitching,
    auto_explicit_deactivate_missing_only_state_consumer_fails);
  FRIEND_TEST(
    TestChainSwitching,
    auto_explicit_deactivate_complete_stop_list_succeeds);
  FRIEND_TEST(TestChainSwitching, auto_noop_when_requested_graph_already_active);
  FRIEND_TEST(TestChainSwitching, force_auto_noop_when_requested_graph_already_active);

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

// =============================================================================
// Proposed test: state-provider isolation under FORCE_AUTO
// =============================================================================

// Documentation contract (SwitchController.srv):
//   FORCE_AUTO "will deactivate any controllers that BLOCK THE ACTIVATION of
//   the requested controller, following the MUTUALLY EXCLUSIVE JOINT INTERFACE
//   SWITCHING PRINCIPLE."
//
// A controller that only provides state to a conflict candidate does NOT block
// activation through any command interface.  FORCE_AUTO must not deactivate it.
//
// Topology:
//   state_filter ──state──→ wheel_left/position (HW state, not used by PID chain)
//   state_filter exports state "sig"
//   sig_consumer reads state_filter/sig  AND commands wheel_left/velocity
//                (the latter conflicts with pid_left)
//
//   Build:   controller_chain_spec_[sig_consumer].preceding_controllers
//            contains state_filter (via the state-interface path in
//            build_controllers_topology_info lines ~5109-5110).
//
// When FORCE_AUTO activates pid_left (or anything that needs wheel_left/velocity):
//   - sig_consumer is a conflict candidate → added to deactivate list.
//   - The FORCE_AUTO expansion loop walks sig_consumer.preceding_controllers.
//   - BUG: state_filter is in that list, so it is incorrectly deactivated.
//   - CORRECT behaviour per doc: state_filter does not hold any interface that
//     blocks activation → it must remain active.
//
// This test will PASS once the FORCE_AUTO walk is restricted to command-chain
// predecessors only (from controller_chained_reference_interfaces_cache_)
// instead of the mixed preceding_controllers list.
TEST_P(
  TestChainSwitching,
  force_auto_does_not_deactivate_state_only_provider_of_conflict_candidate)
{
  static constexpr char STATE_FILTER[] = "state_filter_ctrl";
  static constexpr char SIG_CONSUMER[] = "sig_consumer_ctrl";

  // state_filter: chainable, reads wheel_left/position (state-only HW interface)
  // and exports a state interface "sig".  No command interfaces — pure state
  // processor.  Does NOT conflict with pid_left.
  auto state_filter = std::make_shared<test_chainable_controller::TestChainableController>();
  state_filter->set_command_interface_configuration(
    {controller_interface::interface_configuration_type::NONE, {}});
  state_filter->set_state_interface_configuration(
    {controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_left/position"}});
  state_filter->set_reference_interface_names({});
  state_filter->set_exported_state_interface_names({"sig"});

  // sig_consumer: reads state_filter/sig AND commands wheel_left/velocity —
  // the command interface conflicts with pid_left.
  auto sig_consumer = std::make_shared<test_controller::TestController>();
  sig_consumer->set_command_interface_configuration(
    {controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_left/velocity"}});
  sig_consumer->set_state_interface_configuration(
    {controller_interface::interface_configuration_type::INDIVIDUAL,
     {std::string(STATE_FILTER) + "/sig"}});

  // Add and configure ALL controllers before any executor activity.
  // add_controller() must not be called while the executor is already spinning.
  SetupNStructureControllers();
  AddAllControllers();
  cm_->add_controller(
    state_filter, STATE_FILTER, test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(
    sig_consumer, SIG_CONSUMER, test_controller::TEST_CONTROLLER_CLASS_NAME);

  {
    ControllerManagerRunner<TestableControllerManager> cm_runner(this);
    for (const auto & name :
         {PID_LEFT, PID_RIGHT, DIFF_DRIVE, POSITION_TRACKING, VELOCITY_CMD, ODOM_PUBLISHER,
          STATE_FILTER, SIG_CONSUMER})
    {
      ASSERT_EQ(controller_interface::return_type::OK, cm_->configure_controller(name))
        << "Failed to configure " << name;
    }
  }

  // Activate state_filter first so its exported state "sig" becomes available.
  switch_test_controllers({STATE_FILTER}, {}, STRICT);
  // Activate sig_consumer: it reads state_filter/sig (now available) and
  // commands wheel_left/velocity (conflict with pid_left).
  switch_test_controllers({SIG_CONSUMER}, {}, STRICT);

  ExpectActive(STATE_FILTER);
  ExpectActive(SIG_CONSUMER);

  // FORCE_AUTO activate pid_left: wheel_left/velocity is held by sig_consumer →
  // sig_consumer must be deactivated.
  // state_filter only provides state to sig_consumer; it holds no conflicting
  // command interface → per the doc it must NOT be deactivated.
  // Bug: preceding_controllers[sig_consumer] contains state_filter (state path),
  // so the FORCE_AUTO walk incorrectly adds state_filter to the deactivate list.
  switch_test_controllers({PID_LEFT}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectInactive(SIG_CONSUMER);  // correctly stopped — held conflicting interface
  ExpectActive(STATE_FILTER);    // must survive — state-only provider, not a blocker
}

// =============================================================================
// Regression tests for bugs found during review
// =============================================================================

// Bug: Step 4 early-continue skips upstream propagation when a controller is
// already in the user-supplied deactivate list.
//
// Topology: position_tracking ──cmd──→ diff_drive ──cmd──→ pid_left/pid_right
// velocity_cmd also commands pid_left/pid_right.
//
// Active state: full diff_drive chain (position_tracking + diff_drive + PIDs).
// User request: activate=[velocity_cmd], deactivate=[diff_drive] (explicit).
//
// FORCE_AUTO must recognise that position_tracking depends on diff_drive and
// auto-add position_tracking to the deactivate list even though diff_drive is
// already explicitly listed.  Without the fix, the early-continue in the
// FORCE_AUTO expansion loop skips diff_drive (already in deactivate_request),
// so position_tracking is never enqueued and ends up active while its command
// target (diff_drive) is gone.
TEST_P(TestChainSwitching, force_auto_propagates_through_explicit_deactivate_entry)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();

  // User explicitly lists diff_drive for deactivation; velocity_cmd should
  // activate and the whole upstream of diff_drive must be cleaned up.
  switch_test_controllers({VELOCITY_CMD}, {DIFF_DRIVE}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
  ExpectActive(VELOCITY_CMD);
}

// Bug: propagate_deactivation_of_chained_mode uses `break` instead of
// `continue` when it encounters an inactive controller in the deactivate list.
// This aborts the whole pass, so a subsequent active controller in the list
// never gets its followers added to from_chained_mode_request.
//
// Expose this by including an already-inactive controller in the explicit stop
// list alongside diff_drive which is active and chainable.  Without the fix,
// diff_drive's followers (pid_left, pid_right) are not switched out of chained
// mode and the switch fails or leaves the system in an inconsistent state.
TEST_P(TestChainSwitching, deactivate_active_controller_after_inactive_one_in_list)
{
  PrepareAllControllers();
  // Only diff_drive and its PIDs are active; position_tracking stays inactive.
  switch_test_controllers({PID_LEFT, PID_RIGHT}, {}, STRICT);
  switch_test_controllers({DIFF_DRIVE}, {}, STRICT);

  // Stop list: position_tracking (already inactive) first, then diff_drive
  // (active + chainable).  With the break→continue fix, BEST_EFFORT prunes
  // position_tracking and still processes diff_drive, switching the PIDs out
  // of chained mode.  Without the fix, the break exits after position_tracking
  // and diff_drive's followers never get added to from_chained_mode_request,
  // causing the subsequent STRICT activation of diff_drive to fail.
  switch_test_controllers({}, {POSITION_TRACKING, DIFF_DRIVE}, BEST_EFFORT);

  ExpectInactive(POSITION_TRACKING);
  ExpectInactive(DIFF_DRIVE);
  // PIDs should have left chained mode; they remain active and are usable.
  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);

  // Prove that PIDs actually left chained mode: re-activate diff_drive with STRICT.
  // If the break→continue bug were present, pid_left/pid_right would still be in
  // chained mode and diff_drive activation would fail because it could not re-claim
  // their reference interfaces.
  switch_test_controllers({DIFF_DRIVE}, {}, STRICT);
  ExpectActive(DIFF_DRIVE);
}

// FORCE_AUTO must deactivate state consumers of a disappearing controller.
// odom_publisher reads diff_drive's exported state; when FORCE_AUTO deactivates
// diff_drive (conflict with velocity_cmd), odom_publisher's state source
// disappears and it must also be stopped.  This is handled via
// controller_chained_state_interfaces_cache_ (not preceding_controllers).
//
// Topology:
//   odom_publisher ──state──→ diff_drive
//   velocity_cmd   ──cmd───→ pid_left/pid_right (conflicts with diff_drive)
//
// Note: this covers the state-consumer deactivation path.  For the
// complementary case (a state-only *provider* of a conflict candidate must
// NOT be deactivated) see force_auto_does_not_deactivate_state_only_provider_of_conflict_candidate.
TEST_P(TestChainSwitching, force_auto_does_not_deactivate_unrelated_state_provider)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();
  switch_test_controllers({ODOM_PUBLISHER}, {}, STRICT);

  // Switch to velocity_cmd: diff_drive and its upstream (position_tracking)
  // must be deactivated; odom_publisher must be deactivated because its state
  // source (diff_drive) disappears.  PIDs and velocity_cmd should be active.
  switch_test_controllers({VELOCITY_CMD}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
  ExpectInactive(ODOM_PUBLISHER);
  ExpectActive(VELOCITY_CMD);
}

// When FORCE_AUTO is given an explicit deactivate=[DIFF_DRIVE], it must still
// propagate and stop everything that depended on diff_drive:
//   - position_tracking (command-chain predecessor of diff_drive)
//   - odom_publisher (state consumer of diff_drive)
// The PIDs are not in the conflict path and must remain active.
TEST_P(TestChainSwitching, force_auto_explicit_deactivate_state_provider_propagates_to_dependents)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();
  switch_test_controllers({ODOM_PUBLISHER}, {}, STRICT);

  // User explicitly lists only diff_drive; FORCE_AUTO must auto-stop the rest.
  switch_test_controllers({}, {DIFF_DRIVE}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);  // command-chain predecessor — auto-stopped
  ExpectInactive(ODOM_PUBLISHER);     // state consumer — auto-stopped
}

// AUTO must reject a deactivation that would strand a state consumer unless
// the caller also explicitly lists the consumer for deactivation.
// Topology: odom_publisher reads diff_drive state.
// deactivate=[DIFF_DRIVE] without also listing ODOM_PUBLISHER must fail:
// AUTO converts to STRICT semantics internally, so check_preceding_controllers
// rejects the deactivation of diff_drive while position_tracking is still active.
TEST_P(TestChainSwitching, auto_explicit_deactivate_state_provider_requires_complete_stop_list)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();
  switch_test_controllers({ODOM_PUBLISHER}, {}, STRICT);

  // AUTO does not auto-deactivate odom_publisher or position_tracking — the user
  // must list them.  AUTO converts to STRICT internally, so this always errors.
  switch_test_controllers(
    {}, {DIFF_DRIVE}, AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  // Nothing should have changed.
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(ODOM_PUBLISHER);
}

// Requesting activate=[ODOM_PUBLISHER] while deactivate=[DIFF_DRIVE] in the
// same call is contradictory: AUTO expansion of ODOM_PUBLISHER needs diff_drive
// active, but deactivate=[DIFF_DRIVE] removes it.  The call must be rejected.
TEST_P(
  TestChainSwitching, auto_or_force_auto_rejects_dependency_also_explicitly_deactivated)
{
  PrepareAllControllers();

  // AUTO: expanding odom_publisher pulls in diff_drive, but diff_drive is
  // explicitly being deactivated in the same call — impossible combination.
  switch_test_controllers(
    {ODOM_PUBLISHER}, {DIFF_DRIVE}, AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectInactive(ODOM_PUBLISHER);
  ExpectInactive(DIFF_DRIVE);

  // Same check for FORCE_AUTO.
  switch_test_controllers(
    {ODOM_PUBLISHER}, {DIFF_DRIVE}, FORCE_AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectInactive(ODOM_PUBLISHER);
  ExpectInactive(DIFF_DRIVE);
}

// deactivate=[DIFF_DRIVE, POSITION_TRACKING] while ODOM_PUBLISHER is active must
// fail because odom_publisher reads diff_drive's exported state — removing
// diff_drive would strand it.  check_preceding_controllers_for_deactivate
// enforces this via controller_chained_state_interfaces_cache_.
// This test isolates the state-consumer enforcement independently of the
// command-chain enforcement (POSITION_TRACKING is listed, so that path is clear).
TEST_P(TestChainSwitching, auto_explicit_deactivate_missing_only_state_consumer_fails)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();
  switch_test_controllers({ODOM_PUBLISHER}, {}, STRICT);

  // POSITION_TRACKING is listed so the command-chain check passes for DIFF_DRIVE.
  // The missing ODOM_PUBLISHER is the only reason this must fail.
  switch_test_controllers(
    {}, {DIFF_DRIVE, POSITION_TRACKING}, AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(ODOM_PUBLISHER);
}

// Positive counterpart: listing all three consumers succeeds and leaves only
// the PIDs active.
TEST_P(TestChainSwitching, auto_explicit_deactivate_complete_stop_list_succeeds)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();
  switch_test_controllers({ODOM_PUBLISHER}, {}, STRICT);

  switch_test_controllers({}, {DIFF_DRIVE, POSITION_TRACKING, ODOM_PUBLISHER}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
  ExpectInactive(ODOM_PUBLISHER);
}

// AUTO activate=[POSITION_TRACKING] when the full chain is already active:
// Step 5 prunes every controller from the activation list (all already active),
// leaving empty activate and deactivate lists — the switch returns immediately
// (no update cycle needed) with OK and no state changes.
TEST_P(TestChainSwitching, auto_noop_when_requested_graph_already_active)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();

  switch_test_controllers(
    {POSITION_TRACKING}, {}, AUTO, std::future_status::ready,
    controller_interface::return_type::OK);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
}

// Same as above for FORCE_AUTO — its extra deactivation propagation must not
// incorrectly stop anything when there are no conflicts and the graph is
// already active.  Also verifies the early-return path works under FORCE_AUTO.
TEST_P(TestChainSwitching, force_auto_noop_when_requested_graph_already_active)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();

  switch_test_controllers(
    {POSITION_TRACKING}, {}, FORCE_AUTO, std::future_status::ready,
    controller_interface::return_type::OK);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
}

INSTANTIATE_TEST_SUITE_P(
  test_strict_best_effort, TestChainSwitching, testing::Values(strict, best_effort));
