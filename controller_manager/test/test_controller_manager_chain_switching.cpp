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

// Tests for controller chain switching: topological sorting and AUTO/FORCE_AUTO strictness.
//
// N-structure topology shared by all tests:
//
//   position_tracking ──cmd──→ diff_drive ──cmd──┬──→ pid_left ──cmd──→ HW
//                                                └──→ pid_right ──cmd──→ HW
//   velocity_cmd ──cmd───────────────────────────┬──→ pid_left
//                                                └──→ pid_right
//   odom_publisher ──state──→ diff_drive
//
// diff_drive and velocity_cmd both command the PID reference interfaces, so they can never run
// at the same time. odom_publisher only reads diff_drive's exported state, which makes it a
// dependent of diff_drive without competing for any resource.

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

// Common topology, helpers and expectations. Derived fixtures only decide whether a test is
// parameterized over STRICT/BEST_EFFORT.
class ChainSwitchingFixture : public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  static constexpr char PID_LEFT[] = "pid_left_ctrl";
  static constexpr char PID_RIGHT[] = "pid_right_ctrl";
  static constexpr char DIFF_DRIVE[] = "diff_drive_ctrl";
  static constexpr char POSITION_TRACKING[] = "position_tracking_ctrl";
  static constexpr char VELOCITY_CMD[] = "velocity_cmd_ctrl";
  static constexpr char ODOM_PUBLISHER[] = "odom_publisher_ctrl";

  static constexpr int32_t AUTO = controller_manager_msgs::srv::SwitchController::Request::AUTO;
  static constexpr int32_t FORCE_AUTO =
    controller_manager_msgs::srv::SwitchController::Request::FORCE_AUTO;

  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // Raise the joint velocity limits so that the limiter never clamps a command and interferes
    // with the switching behaviour under test.
    const std::regex velocity_pattern(R"(velocity\s*=\s*"-?[0-9]+(\.[0-9]+)?")");
    const std::string diffbot_urdf = std::regex_replace(
      ros2_control_test_assets::diffbot_urdf, velocity_pattern, R"(velocity="10000.0")");
    cm_ = std::make_shared<controller_manager::ControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(
        diffbot_urdf, rm_node_->get_node_clock_interface(), rm_node_->get_node_logging_interface(),
        true),
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
    cm_->add_controller(pid_left, PID_LEFT, test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
    cm_->add_controller(
      pid_right, PID_RIGHT, test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
    cm_->add_controller(
      diff_drive, DIFF_DRIVE, test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
    cm_->add_controller(
      position_tracking, POSITION_TRACKING, test_controller::TEST_CONTROLLER_CLASS_NAME);
    cm_->add_controller(velocity_cmd, VELOCITY_CMD, test_controller::TEST_CONTROLLER_CLASS_NAME);
    cm_->add_controller(
      odom_publisher, ODOM_PUBLISHER, test_controller::TEST_CONTROLLER_CLASS_NAME);
  }

  void ConfigureControllers(const std::vector<std::string> & names)
  {
    ControllerManagerRunner<controller_manager::ControllerManager> cm_runner(this);
    for (const auto & name : names)
    {
      ASSERT_EQ(controller_interface::return_type::OK, cm_->configure_controller(name))
        << "Failed to configure " << name;
    }
  }

  // Set up the full N-structure with every controller in 'inactive' state.
  void PrepareAllControllers()
  {
    SetupNStructureControllers();
    AddAllControllers();
    ConfigureControllers(
      {PID_LEFT, PID_RIGHT, DIFF_DRIVE, POSITION_TRACKING, VELOCITY_CMD, ODOM_PUBLISHER});
  }

  void ExpectState(const std::string & ctrl_name, uint8_t expected_state)
  {
    auto loaded = cm_->get_loaded_controllers();
    auto it = std::find_if(
      loaded.begin(), loaded.end(), [&](const auto & spec) { return spec.info.name == ctrl_name; });
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

  // Bring up the diff_drive branch the manual way: dependencies first, one switch per level.
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

  std::shared_ptr<test_chainable_controller::TestChainableController> pid_left;
  std::shared_ptr<test_chainable_controller::TestChainableController> pid_right;
  std::shared_ptr<test_chainable_controller::TestChainableController> diff_drive;
  std::shared_ptr<test_controller::TestController> position_tracking;
  std::shared_ptr<test_controller::TestController> velocity_cmd;
  std::shared_ptr<test_controller::TestController> odom_publisher;
};

// For behaviour that must hold identically under STRICT and BEST_EFFORT.
class TestChainSwitchingStrictness : public ChainSwitchingFixture,
                                     public testing::WithParamInterface<Strictness>
{
};

// For AUTO / FORCE_AUTO behaviour. These pick their own strictness, so parameterizing them over
// STRICT/BEST_EFFORT would run the same scenario twice.
class TestChainSwitching : public ChainSwitchingFixture
{
};

// =============================================================================
// Topological sorting: the order controllers appear in the request must not matter
// =============================================================================

// The manager sorts the activation list itself, so listing a chain top-down works.
TEST_P(TestChainSwitchingStrictness, activate_chain_wrong_order)
{
  PrepareAllControllers();

  switch_test_controllers(
    {POSITION_TRACKING, DIFF_DRIVE, PID_LEFT, PID_RIGHT}, {}, GetParam().strictness);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
}

// Deactivation releases the chained interfaces; re-activating the same chain has to work again.
TEST_P(TestChainSwitchingStrictness, deactivate_all_reactivate_wrong_order)
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

// Handing over the PID reference interfaces from diff_drive to velocity_cmd in a single switch.
// The PIDs are shared by both branches and stay active throughout.
TEST_P(TestChainSwitchingStrictness, atomic_switch_diff_drive_to_velocity_cmd)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();

  switch_test_controllers({VELOCITY_CMD}, {POSITION_TRACKING, DIFF_DRIVE}, GetParam().strictness);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
  ExpectActive(VELOCITY_CMD);
}

// The same handover in the opposite direction.
TEST_P(TestChainSwitchingStrictness, atomic_switch_velocity_cmd_to_diff_drive)
{
  PrepareAllControllers();
  ActivateVelocityCmdChainBottomUp();

  switch_test_controllers({DIFF_DRIVE, POSITION_TRACKING}, {VELOCITY_CMD}, GetParam().strictness);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectInactive(VELOCITY_CMD);
}

// A state-only consumer can be activated in the same request as the chain it reads from.
TEST_P(TestChainSwitchingStrictness, activate_independent_chains)
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

// propagate_deactivation_of_chained_mode() has to keep scanning the stop list after it meets a
// controller that is not active. If it stopped early, diff_drive's followers would never be added
// to the 'from chained mode' request and the PIDs would stay stuck in chained mode.
TEST_F(TestChainSwitching, deactivate_active_controller_after_inactive_one_in_list)
{
  PrepareAllControllers();
  switch_test_controllers({PID_LEFT, PID_RIGHT}, {}, STRICT);
  switch_test_controllers({DIFF_DRIVE}, {}, STRICT);

  // position_tracking is still inactive and precedes the active diff_drive in the stop list.
  // BEST_EFFORT prunes it and must still process diff_drive.
  switch_test_controllers({}, {POSITION_TRACKING, DIFF_DRIVE}, BEST_EFFORT);

  ExpectInactive(POSITION_TRACKING);
  ExpectInactive(DIFF_DRIVE);
  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);

  // Proof that the PIDs really left chained mode: diff_drive can re-claim their reference
  // interfaces. If they were still chained, this STRICT activation would fail.
  switch_test_controllers({DIFF_DRIVE}, {}, STRICT);
  ExpectActive(DIFF_DRIVE);
}

// =============================================================================
// AUTO: pull in dependencies, refuse to stop anything the caller did not name
// =============================================================================

// Requesting the top of a chain activates everything below it.
TEST_F(TestChainSwitching, auto_expands_full_chain)
{
  PrepareAllControllers();

  switch_test_controllers({POSITION_TRACKING}, {}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
}

// Expansion follows dependencies downwards only: position_tracking commands diff_drive, so
// activating diff_drive must not drag position_tracking along.
TEST_F(TestChainSwitching, auto_expands_mid_chain)
{
  PrepareAllControllers();

  switch_test_controllers({DIFF_DRIVE}, {}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
}

// Dependencies that already run are not restarted, only the missing ones are activated.
TEST_F(TestChainSwitching, auto_skips_already_active)
{
  PrepareAllControllers();
  switch_test_controllers({PID_LEFT, PID_RIGHT}, {}, STRICT);

  switch_test_controllers({POSITION_TRACKING}, {}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
}

// A controller exporting state is a dependency too: odom_publisher cannot read diff_drive's
// odometry unless the whole diff_drive branch runs.
TEST_F(TestChainSwitching, auto_expands_state_providers)
{
  PrepareAllControllers();

  switch_test_controllers({ODOM_PUBLISHER}, {}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(ODOM_PUBLISHER);
  ExpectInactive(POSITION_TRACKING);
}

TEST_F(TestChainSwitching, auto_state_provider_skips_already_active)
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

// The reverse of auto_expands_state_providers: a controller reading diff_drive's exported state
// is an optional consumer, so activating diff_drive must not start odom_publisher.
TEST_F(TestChainSwitching, auto_does_not_expand_state_consumers)
{
  PrepareAllControllers();

  switch_test_controllers({POSITION_TRACKING}, {}, AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectInactive(ODOM_PUBLISHER);
}

// AUTO resolves dependencies but never picks victims; naming the blocker makes the switch legal.
TEST_F(TestChainSwitching, auto_with_explicit_deactivation)
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

// Without that explicit stop list the same request is rejected and nothing changes.
TEST_F(TestChainSwitching, auto_fails_on_conflict)
{
  PrepareAllControllers();
  ActivateVelocityCmdChainBottomUp();

  switch_test_controllers(
    {POSITION_TRACKING}, {}, AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(VELOCITY_CMD);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
}

// The conflict is detected even when it is introduced by an expanded state provider rather than
// by a controller the caller named.
TEST_F(TestChainSwitching, auto_fails_on_state_provider_conflict)
{
  PrepareAllControllers();
  ActivateVelocityCmdChainBottomUp();

  switch_test_controllers(
    {ODOM_PUBLISHER}, {}, AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(VELOCITY_CMD);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(ODOM_PUBLISHER);
}

// Two controllers in the same activation set claiming one resource cannot be satisfied by
// deactivating anything, so this is rejected before any conflict resolution.
TEST_F(TestChainSwitching, auto_rejects_impossible_combination)
{
  PrepareAllControllers();

  switch_test_controllers(
    {DIFF_DRIVE, VELOCITY_CMD}, {}, AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(VELOCITY_CMD);
}

// Same impossible combination, reached through expansion: odom_publisher pulls in diff_drive,
// which competes with the explicitly requested velocity_cmd.
TEST_F(TestChainSwitching, auto_rejects_state_cmd_conflict)
{
  PrepareAllControllers();

  switch_test_controllers(
    {ODOM_PUBLISHER, VELOCITY_CMD}, {}, AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectInactive(ODOM_PUBLISHER);
  ExpectInactive(VELOCITY_CMD);
}

// A dependency that is needed and deactivated in the same request contradicts itself.
TEST_F(TestChainSwitching, auto_rejects_dependency_that_is_also_deactivated)
{
  PrepareAllControllers();

  switch_test_controllers(
    {ODOM_PUBLISHER}, {DIFF_DRIVE}, AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectInactive(ODOM_PUBLISHER);
  ExpectInactive(DIFF_DRIVE);
}

TEST_F(TestChainSwitching, force_auto_rejects_dependency_that_is_also_deactivated)
{
  PrepareAllControllers();

  switch_test_controllers(
    {ODOM_PUBLISHER}, {DIFF_DRIVE}, FORCE_AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectInactive(ODOM_PUBLISHER);
  ExpectInactive(DIFF_DRIVE);
}

// AUTO applies STRICT semantics, so an unknown controller name aborts the whole request instead
// of being silently skipped.
TEST_F(TestChainSwitching, auto_rejects_unknown_controller)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();

  switch_test_controllers(
    {"no_such_controller"}, {}, AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
}

// Expansion can only activate controllers that reached 'inactive'. An unconfigured dependency is
// reported rather than skipped, and the whole switch is aborted.
TEST_F(TestChainSwitching, auto_rejects_unconfigured_dependency)
{
  SetupNStructureControllers();
  AddAllControllers();
  // pid_left stays 'unconfigured' while diff_drive, which commands it, is ready to run.
  ConfigureControllers({PID_RIGHT, DIFF_DRIVE, POSITION_TRACKING, VELOCITY_CMD, ODOM_PUBLISHER});

  switch_test_controllers(
    {DIFF_DRIVE}, {}, AUTO, std::future_status::ready, controller_interface::return_type::ERROR);

  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(PID_RIGHT);
  ExpectState(PID_LEFT, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

// Stopping a controller that others depend on requires naming all of them under AUTO. Here
// position_tracking (command chain) and odom_publisher (state) are both missing.
TEST_F(TestChainSwitching, auto_rejects_incomplete_stop_list)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();
  switch_test_controllers({ODOM_PUBLISHER}, {}, STRICT);

  switch_test_controllers(
    {}, {DIFF_DRIVE}, AUTO, std::future_status::ready, controller_interface::return_type::ERROR);

  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(ODOM_PUBLISHER);
}

// Narrower version of the above: the command-chain dependent is listed, so the state consumer is
// the only thing missing. Isolates the state-interface check from the command-chain check.
TEST_F(TestChainSwitching, auto_rejects_stop_list_missing_only_state_consumer)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();
  switch_test_controllers({ODOM_PUBLISHER}, {}, STRICT);

  switch_test_controllers(
    {}, {DIFF_DRIVE, POSITION_TRACKING}, AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(ODOM_PUBLISHER);
}

// Listing every dependent makes the same deactivation succeed.
TEST_F(TestChainSwitching, auto_accepts_complete_stop_list)
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

// Everything requested already runs, so expansion empties both lists. The switch returns OK
// immediately without waiting for an update cycle.
TEST_F(TestChainSwitching, auto_noop_when_requested_graph_already_active)
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

// =============================================================================
// FORCE_AUTO: pull in dependencies and stop whatever is in the way
// =============================================================================

// The blocker (diff_drive) and its dependent (position_tracking) are both stopped, without the
// caller naming either.
TEST_F(TestChainSwitching, force_auto_deactivates_conflict_and_upstream)
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

// The same switch in reverse: velocity_cmd is stopped and the diff_drive branch is built up.
TEST_F(TestChainSwitching, force_auto_reverse_n_switch)
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

// Requesting a mid-chain controller stops the sibling branch competing for the same PIDs.
TEST_F(TestChainSwitching, force_auto_mid_chain_deactivates_sibling)
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

// When the blocker has no dependents, only the blocker itself is stopped.
TEST_F(TestChainSwitching, force_auto_sibling_without_upstream)
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

// Propagation follows exported state interfaces as well: odom_publisher loses its state source
// when diff_drive stops, so it has to stop too.
TEST_F(TestChainSwitching, force_auto_deactivates_state_dependents)
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

// A conflict introduced by an expanded state provider is resolved the same way as a direct one.
TEST_F(TestChainSwitching, force_auto_state_provider_conflict)
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

// Two branches that do not compete are both expanded and activated in one request.
TEST_F(TestChainSwitching, force_auto_expands_independent_chains)
{
  PrepareAllControllers();

  switch_test_controllers({POSITION_TRACKING, ODOM_PUBLISHER}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(POSITION_TRACKING);
  ExpectActive(ODOM_PUBLISHER);
}

// Dependency expansion is identical to AUTO; only conflict handling differs.
TEST_F(TestChainSwitching, force_auto_expands_state_providers)
{
  PrepareAllControllers();

  switch_test_controllers({ODOM_PUBLISHER}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectActive(DIFF_DRIVE);
  ExpectActive(ODOM_PUBLISHER);
  ExpectInactive(POSITION_TRACKING);
}

// FORCE_AUTO only stops what is actually in the way. odom_publisher claims no command interface,
// so the running diff_drive branch is left alone.
TEST_F(TestChainSwitching, force_auto_no_unnecessary_deactivation)
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

// A state consumer survives as long as its provider keeps running.
TEST_F(TestChainSwitching, force_auto_odom_survives_when_chain_stays)
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

// FORCE_AUTO resolves conflicts with running controllers, but a set that conflicts with itself
// stays impossible.
TEST_F(TestChainSwitching, force_auto_rejects_impossible_combination)
{
  PrepareAllControllers();

  switch_test_controllers(
    {DIFF_DRIVE, VELOCITY_CMD}, {}, FORCE_AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(VELOCITY_CMD);
}

TEST_F(TestChainSwitching, force_auto_rejects_state_cmd_conflict)
{
  PrepareAllControllers();

  switch_test_controllers(
    {ODOM_PUBLISHER, VELOCITY_CMD}, {}, FORCE_AUTO, std::future_status::ready,
    controller_interface::return_type::ERROR);

  ExpectInactive(ODOM_PUBLISHER);
  ExpectInactive(VELOCITY_CMD);
}

// A controller the caller named explicitly must still hand its dependents to the propagation
// walk, otherwise position_tracking would keep running with no command target.
TEST_F(TestChainSwitching, force_auto_propagates_through_explicit_deactivate_entry)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();

  switch_test_controllers({VELOCITY_CMD}, {DIFF_DRIVE}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
  ExpectActive(VELOCITY_CMD);
}

// Propagation from an explicit stop list reaches both kinds of dependent: position_tracking
// through the command chain and odom_publisher through the exported state interfaces. The PIDs
// depend on nothing being stopped and keep running.
TEST_F(TestChainSwitching, force_auto_explicit_deactivate_propagates_to_all_dependents)
{
  PrepareAllControllers();
  ActivateDiffDriveChainBottomUp();
  switch_test_controllers({ODOM_PUBLISHER}, {}, STRICT);

  switch_test_controllers({}, {DIFF_DRIVE}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectActive(PID_RIGHT);
  ExpectInactive(DIFF_DRIVE);
  ExpectInactive(POSITION_TRACKING);
  ExpectInactive(ODOM_PUBLISHER);
}

// Providing state to a blocker is not itself a reason to be stopped. Only controllers that lose
// a resource they need are taken down.
//
// Extra topology for this test:
//   state_filter ──state──→ wheel_left/position (hardware state), exports state "sig"
//   sig_consumer ──state──→ state_filter/sig, ──cmd──→ wheel_left/velocity
//
// Activating pid_left claims wheel_left/velocity, so sig_consumer is a blocker and stops.
// state_filter holds no conflicting interface and nothing it provides disappears, so it stays.
TEST_F(TestChainSwitching, force_auto_keeps_state_only_provider_of_a_blocker)
{
  static constexpr char STATE_FILTER[] = "state_filter_ctrl";
  static constexpr char SIG_CONSUMER[] = "sig_consumer_ctrl";

  auto state_filter = std::make_shared<test_chainable_controller::TestChainableController>();
  state_filter->set_command_interface_configuration(
    {controller_interface::interface_configuration_type::NONE, {}});
  state_filter->set_state_interface_configuration(
    {controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_left/position"}});
  state_filter->set_reference_interface_names({});
  state_filter->set_exported_state_interface_names({"sig"});

  auto sig_consumer = std::make_shared<test_controller::TestController>();
  sig_consumer->set_command_interface_configuration(
    {controller_interface::interface_configuration_type::INDIVIDUAL, {"wheel_left/velocity"}});
  sig_consumer->set_state_interface_configuration(
    {controller_interface::interface_configuration_type::INDIVIDUAL,
     {std::string(STATE_FILTER) + "/sig"}});

  // All controllers have to be added before the executor starts spinning.
  SetupNStructureControllers();
  AddAllControllers();
  cm_->add_controller(
    state_filter, STATE_FILTER, test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->add_controller(sig_consumer, SIG_CONSUMER, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ConfigureControllers(
    {PID_LEFT, PID_RIGHT, DIFF_DRIVE, POSITION_TRACKING, VELOCITY_CMD, ODOM_PUBLISHER, STATE_FILTER,
     SIG_CONSUMER});

  // state_filter first, so that its exported "sig" interface is available to sig_consumer.
  switch_test_controllers({STATE_FILTER}, {}, STRICT);
  switch_test_controllers({SIG_CONSUMER}, {}, STRICT);
  ExpectActive(STATE_FILTER);
  ExpectActive(SIG_CONSUMER);

  switch_test_controllers({PID_LEFT}, {}, FORCE_AUTO);

  ExpectActive(PID_LEFT);
  ExpectInactive(SIG_CONSUMER);
  ExpectActive(STATE_FILTER);
}

// Nothing to do and nothing to stop: the extra deactivation propagation must not fire.
TEST_F(TestChainSwitching, force_auto_noop_when_requested_graph_already_active)
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
  test_strict_best_effort, TestChainSwitchingStrictness, testing::Values(strict, best_effort));
