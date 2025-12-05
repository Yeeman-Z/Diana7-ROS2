#pragma once

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace diana7_hardware {
    class Diana7System : public hardware_interface::SystemInterface {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(Diana7System)

            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo& info) override;
            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State& previous_state) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State& previous_state) override;
            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State& previous_state) override;
            hardware_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State& previous_state) override;
            hardware_interface::CallbackReturn on_shutdown(
                const rclcpp_lifecycle::State& previous_state) override;
            hardware_interface::CallbackReturn on_error(
                const rclcpp_lifecycle::State& previous_state) override;

            hardware_interface::return_type read(const rclcpp::Time& time,
                                                const rclcpp::Duration& period) override;
            hardware_interface::return_type write(const rclcpp::Time& time,
                                                const rclcpp::Duration& period) override;

        public:
            // Parameters
            std::vector<std::string> joint_names_;
            std::string ip_;
            int port_{0};
            int timeout_ms_{100};
            bool auto_enable_{false};

            // Mirrors
            std::vector<double> position_cmd_;
            std::vector<double> position_state_;
            std::vector<double> velocity_state_;

            // Connection status
            bool is_connected_{false};
    };
} // namespace diana7_hardware
