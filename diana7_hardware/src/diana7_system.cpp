#include "diana7_hardware/diana7_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

// Vendor C API
#include "DianaAPI.h"


namespace diana7_hardware {

hardware_interface::CallbackReturn Diana7System::on_init(
    const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    joint_names_.clear();
    for (const auto& joint : info.joints) {
        joint_names_.push_back(joint.name);
    }

    position_cmd_.assign(joint_names_.size(), 0.0);
    position_state_.assign(joint_names_.size(), 0.0);
    velocity_state_.assign(joint_names_.size(), 0.0);

    // Parameters (optional, for future real hardware)
    if (info.hardware_parameters.count("ip"))
        ip_ = info.hardware_parameters.at("ip");
    if (info.hardware_parameters.count("port"))
        port_ = std::stoi(info.hardware_parameters.at("port"));
    if (info.hardware_parameters.count("timeout_ms"))
        timeout_ms_ = std::stoi(info.hardware_parameters.at("timeout_ms"));
    if (info.hardware_parameters.count("auto_enable"))
        auto_enable_ = info.hardware_parameters.at("auto_enable") == "true";

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Diana7System::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("Diana7System"), "Hardware interface configured");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Diana7System::export_state_interfaces() {
    RCLCPP_INFO(rclcpp::get_logger("Diana7System"), "🙀 export_command_interfaces called!");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &position_state_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_state_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Diana7System::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &position_cmd_[i]));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn Diana7System::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("Diana7System"), "Hardware interface activated");

    // Initialize C API connection
    srv_net_st* pinfo = new srv_net_st();
    memset(pinfo->SrvIp, 0x00, sizeof(pinfo->SrvIp));
    memcpy(pinfo->SrvIp, ip_.c_str(), strlen(ip_.c_str()));
    pinfo->LocHeartbeatPort = 0;
    pinfo->LocRobotStatePort = 0;
    pinfo->LocSrvPort = 0;

    int ret = initSrv(nullptr, nullptr, pinfo);
    if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("Diana7System"),
                     "Failed to initialize service, returned: %d", ret);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Wait for connection to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Release brake
    ret = releaseBrake(ip_.c_str());
    if (ret == 0) {
        RCLCPP_INFO(rclcpp::get_logger("Diana7System"), "Brake released successfully");
    } else {
        RCLCPP_WARN(rclcpp::get_logger("Diana7System"), "Failed to release brake, returned: %d",
                    ret);
    }

    is_connected_ = true;

    // Reset states to current commands
    double joint_positions[7];
    if (getJointPos(joint_positions, ip_.c_str()) == 0) {
        for (size_t i = 0; i < position_cmd_.size() && i < 7; ++i) {
            position_cmd_[i] = joint_positions[i];
            position_state_[i] = joint_positions[i]; // 保证 delta = 0
        }
    }
    std::fill(velocity_state_.begin(), velocity_state_.end(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Diana7System::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    // Re-engage brake and disconnect
    if (is_connected_) {
        try {
            int ret = holdBrake(ip_.c_str());
            if (ret == 0) {
                RCLCPP_INFO(rclcpp::get_logger("Diana7System"), "Brake engaged successfully");
            } else {
                RCLCPP_WARN(rclcpp::get_logger("Diana7System"),
                            "Failed to engage brake, returned: %d", ret);
            }

            ret = destroySrv(ip_.c_str());
            if (ret == 0) {
                RCLCPP_INFO(rclcpp::get_logger("Diana7System"),
                            "Disconnected from robot successfully");
            } else {
                RCLCPP_WARN(rclcpp::get_logger("Diana7System"),
                            "Failed to disconnect, returned: %d", ret);
            }
        } catch (...) {
            RCLCPP_WARN(rclcpp::get_logger("Diana7System"), "Vendor deactivate sequence threw");
        }
        is_connected_ = false;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Diana7System::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("Diana7System"), "Hardware interface cleaned up");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Diana7System::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("Diana7System"), "Hardware interface shutdown");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Diana7System::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_ERROR(rclcpp::get_logger("Diana7System"), "Hardware interface error");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Diana7System::read(const rclcpp::Time& /*time*/,
                                                   const rclcpp::Duration& period) {
    if (!is_connected_) {
        return hardware_interface::return_type::OK;
    }

    try {
        // Get current joint positions from robot
        double joint_positions[7];
        int ret = getJointPos(joint_positions, ip_.c_str());
        if (ret == 0) {
            for (size_t i = 0; i < position_state_.size() && i < 7; ++i) {
                position_state_[i] = joint_positions[i];
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("Diana7System"),
                        "Failed to get joint positions, returned: %d", ret);
        }

        // Get current joint velocities from robot
        double joint_velocities[7];
        ret = getJointAngularVel(joint_velocities, ip_.c_str());
        if (ret == 0) {
            for (size_t i = 0; i < velocity_state_.size() && i < 7; ++i) {
                velocity_state_[i] = joint_velocities[i];
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("Diana7System"),
                        "Failed to get joint velocities, returned: %d", ret);
        }
    } catch (...) {
        RCLCPP_WARN(rclcpp::get_logger("Diana7System"), "Exception in read method");
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Diana7System::write(const rclcpp::Time& /*time*/,
                                                    const rclcpp::Duration& /*period*/) {
    if (!is_connected_) {
        return hardware_interface::return_type::OK;
    }

    try {
        // Check robot state
        char robot_state = getRobotState(ip_.c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("Diana7System"), "Robot state: %d", robot_state);

        if (robot_state != 2) {
            // Robot is moving or in error state, skip this command
            RCLCPP_INFO(rclcpp::get_logger("Diana7System"),
                        "Robot is busy (state=%d), skipping command", robot_state);
            return hardware_interface::return_type::OK;
        }

        // Check if there's a significant change in position
        bool has_movement = false;
        double max_delta = 0.0;
        for (size_t i = 0; i < position_cmd_.size(); ++i) {
            double delta = std::abs(position_cmd_[i] - position_state_[i]);
            if (delta > 0.001) // Reduced threshold to 0.001 rad ≈ 0.057 degrees
            {
                has_movement = true;
                max_delta = std::max(max_delta, delta);
            }
        }

        if (has_movement) {
            RCLCPP_INFO(rclcpp::get_logger("Diana7System"),
                        "Detected movement: max delta = %.6f rad (%.3f degrees)", max_delta,
                        max_delta * 180.0 / M_PI);

            // Send joint position command to robot
            double joint_positions[7];
            for (size_t i = 0; i < position_cmd_.size() && i < 7; ++i) {
                joint_positions[i] = position_cmd_[i];
            }

            RCLCPP_INFO(rclcpp::get_logger("Diana7System"),
                        "Sending moveJToTarget command: [%.4f, %.4f, %.4f, %.4f, "
                        "%.4f, %.4f, %.4f]",
                        joint_positions[0], joint_positions[1], joint_positions[2],
                        joint_positions[3], joint_positions[4], joint_positions[5],
                        joint_positions[6]);

            // Try different parameters if initial attempt fails
            int ret = moveJToTarget(joint_positions, 0.1, 0.05, 0, 0.0, 0.0, ip_.c_str());

            if (ret == 0) {
                RCLCPP_INFO(rclcpp::get_logger("Diana7System"),
                            "moveJToTarget command sent successfully");
                // Note: We don't update position_state_ here anymore
                // The actual joint positions will be read in the read() function
            } else {
                RCLCPP_WARN(rclcpp::get_logger("Diana7System"),
                            "moveJToTarget failed with code: %d", ret);

                // Try alternative parameters
                RCLCPP_INFO(rclcpp::get_logger("Diana7System"),
                            "Retrying with different parameters...");
                ret = moveJToTarget(joint_positions, 0.05, 0.05, 0, 0.0, 0.0, ip_.c_str());

                if (ret == 0) {
                    RCLCPP_INFO(rclcpp::get_logger("Diana7System"),
                                "Alternative moveJToTarget succeeded");
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("Diana7System"),
                                 "moveJToTarget failed with alternative parameters too: %d", ret);
                }
            }
        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("Diana7System"), "No significant movement detected");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Diana7System"), "Exception in write method: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("Diana7System"), "Unknown exception in write method");
    }

    return hardware_interface::return_type::OK;
}
} // namespace diana7_hardware

PLUGINLIB_EXPORT_CLASS(diana7_hardware::Diana7System, hardware_interface::SystemInterface)

