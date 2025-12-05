#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// Vendor API
#include "DianaAPI.h"

class BrakeServiceNode : public rclcpp::Node {
   public:
    BrakeServiceNode() : Node("diana7_brake_service") {
        this->declare_parameter<std::string>("ip", "192.168.10.78");

        release_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "release_brake", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                const auto ip = this->get_parameter("ip").as_string();
                try {
                    releaseBrake(ip.c_str());
                    res->success = true;
                    res->message = "releaseBrake ok";
                } catch (...) {
                    res->success = false;
                    res->message = "releaseBrake exception";
                }
            });

        hold_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "hold_brake", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                const auto ip = this->get_parameter("ip").as_string();
                try {
                    holdBrake(ip.c_str());
                    res->success = true;
                    res->message = "holdBrake ok";
                } catch (...) {
                    res->success = false;
                    res->message = "holdBrake exception";
                }
            });

        poweroff_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "poweroff", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                const auto ip = this->get_parameter("ip").as_string();
                try {
                    // Optional: stop any motion then hold brake
                    stop(ip.c_str());
                    holdBrake(ip.c_str());
                    res->success = true;
                    res->message = "stop + holdBrake ok";
                } catch (...) {
                    res->success = false;
                    res->message = "poweroff exception";
                }
            });
    }

   private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr poweroff_srv_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrakeServiceNode>());
    rclcpp::shutdown();
    return 0;
}
