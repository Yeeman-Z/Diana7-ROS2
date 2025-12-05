#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "DianaAPI.h"

namespace {

using steady_clock = std::chrono::steady_clock;

std::atomic_bool g_running{true};

void signalHandler(int) { g_running.store(false); }

struct Config {
    std::string ip = "192.168.10.75";
    double distance_m = 0.10;          // meters along tool +Z
    double linear_speed_mps = 0.02;    // meters / second
    double linear_acc_mps2 = 0.10;     // meters / second^2
    double angular_acc_rps2 = 0.50;    // rad / second^2
    double sample_period_s = 0.02;     // seconds
    double settle_duration_s = 0.50;   // seconds after command
    std::filesystem::path log_path = "line_motion_samples.csv";
    bool has_active_tcp = false;
    std::array<double, 6> active_tcp{};
};

enum class ParseStatus { kOk, kShowUsage, kError };

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  -i, --ip <addr>           Robot controller IP (default 192.168.10.78)\n"
              << "  -d, --distance <m>        Distance along tool Z in meters (default 0.10)\n"
              << "  -s, --speed <mps>         Linear speed magnitude (default 0.02)\n"
              << "      --lin-acc <mps2>      Linear acceleration (default 0.10)\n"
              << "      --ang-acc <radps2>    Angular acceleration (default 0.50)\n"
              << "      --sample <s>          Sample period in seconds (default 0.02)\n"
              << "      --settle <s>          Extra logging time after motion (default 0.50)\n"
              << "      --log <path>          Output log file (default line_motion_samples.csv)\n"
              << "      --tcp <x y z rx ry rz> Optional active TCP pose (meters / radians)\n"
              << "  -h, --help                Show this message\n";
}

bool parseDouble(const char* text, double& value) {
    try {
        value = std::stod(text);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

ParseStatus parseArgs(int argc, char** argv, Config& cfg, std::string& error) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto needValue = [&](const std::string& flag) -> const char* {
            if (i + 1 >= argc) {
                error = "Missing value for " + flag;
                throw std::runtime_error(error);
            }
            return argv[++i];
        };

        try {
            if (arg == "-i" || arg == "--ip") {
                cfg.ip = needValue(arg);
            } else if (arg == "-d" || arg == "--distance") {
                double value = 0.0;
                if (!parseDouble(needValue(arg), value)) {
                    error = "Invalid distance value";
                    return ParseStatus::kError;
                }
                cfg.distance_m = value;
            } else if (arg == "-s" || arg == "--speed") {
                double value = 0.0;
                if (!parseDouble(needValue(arg), value)) {
                    error = "Invalid speed value";
                    return ParseStatus::kError;
                }
                cfg.linear_speed_mps = std::abs(value);
            } else if (arg == "--lin-acc") {
                double value = 0.0;
                if (!parseDouble(needValue(arg), value)) {
                    error = "Invalid linear acceleration value";
                    return ParseStatus::kError;
                }
                cfg.linear_acc_mps2 = std::abs(value);
            } else if (arg == "--ang-acc") {
                double value = 0.0;
                if (!parseDouble(needValue(arg), value)) {
                    error = "Invalid angular acceleration value";
                    return ParseStatus::kError;
                }
                cfg.angular_acc_rps2 = std::abs(value);
            } else if (arg == "--sample") {
                double value = 0.0;
                if (!parseDouble(needValue(arg), value)) {
                    error = "Invalid sample period value";
                    return ParseStatus::kError;
                }
                cfg.sample_period_s = std::abs(value);
            } else if (arg == "--settle") {
                double value = 0.0;
                if (!parseDouble(needValue(arg), value)) {
                    error = "Invalid settle duration value";
                    return ParseStatus::kError;
                }
                cfg.settle_duration_s = std::abs(value);
            } else if (arg == "--log") {
                cfg.log_path = needValue(arg);
            } else if (arg == "--tcp") {
                if (i + 6 >= argc) {
                    error = "--tcp expects 6 floating-point values";
                    return ParseStatus::kError;
                }
                cfg.has_active_tcp = true;
                for (size_t idx = 0; idx < cfg.active_tcp.size(); ++idx) {
                    double value = 0.0;
                    if (!parseDouble(argv[++i], value)) {
                        error = "Invalid TCP value at index " + std::to_string(idx);
                        return ParseStatus::kError;
                    }
                    cfg.active_tcp[idx] = value;
                }
            } else if (arg == "-h" || arg == "--help") {
                return ParseStatus::kShowUsage;
            } else {
                error = "Unknown argument: " + arg;
                return ParseStatus::kError;
            }
        } catch (const std::runtime_error&) {
            return ParseStatus::kError;
        }
    }

    if (std::abs(cfg.distance_m) < 1e-6) {
        error = "Distance must be non-zero";
        return ParseStatus::kError;
    }
    if (cfg.linear_speed_mps <= 0.0) {
        error = "Speed must be greater than zero";
        return ParseStatus::kError;
    }
    if (cfg.linear_acc_mps2 <= 0.0) {
        error = "Linear acceleration must be greater than zero";
        return ParseStatus::kError;
    }
    if (cfg.angular_acc_rps2 <= 0.0) {
        error = "Angular acceleration must be greater than zero";
        return ParseStatus::kError;
    }
    if (cfg.sample_period_s <= 0.0) {
        error = "Sample period must be greater than zero";
        return ParseStatus::kError;
    }
    if (cfg.settle_duration_s < 0.0) {
        error = "Settle duration cannot be negative";
        return ParseStatus::kError;
    }
    if (cfg.ip.empty()) {
        error = "Robot IP cannot be empty";
        return ParseStatus::kError;
    }

    return ParseStatus::kOk;
}

std::string describeLastVendorError(const std::string& ip) {
    int err = getLastError(ip.c_str());
    const char* msg = formatError(err, ip.c_str());
    std::ostringstream oss;
    oss << "code=" << err;
    if (msg != nullptr) {
        oss << " (" << msg << ")";
    }
    return oss.str();
}

struct SessionGuard {
    explicit SessionGuard(std::string ip) : ip_(std::move(ip)) {}
    ~SessionGuard() {
        if (!connected_) {
            return;
        }
        try {
            stop(ip_.c_str());
        } catch (...) {
        }
        try {
            holdBrake(ip_.c_str());
        } catch (...) {
        }
        try {
            destroySrv(ip_.c_str());
        } catch (...) {
        }
    }

    void setConnected(bool connected) { connected_ = connected; }

   private:
    std::string ip_;
    bool connected_{false};
};

void writePoseSample(std::ofstream& out, double elapsed_s, const double pose[6]) {
    out << std::fixed << std::setprecision(6) << elapsed_s;
    for (int i = 0; i < 6; ++i) {
        out << ',' << pose[i];
    }
    out << '\n';
}

std::array<double, 3> computeToolZAxis(const double rpy[3]) {
    const double roll = rpy[0];
    const double pitch = rpy[1];
    const double yaw = rpy[2];

    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    // ROS convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    const double r13 = cy * sp * cr + sy * sr;
    const double r23 = sy * sp * cr - cy * sr;
    const double r33 = cp * cr;

    const double norm = std::sqrt(r13 * r13 + r23 * r23 + r33 * r33);
    if (norm < 1e-9) {
        return {0.0, 0.0, 1.0};
    }

    return {r13 / norm, r23 / norm, r33 / norm};
}

}  // namespace

int main(int argc, char** argv) {
    Config cfg;
    std::string error;
    const ParseStatus status = parseArgs(argc, argv, cfg, error);
    if (status == ParseStatus::kShowUsage) {
        printUsage(argv[0]);
        return 0;
    }
    if (status == ParseStatus::kError) {
        std::cerr << "Argument error: " << error << "\n\n";
        printUsage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    srv_net_st net_info{};
    initSrvNetInfo(&net_info);
    std::snprintf(net_info.SrvIp, sizeof(net_info.SrvIp), "%s", cfg.ip.c_str());
    net_info.SrvIp[sizeof(net_info.SrvIp) - 1] = '\0';
    net_info.LocHeartbeatPort = 0;
    net_info.LocRobotStatePort = 0;
    net_info.LocSrvPort = 0;
    net_info.LocRealtimeSrvPort = 0;
    net_info.LocPassThroughSrvPort = 0;

    int ret = initSrv(nullptr, nullptr, &net_info);
    if (ret != 0) {
        std::cerr << "initSrv failed (" << ret << "): " << describeLastVendorError(cfg.ip) << '\n';
        return 1;
    }

    SessionGuard guard(cfg.ip);
    guard.setConnected(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ret = releaseBrake(cfg.ip.c_str());
    if (ret != 0) {
        std::cerr << "releaseBrake failed (" << ret << "): " << describeLastVendorError(cfg.ip)
                  << '\n';
    } else {
        std::cout << "Brake released.\n";
    }

    double pose[6] = {0.0};
    ret = getTcpPos(pose, cfg.ip.c_str());
    if (ret != 0) {
        std::cerr << "Failed to read initial TCP pose (" << ret
                  << "): " << describeLastVendorError(cfg.ip) << '\n';
        return 1;
    }

    std::cout << "Start pose: [" << pose[0] << ", " << pose[1] << ", " << pose[2] << ", "
              << pose[3] << ", " << pose[4] << ", " << pose[5] << "]\n";

    const auto log_path = std::filesystem::absolute(cfg.log_path);
    try {
        if (log_path.has_parent_path() && !log_path.parent_path().empty()) {
            std::filesystem::create_directories(log_path.parent_path());
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to create log directory: " << e.what() << '\n';
        return 1;
    }

    std::ofstream log_file(log_path);
    if (!log_file) {
        std::cerr << "Failed to open log file: " << log_path << '\n';
        return 1;
    }
    log_file << "# t[s],px,py,pz,rx,ry,rz\n";
    writePoseSample(log_file, 0.0, pose);

    const double travel_time_s = std::abs(cfg.distance_m) / cfg.linear_speed_mps;
    const double signed_speed = (cfg.distance_m >= 0.0 ? 1.0 : -1.0) * cfg.linear_speed_mps;
    const auto tool_z_axis = computeToolZAxis(&pose[3]);

    std::array<double, 6> speed_cmd{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (size_t i = 0; i < 3; ++i) {
        speed_cmd[i] = signed_speed * tool_z_axis[i];
    }
    std::array<double, 2> acc_cmd{cfg.linear_acc_mps2, cfg.angular_acc_rps2};

    std::cout << "Commanding " << cfg.distance_m * 100.0
              << " cm along tool Z at " << cfg.linear_speed_mps * 100.0 << " cm/s (duration ~"
              << travel_time_s << " s).\n";
    std::cout << "Tool Z axis in base frame: [" << tool_z_axis[0] << ", " << tool_z_axis[1] << ", "
              << tool_z_axis[2] << "]\n";

    ret = speedL(speed_cmd.data(), acc_cmd.data(), travel_time_s,
                 cfg.has_active_tcp ? cfg.active_tcp.data() : nullptr, cfg.ip.c_str());
    if (ret != 0) {
        std::cerr << "speedL failed (" << ret << "): " << describeLastVendorError(cfg.ip) << '\n';
        return 1;
    }

    const auto motion_start = steady_clock::now();
    const auto stop_time = motion_start + std::chrono::duration<double>(travel_time_s + cfg.settle_duration_s);
    auto sample_period = std::chrono::duration<double>(cfg.sample_period_s);
    if (sample_period <= std::chrono::duration<double>::zero()) {
        sample_period = std::chrono::duration<double>(0.02);
    }
    const auto sample_period_ticks =
        std::chrono::duration_cast<steady_clock::duration>(sample_period);
    auto next_sample = motion_start;

    while (g_running.load()) {
        const auto now = steady_clock::now();
        if (now >= stop_time) {
            break;
        }

        double current_pose[6] = {0.0};
        ret = getTcpPos(current_pose, cfg.ip.c_str());
        if (ret == 0) {
            const double elapsed =
                std::chrono::duration<double>(now - motion_start).count();
            writePoseSample(log_file, elapsed, current_pose);
        } else {
            std::cerr << "getTcpPos failed (" << ret << "): "
                      << describeLastVendorError(cfg.ip) << '\n';
        }

        next_sample += sample_period_ticks;
        std::this_thread::sleep_until(next_sample);
    }

    stop(cfg.ip.c_str());
    double final_pose[6] = {0.0};
    if (getTcpPos(final_pose, cfg.ip.c_str()) == 0) {
        const double elapsed =
            std::chrono::duration<double>(steady_clock::now() - motion_start).count();
        writePoseSample(log_file, elapsed, final_pose);
        std::cout << "Final pose: [" << final_pose[0] << ", " << final_pose[1] << ", "
                  << final_pose[2] << ", " << final_pose[3] << ", " << final_pose[4] << ", "
                  << final_pose[5] << "]\n";
    }

    log_file.flush();
    std::cout << "Pose samples saved to: " << log_path << '\n';
    return 0;
}


