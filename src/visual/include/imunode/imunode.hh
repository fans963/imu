#pragma once
#include <chrono>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <thread>

#include "config/config.hh"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "lpms_nav3/LpLog.hh"
#include "lpms_nav3/LpmsIG1I.hh"
#include "lpms_nav3/LpmsIG1Registers.hh"
#include "lpms_nav3/SensorDataI.hh"

namespace imu {

struct IG1Command {
    short command;
    union Data {
        uint32_t i[64];
        float f[64];
        unsigned char c[256];
    } data;
    int dataLength;
};

class LpNAV3Proxy : public rclcpp::Node {
public:
    explicit LpNAV3Proxy(
        std::string port, int baudrate, bool autoReconnect, std::string frameId, int rate)
        : Node("imu_node")
        , comportNo_(port)
        , baudrate_(baudrate)
        , autoReconnect_(autoReconnect)
        , frameId_(frameId)
        , rate_(rate) {
        // this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        // this->declare_parameter<int>("baudrate_", 115200);
        // this->declare_parameter<bool>("autoreconnect_", true);
        // this->declare_parameter<std::string>("frameId_", "imu");
        // this->declare_parameter<int>("rate", 200);
        // this->get_parameter("port", comportNo_);
        // this->get_parameter("baudrate_", baudrate_);
        // this->get_parameter("autoreconnect_", autoReconnect_);
        // this->get_parameter("frameId_", frameId_);
        // this->get_parameter("rate", rate);

        // 创建imudata文件夹
        std::string folder_name = "imudata";
        if (mkdir(folder_name.c_str(), 0777) == -1) {
            if (errno != EEXIST) {
                RCLCPP_ERROR(
                    this->get_logger(), "Failed to create folder: %s", folder_name.c_str());
            }
        }
        // 创建数据记录文件并保存到imudata文件夹
        std::string file_name = "imu_data_" + std::to_string(std::time(nullptr)) + ".txt";
        file.open(folder_name + "/" + file_name, std::ios::out);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_name.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Data will be logged to: %s", file_name.c_str());
        }

        // Create LpmsBE1 object
        sensor1 = IG1Factory();
        sensor1->setVerbose(VERBOSE_INFO);
        sensor1->setAutoReconnectStatus(autoReconnect_);

        RCLCPP_INFO(this->get_logger(), "Settings");
        RCLCPP_INFO(this->get_logger(), "Port: %s", comportNo_.c_str());
        RCLCPP_INFO(this->get_logger(), "baudrate_: %d", baudrate_);
        RCLCPP_INFO(
            this->get_logger(), "Auto reconnect: %s", autoReconnect_ ? "Enabled" : "Disabled");

        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("data", 1);
        autocalibration_status_pub =
            this->create_publisher<std_msgs::msg::Bool>("is_autocalibration_active", 1);
        tf_broadcaster_      = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        autocalibration_serv = this->create_service<std_srvs::srv::SetBool>(
            "enable_gyro_autocalibration",
            [this](
                const std::shared_ptr<std_srvs::srv::SetBool::Request>& req,
                const std::shared_ptr<std_srvs::srv::SetBool::Response>& res) {
                return setAutocalibration(req, res);
            });
        autoReconnect_serv = this->create_service<std_srvs::srv::SetBool>(
            "enable_auto_reconnect",
            [this](
                const std::shared_ptr<std_srvs::srv::SetBool::Request>& req,
                const std::shared_ptr<std_srvs::srv::SetBool::Response>& res) {
                return setAutoReconnect(req, res);
            });
        gyrocalibration_serv = this->create_service<std_srvs::srv::Trigger>(
            "calibrate_gyroscope",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request>& req,
                const std::shared_ptr<std_srvs::srv::Trigger::Response>& res) {
                return calibrateGyroscope(req, res);
            });
        resetHeading_serv = this->create_service<std_srvs::srv::Trigger>(
            "reset_heading", [this](
                                 const std::shared_ptr<std_srvs::srv::Trigger::Request>& req,
                                 const std::shared_ptr<std_srvs::srv::Trigger::Response>& res) {
                return resetHeading(req, res);
            });
        getImuData_serv = this->create_service<std_srvs::srv::Trigger>(
            "get_imu_data", [this](
                                const std::shared_ptr<std_srvs::srv::Trigger::Request>& req,
                                const std::shared_ptr<std_srvs::srv::Trigger::Response>& res) {
                return getImuData(req, res);
            });
        setStreamingMode_serv = this->create_service<std_srvs::srv::Trigger>(
            "set_streaming_mode",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request>& req,
                const std::shared_ptr<std_srvs::srv::Trigger::Response>& res) {
                return setStreamingMode(req, res);
            });
        setCommandMode_serv = this->create_service<std_srvs::srv::Trigger>(
            "set_command_mode", [this](
                                    const std::shared_ptr<std_srvs::srv::Trigger::Request>& req,
                                    const std::shared_ptr<std_srvs::srv::Trigger::Response>& res) {
                return setCommandMode(req, res);
            });

        // Connects to sensor
        if (!sensor1->connect(comportNo_, baudrate_)) {
            RCLCPP_ERROR(this->get_logger(), "Error connecting to sensor\n");
            sensor1->release();
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        }

        do {
            RCLCPP_INFO(
                this->get_logger(), "Waiting for sensor to connect %d", sensor1->getStatus());
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } while (rclcpp::ok()
                 && (!(sensor1->getStatus() == STATUS_CONNECTED)
                     && !(sensor1->getStatus() == STATUS_CONNECTION_ERROR)));

        if (sensor1->getStatus() == STATUS_CONNECTED) {
            RCLCPP_INFO(this->get_logger(), "Sensor connected");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            sensor1->commandGotoStreamingMode();
        } else {
            RCLCPP_INFO(this->get_logger(), "Sensor connection error: %d.", sensor1->getStatus());
            rclcpp::shutdown();
        }
    }

    ~LpNAV3Proxy(void) {
        file.close();
        sensor1->release();
    }

    void update() {
        static bool runOnce = false;

        if (sensor1->getStatus() == STATUS_CONNECTED && sensor1->hasImuData()) {
            if (!runOnce) {
                publishIsAutocalibrationActive();
                runOnce = true;
            }
            IG1ImuDataI sd;
            sensor1->getImuData(sd);

            /* Fill the IMU message */

            // Fill the header
            // imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.stamp    = this->now();
            imu_msg.header.frame_id = frameId_;

            // Fill orientation quaternion
            imu_msg.orientation.w = sd.quaternion.data[0];
            imu_msg.orientation.x = sd.quaternion.data[1];
            imu_msg.orientation.y = sd.quaternion.data[2];
            imu_msg.orientation.z = sd.quaternion.data[3];

            if (!isFirstImuData) {
                using namespace std::chrono_literals;
                if (std::chrono::system_clock::now() - lastTime > 10s) {
                    file << imu_msg.orientation.w << " " << imu_msg.orientation.x << " "
                         << imu_msg.orientation.y << " " << imu_msg.orientation.z << std::endl;
                    lastTime = std::chrono::system_clock::now();
                }
            } else {
                lastTime       = std::chrono::system_clock::now();
                isFirstImuData = false;
                file << imu_msg.orientation.w << " " << imu_msg.orientation.x << " "
                     << imu_msg.orientation.y << " " << imu_msg.orientation.z << std::endl;
            }

            // Fill angular velocity data
            // - scale from deg/s to rad/s
            imu_msg.angular_velocity.x = sd.gyroIAlignmentCalibrated.data[0] * 3.1415926 / 180;
            imu_msg.angular_velocity.y = sd.gyroIAlignmentCalibrated.data[1] * 3.1415926 / 180;
            imu_msg.angular_velocity.z = sd.gyroIAlignmentCalibrated.data[2] * 3.1415926 / 180;

            // Fill linear acceleration data
            imu_msg.linear_acceleration.x = sd.accCalibrated.data[0] * 9.81;
            imu_msg.linear_acceleration.y = sd.accCalibrated.data[1] * 9.81;
            imu_msg.linear_acceleration.z = sd.accCalibrated.data[2] * 9.81;

            // Publish the messages
            imu_pub->publish(imu_msg);

            // 创建并发布TF变换
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp    = this->now();
            transform.header.frame_id = "base_link"; // 父坐标系
            transform.child_frame_id  = frameId_;    // IMU坐标系

            // 设置平移
            transform.transform.translation.x = 0.0; // 根据实际安装位置设置
            transform.transform.translation.y = 0.0;
            transform.transform.translation.z = 0.0;

            // 设置旋转（使用IMU的四元数）
            transform.transform.rotation = imu_msg.orientation;

            // 发布TF变换
            tf_broadcaster_->sendTransform(transform);
        }
    }

    void run() {
        updateTimer = this->create_wall_timer(
            std::chrono::milliseconds(1000 / abs(rate_)), [this]() { update(); });
    }

    void publishIsAutocalibrationActive() {
        std_msgs::msg::Bool msg;
        IG1SettingsI settings;
        sensor1->getSettings(settings);
        msg.data = settings.enableGyroAutocalibration;
        autocalibration_status_pub->publish(msg);
    }

    ///////////////////////////////////////////////////
    // Service Callbacks
    ///////////////////////////////////////////////////
    bool setAutocalibration(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>& req,
        const std::shared_ptr<std_srvs::srv::SetBool::Response>& res) {
        RCLCPP_INFO(this->get_logger(), "set_autocalibration");

        // clear current settings
        IG1SettingsI settings;
        sensor1->getSettings(settings);

        // Send command
        cmdSetEnableAutocalibration(req->data);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        cmdGetEnableAutocalibration();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        double retryElapsedTime = 0;
        int retryCount          = 0;
        while (!sensor1->hasSettings()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            RCLCPP_INFO(this->get_logger(), "set_autocalibration wait");

            retryElapsedTime += 0.1;
            if (retryElapsedTime > 2.0) {
                retryElapsedTime = 0;
                cmdGetEnableAutocalibration();
                retryCount++;
            }

            if (retryCount > 5)
                break;
        }
        RCLCPP_INFO(this->get_logger(), "set_autocalibration done");

        // Get settings
        sensor1->getSettings(settings);

        std::string msg;
        if (settings.enableGyroAutocalibration == req->data) {
            res->success = true;
            msg.append(
                std::string("[Success] autocalibration status set to: ")
                + (settings.enableGyroAutocalibration ? "True" : "False"));
        } else {
            res->success = false;
            msg.append(
                std::string("[Failed] current autocalibration status set to: ")
                + (settings.enableGyroAutocalibration ? "True" : "False"));
        }

        RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        res->message = msg;

        publishIsAutocalibrationActive();
        return res->success;
    }

    // Auto reconnect
    bool setAutoReconnect(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>& req,
        const std::shared_ptr<std_srvs::srv::SetBool::Response>& res) {
        RCLCPP_INFO(this->get_logger(), "set_auto_reconnect");
        sensor1->setAutoReconnectStatus(req->data);

        res->success = true;
        std::string msg;
        msg.append(
            std::string("[Success] auto reconnection status set to: ")
            + (sensor1->getAutoReconnectStatus() ? "True" : "False"));

        RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        res->message = msg;

        return res->success;
    }

    // reset heading
    bool resetHeading(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>& req,
        const std::shared_ptr<std_srvs::srv::Trigger::Response>& res) {
        RCLCPP_INFO(this->get_logger(), "reset_heading");

        // Send command
        cmdResetHeading();

        res->success = true;
        res->message = "[Success] Heading reset";
        return true;
    }

    bool calibrateGyroscope(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>& req,
        const std::shared_ptr<std_srvs::srv::Trigger::Response>& res) {
        RCLCPP_INFO(
            this->get_logger(),
            "calibrate_gyroscope: Please make sure the sensor is stationary for 4 seconds");

        cmdCalibrateGyroscope();

        std::this_thread::sleep_for(std::chrono::milliseconds(4000));
        res->success = true;
        res->message = "[Success] Gyroscope calibration procedure completed";
        RCLCPP_INFO(
            this->get_logger(), "calibrate_gyroscope: Gyroscope calibration procedure completed");
        return true;
    }

    bool getImuData(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>& req,
        const std::shared_ptr<std_srvs::srv::Trigger::Response>& res) {
        cmdGotoCommandMode();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        cmdGetImuData();
        res->success = true;
        res->message = "[Success] Get imu data";
        return true;
    }

    bool setStreamingMode(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>& req,
        const std::shared_ptr<std_srvs::srv::Trigger::Response>& res) {
        cmdGotoStreamingMode();
        res->success = true;
        res->message = "[Success] Set streaming mode";
        return true;
    }

    bool setCommandMode(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>& req,
        const std::shared_ptr<std_srvs::srv::Trigger::Response>& res) {
        cmdGotoCommandMode();
        res->success = true;
        res->message = "[Success] Set command mode";
        return true;
    }

    ///////////////////////////////////////////////////
    // Helpers
    ///////////////////////////////////////////////////

    void cmdGotoCommandMode() {
        IG1Command cmd;
        cmd.command    = GOTO_COMMAND_MODE;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

    void cmdGotoStreamingMode() {
        IG1Command cmd;
        cmd.command    = GOTO_STREAM_MODE;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

    void cmdGetImuData() {
        IG1Command cmd;
        cmd.command    = GET_IMU_DATA;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

    void cmdCalibrateGyroscope() {
        IG1Command cmd;
        cmd.command    = START_GYR_CALIBRATION;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

    void cmdResetHeading() {
        IG1Command cmd;
        cmd.command    = SET_ORIENTATION_OFFSET;
        cmd.dataLength = 4;
        cmd.data.i[0]  = LPMS_OFFSET_MODE_HEADING;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

    void cmdSetEnableAutocalibration(int status) {
        IG1Command cmd;
        cmd.command    = SET_ENABLE_GYR_AUTOCALIBRATION;
        cmd.dataLength = 4;
        cmd.data.i[0]  = status;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

    void cmdGetEnableAutocalibration() {
        IG1Command cmd;
        cmd.command    = GET_ENABLE_GYR_AUTOCALIBRATION;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

private:
    // Access to LPMS data
    IG1I* sensor1;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr autocalibration_status_pub;

    // Service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr autocalibration_serv;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr autoReconnect_serv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gyrocalibration_serv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetHeading_serv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getImuData_serv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr setStreamingMode_serv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr setCommandMode_serv;

    sensor_msgs::msg::Imu imu_msg;

    bool isFirstImuData = true;
    std::chrono::time_point<std::chrono::system_clock> lastTime;

    // Parameters
    std::string comportNo_;
    int baudrate_;
    bool autoReconnect_;
    std::string frameId_;
    int rate_;

    // File stream for data logging
    std::ofstream file;
};
} // namespace imu
