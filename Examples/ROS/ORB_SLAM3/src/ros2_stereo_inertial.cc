/**
 * ORB-SLAM3 ROS 2 Stereo-Inertial Node
 * Ported from ROS 1 version to ROS 2 Jazzy
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "System.h"
#include "ImuTypes.h"

using namespace std;

class StereoInertialNode : public rclcpp::Node
{
public:
    StereoInertialNode(ORB_SLAM3::System* pSLAM, bool bRect, bool bClahe)
        : Node("stereo_inertial_slam"),
          mpSLAM(pSLAM),
          do_rectify(bRect),
          mbClahe(bClahe)
    {
        // Declare parameters for topic names with defaults for RealSense D435i
        this->declare_parameter<std::string>("imu_topic", "/camera/camera/imu");
        this->declare_parameter<std::string>("left_image_topic", "/camera/camera/infra1/image_rect_raw");
        this->declare_parameter<std::string>("right_image_topic", "/camera/camera/infra2/image_rect_raw");

        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string left_topic = this->get_parameter("left_image_topic").as_string();
        std::string right_topic = this->get_parameter("right_image_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "Subscribing to IMU: %s", imu_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to Left: %s", left_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to Right: %s", right_topic.c_str());

        // Create subscriptions with QoS for sensor data (BEST_EFFORT to match RealSense)
        rclcpp::QoS sensor_qos(10);
        sensor_qos.best_effort();

        rclcpp::QoS imu_qos(1000);
        imu_qos.best_effort();

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, imu_qos,
            std::bind(&StereoInertialNode::GrabImu, this, std::placeholders::_1));

        sub_img_left_ = this->create_subscription<sensor_msgs::msg::Image>(
            left_topic, sensor_qos,
            std::bind(&StereoInertialNode::GrabImageLeft, this, std::placeholders::_1));

        sub_img_right_ = this->create_subscription<sensor_msgs::msg::Image>(
            right_topic, sensor_qos,
            std::bind(&StereoInertialNode::GrabImageRight, this, std::placeholders::_1));

        // Create CLAHE object if needed
        if (mbClahe) {
            mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        }

        // Start sync thread
        sync_thread_ = std::thread(&StereoInertialNode::SyncWithImu, this);

        RCLCPP_INFO(this->get_logger(), "Stereo-Inertial SLAM node initialized");
    }

    ~StereoInertialNode()
    {
        bFinish_ = true;
        if (sync_thread_.joinable()) {
            sync_thread_.join();
        }
    }

    void SetRectifyMaps(const cv::Mat& M1l, const cv::Mat& M2l,
                        const cv::Mat& M1r, const cv::Mat& M2r)
    {
        M1l_ = M1l.clone();
        M2l_ = M2l.clone();
        M1r_ = M1r.clone();
        M2r_ = M2r.clone();
    }

private:
    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        std::lock_guard<std::mutex> lock(mBufMutexImu_);
        imuBuf_.push(imu_msg);
    }

    void GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        std::lock_guard<std::mutex> lock(mBufMutexLeft_);
        if (!imgLeftBuf_.empty())
            imgLeftBuf_.pop();
        imgLeftBuf_.push(img_msg);
    }

    void GrabImageRight(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        std::lock_guard<std::mutex> lock(mBufMutexRight_);
        if (!imgRightBuf_.empty())
            imgRightBuf_.pop();
        imgRightBuf_.push(img_msg);
    }

    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr& img_msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return cv::Mat();
        }
        return cv_ptr->image.clone();
    }

    double GetTimestamp(const sensor_msgs::msg::Image::SharedPtr& img_msg)
    {
        return rclcpp::Time(img_msg->header.stamp).seconds();
    }

    double GetTimestamp(const sensor_msgs::msg::Imu::SharedPtr& imu_msg)
    {
        return rclcpp::Time(imu_msg->header.stamp).seconds();
    }

    void SyncWithImu()
    {
        const double maxTimeDiff = 0.01;

        while (!bFinish_) {
            cv::Mat imLeft, imRight;
            double tImLeft = 0, tImRight = 0;

            if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty()) {
                tImLeft = GetTimestamp(imgLeftBuf_.front());
                tImRight = GetTimestamp(imgRightBuf_.front());

                // Sync right image to left
                {
                    std::lock_guard<std::mutex> lock(mBufMutexRight_);
                    while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1) {
                        imgRightBuf_.pop();
                        tImRight = GetTimestamp(imgRightBuf_.front());
                    }
                }

                // Sync left image to right
                {
                    std::lock_guard<std::mutex> lock(mBufMutexLeft_);
                    while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1) {
                        imgLeftBuf_.pop();
                        tImLeft = GetTimestamp(imgLeftBuf_.front());
                    }
                }

                if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                // Check if we have IMU data up to the image time
                {
                    std::lock_guard<std::mutex> lock(mBufMutexImu_);
                    if (imuBuf_.empty() || tImLeft > GetTimestamp(imuBuf_.back())) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                }

                // Get images
                {
                    std::lock_guard<std::mutex> lock(mBufMutexLeft_);
                    imLeft = GetImage(imgLeftBuf_.front());
                    imgLeftBuf_.pop();
                }

                {
                    std::lock_guard<std::mutex> lock(mBufMutexRight_);
                    imRight = GetImage(imgRightBuf_.front());
                    imgRightBuf_.pop();
                }

                if (imLeft.empty() || imRight.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Empty image received");
                    continue;
                }

                // Get IMU measurements
                vector<ORB_SLAM3::IMU::Point> vImuMeas;
                {
                    std::lock_guard<std::mutex> lock(mBufMutexImu_);
                    while (!imuBuf_.empty() && GetTimestamp(imuBuf_.front()) <= tImLeft) {
                        double t = GetTimestamp(imuBuf_.front());
                        cv::Point3f acc(
                            imuBuf_.front()->linear_acceleration.x,
                            imuBuf_.front()->linear_acceleration.y,
                            imuBuf_.front()->linear_acceleration.z);
                        cv::Point3f gyr(
                            imuBuf_.front()->angular_velocity.x,
                            imuBuf_.front()->angular_velocity.y,
                            imuBuf_.front()->angular_velocity.z);
                        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                        imuBuf_.pop();
                    }
                }

                // Apply CLAHE if enabled
                if (mbClahe) {
                    mClahe->apply(imLeft, imLeft);
                    mClahe->apply(imRight, imRight);
                }

                // Rectify if needed
                if (do_rectify) {
                    cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                    cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
                }

                // Track
                mpSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // SLAM system
    ORB_SLAM3::System* mpSLAM;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_left_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_right_;

    // Buffers
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf_;
    std::queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf_, imgRightBuf_;

    // Mutexes
    std::mutex mBufMutexImu_;
    std::mutex mBufMutexLeft_;
    std::mutex mBufMutexRight_;

    // Rectification maps
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    // Options
    const bool do_rectify;
    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe;

    // Sync thread
    std::thread sync_thread_;
    std::atomic<bool> bFinish_{false};
};


void PrintUsage()
{
    cerr << endl << "Usage: ros2 run orb_slam3_ros2 stereo_inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Remove ROS arguments to get only our arguments
    std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

    if (args.size() < 4 || args.size() > 5) {
        PrintUsage();
        cerr << "Got " << args.size() << " arguments: ";
        for (const auto& arg : args) cerr << arg << " ";
        cerr << endl;
        rclcpp::shutdown();
        return 1;
    }

    std::string vocPath = args[1];
    std::string settingsPath = args[2];
    bool bRect = (args[3] == "true");
    bool bEqual = false;
    if (args.size() == 5) {
        bEqual = (args[4] == "true");
    }

    // Create SLAM system
    ORB_SLAM3::System SLAM(vocPath, settingsPath, ORB_SLAM3::System::IMU_STEREO, true);

    // Create node
    auto node = std::make_shared<StereoInertialNode>(&SLAM, bRect, bEqual);

    // Load rectification parameters if needed
    if (bRect) {
        cv::FileStorage fsSettings(settingsPath, cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;
        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;
        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;
        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() ||
            R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::Mat M1l, M2l, M1r, M2r;
        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3),
                                    cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3),
                                    cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
        node->SetRectifyMaps(M1l, M2l, M1r, M2r);
    }

    RCLCPP_INFO(node->get_logger(), "Starting ORB-SLAM3 Stereo-Inertial...");
    RCLCPP_INFO(node->get_logger(), "Vocabulary: %s", vocPath.c_str());
    RCLCPP_INFO(node->get_logger(), "Settings: %s", settingsPath.c_str());
    RCLCPP_INFO(node->get_logger(), "Rectify: %s, Equalize: %s",
                bRect ? "true" : "false", bEqual ? "true" : "false");

    rclcpp::spin(node);

    // Shutdown
    SLAM.Shutdown();
    rclcpp::shutdown();

    return 0;
}
