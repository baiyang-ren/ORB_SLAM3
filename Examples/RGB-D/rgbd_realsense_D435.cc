/**
* This file is part of ORB-SLAM3
*
* RGB-D example for Intel RealSense D435 (no IMU)
* Modified from rgbd_realsense_D435i.cc to remove IMU dependencies
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gomez Rodriguez,
* Jose M.M. Montiel and Juan D. Tardos, University of Zaragoza.
* Copyright (C) 2014-2016 Raul Mur-Artal, Jose M.M. Montiel and Juan D. Tardos,
* University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
* WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
* A PARTICULAR PURPOSE. See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with
* ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
*/

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

#include <System.h>

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s) {
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current,
                     const std::vector<rs2::stream_profile>& prev);

static rs2_option get_sensor_option(const rs2::sensor& sensor) {
    std::cout << "Sensor supports the following options:\n" << std::endl;

    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++) {
        rs2_option option_type = static_cast<rs2_option>(i);
        std::cout << "  " << i << ": " << option_type;

        if (sensor.supports(option_type)) {
            std::cout << std::endl;
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;
        } else {
            std::cout << " is not supported" << std::endl;
        }
    }

    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}

int main(int argc, char **argv) {

    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./rgbd_realsense_D435 path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;
    bool bFileName = false;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
        bFileName = true;
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0) {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return 0;
    } else {
        selected_device = devices[0];
    }

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    int index = 0;
    // Iterate the sensors and configure them (no IMU sensor on D435)
    for (rs2::sensor sensor : sensors) {
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
            ++index;
            std::cout << "  " << index << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;

            if (index == 1) {
                // Depth sensor
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                // Enable emitter for depth
                sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1);
            }
            if (index == 2) {
                // RGB camera - enable auto exposure
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
            }
            get_sensor_option(sensor);
        }
    }

    // Declare RealSense pipeline
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // RGB stream
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);

    // Depth stream
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // No IMU streams - D435 doesn't have IMU

    // Mutex for thread-safe access
    std::mutex data_mutex;
    std::condition_variable cond_image_rec;

    cv::Mat imCV, depthCV;
    int width_img, height_img;
    double timestamp_image = -1.0;
    bool image_ready = false;
    int count_im_buffer = 0;

    // Start and stop just to get necessary profile
    rs2::pipeline_profile pipe_profile = pipe.start(cfg);
    pipe.stop();

    // Align depth and RGB frames
    rs2_stream align_to = find_stream_to_align(pipe_profile.get_streams());
    rs2::align align(align_to);
    rs2::frameset fsSLAM;

    // Frame callback
    auto frame_callback = [&](const rs2::frame& frame) {
        std::unique_lock<std::mutex> lock(data_mutex);

        if (rs2::frameset fs = frame.as<rs2::frameset>()) {
            count_im_buffer++;

            double new_timestamp_image = fs.get_timestamp() * 1e-3;
            if (abs(timestamp_image - new_timestamp_image) < 0.001) {
                count_im_buffer--;
                return;
            }

            if (profile_changed(pipe.get_active_profile().get_streams(),
                                pipe_profile.get_streams())) {
                pipe_profile = pipe.get_active_profile();
                align_to = find_stream_to_align(pipe_profile.get_streams());
                align = rs2::align(align_to);
            }

            fsSLAM = fs;
            timestamp_image = fs.get_timestamp() * 1e-3;
            image_ready = true;

            lock.unlock();
            cond_image_rec.notify_all();
        }
    };

    pipe_profile = pipe.start(cfg, frame_callback);

    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);

    rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_cam.width;
    height_img = intrinsics_cam.height;
    std::cout << " fx = " << intrinsics_cam.fx << std::endl;
    std::cout << " fy = " << intrinsics_cam.fy << std::endl;
    std::cout << " cx = " << intrinsics_cam.ppx << std::endl;
    std::cout << " cy = " << intrinsics_cam.ppy << std::endl;
    std::cout << " height = " << intrinsics_cam.height << std::endl;
    std::cout << " width = " << intrinsics_cam.width << std::endl;
    std::cout << " Coeff = " << intrinsics_cam.coeffs[0] << ", " << intrinsics_cam.coeffs[1]
              << ", " << intrinsics_cam.coeffs[2] << ", " << intrinsics_cam.coeffs[3] << ", "
              << intrinsics_cam.coeffs[4] << std::endl;
    std::cout << " Model = " << intrinsics_cam.model << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    double timestamp;
    cv::Mat im, depth;

    double t_resize = 0.f;
    double t_track = 0.f;
    rs2::frameset fs;

    while (!SLAM.isShutDown()) {
        {
            std::unique_lock<std::mutex> lk(data_mutex);
            if (!image_ready)
                cond_image_rec.wait(lk);

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point time_Start_Process =
                std::chrono::steady_clock::now();
#else
            std::chrono::steady_clock::time_point time_Start_Process =
                std::chrono::steady_clock::now();
#endif

            fs = fsSLAM;

            if (count_im_buffer > 1)
                cout << count_im_buffer - 1 << " dropped frs\n";
            count_im_buffer = 0;

            timestamp = timestamp_image;
            im = imCV.clone();
            depth = depthCV.clone();

            image_ready = false;
        }

        // Perform alignment here
        auto processed = align.process(fs);

        // Trying to get both other and aligned depth frames
        rs2::video_frame color_frame = processed.first(align_to);
        rs2::depth_frame depth_frame = processed.get_depth_frame();

        im = cv::Mat(cv::Size(width_img, height_img), CV_8UC3,
                     (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);
        depth = cv::Mat(cv::Size(width_img, height_img), CV_16U,
                        (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);

        if (imageScale != 1.f) {
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize =
                std::chrono::steady_clock::now();
#else
            std::chrono::steady_clock::time_point t_Start_Resize =
                std::chrono::steady_clock::now();
#endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(depth, depth, cv::Size(width, height));

#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize =
                std::chrono::steady_clock::now();
#else
            std::chrono::steady_clock::time_point t_End_Resize =
                std::chrono::steady_clock::now();
#endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                           t_End_Resize - t_Start_Resize)
                           .count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
#else
        std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
#endif
#endif
        // Pass the image to the SLAM system
        SLAM.TrackRGBD(im, depth, timestamp);

#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();
#else
        std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();
#endif
        t_track = t_resize +
                  std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                      t_End_Track - t_Start_Track)
                      .count();
        SLAM.InsertTrackTime(t_track);
#endif
    }
    cout << "System shutdown!\n";
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams) {
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams) {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH) {
            if (!color_stream_found)
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR) {
                color_stream_found = true;
            }
        } else {
            depth_stream_found = true;
        }
    }

    if (!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current,
                     const std::vector<rs2::stream_profile>& prev) {
    for (auto&& sp : prev) {
        auto itr = std::find_if(std::begin(current), std::end(current),
                                [&sp](const rs2::stream_profile& current_sp) {
                                    return sp.unique_id() == current_sp.unique_id();
                                });
        if (itr == std::end(current))
            return true;
    }
    return false;
}
