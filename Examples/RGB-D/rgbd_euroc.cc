/**
* This file is part of ORB-SLAM3
*
* RGB-D example for EuRoC-style datasets (recorded data)
* Based on rgbd_tum.cc and stereo_euroc.cc
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, const string &strAssociationFilename,
                vector<string> &vstrImageRGB, vector<string> &vstrImageDepth, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        cerr << endl << "Usage: ./rgbd_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_association_file_1 (path_to_sequence_folder_2 path_to_association_file_2 ... path_to_sequence_folder_N path_to_association_file_N) (trajectory_file_name)" << endl;
        cerr << "Example: ./rgbd_euroc Vocabulary/ORBvoc.txt Examples/RGB-D/RealSense_D435i.yaml /path/to/dataset /path/to/dataset/associations.txt" << endl;
        cerr << endl;
        cerr << "Expected dataset structure:" << endl;
        cerr << "  path_to_sequence_folder/" << endl;
        cerr << "    rgb/              (RGB images)" << endl;
        cerr << "    depth/            (Depth images)" << endl;
        cerr << "    associations.txt  (TUM format: timestamp rgb_path timestamp depth_path)" << endl;
        return 1;
    }

    const int num_seq = (argc-3)/2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageRGB;
    vector< vector<string> > vstrImageDepth;
    vector< vector<double> > vTimestampsCam;
    vector<int> nImages;

    vstrImageRGB.resize(num_seq);
    vstrImageDepth.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2*seq) + 3]);
        string pathAssociation(argv[(2*seq) + 4]);

        LoadImages(pathSeq, pathAssociation, vstrImageRGB[seq], vstrImageDepth[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageRGB[seq].size();
        tot_images += nImages[seq];

        if(vstrImageRGB[seq].empty())
        {
            cerr << "ERROR: No images found in sequence " << seq << endl;
            return 1;
        }
        else if(vstrImageDepth[seq].size() != vstrImageRGB[seq].size())
        {
            cerr << "ERROR: Different number of images for RGB and depth in sequence " << seq << endl;
            return 1;
        }
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD, true);
    float imageScale = SLAM.GetImageScale();

    cv::Mat imRGB, imDepth;
    int proccIm = 0;

    for (seq = 0; seq<num_seq; seq++)
    {
        // Seq loop
        double t_resize = 0;
        double t_track = 0;

        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            // Read RGB and depth images from file
            imRGB = cv::imread(vstrImageRGB[seq][ni],cv::IMREAD_UNCHANGED);
            imDepth = cv::imread(vstrImageDepth[seq][ni],cv::IMREAD_UNCHANGED);

            if(imRGB.empty())
            {
                cerr << endl << "Failed to load RGB image at: "
                     << string(vstrImageRGB[seq][ni]) << endl;
                return 1;
            }

            if(imDepth.empty())
            {
                cerr << endl << "Failed to load depth image at: "
                     << string(vstrImageDepth[seq][ni]) << endl;
                return 1;
            }

            double tframe = vTimestampsCam[seq][ni];

            if(imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#else
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#endif
#endif
                int width = imRGB.cols * imageScale;
                int height = imRGB.rows * imageScale;
                cv::resize(imRGB, imRGB, cv::Size(width, height));
                cv::resize(imDepth, imDepth, cv::Size(width, height));

#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#else
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#endif
                t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif
            }

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif

            // Pass the images to the SLAM system
            SLAM.TrackRGBD(imRGB,imDepth,tframe);

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[proccIm]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }

        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;
            SLAM.ChangeDataset();
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<tot_images; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[tot_images/2] << endl;
    cout << "mean tracking time: " << totaltime/tot_images << endl;

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strPathToSequence, const string &strAssociationFilename,
                vector<string> &vstrImageRGB, vector<string> &vstrImageDepth, vector<double> &vTimeStamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    vTimeStamps.reserve(5000);
    vstrImageRGB.reserve(5000);
    vstrImageDepth.reserve(5000);

    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            // Skip comments
            if(s[0] == '#')
                continue;

            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimeStamps.push_back(t);
            ss >> sRGB;
            ss >> t;
            ss >> sD;
            vstrImageRGB.push_back(strPathToSequence + "/" + sRGB);
            vstrImageDepth.push_back(strPathToSequence + "/" + sD);
        }
    }
}
