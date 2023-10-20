#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "ImuTypes.h"

using namespace std;

void LoadImages(const string &strImageAssociation, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);
void LoadIMU(const string &strImuAssociation, vector<string> &vstrImuFilenames, vector<double> &vTimestampsImu);
void ReadImu(const string &strImuPath, cv::Point3f &vAcc, cv::Point3f &vGyro);

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        cerr << endl
             << "Usage: ./rgbd_inertial path_to_vocabulary path_to_settings path_to_image_association path_to_imu_association" << endl;
        return 1;
    }

    // Retrieve paths to images
    string strImageAssociation = string(argv[3]), strImuAssociation = string(argv[4]);
    vector<double> vTimestampsCam, vTimestampsImu;
    vector<string> vstrImageFilenamesRGB, vstrImageFilenamesD, vstrImuFilenames;
    LoadImages(strImageAssociation, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestampsCam);
    LoadIMU(strImuAssociation, vstrImuFilenames, vTimestampsImu);

    // Check consistency in the number of images, depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if (vstrImageFilenamesRGB.empty())
    {
        cerr << endl
             << "No rgb images found in provided path." << endl;
        return 1;
    }
    else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size())
    {
        cerr << endl
             << "Different number of frames for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true);
    float imageScale = SLAM.GetImageScale();

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    // Main loop
    cv::Mat imRGB, imD;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    int idx_imu = 0, idx_image = 0;
    while (vTimestampsImu[idx_imu] < vTimestampsCam[idx_image])
    {
        idx_imu++;
    }
    if (idx_imu > 0 && vTimestampsImu[idx_imu] >= vTimestampsCam[idx_image])
    {
        idx_imu--;
    }
    while (vTimestampsImu[idx_imu] >= vTimestampsCam[idx_image])
    {
        idx_image++;
    }

    for (; idx_image < nImages; idx_image++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(vstrImageFilenamesRGB[idx_image], cv::IMREAD_UNCHANGED);
        imD = cv::imread(vstrImageFilenamesD[idx_image], cv::IMREAD_UNCHANGED);

        if (imRGB.empty())
        {
            cerr << endl
                 << "Failed to load rgb image at: " << vstrImageFilenamesRGB[idx_image] << endl;
            return 1;
        }
        if (imD.empty())
        {
            cerr << endl
                 << "Failed to load depth image at: " << vstrImageFilenamesD[idx_image] << endl;
        }

        if (imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }

        // Read imu measurements from previous frame
        vImuMeas.clear();
        while (vTimestampsImu[idx_imu] <= vTimestampsCam[idx_image])
        {
            cv::Point3f vAcc, vGyro;
            ReadImu(vstrImuFilenames[idx_imu], vAcc, vGyro);
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc, vGyro, vTimestampsImu[idx_imu]));
            idx_imu++;
        }

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB, imD, vTimestampsCam[idx_image], vImuMeas);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &strImageAssociation, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strImageAssociation.c_str());
    while (!fAssociation.eof())
    {
        string s;
        getline(fAssociation, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }
}

void LoadIMU(const string &strImuAssociation, vector<string> &vstrImuFilenames, vector<double> &vTimestampsImu)
{
    ifstream fAssociation;
    fAssociation.open(strImuAssociation.c_str());

    while (!fAssociation.eof())
    {
        string s;
        getline(fAssociation, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sImu;
            ss >> t;
            vTimestampsImu.push_back(t);
            ss >> sImu;
            vstrImuFilenames.push_back(sImu);
        }
    }
}

void ReadImu(const string &strImuPath, cv::Point3f &vAcc, cv::Point3f &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());

    string s;
    getline(fImu, s);

    stringstream ss;
    ss << s;
    double fAccX, fAccY, fAccZ;
    double fGyroX, fGyroY, fGyroZ;

    ss >> fAccX >> fAccY >> fAccZ;
    vAcc = cv::Point3f(fAccX * 1000, fAccY * 1000, fAccZ * 1000);
    ss >> fGyroX >> fGyroY >> fGyroZ;
    vGyro = cv::Point3f(fGyroX, fGyroY, fGyroZ);
}
