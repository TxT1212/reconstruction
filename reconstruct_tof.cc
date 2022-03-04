#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <map>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
using std::cout;
using std::endl;
using std::string;
#include <opencv2/highgui/highgui.hpp>
#include <Open3D/Open3D.h>
#include <iostream>
#include <memory>
#include <thread>
// #include <PinholeCameraIntrinsic.h>
// #include <TSDFVolume.h>
#include "CUDAFilter.h"
#include "cuda.h"
#include <cuda_runtime_api.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
using namespace open3d;
using namespace std;
using namespace Eigen;
using namespace cv;
struct DepthCameraParams
{
    float fx; //4bytes
    float fy;
    float mx;
    float my;

    unsigned int m_imageWidth; //4bytes
    unsigned int m_imageHeight;

    float m_sensorDepthWorldMin; //render depth min
    float m_sensorDepthWorldMax; //render depth max
};
DepthCameraParams depthCameraParams;

std::vector<std::vector<std::string>> ReadSpaceSeparatedText(std::string filepath)
{
    //打开文件
    std::cout << "Begin reading space separated text file ..." << filepath << std::endl;
    std::ifstream file(filepath);
    if (!file)
    {
        std::cerr << "fail to open file" << filepath << std::endl;
        exit(1);
    }

    std::vector<std::vector<std::string>> file_content; //存储每个数据的数据结构
    std::string file_line;

    while (std::getline(file, file_line))
    {                                               //读取每一行数据
        std::vector<std::string> file_content_line; //存储分割之后每一行的四个数据
        std::stringstream ss;                       //利用内存流进行每行数据分割内存流
        std::string each_elem;
        ss << file_line;

        while (ss >> each_elem)
        {
            file_content_line.push_back(each_elem);
        }
        file_content.emplace_back(file_content_line); //数据组装
    };
    return file_content;
}

Eigen::Matrix4d ParsePose(std::vector<std::string> pose)
{
    float w = stof(pose[6]);
    float x = stof(pose[3]);
    float y = stof(pose[4]);
    float z = stof(pose[5]);
    float cx = stof(pose[0]);
    float cy = stof(pose[1]);
    float cz = stof(pose[2]);
    Eigen::Quaterniond q(w, x, y, z);
    Eigen::Matrix4d anwser;
    anwser.block<3, 3>(0, 0) = q.toRotationMatrix();
    anwser(0, 3) = cx;
    anwser(1, 3) = cy;
    anwser(2, 3) = cz;
    anwser(3, 3) = 1;
    anwser(3, 0) = anwser(3, 1) = anwser(3, 2) = 0;
    // cout << anwser << endl;
    return anwser;
}

void MakeDepthFrame(string color_path, string depth_path, float param_depth_scale, float *dev_depthIntegration, uchar *colorIntegration)
{
    int rows = depthCameraParams.m_imageHeight;
    int cols = depthCameraParams.m_imageWidth;
    // cout << rows << cols << endl;
    float *depthIntegration = new float[cols * rows]; //on the cpu
    //读入到CPU
    cv::Mat color_bgr = cv::imread(color_path);
    if (color_bgr.empty())
    {
        cout << "Reading color image failed , path = " << color_path << endl;
    }
    cv::Mat color_rgb = cv::Mat::zeros(1536, 2048, CV_8UC3);

    cv::cvtColor(color_bgr, color_rgb, cv::COLOR_BGR2RGB);
    // if (color_bgr.rows != rows)
    // {
    //     cout << "resize" << endl;
    //     resize(color_rgb, color_rgb, Size(cols, rows));
    // }
    cv::Mat depth_uchar = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
    if (depth_uchar.empty())
    {
        cout << "Reading depth image failed , path = " << depth_path << endl;
    }
    cv::Mat depth_uchar_;
    unsigned short *depth = (unsigned short *)depth_uchar.data;

    //将color变为uchar4类型，将depth变为float类型
    for (unsigned int i = 0; i < cols * rows; i++)
    {
        depthIntegration[i] = depth[i] / param_depth_scale;
        if (depthIntegration[i] > depthCameraParams.m_sensorDepthWorldMax || depthIntegration[i] < depthCameraParams.m_sensorDepthWorldMin) //truncate
        {
            depthIntegration[i] = 0;
        }
    }
    cudaMemcpy(dev_depthIntegration, depthIntegration,
               sizeof(float) * depthCameraParams.m_imageWidth * depthCameraParams.m_imageHeight, cudaMemcpyHostToDevice);
    memcpy(colorIntegration, (uchar *)color_rgb.data, depthCameraParams.m_imageWidth * depthCameraParams.m_imageHeight * 3);
}
void printResult(float *arr, int rows = 1536, int cols = 2048)
{
    for (int i = 0; i < rows * cols; i++)
    {
        cout << arr[i] << " ";
        if (i % cols == 0)
        {
            cout << endl;
        }
    }
}

int main(int argc, char **argv)
{
    cv::FileStorage *fSettings_;

    if (argc > 2)
    {
        cout << "usage: ./BundleFusionDenseRecon CONFIG_FILE\n or \n ./BundleFusionDenseRecon" << endl;
        exit(-1);
    }
    else if (argc == 2)
    {
        fSettings_ = new cv::FileStorage(argv[1], cv::FileStorage::READ);
    }
    else
    {
        fSettings_ = new cv::FileStorage("../reconstruct.conf", cv::FileStorage::READ);
    }
    cv::FileStorage &fSettings = *fSettings_;
    // cout << "cv::COLOR_BGR2RGB " << cv::COLOR_BGR2RGB << endl;
    cv::Mat K;
    fSettings["K_rgb"] >> K;
    Matrix3d K_rgb_eigen;
    cv2eigen(K, K_rgb_eigen);

    depthCameraParams.fx = K_rgb_eigen(0, 0);
    depthCameraParams.fy = K_rgb_eigen(1, 1);
    depthCameraParams.mx = K_rgb_eigen(0, 2);
    depthCameraParams.my = K_rgb_eigen(1, 2);
    depthCameraParams.m_sensorDepthWorldMin = 0.3;
    depthCameraParams.m_sensorDepthWorldMax = 2.5;
    int m_width = fSettings["width"];
    int m_heigt = fSettings["height"];
    depthCameraParams.m_imageWidth = m_width;
    depthCameraParams.m_imageHeight = m_heigt;
    open3d::camera::PinholeCameraIntrinsic intrinsic(depthCameraParams.m_imageWidth, depthCameraParams.m_imageHeight, depthCameraParams.fx, depthCameraParams.fy, depthCameraParams.mx, depthCameraParams.my);
    string ws_folder = fSettings["ws_folder"];
    // std::vector<std::vector<std::string>> associations = ReadSpaceSeparatedText(ws_folder + fSettings["associations"]);
    std::vector<std::vector<std::string>> associations = ReadSpaceSeparatedText(ws_folder + fSettings["associations_tof"]);
    std::vector<std::vector<std::string>> poses = ReadSpaceSeparatedText(ws_folder + fSettings["campose_path"]);
    float param_depth_scale = 1000.0f;
    const int iterStep = fSettings["iterStep"];
    const float param_voxel_size = fSettings["param_voxel_size"];

    float *dev_depthIntegration;        //on the gpu
    uchar *colorIntegration;            //on the cpu
    float *dev_depthIntegration_filter; //on the gpu

    cudaMalloc((void **)&dev_depthIntegration, sizeof(float) * depthCameraParams.m_imageWidth * depthCameraParams.m_imageHeight);
    cudaMalloc((void **)&dev_depthIntegration_filter, sizeof(float) * depthCameraParams.m_imageWidth * depthCameraParams.m_imageHeight);
    // cout << sizeof(float4) <<" " << depthCameraParams.m_imageWidth <<" " << depthCameraParams.m_imageHeight << endl;

    int rows = depthCameraParams.m_imageHeight;
    int cols = depthCameraParams.m_imageWidth;
    unsigned int numIter = 2;
    int iterIndex;
    numIter = 2 * ((numIter + 1) / 2);
    open3d::integration::ScalableTSDFVolume volume(
        param_voxel_size,
        param_voxel_size * 3,
        open3d::integration::TSDFVolumeColorType::RGB8
        // ,
        // int     volume_unit_resolution = 16,
        // int     depth_sampling_stride = 4
    );
    auto depth_image_ptr = std::make_shared<geometry::Image>();
    depth_image_ptr->Prepare(cols, rows, 1, 4);
    auto color_image_ptr = std::make_shared<geometry::Image>();
    color_image_ptr->Prepare(cols, rows, 3, 1);

    float *depthOutputFiltered = new float[cols * rows];
    // std::shared_ptr< open3d::geometry::RGBDImage > rgbd(
    open3d::geometry::RGBDImage rgbd(
        *color_image_ptr,
        *depth_image_ptr);
    colorIntegration = rgbd.color_.data_.data();
    cout << "poses " << poses.size() << endl;
    // for (int i = 0; i < 1; i+=2)
    int iter_end = fSettings["iter_end"];
    if (iter_end<=0)
    {
        iter_end = poses.size();
    }
    
    for (int i = 0; i < iter_end; i += iterStep)
    {

        vector<string> pose_vec = poses[i];
        vector<string>::const_iterator first = pose_vec.begin() + 1;
        vector<string>::const_iterator end = pose_vec.begin() + 8;
        vector<string> pose_vec_new(first, end);
        Eigen::Matrix4d cur_pose = ParsePose(pose_vec_new);
        iterIndex = stoi(pose_vec[0]);
        // string colorNum = associations_tof[iterIndex][0];
        // iterIndex = -1;
        // for (size_t tofi = 0; tofi < associations.size(); tofi++)
        // {
        //     if (associations[tofi][0].compare(colorNum) == 0)
        //     {
        //         iterIndex = tofi;
        //         break;
        //     }
        // }
        // if (iterIndex == -1)
        // {
        //     continue;
        // }
        
        string color_path = ws_folder + associations[iterIndex][1];
        string depth_path = ws_folder + associations[iterIndex][3];
        cout << "\ncolor_path = " << color_path << endl;
        cout << "depth_path = " << depth_path << endl;

        cout << i << "/" << poses.size() << endl;

        // MakeDepthFrame(color_path,depth_path,param_depth_scale,dev_depthIntegration,dev_colorIntegration);
        MakeDepthFrame(color_path, depth_path, param_depth_scale, dev_depthIntegration, rgbd.color_.data_.data());
        if (true)
        {
            for (unsigned int i = 0; i < numIter; i++)
            { //对输入的深度图erode两回，并存回d_depthInputRaw中
                if (i % 2 == 0)
                {
                    erodeDepthMap(dev_depthIntegration_filter, dev_depthIntegration, 3, cols, rows, 0.05f, 0.3f); //默认0.05f, 0.3f
                }
                else
                {
                    erodeDepthMap(dev_depthIntegration, dev_depthIntegration_filter, 3, cols, rows, 0.05f, 0.3f);
                }
            }
        }
        if (1)
        { //smooth
            gaussFilterDepthMap(dev_depthIntegration_filter, dev_depthIntegration, 2.0, 0.05, cols, rows);
        }
        cudaMemcpy((void **)rgbd.depth_.data_.data(), dev_depthIntegration_filter, sizeof(float) * cols * rows, cudaMemcpyDeviceToHost);
        // cudaMemcpy((void **)rgbd.depth_.data_.data(), dev_depthIntegration, sizeof(float) * cols * rows, cudaMemcpyDeviceToHost);

        // cudaMemcpy(depthOutputFiltered, dev_depthIntegration_filter, sizeof(float) * cols * rows,cudaMemcpyDeviceToHost);
        // printResult((float*) rgbd.depth_.data_.data());
        volume.Integrate(
            rgbd,
            intrinsic,
            cur_pose.inverse());
    }
    auto pcd = volume.ExtractTriangleMesh();
    open3d::io::WriteTriangleMesh(ws_folder + fSettings["pointCloud"], *pcd, true);
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
