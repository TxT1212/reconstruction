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

using namespace std;

std::vector<std::vector<std::string>> ReadSpaceSeparatedText(std::string filepath)
{
	//打开文件
	std::cout << "Begin reading space separated text file ..." << std::endl;
	std::ifstream file(filepath);
	if (!file)
	{
		std::cerr << "fail to open file" << filepath << std::endl;
		exit(1);
	}

	std::vector<std::vector<std::string>> file_content; //存储每个数据的数据结构
	std::string file_line;

	while (std::getline(file, file_line))
	{												//读取每一行数据
		std::vector<std::string> file_content_line; //存储分割之后每一行的四个数据
		std::stringstream ss;						//利用内存流进行每行数据分割内存流
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
	// cout << anwser << endl;
	return anwser;
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

	string ws_folder = fSettings["ws_folder"];
	cout << ws_folder << "associations.txt" << endl;
	std::vector<std::vector<std::string>> associations = ReadSpaceSeparatedText(ws_folder + fSettings["associations"]);
	std::vector<std::vector<std::string>> poses = ReadSpaceSeparatedText(ws_folder + fSettings["campose_path"]);
	float param_depth_scale = 1000.0f;
	const int iterStep = fSettings["iterStep"];
	const float param_voxel_size = fSettings["param_voxel_size"];

	std::ofstream of_strm(ws_folder + "frames2reconstr.txt");
	cout << "poses " << poses.size() << endl;

	for (int i = 0; i < poses.size(); i += iterStep)
	{
		// for(int i=0;i<50;i+=iterStep){

		vector<string> pose_vec = poses[i];
		vector<string>::const_iterator first = pose_vec.begin() + 1;
		vector<string>::const_iterator end = pose_vec.begin() + 8;
		vector<string> pose_vec_new(first, end);
		Eigen::Matrix4d cur_pose = ParsePose(pose_vec_new);
		int iterIndex = stoi(pose_vec[0]);
		// of_strm << setw(6)<<setfill('0') << iterIndex << endl;
		string color_path = ws_folder + associations[iterIndex][1];

		// string depth_path = ws_folder + "jbf" + associations[iterIndex][3];
		string depth_path = ws_folder + associations[iterIndex][3];
		cout << "\ncolor_path = " << color_path << endl;
		cout << "depth_path = " << depth_path << endl;

		of_strm << setw(6) << setfill('0') << stoi(associations[iterIndex][2]) << endl;

		// cout << i << endl;
	}
	return 0;
}
