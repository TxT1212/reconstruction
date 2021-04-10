#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

std::vector<std::vector<std::string>> ReadSpaceSeparatedText(std::string filepath);
void convertDepth2ply(Mat input, int color_flag, string saveName);
int main(int argc, char **argv)
{
    cv::FileStorage *fSettings_;
    cout << 11 << endl;

    if (argc != 2)
    {
        cout << "config file missing!" << endl;
        exit(-1);
    }
    else if (argc == 2)
    {
        fSettings_ = new cv::FileStorage(argv[1], cv::FileStorage::READ);
    }
    cv::FileStorage &fSettings = *fSettings_;
    cout << 11 << endl;
    string ws_folder = fSettings["ws_folder"];
    cout << ws_folder + fSettings["associations"] << endl;
    std::vector<std::vector<std::string>> associations = ReadSpaceSeparatedText(ws_folder + fSettings["associations"]);
    namedWindow("Display frame", CV_WINDOW_AUTOSIZE);
    for (size_t i = 0; i < associations.size(); i += 15)
    {
        Mat depth = imread(ws_folder + associations[i][1], -1);
        Mat tof = imread(ws_folder + associations[i][3], -1);
        convertDepth2ply(depth, 1, ws_folder + associations[i][1] + ".ply");
        convertDepth2ply(tof, 0, ws_folder + associations[i][3] + ".ply");
    }
}
void convertDepth2ply(Mat input, int color_flag, string saveName)
{
    double fx = 976.761670;
    double fy = 1024.149174;
    double cx = 975.702239;
    double cy = 776.218530;
    typedef cv::Point_<uint16_t> Pixel;

    input.forEach<Pixel>([&](Pixel &pixel, const int position[]) -> void {
        pixel.x-cx;
        pixel.y-cy;
        pixel.
        
    });
}
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