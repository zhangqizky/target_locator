#include<iostream>
#include<fstream>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"   
#include <opencv2/legacy/legacy.hpp>    //#include "cvAux.h"  
#include <opencv2/highgui/highgui.hpp>  
#include<vector>
#include<sstream>
#include<math.h>
#include<stdio.h>

using std::vector;
using namespace cv;
using namespace std;


class Tracker
{
private:
	//成员变量
	vector<double> lastxy;//上一帧的目标坐标
	Mat rotatematrix;//旋转矩阵的第一个，卫星的初始位置，记录下来
	int weight;//图像宽
	int height;//图像高
	Mat K;//相机内参数标定矩阵
	Mat Rsc;//卫星和相机之间的旋转矩阵
    static size_t cellsize;
    cv::Mat last_pic;
    cv::Mat last2_pic;
    cv::Mat last_rot;
    cv::Mat last2_rot;
private:
	//成员函数
	void FrameMinus(vector<Mat> pic, int num, vector<Mat>& picout);//直接帧差，本程序所用，可消除大部分恢复后的静态背景
	//void Rotateimage(Mat& input, Mat rotate, Mat& output);//恢复图片背景旋转的函数，参数分别为动背景图片--卫星此刻旋转矩阵--静背景图片
	int dirfilter(vector<Mat> &pic, int num);//方向滤波
	int maxfilter(Mat pic);//领域最大值滤波，用在方向滤波里面
	int mixsolo(Mat dest, Mat source);//混合两幅图像，用在方向滤波里面
	void quickSort(vector<int> &vec, int l, int r);//快速排序，用在过门限阈值分割里面
	bool Trace(vector<Mat>& pic, int num, vector<double>& coordinate);//寻找每幅图像上最亮的点并返回坐标值，检测函数，捕获目标的函数
public:
	Mat getRsc() const { return Rsc; }
    cv::Mat getK() const { return K; }
    cv::Mat getRps() const { return rotatematrix; }
	void Rotateimage(Mat& input, Mat rotate, Mat& output);//单帧图片恢复背景旋转的函数，用于跟踪部分。参数分别为此刻动背景图片--卫星此刻旋转矩阵--静背景图片
    void Rotateimageinv(Mat rotate);
	//目标图像坐标提取接口函数，输入为:图像序列pic1【20张】，旋转矩阵  输出为：直接解算出的目标坐标，存储形式为{(x1,y1),(x2,y2),(x3,y3)....}
	bool ExtractTarget(vector<Mat>&pic1, vector<Mat>& Rotatematrix, vector<double>&coordinate);//也就是师兄你说的初始化函数
	//跟踪目标，使用了卡尔曼滤波
	bool Tracking(Mat& thisimage, double& thisx, double& thisy);

	void setParameter(int sizex, int sizey, double fx, double fy, cv::Mat Rsc);
    // 当前图像和之前两帧的图像作帧差
    cv::Mat diffMat(cv::Mat current_pic, cv::Mat current_rot);
};
