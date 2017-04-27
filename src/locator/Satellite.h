#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <exception>

using namespace cv;
using namespace std;

class Camera
{
private:
    int n, m;  // 像平面行/列上的像素个数
    double fx, fy;   // 摄像机焦距(以像元的尺寸的倍数给出)
                // 即实际焦距为 f * sizeOfPixle
    double sizeOfPixle; // 像元尺寸
    Mat dirVec; // 摄像机坐标系坐标轴方向向量在卫星坐标系下的表示
    // 旋转矩阵, 使坐标从卫星坐标系变换到相机坐标系
    // 默认使相机光轴(z轴)指向卫星坐标系x轴,相机x轴指向卫星y轴
    // 相机y轴指向卫星z轴
    Mat Rsc;    
    // Rsc 对应的四元数
    Scalar quat;
public:
    Camera(double sizeOfPixle, int n, int m, double fx, double fy = 0);
    Camera();
    void setParams(double sizeOfPixle, int n, int m, double fx, double fy = 0);
    void updateQuaternion(const Scalar quat);
    int getN() const { return n; }
    int getM() const { return m; }
    double getFx() const { return fx; }
    double getFy() const { return fy; }
    Mat getRsc() const { return Rsc.clone(); }
};

class Satellite
{
private:
    // 卫星搭载相机
    Camera c;
    // 卫星当前时刻瞬时位置
    Mat position;
    // 旋转矩阵,使目标坐标从天球坐标系变换到卫星坐标系
    Mat Rps;
    // 与Rps对应的四元数
    Scalar quat;
    // 计算天球坐标系到卫星坐标系的旋转矩阵
    void calcRps(const Mat dirVec);
public:
    Satellite();
    // 更新卫星位置 
    void updatePos(const cv::Mat pos);
    // 更新卫星姿态
    void updateDirectionVec(const Mat dirVec);
    void updateQuaternion(const Scalar quat);

    void setCameraParams(double sizeOfPixle, int n, int m, double fx, double fy = 0) 
    {
        c.setParams(sizeOfPixle, n, m, fx, fy);
    }

    // getters
    Mat getPosition() const { return position.clone(); }
    cv::Mat getImgPlaneSize() const {return (cv::Mat_<int>(2, 1) << c.getN(), c.getM()); } 
    cv::Mat getCameraFocalLength() const { return (cv::Mat_<double>(2, 1) << c.getFx() , c.getFy()); }
    Mat getRotateMatrixRps() const { return Rps.clone(); }
    Mat getRotateMatrixRsc() const { return c.getRsc(); }
    Scalar getQuat() const { return quat; }
};

Mat calcRotateMatrix(const Mat dirVec);
void calcVecCosines(const Mat dir, Mat cosines);
// 四元数转换为旋转矩阵
Mat quaternion2RotateMat(const cv::Scalar q);
// 旋转矩阵转换为四元数
Scalar rotataMat2Quaternion(const cv::Mat rot);  
