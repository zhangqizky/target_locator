#ifndef __OBJ_CALC__
#define __OBJ_CALC__
#include "Satellite.h"
#include "opencv2/opencv.hpp"
using namespace cv;

// 计算目标空间位置(天球坐标系)
// 传入参数为两颗卫星的参数以及目标在两颗卫星上投影得到的图像坐标
// 最后传入存储目标位置的Mat, 返回目标位置的计算误差
// 目标位置误差以(deltaX, deltaY, deltaZ)的形式给出
Mat calcObjPosition(const Satellite &s1, const Satellite &s2, 
        const Mat imgCoor1, const Mat imgCoor2,
        Mat dst, 
        const Mat imgCoorError = Mat::zeros(2, 1, CV_64F));
Mat calcIntersection(const cv::Mat v1, const cv::Mat v2,
        const cv::Mat p1, const cv::Mat p2);
#endif
