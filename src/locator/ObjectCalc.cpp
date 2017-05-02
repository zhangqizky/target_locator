#include "ObjectCalc.h"
Mat calcObjPosition(const Satellite &s1, const Satellite &s2, 
        const Mat imgCoor1, const Mat imgCoor2,
        Mat dst,
        const Mat imgCoorError)
{
    // 由图像坐标计算天球坐标系下目标与卫星连线的方向向量
    auto f1 = s1.getCameraFocalLength(), f2 = s2.getCameraFocalLength();
    double fx1 = f1.at<double>(0), fy1 = f1.at<double>(1);
    double fx2 = f2.at<double>(0), fy2 = f2.at<double>(1);
    Mat Rps1 = s1.getRotateMatrixRps(); Mat Rps2 = s2.getRotateMatrixRps();
    Mat Rsc1 = s1.getRotateMatrixRsc(); Mat Rsc2 = s2.getRotateMatrixRsc();
    auto size1 = s1.getImgPlaneSize(); auto size2 = s2.getImgPlaneSize();
    int n1 = size1.at<int>(0), m1 = size1.at<int>(1);
    int n2 = size2.at<int>(0), m2 = size2.at<int>(1);
    double row1, row2, col1, col2;
    row1 = imgCoor1.at<double>(0); col1 = imgCoor1.at<double>(1);
    row2 = imgCoor2.at<double>(0); col2 = imgCoor2.at<double>(1);
    Mat temp1 = Mat::ones(3, 1, CV_64F); Mat temp2 = Mat::ones(3, 1, CV_64F); 
    temp1.at<double>(0) = (col1 - m1 / 2) / fx1;
    temp1.at<double>(1) = (n1 / 2 - row1) / fy1; 
    temp2.at<double>(0) = (col2 - m2 / 2) / fx2;
    temp2.at<double>(1) = (n2 / 2 - row2) / fy2; 



    Mat vec1 = Rps1.inv() * Rsc1.inv() * temp1;
    Mat vec2 = Rps2.inv() * Rsc2.inv() * temp2;

    // 计算天球坐标系到两颗卫星组成的编队坐标系(卫星连线为x轴, 地心与两卫星构成的平面的法向量为z轴)
    // 的旋转矩阵
    Mat rVecData = Mat::eye(3, 3, CV_64F);
    rVecData.row(0) = (s2.getPosition() - s1.getPosition()).t();
    // rVecData.row(2) = s1.getOrbitPlaneNormVec().t();
    rVecData.row(2) = (s1.getPosition().cross(s2.getPosition()).t());
    Mat tempRow = rVecData.row(2).cross(rVecData.row(0));
    tempRow.copyTo(rVecData.row(1));
    Mat RMplanet2Group = calcRotateMatrix(rVecData);
    // 计算目标到两颗卫星的俯仰角beta1, beta2
    Mat vec1InGroup = RMplanet2Group * vec1;
    Mat vec2InGroup = RMplanet2Group * vec2;
    double beta1 = asin(vec1InGroup.at<double>(2) / norm(vec1InGroup));
    double beta2 = asin(vec2InGroup.at<double>(2) / norm(vec2InGroup));

    // 计算目标与两颗卫星的方向角alfa1, alfa2
    double alpha1 = acos(vec1InGroup.at<double>(0) / (norm(vec1InGroup) * cos(beta1)));
    double alpha2 = acos(vec2InGroup.at<double>(0) / (norm(vec2InGroup) * cos(beta2)));
    // 计算两颗卫星距离b
    double b = norm(s2.getPosition() - s1.getPosition());

    // 计算目标在编队坐标系下的坐标
    Mat A = Mat::zeros(4, 3, CV_64F);
    A.at<double>(0, 0) = sin(alpha1) * cos(beta1);
    A.at<double>(0, 1) = -cos(alpha1) * cos(beta1);
    A.at<double>(1, 1) = sin(beta1);
    A.at<double>(1, 2) = -cos(beta1) * sin(alpha1);
    A.at<double>(2, 0) = sin(alpha2) * cos(beta2);
    A.at<double>(2, 1) = -cos(alpha2) * cos(beta2);
    A.at<double>(3, 1) = sin(beta2);
    A.at<double>(3, 2) = -cos(beta2) * sin(alpha2);

    Mat B = Mat::zeros(4, 1, CV_64F);
    B.at<double>(2) = b * sin(alpha2) * cos(beta2);

    Mat obj = (A.t() * A).inv() * A.t() * B;
    obj = RMplanet2Group.inv() * obj + s1.getPosition();
    obj.copyTo(dst);

    cv::Mat error = cv::Mat::zeros(3, 1, CV_64F); 
    

    return error;
}
