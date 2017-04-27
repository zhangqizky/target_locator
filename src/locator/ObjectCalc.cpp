#include "ObjectCalc.h"

Mat calcObjPosition(const Satellite &s1, const Satellite &s2, 
        const Mat imgCoor1, const Mat imgCoor2,
        Mat dst,
        const Mat imgCoorError)
{
    // 由图像坐标计算天球坐标系下目标与卫星连线的方向向量
    cv::Mat focus1 = s1.getCameraFocalLength(); cv::Mat focus2 = s2.getCameraFocalLength();  
    double fx1 = focus1.at<double>(0); double fy1 = focus2.at<double>(1);
    double fx2 = focus2.at<double>(0); double fy2 = focus2.at<double>(1);

    Mat Rps1 = s1.getRotateMatrixRps(); Mat Rps2 = s2.getRotateMatrixRps();
    Mat Rsc1 = s1.getRotateMatrixRsc(); Mat Rsc2 = s2.getRotateMatrixRsc();

    Mat num_pixels1 = s1.getImgPlaneSize(); Mat num_pixels2 = s2.getImgPlaneSize();
    int n1 = num_pixels1.at<int>(0); int m1 = num_pixels1.at<int>(1);
    int n2 = num_pixels2.at<int>(0); int m2 = num_pixels2.at<int>(1);

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

    
    Mat obj = calcIntersection(vec1, vec2, s1.getPosition(), s2.getPosition());
    obj.copyTo(dst);

    Mat error = Mat::zeros(3, 1, CV_64F);
    return error;
}

Mat calcIntersection(const cv::Mat v1, const cv::Mat v2,
        const cv::Mat p1, const cv::Mat p2) {
    cv::Mat i1, i2;
    cv::normalize(v1, i1);
    cv::normalize(v2, i2);
    cv::Mat m1 = cv::Mat::eye(3, 3, CV_64F) - i1 * i1.t();
    cv::Mat m2 = cv::Mat::eye(3, 3, CV_64F) - i2 * i2.t();
    cv::Mat intersection = (m1 + m2).inv(cv::DECOMP_SVD) * (m1 * p1 + m2 * p2);
    return intersection;
}
