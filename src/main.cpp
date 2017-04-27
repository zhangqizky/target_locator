#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "locator/locator.h"
#include <fstream>

int main() {
    Locator locator;
    double time1, x1, y1, z1, q11, q21, q31, q41;
    double time2, x2, y2, z2, q12, q22, q32, q42;
    cv::Mat img1, img2;
    std::fstream s1("filename");
    std::fstream s2("filename");
    std::string name1, name2;
    while (s1 >> time1 >> x1 >> y1 >> z1 >> q11 >> q21 >> q31 >> q41 && 
            s2 >> time2 >> x2 >> y2 >> z2 >> q12 >> q22 >> q32 >> q42) {
        img1 = imread(name1, 0);
        img2 = imread(name2, 0);
        cv::Mat pos1 = cv::Mat_<double>(3, 1) << x1, y1, z1;
        cv::Mat pos2 = cv::Mat_<double>(3, 1) << x2, y2, z2;
        cv::Scalar quat1(q11, q21, q31, q41);
        cv::Scalar quat2(q12, q22, q32, q42);
        locator.updatePositionAndQuat(pos1, quat1, pos2, quat2);
        cv::Mat res = locator.locate(img1, img2);
    }
    
}
