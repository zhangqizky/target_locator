#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "locator/locator.h"
#include <fstream>
bool checkEqual(Mat target, Mat coor)
{
    for (int i = 0; i < 3; i++)
    {
        if (abs(target.at<double>(i) - coor.at<double>(i)) > 1E-7)
            return false;
    }
    return true;
}

int main() {
    Locator locator;
    double time1, x1, y1, z1, q11, q21, q31, q41;
    double time2, x2, y2, z2, q12, q22, q32, q42;
    double sizeOfPixle, fx, fy;
    int n, m;
    cv::Mat img1, img2;
    std::string img1_name("data/images_data/sequence1-");
    std::string img2_name("data/images_data/sequence2-");
    std::fstream s1("data/s1.dat");
    std::fstream s2("data/s2.dat");
    std::fstream cam("data/cam.dat");
    std::fstream target("data/target.dat");
    std::ofstream location("loc.dat");
    std::ofstream err("err.dat");
    double x, y, z;

    cam >> sizeOfPixle >> n >> m >> fx;
    fy = fx;
    locator.setCameraParams(sizeOfPixle, n, m, fx, fy);
    int cnt = 1;
    int i = 1;
    while (s1 >> time1 >> x1 >> y1 >> z1 >> q11 >> q21 >> q31 >> q41 && 
            s2 >> time2 >> x2 >> y2 >> z2 >> q12 >> q22 >> q32 >> q42) {
        target >> x >> y >> z;
        cv::Mat targ = (cv::Mat_<double>(3, 1) << x, y, z);
        std::ostringstream name1;
        std::ostringstream name2;
        name1 << img1_name << i << ".jpg";
        name2 << img2_name << i << ".jpg";
        i++;
        // std::cout << name1.str() << std::endl;
        img1 = imread(name1.str(), 0);
        img2 = imread(name2.str(), 0);
        cv::Mat pos1 = (cv::Mat_<double>(3, 1) << x1, y1, z1);
        cv::Mat pos2 = (cv::Mat_<double>(3, 1) << x2, y2, z2);
        cv::Scalar quat1(q11, q21, q31, q41);
        cv::Scalar quat2(q12, q22, q32, q42);
        locator.updatePositionAndQuat(pos1, quat1, pos2, quat2);
        cv::Mat res = locator.locate(img1, img2);
        location << res.at<double>(0) << " " << res.at<double>(1) << " " << res.at<double>(2) << std::endl;
        // if (!checkEqual(targ, res)) {
            err << "===============" << std::endl << cnt++ << std::endl;

            cv::Mat error = res - (cv::Mat_<double>(3, 1) << x, y, z);
            // err << x << " " << y << " " << z << std::endl;
            err << error.t() << "\t" << norm(error) << std::endl;
        // }
    }
    
}
