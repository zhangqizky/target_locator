#ifndef __LOCATOR__
#define __LOCATOR__

#include <vector>
#include <opencv2/opencv.hpp>
#include <queue>
#include "Satellite.h"
#include "ObjectCalc.h"
#include "../detector/Target.h"

class Locator {
private:
public:
    Satellite m_s1;
    Satellite m_s2;
    Tracker m_tracker1;
    Tracker m_tracker2;
    std::queue<cv::Mat> m_pics1;
    std::queue<cv::Mat> m_pics2;
    std::queue<cv::Mat> m_rotateMat1;
    std::queue<cv::Mat> m_rotateMat2;
    std::vector<cv::Mat> m_coors1;
    std::vector<cv::Mat> m_coors2;
    bool m_initialized;
    static size_t threshold;
private:
    bool initialize();
public:
    void updatePositionAndQuat(cv::Mat pos1, cv::Scalar quat1,
            cv::Mat pos2, cv::Scalar quat2);
    void setCameraParams(cv::Scalar cam_quat, double sizeOfPixle, int n, int m, double fx, double fy = 0) {
        m_s1.setCameraParams(sizeOfPixle, n, m, fx, fy);
        m_s2.setCameraParams(sizeOfPixle, n, m, fx, fy);
        m_s1.setCamPos(cam_quat);
        m_s2.setCamPos(cam_quat);
        m_tracker1.setParameter(m, n, fx / sizeOfPixle, fy / sizeOfPixle, m_s1.getRotateMatrixRsc());
        m_tracker2.setParameter(m, n, fx / sizeOfPixle, fy / sizeOfPixle, m_s2.getRotateMatrixRsc());
    }
    cv::Mat locate(cv::Mat img1_pic, cv::Mat img2_pic);
};
#endif
