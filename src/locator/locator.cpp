#include "locator.h"

size_t Locator::threshold = 4;

void Locator::updatePositionAndQuat(cv::Mat pos1, cv::Scalar quat1,
        cv::Mat pos2, cv::Scalar quat2) {
    m_s1.updatePos(pos1);
    m_s1.updateQuaternion(quat1);

    m_s2.updatePos(pos2);
    m_s2.updateQuaternion(quat2);
}

cv::Mat Locator::locate(cv::Mat img1_pic, cv::Mat img2_pic) {
    if (!m_initialized) {
        if (m_pics1.size() == threshold) {
            m_pics1.pop(); m_rotateMat1.pop();
            m_pics2.pop(); m_rotateMat2.pop();
        }
        m_pics1.push(img1_pic); m_rotateMat1.push(m_s1.getRotateMatrixRps());
        m_pics2.push(img2_pic); m_rotateMat2.push(m_s2.getRotateMatrixRps());

        if (m_pics1.size() == threshold) {
            m_initialized = initialize();
        }
    }
    if (m_initialized) {
        cv::Mat_<double> imgCoor1(2, 1), imgCoor2(2, 1);
        cv::Mat objCoor;
        double row1, col1, row2, col2;
        cv::Mat img_diffed1 = m_tracker1.diffMat(img1_pic, m_s1.getRotateMatrixRps());
        cv::Mat img_diffed2 = m_tracker2.diffMat(img2_pic, m_s2.getRotateMatrixRps());
        if (m_tracker1.Tracking(img_diffed1, col1, row1) &&
                m_tracker2.Tracking(img_diffed2, col2, row2)) {

            imgCoor1 << row1, col1;
            imgCoor2 << row2, col2;

            cv::Mat error =  calcObjPosition(m_s1, m_s2, imgCoor1, imgCoor2, objCoor);
        }
        else {
            objCoor = cv::Mat::zeros(3, 1, CV_64F);
        }

        return objCoor;
    }
    else {
        return cv::Mat::zeros(3, 1, CV_64F);
    }
}

bool Locator::initialize() {
    std::vector<cv::Mat> pics1, rot1, pics2, rot2;
    std::vector<double> coor1, coor2;
    while (!m_pics1.empty()) {
        pics1.push_back(m_pics1.front()); m_pics1.pop();
        rot1.push_back(m_rotateMat1.front()); m_rotateMat1.pop();
        pics2.push_back(m_pics2.front()); m_pics2.pop();
        rot2.push_back(m_rotateMat2.front()); m_rotateMat2.pop();
    }
    for (size_t i = 0; i < pics1.size(); i++) {
        m_pics1.push(pics1[i]); m_rotateMat1.push(rot1[i]);
        m_pics2.push(pics2[i]); m_rotateMat2.push(rot2[i]);
    }
    bool res =  m_tracker1.ExtractTarget(pics1, rot1, coor1) &&
        m_tracker2.ExtractTarget(pics2, rot2, coor2);
    if (res) {
        for (size_t i = 0; i < pics1.size(); i++) {
            m_coors1.push_back((cv::Mat_<double>(2, 1) << coor1[i * 2 + 1], coor1[i * 2]));
            m_coors2.push_back((cv::Mat_<double>(2, 2) << coor2[i * 2 + 2], coor2[i * 2]));
        }
    }
    return res;
}


