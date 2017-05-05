#include "Satellite.h"

// region Camera
Camera::Camera() : 
    n(0), m(0),
    fx(0), fy(0),
    sizeOfPixle(0),
    dirVec(cv::Mat::eye(3, 3, CV_64F)),
    Rsc(cv::Mat::eye(3, 3, CV_64F)),
    quat(cv::Scalar_<double>(1, 1, 1, 1)) 
{
    Rsc = (cv::Mat_<double>(3, 3) << 0, 1, 0,
        0, 0, 1,
        1, 0, 0);
}
Camera::Camera(double sizeOfPixle, int n, int m, double fx, double fy)
{
    setParams(sizeOfPixle, n, m, fx, fy);
    dirVec = Mat::eye(3, 3, CV_64F);
    Rsc = calcRotateMatrix(dirVec);
    quat = rotataMat2Quaternion(Rsc);
}
void Camera::setParams(double sizeOfPixle, int n, int m, double fx, double fy) {
    this->n = n;
    this->m = m;
    this->fx = fx / sizeOfPixle;
    if (std::abs(fy - 0) < 1e-8) {
        this->fy = fx / sizeOfPixle;
    }
    else {
        this->fy = fy / sizeOfPixle;
    }
    this->sizeOfPixle = sizeOfPixle;
}
void Camera::updateQuaternion(const Scalar quat)
{
    this->quat = quat;
    Rsc = quaternion2RotateMat(quat);
}


// endregion

// region Satellite

Satellite::Satellite() : 
    position(cv::Mat::zeros(3, 1, CV_64F)),
    Rps(cv::Mat::eye(3, 3, CV_64F)),
    quat(cv::Scalar_<double>(1, 1, 1, 1))
{}

void Satellite::updatePos(const cv::Mat pos) {
    pos.copyTo(this->position);
}

void Satellite::updateDirectionVec(const Mat dirVec)
{
    calcRps(dirVec);
    quat = rotataMat2Quaternion(Rps);
}

void Satellite::updateQuaternion(const Scalar quat)
{
    this->quat = quat;
    Rps = quaternion2RotateMat(quat);
}

void Satellite::calcRps(const Mat dirVec)
{
    Rps = calcRotateMatrix(dirVec);
}


// endregion


Mat calcRotateMatrix(const Mat dirVec)
{
    Mat c1(3, 1, CV_64F), c2(3, 1, CV_64F), c3(3, 1, CV_64F);
    calcVecCosines(dirVec.row(0), c1);
    calcVecCosines(dirVec.row(1), c2);
    calcVecCosines(dirVec.row(2), c3);
    Mat temp = Mat::eye(3, 3, CV_64F);
    temp.row(0) = c1.t();
    temp.row(1) = c2.t();
    temp.row(2) = c3.t();
    return temp;
}
void calcVecCosines(const Mat dir, Mat cosines)
{
    double v1 = dir.at<double>(0);
    double v2 = dir.at<double>(1);
    double v3 = dir.at<double>(2);
    double length = sqrt(v1*v1 + v2*v2 + v3*v3);
    double tmp[] = { v1 / length, v2 / length, v3 / length };
    Mat cosinesTmp(3, 1, CV_64F, tmp);
    cosinesTmp.copyTo(cosines);
}

Mat quaternion2RotateMat(const cv::Scalar q)
{
    double w = q(0), x = q(1), y = q(2), z = q(3);
    double sqw = w*w, sqx = x*x, sqy = y*y, sqz = z*z;

    double invs = 1 / (sqx + sqy + sqz + sqw);
    double m00 = ( sqx - sqy - sqz + sqw)*invs;
    double m11 = (-sqx + sqy - sqz + sqw)*invs ;
    double m22 = (-sqx - sqy + sqz + sqw)*invs ;

    double tmp1 = x * y;
    double tmp2 = z * w;
    double m10 = 2.0 * (tmp1 + tmp2)*invs ;
    double m01 = 2.0 * (tmp1 - tmp2)*invs ;

    tmp1 = x*z;
    tmp2 = y*w;
    double m20 = 2.0 * (tmp1 - tmp2)*invs ;
    double m02 = 2.0 * (tmp1 + tmp2)*invs ;

    tmp1 = y*z;
    tmp2 = x*w;
    double m21 = 2.0 * (tmp1 + tmp2)*invs ;
    double m12 = 2.0 * (tmp1 - tmp2)*invs ;


    Mat rotateMat = (Mat_<double>(3, 3) << m00, m01, m02,
                                           m10, m11, m12,
                                           m20, m21, m22);
    return rotateMat;
}
double SIGN(double x) {return (x >= 0.0 ? 1.0 : -1.0) ;}

Scalar rotataMat2Quaternion(const cv::Mat rot)
{
    double r[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r[i][j] = rot.at<double>(i, j);
        }
    }
    double q0, q1, q2, q3;
    q0 = (r[0][0] + r[1][1] + r[2][2] + 1.0) / 4.0;
    q1 = (r[0][0] - r[1][1] - r[2][2] + 1.0) / 4.0;
    q2 = (-r[0][0] + r[1][1] - r[2][2] + 1.0) / 4.0;
    q3 = (-r[0][0] - r[1][1] + r[2][2] + 1.0) / 4.0;
    if (q0 < 0.0) q0 = 0.0;
    if (q1 < 0.0) q1 = 0.0;
    if (q2 < 0.0) q2 = 0.0;
    if (q3 < 0.0) q3 = 0.0;
    q0 = std::sqrt(q0);
    q1 = std::sqrt(q1);
    q2 = std::sqrt(q2);
    q3 = std::sqrt(q3);

    if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
        q0 *= 1.0;
        q1 *= SIGN(r[2][1] - r[1][2]);
        q2 *= SIGN(r[0][2] - r[2][0]);
        q3 *= SIGN(r[1][0] - r[0][1]);
    } else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
        q0 *= SIGN(r[2][1] - r[1][2]);
        q1 *= 1.0;
        q2 *= SIGN(r[1][0] + r[0][1]);
        q3 *= SIGN(r[0][2] + r[2][0]);
    } else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
        q0 *= SIGN(r[0][2] - r[2][0]);
        q1 *= SIGN(r[1][0] + r[0][1]);
        q2 *= 1.0;
        q3 *= SIGN(r[2][1] + r[1][2]);
    } else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
        q0 *= SIGN(r[1][0] - r[0][1]);
        q1 *= SIGN(r[2][0] + r[0][2]);
        q2 *= SIGN(r[2][1] + r[1][2]);
        q3 *= 1.0;
    }

    double norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    return cv::Scalar_<double> (q0, q1, q2, q3);
}
