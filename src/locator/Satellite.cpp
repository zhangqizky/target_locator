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

Scalar rotataMat2Quaternion(const cv::Mat rot)
{
    double tr = trace(rot)(0);
    double w, x, y, z;
    double a[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            a[i][j] = rot.at<double>(i, j);
        }
    }
    if (tr > 0) 
    {
        double s = 0.5 / sqrt(tr + 1);
        w = 0.25 / s;
        x = ( a[2][1]- a[1][2]) * s;
        y = (a[0][2] - a[2][0] ) * s;
        z = (a[1][0] - a[0][1] ) * s;
        
    }
    else 
    {
        if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] ) 
        {
            double s = 2 * sqrt(1+a[0][0]-a[1][1]-a[2][2]);
            w = (a[2][1] - a[1][2]) / s;
            x = 0.25 * s;
            y = (a[0][1] + a[1][0] ) * s;
            z = (a[0][2] + a[2][0] ) * s;
        }
        else if (a[1][1] > a[2][2])
        {
            double s = 2 * sqrt(1+a[1][1]-a[0][0]-a[2][2]);
            w = (a[0][2] - a[2][0] ) / s;
            x = (a[0][1] + a[1][0] ) / s;
            y = 0.25 * s;
            z = (a[1][2] + a[2][1] ) / s;
        }
        else 
        {
            double s = 2 * sqrt(1 + a[2][2] - a[0][0] - a[1][1]);
            w = (a[1][0] - a[0][1] ) / s;
            x = (a[0][2] + a[2][0] ) / s;
            y = (a[1][2] + a[2][1] ) / s;
            z = 0.25 * s;
        }
    }

    Scalar_<double> q = {w, x, y, z};
    return q;
}
