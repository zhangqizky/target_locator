#include<iostream>
#include<fstream>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"   
#include <opencv2/legacy/legacy.hpp>    //#include "cvAux.h"  
#include <opencv2/highgui/highgui.hpp>  
#include<vector>
#include<sstream>
#include<math.h>
#include<stdio.h>

using std::vector;
using namespace cv;
using namespace std;


class Tracker
{
private:
	//��Ա����
	vector<double> lastxy;//��һ֡��Ŀ������
	Mat rotatematrix;//��ת����ĵ�һ�������ǵĳ�ʼλ�ã���¼����
	int weight;//ͼ���
	int height;//ͼ���
	Mat K;//����ڲ����궨����
	Mat Rsc;//���Ǻ����֮�����ת����
    static size_t cellsize;
    cv::Mat last_pic;
    cv::Mat last2_pic;
    cv::Mat last_rot;
    cv::Mat last2_rot;
private:
	//��Ա����
	void FrameMinus(vector<Mat> pic, int num, vector<Mat>& picout);//ֱ��֡����������ã��������󲿷ָֻ���ľ�̬����
	//void Rotateimage(Mat& input, Mat rotate, Mat& output);//�ָ�ͼƬ������ת�ĺ����������ֱ�Ϊ������ͼƬ--���Ǵ˿���ת����--������ͼƬ
	int dirfilter(vector<Mat> &pic, int num);//�����˲�
	int maxfilter(Mat pic);//�������ֵ�˲������ڷ����˲�����
	int mixsolo(Mat dest, Mat source);//�������ͼ�����ڷ����˲�����
	void quickSort(vector<int> &vec, int l, int r);//�����������ڹ�������ֵ�ָ�����
	bool Trace(vector<Mat>& pic, int num, vector<double>& coordinate);//Ѱ��ÿ��ͼ���������ĵ㲢��������ֵ����⺯��������Ŀ��ĺ���
public:
	Mat getRsc() const { return Rsc; }
    cv::Mat getK() const { return K; }
    cv::Mat getRps() const { return rotatematrix; }
	void Rotateimage(Mat& input, Mat rotate, Mat& output);//��֡ͼƬ�ָ�������ת�ĺ��������ڸ��ٲ��֡������ֱ�Ϊ�˿̶�����ͼƬ--���Ǵ˿���ת����--������ͼƬ
    void Rotateimageinv(Mat rotate);
	//Ŀ��ͼ��������ȡ�ӿں���������Ϊ:ͼ������pic1��20�š�����ת����  ���Ϊ��ֱ�ӽ������Ŀ�����꣬�洢��ʽΪ{(x1,y1),(x2,y2),(x3,y3)....}
	bool ExtractTarget(vector<Mat>&pic1, vector<Mat>& Rotatematrix, vector<double>&coordinate);//Ҳ����ʦ����˵�ĳ�ʼ������
	//����Ŀ�꣬ʹ���˿������˲�
	bool Tracking(Mat& thisimage, double& thisx, double& thisy);

	void setParameter(int sizex, int sizey, double fx, double fy, cv::Mat Rsc);
    // ��ǰͼ���֮ǰ��֡��ͼ����֡��
    cv::Mat diffMat(cv::Mat current_pic, cv::Mat current_rot);
};
