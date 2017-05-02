#include"Target.h"

size_t Tracker::cellsize = 5;
void Tracker::FrameMinus(vector<Mat> pic, int num, vector<Mat>& picout)
{
	int col = pic[0].cols;//ͼ����У�Ҳ����x���꣬���
	int row = pic[0].rows;//ͼ����У�Ҳ����y���꣬�߶�
	picout.push_back(pic[0] - pic[1]);
	for (int i = 0; i < num - 3; i++)
	{
		Mat temp;
		temp.create(row, col, CV_8U);
		temp = pic[i + 1] - pic[i + 3];
		picout.push_back(temp);
	}
	picout.push_back(pic[num - 2] - pic[num - 4]);
	picout.push_back(pic[num - 1] - pic[num - 3]);
}
int Tracker::dirfilter(vector<Mat> &pic1, int num)//�����˲�
{
	Mat *pic;
	pic = new Mat[num];
	for (int i = 0; i<num; i++)
	{
		pic[i] = pic1[i];              //��vector����ת��Ϊָ��İ취
	}
	for (int i = 0; i<num; i++)//��ѭ�����ã���Ե��������
	{
		for (int j = 0; j<pic[0].rows; j++)
		{
			uchar* data = pic[0].ptr<uchar>(j);
			uchar* data2 = pic[i].ptr<uchar>(j);

			for (int k = 0; k<pic[0].cols; k++)
			{
				if (i == 0)
				{
					if (j<5 || j>(pic[0].rows - 6) || k<5 || k>(pic[0].cols - 6))//��Ե���ص�ȫ������
						data2[k] = 0;
				}
				else if (j<5 || j>(pic[0].rows - 6) || k<5 || k>(pic[0].cols - 6))//��Ե���ص�ȫ������
					data2[k] = 0;
			}
		}
	}

	Mat *guestpic = new Mat[num]; Mat *hostpic = new Mat[num];
	for (int i = 0; i<num; i++)//��ѭ����pic���и�ֵ��hostpic���ٽ�pic��������
	{
		hostpic[i] = pic[i].clone();
		for (int j = 3; j<pic[0].rows - 3; j++)
		{
			uchar* data = pic[i].ptr<uchar>(j);
			for (int k = 3; k<pic[0].cols - 3; k++)
			{
				data[k] = 0;
			}
		}
	}


	for (int m = -3; m <= 3; m = m + 3)//��ѭ��ִ��˫�����˲���m��n���ڿ���8����ͬ����
	{
		for (int n = -3; n <= 3; n = n + 3)
		{
			if (m == 0 && n == 0)//�ų����ĵ�
				continue;

			for (int i = 0; i<num; i++)
			{
				guestpic[i] = hostpic[i].clone();
			}
			Mat temppic = guestpic[0].clone();//��¼��һ�Ŵ�����ͼ

			for (int i = 1; i<num; i++)//��ѭ��Ϊ�������˲�
			{
				maxfilter(temppic);//�������ֵ�˲�
				Mat kernal = (Mat_<float>(3, 3) << 0.2, 0.2, 0.2, 0.2, 0.6, 0.2, 0.2, 0.2, 0.2);//ע�����˵ĸ���ʽ
				filter2D(temppic, temppic, -1, kernal);
				for (int j = 3; j<guestpic[0].rows - 3; j++)
				{
					uchar* data = temppic.ptr<uchar>(j + m);
					uchar* mixdata = guestpic[i].ptr<uchar>(j);

					for (int k = 3; k<guestpic[0].cols - 3; k++)
					{
						int x1 = data[k + n] / 50;
						int x = mixdata[k] / 50;//˫���˼�룬��ǰ֡��j,k������ǰһ֡(j+2,k-2)��������໥Ӱ�죬���ҽ����������ȶ��ϸ�ʱ�Ż��
						//��ǰ�㷢���ϴ��ǿ�������������
						float y = x*0.5;
						float y1 = x1*0.5;
						int value = y*data[k + n] + y1*mixdata[k];
						if (value>255) value = 255;//��ֹ�Ҷ�ֵ����255
						mixdata[k] = value;

					}
				}
				temppic = guestpic[i].clone();
			}

			for (int i = num - 2; i >= 0; i--)//��ѭ��Ϊ�������˲�
			{

				maxfilter(temppic);//�������ֵ�˲�
				Mat kernal = (Mat_<float>(3, 3) << 0.2, 0.2, 0.2, 0.2, 0.6, 0.2, 0.2, 0.2, 0.2);//ע�����˵ĸ���ʽ
				filter2D(temppic, temppic, -1, kernal);
				for (int j = 3; j<guestpic[0].rows - 3; j++)
				{
					uchar* data = temppic.ptr<uchar>(j - m);
					uchar* mixdata = guestpic[i].ptr<uchar>(j);

					for (int k = 3; k<guestpic[0].cols - 3; k++)
					{

						int x1 = data[k - n] / 50;
						int x = mixdata[k] / 50;
						float y = x*0.5;
						float y1 = x1*0.5;
						int value = y*data[k - n] + y1*mixdata[k];
						if (value>255) value = 255;
						mixdata[k] = value;
					}
				}
				temppic = guestpic[i].clone();
			}

			for (int i = 0; i<num; i++)//��Сѭ���ǰѴ�����guestpic�������¸�ֵ��pic���У��Ӷ�����������pic���д洢�ľ����˲��������
				//pic[i]=guestpic[i].clone();
				mixsolo(pic[i], guestpic[i]);//����ǰѴ���Ľ�����¸�����pic��pic���������ã��Ӷ�pic���Ǳ��˲��������

		}
	}

	delete[]hostpic; delete[]guestpic;

	return 1;
}
int Tracker::maxfilter(Mat pic)//�������ֵ�˲�
{
	Mat tpic = pic.clone();
	for (int i = 2; i<pic.rows - 2; i++)
	{
		uchar* pdata = pic.ptr<uchar>(i);
		for (int j = 2; j<pic.cols - 2; j++)
		{
			uchar* data = tpic.ptr<uchar>(i);
			float x = data[j];
			for (int m = 0; m<5; m++)//��ѭ��Ѱ��5*5�������ֵ
			{
				data = tpic.ptr<uchar>(i - 2 + m);
				for (int n = 0; n<5; n++)
				{

					if (data[j - 2 + n]>x)
						x = data[j - 2 + n];
				}
			}
			pdata[j] = x;

		}
	}
	return 1;
}
int Tracker::mixsolo(Mat dest, Mat source)//�ϳ�����ͼ��
{
	int data = 0;
	for (int i = 0; i<dest.rows; i++)
	{
		uchar* data1 = dest.ptr<uchar>(i);
		uchar* data2 = source.ptr<uchar>(i);
		for (int j = 0; j<dest.cols; j++)
		{
			data = data1[j] + data2[j];
			if (data>255) data = 255;
			data1[j] = data;
		}
	}
	return 1;
}
void Tracker::quickSort(vector<int> &vec, int l, int r)  //��������
{
	if (l< r)
	{
		int i = l, j = r, x = vec[l];
		while (i < j)
		{
			while (i < j && vec[j] >= x) // ���������ҵ�һ��С��x����  
				j--;
			if (i < j)
				vec[i++] = vec[j];
			while (i < j && vec[i]< x) // ���������ҵ�һ�����ڵ���x����  
				i++;
			if (i < j)
				vec[j--] = vec[i];
		}
		vec[i] = x;
		quickSort(vec, l, i - 1); // �ݹ����  
		quickSort(vec, i + 1, r);
	}
}
bool Tracker::Trace(vector<Mat>& pic, int num, vector<double>& coordinate)
{
	int size = (pic[0].cols)*(pic[0].rows);
	for (int i = 0; i < num; i++)
	{
		vector<int> vec;//��¼���е�����Ҷ�ֵ��Ȼ��Ը������еĻҶ�ֵ��������������ߵ����ɵ㵽vecpic����ȡ������λ��
		vector<int> vecpic;//�����Ǽ�¼ĳ������ֵ���ڵ�ͼ������
		for (int j = 0; j<pic[0].rows; j++)
		{
			uchar* data = pic[i].ptr<uchar>(j);
			for (int k = 0; k<pic[0].cols; k++)
			{
				if (data[k]>30)
				{
					vec.push_back(data[k]);
					vecpic.push_back(data[k]);
					vecpic.push_back(j);
					vecpic.push_back(k);
				}
				data[k] = 0;//������Ԫ�ػҶ�ֵȫ����0�������ٽ����ȸߵ���255��ʵ�ֶ�ֵ��
			}
		}

		int lightnum = 3;
		if (vec.size()>0)
		{
			quickSort(vec, 0, vec.size() - 1);//��������������Ԫ�ظ�������size-1����ס����Ϊ�������򣬴Ӻ���ǰ�������ҵ���������������
			int vecsize = vec.size();
			int lastvalue = vec[vecsize - 1];//�����ų��ظ�����ֵ���˴�ֻ����㸳һ����ֵ
			int num = 0;//��¼�Ѿ��ҵ������ص������������ֵ������ѭ��
			for (int n = vecsize - 1; n >= 0; n--)//��ѭ�����ã������������ɸ�ֵ��vecpic�ж�ȡ��Ӧ�����꣬���ҽ���Ӧ������Ϊ255
			{
				int value = vec[n];
				if (n != (vecsize - 1) && value == lastvalue)//�������ظ�ֵʱ�������Ѿ���¼��һ�飬�������
					continue;
				lastvalue = value;
				for (unsigned long m = 0; m<(vecpic.size() / 3); m++)
				{
					if (vecpic[3 * m] == value)
					{
						uchar* pdata = pic[i].ptr<uchar>(vecpic[3 * m + 1]);
						//pdata[vecpic[3 * m + 2]] = 255;
						pdata[vecpic[3 * m + 2]] = vecpic[3 * m];
						num++;
					}
					if (num >= lightnum)
						break;
				}
				if (num >= lightnum)
					break;
			}
			//���ûҶ����ļ�Ȩ�㷨����Ŀ������������
			double x = 0, y = 0;
			double xsum = 0, ysum = 0, sum = 0;
			for (unsigned long m = 0; m < (vecpic.size() / 3); m++)
			{
				sum += vecpic[3 * m];
				xsum += vecpic[3 * m] * vecpic[3 * m + 2];
				ysum += vecpic[3 * m] * vecpic[3 * m + 1];
			}
			x = xsum / sum;//��Ӧ����k
			y = ysum / sum;
			coordinate.push_back(x);
			coordinate.push_back(y);
			vector<int>().swap(vec);
			vector<int>().swap(vecpic);
		}
	}
	if (coordinate.size() == cellsize * 2)
	{
		return true;
	}
	else
	{
		return false;
	}
}
void Tracker::Rotateimage(Mat& input, Mat rotate, Mat& output)
{
	//Mat back1 = Mat::zeros(Size(size, size), CV_8U);//����һ��������Ϊ����ʹ��
	Mat tmp(Size(3, 3), CV_64FC1);
	Mat HPI(Size(3, 3), CV_64FC1);
	Mat warp(Size(3, 2), CV_64FC1);

	tmp = (Rsc*rotatematrix).inv();
	tmp = (Rsc*rotate)*tmp;
	HPI = K*tmp.inv()*K.inv();

	warp.at<double>(0, 0) = HPI.at<double>(0, 0);
	warp.at<double>(0, 1) = HPI.at<double>(0, 1);
	warp.at<double>(0, 2) = (HPI.at<double>(0, 2));
	warp.at<double>(1, 0) = HPI.at<double>(1, 0);
	warp.at<double>(1, 1) = HPI.at<double>(1, 1);
	warp.at<double>(1, 2) = HPI.at<double>(1, 2);

	warpAffine(input, output, warp, output.size(), 2);//�����Բ�ֵӳ���ȥ
}
void Tracker::Rotateimageinv(Mat rotate)
{
	//Mat back1 = Mat::zeros(Size(size, size), CV_8U);//����һ��������Ϊ����ʹ��
	Mat tmp(Size(3, 3), CV_64FC1);
	Mat HPI(Size(3, 3), CV_64FC1);
	Mat warp(Size(3, 2), CV_64FC1);

	tmp = (Rsc*rotatematrix).inv();
	tmp = (Rsc*rotate)*tmp;
	HPI = K*tmp.inv()*K.inv();
	HPI = HPI.inv();
	warp.at<double>(0, 0) = HPI.at<double>(0, 0);
	warp.at<double>(0, 1) = HPI.at<double>(0, 1);
	warp.at<double>(0, 2) = (HPI.at<double>(0, 2));
	warp.at<double>(1, 0) = HPI.at<double>(1, 0);
	warp.at<double>(1, 1) = HPI.at<double>(1, 1);
	warp.at<double>(1, 2) = HPI.at<double>(1, 2);
	// cout <<warp << endl;
	// warpAffine(input, output, warp, output.size(), 2);//�����Բ�ֵӳ���ȥ
}
bool Tracker::ExtractTarget(vector<Mat>&pic1, vector<Mat>& Rotatematrix, vector<double>& coordinate)
{
	int col = pic1[0].cols;
	int row = pic1[0].rows;
	vector<Mat> pictemp;
	vector<Mat> picNotMove;
	for (size_t i = 0; i < cellsize; i++)
	{
		pictemp.push_back(pic1[i]);
	}
	//����������ת������Ϣ��ͼ�񱳾���������
	picNotMove.push_back(pictemp[0]);
	for (size_t i = 1; i < cellsize; i++)
	{
		Mat back1 = Mat::zeros(Size(col, row), CV_8U);//����һ��������Ϊ����ʹ��
		Mat tmp(Size(3, 3), CV_64FC1);
		Mat HPI(Size(3, 3), CV_64FC1);
		Mat warp(Size(3, 2), CV_64FC1);
		tmp = (Rsc*Rotatematrix[0]).inv();//��ʼʱ�����ǵ���ת��������һ����׼λ�ã����ֵ��Ҫ��һ��
		tmp = (Rsc*Rotatematrix[i])*tmp;
		HPI = K*tmp.inv(DECOMP_SVD)*K.inv(DECOMP_SVD);
		warp.at<double>(0, 0) = HPI.at<double>(0, 0);
		warp.at<double>(0, 1) = HPI.at<double>(0, 1);
		warp.at<double>(0, 2) = HPI.at<double>(0, 2);
		warp.at<double>(1, 0) = HPI.at<double>(1, 0);
		warp.at<double>(1, 1) = HPI.at<double>(1, 1);
		warp.at<double>(1, 2) = HPI.at<double>(1, 2);

		warpAffine(pictemp[i], back1, warp, back1.size(), 2);//�����Բ�ֵӳ�䷨
		picNotMove.push_back(back1);
	}
	//�ָ�֮��Ĵ�������
	vector<Mat> picout;
	FrameMinus(picNotMove, cellsize, picout);
    dirfilter(picout, cellsize);//���з����˲�
    Trace(picout, cellsize, coordinate);//�ҳ�Ŀ����ÿ��ͼ���ϵĽ���λ��
    lastxy.push_back(coordinate[cellsize * 2 - 2]);//�ѳ�ʼ�������һ֡��Ŀ������x����push��ȥ
    lastxy.push_back(coordinate[cellsize * 2 - 1]);//�ѳ�ʼ�������һ֡��Ŀ������y����push��ȥ
    rotatematrix = Rotatematrix[0];
    if (coordinate.size() == cellsize*2)
    {
        last_pic = pic1[cellsize - 1].clone();
        last2_pic = pic1[cellsize - 2].clone();
        last_rot = Rotatematrix[cellsize - 1].clone();
        last2_rot = Rotatematrix[cellsize - 2].clone();
        return true;
    }
    else return false;
}
bool Tracker::Tracking(Mat& thisimage, double& thisx, double& thisy)
{
	//�������˲���ز���
	KalmanFilter KF(6, 4, 0);
	KF.transitionMatrix = *(Mat_<float>(6, 6) << 1, 0, 1, 0, 0.5*1*1, 0,
		0, 1, 0, 1, 0, 0.5*1*1,
		0, 0, 1, 0, 1, 0,
		0, 0, 0, 1, 0, 1,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1);  //ת�ƾ���Fai   
	setIdentity(KF.measurementMatrix);                                             //��������H ,��Ϊ��λ��ֱ�Ӳ����ľ���λ��  
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //ϵͳ�����������Q    
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //���������������R    
	setIdentity(KF.errorCovPost, Scalar::all(1));
	Mat measurement = Mat::zeros(4, 1, CV_32F);


	vector<int> vec;//��¼���е�����Ҷ�ֵ��Ȼ��Ը������еĻҶ�ֵ��������������ߵ����ɵ㵽vecpic����ȡ������λ��
	vector<int> vecpic;//�����Ǽ�¼ĳ������ֵ���ڵ�ͼ������
	int templastx = lastxy[0];
	int templasty = lastxy[1];//��������ֵ��������������Լ�������֮֡����ٶ�
	for (int j = templasty - 10; j < templasty + 10; j++)
	{
		uchar* data = thisimage.ptr<uchar>(j);
		for (int k = templastx - 10; k < templastx + 10; k++)
		{
			if (data[k]>5 && j != templasty&&k != templastx)
			{
				vec.push_back(data[k]);
				vecpic.push_back(data[k]);
				vecpic.push_back(j);
				vecpic.push_back(k);
			}
			data[k] = 0;//������Ԫ�ػҶ�ֵȫ����0�������ٽ����ȸߵ���255��ʵ�ֶ�ֵ��
		}
	}
	int lightnum = 2;
	if (vec.size()>0)
	{
		quickSort(vec, 0, vec.size() - 1);//��������������Ԫ�ظ�������size-1����ס����Ϊ�������򣬴Ӻ���ǰ�������ҵ������ĸ�������
		int vecsize = vec.size();
		int lastvalue = vec[vecsize - 1];//�����ų��ظ�����ֵ���˴�ֻ����㸳һ����ֵ
		int num = 0;//��¼�Ѿ��ҵ������ص������������ֵ������ѭ��
		for (int n = vecsize - 1; n >= 0; n--)//��ѭ�����ã������������ɸ�ֵ��vecpic�ж�ȡ��Ӧ�����꣬���ҽ���Ӧ������Ϊ255
		{
			int value = vec[n];
			if (n != (vecsize - 1) && value == lastvalue)//�������ظ�ֵʱ�������Ѿ���¼��һ�飬�������
				continue;
			lastvalue = value;
			for (unsigned long m = 0; m<(vecpic.size() / 3); m++)
			{
				if (vecpic[3 * m] == value)
				{
					uchar* pdata = thisimage.ptr<uchar>(vecpic[3 * m + 1]);
					pdata[vecpic[3 * m + 2]] = vecpic[3 * m];
					num++;
				}
				if (num >= lightnum)
					break;
			}
			if (num >= lightnum)
				break;
		}
		//���ûҶ����ļ�Ȩ�㷨����Ŀ������������
		double x = 0, y = 0;
		double xsum = 0, ysum = 0, sum = 0;
		for (unsigned long m = 0; m < (vecpic.size() / 3); m++)
		{
			sum += vecpic[3 * m];
			xsum += vecpic[3 * m] * vecpic[3 * m + 2];
			ysum += vecpic[3 * m] * vecpic[3 * m + 1];
		}
		x = xsum / sum;
		y = ysum / sum;
		thisx = x;
		thisy = y;
	}
	else
	{
		return false;
	}
	//��ʼ״ֵ̬  
	KF.statePost = *(Mat_<float>(6, 1) << lastxy[0], lastxy[1], 1.2, 0, 0, 0);
	Mat prediction = KF.predict();
	double vx = abs(thisx - lastxy[0]);
	double vy = abs(thisy - lastxy[1]);
	//�������ֵ  
	measurement.at<float>(0) = thisx;
	measurement.at<float>(1) = thisy;
	measurement.at<float>(2) = vx;
	measurement.at<float>(3) = vy;
	//����  
	KF.correct(measurement);
	//������  

	thisx = KF.statePost.at<float>(0);
	thisy = KF.statePost.at<float>(1);
	lastxy[0] = thisx;
	lastxy[1] = thisy;//������һ��ͼ���Ŀ������
	vector<int>().swap(vec);
	vector<int>().swap(vecpic);
	return true;

}
void Tracker::setParameter(int sizex,int sizey, double fx, double fy, cv::Mat Rsc)
{
	weight = sizex;
	height = sizey;
	double temp[] = { fx, 0, sizex / 2.0, 0, fy, sizey / 2.0, 0, 0, 1 };
	Mat(3, 3, CV_64FC1, temp).copyTo(K);
    Rsc.copyTo(this->Rsc);
}

cv::Mat Tracker::diffMat(cv::Mat current_pic, cv::Mat current_rot) {
    cv::Mat current_trans;
    cv::Mat last2_trans;
    Rotateimage(current_pic, current_rot, current_trans);
    Rotateimage(last2_pic, last2_rot, last2_trans);
    cv::Mat img = current_trans - last2_trans;
    // cv::cvtColor(img, img, CV_BGR2GRAY);
    last_pic.copyTo(last2_pic);
    last_rot.copyTo(last2_rot);
    current_pic.copyTo(last_pic);
    current_rot.copyTo(last_rot);
    return img;
}
