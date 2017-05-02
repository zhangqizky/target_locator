#include"Target.h"

size_t Tracker::cellsize = 5;
void Tracker::FrameMinus(vector<Mat> pic, int num, vector<Mat>& picout)
{
	int col = pic[0].cols;//图像的列，也就是x坐标，宽度
	int row = pic[0].rows;//图像的行，也就是y坐标，高度
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
int Tracker::dirfilter(vector<Mat> &pic1, int num)//方向滤波
{
	Mat *pic;
	pic = new Mat[num];
	for (int i = 0; i<num; i++)
	{
		pic[i] = pic1[i];              //将vector类型转换为指针的办法
	}
	for (int i = 0; i<num; i++)//该循环作用：边缘像素置零
	{
		for (int j = 0; j<pic[0].rows; j++)
		{
			uchar* data = pic[0].ptr<uchar>(j);
			uchar* data2 = pic[i].ptr<uchar>(j);

			for (int k = 0; k<pic[0].cols; k++)
			{
				if (i == 0)
				{
					if (j<5 || j>(pic[0].rows - 6) || k<5 || k>(pic[0].cols - 6))//边缘像素点全部置零
						data2[k] = 0;
				}
				else if (j<5 || j>(pic[0].rows - 6) || k<5 || k>(pic[0].cols - 6))//边缘像素点全部置零
					data2[k] = 0;
			}
		}
	}

	Mat *guestpic = new Mat[num]; Mat *hostpic = new Mat[num];
	for (int i = 0; i<num; i++)//该循环将pic序列赋值到hostpic，再将pic序列置零
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


	for (int m = -3; m <= 3; m = m + 3)//该循环执行双方向滤波，m、n用于控制8个不同方向
	{
		for (int n = -3; n <= 3; n = n + 3)
		{
			if (m == 0 && n == 0)//排除中心点
				continue;

			for (int i = 0; i<num; i++)
			{
				guestpic[i] = hostpic[i].clone();
			}
			Mat temppic = guestpic[0].clone();//记录上一张处理后的图

			for (int i = 1; i<num; i++)//该循环为正向方向滤波
			{
				maxfilter(temppic);//邻域最大值滤波
				Mat kernal = (Mat_<float>(3, 3) << 0.2, 0.2, 0.2, 0.2, 0.6, 0.2, 0.2, 0.2, 0.2);//注意卷积核的该形式
				filter2D(temppic, temppic, -1, kernal);
				for (int j = 3; j<guestpic[0].rows - 3; j++)
				{
					uchar* data = temppic.ptr<uchar>(j + m);
					uchar* mixdata = guestpic[i].ptr<uchar>(j);

					for (int k = 3; k<guestpic[0].cols - 3; k++)
					{
						int x1 = data[k + n] / 50;
						int x = mixdata[k] / 50;//双相关思想，当前帧（j,k）点与前一帧(j+2,k-2)点的亮度相互影响，当且仅当两点亮度都较高时才会对
						//当前点发生较大加强，否则会大幅削弱
						float y = x*0.5;
						float y1 = x1*0.5;
						int value = y*data[k + n] + y1*mixdata[k];
						if (value>255) value = 255;//防止灰度值超过255
						mixdata[k] = value;

					}
				}
				temppic = guestpic[i].clone();
			}

			for (int i = num - 2; i >= 0; i--)//该循环为逆向方向滤波
			{

				maxfilter(temppic);//邻域最大值滤波
				Mat kernal = (Mat_<float>(3, 3) << 0.2, 0.2, 0.2, 0.2, 0.6, 0.2, 0.2, 0.2, 0.2);//注意卷积核的该形式
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

			for (int i = 0; i<num; i++)//该小循环是把处理后的guestpic序列重新赋值给pic序列，从而函数结束后pic序列存储的就是滤波后的序列
				//pic[i]=guestpic[i].clone();
				mixsolo(pic[i], guestpic[i]);//最后还是把处理的结果重新赋给了pic，pic传的是引用，从而pic就是被滤波后的序列

		}
	}

	delete[]hostpic; delete[]guestpic;

	return 1;
}
int Tracker::maxfilter(Mat pic)//邻域最大值滤波
{
	Mat tpic = pic.clone();
	for (int i = 2; i<pic.rows - 2; i++)
	{
		uchar* pdata = pic.ptr<uchar>(i);
		for (int j = 2; j<pic.cols - 2; j++)
		{
			uchar* data = tpic.ptr<uchar>(i);
			float x = data[j];
			for (int m = 0; m<5; m++)//该循环寻找5*5邻域最大值
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
int Tracker::mixsolo(Mat dest, Mat source)//合成两幅图像
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
void Tracker::quickSort(vector<int> &vec, int l, int r)  //快速排序法
{
	if (l< r)
	{
		int i = l, j = r, x = vec[l];
		while (i < j)
		{
			while (i < j && vec[j] >= x) // 从右向左找第一个小于x的数  
				j--;
			if (i < j)
				vec[i++] = vec[j];
			while (i < j && vec[i]< x) // 从左向右找第一个大于等于x的数  
				i++;
			if (i < j)
				vec[j--] = vec[i];
		}
		vec[i] = x;
		quickSort(vec, l, i - 1); // 递归调用  
		quickSort(vec, i + 1, r);
	}
}
bool Tracker::Trace(vector<Mat>& pic, int num, vector<double>& coordinate)
{
	int size = (pic[0].cols)*(pic[0].rows);
	for (int i = 0; i < num; i++)
	{
		vector<int> vec;//记录所有的亮点灰度值，然后对该向量中的灰度值排序，再找亮度最高的若干点到vecpic中提取出坐标位置
		vector<int> vecpic;//作用是记录某个像素值所在的图像坐标
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
				data[k] = 0;//将所有元素灰度值全部置0，后面再将亮度高的置255，实现二值化
			}
		}

		int lightnum = 3;
		if (vec.size()>0)
		{
			quickSort(vec, 0, vec.size() - 1);//快速排序，向量的元素个数等于size-1，记住！且为升序排序，从后往前处理即可找到最大的三个点像素
			int vecsize = vec.size();
			int lastvalue = vec[vecsize - 1];//用于排除重复像素值，此处只是随便赋一个初值
			int num = 0;//记录已经找到的像素点个数，到达阈值就跳出循环
			for (int n = vecsize - 1; n >= 0; n--)//该循环作用：根据最大的若干个值从vecpic中读取相应的坐标，并且将相应像素置为255
			{
				int value = vec[n];
				if (n != (vecsize - 1) && value == lastvalue)//当出现重复值时，由于已经记录过一遍，因此跳过
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
			//利用灰度质心加权算法计算目标点的像素坐标
			double x = 0, y = 0;
			double xsum = 0, ysum = 0, sum = 0;
			for (unsigned long m = 0; m < (vecpic.size() / 3); m++)
			{
				sum += vecpic[3 * m];
				xsum += vecpic[3 * m] * vecpic[3 * m + 2];
				ysum += vecpic[3 * m] * vecpic[3 * m + 1];
			}
			x = xsum / sum;//对应的是k
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
	//Mat back1 = Mat::zeros(Size(size, size), CV_8U);//生成一个背景作为后面使用
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

	warpAffine(input, output, warp, output.size(), 2);//三线性插值映射过去
}
void Tracker::Rotateimageinv(Mat rotate)
{
	//Mat back1 = Mat::zeros(Size(size, size), CV_8U);//生成一个背景作为后面使用
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
	// warpAffine(input, output, warp, output.size(), 2);//三线性插值映射过去
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
	//首先利用旋转矩阵信息对图像背景进行修正
	picNotMove.push_back(pictemp[0]);
	for (size_t i = 1; i < cellsize; i++)
	{
		Mat back1 = Mat::zeros(Size(col, row), CV_8U);//生成一个背景作为后面使用
		Mat tmp(Size(3, 3), CV_64FC1);
		Mat HPI(Size(3, 3), CV_64FC1);
		Mat warp(Size(3, 2), CV_64FC1);
		tmp = (Rsc*Rotatematrix[0]).inv();//初始时刻卫星的旋转矩阵，这是一个基准位置，这个值需要存一下
		tmp = (Rsc*Rotatematrix[i])*tmp;
		HPI = K*tmp.inv(DECOMP_SVD)*K.inv(DECOMP_SVD);
		warp.at<double>(0, 0) = HPI.at<double>(0, 0);
		warp.at<double>(0, 1) = HPI.at<double>(0, 1);
		warp.at<double>(0, 2) = HPI.at<double>(0, 2);
		warp.at<double>(1, 0) = HPI.at<double>(1, 0);
		warp.at<double>(1, 1) = HPI.at<double>(1, 1);
		warp.at<double>(1, 2) = HPI.at<double>(1, 2);

		warpAffine(pictemp[i], back1, warp, back1.size(), 2);//三线性插值映射法
		picNotMove.push_back(back1);
	}
	//恢复之后的处理问题
	vector<Mat> picout;
	FrameMinus(picNotMove, cellsize, picout);
    dirfilter(picout, cellsize);//进行方向滤波
    Trace(picout, cellsize, coordinate);//找出目标在每幅图像上的近似位置
    lastxy.push_back(coordinate[cellsize * 2 - 2]);//把初始化中最后一帧的目标坐标x坐标push进去
    lastxy.push_back(coordinate[cellsize * 2 - 1]);//把初始化中最后一帧的目标坐标y坐标push进去
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
	//卡尔曼滤波相关参数
	KalmanFilter KF(6, 4, 0);
	KF.transitionMatrix = *(Mat_<float>(6, 6) << 1, 0, 1, 0, 0.5*1*1, 0,
		0, 1, 0, 1, 0, 0.5*1*1,
		0, 0, 1, 0, 1, 0,
		0, 0, 0, 1, 0, 1,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1);  //转移矩阵Fai   
	setIdentity(KF.measurementMatrix);                                             //测量矩阵H ,设为单位阵，直接测量的就是位置  
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q    
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R    
	setIdentity(KF.errorCovPost, Scalar::all(1));
	Mat measurement = Mat::zeros(4, 1, CV_32F);


	vector<int> vec;//记录所有的亮点灰度值，然后对该向量中的灰度值排序，再找亮度最高的若干点到vecpic中提取出坐标位置
	vector<int> vecpic;//作用是记录某个像素值所在的图像坐标
	int templastx = lastxy[0];
	int templasty = lastxy[1];//把这两个值保存下来后面可以计算这两帧之间的速度
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
			data[k] = 0;//将所有元素灰度值全部置0，后面再将亮度高的置255，实现二值化
		}
	}
	int lightnum = 2;
	if (vec.size()>0)
	{
		quickSort(vec, 0, vec.size() - 1);//快速排序，向量的元素个数等于size-1，记住！且为升序排序，从后往前处理即可找到最大的四个点像素
		int vecsize = vec.size();
		int lastvalue = vec[vecsize - 1];//用于排除重复像素值，此处只是随便赋一个初值
		int num = 0;//记录已经找到的像素点个数，到达阈值就跳出循环
		for (int n = vecsize - 1; n >= 0; n--)//该循环作用：根据最大的若干个值从vecpic中读取相应的坐标，并且将相应像素置为255
		{
			int value = vec[n];
			if (n != (vecsize - 1) && value == lastvalue)//当出现重复值时，由于已经记录过一遍，因此跳过
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
		//利用灰度质心加权算法计算目标点的像素坐标
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
	//初始状态值  
	KF.statePost = *(Mat_<float>(6, 1) << lastxy[0], lastxy[1], 1.2, 0, 0, 0);
	Mat prediction = KF.predict();
	double vx = abs(thisx - lastxy[0]);
	double vy = abs(thisy - lastxy[1]);
	//计算测量值  
	measurement.at<float>(0) = thisx;
	measurement.at<float>(1) = thisy;
	measurement.at<float>(2) = vx;
	measurement.at<float>(3) = vy;
	//更新  
	KF.correct(measurement);
	//输出结果  

	thisx = KF.statePost.at<float>(0);
	thisy = KF.statePost.at<float>(1);
	lastxy[0] = thisx;
	lastxy[1] = thisy;//更新上一幅图像的目标坐标
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
