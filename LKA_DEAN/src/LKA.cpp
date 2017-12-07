#include"LKA.hpp"

using namespace cv;
using namespace std;

int countL = 0,countR=0;
int preL = 0,preR=0;
int framecount = 0;		//视频帧计数

int main()
{
	Mat src, frameimage,cutimage, preprocessimage;
	VideoCapture cap(0);
	if (!cap.isOpened())
	{
		cout << "打开视频失败" << '\n';
		return -1;
	}
	
	//--------------------左边车道的卡尔曼滤波器----------------------
	KalmanFilter KFL(stateNum, measureNum, 0);
	Mat stateL(stateNum, 1, CV_32F);
	Mat processNoiseL(stateNum, 1, CV_32F);
	Mat measurementL = Mat::zeros(measureNum, 1, CV_32F);
	//--------------------右边车道的卡尔曼滤波器----------------------
	KalmanFilter KFR(stateNum, measureNum, 0);
	Mat stateR(stateNum, 1, CV_32F);
	Mat processNoiseR(stateNum, 1, CV_32F);
	Mat measurementR = Mat::zeros(measureNum, 1, CV_32F);
	bool stop = false;
	Mat frame;
	Mat left_image;
	while (cap.isOpened())
	{
		framecount++;
		if (framecount >= frameThesh)
		{
			cap >> src;
			countL++;
			countR++;
			imshow("src", src);
			resize(src, frame, Size(640, 240));
			left_image = src(cv::Rect(0, 0, frame.cols / 2, frame.rows));
		//	undistort(left_image, frameimage, camera_matrix1, dist_coeffs1);
			cutimage = ROI_Operation(frameimage);			//裁剪图像；
			preprocessimage = imagepreprocess(cutimage);	//图像预处理得到二值图像；
			imshow("Canny_image", preprocessimage);
			/*霍夫变换*/
			vector<Vec4i>lines;
			Vec4i lL[100], lR[100];//存放每个车道线上的两个坐标点；
			HoughLinesP(preprocessimage, lines, 1, CV_PI / 90, 30, 50, 50);
		/*	for (int i = 0; i < lines.size(); i++)
			{
				Vec4i l = lines[i];
				line(cutimage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 0), 1, CV_AA);
			}
		*/
			double kL[100], kR[100], bL[100], bR[100];//存放有效左右车道线斜率和截距;
			int countkL = 0, countkR = 0;
			linesSeparation(lines, kL, kR, bL, bR, lL, lR, countkL, countkR);//分离左右车道线；

			struct node dL[100], dR[100];
			distancesort(dL, kL, bL, countkL);				//计算左车道线的斜率截距；
			distancesort(dR, kR, bR, countkR);				//计算右车道线的斜率截距；
			cout << "countkL=" << countkL << '\n';
			cout << "countkR=" << countkR << '\n';
			qsort(dL, countkL, sizeof(struct node), compare);
			qsort(dR, countkR, sizeof(struct node), compare);
			//-------------------左，右边车道卡尔曼滤波参数初始化----------------
			randn(stateL, Scalar::all(0), Scalar::all(0.1));
			KFL.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 10, 0, 0, 1, 0, 10, 0, 0, 1, 0, 0, 0, 0, 1);  //状态转移矩阵A
			setIdentity(KFL.measurementMatrix);                                                             //观测矩阵H5
			setIdentity(KFL.processNoiseCov, Scalar::all(1e-5));                                            //过程噪声协方差矩阵
			setIdentity(KFL.measurementNoiseCov, Scalar::all(1e-2));                                        //测量噪声协方差矩阵
			setIdentity(KFL.errorCovPost, Scalar::all(1));                                                  //后验估计误差协方差矩阵

			randn(stateR, Scalar::all(0), Scalar::all(0.1));
			KFR.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 10, 0, 0, 1, 0, 10, 0, 0, 1, 0, 0, 0, 0, 1);  //状态转移矩阵A
			setIdentity(KFR.measurementMatrix);
			setIdentity(KFR.processNoiseCov, Scalar::all(1e-5));
			setIdentity(KFR.measurementNoiseCov, Scalar::all(1e-2));
			setIdentity(KFR.errorCovPost, Scalar::all(1));
			//---------------------求单车道的左右侧车道线初值作为状态初始值-----------------------
			int initial_L = 0, initial_R = 0;
			int numL = 0, numR = 0;
			float initial_kL, initial_bL, initial_kR, initial_bR;
			if (framecount == frameThesh)						//初始第一帧判断
			{
				while ((numL < countkL) && (((-bL[dL[numL].num] / kL[dL[numL].num]) < Leftborder) || ((-bL[dL[numL].num] / kL[dL[numL].num])>Rightborder)))
				{
					numL++;
				}
				initial_L = dL[numL].num;
				while ((numR < countkR) && (((-bR[dR[numR].num] / kR[dR[numR].num]) < Leftborder) || ((-bR[dR[numR].num] / kR[dR[numR].num])>Rightborder)))
				{
					numR++;
				}
				initial_R = dR[numR].num;

				initial_kL = kL[initial_L];
				initial_bL = bL[initial_L];
				initial_kR = kR[initial_R];
				initial_bR = bR[initial_R];
			}

			KFL.statePost.at<float>(0) = initial_kL;                                   //将初值赋给卡尔曼滤波器 状态向量为x=(k,b,△k,△b)，这里KFL.statePost.at<float>(0)对应的是k
			KFL.statePost.at<float>(1) = initial_bL;                                   //KFL.statePost.at<float>(1)对应的是b

			KFR.statePost.at<float>(0) = initial_kR;
			KFR.statePost.at<float>(1) = initial_bR;
			//假设没有变道进行车道线跟踪
			//--------------------------进行左车道线追踪-----------------------
		asd1:Mat predictionL = KFL.predict();                                         //得到预测参数 k和b
			float predict_kL = predictionL.at<float>(0);
			float predict_bL = predictionL.at<float>(1);
			Point pt1, pt11, pt2, pt22;
			predict_point(pt1, pt11, pt2, pt22, predict_kL, predict_bL, predict_shift, predict_y1, predict_y2);  // 将预测得到的直线向左向右平移predict_shift，并求得与predict_y1, predict_y2的交点，一共四个交点
			int s1 = 0;
			if (countL < 5)                                                                                    //左侧不能用预测范围对车道线进行限制的帧数
			{
				while ((s1 < countkL) && ((((-bL[dL[s1].num] * 1.0 / kL[dL[s1].num]) < Leftborder) || ((-bL[dL[s1].num] * 1.0 / kL[dL[s1].num]) > Rightborder))))  //未使用预测范围，限制条件与之间的相同
				{
					s1++;
				}
			}
			else
			{
				while ((s1 < countkL) && (((-bL[dL[s1].num] * 1.0 / kL[dL[s1].num]) < Leftborder || ((-bL[dL[s1].num] * 1.0 / kL[dL[s1].num]) > Rightborder) || (((predict_y1-bL[dL[s1].num]) * 1.0 / kL[dL[s1].num]) < pt1.x) || (((predict_y1-bL[dL[s1].num]) * 1.0 / kL[dL[s1].num]) > pt11.x) || (((predict_y2 - bL[dL[s1].num])*1.0 / kL[dL[s1].num]) < pt2.x) || ((predict_y2 - bL[dL[s1].num])*1.0 / (kL[dL[s1].num]) > pt22.x))))
				{                                                                                               //使用预测范围，则车道线需要在预测范围内
					s1++;
				}
			}
			int m1 = dL[s1].num;

			if (s1 < countkL)
			{
				preL = 0;                             //为防止不连续未检测出车道线的帧数相加导致跟踪重启，设置只要找到车道线，则preL置0
				measurementL.at<float>(0) = kL[m1];   //如果检测到车道线，则车道线参数为系统测量值
				measurementL.at<float>(1) = bL[m1];
			}
			if (preL == resetframe)                   //如果连续未检测到车道线参数等于设置好的重启帧数，则跟踪进行重启，需要再次找到系统初始值
			{
				int s1 = 0;
				while ((s1 < countkL) && ((-bL[dL[s1].num] / kL[dL[s1].num]) < Leftborder || (-bL[dL[s1].num] / kL[dL[s1].num]) > Rightborder))
				{
					s1++;
				}
				m1 = dL[s1].num;
				if (s1 < countkL)                        //重新检测到车道线，并赋给卡尔曼滤波器
				{
					KFL.statePost.at<float>(0) = kL[m1];
					KFL.statePost.at<float>(1) = bL[m1];
				}
				if (s1 == countkL)                      //如果重启仍未找到，继续将预测值作为系统测量值
				{
					preL = 0;
				}
				countL = 0;                         //重启后帧计数重新开始
				goto asd1;
			}
			if (s1 == countkL)                         //如果没有检测到车道线，则预测值作为系统测量值
			{
				preL++;                             //未检测到车道线帧数加1
				measurementL.at<float>(0) = predict_kL;
				measurementL.at<float>(1) = predict_bL;
			}

			Mat correctedL = KFL.correct(measurementL);              //得到系统预测值和测量值后，得到修正值，即该时刻的最优估计值
			float corrected_kL = correctedL.at<float>(0);
			float corrected_bL = correctedL.at<float>(1);
			cout << "右车道线斜率：" << corrected_kL << '\n';
			line(cutimage, Point((-corrected_bL)*1.0 / corrected_kL, 0), Point((predict_y2 - corrected_bL)*1.0 / corrected_kL, predict_y2), Scalar(0, 0, 255), 2, CV_AA);
			randn(processNoiseL, Scalar(0), Scalar::all(sqrt(KFL.processNoiseCov.at<float>(0, 0))));
			stateL = KFL.transitionMatrix*stateL + processNoiseL;    //更新状态方程
			//--------------------------进行右车道线追踪-----------------------
		asd2:Mat predictionR = KFR.predict();                                         //得到预测参数 k和b
			float predict_kR = predictionR.at<float>(0);
			float predict_bR = predictionR.at<float>(1);
			Point pt3, pt33, pt4, pt44;
			predict_point(pt3, pt33, pt4, pt44, predict_kR, predict_bR, predict_shift, predict_y1, predict_y2);  // 将预测得到的直线向左向右平移predict_shift，并求得与predict_y1, predict_y2的交点，一共四个交点
			int s2 = 0;
			if (countR < 5)                                                                                    //右侧不能用预测范围对车道线进行限制的帧数
			{
				while ((s2 < countkR) && (((-bR[dR[s2].num]*1.0 / kR[dR[s2].num]) < Leftborder || (-bR[dR[s2].num]*1.0 / kR[dR[s2].num]) > Rightborder)))  //未使用预测范围，限制条件与之间的相同
				{
					s2++;
				}
			}
			else
			{
				while ((s2 < countkR) && ((((-bR[dR[s2].num] * 1.0 / kR[dR[s2].num]) < Leftborder) || ((-bR[dR[s2].num] * 1.0 / kR[dR[s2].num]) > Rightborder) || ((predict_y1-bR[dR[s2].num] * 1.0 / kR[dR[s2].num]) < pt4.x) || (((predict_y1-bR[dR[s2].num] * 1.0) / kR[dR[s2].num]) > pt44.x) || (((predict_y2 - bR[dR[s2].num])*1.0 / kR[dR[s2].num]) < pt3.x) || (((predict_y2 - bR[dR[s2].num])*1.0 / kR[dR[s2].num]) > pt33.x))))
				{                                                                                               //使用预测范围，则车道线需要在预测范围内
					s2++;
				}
			}
			int m2 = dR[s2].num;

			if (s2 < countkR)
			{
				preR = 0;								 //为防止不连续未检测出车道线的帧数相加导致跟踪重启，设置只要找到车道线，则preL置0
				measurementR.at<float>(0) = kR[m2];		 //如果检测到车道线，则车道线参数为系统测量值
				measurementR.at<float>(1) = bR[m2];
			}
			if (preR == 10)							     //如果连续未检测到车道线参数等于设置好的重启帧数，则跟踪进行重启，需要再次找到系统初始值
			{
				int s2 = 0;
				while ((s2 < countkR) && (((-bR[dR[s2].num] / kR[dR[s2].num]) < Leftborder) || ((-bR[dR[s2].num] / kR[dR[s2].num]) > Rightborder)))  //未使用预测范围，限制条件与之间的相同
				{
					s2++;
				}
				m2 = dR[s2].num;
				if (s2 < countkR)                        //重新检测到车道线，并赋给卡尔曼滤波器
				{
					KFR.statePost.at<float>(0) = kR[m2];
					KFR.statePost.at<float>(1) = bR[m2];
				}
				if (s2 == countkR)                      //如果重启仍未找到，继续将预测值作为系统测量值
				{
					preR = 0;
				}
				countR = 0;                         //重启后帧计数重新开始
				goto asd2;
			}
			if (s2 == countkR)                         //如果没有检测到车道线，则预测值作为系统测量值
			{
				preR++;							       //未检测到车道线帧数加1
				measurementR.at<float>(0) = predict_kR;
				measurementR.at<float>(1) = predict_bR;
			}
			Mat correctedR = KFR.correct(measurementR);              //得到系统预测值和测量值后，得到修正值，即该时刻的最优估计值
			float corrected_kR = correctedR.at<float>(0);
			float corrected_bR = correctedR.at<float>(1);
			cout << "右车道线斜率：" << corrected_kR << '\n';
			line(cutimage, Point((-corrected_bR)*1.0 / corrected_kR, 0), Point((predict_y2 - corrected_bR)*1.0 / corrected_kR, predict_y2), Scalar(0, 0, 255), 2, CV_AA);
			randn(processNoiseR, Scalar(0), Scalar::all(sqrt(KFR.processNoiseCov.at<float>(0, 0))));
			stateR = KFR.transitionMatrix*stateR + processNoiseR;    //更新状态方程
			//调试所用
		/*	Vanish.y = (bL[dL[0].num] * kR[dR[0].num] - bR[dR[0].num] * kL[dL[0].num]) / (kR[dR[0].num] - kL[dL[0].num]);
			Vanish.x = (Vanish.y - bL[dL[0].num]) / kL[dL[0].num];
			cout << "Vanish" << "(" << Vanish.x << "," << Vanish.y << ")" << '\n';
			line(cutimage, Point(lL[dL[0].num][0], lL[dL[0].num][1]), Point(Vanish.x, Vanish.y), Scalar(0, 0, 255), 2, CV_AA);
			line(cutimage, Point(lR[dR[0].num][0], lR[dR[0].num][1]), Point(Vanish.x, Vanish.y), Scalar(0, 0, 255), 2, CV_AA);
		*/
			int DL = 0, DR = 0;//离左边车道线的距离

			DL = LinesDistance(corrected_kL, corrected_bL)+60;
			DR = LinesDistance(corrected_kR, corrected_bR)-60;
			cout << "距离左边车道线的距离=" << DL << "mm" << '\n';
			cout << "距离右边车道线的距离=" << DR<<"mm" << '\n';
			cout << "车道线总宽度=" << DL+DR<<"mm" << '\n';
			string textL = "Left Lane", textR = "Right Lane";
						
			putText(cutimage, textL, Point(cutimage.rows / 2-50, cutimage.cols / 2-10), CV_FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 255));//参数:承载的图片，插入的文字，文字的位置（文本框左下角），字体，大小，颜色
			putText(cutimage, textR, Point(cutimage.rows / 2 +140, cutimage.cols / 2-10 ), CV_FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 255));
			/* imshow("cutimage", cutimage);*/
			waitKey(10);
		}
	}
	cap.release();
	return(0);
}

