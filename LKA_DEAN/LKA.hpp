#ifndef LKA_H_
#define LKA_H_

#include <iostream>
#include <fstream>
#include <opencv2\opencv.hpp>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <vector>

struct node
{
	double distance;
	int num;
};

using namespace cv;
using namespace std;

/******以下为摄像机内部和外部参数**************************/
const double pi =3.1415926;
const int u_0 = 158, v_0 = 116;//图像平面中心点
Point UV_0(u_0, v_0);//图像平面中心点；
Point UV_1(u_0, 2*v_0-60);//图像平面底部中点坐标；
Point UV_2(u_0, 0);//图像顶部中心点；
const double f = 3.60;//摄像机焦距（单位/mm）；
const double h = 1420.4;//摄像机安装高度(单位/mm)；
const double alpha =9*pi/180;//摄像机安装俯仰角度(弧度值);
const double f_x = 160.92, f_y = 215.98;//摄像机标定单位像素焦距f/dx,f/dy；1648,1649
Mat camera_matrix1 = (Mat_<double>(3, 3) << 160.92, 0.0, 158,
											0.0, 215.98, 116,
											0.0, 0.0, 1.0);
Mat dist_coeffs1 = (Mat_<double>(5, 1) << -0.15858, -0.00058 , -0.00139 , 0.00156 ,0.00000);
/*---------------------------------------------------------*/
const int stateNum = 4;	//卡尔曼滤波的状态和测量向量的维度
const int measureNum = 2;
Point pt_0(160, 140);	//区域选择点；
Point Vanish(160, 55);	//消失点；
int Leftborder = 45, Rightborder = 285;//消失点左右边界；
int framecount = 0;		//视频帧计数
int predict_shift =15;
int predict_y1 = 100, predict_y2 = 300;
int countL = 0,countR=0;
int preL = 0,preR=0;
int resetframe = 10;
Size imageSize;
const int height = 240;
const int width = 320;
const int frameThesh = 60000;//定义帧后开始检测


string double_to_string(long double value, int decplaces);//将double距离数据转换成string;

int compare(const void *a, const void *b);

Mat imagepreprocess(Mat src);

Mat ROI_Operation(Mat src);

void linesSeparation(vector<Vec4i>lines, double kL[100], double kR[100], double bL[100], double bR[100], Vec4i lL[100], Vec4i lR[100],int &countkL, int &countkR);

void distancesort(struct node d[100], double k[], double b[], int count);

double LinesDistance(double k,double b);

#endif
