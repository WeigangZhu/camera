#ifndef __DET_H__
#define __DET_H__
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <stdio.h>



using namespace cv;
using namespace std;

struct node
{
	double distance;
	int num;
}; 

const double f = 1.89;//
const double h = 380;//the height of fix position
const double alpha =5;//the fix angle
const double f_x = 944, f_y = 942;//
const double pi =3.1415926;
const int u_0 = 350, v_0 = 305;// center of the image 
int compare(const void *a, const void *b);

Mat imagepreprocess(Mat src);

Mat ROI_Operation(Mat src);

void linesSeparation(vector<Vec4i>lines, double kL[100], double kR[100], double bL[100], double bR[100], Vec4i lL[100], Vec4i lR[100],int &countkL, int &countkR);

void distancesort(struct node d[100], double k[], double b[], int count);

double LinesDistance(double k,double b);

void predict_point(Point &p1, Point &p2, Point &p3, Point &p4, float k, float b, float shift, float y1, float y2);





#endif
