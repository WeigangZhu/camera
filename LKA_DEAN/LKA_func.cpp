#include"LKA.h"

string double_to_string(long double value, int decplaces)//将double距离数据转换成string;
{
	ostringstream out;
	int prec = 6; // 

	out.precision(prec);//覆盖默认精度
	out << value;
	string str = out.str(); //从流中取出字符串
	size_t n = str.find(4);
	if ((n != string::npos) //有小数点吗？
		&& (str.size()> n + decplaces)) //后面至少还有decplaces位吗？
	{
		str[n + decplaces] = '\0';//覆盖第一个多余的数
	}

	str.swap(string(str.c_str()));//删除nul之后的多余字符

	return str;
}

int compare(const void *a, const void *b)
{
	return (*(struct node*)a).distance - (*(struct node*)b).distance;
}


Mat imagepreprocess(Mat src)
{
	Mat dst, dst1, dst2, dst3,dst4,grad_x,abs_grad_x;
	cvtColor(src, dst, CV_BGR2GRAY);
	medianBlur(dst, dst1, 3);
	GaussianBlur(dst1, dst2, Size(3, 3), 0.5, 0.5);
	Canny(dst2, dst3, 50, 160);
	threshold(dst3, dst4, 155, 255, CV_THRESH_BINARY);//阈值处理转换成二值图像
	return(dst4);
}

Mat ROI_Operation(Mat src)
{
	Mat cutimage = src(Range(20, 180), Range::all());
	return (cutimage);
}

void linesSeparation(vector<Vec4i>lines, double kL[100], double kR[100], double bL[100], double bR[100], Vec4i lL[100], Vec4i lR[100],int &countkL, int &countkR)
{
	int countbL = 0, countbR = 0, m = 0, n = 0;
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		if (abs(l[0] - l[2])>0)
		{
			double k = (l[1] - l[3])*1.0 / (l[0] - l[2]);//计算斜率
			if (k < 0 && abs(k)>0.2&&abs(k)<2.0)
			{
				kL[countkL++] = k;
				bL[countbL++] = l[1] - k*l[0];
				lL[n++] = l;
				cout << "kL=" << kL[countkL-1] << '\n';
			}
			if (k>0.2&&k<2.0)
			{
				kR[countkR++] = k;
				bR[countbR++] = l[1] - k*l[0];
				lR[m++] = l;
				cout << "kR=" << kR[countkR-1] << '\n';
			}
		}
	}
}


void distancesort(struct node d[100], double k[], double b[], int count)
{
	for (int j = 0; j < count; j++)
	{
		d[j].distance = abs(k[j] * pt_0.x - pt_0.y + b[j])*1.0 / sqrt(1 + k[j] * k[j]);
		d[j].num = j;
	}
}

double LinesDistance(double k,double b)
{
	Point_<float> point_1, point_2, point_3,point_4;	//存放车道线上两点的世界坐标值；
	double distance;									//距离
	Vec4i lines;										//取图像平面车道线上两点；
	lines[0] = 200.0, lines[1] = k*lines[0] + b;
	lines[2] = 150.0, lines[3] = k*lines[2] + b;
	point_1.x = h*(f_y*cos(alpha) - (lines[1] + 20 - v_0)*sin(alpha))*1.0 / (f_y*sin(alpha) + (lines[1] + 20 - v_0)*cos(alpha));
	point_1.y = f_y / f_x*h*(u_0 - lines[0])*1.0 /(f_y*sin(alpha) + (lines[1] + 20 - v_0)*cos(alpha));
	point_2.x = h*(f_y*cos(alpha) - (lines[3] + 20 - v_0)*sin(alpha))*1.0 / (f_y*sin(alpha) + (lines[3] + 20 - v_0)*cos(alpha));
	point_2.y = f_y / f_x*h*(u_0 - lines[2])*1.0 / (f_y*sin(alpha) + (lines[3] + 20 - v_0)*cos(alpha));

	point_3.x = h*(f_y*cos(alpha) - (UV_1.y - v_0)*sin(alpha))*1.0 / (f_y*sin(alpha) + (UV_1.y - v_0)*cos(alpha));
	point_3.y = f_y / f_x*h*(u_0 - UV_1.x)*1.0 / (f_y*sin(alpha) + (UV_1.y- v_0)*cos(alpha));
	float K, B,D,beta;
	K = (point_2.y - point_1.y)*1.0 / (point_2.x - point_1.x);//计算世界坐标系下车道线直线斜率；
	if (K > 0)												  //计算偏离角；
	{
		beta = cvFastArctan(K, 1);
	}
	else
		beta = cvFastArctan(abs(K), 1);
	cout << "偏离角度：" << beta << '\n';
	B = point_2.y - K*point_2.x;
	D = abs(K*point_3.x - point_3.y + B)*1.0 / sqrt(1 + K*K);
	return D;
}

void predict_point(Point &p1, Point &p2, Point &p3, Point &p4, float k, float b, float shift, float y1, float y2)
{
	//shift:表示平移的量；k:预测直线斜率；b:预测直线截距；y1:求与y=y1的交点；y2:求y=y2的交点
	p1.x = (y2 - b)*1.0 / k - shift;
	p1.y = y2;

	p2.x = (y2 - b)*1.0 / k + shift;
	p2.y = y2;

	p3.x = (y1 - b)*1.0 / k - shift;
	p3.y = y1;

	p4.x = (y1 - b)*1.0 / k + shift;
	p4.y = y1;
}
