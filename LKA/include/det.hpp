#ifndef __DET_H__
#define __DET_H__

int compare(const void *a, const void *b);

Mat imagepreprocess(Mat src);

Mat ROI_Operation(Mat src);

void linesSeparation(vector<Vec4i>lines, double kL[100], double kR[100], double bL[100], double bR[100], Vec4i lL[100], Vec4i lR[100],int &countkL, int &countkR);

void distancesort(struct node d[100], double k[], double b[], int count);

double LinesDistance(double k,double b);

void predict_point(Point &p1, Point &p2, Point &p3, Point &p4, float k, float b, float shift, float y1, float y2);





#endif
