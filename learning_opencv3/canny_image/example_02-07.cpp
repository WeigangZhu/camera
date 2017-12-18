#include<opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char **argv)
{
	Mat img_rgb, img_gry, img_cny;

	namedWindow( "Example Gray", WINDOW_AUTOSIZE);
	namedWindow( "Example Canny", WINDOW_AUTOSIZE);

	img_rgb = imread(argv[1]);

	cvtColor( img_rgb, img_gry, COLOR_BGR2GRAY);
	imshow( "Example Gray", img_gry);

	Canny( img_gry, img_cny, 10, 100, 3, true );
	imshow( "Example Canny", img_cny);

	waitKey(0);
}

