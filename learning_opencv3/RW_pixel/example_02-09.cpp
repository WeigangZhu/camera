#include<opencv2/opencv.hpp>

int main(int argc, char **argv)
{
	int x = 16, y = 32;
	cv::Mat img_rgb = cv::imread( argv[1] );
	cv::Vec3b intensity = img_rgb.at< cv::Vec3b >(x, y);

	// ( note: we could write img_rgb.at< cv::Vec3b >(x,y)[0]
	//
	uchar blue  = intensity[0];
	uchar green = intensity[1];
	uchar red	= intensity[2];

	std::cout << "At (x, y) = (" << x << ", "<< y 
			  <<"): (blue, green, red) = (" 
			  << (unsigned int) blue	<<	 ", " 
			  << (unsigned int) green	<<	 ", "
			  << (unsigned int) red		<<	 ")" 
			  << std::endl;

	cv::Mat img_gry;
   	cv::cvtColor(img_rgb, img_gry, cv::COLOR_BGR2GRAY);
	std::cout << "gray pixel there is: " 
			  << (unsigned int ) img_rgb.at<uchar>(y, x) 
			  << std::endl;

	cv::Mat img_cny;
   	cv::Canny(img_gry, img_cny, 10, 100, 3, true);
	x /= 4;
	y /= 4;

	std::cout << "Canny pixel there is: "
			  << (unsigned int)img_cny.at<uchar>(y, x)
			  << std::endl;
	img_cny.at<uchar>(x, y) = 128;
	
	return 0;

}
