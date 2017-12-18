#include<opencv2/opencv.hpp>

void example2_5(const cv::Mat &image)
{
	// Creat some windows to show the input 
	// and output images in.
	//
	cv::namedWindow( "Example2_5-in", cv::WINDOW_AUTOSIZE);
	cv::namedWindow( "Example2_5-out", cv::WINDOW_AUTOSIZE);

	// Creat a window to show our input image
	//
	cv::imshow( "Example2_5-in", image);

	// Creat an image to hold the smoothed output
	//
	cv::Mat out;

	// Do the smoothing 
	// ( Note: Could use GaussianBlur(), blur(), medianBlur() or bilateralFilter(). )
	//
	cv::GaussianBlur( image, out, cv::Size(5, 5), 3, 3);
	cv::GaussianBlur(	out, out, cv::Size(5, 5), 3, 3);

	// Show the smoothed image in the output window
	//
	cv::imshow( "Example2_5-out", out);

	// Wait for the user to hit a key, windows will self destruct 
	//
	cv::waitKey(0);

}

int main(int argc, char **argv)
{
	cv::Mat input_image;
	if( argc != 2)
	{
		printf("parameters error\n");
	}
	input_image = cv::imread(std::string(argv[1]));
	example2_5(input_image);

	return 0;
}

