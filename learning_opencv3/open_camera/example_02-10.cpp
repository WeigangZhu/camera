#include<opencv2/opencv.hpp>
#include<iostream>

int main(int argc, char **argv)
{
	cv::namedWindow( "Example2_10", cv::WINDOW_AUTOSIZE);

	cv::VideoCapture cap;
	if( argc == 1)
		cap.open(0); // open the first camera
	else
		cap.open(argv[1]);
	if( !cap.isOpened() )
	{
		// chech if we succeeded
		std::cerr << "couldn't open capture. "
				  << std::endl;

		return -1;
	}

	// The rest of program proceeds as in Example 2-3
	cv::namedWindow("Example3", cv::WINDOW_AUTOSIZE);
    cv::Mat frame;
    for(;;)
    {
        cap >> frame;
        cv::imshow("Example3", frame);
        if(cv::waitKey(33) >= 0)
            break;
    }
 
    return 0;


}
