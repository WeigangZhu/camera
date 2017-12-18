#include<opencv2/opencv.hpp>
#include<iostream>

using namespace cv;

int main(int argc, char **argv)
{
	namedWindow( "Example2_11", WINDOW_AUTOSIZE);
	namedWindow( "Log_Polar"  , WINDOW_AUTOSIZE);

	// ( Note: could capture from a camera by giving a camera id as an int)
	//
	VideoCapture capture( argv[1] );

	double fps = capture.get( CAP_PROP_FPS );
	Size size( (int)capture.get(CAP_PROP_FRAME_WIDTH),
			   (int)capture.get(CAP_PROP_FRAME_HEIGHT)
			  );
	VideoWriter writer;
	writer.open( argv[2], CV_FOURCC('M', 'J', 'P', 'G'), fps, size );

	Mat logpolar_frame, bgr_frame;
	for(;;)
	{
		capture >> bgr_frame;
		if(bgr_frame.empty()) 
			break; // end if done

		imshow("Example2_11", bgr_frame);

		logPolar(
				bgr_frame,	// Input color frame
				logpolar_frame,	// Output log-polar frame
				Point2f(        // Centerpoint for log-polar transformation
					bgr_frame.cols/2, // x
					bgr_frame.rows/2  // y
					),
				40,		// Magnitude ( scale parameter)
				WARP_FILL_OUTLIERS //Fill outlines with 'zero'
				);
		imshow("Log_Polar", logpolar_frame);
		writer << logpolar_frame;

		char c = waitKey(10); 
		if( c == 27 )
			break; // allow the user to break out
	}

	capture.release();
}

