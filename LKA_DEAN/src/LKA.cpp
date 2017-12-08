#include"LKA.hpp"

int countL = 0,countR=0;
int preL = 0,preR=0;

using namespace cv;
using namespace std;

int main()
{
	Mat src, frameimage,cutimage, preprocessimage;
	VideoCapture cap(0);
	if (!cap.isOpened())
	{
		cout << "Can't Open Cap" << '\n';
		return -1;
	}
	
	//------------------------------------------
	KalmanFilter KFL(stateNum, measureNum, 0);
	Mat stateL(stateNum,    1, CV_32F);
	Mat processNoiseL(stateNum, 1, CV_32F);
	Mat measurementL = Mat::zeros(measureNum, 1, CV_32F);
	//-----------------------------------------
	KalmanFilter KFR(stateNum, measureNum, 0);
	Mat stateR(stateNum, 1, CV_32F);
	Mat processNoiseR(stateNum, 1, CV_32F);
	Mat measurementR = Mat::zeros(measureNum, 1, CV_32F);
	bool stop = false;
	Mat frame;
	Mat left_image;
	int framecount = 0;
	while (cap.isOpened())
	{
		framecount++;
		if (framecount >= frameThesh)
		{
			cap >> src;
			countL++;
			countR++;
			resize(src, frame, Size(640, 240));
			left_image = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
			undistort(left_image, frameimage, camera_matrix1, dist_coeffs1);
			cutimage = ROI_Operation(frameimage);			
			preprocessimage = imagepreprocess(cutimage);	
			imshow("Canny_image", preprocessimage);
			
			vector<Vec4i>lines;
			Vec4i lL[100], lR[100];
			HoughLinesP(preprocessimage, lines, 1, CV_PI / 90, 30, 50, 50);
		
			double kL[100], kR[100], bL[100], bR[100];
			int countkL = 0, countkR = 0;
			linesSeparation(lines, kL, kR, bL, bR, lL, lR, countkL, countkR);

			struct node dL[100], dR[100];
			distancesort(dL, kL, bL, countkL);				
			distancesort(dR, kR, bR, countkR);				
			//cout << "countkL=" << countkL << '\n';
			//cout << "countkR=" << countkR << '\n';
			qsort(dL, countkL, sizeof(struct node), compare);
			qsort(dR, countkR, sizeof(struct node), compare);
			//-----------------------------------
			randn(stateL, Scalar::all(0), Scalar::all(0.1));
			KFL.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 10, 0, 0, 1, 0, 10, 0, 0, 1, 0, 0, 0, 0, 1);  
			setIdentity(KFL.measurementMatrix);                                                             
			setIdentity(KFL.processNoiseCov, Scalar::all(1e-5));                                           
			setIdentity(KFL.measurementNoiseCov, Scalar::all(1e-2));                                        
			setIdentity(KFL.errorCovPost, Scalar::all(1));                                                  

			randn(stateR, Scalar::all(0), Scalar::all(0.1));
			KFR.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 10, 0, 0, 1, 0, 10, 0, 0, 1, 0, 0, 0, 0, 1);  
			setIdentity(KFR.measurementMatrix);
			setIdentity(KFR.processNoiseCov, Scalar::all(1e-5));
			setIdentity(KFR.measurementNoiseCov, Scalar::all(1e-2));
			setIdentity(KFR.errorCovPost, Scalar::all(1));
			//-------------------------------------------
			int initial_L = 0, initial_R = 0;
			int numL = 0, numR = 0;
			float initial_kL, initial_bL, initial_kR, initial_bR;
			if (framecount == frameThesh)						
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

			KFL.statePost.at<float>(0) = initial_kL;                                   
			KFL.statePost.at<float>(1) = initial_bL;                              

			KFR.statePost.at<float>(0) = initial_kR;
			KFR.statePost.at<float>(1) = initial_bR;
			
		asd1:Mat predictionL = KFL.predict();                                     
			float predict_kL = predictionL.at<float>(0);
			float predict_bL = predictionL.at<float>(1);
			Point pt1, pt11, pt2, pt22;
			predict_point(pt1, pt11, pt2, pt22, predict_kL, predict_bL, predict_shift, predict_y1, predict_y2);  
			int s1 = 0;
			if (countL < 5)                                                                                    
			{
				while ((s1 < countkL) && ((((-bL[dL[s1].num] * 1.0 / kL[dL[s1].num]) < Leftborder) || ((-bL[dL[s1].num] * 1.0 / kL[dL[s1].num]) > Rightborder)))) 
				{
					s1++;
				}
			}
			else
			{
				while ((s1 < countkL) && (((-bL[dL[s1].num] * 1.0 / kL[dL[s1].num]) < Leftborder || ((-bL[dL[s1].num] * 1.0 / kL[dL[s1].num]) > Rightborder) || (((predict_y1-bL[dL[s1].num]) * 1.0 / kL[dL[s1].num]) < pt1.x) || (((predict_y1-bL[dL[s1].num]) * 1.0 / kL[dL[s1].num]) > pt11.x) || (((predict_y2 - bL[dL[s1].num])*1.0 / kL[dL[s1].num]) < pt2.x) || ((predict_y2 - bL[dL[s1].num])*1.0 / (kL[dL[s1].num]) > pt22.x))))
				{                                                                                               
					s1++;
				}
			}
			int m1 = dL[s1].num;

			if (s1 < countkL)
			{
				preL = 0;                             
				measurementL.at<float>(0) = kL[m1];   
				measurementL.at<float>(1) = bL[m1];
			}
			if (preL == resetframe)                 
			{
				int s1 = 0;
				while ((s1 < countkL) && ((-bL[dL[s1].num] / kL[dL[s1].num]) < Leftborder || (-bL[dL[s1].num] / kL[dL[s1].num]) > Rightborder))
				{
					s1++;
				}
				m1 = dL[s1].num;
				if (s1 < countkL)                       
				{
					KFL.statePost.at<float>(0) = kL[m1];
					KFL.statePost.at<float>(1) = bL[m1];
				}
				if (s1 == countkL)                     
				{
					preL = 0;
				}
				countL = 0;                         
				goto asd1;
			}
			if (s1 == countkL)                         
			{
				preL++;                           
				measurementL.at<float>(0) = predict_kL;
				measurementL.at<float>(1) = predict_bL;
			}

			Mat correctedL = KFL.correct(measurementL);            
			float corrected_kL = correctedL.at<float>(0);
			float corrected_bL = correctedL.at<float>(1);
			
			line(cutimage, Point((-corrected_bL)*1.0 / corrected_kL, 0), Point((predict_y2 - corrected_bL)*1.0 / corrected_kL, predict_y2), Scalar(0, 0, 255), 2, CV_AA);
			randn(processNoiseL, Scalar(0), Scalar::all(sqrt(KFL.processNoiseCov.at<float>(0, 0))));
			stateL = KFL.transitionMatrix*stateL + processNoiseL;    
			
		asd2:Mat predictionR = KFR.predict();                                       
			float predict_kR = predictionR.at<float>(0);
			float predict_bR = predictionR.at<float>(1);
			Point pt3, pt33, pt4, pt44;
			predict_point(pt3, pt33, pt4, pt44, predict_kR, predict_bR, predict_shift, predict_y1, predict_y2);  
			int s2 = 0;
			if (countR < 5)                                                                                  
			{
				while ((s2 < countkR) && (((-bR[dR[s2].num]*1.0 / kR[dR[s2].num]) < Leftborder || (-bR[dR[s2].num]*1.0 / kR[dR[s2].num]) > Rightborder)))  
				{
					s2++;
				}
			}
			else
			{
				while ((s2 < countkR) && ((((-bR[dR[s2].num] * 1.0 / kR[dR[s2].num]) < Leftborder) || ((-bR[dR[s2].num] * 1.0 / kR[dR[s2].num]) > Rightborder) || ((predict_y1-bR[dR[s2].num] * 1.0 / kR[dR[s2].num]) < pt4.x) || (((predict_y1-bR[dR[s2].num] * 1.0) / kR[dR[s2].num]) > pt44.x) || (((predict_y2 - bR[dR[s2].num])*1.0 / kR[dR[s2].num]) < pt3.x) || (((predict_y2 - bR[dR[s2].num])*1.0 / kR[dR[s2].num]) > pt33.x))))
				{                                                                                              
					s2++;
				}
			}
			int m2 = dR[s2].num;

			if (s2 < countkR)
			{
				preR = 0;								 
				measurementR.at<float>(0) = kR[m2];		 
				measurementR.at<float>(1) = bR[m2];
			}
			if (preR == 10)							     
			{
				int s2 = 0;
				while ((s2 < countkR) && (((-bR[dR[s2].num] / kR[dR[s2].num]) < Leftborder) || ((-bR[dR[s2].num] / kR[dR[s2].num]) > Rightborder)))  
				{
					s2++;
				}
				m2 = dR[s2].num;
				if (s2 < countkR)                        
				{
					KFR.statePost.at<float>(0) = kR[m2];
					KFR.statePost.at<float>(1) = bR[m2];
				}
				if (s2 == countkR)                      
				{
					preR = 0;
				}
				countR = 0;                        
				goto asd2;
			}
			if (s2 == countkR)                         
			{
				preR++;							      
				measurementR.at<float>(0) = predict_kR;
				measurementR.at<float>(1) = predict_bR;
			}
			Mat correctedR = KFR.correct(measurementR);              
			float corrected_kR = correctedR.at<float>(0);
			float corrected_bR = correctedR.at<float>(1);
			
			line(cutimage, Point((-corrected_bR)*1.0 / corrected_kR, 0), Point((predict_y2 - corrected_bR)*1.0 / corrected_kR, predict_y2), Scalar(0, 0, 255), 2, CV_AA);
			randn(processNoiseR, Scalar(0), Scalar::all(sqrt(KFR.processNoiseCov.at<float>(0, 0))));
			stateR = KFR.transitionMatrix*stateR + processNoiseR;    
			int DL = 0, DR = 0;

			DL = LinesDistance(corrected_kL, corrected_bL)+60;
			DR = LinesDistance(corrected_kR, corrected_bR)-60;
			cout << "Left Lane Distance=" << DL << "mm" << '\n';
			cout << "Right Lane Distance=" << DR<<"mm" << '\n';
			cout << "Lane Distance=" << DL+DR<<"mm" << '\n';
			
			imshow("cutimage", cutimage);
			waitKey(10);
		}
	}
	return(0);
}
