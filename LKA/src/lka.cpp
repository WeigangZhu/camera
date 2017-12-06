#include "det.hpp"





using namespace cv;
using namespace std;


/** ******摄像头内部外部参数************************/
 
Point UV_0(u_0, v_0);

Point UV_2(u_0, 0);



Mat camera_matrix1 = (Mat_<double>(3, 3) << 944, 0.0, 350,
											0.0, 942, 305,
											0.0, 0.0, 1.0);
Mat dist_coeffs1 = (Mat_<double>(5, 1) << -0.3523, 0.13333 , 0 , 0 ,0.00000);

/*---------------------------------------------------------*/
const int stateNum = 4;	//the dim of kalman state and meature vector
const int measureNum = 2;

Point Vanish(160, 55);	
int Leftborder = 45, Rightborder = 285;
int framecount = 0;		
int predict_shift =15;
int predict_y1 = 100, predict_y2 = 300;
int countL = 0,countR=0;
int preL = 0,preR=0;
int resetframe = 10;
Size imageSize;
const int height = 240;
const int width = 320;
const int frameThesh = 60000;






int main()
{
	Mat src,frameimage,cutimage,preprocessimage;
	VideoCapture cap(0);
	if (!cap.isOpened())
	{
		cout << "打开摄像头失败" << '\n';
		return -1;	
	}

	//------------------------   左车道卡尔曼----------------------
	KalmanFilter KFL(stateNum,measureNum,0);
	Mat stateL(stateNum,1,CV_32F);
	Mat processNoiseL(stateNum, 1, CV_32F);
	Mat measurementL = Mat::zeros(measureNum, 1, CV_32F);
	//--------------------------右车道卡尔曼-----------------------
	KalmanFilter KFR(stateNum, measureNum, 0);
	Mat stateR(stateNum, 1, CV_32F);
	Mat processNoiseR(stateNum, 1, CV_32F);
	Mat measurementR = Mat::zeros(measureNum, 1, CV_32F);
	
	
	bool stop = false;
	Mat frame;
	Mat left_image;



	while (cap.isOpened())
	{
		framecount++;
		if (framecount >= frameThesh)
		{
			cap >> src;
			countL++;
			countR++;
			imshow("src", src);
			resize(src, frame, Size(1280, 320));
			left_image = src(cv::Rect(0, 0, frame.cols / 2, frame.rows));
			undistort(left_image, frameimage, camera_matrix1, dist_coeffs1);
			cutimage = ROI_Operation(frameimage);			
			preprocessimage = imagepreprocess(cutimage);	
			imshow("Canny_image", preprocessimage);
			
			/*直线检测*/
			vector<Vec4i>lines;
			Vec4i lL[100], lR[100];
			HoughLinesP(preprocessimage, lines, 1, CV_PI / 90, 30, 50, 50);
			
		/*	for (int i = 0; i < lines.size(); i++)
			{
				Vec4i l = lines[i];
				line(cutimage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 0), 1, CV_AA);
			}
		*/
		
		
			double kL[100], kR[100], bL[100], bR[100];
			int countkL = 0, countkR = 0;
			linesSeparation(lines, kL, kR, bL, bR, lL, lR, countkL, countkR);

			struct node dL[100], dR[100];
			distancesort(dL, kL, bL, countkL);				
			distancesort(dR, kR, bR, countkR);	
						
			cout << "countkL=" << countkL << '\n';
			cout << "countkR=" << countkR << '\n';
			qsort(dL, countkL, sizeof(struct node), compare);
			qsort(dR, countkR, sizeof(struct node), compare);
			
			//-------------------initialize the klman parameters----------------
			randn(stateL, Scalar::all(0), Scalar::all(0.1));
			KFL.transitionMatrix =(cv::Mat_<float>(4, 4) << 1, 0, 10, 0, 0, 1, 0, 10, 0, 0, 1, 0, 0, 0, 0, 1);  
			setIdentity(KFL.measurementMatrix);                                                             
			setIdentity(KFL.processNoiseCov, Scalar::all(1e-5));                                           
			setIdentity(KFL.measurementNoiseCov, Scalar::all(1e-2));                                       
			setIdentity(KFL.errorCovPost, Scalar::all(1));                                                 


			randn(stateR, Scalar::all(0), Scalar::all(0.1));
			KFR.transitionMatrix =(cv::Mat_<float>(4, 4) << 1, 0, 10, 0, 0, 1, 0, 10, 0, 0, 1, 0, 0, 0, 0, 1);  
			setIdentity(KFR.measurementMatrix);
			setIdentity(KFR.processNoiseCov, Scalar::all(1e-5));
			setIdentity(KFR.measurementNoiseCov, Scalar::all(1e-2));
			setIdentity(KFR.errorCovPost, Scalar::all(1));
			
			
			
			//-----------------------------------------------------------------------------------------------
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
			
			//ŒÙÉèÃ»ÓÐ±äµÀœøÐÐ³µµÀÏßžú×Ù
			//--------------------------œøÐÐ×ó³µµÀÏß×·×Ù-----------------------
		asd1:Mat predictionL = KFL.predict();                                         //µÃµœÔ€²â²ÎÊý kºÍb
			float predict_kL = predictionL.at<float>(0);
			float predict_bL = predictionL.at<float>(1);
			Point pt1, pt11, pt2, pt22;
			predict_point(pt1, pt11, pt2, pt22, predict_kL, predict_bL, predict_shift, predict_y1, predict_y2);  // œ«Ô€²âµÃµœµÄÖ±ÏßÏò×óÏòÓÒÆœÒÆpredict_shift£¬²¢ÇóµÃÓëpredict_y1, predict_y2µÄœ»µã£¬Ò»¹²ËÄžöœ»µã
			int s1 = 0;
			if (countL < 5)                                                                                    //×ó²à²»ÄÜÓÃÔ€²â·¶Î§¶Ô³µµÀÏßœøÐÐÏÞÖÆµÄÖ¡Êý
			{
				while ((s1 < countkL) && ((((-bL[dL[s1].num] * 1.0 / kL[dL[s1].num]) < Leftborder) || ((-bL[dL[s1].num] * 1.0 / kL[dL[s1].num]) > Rightborder))))  //ÎŽÊ¹ÓÃÔ€²â·¶Î§£¬ÏÞÖÆÌõŒþÓëÖ®ŒäµÄÏàÍ¬
				{
					s1++;
				}
			}
			else
			{
				while ((s1 < countkL) && (((-bL[dL[s1].num] * 1.0 / kL[dL[s1].num]) < Leftborder || ((-bL[dL[s1].num] * 1.0 / kL[dL[s1].num]) > Rightborder) || (((predict_y1-bL[dL[s1].num]) * 1.0 / kL[dL[s1].num]) < pt1.x) || (((predict_y1-bL[dL[s1].num]) * 1.0 / kL[dL[s1].num]) > pt11.x) || (((predict_y2 - bL[dL[s1].num])*1.0 / kL[dL[s1].num]) < pt2.x) || ((predict_y2 - bL[dL[s1].num])*1.0 / (kL[dL[s1].num]) > pt22.x))))
				{                                                                                               //Ê¹ÓÃÔ€²â·¶Î§£¬Ôò³µµÀÏßÐèÒªÔÚÔ€²â·¶Î§ÄÚ
					s1++;
				}
			}
			int m1 = dL[s1].num;

			if (s1 < countkL)
			{
				preL = 0;                             //Îª·ÀÖ¹²»Á¬ÐøÎŽŒì²â³ö³µµÀÏßµÄÖ¡ÊýÏàŒÓµŒÖÂžú×ÙÖØÆô£¬ÉèÖÃÖ»ÒªÕÒµœ³µµÀÏß£¬ÔòpreLÖÃ0
				measurementL.at<float>(0) = kL[m1];   //Èç¹ûŒì²âµœ³µµÀÏß£¬Ôò³µµÀÏß²ÎÊýÎªÏµÍ³²âÁ¿Öµ
				measurementL.at<float>(1) = bL[m1];
			}
			if (preL == resetframe)                   //Èç¹ûÁ¬ÐøÎŽŒì²âµœ³µµÀÏß²ÎÊýµÈÓÚÉèÖÃºÃµÄÖØÆôÖ¡Êý£¬Ôòžú×ÙœøÐÐÖØÆô£¬ÐèÒªÔÙŽÎÕÒµœÏµÍ³³õÊŒÖµ
			{
				int s1 = 0;
				while ((s1 < countkL) && ((-bL[dL[s1].num] / kL[dL[s1].num]) < Leftborder || (-bL[dL[s1].num] / kL[dL[s1].num]) > Rightborder))
				{
					s1++;
				}
				m1 = dL[s1].num;
				if (s1 < countkL)                        //ÖØÐÂŒì²âµœ³µµÀÏß£¬²¢ž³žø¿š¶ûÂüÂË²šÆ÷
				{
					KFL.statePost.at<float>(0) = kL[m1];
					KFL.statePost.at<float>(1) = bL[m1];
				}
				if (s1 == countkL)                      //Èç¹ûÖØÆôÈÔÎŽÕÒµœ£¬ŒÌÐøœ«Ô€²âÖµ×÷ÎªÏµÍ³²âÁ¿Öµ
				{
					preL = 0;
				}
				countL = 0;                         //ÖØÆôºóÖ¡ŒÆÊýÖØÐÂ¿ªÊŒ
				goto asd1;
			}
			if (s1 == countkL)                         //Èç¹ûÃ»ÓÐŒì²âµœ³µµÀÏß£¬ÔòÔ€²âÖµ×÷ÎªÏµÍ³²âÁ¿Öµ
			{
				preL++;                             //ÎŽŒì²âµœ³µµÀÏßÖ¡ÊýŒÓ1
				measurementL.at<float>(0) = predict_kL;
				measurementL.at<float>(1) = predict_bL;
			}

			Mat correctedL = KFL.correct(measurementL);              //µÃµœÏµÍ³Ô€²âÖµºÍ²âÁ¿Öµºó£¬µÃµœÐÞÕýÖµ£¬ŒŽžÃÊ±¿ÌµÄ×îÓÅ¹ÀŒÆÖµ
			float corrected_kL = correctedL.at<float>(0);
			float corrected_bL = correctedL.at<float>(1);
			cout << "ÓÒ³µµÀÏßÐ±ÂÊ£º" << corrected_kL << '\n';
			line(cutimage, Point((-corrected_bL)*1.0 / corrected_kL, 0), Point((predict_y2 - corrected_bL)*1.0 / corrected_kL, predict_y2), Scalar(0, 0, 255), 2, CV_AA);
			randn(processNoiseL, Scalar(0), Scalar::all(sqrt(KFL.processNoiseCov.at<float>(0, 0))));
			stateL = KFL.transitionMatrix*stateL + processNoiseL;    //žüÐÂ×ŽÌ¬·œ³Ì
			//--------------------------œøÐÐÓÒ³µµÀÏß×·×Ù-----------------------
		asd2:Mat predictionR = KFR.predict();                                         //µÃµœÔ€²â²ÎÊý kºÍb
			float predict_kR = predictionR.at<float>(0);
			float predict_bR = predictionR.at<float>(1);
			Point pt3, pt33, pt4, pt44;
			predict_point(pt3, pt33, pt4, pt44, predict_kR, predict_bR, predict_shift, predict_y1, predict_y2);  // œ«Ô€²âµÃµœµÄÖ±ÏßÏò×óÏòÓÒÆœÒÆpredict_shift£¬²¢ÇóµÃÓëpredict_y1, predict_y2µÄœ»µã£¬Ò»¹²ËÄžöœ»µã
			int s2 = 0;
			if (countR < 5)                                                                                    //ÓÒ²à²»ÄÜÓÃÔ€²â·¶Î§¶Ô³µµÀÏßœøÐÐÏÞÖÆµÄÖ¡Êý
			{
				while ((s2 < countkR) && (((-bR[dR[s2].num]*1.0 / kR[dR[s2].num]) < Leftborder || (-bR[dR[s2].num]*1.0 / kR[dR[s2].num]) > Rightborder)))  //ÎŽÊ¹ÓÃÔ€²â·¶Î§£¬ÏÞÖÆÌõŒþÓëÖ®ŒäµÄÏàÍ¬
				{
					s2++;
				}
			}
			else
			{
				while ((s2 < countkR) && ((((-bR[dR[s2].num] * 1.0 / kR[dR[s2].num]) < Leftborder) || ((-bR[dR[s2].num] * 1.0 / kR[dR[s2].num]) > Rightborder) || ((predict_y1-bR[dR[s2].num] * 1.0 / kR[dR[s2].num]) < pt4.x) || (((predict_y1-bR[dR[s2].num] * 1.0) / kR[dR[s2].num]) > pt44.x) || (((predict_y2 - bR[dR[s2].num])*1.0 / kR[dR[s2].num]) < pt3.x) || (((predict_y2 - bR[dR[s2].num])*1.0 / kR[dR[s2].num]) > pt33.x))))
				{                                                                                               //Ê¹ÓÃÔ€²â·¶Î§£¬Ôò³µµÀÏßÐèÒªÔÚÔ€²â·¶Î§ÄÚ
					s2++;
				}
			}
			int m2 = dR[s2].num;

			if (s2 < countkR)
			{
				preR = 0;								 //Îª·ÀÖ¹²»Á¬ÐøÎŽŒì²â³ö³µµÀÏßµÄÖ¡ÊýÏàŒÓµŒÖÂžú×ÙÖØÆô£¬ÉèÖÃÖ»ÒªÕÒµœ³µµÀÏß£¬ÔòpreLÖÃ0
				measurementR.at<float>(0) = kR[m2];		 //Èç¹ûŒì²âµœ³µµÀÏß£¬Ôò³µµÀÏß²ÎÊýÎªÏµÍ³²âÁ¿Öµ
				measurementR.at<float>(1) = bR[m2];
			}
			if (preR == 10)							     //Èç¹ûÁ¬ÐøÎŽŒì²âµœ³µµÀÏß²ÎÊýµÈÓÚÉèÖÃºÃµÄÖØÆôÖ¡Êý£¬Ôòžú×ÙœøÐÐÖØÆô£¬ÐèÒªÔÙŽÎÕÒµœÏµÍ³³õÊŒÖµ
			{
				int s2 = 0;
				while ((s2 < countkR) && (((-bR[dR[s2].num] / kR[dR[s2].num]) < Leftborder) || ((-bR[dR[s2].num] / kR[dR[s2].num]) > Rightborder)))  //ÎŽÊ¹ÓÃÔ€²â·¶Î§£¬ÏÞÖÆÌõŒþÓëÖ®ŒäµÄÏàÍ¬
				{
					s2++;
				}
				m2 = dR[s2].num;
				if (s2 < countkR)                        //ÖØÐÂŒì²âµœ³µµÀÏß£¬²¢ž³žø¿š¶ûÂüÂË²šÆ÷
				{
					KFR.statePost.at<float>(0) = kR[m2];
					KFR.statePost.at<float>(1) = bR[m2];
				}
				if (s2 == countkR)                      //Èç¹ûÖØÆôÈÔÎŽÕÒµœ£¬ŒÌÐøœ«Ô€²âÖµ×÷ÎªÏµÍ³²âÁ¿Öµ
				{
					preR = 0;
				}
				countR = 0;                         //ÖØÆôºóÖ¡ŒÆÊýÖØÐÂ¿ªÊŒ
				goto asd2;
			}
			if (s2 == countkR)                         //Èç¹ûÃ»ÓÐŒì²âµœ³µµÀÏß£¬ÔòÔ€²âÖµ×÷ÎªÏµÍ³²âÁ¿Öµ
			{
				preR++;							       //ÎŽŒì²âµœ³µµÀÏßÖ¡ÊýŒÓ1
				measurementR.at<float>(0) = predict_kR;
				measurementR.at<float>(1) = predict_bR;
			}
			Mat correctedR = KFR.correct(measurementR);              //µÃµœÏµÍ³Ô€²âÖµºÍ²âÁ¿Öµºó£¬µÃµœÐÞÕýÖµ£¬ŒŽžÃÊ±¿ÌµÄ×îÓÅ¹ÀŒÆÖµ
			float corrected_kR = correctedR.at<float>(0);
			float corrected_bR = correctedR.at<float>(1);
			cout << "ÓÒ³µµÀÏßÐ±ÂÊ£º" << corrected_kR << '\n';
			line(cutimage, Point((-corrected_bR)*1.0 / corrected_kR, 0), Point((predict_y2 - corrected_bR)*1.0 / corrected_kR, predict_y2), Scalar(0, 0, 255), 2, CV_AA);
			randn(processNoiseR, Scalar(0), Scalar::all(sqrt(KFR.processNoiseCov.at<float>(0, 0))));
			stateR = KFR.transitionMatrix*stateR + processNoiseR;    //žüÐÂ×ŽÌ¬·œ³Ì
			
			
			//µ÷ÊÔËùÓÃ
			
		/*	Vanish.y = (bL[dL[0].num] * kR[dR[0].num] - bR[dR[0].num] * kL[dL[0].num]) / (kR[dR[0].num] - kL[dL[0].num]);
			Vanish.x = (Vanish.y - bL[dL[0].num]) / kL[dL[0].num];
			cout << "Vanish" << "(" << Vanish.x << "," << Vanish.y << ")" << '\n';
			line(cutimage, Point(lL[dL[0].num][0], lL[dL[0].num][1]), Point(Vanish.x, Vanish.y), Scalar(0, 0, 255), 2, CV_AA);
			line(cutimage, Point(lR[dR[0].num][0], lR[dR[0].num][1]), Point(Vanish.x, Vanish.y), Scalar(0, 0, 255), 2, CV_AA);
		*/
		
			int DL = 0, DR = 0;//Àë×ó±ß³µµÀÏßµÄŸàÀë

			DL = LinesDistance(corrected_kL, corrected_bL)+60;
			DR = LinesDistance(corrected_kR, corrected_bR)-60;
			cout << "the distance to the left line=" << DL << "mm" << '\n';
			cout << "the distance to the right line=" << DR<<"mm" << '\n';
																																																																		
			cout << "the total width of lane=" << DL+DR<<"mm" << '\n';
			string textL = "Left Lane", textR = "Right Lane";
			//string dataL = double_to_string(DL, 3), dataR = double_to_string(DR, 3);
			//
			//serialport_translationL(dataL);
			//serialport_translationR(dataR);
			putText(cutimage, textL, Point(cutimage.rows / 2-50, cutimage.cols / 2-10), CV_FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 255));
			putText(cutimage, textR, Point(cutimage.rows / 2 +140, cutimage.cols / 2-10 ), CV_FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 255));
			//putText(cutimage, DL, Point(cutimage.rows / 2 - 30, cutimage.cols / 2 +45), CV_FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 255));
			//putText(cutimage, DR, Point(cutimage.rows / 2 + 120, cutimage.cols / 2 +45), CV_FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 255));
			imshow("cutimage", cutimage);
			waitKey(10);
		}
	}
	cap.release();
	return(0);


}
