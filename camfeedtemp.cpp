// camera feed template

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
 
using namespace cv;
using namespace std;

//char* argv[] can also be written as char **argv 
//continuous feed from camera
int main(int argc, char* argv[])
{
	VideoCapture Cap(1);
	if (Cap.isOpened()==0) 
	{
		cout<< "Cannot open video cam" << endl;
		return -1;
	}
	while(1)
	{
		Mat frame;
		bool success = Cap.read(frame);
		if (!success)
		{
			cout<< "Cannot read frames from the video stream" << endl;
			break;
		}
		// Here goes the code for processing of one frame 
		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop  
    	{  
    		cout << "esc key is pressed by user" << endl;  
    		break;   
    	}
	}
	return 0;
}