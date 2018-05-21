//stitching.cpp
//Code to stitch two images.


/*
steps:
read two images
detect kepoints using feature detector
compute descriptors of features
match discriptors
find homography (transformation matrix)
perspective transformation on second image
stich second image to first. 
*/


#include <stdio.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp" // This is where actual SURF and SIFT algorithm is located
#include "opencv2/features2d/features2d.hpp" // "features2d" library has to be included
#include "opencv2/stitching/stitcher.hpp"

//#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

vector<Point> get_Size(Mat next_image, Mat Homo);
Mat Homography_from_images(Mat image1, Mat image2);

int main(int argc, char** argv)
{	
	/// Read images
	//Mat image1 = imread("/home/radha/im1.jpg");
	Mat image1 = imread("/home/radha/photo1.jpg");
	if(!image1.data)
	{
		cout << "Could not read image1" << endl;
	}
	//Mat image2 = imread("/home/radha/im2.jpg");
	Mat image2 = imread("/home/radha/photo2.jpg");
	if(!image2.data)
	{
		cout << "Could not read image2" << endl;
	}

	/// Show images
	imshow("Image1", image1);
	imshow("Image2", image2);

	Mat H = Homography_from_images(image1, image2);	

	vector<Point> size;
	size = get_Size(image2, H);
	int wd = size[0].x;
	int ht = size[0].y;

	///Translation downwards prior to transformation to prevent the cutting of transformed image
	//Mat warp_matrix = Mat::eye(2, 3, CV_32F);
	//Mat warp_matrix = Mat(2,3,CV_64F);  //When using warpAffine
    Mat warp_matrix = Mat(3,3,CV_64F);    //When using warpPerspective
    //If you're setting arbitrary values, there is a possibility of erroneous results
    int tx = -size[1].x;
    int ty = -size[1].y;
    warp_matrix.at<double>(0,0) = 1;
    warp_matrix.at<double>(0,1) = 0;
    warp_matrix.at<double>(0,2) = tx;
    warp_matrix.at<double>(1,0) = 0;
    warp_matrix.at<double>(1,1) = 1;
    warp_matrix.at<double>(1,2) = ty;
    warp_matrix.at<double>(2,0) = 0;
    warp_matrix.at<double>(2,1) = 0;
    warp_matrix.at<double>(2,2) = 1;

    //warpAffine(image1, image1, warp_matrix, Size(width, height));
    warpPerspective(image1, image1, warp_matrix, Size(wd, ht));
    imshow("translated_image1", image1);
    //warpAffine(image2, image2, warp_matrix, Size(width, height));
    
    
    ///Warp image2 based on homography
    Mat transformed_image2;
    H = warp_matrix * H;
    //transformation may increase the size of the image, which has to be accounted for
    warpPerspective(image2, transformed_image2, H, Size(wd, ht));
    imshow("transformed_image2", transformed_image2);

   	Mat mask, gray; 
   	cvtColor(image1, gray, CV_BGR2GRAY);
   	threshold(gray, mask, 15, 255, CV_THRESH_BINARY_INV);
	imshow("mask", mask);
	//in_mat.copyTo(out_mat, mask_mat);
	transformed_image2.copyTo(image1, mask);

	//imshow("image2add", image2add);
	imshow("image1", image1);
	/*
	///Stitching
    	// Mat mosaic = zeros(height,width,CV_8UC3); gives error: ‘zeros’ was not declared in this scope
   	 Mat mosaic = Mat::zeros(height,width,CV_8UC3);
	image1.copyTo(mosaic);
   	imshow("mosaic", mosaic);
	*/
	/*
    	//Display mosaic
    	if(!mosaic.empty())
    	{
    	imshow("Mosaic", mosaic);
	}
	*/
	waitKey(0);
	return(0);
}

Mat Homography_from_images(Mat image1, Mat image2)
{
/// Detect the keypoints using SURF Detector
  	int minHessian = 400; //what is this parameter?? 
  	SurfFeatureDetector detector(minHessian);
	vector<KeyPoint> keypoints1;
	vector<KeyPoint> keypoints2;
	detector.detect(image1, keypoints1);
	detector.detect(image2, keypoints2);

	/// Computing descriptors
	SurfDescriptorExtractor extractor;
	Mat descriptors1, descriptors2;
	extractor.compute(image1, keypoints1, descriptors1);
	extractor.compute(image2, keypoints2, descriptors2);

	/// Match descriptors
	BFMatcher matcher(NORM_L2);
	vector <DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);

	/// Find Homography
    vector<Point2f> kp1;
    vector<Point2f> kp2;
    for(int i = 0; i < matches.size(); i++)
    {
       ///Get the keypoints from the matches
       kp1.push_back( keypoints1[ matches[i].queryIdx ].pt );
       kp2.push_back( keypoints2[ matches[i].trainIdx ].pt );
    }

    /*
    //REMEMBER: findhomography() calculates the transformation matrix that will warp image1 in alignment with image2
    //to do the opposite you need to reverse the sequence of input keypoints
    //here i want to warp image2 according to image1, hence the seq. kp2->kp1.
    */
    Mat H = findHomography(kp2, kp1, CV_RANSAC);
    return(H);
}


/*
vector<Point> Size_of_transformed_image(Mat image2, Mat Homography)
{	
	vector<Point> size_point(1);
	size_point[0] = Point(w,h);
    return(size_point);
}
vector<Point> Image_size(1);
Image_size[0] = Size_of_transformed_image(image2, H);
int width = Image_size[0].x;
int height = Image_size[0].y;

*/
vector<Point> get_Size(Mat next_image, Mat Homo)
{
	vector<Point2f> img_corners(4);
 	vector<Point2f> trans_img_corners(4);
 	vector<Point2f> total_corners(8);

  	img_corners[0] = Point(0,0); 	
  	img_corners[1] = Point(next_image.cols,0);
  	img_corners[2] = Point(next_image.cols,next_image.rows);
  	img_corners[3] = Point(0,next_image.rows);

  	perspectiveTransform(img_corners, trans_img_corners, Homo);

  	total_corners[0] = img_corners[0];
	total_corners[1] = img_corners[1];
	total_corners[2] = img_corners[2];
	total_corners[3] = img_corners[3];
	total_corners[4] = trans_img_corners[0];
    total_corners[5] = trans_img_corners[1];
	total_corners[6] = trans_img_corners[2];
	total_corners[7] = trans_img_corners[3];
  		
  		//initializing all values to the zeroth elements
  	int xmin = 0;
	int xmax = 0;
	int ymin = 0;
	int ymax = 0;

  	for(int i=1;i<8;i++)
	{
		if (total_corners[i].x < xmin)
		{
			xmin = total_corners[i].x;
		}
		if (total_corners[i].x > xmax)
		{
			xmax = total_corners[i].x;
		}
		if (total_corners[i].y < ymin)
		{
			ymin = total_corners[i].y;
		}
		if (total_corners[i].y > ymax)
		{
			ymax = total_corners[i].y;
		}
    }
    /*
    cout << "xmin:" << xmin << "\n" << endl;
    cout << "xmax:" << xmax << "\n" << endl;
    cout << "ymin:" << ymin << "\n" << endl;
    cout << "ymax:" << ymax << "\n" << endl;
	*/
    int width = xmax-xmin;
    int height = ymax-ymin;
    //declaration means there are 2 Point type elements in the array Image_size
    vector<Point> Image_size(2);
    //Zeroth Point elelment
 	Image_size[0].x = width; 
 	Image_size[0].y = height;
 	//First Point elelment
 	Image_size[1].x = xmin; 
 	Image_size[1].y = ymin;
    return Image_size; 
}	



