//#include "FaceDetector.h"
#include <vector>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <ros/package.h> //to get pkg path

class FaceDetector
{
public:
	FaceDetector();
	~FaceDetector();
	void detectFaces(cv::Mat &frame, std::vector<cv::Rect> &faces, bool drawBoxes);
	// std::vector<cv::Rect> detectFaces(cv::Mat frame);
	void detectEyes(cv::Mat &frame, std::vector<cv::Rect> &eyes);
	// std::vector<cv::Mat> getFaceImgs(cv::Mat &frame);
	std::vector<cv::Mat> getFaceImgs(cv::Mat &frame, std::vector<cv::Rect> &faces, bool drawBoxes);

private:
	cv::CascadeClassifier _faceCascade;
	cv::CascadeClassifier _eyesCascade;

protected:
	void _drawBoxes(cv::Mat &frame, std::vector<cv::Rect> &faces);
};
/////////

using namespace std;
using namespace cv;

// TODO
const string face_cascade_file = ros::package::getPath("corobot_face_recognition") + "/data/haarcascade_frontalface_alt.xml";
const string eyes_cascade_file = ros::package::getPath("corobot_face_recognition") + "/data/haarcascade_eye_tree_eyeglasses.xml";

FaceDetector::FaceDetector() {
	// Load the cascades
  	if ( !_faceCascade.load( face_cascade_file ) ) {
  		cout << "Error loading face_cascade from file." << endl;
  	}
  
  	if ( !_eyesCascade.load( eyes_cascade_file ) ) {
  		cout << "Error loading eyes_cascade from file." << endl;
  	}
}

FaceDetector::~FaceDetector() {
}

// void FaceDetector::detectFaces(Mat frame, vector<Rect> faces) {
void FaceDetector::detectFaces(Mat &frame, vector<Rect> &faces, bool drawBoxes) {
	Mat frame_gray;
	cvtColor( frame, frame_gray, CV_BGR2GRAY );
	equalizeHist( frame_gray, frame_gray );

	// _faceCascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
	_faceCascade.detectMultiScale( frame_gray, faces, 1.1, 2, CV_HAAR_DO_CANNY_PRUNING, Size(30, 30) );

	if (drawBoxes)
		_drawBoxes(frame, faces);

}

void FaceDetector::detectEyes(Mat &frame, vector<Rect> &eyes) {

}

vector<Mat> FaceDetector::getFaceImgs(Mat &frame, vector<Rect> &faces, bool drawBoxes) {
	// vector<Rect> faces;
	vector<Mat> faceImgs;
	
	detectFaces(frame, faces, false);

	for (int i = 0; i < faces.size(); ++i) {
		faceImgs.push_back( frame(faces[i]) );
	}

	if (drawBoxes)
		_drawBoxes(frame, faces);

	return faceImgs;
}

void FaceDetector::_drawBoxes(Mat &frame, vector<Rect> &faces) {
	for (int i = 0; i < faces.size(); ++i) {
		rectangle( frame, Point( faces[i].x, faces[i].y ), Point( faces[i].x + faces[i].width, faces[i].y + faces[i].height ), Scalar( 0, 255, 255 ) );
	}
}