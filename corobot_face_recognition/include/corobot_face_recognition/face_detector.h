#ifndef FACEDETECTOR_H
#define FACEDETECTOR_H

#include <vector>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

class FaceDetector
{
public:
	FaceDetector();
	~FaceDetector();
	void detectFaces(cv::Mat &frame, std::vector<cv::Rect> &faces);
	// std::vector<cv::Rect> detectFaces(cv::Mat frame);
	void detectEyes(cv::Mat &frame, std::vector<cv::Rect> &eyes);
	// std::vector<cv::Mat> getFaceImgs(cv::Mat &frame);
	std::vector<cv::Mat> getFaceImgs(cv::Mat &frame, std::vector<cv::Rect> &faces, bool drawBoxes);

private:
	cv::CascadeClassifier _faceCascade;
	cv::CascadeClassifier _eyesCascade;
};

#endif