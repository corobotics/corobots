//#ifndef FACEIMGCAPTURER_H
//#define FACEIMGCAPTURER_H

//#include "FaceDetector.h"
#include <ros/ros.h>
#include <sys/stat.h>
#include <iostream>
#include <sstream> 
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <dirent.h>
#include <ros/package.h>
#include "tantriggs.cpp"

class FaceImgCapturer
{
public:
	FaceImgCapturer(std::string name);
	// FaceImgCapturer(cv::VideoCapture*);
	~FaceImgCapturer();
	void capture(cv::Mat &face);
	// void clearLastImage();
	int checkIfDirExistsOtherwiseCreate(std::string, bool computeLastImgIndex=false);
	int getNoImgsClicked();

private:
	FaceImgCapturer();
	std::string _personName;
	int _imagesClicked;
	cv::Mat _lastImage;
	int _lastImgIndex;
	double _timeOfLastImage;
	double getSimilarity(cv::Mat A, cv::Mat B);
};

//#endif

//#include "FaceImgCapturer.h"

using namespace cv;
using namespace std;

// TODO
const string face_database_path_2 = ros::package::getPath("corobot_face_recognition") + "/face_database/";
const double MIN_TIME_DIFF = 1;
const double MIN_IMG_DIFF = 0.3;
const int no_of_images_to_click = 15;
const double face_img_size = 100.0;

FaceImgCapturer::FaceImgCapturer(string name):
// FaceImgCapturer::FaceImgCapturer(VideoCapture* capture)
	_personName(name),
	_imagesClicked(0)
{
	checkIfDirExistsOtherwiseCreate("");
	checkIfDirExistsOtherwiseCreate(name);
	checkIfDirExistsOtherwiseCreate(name + "/original", true);
	checkIfDirExistsOtherwiseCreate(name + "/grayscale");
	checkIfDirExistsOtherwiseCreate(name + "/histeq");
	checkIfDirExistsOtherwiseCreate(name + "/tantriggs");
	checkIfDirExistsOtherwiseCreate(name + "/avg4");
}

/*FaceImgCapturer::FaceImgCapturer() {
	
}*/

FaceImgCapturer::~FaceImgCapturer() {
	
}

void FaceImgCapturer::capture(Mat &face) {
    double imageDiff = 10000000000.0;
	double currentTime, timeDiff_seconds;
	Mat mirroredFace;
	Mat faceHisteq;
	Mat mirroredFaceHisteq;
	Mat frame;
	string saveName;
	stringstream ss;
	ss << _lastImgIndex;
	string _lastImgIndexStr = ss.str();


	if (_lastImage.data) {
		imageDiff = getSimilarity(face, _lastImage);
	}

	currentTime = (double) getTickCount();
	timeDiff_seconds = (currentTime - _timeOfLastImage)/getTickFrequency();

	// Only process the face if it is noticeably different from the previous frame and there has been noticeable time gap.
    if ((imageDiff > MIN_IMG_DIFF) && (timeDiff_seconds > MIN_TIME_DIFF)) {
    	flip(face, mirroredFace, 1);
    	
    	// Resize original and mirrored face imgs
    	resize(face, face, Size(face_img_size, face_img_size));
    	resize(mirroredFace, mirroredFace, Size(face_img_size, face_img_size));

    	// Save original face img
    	saveName = face_database_path_2 + "/" + _personName + "/original/" + _lastImgIndexStr + ".jpg";
    	imwrite(saveName, face);
    	
    	// Save grayscale face img
    	cvtColor(face, face, CV_BGR2GRAY);
    	saveName = face_database_path_2 + "/" + _personName + "/grayscale/" + _lastImgIndexStr + ".jpg";
    	imwrite(saveName, face);

    	// Save histogram equalized face img
    	equalizeHist(face, faceHisteq);
    	saveName = face_database_path_2 + "/" + _personName + "/histeq/" + _lastImgIndexStr + ".jpg";
    	imwrite(saveName, faceHisteq);


    	namedWindow("face", WINDOW_AUTOSIZE);
		imshow("face", face);
		waitKey(30);

    	// Save tantriggs face img
    	Mat faceTantriggs = norm_0_255(tan_triggs_preprocessing(face));
    	saveName = face_database_path_2 + "/" + _personName + "/tantriggs/" + _lastImgIndexStr + ".jpg";
    	imwrite(saveName, faceTantriggs);
    	
    	namedWindow("faceTantriggs", WINDOW_AUTOSIZE);
		imshow("faceTantriggs", faceTantriggs);
		waitKey(30);


    	// TODO
    	// save avg4 face img

    	_lastImgIndex++;
    	ss.str("");
    	ss << _lastImgIndex;
    	_lastImgIndexStr = ss.str();


    	// Save mirrored original face img
    	saveName = face_database_path_2 + "/" + _personName + "/original/" + _lastImgIndexStr + ".jpg";
    	imwrite(saveName, mirroredFace);
    	
    	// Save mirrored grayscale face img
    	cvtColor(mirroredFace, mirroredFace, CV_BGR2GRAY);
    	saveName = face_database_path_2 + "/" + _personName + "/grayscale/" + _lastImgIndexStr + ".jpg";
    	imwrite(saveName, mirroredFace);

    	// Save mirrored histogram equalized face img
    	equalizeHist(mirroredFace, mirroredFaceHisteq);
    	saveName = face_database_path_2 + "/" + _personName + "/histeq/" + _lastImgIndexStr + ".jpg";
    	imwrite(saveName, mirroredFaceHisteq);

    	/// Save tantriggs mirrored face img
    	faceTantriggs = norm_0_255(tan_triggs_preprocessing(mirroredFace));
    	saveName = face_database_path_2 + "/" + _personName + "/tantriggs/" + _lastImgIndexStr + ".jpg";
    	imwrite(saveName, faceTantriggs);
    	 
    	// TODO
    	// save mirrored avg4 face img

    	_lastImgIndex++;

        // Keep a copy of the current face, to compare on next iteration.
        _lastImage = face;
        _timeOfLastImage = currentTime;
        _imagesClicked++;

    }
	
}


int FaceImgCapturer::getNoImgsClicked() {
	return _imagesClicked;
}


// Compare two images by getting the L2 error (square-root of sum of squared error).
// Source: https://github.com/MasteringOpenCV/code/blob/master/Chapter8_FaceRecognition/recognition.cpp
double FaceImgCapturer::getSimilarity(Mat A, Mat B) {
	if ( A.rows > 0 && A.rows == B.rows && A.cols > 0 && A.cols == B.cols ) {
	    // Calculate the L2 relative error between images.
	    double errorL2 = norm( A, B, CV_L2 );
	    // Convert to a reasonable scale, since L2 error is summed across all pixels of the image.
	    double similarity = errorL2 / (double)( A.rows * A.cols );
	    return similarity;
	}
	else {
	    //Images have a different size
	    return 100000000.0;  // Return a bad value
	}
}

// WARNING: LINUX ONLY
int FaceImgCapturer::checkIfDirExistsOtherwiseCreate(string name, bool computeLastImgIndex) {
	string path = face_database_path_2 + name;

	//cout << "path: " << path << endl;
	struct stat sb;

	// check if dir exists
	if (stat(path.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
	    // cout << "User record exists." << endl;
	    ROS_INFO ("Directory exists: %s", path.c_str());
	    
	    if (computeLastImgIndex) {
		   	struct dirent *entry;
	    	int ret = 1;
	    	DIR *dir;
	    	dir = opendir (path.c_str());
	    	string f_name;
	    	int index;
	    	_lastImgIndex = 0;

	    	while ((entry = readdir (dir)) != NULL) {
	        	f_name = entry->d_name;
	        	index = atoi(f_name.c_str());
	        	// cout << "index:" << to_string(index) << endl;
	        	if (index > _lastImgIndex)
	        		_lastImgIndex = index;
	    	}

	    	// cout << "_lastImgIndex: " << _lastImgIndex << endl;      
		   	(void)closedir(dir);
	    }

	    return 0;
	}
	else {
		const int dir_err = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		if (-1 == dir_err) {
		    // cout << "Error creating directory: " << path << endl;
		    ROS_ERROR ("Error creating directory: %s", path.c_str());
		    return -1;
		} else {
			// cout << "Directory created: " << path << endl;
			ROS_INFO ("Directory created: %s", path.c_str());
			_lastImgIndex = 0;
			return 1;
		}
	}
}



