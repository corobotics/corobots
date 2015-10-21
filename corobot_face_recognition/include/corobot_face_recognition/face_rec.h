#ifndef FACEREC_H
#define FACEREC_H

#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <map>
#include <utility>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>

class FaceRec
{
public:
	FaceRec(bool train=true);
	~FaceRec();
	int generateCsv();
	void loadDataFromCsv();
	void trainModelsAndSave();
	void loadModels();
	std::map<std::string, std::pair<std::string, double> > recognize(cv::Mat &frame, bool, bool, bool);

private:
	cv::Ptr<cv::FaceRecognizer> _eigenfacesModel;
	cv::Ptr<cv::FaceRecognizer> _fisherfacesModel;
	cv::Ptr<cv::FaceRecognizer> _lbphModel;
	std::vector<cv::Mat> _faceImgs;
	std::vector<int> _faceLabels;
	std::map<int, std::string> _faceNames;
};

#endif