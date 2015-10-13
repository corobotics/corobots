//#include "FaceRec.h"

#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <map>
#include <utility>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/package.h>

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

using namespace std;
using namespace cv;

// TODO
const string pkg_path = ros::package::getPath("corobot_face_recognition");
const string face_database_path = pkg_path + "/face_database";
const string csv_file = pkg_path + "/data/training_set.csv";
const string eigenfacesModel_file = pkg_path + "/data/eigenfacesModel.xml";
const string fisherfacesModel_file = pkg_path + "/data/fisherfacesModel.xml";
const string lbphModel_file = pkg_path + "/data/lbphModel.xml";
const string script_file = pkg_path + "/scripts/create_csv.py";

FaceRec::FaceRec(bool train) 
: _eigenfacesModel( createEigenFaceRecognizer() )
, _fisherfacesModel( createFisherFaceRecognizer() )
, _lbphModel( createLBPHFaceRecognizer() )
{
	if (train)
		generateCsv();

	loadDataFromCsv();

	if (train)
		trainModelsAndSave();
	else
		loadModels();
}

FaceRec::~FaceRec() {
	
}

int FaceRec::generateCsv() {
	string cmd = "python " + script_file + " " + face_database_path + " " + csv_file;
	return system(cmd.c_str());
}

void FaceRec::loadDataFromCsv() {
	ifstream file(csv_file.c_str(), ifstream::in);
    if (!file) {
        cout << "No valid input file was given, please check the given filename." << endl;
        return;
    }
    string line, imgPath, imgLabel, imgName;
    char separator = ';';
    int label;
    
    while (getline(file, line)) {
        stringstream liness(line);
        getline(liness, imgPath, separator);
        getline(liness, imgLabel, separator);
        getline(liness, imgName);
        if(!imgPath.empty() && !imgLabel.empty() && !imgName.empty()) {
            _faceImgs.push_back(imread(imgPath, 0));
            label = atoi(imgLabel.c_str());
            _faceLabels.push_back(label);
            _faceNames[label] = imgName;
        }
    }
}

void FaceRec::trainModelsAndSave() {
	_eigenfacesModel->train(_faceImgs, _faceLabels);
	_fisherfacesModel->train(_faceImgs, _faceLabels);
	_lbphModel->train(_faceImgs, _faceLabels);

	_eigenfacesModel->save(eigenfacesModel_file);
	_fisherfacesModel->save(fisherfacesModel_file);
	_lbphModel->save(lbphModel_file);
}

void FaceRec::loadModels() {
	_eigenfacesModel->load(eigenfacesModel_file);
	_fisherfacesModel->load(fisherfacesModel_file);
	_lbphModel->load(lbphModel_file);
}

map<string, pair<string, double> > FaceRec::recognize(Mat &frame, bool eigenfaces, bool fisherfaces, bool lbph) {
	map<string, pair<string, double> > results;
	int label;
	double confidence;

	if (eigenfaces) {
		label = -1;
		confidence = 0.0;
		_eigenfacesModel->predict(frame, label, confidence);
		results["eigenfaces"] = make_pair( _faceNames[label], confidence );
	}

	if (fisherfaces) {
		label = -1;
		confidence = 0.0;
		_fisherfacesModel->predict(frame, label, confidence);
		results["fisherfaces"] = make_pair( _faceNames[label], confidence );
	}

	if (lbph) {
		label = -1;
		confidence = 0.0;
		_lbphModel->predict(frame, label, confidence);
		results["lbph"] = make_pair( _faceNames[label], confidence );
	}

	return results;
}


