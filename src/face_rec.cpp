//#include "FaceRec.h"
#include <ros/ros.h>
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <map>
#include <utility>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/package.h>
#include <dirent.h>

class FaceRec
{
public:
	FaceRec(bool train=true, std::string preprocessing="tantriggs");
	~FaceRec();
	int generateCsv();
	void loadDataFromCsv();
	void readTrainingDb(std::string dbpath);
	void trainModelsAndSave(bool trainAll=false);
	void loadModels();
	void train(std::string);
	std::map<std::string, std::pair<std::string, double> > recognize(cv::Mat &frame, bool, bool, bool);

private:
	cv::Ptr<cv::FaceRecognizer> _eigenfacesModel;
	cv::Ptr<cv::FaceRecognizer> _fisherfacesModel;
	cv::Ptr<cv::FaceRecognizer> _lbphModel;
	std::vector<cv::Mat> _faceImgs;
	std::vector<int> _faceLabels;

	std::vector<cv::Mat> _grayscaleFaceImgs;
	std::vector<int> _grayscaleFaceLabels;
	std::vector<cv::Mat> _histeqFaceImgs;
	std::vector<int> _histeqFaceLabels;
	std::vector<cv::Mat> _avg4FaceImgs;
	std::vector<int> _avg4FaceLabels;
	std::vector<cv::Mat> _tantriggsFaceImgs;
	std::vector<int> _tantriggsFaceLabels;

	std::map<int, std::string> _faceNames;
	std::string _preprocessing;

	void readimgs(std::string dirpath,
		std::vector<cv::Mat> &faceImgs, std::vector<int> &faceLabels, int currentLabel);
};

using namespace std;
using namespace cv;

// TODO
const string pkg_path = ros::package::getPath("corobot_face_recognition");
const string face_database_path = pkg_path + "/face_database";
const string csv_file = pkg_path + "/data/training_set.csv";
const string script_file = pkg_path + "/scripts/create_csv.py";

const string eigenfacesModel_file = pkg_path + "/data/eigenfacesModel.xml";
const string fisherfacesModel_file = pkg_path + "/data/fisherfacesModel.xml";
const string lbphModel_file = pkg_path + "/data/lbphModel.xml";

// Grayscale
const string eigenfacesGrayscaleModel_file = pkg_path + "/data/eigenfacesGrayscaleModel.xml";
const string fisherfacesGrayscaleModel_file = pkg_path + "/data/fisherfacesGrayscaleModel.xml";
const string lbphGrayscaleModel_file = pkg_path + "/data/lbphGrayscaleModel.xml";
// Histeq
const string eigenfacesHisteqModel_file = pkg_path + "/data/eigenfacesHisteqModel.xml";
const string fisherfacesHisteqModel_file = pkg_path + "/data/fisherfacesHisteqModel.xml";
const string lbphHisteqModel_file = pkg_path + "/data/lbphHisteqModel.xml";
// Avg4
const string eigenfacesAvg4Model_file = pkg_path + "/data/eigenfacesAvg4Model.xml";
const string fisherfacesAvg4Model_file = pkg_path + "/data/fisherfacesAvg4Model.xml";
const string lbphAvg4Model_file = pkg_path + "/data/lbphAvg4Model.xml";
// TanTriggs
const string eigenfacesTantriggsModel_file = pkg_path + "/data/eigenfacesTantriggsModel.xml";
const string fisherfacesTantriggsModel_file = pkg_path + "/data/fisherfacesTantriggsModel.xml";
const string lbphTantriggsModel_file = pkg_path + "/data/lbphTantriggsModel.xml";


FaceRec::FaceRec(bool train, string preprocessing) 
: _eigenfacesModel( createEigenFaceRecognizer() )
, _fisherfacesModel( createFisherFaceRecognizer() )
, _lbphModel( createLBPHFaceRecognizer() )
, _preprocessing(preprocessing)
{
	// if (train) 
	// 	generateCsv();

	// loadDataFromCsv();
	readTrainingDb(face_database_path);

	if (train)
		trainModelsAndSave();
	else
		loadModels();
}

FaceRec::~FaceRec() {
	
}

void FaceRec::train(string preprocessing) {
	_preprocessing = preprocessing;
	readTrainingDb(face_database_path);
	trainModelsAndSave(false);
	loadModels();
}

int FaceRec::generateCsv() {
	string cmd = "python " + script_file + " " + face_database_path + " " + csv_file;
	return system(cmd.c_str());
}

void FaceRec::loadDataFromCsv() {
	ifstream file(csv_file.c_str(), ifstream::in);
    if (!file) {
        // cout << "No valid input file was given, please check the given filename." << endl;
        ROS_ERROR ("No valid input file was given, please check the given filename.");
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

    /*
    cout << "############################" << endl << endl;

    cout << "_faceImgs:" << endl;
    cout << _faceImgs.size() << endl;
    // for (int i = 0; i < _faceImgs.size(); ++i) {
    // 	// cout << "\t" << _faceImgs[i] << endl;
    // 	cout << "\t" << "[]" << endl;
    // }
    cout << endl;

    cout << "_faceLabels:" << endl;
    cout << _faceLabels.size() << endl;
    // for (int i = 0; i < _faceLabels.size(); ++i) {
    // 	cout << "\t" << _faceLabels[i] << endl;
    // }
    cout << endl;

    cout << "_faceNames:" << endl;
    for (map<int,string>::iterator it=_faceNames.begin(); it!=_faceNames.end(); ++it)
	    cout << it->first << " => " << it->second << endl;

	cout << endl;
    cout << "############################" << endl;
    */
}


void FaceRec::readimgs(string dirpath, vector<Mat> &faceImgs,
		vector<int> &faceLabels, int currentLabel)
{
	DIR* dir_point = opendir(dirpath.c_str());
	dirent* entry = readdir(dir_point);
	string fname, fpath;
	// if !entry then end of directory
	while (entry) {
		// if entry is a regular file
		if (entry->d_type == DT_REG) {
			// filename
			fname = entry->d_name;
			fpath = dirpath + "/" + fname;
			if (fname.at(0) != '.') {
				// cout << dirpath + "/" + fname << endl;
				faceLabels.push_back(currentLabel);
				faceImgs.push_back(imread(fpath, 0));
			}
		}
		entry = readdir(dir_point);
	}
	return;
}


void FaceRec::readTrainingDb(string dbpath) {
	ROS_INFO("Reading images from %s.", face_database_path.c_str());

	_faceImgs.clear();
	_faceLabels.clear();
	_faceNames.clear();
	_grayscaleFaceImgs.clear();
	_grayscaleFaceLabels.clear();
	_histeqFaceImgs.clear();
	_histeqFaceLabels.clear();
	_avg4FaceImgs.clear();
	_avg4FaceLabels.clear();
	_tantriggsFaceImgs.clear();
	_tantriggsFaceLabels.clear();

	int currentLabel = -1;

	DIR* dir_point = opendir(dbpath.c_str());
	dirent* entry = readdir(dir_point);
	
	// if !entry then end of directory
	while (entry) {	
		// if entry is a directory				
		if (entry->d_type == DT_DIR) {
			string sub_name = entry->d_name;
			currentLabel++;
			if (sub_name != "." && sub_name != "..") {
				_faceNames[currentLabel] = sub_name;
				string sub_path = dbpath + "/" + sub_name;
				DIR* sub_dir_point = opendir(sub_path.c_str());
				dirent* sub_entry = readdir(sub_dir_point);
				while (sub_entry) {
					if (sub_entry->d_type == DT_DIR) {
						string sub_sub_name = sub_entry->d_name;
						
						if (sub_sub_name == "grayscale") {
							readimgs(dbpath + "/" + sub_name + "/" + sub_sub_name,
								_grayscaleFaceImgs, _grayscaleFaceLabels, currentLabel);
						}

						else if (sub_sub_name == "histeq") {
							readimgs(dbpath + "/" + sub_name + "/" + sub_sub_name,
								_histeqFaceImgs, _histeqFaceLabels, currentLabel);
						}
						
						else if (sub_sub_name == "avg4") {
							readimgs(dbpath + "/" + sub_name + "/" + sub_sub_name,
								_avg4FaceImgs, _avg4FaceLabels, currentLabel);
						}

						else if (sub_sub_name == "tantriggs") {
							readimgs(dbpath + "/" + sub_name + "/" + sub_sub_name,
								_tantriggsFaceImgs, _tantriggsFaceLabels, currentLabel);
						}
						
					}
					sub_entry = readdir(sub_dir_point);
				}
			}
		}
		entry = readdir(dir_point);
	}

	ROS_INFO("Reading images completed");

	string subjectsList = "";
	for (map<int,string>::iterator it=_faceNames.begin(); it!=_faceNames.end(); ++it){
	    subjectsList += it->second;
	    subjectsList += ", ";
	}

	ROS_INFO("%lu Subjects: %s", _faceNames.size(), subjectsList.c_str());

	return;
}


void FaceRec::trainModelsAndSave(bool trainAll) {
	ROS_INFO("Training started.");
	
	if (trainAll || _preprocessing == "grayscale") {
		_eigenfacesModel->train(_grayscaleFaceImgs, _grayscaleFaceLabels);
		_fisherfacesModel->train(_grayscaleFaceImgs, _grayscaleFaceLabels);
		_lbphModel->train(_grayscaleFaceImgs, _grayscaleFaceLabels);

		_eigenfacesModel->save(eigenfacesGrayscaleModel_file);
		_fisherfacesModel->save(fisherfacesGrayscaleModel_file);
		_lbphModel->save(lbphGrayscaleModel_file);
	}

	else if (trainAll || _preprocessing == "histeq") {
		_eigenfacesModel->train(_histeqFaceImgs, _histeqFaceLabels);
		_fisherfacesModel->train(_histeqFaceImgs, _histeqFaceLabels);
		_lbphModel->train(_histeqFaceImgs, _histeqFaceLabels);

		_eigenfacesModel->save(eigenfacesHisteqModel_file);
		_fisherfacesModel->save(fisherfacesHisteqModel_file);
		_lbphModel->save(lbphHisteqModel_file);
	}

	else if (trainAll || _preprocessing == "avg4") {
		_eigenfacesModel->train(_avg4FaceImgs, _avg4FaceLabels);
		_fisherfacesModel->train(_avg4FaceImgs, _avg4FaceLabels);
		_lbphModel->train(_avg4FaceImgs, _avg4FaceLabels);

		_eigenfacesModel->save(eigenfacesAvg4Model_file);
		_fisherfacesModel->save(fisherfacesAvg4Model_file);
		_lbphModel->save(lbphAvg4Model_file);
	}

	else if (trainAll || _preprocessing == "tantriggs") {
		_eigenfacesModel->train(_tantriggsFaceImgs, _tantriggsFaceLabels);
		_fisherfacesModel->train(_tantriggsFaceImgs, _tantriggsFaceLabels);
		_lbphModel->train(_tantriggsFaceImgs, _tantriggsFaceLabels);

		_eigenfacesModel->save(eigenfacesTantriggsModel_file);
		_fisherfacesModel->save(fisherfacesTantriggsModel_file);
		_lbphModel->save(lbphTantriggsModel_file);
	}
	ROS_INFO("Training complete.");

}

void FaceRec::loadModels() {
	if (_preprocessing == "grayscale") {
		_eigenfacesModel->load(eigenfacesGrayscaleModel_file);
		_fisherfacesModel->load(fisherfacesGrayscaleModel_file);
		_lbphModel->load(lbphGrayscaleModel_file);
	}

	else if (_preprocessing == "histeq") {
		_eigenfacesModel->load(eigenfacesHisteqModel_file);
		_fisherfacesModel->load(fisherfacesHisteqModel_file);
		_lbphModel->load(lbphHisteqModel_file);
	}

	else if (_preprocessing == "avg4") {
		_eigenfacesModel->load(eigenfacesAvg4Model_file);
		_fisherfacesModel->load(fisherfacesAvg4Model_file);
		_lbphModel->load(lbphAvg4Model_file);
	}

	else if (_preprocessing == "tantriggs") {
		_eigenfacesModel->load(eigenfacesTantriggsModel_file);
		_fisherfacesModel->load(fisherfacesTantriggsModel_file);
		_lbphModel->load(lbphTantriggsModel_file);
	}
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


