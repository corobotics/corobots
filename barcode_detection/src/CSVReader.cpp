#include "../include/CSVReader.h"



void CSVReader::init()
{

    infile.open("src/barcodePoints.csv");


}

void CSVReader::close()
{

    infile.close();

}

void CSVReader::readFile()
{
    data.clear();
	
    while (infile) {
	string s;
	if (!getline(infile, s))
	    break;

	istringstream ss(s);
	vector < string > record;

	while (ss) {
	    string s;
	    if (!getline(ss, s, ','))
		break;
	    record.push_back(s);
	}

	data.push_back(record);
    }
    if (!infile.eof()) {
	cerr << "Error!!!";
    }
}

string CSVReader::getX(string url)
{
    string temp;
    for (int i = 1; i < data.size(); i++) {
	temp = data[i][6];
	if (url.compare(temp) == 0)
	    return data[i][1];
    }
    return "";
}

string CSVReader::getY(string url)
{
    string temp;
    for (int i = 1; i < data.size(); i++) {
	temp = data[i][6];
	if (url.compare(temp) == 0)
	    return data[i][2];
    }
    return "";
}

string CSVReader::getOrientation(string url)
{
    string temp;
    for (int i = 1; i < data.size(); i++) {
	temp = data[i][6];
	if (url.compare(temp) == 0)
	    return data[i][5];
    }
    return "";
}

