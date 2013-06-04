#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "CSVReader.h"

using namespace std;

void CSVReader::init(string filename)
{
  infile.open(filename.c_str());
  readFile();
  infile.close();
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
    url.erase(url.find_last_not_of(" \n\r\t")+1);
    string temp;
    for (unsigned int i = 1; i < data.size(); i++) {
        temp = data[i][6];
        if (url.compare(temp) == 0)
            return data[i][3];
    }
    return "";
}

string CSVReader::getY(string url)
{
    url.erase(url.find_last_not_of(" \n\r\t")+1);
    string temp;
    for (unsigned int i = 1; i < data.size(); i++) {
        temp = data[i][6];
        if (url.compare(temp) == 0)
            return data[i][4];
    }
    return "";
}

string CSVReader::getOrientation(string url)
{
    url.erase(url.find_last_not_of(" \n\r\t")+1);
    string temp;
    for (unsigned int i = 1; i < data.size(); i++) {
        temp = data[i][6];
        if (url.compare(temp) == 0)
            return data[i][5];
    }
    return "";
}

