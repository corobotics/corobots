#ifndef CSVReader_H
#define CSVReader_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace std;

class CSVReader {

    ifstream infile;

  public:

    vector < vector < string > >data;
    void init();
    void close();
    void readFile();
    string getX(string url);
    string getY(string url);
};

#endif

