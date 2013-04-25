#ifndef CSVReader_H
#define CSVReader_H

#include <fstream>
#include <string>
#include <vector>

class CSVReader {
public:

    std::vector < std::vector < std::string > > data;
    void init();
    void close();
    void readFile();
    std::string getX(std::string url);
    std::string getY(std::string url);
    std::string getOrientation(std::string url);

private:

    std::ifstream infile;

};

#endif

