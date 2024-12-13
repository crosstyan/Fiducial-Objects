#ifndef _SVGREADER_
#define _SVGREADER_

#include <iostream>
#include <tinyxml2.h>
#include <map>
#include <set>
#include <vector>
#include "parser.h"
#include "../cucomarker.h"

namespace cucomarker {

class SVGFile{
public:
    //path to svg
    //nCrcBits number of crc bits
    //returns maximum id
    uint64_t load(const std::string path,int nCrcBits);
    void save(const int markerId, const std::string path);
    void exportConfigFile(const std::string path);


    inline std::map<int,tinyxml2::XMLElement *> getBitsInfo(){return bit_info;};
    inline std::map<int,tinyxml2::XMLElement *> getBitsCRC(){return bit_crc;};
    inline std::map<int,tinyxml2::XMLElement *> getMarker(){return marker;};
    inline CRC getCRCPoly(){return polycrc;};

    MarkerDetector createDetector();

    int getTotalBits()const{return bit_info.size()+ bit_crc.size();}
private:
    std::map<int,tinyxml2::XMLElement *> bit_info ;
    std::map<int,tinyxml2::XMLElement *> bit_crc;
    std::map<int, tinyxml2::XMLElement *> marker;




    tinyxml2::XMLDocument doc;
    cucomarker::CRC polycrc;
    std::vector<std::string> colors;
};
}
#endif
