#include "svgreader.h"

namespace cucomarker {

void changeColor(tinyxml2::XMLElement *node, std::string color)
{
    const tinyxml2::XMLAttribute *style = node->FindAttribute("style");
    std::string value;
    if(style)
    {
        value=style->Value();
        size_t pos = value.find("fill:");
        value.replace(pos+5, 7, "#"+color);
    }
    else
        value="fill:#"+color;

    node->SetAttribute("style",value.c_str());
}
void recursiveSearch2(tinyxml2::XMLNode *node, std::vector<tinyxml2::XMLElement *> &marker, std::vector<tinyxml2::XMLElement*> &bits){

    for(tinyxml2::XMLElement* child = node->FirstChildElement(); child != NULL; child = child->NextSiblingElement())
    {
        const char *att =child->Attribute("id");
        if(att)
        {
            std::string id = att;
            if(id.find("marker") != std::string::npos)
                marker.push_back(child);
            else  if(id.find("bit") != std::string::npos)
                bits.push_back(child);
            recursiveSearch2(child, marker, bits);
        }
    }
}

void recursiveSearch(tinyxml2::XMLNode *node, std::map<int,tinyxml2::XMLElement *> &marker, std::map<int, tinyxml2::XMLElement*> &bit_info, std::map<int,tinyxml2::XMLElement*> &bit_crc){


    for(tinyxml2::XMLElement* child = node->FirstChildElement(); child != NULL; child = child->NextSiblingElement())
    {

        const char *att =child->Attribute("id");
        if(att)
        {
            std::string id = att;
            char type[100];
            int nbit=0;

            if(id.find(":") != std::string::npos)
            {
                std::replace(id.begin(), id.end(), ':', ' ');
                sscanf(id.c_str(), "%s %d", type, &nbit);
            }
            else
                strcpy(type, id.c_str());

            if(strcmp(type, "marker") == 0)
                marker.insert(std::pair<int, tinyxml2::XMLElement *> (nbit, child));

            else if (strcmp(type, "bit-info") == 0)
                bit_info.insert(std::pair<int, tinyxml2::XMLElement *> (nbit, child));

            else if (strcmp(type, "bit-crc") == 0)
                bit_crc.insert(std::pair<int, tinyxml2::XMLElement *> (nbit, child));

        }
        recursiveSearch(child, marker, bit_info, bit_crc);
    }
}



uint64_t SVGFile::load(const std::string path,int ncrcBits)
{
    //rms
     std::vector<tinyxml2::XMLElement *> bits ;
    std::vector<tinyxml2::XMLElement *> markercontour;
    std::set<std::string> xcolors;

    doc.LoadFile(path.c_str());
    tinyxml2::XMLNode *root= doc.FirstChildElement("svg");
    recursiveSearch2(root, markercontour, bits);
    if(markercontour.size()!=1)
        throw std::invalid_argument( "markers not found or more than one" );
    marker.insert({0,markercontour[0]});
    if(bits.size()==0)
        throw std::invalid_argument( "no bits found" );

    //now, check if the number of databits is ok
    if(ncrcBits!=-1){
         if(ncrcBits!=8 && ncrcBits!=16 && ncrcBits!=32)
            throw std::invalid_argument( "Invalid number of data bits given the total number of bits: #bits="+std::to_string(bits.size()) );
    }
    else {//auto select
        ncrcBits=bits.size()/2;
        if(ncrcBits<=8)ncrcBits=8;
        else  if(ncrcBits<=16) ncrcBits=16;
        else  if(ncrcBits<=32) ncrcBits=32;

    }

     int ndatabits=bits.size()-ncrcBits;
     if(ndatabits>64)
         throw std::invalid_argument( "Too many data bits >64" );


    //read all colors
    for(auto b:bits){
        const tinyxml2::XMLAttribute *style = b->FindAttribute("style");
        if(style)
        {
            std::string value;
            value=style->Value();
            size_t pos = value.find("fill:#");
            xcolors.insert(value.substr(pos+6,6));
        }
        else
        {
            const tinyxml2::XMLAttribute *fill = b->FindAttribute("fill");
            if(fill)
            {
                std::string value;
                value=fill->Value();
                size_t pos = value.find("#");
                xcolors.insert(value.substr(pos+1, 6));
            }
        }

    }
    if(xcolors.size()!=2) throw std::invalid_argument( "number of different bit colors!=2" );

    //add the colors
    for(auto &c:xcolors) colors.push_back(c);

    //Assign bits to the corresponding set in order

    if( ncrcBits>ndatabits){//more crc than data
        int step=ncrcBits/ndatabits;
        size_t index=0,d_index=0,crc_index=0;
        for(int i=0;i<ndatabits;i++){
            bit_info.insert({d_index++,bits[index++]});
            for(int j=0;j<step;j++)
                bit_crc.insert({crc_index++,bits[index++]});
        }
        //now, add the remaining ones
        for(;index<bits.size();index++)
            bit_crc.insert({crc_index++,bits[index]});
    }
    else{//more data than crc, not recommended
        int step=ndatabits/ncrcBits;
        size_t index=0,d_index=0,crc_index=0;
        for(int i=0;i<ncrcBits;i++){
            bit_crc.insert({crc_index++,bits[index++]});
            for(int j=0;j<step;j++)
                bit_info.insert({d_index++,bits[index++]});
        }
        //now, add the remaining ones
        for(;index<bits.size();index++)
            bit_info.insert({d_index++,bits[index]});
    }


    if(bit_crc.size()==8) polycrc = cucomarker::CRC8;
    else if(bit_crc.size()==16) polycrc = cucomarker::CRC16;
    else if(bit_crc.size()==32) polycrc = cucomarker::CRC32;
    else throw std::invalid_argument( "Incorrect number of bit for crc (8, 16 or 32).");
    uint64_t lastId=0;
    for(int i=0;i<ndatabits;i++){
        lastId=lastId<<1;
        lastId|=1;
    }
    return lastId;
}

void SVGFile::save(const int markerId, const std::string outPath)
{
    std::string crc;
    switch(polycrc)
    {
    case 8:
        crc= std::bitset<cucomarker::CRC8>(cucomarker::_private::crc_8(markerId)).to_string();
        break;
    case 16:
        crc= std::bitset<cucomarker::CRC16>(cucomarker::_private::crc_16(markerId)).to_string();
        break;
    case 32:
        crc= std::bitset<cucomarker::CRC32>(cucomarker::_private::crc_32(markerId)).to_string();
        break;
    default:
        break;
    }

    std::string id_bin=cucomarker::_private::toBinary(markerId); //Marker id (bin)
    if(id_bin.size()>bit_info.size())
        throw std::invalid_argument( "The markerid size is higher than number of bits for info");


    while(id_bin.size()<bit_info.size())
        id_bin='0'+id_bin;

    //Marker info code
    for(size_t i=0; i<bit_info.size(); i++)
    {
        if(bit_info.find(i)==bit_info.end())
            throw std::invalid_argument( "bit_info "+std::to_string(i)+" no found" );

        std::string color;
        if(id_bin[id_bin.size()-1-i] == '0')
            color=colors[0];
        else
            color=colors[1];

        changeColor(bit_info.at(i), color);
    }

    //Checksum code
    for(size_t i=0; i<bit_crc.size(); i++)
    {
        if(bit_crc.find(i)==bit_crc.end())
            throw std::invalid_argument( "bit_crc "+std::to_string(i)+" no found" );

        std::string color;
        if(crc[crc.size()-1-i] == '0')
            color=colors[0];
        else
            color=colors[1];

        changeColor(bit_crc.at(i), color);
    }

    doc.SaveFile(outPath.c_str() );
}

void SVGFile::exportConfigFile(const std::string path)
{

    auto getAttribute=[](tinyxml2::XMLElement *elm,std::string att){
       auto *ptr= elm->Attribute(att.c_str());
       if(ptr!=nullptr) return std::string(ptr);
       else return std::string();
    };

    auto parseAsVector=[](tinyxml2::XMLElement *elm){

        std::string value(elm->Value());
        if(value=="rect"){
            return parseRect(elm->Attribute("width"),elm->Attribute("height"),elm->Attribute("x"),elm->Attribute("y"));
        }
        else if( value=="path")
            return parsePath(elm->Attribute("d"));
        else throw std::runtime_error("Invalid elem type:"+value);
    };
    auto getCenter=[](tinyxml2::XMLElement *elm){

        std::string value(elm->Value());
        if(value=="rect"){
            return getRectCenter(elm->Attribute("width"),elm->Attribute("height"),elm->Attribute("x"),elm->Attribute("y"));
        }
        else if( value=="path")
            return getPathCenter(elm->Attribute("d"));

        else if(value=="ellipse"){
            float cx=std::stof(elm->Attribute("cx"));
            float cy=std::stof(elm->Attribute("cy"));
            return cv::Point2f{cx,cy};
        }

        else throw std::runtime_error("Invalid elem type:"+value);
    };

    cv::FileStorage fs(path, cv::FileStorage::WRITE);

    Parameters _params = MarkerDetector().getParams();

    fs << "params"<< "{";
    fs << "minMarkerSize" << _params.minMarkerSize;
    fs << "thrGray" << _params.thrGray;
    fs << "thrHsv" << _params.thrHsv;
    fs << "border" << _params.border;
    fs <<"}";


    Marker marker = Marker(parseAsVector(   getMarker().begin()->second));
    //Sort corners anti-clockwise order
    _private::anticlockwise(marker);

    fs << "marker"<< "{";
    fs << "corners"<<"[:";
    for(auto corner:marker) fs << corner;
    fs << "]";
    fs << "colors"<<"[:"<<"#"+colors[0]<<"#"+colors[1]<<"]";
    fs <<"}";


    fs << "bit-info"<< "[";
    std::map<int,tinyxml2::XMLElement *> _bit_info = getBitsInfo();
    for(int i=_bit_info.size()-1; i>=0; i--)
    {
        if(_bit_info.find(i) != _bit_info.end())
        {
            cv::Point2f center=getCenter(_bit_info.at(i));
            fs << "{:"<< "id"<<i<<"center"<<center<<"}";
        }
        else
            throw std::runtime_error( "error bit-info svg file!!!" );
    }
    fs << "]";


    fs << "bit-crc"<< "[";
    std::map<int,tinyxml2::XMLElement *> _bit_crc = getBitsCRC();
    for(int i=_bit_crc.size()-1; i>=0; i--)
    {
        if(_bit_crc.find(i) != _bit_crc.end())
        {
            cv::Point2f center=getCenter(_bit_crc.at(i));
            fs << "{:"<< "id"<<i<<"center"<<center<<"}";
        }
        else
            throw std::runtime_error( "error bit-info svg file!!!" );
    }
    fs << "]";

    fs.release();

}

MarkerDetector SVGFile::createDetector()
{
    MarkerDetector detector;
    if(doc.NoChildren())
    {
        throw std::runtime_error( "error to load SVG file!!!" );
        return detector;
    }

    Marker TheMarker = Marker(parsePath(getMarker().begin()->second->Attribute("d")));
    TheMarker.polycrc = getCRCPoly();

    std::map<int,tinyxml2::XMLElement *> _bit_info = getBitsInfo();
    std::map<int,tinyxml2::XMLElement *> _bit_crc = getBitsCRC();

    for(int i=_bit_info.size()-1; i>=0; i--)
    {
        if(_bit_info.find(i) != _bit_info.end())
        {
            std::string d = _bit_info.at(i)->Attribute("d");
            std::vector<cv::Point2f> points=parsePath(d);
            cv::Point2f center(0,0);
            for(auto pt:points) center+=pt;
            center/=(int)points.size();
            TheMarker.bits.push_back(center);
        }
        else
            throw std::runtime_error( "error bit-info svg file!!!" );
    }

    for(int i=_bit_crc.size()-1; i>=0; i--)
    {
        if(_bit_crc.find(i) != _bit_crc.end())
        {
            std::string d = _bit_crc.at(i)->Attribute("d");
            std::vector<cv::Point2f> points=parsePath(d);
            cv::Point2f center(0,0);
            for(auto pt:points) center+=pt;
            center/=(int)points.size();
            TheMarker.bits.push_back(center);
        }
        else
        throw std::runtime_error( "error bit-crc svg file!!!" );
    }

    detector.setMarker(TheMarker);


    return detector;
}
}
