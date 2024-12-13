
#ifndef _CUCOMARKER_H_
#define _CUCOMARKER_H_

#include <iostream>
#include <bitset>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace cucomarker {

#define CRC_START_8 0x00
#define	CRC_START_16 0x0000
#define	CRC_START_32 0xFFFFFFFFul

#define CRC_POLY_16 0xA001
#define CRC_POLY_32 0xEDB88320ul

enum CRC{CRC8=8,CRC16=16,CRC32=32};
enum Color{ALL, GRAY, HSV};




/**
 * @brief Contains fundamental definitions used
 *        in the project
 */
namespace _private {

//////CRC
///
static bool crc_tab16_init = false;
static bool crc_tab32_init = false;

static uint16_t crc_tab16[256];
static uint64_t crc_tab32[256];

static uint8_t sht75_crc_table[] = {
    0,   49,  98,  83,  196, 245, 166, 151, 185, 136, 219, 234, 125, 76,  31,  46,
    67,  114, 33,  16,  135, 182, 229, 212, 250, 203, 152, 169, 62,  15,  92,  109,
    134, 183, 228, 213, 66,  115, 32,  17,  63,  14,  93,  108, 251, 202, 153, 168,
    197, 244, 167, 150, 1,   48,  99,  82,  124, 77,  30,  47,  184, 137, 218, 235,
    61,  12,  95,  110, 249, 200, 155, 170, 132, 181, 230, 215, 64,  113, 34,  19,
    126, 79,  28,  45,  186, 139, 216, 233, 199, 246, 165, 148, 3,   50,  97,  80,
    187, 138, 217, 232, 127, 78,  29,  44,  2,   51,  96,  81,  198, 247, 164, 149,
    248, 201, 154, 171, 60,  13,  94,  111, 65,  112, 35,  18,  133, 180, 231, 214,
    122, 75,  24,  41,  190, 143, 220, 237, 195, 242, 161, 144, 7,   54,  101, 84,
    57,  8,   91,  106, 253, 204, 159, 174, 128, 177, 226, 211, 68,  117, 38,  23,
    252, 205, 158, 175, 56,  9,   90,  107, 69,  116, 39,  22,  129, 176, 227, 210,
    191, 142, 221, 236, 123, 74,  25,  40,  6,   55,  100, 85,  194, 243, 160, 145,
    71,  118, 37,  20,  131, 178, 225, 208, 254, 207, 156, 173, 58,  11,  88,  105,
    4,   53,  102, 87,  192, 241, 162, 147, 189, 140, 223, 238, 121, 72,  27,  42,
    193, 240, 163, 146, 5,   52,  103, 86,  120, 73,  26,  43,  188, 141, 222, 239,
    130, 179, 224, 209, 70,  119, 36,  21,  59,  10,  89,  104, 255, 206, 157, 172
};

inline uint8_t crc_8( uint8_t num  ) {

    char hexval[1];
    hexval[0] = num ;

    uint8_t crc = CRC_START_8;
    const unsigned char *str=(const unsigned char *)hexval;

    if ( str != NULL )
        crc = sht75_crc_table[(*str++) ^ crc];

    return crc;
}

inline static void init_crc16_tab( void ){
    uint16_t i, j, crc, c;

    for (i=0; i<256; i++) {
        crc = 0;
        c = i;

        for (j=0; j<8; j++) {
            if ((crc ^ c) & 0x0001)
                crc = ( crc >> 1 ) ^ CRC_POLY_16;
            else
                crc =   crc >> 1;
            c = c >> 1;
        }
        crc_tab16[i] = crc;
    }
    crc_tab16_init = true;
}

inline uint16_t crc_16(uint16_t num ) {

    //Convert num to hex
    char hexval[2];
    for(auto &h:hexval) h=0;

    size_t j=0;
    hexval[j] = num >> 8;
    if(hexval[j] != 0x00) j++;
    hexval[j] = num & 0xff;


    const unsigned char *str=(const unsigned char *)hexval;
    size_t nbytes=strlen((char*)str);
    uint16_t crc;

    if ( ! crc_tab16_init ) init_crc16_tab();

    crc = CRC_START_16;
    str = (const unsigned char *)hexval;

    if ( str != NULL ) for (size_t a=0; a<nbytes; a++)
        crc = (crc >> 8) ^ crc_tab16[ (crc ^ (uint16_t) *str++) & 0x00FF ];

    return crc;
}

inline void init_crc32_tab( void ) {

    uint32_t i, j, crc;

    for (i=0; i<256; i++) {
        crc = i;

        for (j=0; j<8; j++) {

            if ( crc & 0x00000001L ) crc = ( crc >> 1 ) ^ CRC_POLY_32;
            else                     crc =   crc >> 1;
        }

        crc_tab32[i] = crc;
    }
}

inline uint32_t crc_32( uint32_t num ) {

    //Convert num to hex
    char hexval[4];
    for(auto &h:hexval) h=0;

    size_t j=0;
    for(size_t i=3; i>=1; i--)
    {
        hexval[j] = num >> 8*i;
        if(hexval[j] != 0x00) j++;
    }
    hexval[j] = num & 0xff;


    const unsigned char *str=(const unsigned char *)hexval;
    size_t nbytes=strlen((char*)str);
    uint32_t crc;

    if ( ! crc_tab32_init ) init_crc32_tab();

    crc = CRC_START_32;

    if ( str != NULL ) for (size_t a=0; a<nbytes; a++) {
        crc = (crc >> 8) ^ crc_tab32[ (crc ^ (uint32_t) *str++) & 0x000000FFul ];
    }

    return (crc ^ 0xFFFFFFFFul);
}


inline float variance(std::vector<int> values, float mean, int start, int end)
{
    float var=0, n=0;

    for(auto v:values)
    {
        if(v>=start && v<end)
        {
            var+=(v - mean)*(v - mean);
            n++;
        }
    }

    return var/n;
}

inline float average(std::vector<int> values, int start, int end)
{
    float avg=0, n=0;

    for(auto v:values)
    {
        if(v>=start && v<end)
        {
            avg+=v;
            n++;
        }
    }

    return avg/n;
}

inline void rotate(std::vector<int> &values, int inc)
{
    for(auto &c:values) {
        c=(c+inc);
        if(c>=180)c=c-180;
    }
}

inline std::pair<int,float> circularAverage(std::vector<int> values)
{
    int rotation=0;
    std::pair<int,float> best(-1,std::numeric_limits<float>::lowest());

    do{
        float SigmaExtra=_private::variance(values, 90, 0, 180);
        float Avrg1=_private::average(values,0,90);
        float Sigma1Intra=_private::variance(values, Avrg1, 0, 90);
        float Avrg2=_private::average(values,90,180);
        float Sigma2Intra=_private::variance(values, Avrg2, 90, 180);

        if(Sigma1Intra<1e-4)Sigma1Intra=1e10;
        if(Sigma2Intra<1e-4)Sigma2Intra=1e10;

        float Goodness = SigmaExtra/(Sigma1Intra+Sigma2Intra);
        if(std::isnan(Goodness) || std::isinf(Goodness))
            Goodness=0;

        if(Goodness > best.second)
            best={rotation,Goodness};

        rotation++;
        rotate(values, 1);

    }while(rotation<180);

    return best;
}

inline bool anticlockwise(std::vector<cv::Point2f>& points)
{
    double dx1 = points[1].x - points[0].x;
    double dy1 = points[1].y - points[0].y;
    double dx2 = points[2].x - points[0].x;
    double dy2 = points[2].y - points[0].y;
    double o = (dx1 * dy2) - (dy1 * dx2);

    if (o < 0.0){
        std::reverse(points.begin(), points.end());
        return true;
    }
    else return false;
}

inline std::string toBinary(int n)
{
    std::string r;
    while(n!=0) {r=(n%2==0 ?"0":"1")+r; n/=2;}
    return r;
}

inline int binaryToDecimal(int n)
{
    int num = n;
    int dec_value = 0;

    // Initializing base value to 1, i.e 2^0
    int base = 1;

    int temp = num;
    while (temp) {
        int last_digit = temp % 10;
        temp = temp / 10;
        dec_value += last_digit * base;
        base = base * 2;
    }

    return dec_value;
}

inline std::pair<float, float> grayAnalysis(cv::Mat pxGray)
{
    std::vector<int> values;
    for(int i=0; i<pxGray.cols; i++)
        values.push_back(int(pxGray.at<uchar>(0,i)));

    cv::Mat img_bw;
//    float grayAvg=cv::threshold(pxGray, img_bw, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    //Calc average gray pixels
    float grayAvg=_private::average(values,0,256);
    float grayVar=_private::variance(values,grayAvg, 0,256);

    //Calc average/var two groups (upper/below avg)
    float avgG1 = _private::average(values, 0, ceil(grayAvg));
    float avgG2 = _private::average(values, ceil(grayAvg), 256);
    float varG1 = _private::variance(values,avgG1,0,ceil(grayAvg));
    float varG2 = _private::variance(values,avgG2,ceil(grayAvg),256);


//    std::cout <<"Global ("<<grayAvg<<", "<<grayVar<<"), ";
//    std::cout <<"G1 ("<<avgG1<<", "<<varG1<<"), ";
//    std::cout <<"G2 ("<<varG2<<","<<varG2<<"), ";
//    std::cout <<"Sigma ("<<grayVar/(varG1+varG2)<<")"<<std::endl;

    return std::make_pair(grayAvg, grayVar/(varG1+varG2));
}
}
//end _private namespace




/**
 * @brief Class made in order to manage the parameters
 *        in the project
 */
class Parameters
{
public:
    /* Parameters */
    bool border=false; //analyze border
    float thrGray=10; //gray threshold
    float thrHsv=800; //hsv threshold
    float minMarkerSize=0.0005;//min marker area
    Color colorDetectionMode=ALL; //Color space which the marker is detected
    /* Copy */
    inline void copyToParameters(Parameters &params) const;
    /* Stream functions */
    inline void toStream(std::ostream &str)const;
    inline void fromStream(std::istream &str);
};

/* Copy */
    void  Parameters::copyToParameters(Parameters &p)const{
    p.border = this->border;
    p.thrGray = this->thrGray;
    p.thrHsv = this->thrHsv;
    p.minMarkerSize = this->minMarkerSize;
    p.colorDetectionMode = this->colorDetectionMode;
}

/* Stream functions */
    // toStream
    void Parameters::toStream(std::ostream &str)const{
        str.write((char*)&border,sizeof(border));
        str.write((char*)&thrGray,sizeof(thrGray));
        str.write((char*)&thrHsv,sizeof(thrHsv));
        str.write((char*)&minMarkerSize,sizeof(minMarkerSize));
        str.write((char*)&colorDetectionMode,sizeof(colorDetectionMode));
    }
    // fromStream
    void Parameters::fromStream(std::istream &str){
        str.read((char*)&border,sizeof(border));
        str.read((char*)&thrGray,sizeof(thrGray));
        str.read((char*)&thrHsv,sizeof(thrHsv));
        str.read((char*)&minMarkerSize,sizeof(minMarkerSize));
        str.read((char*)&colorDetectionMode,sizeof(colorDetectionMode));
    }







/**
 * @brief Class made in order to manage easily the information
 *        related to each different detected marker
 */
class Marker : public std::vector<cv::Point2f>{
  friend class MarkerDetector;
public:
    /* Constructors */
    inline Marker(){};
    inline Marker(const std::vector<cv::Point2f>& corners);
    inline Marker(const std::vector<cv::Point3f>& corners3d);
    /* Copy */
    inline void copyToMarker(Marker &marker) const;
    /* Getters */
    inline size_t nBitsInfo(){return bits.size()-polycrc;}
    inline cv::Point2f getCenter();
    inline std::vector<cv::Point3f> get3DCorners(float size);
    inline float getArea() const;
    inline int getConfigurationSchema(){return this->_configurationSchema;};
    /* Setters */
    inline void setConfigurationSchema(int configurationSchema){this->_configurationSchema = configurationSchema;};
    /* Info */
    std::vector<cv::Point2f> bits; //info-crc
    CRC polycrc;
    int id=-1;
    std::vector<cv::Point> contours;
    /* Drawing */
    inline void draw(cv::Mat &image, const cv::Scalar color,const bool writeId);
    inline void drawBits(cv::Mat &image);
    /* Stream functions */
    inline void toStream(std::ostream &str)const;
    inline void fromStream(std::istream &str);
private:
    /* Checks */
    inline bool isInto(const cv::Point2f &p); //Check point is into Polygon
    /* Info */
    std::vector<std::string> colors; //bits color (hex)
    std::vector<cv::Point3f> p3d;
    double area=-1;
    int _configurationSchema = -1;
};

/* Constructors */
    Marker::Marker(const std::vector<cv::Point2f>& corners) : std::vector<cv::Point2f>(corners){};
    Marker::Marker(const std::vector<cv::Point3f>& corners){p3d=corners;};

/* Copy */
    void Marker::copyToMarker(Marker &m)const{

    m = Marker();
    m.clear();
    for(auto c: *this){
        m.push_back(c);
    }


    m.bits.clear();
    for(auto b: this->bits){
        m.bits.push_back(b);
    }

    m.polycrc = this->polycrc;
    m.id = this->id;

    m.contours.clear();
    for(auto c: this->contours){
        m.contours.push_back(c);
    }

    m.colors.clear();
    for(auto c: this->colors){
        m.colors.push_back(c);
    }

    m.p3d.clear();
    for(auto p: this->p3d){
        m.p3d.push_back(p);
    }

    m.area = this->area;

}

/* Checks */
    // isInto
    bool Marker::isInto(const cv::Point2f &p)
{
    auto  signD=[](cv::Point2f p0, cv::Point2f p1, cv::Point2f p)
    {
        return  ((p0.y-p1.y)*p.x + (p1.x-p0.x)*p.y + (p0.x*p1.y-p1.x*p0.y)) /  sqrt( (p1.x-p0.x)*(p1.x-p0.x)+(p1.y-p0.y)*(p1.y-p0.y));
    };

    for(uint i=0; i<this->size();i++)
    {
        cv::Point2f a = this->at(i);
        cv::Point2f b = this->at((i+1)%this->size());
        if( signD(a,b,p)<0)return  false;
    }
    return true;
}

/* Getters */
    // getCenter
    cv::Point2f Marker::getCenter()
    {
        cv::Point2f center{0.f,0.f};
        for(auto p:*this) center+=p;
        center/=float(this->size());
        return center;
    }
    // getArea
    float Marker::getArea() const
    {
        return area;
    }
    // get3DCorners
    std::vector<cv::Point3f> Marker::get3DCorners(float meters)
{
    std::vector<cv::Point3f> corners3d;
    float side = sqrt(pow(p3d.at(0).x - p3d.at(1).x, 2) + pow(p3d.at(0).y - p3d.at(1).y, 2));
    float scale = meters/side;

    cv::Point3f center(0.f,0.f,0.f);
    for(auto c:p3d)
    {
        corners3d.push_back(c*scale);
        center+=corners3d.back();
    }

    center.x/=p3d.size();
    center.y/=p3d.size();
    center.z/=p3d.size();

    for(auto &c:corners3d)
        c-=center;

    return corners3d;
}

/* Drawing */
    // draw
    void Marker::draw(cv::Mat &image, const cv::Scalar color=cv::Scalar(255,0,0),const bool writeid=true)
{
    auto _to_string=[](int i){
        std::stringstream str;str<<i;return str.str();
    };

    float flineWidth=  std::max(1.f, std::min(5.f, float(image.cols) / 500.f));
    int lineWidth= round( flineWidth);

    for(size_t j=0; j<(*this).size(); j++)
        cv::line(image, (*this)[j], (*this)[(j+1) % (*this).size()], color, lineWidth);

    auto p2 =  cv::Point2f(2.f * static_cast<float>(lineWidth), 2.f * static_cast<float>(lineWidth));
    cv::rectangle(image, (*this)[0] - p2, (*this)[0] + p2, cv::Scalar(0, 0, 255, 255), -1);
    cv::rectangle(image, (*this)[1] - p2, (*this)[1] + p2, cv::Scalar(0, 255, 0, 255), lineWidth);

    if(writeid)
    {
        float fsize=  std::min(3.0f, flineWidth * 0.75f);
        cv::putText(image,_to_string(id), getCenter()-cv::Point2f(10*flineWidth,0),  cv::FONT_HERSHEY_SIMPLEX,fsize,cv::Scalar(255,255,255)-color, lineWidth,cv::LINE_AA);
    }
}
    // drawBits
    void Marker::drawBits(cv::Mat &image)
{
    std::cout<<"Size bits: "<<this->bits.size()<<std::endl;
    std::cout<<"N bits info: "<<this->nBitsInfo()<<std::endl;
    for(int b_i = 0; b_i < this->bits.size() ; b_i++){
        cv::Point2f b = this->bits[b_i];
        if(b_i < this->nBitsInfo()){
            cv::putText(image,std::to_string(b_i),b,  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
            cv::circle(image, b, 1, cv::Scalar(0,255,0), 2);
        }else{
            cv::putText(image,std::to_string(b_i-this->nBitsInfo()),b,  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
            cv::circle(image, b, 1, cv::Scalar(255,0,0), 2);
        }
    }
}

/* Stream functions */
    // toStream
    void Marker::toStream(std::ostream &str)const{
        str.write((char*)&id,sizeof(id));
        str.write((char*)&polycrc,sizeof(polycrc));

        // write external marker corners
        uint32_t ncorners = static_cast<uint32_t> (size());
        str.write((char*)&ncorners, sizeof(ncorners));
        for (size_t i = 0; i < size(); i++)
            str.write((char*)&at(i), sizeof(cv::Point2f));

        // write inner bits
        uint32_t nbits = static_cast<uint32_t> (bits.size());
        str.write((char*)&nbits, sizeof(nbits));
        for (size_t i = 0; i < bits.size(); i++)
            str.write((char*)&bits.at(i), sizeof(cv::Point2f));

        // write colors
        uint32_t s=colors[0].size();
        str.write((char*)&s, sizeof(s));
        str.write((char*)&colors[0][0], colors[0].size());
        s=colors[1].size();
        str.write((char*)&s, sizeof(s));
        str.write((char*)&colors[1][0], colors[1].size());

        // write 3d corners
        uint32_t np3d = static_cast<uint32_t> (p3d.size());
        str.write((char*)&np3d, sizeof(np3d));
        for (size_t i = 0; i < np3d; i++)
            str.write((char*)&p3d.at(i), sizeof(cv::Point3f));
    }
    // fromStream
    void Marker::fromStream(std::istream &str){
        str.read((char*)&id,sizeof(id));
        str.read((char*)&polycrc,sizeof(polycrc));

        uint32_t ncorners;
        str.read((char*)&ncorners, sizeof(ncorners));
        resize(ncorners);
        for (size_t i = 0; i < size(); i++)
            str.read((char*)&(*this)[i], sizeof(cv::Point2f));

        uint32_t nbits;
        str.read((char*)&nbits, sizeof(nbits));
        bits.resize(nbits);
        for (size_t i = 0; i < bits.size(); i++)
            str.read((char*)&(bits)[i], sizeof(cv::Point2f));

        colors.resize(2);
        uint32_t s;
        str.read((char*)&s, sizeof(s));
        colors[0].resize(s);
        str.read((char*)&(colors)[0][0], colors[0].size());
        str.read((char*)&s, sizeof(s));
        colors[1].resize(s);
        str.read((char*)&colors[1][0], colors[1].size());

        uint32_t np3d;
        str.read((char*)&np3d, sizeof(np3d));
        p3d.resize(np3d);
        for (size_t i = 0; i < np3d; i++)
            str.read((char*)&(p3d)[i], sizeof(cv::Point3f));
    }






/**
 * @brief Class made in order to manage the detection of the
 *        customized fiducial markers
 */
class MarkerDetector{
public:
    /* Constructors */
    inline MarkerDetector(){
        // Multi marker configuration
        this->_numberOfMarkersConfigurations = 0;
        /* Tiny management configuration */
        // Adaptive thresholding
            this->_adapThresWindowSize = 5;
            this->_adapThresBlockSize = 5;
        // Area conditions
            this->_areaThreshold = 200;
    };
    /* Config */
    inline void loadConfigFile(const std::string path);
    inline void loadConfigFile(const std::vector<std::string> fileContent);
    inline Parameters getParams();
    /* Detection */
    inline std::vector<Marker> detect(const cv::Mat &img);
    /* Management */
    inline void setMarker(Marker m){_setMarkers.clear(); _setMarkers.push_back(m);};
    inline void setColorDetectionMode(std::string c);
    /* Tiny management*/
    int _adapThresBlockSize;
    int _adapThresWindowSize;
    int _areaThreshold;
    /* Debugging */
    std::vector<Marker> getCandidates()const{return candidates;}
    /* Stream functions */
    // inline void toStream(std::ostream &str)const;
    // inline void fromStream(std::istream &str);

private:
    /* Config */
        std::vector<Marker> _setMarkers;
        std::vector<Parameters> _setParams;
        int _numberOfMarkersConfigurations;
    /* Detection info */
    std::vector<Marker> candidates;
    /* Conditions */
    inline float __getSubpixelValue(const cv::Mat &im_grey,const cv::Point2f &p);
    inline bool hasGoodBorder(const cv::Mat& gray, Marker marker);
    inline void adaptiveThreshold(cv::Mat src,cv::Mat &dst,int maxVal, int blockSize);
    inline void refineCornerWithContourLines( Marker &marker);
    inline void interpolate2Dline(const std::vector<cv::Point2f>& inPoints, cv::Point3f& outLine);
    inline cv::Point2f getCrossPoint(const cv::Point3f& line1, const cv::Point3f& line2);
};

/* Config */
    // loadConfigFile
    void MarkerDetector::loadConfigFile(const std::string path){


        this->_numberOfMarkersConfigurations += 1;


        cv::FileStorage fs;
        fs.open(path, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            throw std::invalid_argument( "Failed to open "+path);
            return;
        }


        Parameters _params;
        Marker marker;


        cv::FileNode nodep = fs["params"];
        if ( nodep.type()!=cv::FileNode::NONE)
        {
            if ( nodep["minMarkerSize"].type()!=cv::FileNode::NONE)
                nodep["minMarkerSize"]>>_params.minMarkerSize;
            if ( nodep["thrGray"].type()!=cv::FileNode::NONE)
                nodep["thrGray"]>>_params.thrGray;
            if ( nodep["thrHsv"].type()!=cv::FileNode::NONE)
                nodep["thrHsv"]>>_params.thrHsv;
            if ( nodep["border"].type()!=cv::FileNode::NONE)
                nodep["border"]>>_params.border;
            if ( nodep["colorDetectionMode"].type()!=cv::FileNode::NONE)
                nodep["colorDetectionMode"]>>_params.colorDetectionMode;
        }

        cv::FileNode nodem = fs["marker"];
        if (nodem.type() != cv::FileNode::NONE)
        {
            cv::FileNode n = nodem["corners"];
            for (cv::FileNodeIterator itp = n.begin(); itp != n.end(); ++itp)
            {
                std::vector<float> coord;
                (*itp) >> coord;
                if (coord.size() != 2)
                    std::cerr << "bit-info has incorrect corners" << std::endl;
                marker.push_back(cv::Point2f(coord[0], coord[1]));
                marker.p3d.push_back(cv::Point3f(coord[0], coord[1], 0.f));
            }

            cv::FileNode c = nodem["colors"];
            if(c.size()!=2) throw std::invalid_argument( "Two colors are necessary.");
            for (cv::FileNodeIterator itp = c.begin(); itp != c.end(); ++itp)
            {
                std::string color;
                (*itp) >> color;
                marker.colors.push_back(color.substr(1,color.length()));
            }

            _private::anticlockwise(marker);
        }
        else
            throw std::runtime_error( "error yml file. marker doesn't exist!!!" );




        cv::FileNode nodecrc = fs["bit-crc"];
        cv::FileNode nodeinfo = fs["bit-info"];

        marker.bits.resize(nodeinfo.size() + nodecrc.size());

        if (nodecrc.type() == cv::FileNode::SEQ)
        {

            cv::FileNodeIterator it = nodecrc.begin(), it_end = nodecrc.end(); // Go through the node
            for (; it != it_end; ++it)
            {
                std::vector<float> center;
                (*it)["center"] >> center;
                marker.bits[(nodecrc.size()-1)-(int)(*it)["id"] + nodeinfo.size()] = cv::Point2f(center[0], center[1]);
            }
        }
        else
            throw std::runtime_error( "error yml file. bit-crc is not a sequence!!!!" );


        if (nodeinfo.type() == cv::FileNode::SEQ)
        {

            cv::FileNodeIterator it = nodeinfo.begin(), it_end = nodeinfo.end(); // Go through the node
            for (; it != it_end; ++it)
            {
                std::vector<float> center;
                (*it)["center"] >> center;
                marker.bits[(nodeinfo.size()-1)-(int)(*it)["id"]] = cv::Point2f(center[0], center[1]);
            }
        }
        else
            throw std::runtime_error( "error yml file. bit-info is not a sequence!!!!" );


        if(nodecrc.size()==8) marker.polycrc = cucomarker::CRC8;
        else if(nodecrc.size()==16) marker.polycrc = cucomarker::CRC16;
        else if(nodecrc.size()==32) marker.polycrc = cucomarker::CRC32;
        else throw std::invalid_argument( "Incorrect number of bit for crc (8, 16 or 32).");



        this->_setParams.push_back(_params);
        this->_setMarkers.push_back(marker);


        fs.release();
    }
    
    

    void MarkerDetector::loadConfigFile(const std::vector<std::string> fileContent){

        this->_numberOfMarkersConfigurations += 1;


        std::string yml_content;
        for (const auto& line : fileContent) {
            yml_content += line + "\n";
        }

        // Luego, crea un FileStorage a partir de este string.
        cv::FileStorage fs(yml_content, cv::FileStorage::READ | cv::FileStorage::MEMORY);


        Parameters _params;
        Marker marker;


        cv::FileNode nodep = fs["params"];
        if ( nodep.type()!=cv::FileNode::NONE)
        {
            if ( nodep["minMarkerSize"].type()!=cv::FileNode::NONE)
                nodep["minMarkerSize"]>>_params.minMarkerSize;
            if ( nodep["thrGray"].type()!=cv::FileNode::NONE)
                nodep["thrGray"]>>_params.thrGray;
            if ( nodep["thrHsv"].type()!=cv::FileNode::NONE)
                nodep["thrHsv"]>>_params.thrHsv;
            if ( nodep["border"].type()!=cv::FileNode::NONE)
                nodep["border"]>>_params.border;
            if ( nodep["colorDetectionMode"].type()!=cv::FileNode::NONE)
                nodep["colorDetectionMode"]>>_params.colorDetectionMode;
        }

        cv::FileNode nodem = fs["marker"];
        if (nodem.type() != cv::FileNode::NONE)
        {
            cv::FileNode n = nodem["corners"];
            for (cv::FileNodeIterator itp = n.begin(); itp != n.end(); ++itp)
            {
                std::vector<float> coord;
                (*itp) >> coord;
                if (coord.size() != 2)
                    std::cerr << "bit-info has incorrect corners" << std::endl;
                marker.push_back(cv::Point2f(coord[0], coord[1]));
                marker.p3d.push_back(cv::Point3f(coord[0], coord[1], 0.f));
            }

            cv::FileNode c = nodem["colors"];
            if(c.size()!=2) throw std::invalid_argument( "Two colors are necessary.");
            for (cv::FileNodeIterator itp = c.begin(); itp != c.end(); ++itp)
            {
                std::string color;
                (*itp) >> color;
                marker.colors.push_back(color.substr(1,color.length()));
            }

            _private::anticlockwise(marker);
        }
        else
            throw std::runtime_error( "error yml file. marker doesn't exist!!!" );




        cv::FileNode nodecrc = fs["bit-crc"];
        cv::FileNode nodeinfo = fs["bit-info"];

        marker.bits.resize(nodeinfo.size() + nodecrc.size());

        if (nodecrc.type() == cv::FileNode::SEQ)
        {

            cv::FileNodeIterator it = nodecrc.begin(), it_end = nodecrc.end(); // Go through the node
            for (; it != it_end; ++it)
            {
                std::vector<float> center;
                (*it)["center"] >> center;
                marker.bits[(nodecrc.size()-1)-(int)(*it)["id"] + nodeinfo.size()] = cv::Point2f(center[0], center[1]);
            }
        }
        else
            throw std::runtime_error( "error yml file. bit-crc is not a sequence!!!!" );


        if (nodeinfo.type() == cv::FileNode::SEQ)
        {

            cv::FileNodeIterator it = nodeinfo.begin(), it_end = nodeinfo.end(); // Go through the node
            for (; it != it_end; ++it)
            {
                std::vector<float> center;
                (*it)["center"] >> center;
                marker.bits[(nodeinfo.size()-1)-(int)(*it)["id"]] = cv::Point2f(center[0], center[1]);
            }
        }
        else
            throw std::runtime_error( "error yml file. bit-info is not a sequence!!!!" );


        if(nodecrc.size()==8) marker.polycrc = cucomarker::CRC8;
        else if(nodecrc.size()==16) marker.polycrc = cucomarker::CRC16;
        else if(nodecrc.size()==32) marker.polycrc = cucomarker::CRC32;
        else throw std::invalid_argument( "Incorrect number of bit for crc (8, 16 or 32).");



        this->_setParams.push_back(_params);
        this->_setMarkers.push_back(marker);


        fs.release();
    }


    // getParams
    Parameters MarkerDetector::getParams(){
        Parameters params;
        if(this->_numberOfMarkersConfigurations != 0){
            this->_setParams[0].copyToParameters(params);
        }
       return params;
    }


// __getSubpixelValue
float MarkerDetector::__getSubpixelValue(const cv::Mat &im_grey,const cv::Point2f &p){
    float x=int(p.x);
    float y=int(p.y);
    //cheat to avoid seg fault
    if (x<0 || x>=im_grey.cols-1 || y<0 || y>=im_grey.rows-1)   return 0;
    const uchar* ptr_y=im_grey.ptr<uchar>(y);
    const uchar* ptr_yp=im_grey.ptr<uchar>(y+1);
    float tl=float(ptr_y[int(x)]);
    float tr=float(ptr_y[int(x+1)]);
    float bl=float(ptr_yp[int(x)]);
    float br=float(ptr_yp[int(x+1)]);
    float a= float(x+1.f-p.x) * tl  + (p.x-x)*tr;
    float b= float(x+1.f-p.x) * bl  + (p.x-x)*br;
    return   (y+1-p.y)*a   + (p.y-y)*b;
}


/* Detection */
    // detect
    std::vector<Marker> MarkerDetector::detect(const cv::Mat &img)
{
    std::vector<Marker> detectedMarkers;
    cv::Mat gray, thresImage;

    if(img.channels()==3)
        cv::cvtColor(img,gray,cv::COLOR_BGR2GRAY);
    else gray=img;

    int adaptiveWindowSize=this->_adapThresWindowSize;//std::max(int(3),int(15*float(gray.cols)/1920.));
    if( adaptiveWindowSize%2==0) adaptiveWindowSize++;
    cv::adaptiveThreshold(gray, thresImage, 255.,cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, adaptiveWindowSize, this->_adapThresBlockSize);

    //custom one that adapts better to ligth changes using a unique threshold for each pixel
//    adaptiveThreshold(gray, thresImage,1,adaptiveWindowSize);

    // cv::imshow("thres",thresImage);
    //cv::waitKey(0);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> approxCurve;
    cv::findContours(thresImage, contours, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
//        cv::Mat img4;
//        img.copyToAnother(img4);




    float imSize = img.cols * img.rows;
    candidates.clear();


    int cond_0 = 0, cond_1 = 0, cond_2 = 0, cond_3 = 0;


    // Parameters _params = this->_setParams[0];
    // Marker marker = this->_setMarkers[0];

    // Parameters _params = this->_params;
    // Marker marker = this->marker;

    for (unsigned int i = 0; i < contours.size(); i++)
    {


        std::vector<int> validConfigurations;
        for(int c = 0; c < this->_numberOfMarkersConfigurations; c++){
            validConfigurations.push_back(c);
        }


        //if (_params.minMarkerSize * imSize > area  ) continue;
        std::vector<int> validConfigurations_1;
        double area =cv::contourArea(contours[i]);
        for(int c = 0; c < validConfigurations.size(); c++){
            int config = validConfigurations[c];
            if(this->_areaThreshold == 0){
                if(!(_setParams[config].minMarkerSize * imSize > area)){
                    validConfigurations_1.push_back(config);
                }
            }
            else if( area > this->_areaThreshold ){
                validConfigurations_1.push_back(config);
            }
        }
        if(validConfigurations_1.size() == 0) continue;
        cond_0++;



        std::vector<cv::Point3f> p3d;
        Marker markerCandidate(p3d);
        cv::approxPolyDP(contours[i], markerCandidate, 5, true);
        // if (markerCandidate.size() != marker.size() ) continue;
        // Switch on which configuration to review
        std::vector<int> validConfigurations_2;
        for(int c = 0; c < validConfigurations_1.size(); c++){
            int config = validConfigurations_1[c];
            if(!(markerCandidate.size() != this->_setMarkers[config].size())){
                validConfigurations_2.push_back(config);
            }
        }
        if(validConfigurations_2.size() == 0) continue;
        cond_1++;




        if(!cv::isContourConvex(markerCandidate)) continue;
        cond_2++;





        // if(_params.border)  if(!hasGoodBorder(gray, markerCandidate)) continue;
            std::vector<int> validConfigurations_3;
            for(int c = 0; c < validConfigurations_2.size(); c++){
                int config = validConfigurations_2[c];
                if(_setParams[config].border){
                    if((!hasGoodBorder(gray, markerCandidate))){
                        // ...
                    }else{ validConfigurations_3.push_back(config); }
                }else{ validConfigurations_3.push_back(config); }
            }
            if(validConfigurations_3.size() == 0) continue;
            cond_3++;





        bool isAckwise=_private::anticlockwise(markerCandidate);


        // Refine markerCandidate
        candidates.push_back(markerCandidate);
        

        // We have one candidate for validConfigurations.size() configurations
            for(int c = 0; c < validConfigurations_3.size(); c++){

                // Configuration
                int config = validConfigurations_3[c];
                    // Settings
                    Parameters _params; this->_setParams[config].copyToParameters(_params);
                    Marker marker; this->_setMarkers[config].copyToMarker(marker);
                    // Schema into marker candidate
                    markerCandidate.setConfigurationSchema(config);



                /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    Optimize markers Contours before
                ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
                // Save old corners
                std::vector<cv::Point> aux(markerCandidate.contours);
                // Set the contours
                markerCandidate.contours = contours[i];
                if(isAckwise){
                    std::reverse(markerCandidate.contours.begin(),markerCandidate.contours.end());
                }
                // Refine
                refineCornerWithContourLines(markerCandidate);
                // Obtain again old contours
                markerCandidate.contours = aux;
                


                // PER EACH POSSIBLE ROTATION
                for(size_t r = 0; r < marker.size(); r++){

                std::rotate(markerCandidate.begin(), markerCandidate.begin()+1, markerCandidate.end());

                cv::Mat Homography = findHomography(marker, markerCandidate, cv::RANSAC);
                //cv::Mat Homography = findHomography(marker, markerCandidate, cv::RHO, 7, cv::noArray(), 10000, 0.999);

                std::vector<cv::Mat> Hs; Hs.push_back(Homography);

                for(int h = 0; h < Hs.size(); h++){
                    cv::Mat H = Hs[h];

                    if(H.empty()) continue;
                    std::vector<cv::Point2f> img_bits ;
                    cv::perspectiveTransform(marker.bits, img_bits, H);

                    cv::Mat pxGray(1,marker.bits.size(),CV_8UC1);
                    cv::Mat pxColor(1,marker.bits.size(),CV_8UC3);




//                    cv::Mat img3 = img.clone();
//                    markerCandidate.draw(img3);

//        //            auto aux=markerCandidate;
//        //            aux.pop_back();
//        //            aux.draw(img3,cv::Scalar(255,0,0));

//                    for(auto pt:img_bits)
//                        cv::circle(img3,pt,2,cv::Scalar(255,0,0),cv::FILLED);

//                    cv::drawContours(img3,contours,i, cv::Scalar(0,255,0));

//                    cv::imshow("img3", img3);
//                    std::cout<<"\ti: "<<i<<" |r: "<<r<<std::endl;

//                    cv::waitKey();
//                    //std::cout<<"- ("<<h<<") H: "<<H<<std::endl;
//                    //std::cout<<"\ti: "<<i<<" |r: "<<r<<std::endl;




                    size_t n=0;
                    for(auto bit:img_bits)
                    {
                        if(bit.y>=gray.rows || bit.x>=gray.cols || bit.y < 0 || bit.x < 0)
                            break;

                        pxColor.at<cv::Vec3b>(0,n)=img.at<cv::Vec3b>(bit.y, bit.x);
                        pxGray.at<uchar>(0,n)=this->__getSubpixelValue(gray,bit);//gray.at<uchar>(bit.y, bit.x);

                        n++;
                    }
                    if(n != marker.bits.size()) continue;


                    std::pair<float, float> _gray(std::numeric_limits<float>::lowest(),std::numeric_limits<float>::lowest());  //Avg_Sigma gray values
                    std::pair<int, float> _hsv(std::numeric_limits<float>::lowest(),std::numeric_limits<float>::lowest());     //Rotation_Sigma HSV values

                    if(_params.colorDetectionMode==ALL || _params.colorDetectionMode==GRAY)
                        _gray=_private::grayAnalysis(pxGray);

                    std::vector<int> pxH; //pixel values HSV (H channel)
                    if(_params.colorDetectionMode==ALL || _params.colorDetectionMode==HSV)
                    {
                        cv::Mat pxHSV;
                        cv::cvtColor(pxColor, pxHSV, cv::COLOR_BGR2HSV);
                        for(size_t i=0; i<marker.bits.size();i++)
                            pxH.push_back( pxHSV.at<cv::Vec3b>(0,i)[0] );
                        _hsv = _private::circularAverage(pxH);
                    }


                    std::vector<unsigned long> codes;
                    if(_gray.second > _hsv.second || _hsv.second < _params.thrHsv)
                    {
                        //GREY DECTECTION
                        if(_gray.second < _params.thrGray) continue;

                        unsigned long code=0;
                        for(int i=0; i<pxGray.cols; i++)
                        {
                            code = code << 1;
                            if(pxGray.at<uchar>(0,i)>=_gray.first)
                                code+=1;
                        }
                        codes.push_back(code);

                        int x = log2(code) + 1;
                        // Inverting the bits one by one
                        for (int i = 0; i < x; i++)
                           code = (code ^ (1 << i));
                        codes.push_back(code);
                    }
                    else
                    {
                        //HSV DECTECTION
                        if(_hsv.second < _params.thrHsv) continue;

                        unsigned long code=0;
                        for(auto &px:pxH)
                        {
                            code = code << 1;
                            if( (px+_hsv.first)%180 >= 90)
                                code+=1;
                        }
                        codes.push_back(code);

                        int x = log2(code) + 1;
                        // Inverting the bits one by one
                        for (int i = 0; i < x; i++)
                           code = (code ^ (1 << i));
                        codes.push_back(code);
                    }




                    bool detected=false;
                    for(auto c:codes)
                    {
                        markerCandidate.id = c >> marker.polycrc;
                        if(markerCandidate.id==0) continue;

                        uint32_t crc=-1, checksum=-2;
                        switch (marker.polycrc) {
                        case 8:
                            checksum = c&0xFF;
                            crc= _private::crc_8(markerCandidate.id);
                            break;
                        case 16:
                            checksum = c&0xFFFF;
                            crc= _private::crc_16(markerCandidate.id);
                            break;
                        case 32:
                            checksum = c&0xFFFFFFFF;
                            crc= _private::crc_32(markerCandidate.id);
                            break;
                        default:
                            break;
                        }


                        /*
                        cv::Mat img3 = img.clone();
                        markerCandidate.draw(img3);
                        for(auto pt:img_bits)
                            cv::circle(img3,pt,1,cv::Scalar(255,0,0),cv::FILLED);
                        cv::imshow("img3", img3);
                        cv::waitKey();
                        */


                        if(crc == checksum)
                        {
                            markerCandidate.area=area;
                            markerCandidate.contours = contours[i];
                            markerCandidate.polycrc = marker.polycrc;
                                for(auto b: img_bits){
                                    markerCandidate.bits.push_back(b);
                                }
                            if(isAckwise){
                                std::reverse(markerCandidate.contours.begin(),markerCandidate.contours.end());
                            }


                            bool found=false, overlapped=false;
                            for(auto &m1:detectedMarkers)
                            {
                                if(m1.id == markerCandidate.id)
                                {
                                    found=true;

                                    //Overlapped polygons????
                                    for(auto c:m1)
                                    {
                                        if(markerCandidate.isInto(c))
                                        {
                                            m1=markerCandidate;
                                            overlapped = true;
                                            break;
                                        }
                                    }

                                    if(!overlapped)
                                    {
                                        for(auto c:markerCandidate)
                                        {
                                            if(m1.isInto(c))
                                            {
                                                overlapped = true;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }

        //                    std::cout<<"GRAY|HSV: " << _gray.second <<", "<< _hsv.second <<std::endl;


                            if(!found || !overlapped)
                            {

                                detectedMarkers.push_back(markerCandidate);
                            }

                            detected=true;
                            break;
                        }
                    }

                    if(detected)
                    {
                        break;
                    }
                    //else next rotation


                }


            }


        } // for(auto markerCandidate: createdCandidates)

    }



    // std::cout<<"Conditions: "<<std::endl;
    //     std::cout<<"\t- Cond_0: "<<cond_0<<std::endl;
    //     std::cout<<"\t- Cond_1: "<<cond_1<<std::endl;
    //     std::cout<<"\t- Cond_2: "<<cond_2<<std::endl;
    //     std::cout<<"\t- Cond_3: "<<cond_3<<std::endl;


    // //now, lets do line refinement
    // for (auto &marker:  detectedMarkers ){
    //     refineCornerWithContourLines(marker);
    // }





//    //subpixel refinement
//    for (auto &marker:  detectedMarkers ){
//        std::vector<cv::Point2f> Corners;
//        int halfwsize=2;
//        if(marker.getArea()>2500)  halfwsize=3;
//        if(marker.getArea()>9000)  halfwsize=4;
//        for (int c = 0; c < 4; c++) Corners.push_back(marker[c]);
//        cv::cornerSubPix(gray, Corners, cv::Size(halfwsize,halfwsize), cv::Size(-1, -1),cv::TermCriteria( cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 12, 0.005));
//        // copy back
//        for (int c = 0; c < 4; c++)
//            marker[c] = Corners[ c];
//    }

    return detectedMarkers;
}

/* Management */
    // setColorDetectionMode
    void MarkerDetector::setColorDetectionMode(std::string c){
        if(c=="ALL"){
            for(auto &p: this->_setParams)
                p.colorDetectionMode = ALL;
        }
        else if(c=="GRAY"){
            for(auto &p: this->_setParams)
                p.colorDetectionMode = GRAY;
        }
        else if(c=="HSV"){
            for(auto &p: this->_setParams)
                p.colorDetectionMode = HSV;
        }
        else throw std::invalid_argument( "Incorrect color detection mode (ALL, GRAY or HSV)");
    };

/* Conditions */
    // adaptiveThreshold
    void  MarkerDetector::adaptiveThreshold(cv::Mat src,cv::Mat &dst,int maxVal,    int blockSize){

        auto  scale=[](double _inMin, double _inMax, double _outMin, double _outMax, double val)  {
            if(val<=_inMin)return _outMin;
            else if(val>=_inMax)return _outMax;
            double aux = (_inMax - _inMin);
            double aa = (_outMax - _outMin)/aux;
            return val * aa + (_outMax - (aa * _inMax));
        };

        int scaleLUT[256];
        for(int i=0;i<256;i++) scaleLUT[i]=scale(1,150,1,7, i) ;

        cv::boxFilter( src, dst, src.type(), cv::Size(blockSize, blockSize),
                    cv::Point(-1,-1), true, cv::BORDER_REPLICATE|cv::BORDER_ISOLATED );


        cv::Size size = src.size();
        if( src.isContinuous() &&   dst.isContinuous() )
        {
            size.width *= size.height;
            size.height = 1;
        }


        for(int i = 0; i < size.height; i++ )
        {

            const uchar* sdata = src.ptr(i);
            uchar* ddata = dst.ptr(i);
            for(int j = 0; j < size.width; j++ )
                ddata[j] = ((ddata[j]-scaleLUT[ddata[j]]  )> sdata[j]) *255;
        }
        //cv::imshow("thre",dst);

    }
    // hasGoodBorder
    bool MarkerDetector::hasGoodBorder(const cv::Mat& gray, Marker marker)
    {
        std::vector<cv::Point2f> border=marker;

        cv::Point2f center = marker.getCenter();
        for(auto &p:border)
            p-= (p-center)*0.05f;

        cv::Mat img3;
        cv::cvtColor(gray,img3, cv::COLOR_RGB2BGR);

        std::vector<int> borderValues;
        float mean=0;
        int nPixels=30; //Distance between pixels
        for(size_t i=0; i<border.size(); i++)
        {
            int n=borderValues.size();

            cv::Point2f corner1 = border[i];
            cv::Point2f corner2 = border[(i+1)%border.size()];

            int nPoints = sqrt(pow(corner2.x - corner1.x,2) + pow(corner2.y - corner1.y,2)) / nPixels;

            float int_X = (corner2.x - corner1.x) / (nPoints + 1);
            float int_Y = (corner2.y - corner1.y) / (nPoints + 1);

            borderValues.resize(borderValues.size() + nPoints);

            for (int j=0; j < nPoints; j++)
            {
                borderValues[j+n] = int(gray.at<uchar>(corner1.y+int_Y*j, corner1.x+int_X*j));
                mean+=borderValues[j+n];
            }
        }


        if(borderValues.size() < 4) return false;
        mean/=borderValues.size();

        float stdev=0;
        for(auto v:borderValues)
            stdev +=  (v - mean)*(v - mean);
        stdev= sqrt(stdev / borderValues.size());

    //    std::cout <<"Border:"<< stdev <<", "<< borderValues.size()<<std::endl;

        if(stdev > 10) return false;
        else return true;
    }
    // interpolate2Dline
    void MarkerDetector::interpolate2Dline(const std::vector<cv::Point2f>& inPoints, cv::Point3f& outLine)
    {
        float minX, maxX, minY, maxY;
        minX = maxX = inPoints[0].x;
        minY = maxY = inPoints[0].y;
        for (unsigned int i = 1; i < inPoints.size(); i++)
        {
            if (inPoints[i].x < minX)
                minX = inPoints[i].x;
            if (inPoints[i].x > maxX)
                maxX = inPoints[i].x;
            if (inPoints[i].y < minY)
                minY = inPoints[i].y;
            if (inPoints[i].y > maxY)
                maxY = inPoints[i].y;
        }

        // create matrices of equation system
        const int pointsCount = static_cast<int>(inPoints.size());
        cv::Mat A(pointsCount, 2, CV_32FC1, cv::Scalar(0));
        cv::Mat B(pointsCount, 1, CV_32FC1, cv::Scalar(0));
        cv::Mat X;

        if (maxX - minX > maxY - minY)
        {
            // Ax + C = y
            for (int i = 0; i < pointsCount; i++)
            {
                A.at<float>(i, 0) = inPoints[i].x;
                A.at<float>(i, 1) = 1.;
                B.at<float>(i, 0) = inPoints[i].y;
            }

            // solve system
            solve(A, B, X, cv::DECOMP_SVD);
            // return Ax + By + C
            outLine = cv::Point3f(X.at<float>(0, 0), -1., X.at<float>(1, 0));
        }
        else
        {
            // By + C = x
            for (int i = 0; i < pointsCount; i++)
            {
                A.at<float>(i, 0) = inPoints[i].y;
                A.at<float>(i, 1) = 1.;
                B.at<float>(i, 0) = inPoints[i].x;
            }

            // solve system
            solve(A, B, X, cv::DECOMP_SVD);
            // return Ax + By + C
            outLine = cv::Point3f(-1., X.at<float>(0, 0), X.at<float>(1, 0));
        }
    }
    // getCrossPoint
    cv::Point2f MarkerDetector::getCrossPoint(const cv::Point3f& line1, const cv::Point3f& line2)
    {
        // create matrices of equation system
        cv::Mat A(2, 2, CV_32FC1, cv::Scalar(0));
        cv::Mat B(2, 1, CV_32FC1, cv::Scalar(0));
        cv::Mat X;

        A.at<float>(0, 0) = line1.x;
        A.at<float>(0, 1) = line1.y;
        B.at<float>(0, 0) = -line1.z;

        A.at<float>(1, 0) = line2.x;
        A.at<float>(1, 1) = line2.y;
        B.at<float>(1, 0) = -line2.z;

        // solve system
        solve(A, B, X, cv::DECOMP_SVD);
        return cv::Point2f(X.at<float>(0, 0), X.at<float>(1, 0));
    }


    // refineCornerWithContourLines
    void MarkerDetector::refineCornerWithContourLines(  Marker &marker ){
        
        // Search corners on the contour vector
        auto &contour=marker.contours;
        std::vector< int > cornerIndex(marker.size(),-1);
        std::vector<float> dist(marker.size(),std::numeric_limits<float>::max());
        for (unsigned int j = 0; j < contour.size(); j++) {
            for (unsigned int k = 0; k < marker.size(); k++) {
                float d=  (contour[j].x-marker[k].x)*(contour[j].x-marker[k].x) +
                        (contour[j].y-marker[k].y)*(contour[j].y-marker[k].y) ;
                if (d<dist[k]){
                    cornerIndex[k] = j;
                    dist[k]=d;
                }
            }
        }
        
        // Undistort contour
        std::vector< cv::Point2f > contour2f;
            contour2f.reserve(contour.size());
            for(auto p:contour)
                contour2f.push_back(cv::Point2f(p.x,p.y));

        std::vector< std::vector< cv::Point2f > > contourLines;
        contourLines.resize(marker.size());
        for (unsigned int l = 0; l < marker.size(); l++) {
            for (int j = (int)cornerIndex[l]; j != (int)cornerIndex[(l + 1) % marker.size()]; j += 1) {
                if (j == (int)contour.size()) j = 0;
                contourLines[l].push_back(contour2f[j]);
                if (j == (int)cornerIndex[(l + 1) % marker.size()])
                    break; // this has to be added because of the previous ids
            }
        }

        // Interpolate marker lines
        std::vector< cv::Point3f > lines;
        lines.resize(marker.size());
        for (unsigned int j = 0; j < lines.size(); j++)
            interpolate2Dline(contourLines[j], lines[j]);

        marker[0] = getCrossPoint(lines[marker.size()-1],lines[0] );
        for ( int i = 1; i < marker.size(); i++)
            marker[i] = getCrossPoint(lines[i- 1] ,lines[i] );

}

/* Stream functions */
    // // toStream
    // void MarkerDetector::toStream(std::ostream &str)const
    // {
    //     uint64_t sig=13213;
    //     str.write((char*)&sig,sizeof(sig));

    //     _params.toStream(str);
    //     marker.toStream(str);
    // }
    // // fromStream
    // void MarkerDetector::fromStream(std::istream &str)
    // {
    //     uint64_t sig;
    //     str.read((char*)&sig,sizeof(sig));
    //     if (sig!=13213) throw std::runtime_error("MarkerDetector_Impl::fromStream invalid signature");

    //     _params.fromStream(str);
    //     marker.fromStream(str);
    // }



} // end cucomarker namespace


#endif
