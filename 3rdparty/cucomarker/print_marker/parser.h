/*
MIT License

Copyright (c) 2020 Nwutobo Samuel Ugochukwu <sammycageagle@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef _PARSER_
#define _PARSER_

#include <iostream>
#include <opencv2/core.hpp>
#include <string>

#define IS_ALPHA(c) ((c) >= 'a' && (c) <= 'z') || ((c) >= 'A' && (c) <= 'Z')
#define IS_NUM(c) ((c) >= '0' && (c) <= '9')
#define IS_WS(c) ((c) == ' ' || (c) == '\t' || (c) == '\n' || (c) == '\r')



template<typename T>
inline bool parseNumber(const char*& ptr, const char* end, T& number)
{
    T integer, fraction;
    int sign, expsign, exponent;

    static const T numberMax = std::numeric_limits<T>::max();
    fraction = 0;
    integer = 0;
    exponent = 0;
    sign = 1;
    expsign = 1;

    if(ptr < end && *ptr == '+')
        ++ptr;
    else if(ptr < end && *ptr == '-')
    {
        ++ptr;
        sign = -1;
    }

    if(ptr >= end || !(IS_NUM(*ptr) || *ptr == '.'))
        return false;

    if(*ptr != '.')
    {
        do {
            integer = static_cast<T>(10) * integer + (*ptr - '0');
            ++ptr;
        } while(ptr < end && IS_NUM(*ptr));
    }

    if(ptr < end && *ptr == '.')
    {
        ++ptr;
        if(ptr >= end || !IS_NUM(*ptr))
            return false;

        T divisor = 1;
        do {
            fraction = static_cast<T>(10) * fraction + (*ptr - '0');
            divisor *= static_cast<T>(10);
            ++ptr;
        } while(ptr < end && IS_NUM(*ptr));
        fraction /= divisor;
    }

    if(ptr < end && (*ptr == 'e' || *ptr == 'E')
       && (ptr[1] != 'x' && ptr[1] != 'm'))
    {
        ++ptr;
        if(ptr < end && *ptr == '+')
            ++ptr;
        else if(ptr < end && *ptr == '-')
        {
            ++ptr;
            expsign = -1;
        }

        if(ptr >= end || !IS_NUM(*ptr))
            return false;

        do {
            exponent = 10 * exponent + (*ptr - '0');
            ++ptr;
        } while(ptr < end && IS_NUM(*ptr));
    }

    number = sign * (integer + fraction);
    if(exponent)
        number *= static_cast<T>(pow(10.0, expsign*exponent));

    return number >= -numberMax && number <= numberMax;
}

inline bool skipWs(const char*& ptr, const char* end)
{
    while(ptr < end && IS_WS(*ptr))
       ++ptr;

    return ptr < end;
}

inline bool skipWsDelimiter(const char*& ptr, const char* end, const char delimiter)
{
    if(ptr < end && !IS_WS(*ptr) && *ptr != delimiter)
        return false;

    if(skipWs(ptr, end))
    {
        if(ptr < end && *ptr == delimiter)
        {
            ++ptr;
            skipWs(ptr, end);
        }
    }

    return ptr < end;
}

inline bool parseNumberList(const char*& ptr, const char* end, double* values, int count)
{
    for(int i = 0;i < count;i++)
    {
        if(!parseNumber(ptr, end, values[i]))
            return false;
        skipWsDelimiter(ptr, end, ',');
    }

    return true;
}

inline bool parseArcFlag(const char*& ptr, const char* end, bool& flag)
{
    if(ptr < end && *ptr == '0')
        flag = false;
    else if(ptr < end && *ptr == '1')
        flag = true;
    else
        return false;

    ++ptr;
    skipWsDelimiter(ptr, end, ',');
    return true;
}

/*
inline std::vector<cv::Point2f> parsePath(const std::string& string)
{
    std::vector<cv::Point2f> points;

    auto ptr = string.data();
    auto end = ptr + string.size();
    if(ptr >= end || !(*ptr == 'M' || *ptr == 'm'))
        return points;

    auto command = *ptr++;
    double c[6];
    bool f[2];

    cv::Point2f startPoint;
    cv::Point2f currentPoint;
    cv::Point2f controlPoint;

    while(true)
    {
        switch(command) {


        case 'M':
        case 'm':
            if(!parseNumberList(ptr, end, c, 2))
                break;

            if(command == 'm')
            {
                c[0] += currentPoint.x;
                c[1] += currentPoint.y;
            }

            startPoint.x = currentPoint.x = controlPoint.x = c[0];
            startPoint.y = currentPoint.y = controlPoint.y = c[1];
            command = command == 'm' ? 'l' : 'L';

            points.push_back(currentPoint);
            break;

        case 'L':
        case 'l':
            if(!parseNumberList(ptr, end, c, 2))
                break;

            if(command == 'l')
            {
                c[0] += currentPoint.x;
                c[1] += currentPoint.y;
            }

            currentPoint.x = controlPoint.x = c[0];
            currentPoint.y = controlPoint.y = c[1];
            points.push_back(currentPoint);

            break;
        case 'Q':
        case 'q':
            if(!parseNumberList(ptr, end, c, 4))
                break;
            break;
        case 'C':
        case 'c':
            if(!parseNumberList(ptr, end, c, 6))
                break;

            if(command == 'c')
            {
                c[0] += currentPoint.x;
                c[1] += currentPoint.y;
                c[2] += currentPoint.x;
                c[3] += currentPoint.y;
                c[4] += currentPoint.x;
                c[5] += currentPoint.y;
            }

            controlPoint.x = c[2];
            controlPoint.y = c[3];
            currentPoint.x = c[4];
            currentPoint.y = c[5];

            break;
        case 'T':
        case 't':
            c[0] = 2 * currentPoint.x - controlPoint.x;
            c[1] = 2 * currentPoint.y - controlPoint.y;
            if(!parseNumberList(ptr, end, c + 2, 2))
                break;
            break;
        case 'S':
        case 's':
            c[0] = 2 * currentPoint.x - controlPoint.x;
            c[1] = 2 * currentPoint.y - controlPoint.y;
            if(!parseNumberList(ptr, end, c + 2, 4))
                break;
            break;
        case 'H':
        case 'h':
            if(!parseNumberList(ptr, end, c, 1))
                break;

            if(command == 'h')
               c[0] += currentPoint.x;

            currentPoint.x = controlPoint.x = c[0];
            points.push_back(currentPoint);
            break;
        case 'V':
        case 'v':
            if(!parseNumberList(ptr, end, c + 1, 1))
                break;
            if(command == 'v')
               c[1] += currentPoint.y;

            currentPoint.y = controlPoint.y = c[1];
            points.push_back(currentPoint);
            break;
        case 'A':
        case 'a':
            if(!parseNumberList(ptr, end, c, 3)
                    || !parseArcFlag(ptr, end, f[0])
                    || !parseArcFlag(ptr, end, f[1])
                    || !parseNumberList(ptr, end, c + 3, 2))
                break;
            break;
        case 'Z':
        case 'z':
            currentPoint.x = controlPoint.x = startPoint.x;
            currentPoint.y = controlPoint.y = startPoint.y;
            break;

        default:
            return {};
            break;
        }

        skipWsDelimiter(ptr, end, ',');
        if(ptr >= end)
            break;

        if(IS_ALPHA(*ptr))
            command = *ptr++;
    }
    return points;
}
*/

class Path
{
public:
    void moveTo(double, double);
    void lineTo(double, double);
    void cubicTo(double x1, double y1, double x2, double y2, double x3, double y3);
    void quadTo(double cx, double cy, double x1, double y1, double x2, double y2);
    void arcTo(double cx, double cy, double rx, double ry, double xAxisRotation, bool largeArcFlag, bool sweepFlag, double x, double y);


    std::vector<cv::Point2f> m_points;
};

inline void Path::moveTo(double x, double y)
{
    m_points.emplace_back(x, y);
}

inline void Path::lineTo(double x, double y)
{
    m_points.emplace_back(x, y);
}

inline void Path::cubicTo(double x1, double y1, double x2, double y2, double x3, double y3)
{
    m_points.emplace_back(x1, y1);
    m_points.emplace_back(x2, y2);
    m_points.emplace_back(x3, y3);
}

inline void Path::quadTo(double cx, double cy, double x1, double y1, double x2, double y2)
{
    auto cx1 = 2.0 / 3.0 * x1 + 1.0 / 3.0 * cx;
    auto cy1 = 2.0 / 3.0 * y1 + 1.0 / 3.0 * cy;
    auto cx2 = 2.0 / 3.0 * x1 + 1.0 / 3.0 * x2;
    auto cy2 = 2.0 / 3.0 * y1 + 1.0 / 3.0 * y2;
    cubicTo(cx1, cy1, cx2, cy2, x2, y2);
}
static const double pi = 3.14159265358979323846;

inline void Path::arcTo(double cx, double cy, double rx, double ry, double xAxisRotation, bool largeArcFlag, bool sweepFlag, double x, double y)
{
    rx = std::fabs(rx);
    ry = std::fabs(ry);

    auto sin_th = std::sin(xAxisRotation * pi / 180.0);
    auto cos_th = std::cos(xAxisRotation * pi / 180.0);

    auto dx = (cx - x) / 2.0;
    auto dy = (cy - y) / 2.0;
    auto dx1 =  cos_th * dx + sin_th * dy;
    auto dy1 = -sin_th * dx + cos_th * dy;
    auto Pr1 = rx * rx;
    auto Pr2 = ry * ry;
    auto Px = dx1 * dx1;
    auto Py = dy1 * dy1;
    auto check = Px / Pr1 + Py / Pr2;
    if(check > 1)
    {
        rx = rx * std::sqrt(check);
        ry = ry * std::sqrt(check);
    }

    auto a00 =  cos_th / rx;
    auto a01 =  sin_th / rx;
    auto a10 = -sin_th / ry;
    auto a11 =  cos_th / ry;
    auto x0 = a00 * cx + a01 * cy;
    auto y0 = a10 * cx + a11 * cy;
    auto x1 = a00 * x + a01 * y;
    auto y1 = a10 * x + a11 * y;
    auto d = (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0);
    auto sfactor_sq = 1.0 / d - 0.25;
    if(sfactor_sq < 0) sfactor_sq = 0;
    auto sfactor = std::sqrt(sfactor_sq);
    if(sweepFlag == largeArcFlag) sfactor = -sfactor;
    auto xc = 0.5 * (x0 + x1) - sfactor * (y1 - y0);
    auto yc = 0.5 * (y0 + y1) + sfactor * (x1 - x0);

    auto th0 = std::atan2(y0 - yc, x0 - xc);
    auto th1 = std::atan2(y1 - yc, x1 - xc);

    double th_arc = th1 - th0;
    if(th_arc < 0.0 && sweepFlag)
        th_arc += 2.0 * pi;
    else if(th_arc > 0.0 && !sweepFlag)
        th_arc -= 2.0 * pi;

    auto n_segs = static_cast<int>(std::ceil(std::fabs(th_arc / (pi * 0.5 + 0.001))));
    for(int i = 0;i < n_segs;i++)
    {
        auto th2 = th0 + i * th_arc / n_segs;
        auto th3 = th0 + (i + 1) * th_arc / n_segs;

        auto a00 =  cos_th * rx;
        auto a01 = -sin_th * ry;
        auto a10 =  sin_th * rx;
        auto a11 =  cos_th * ry;

        auto thHalf = 0.5 * (th3 - th2);
        auto t = (8.0 / 3.0) * std::sin(thHalf * 0.5) * std::sin(thHalf * 0.5) / std::sin(thHalf);
        auto x1 = xc + std::cos(th2) - t * std::sin(th2);
        auto y1 = yc + std::sin(th2) + t * std::cos(th2);
        auto x3 = xc + std::cos(th3);
        auto y3 = yc + std::sin(th3);
        auto x2 = x3 + t * std::sin(th3);
        auto y2 = y3 - t * std::cos(th3);

        auto cx1 = a00 * x1 + a01 * y1;
        auto cy1 = a10 * x1 + a11 * y1;
        auto cx2 = a00 * x2 + a01 * y2;
        auto cy2 = a10 * x2 + a11 * y2;
        auto cx3 = a00 * x3 + a01 * y3;
        auto cy3 = a10 * x3 + a11 * y3;
        cubicTo(cx1, cy1, cx2, cy2, cx3, cy3);
    }
}

inline std::vector<cv::Point2f> parseRect(  std::string sw,std::string sh,std::string sx,std::string sy){

    float w=std::stof(sw);
    float h=std::stof(sh);
    float x=std::stof(sx);
    float y=std::stof(sy);
    return std::vector<cv::Point2f>{ {x,y},{x+w,y},{x+w,y+h},{x,y+h} };
}




inline std::vector<cv::Point2f> parsePath(  std::string  spath)
{
    Path path;

    const char *ptr = spath.data();
    auto end = ptr + spath.size();
    if(ptr >= end || !(*ptr == 'M' || *ptr == 'm'))
        return path.m_points;

    auto command = *ptr++;
    double c[6];
    bool f[2];

    cv::Point2f startPoint;
    cv::Point2f currentPoint;
    cv::Point2f controlPoint;

    while(true)
    {
        skipWs(ptr, end);
        switch(command) {
        case 'M':
        case 'm':
            if(!parseNumberList(ptr, end, c, 2))
                break;

            if(command == 'm')
            {
                c[0] += currentPoint.x;
                c[1] += currentPoint.y;
            }

            path.moveTo(c[0], c[1]);
            startPoint.x = currentPoint.x = controlPoint.x = c[0];
            startPoint.y = currentPoint.y = controlPoint.y = c[1];
            command = command == 'm' ? 'l' : 'L';
            break;
        case 'L':
        case 'l':
            if(!parseNumberList(ptr, end, c, 2))
                break;

            if(command == 'l')
            {
                c[0] += currentPoint.x;
                c[1] += currentPoint.y;
            }

            path.lineTo(c[0], c[1]);
            currentPoint.x = controlPoint.x = c[0];
            currentPoint.y = controlPoint.y = c[1];
            break;
        case 'Q':
        case 'q':
            if(!parseNumberList(ptr, end, c, 4))
                break;

            if(command == 'q')
            {
                c[0] += currentPoint.x;
                c[1] += currentPoint.y;
                c[2] += currentPoint.x;
                c[3] += currentPoint.y;
            }

            path.quadTo(currentPoint.x, currentPoint.y, c[0], c[1], c[2], c[3]);
            controlPoint.x = c[0];
            controlPoint.y = c[1];
            currentPoint.x = c[2];
            currentPoint.y = c[3];
            break;
        case 'C':
        case 'c':
            if(!parseNumberList(ptr, end, c, 6))
                break;

            if(command == 'c')
            {
                c[0] += currentPoint.x;
                c[1] += currentPoint.y;
                c[2] += currentPoint.x;
                c[3] += currentPoint.y;
                c[4] += currentPoint.x;
                c[5] += currentPoint.y;
            }

            path.cubicTo(c[0], c[1], c[2], c[3], c[4], c[5]);
            controlPoint.x = c[2];
            controlPoint.y = c[3];
            currentPoint.x = c[4];
            currentPoint.y = c[5];
            break;
        case 'T':
        case 't':
            c[0] = 2 * currentPoint.x - controlPoint.x;
            c[1] = 2 * currentPoint.y - controlPoint.y;
            if(!parseNumberList(ptr, end, c + 2, 2))
                break;

            if(command == 't')
            {
                c[2] += currentPoint.x;
                c[3] += currentPoint.y;
            }

            path.quadTo(currentPoint.x, currentPoint.y, c[0], c[1], c[2], c[3]);
            controlPoint.x = c[0];
            controlPoint.y = c[1];
            currentPoint.x = c[2];
            currentPoint.y = c[3];
            break;
        case 'S':
        case 's':
            c[0] = 2 * currentPoint.x - controlPoint.x;
            c[1] = 2 * currentPoint.y - controlPoint.y;
            if(!parseNumberList(ptr, end, c + 2, 4))
                break;

            if(command == 's')
            {
                c[2] += currentPoint.x;
                c[3] += currentPoint.y;
                c[4] += currentPoint.x;
                c[5] += currentPoint.y;
            }

            path.cubicTo(c[0], c[1], c[2], c[3], c[4], c[5]);
            controlPoint.x = c[2];
            controlPoint.y = c[3];
            currentPoint.x = c[4];
            currentPoint.y = c[5];
            break;
        case 'H':
        case 'h':
            if(!parseNumberList(ptr, end, c, 1))
                break;

            if(command == 'h')
               c[0] += currentPoint.x;

            path.lineTo(c[0], currentPoint.y);
            currentPoint.x = controlPoint.x = c[0];
            break;
        case 'V':
        case 'v':
            if(!parseNumberList(ptr, end, c + 1, 1))
                break;

            if(command == 'v')
               c[1] += currentPoint.y;

            path.lineTo(currentPoint.x, c[1]);
            currentPoint.y = controlPoint.y = c[1];
            break;
        case 'A':
        case 'a':
            if(!parseNumberList(ptr, end, c, 3)
                    || !parseArcFlag(ptr, end, f[0])
                    || !parseArcFlag(ptr, end, f[1])
                    || !parseNumberList(ptr, end, c + 3, 2))
                break;

            if(command == 'a')
            {
               c[3] += currentPoint.x;
               c[4] += currentPoint.y;
            }

            path.arcTo(currentPoint.x, currentPoint.y, c[0], c[1], c[2], f[0], f[1], c[3], c[4]);
            currentPoint.x = controlPoint.x = c[3];
            currentPoint.y = controlPoint.y = c[4];
            break;
        case 'Z':
        case 'z':
            currentPoint.x = controlPoint.x = startPoint.x;
            currentPoint.y = controlPoint.y = startPoint.y;
            break;
        default:
            break;
        }

        skipWsDelimiter(ptr, end, ',');
        if(ptr >= end)
            break;

        if(IS_ALPHA(*ptr))
            command = *ptr++;
    }

    return path.m_points;
}

inline  cv::Point2f  getPathCenter(  std::string  spath){
    auto path=parsePath(spath);
    cv::Point2f center{0,0};
    for(auto p:path) center+=p;
    center/=float(path.size());
    return center;
}


inline cv::Point2f getRectCenter(  std::string sw,std::string sh,std::string sx,std::string sy){

    float w=std::stof(sw);
    float h=std::stof(sh);
    float x=std::stof(sx);
    float y=std::stof(sy);
    return  {x+w/2.f,y+h/2.f};
}

#endif
