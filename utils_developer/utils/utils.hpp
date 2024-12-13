#ifndef _CUCOMARKER_EXTRAS_HPP_
#define _CUCOMARKER_EXTRAS_HPP_

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <chrono>

using namespace std;

class CmdLineParser{
  int argc;
  char** argv;
  public:
    CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}
    bool operator[](string param);
    string operator()(string param, string defvalue = "-1");
};

struct TimerAvrg{
    std::vector<double> times;
    size_t curr=0,n;
    std::chrono::high_resolution_clock::time_point begin,end;
    TimerAvrg(int _n=30);
    inline void start(){
        begin= std::chrono::high_resolution_clock::now();
    }
    inline void stop(){
        end= std::chrono::high_resolution_clock::now();
        double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;
        if (times.size()<n)
            times.push_back(duration);
        else{
            times[curr]=duration;
            curr++;
            if (curr>=times.size())
                curr=0;
        }
    }
    double getAvrg();
};

#endif