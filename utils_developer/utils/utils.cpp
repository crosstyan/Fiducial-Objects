#ifndef _CUCOMARKER_EXTRAS_CPP_
#define _CUCOMARKER_EXTRAS_CPP_

#include "utils.hpp"

using namespace std;

// CmdLineParser

bool CmdLineParser::operator[](string param){
  int idx = -1;
  for (int i = 0; i < argc && idx == -1; i++)
    if (string(argv[i]) == param)
      idx = i;
  return (idx != -1);
}

string CmdLineParser::operator()(string param, string defvalue){
  int idx = -1;
  for (int i = 0; i < argc && idx == -1; i++)
    if (string(argv[i]) == param)
      idx = i;
    if (idx == -1)
      return defvalue;
    else
      return (argv[idx + 1]);
}


// TimerAvrg

TimerAvrg:: TimerAvrg(int _n){
    n=_n;times.reserve(n);
}

double TimerAvrg::getAvrg(){
    double sum=0;
    for(auto t:times)
        sum+=t;
    return sum/double(times.size());
}

#endif