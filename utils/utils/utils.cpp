#ifndef _APPS_UTILS_
#define _APPS_UTILS_

#include "colors.hpp"
#include <opencv2/core/core.hpp>

// Alerts
void printAlert(cv::String message){
    std::cout<<BRED<<"[!] "+message+" [!]"<<RESET<<std::endl;
}


#endif