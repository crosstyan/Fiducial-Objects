#include "utils.hpp"


#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <filesystem>
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Geometry operations
~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// obtainCenter2D
cv::Point2f obtainCenter2D(std::vector<cv::Point2f> corners){
    double acc_x, acc_y;
    acc_x = 0.0; acc_y = 0.0;
    for(int h = 0; h < corners.size(); h++){
        acc_x += corners[h].x;
        acc_y += corners[h].y;
    }
    cv::Point2f center2D = cv::Point2f(acc_x/corners.size(), acc_y/corners.size());
    return center2D;
}


// obtainCenter3D
cv::Point3f obtainCenter3D(std::vector<cv::Point3f> corners){
    double acc_x = 0;
    double acc_y = 0;
    double acc_z = 0;
    double numC = 0.0;
    for(int c = 0; c < corners.size(); c++){
        acc_x += corners[c].x;
        acc_y += corners[c].y;
        acc_z += corners[c].z;
        numC += 1.0;
    }
    cv::Point3f center3D = cv::Point3f(acc_x/corners.size(), acc_y/corners.size(), acc_z/corners.size());
    return center3D;
}

// applyRvecToPoint
cv::Point3f applyRvecToPoint(cv::Mat rvec, cv::Point3f point){

    cv::Mat rotationMatrix(3, 3, CV_64F); cv::Rodrigues(rvec, rotationMatrix); 
        std::cout<<"Rodrigues: "<<rotationMatrix<<std::endl; 
        std::cout<<"Rodrigues type: "<<rotationMatrix.type()<<std::endl; 
    cv::Mat pointMat(point); pointMat.convertTo(pointMat, CV_64F);
        std::cout<<"Result: "<<point<<std::endl;
        std::cout<<"Result mat: "<<pointMat<<std::endl;
        std::cout<<"Result mat type: "<<pointMat.type()<<std::endl;
    cv::Mat result = rotationMatrix*pointMat;
        std::cout<<"Result mat: "<<result<<std::endl;
    cv::Point3d resultPD(result.at<double>(0,0),result.at<double>(1,0),result.at<double>(2,0));
        std::cout<<"Result point double: "<<resultPD<<std::endl;
    cv::Point3f resultPF(resultPD);
        std::cout<<"Result point float: "<<resultPF<<std::endl;


    //cv::waitKey();
    return resultPF;
}



// movePointToCamera
cv::Point3f movePointToCamera(cv::Mat rvec, cv::Mat tvec, cv::Point3f point){

    // Get camera-to-world transformation matrix
    cv::Mat m33; cv::Rodrigues(rvec, m33);
    cv::Mat m(cv::Size(4,4), CV_64F);
    m.at<double>(0,0) = m33.at<double>(0,0);
    m.at<double>(0,1) = m33.at<double>(0,1);
    m.at<double>(0,2) = m33.at<double>(0,2);
    m.at<double>(0,3) = tvec.at<double>(0,0);
    m.at<double>(1,0) = m33.at<double>(1,0);
    m.at<double>(1,1) = m33.at<double>(1,1);
    m.at<double>(1,2) = m33.at<double>(1,2);
    m.at<double>(1,3) = tvec.at<double>(0,1);
    m.at<double>(2,0) = m33.at<double>(2,0);
    m.at<double>(2,1) = m33.at<double>(2,1);
    m.at<double>(2,2) = m33.at<double>(2,2);
    m.at<double>(2,3) = tvec.at<double>(0,2);
    m.at<double>(3,0) = 0.0;
    m.at<double>(3,1) = 0.0;
    m.at<double>(3,2) = 0.0;
    m.at<double>(3,3) = 1.0;
    cv::Mat cameraToWorldMatrix = m;

    // Get the inverse to obtain the world-to-camera matrix
    cv::Mat worldToCameraMatrix = cameraToWorldMatrix.inv();


    // Apply matrix to point
    cv::Vec4d vec4d(point.x, point.y, point.z, 1);

    std::cout<<"Point: "<<point<<std::endl;
    std::cout<<"Vec4d: "<<vec4d<<std::endl;

    cv::Mat matp(vec4d); matp.convertTo(matp, CV_64F);
    std::cout<<"Matp: "<<matp<<std::endl;

    // Apply mat
    cv::Mat res = worldToCameraMatrix * matp;
    //cv::Mat res = cameraToWorldMatrix * matp;
    std::cout<<"Res: "<<res<<std::endl;


    // Convert to point
    cv::Point3d resultPD(res.at<double>(0,0),res.at<double>(0,1),res.at<double>(0,2));
    cv::Point3f resultPF(resultPD);

    return resultPF;
}




double computePercentageDifference(double a, double b) {
    double absDiff = std::abs(a - b);
    double avg = (a + b) / 2.0;
    return (absDiff / avg) * 100.0;
}




// getPoseLine
std::string getPoseLine(int id, cv::Mat rvec, cv::Mat tvec, bool verboseDebug){
    cv::Mat m33; cv::Rodrigues(rvec, m33);
    cv::Mat m(cv::Size(4,4), CV_64F);
    m.at<double>(0,0) = m33.at<double>(0,0);
    m.at<double>(0,1) = m33.at<double>(0,1);
    m.at<double>(0,2) = m33.at<double>(0,2);
    m.at<double>(0,3) = tvec.at<double>(0,0);
    m.at<double>(1,0) = m33.at<double>(1,0);
    m.at<double>(1,1) = m33.at<double>(1,1);
    m.at<double>(1,2) = m33.at<double>(1,2);
    m.at<double>(1,3) = tvec.at<double>(0,1);
    m.at<double>(2,0) = m33.at<double>(2,0);
    m.at<double>(2,1) = m33.at<double>(2,1);
    m.at<double>(2,2) = m33.at<double>(2,2);
    m.at<double>(2,3) = tvec.at<double>(0,2);
    m.at<double>(3,0) = 0.0;
    m.at<double>(3,1) = 0.0;
    m.at<double>(3,2) = 0.0;
    m.at<double>(3,3) = 1.0;
    m = m.inv();


    // if(verboseDebug){
    //     std::cout<<YELLOW<<"--------"<<std::endl;
    //     std::cout<<"rvec: "<<rvec<<std::endl<<"--------"<<std::endl;
    //     std::cout<<"rodri: "<<m33<<std::endl<<"--------"<<std::endl;
    //     std::cout<<"tvec: "<<tvec<<std::endl<<"--------"<<std::endl;
    //     std::cout<<"m: "<<m<<std::endl<<"--------"<<RESET<<std::endl;
    // }


    cv::Mat M = m;
    cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
    //use now eigen
    Eigen::Matrix3f e_r33;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            e_r33(i,j)=M.at<double>(i,j);

    //now, move to a angle axis
    Eigen::Quaternionf q(e_r33);
    double qx=q.x();
    double qy=q.y();
    double qz=q.z();
    double qw=q.w();
    double tx=M.at<double>(0,3);
    double ty=M.at<double>(1,3);
    double tz=M.at<double>(2,3);
    
    /*
        Format to print the information
        (given by optitrack pose files)
        id_frame tx ty tz qx qy qz qw
    */
    std::string poseLine = std::to_string(id)+" ";
    poseLine += std::to_string(tx)+" ";
    poseLine += std::to_string(-1*ty)+" ";
    poseLine += std::to_string(tz)+" ";
    poseLine += std::to_string(qx)+" ";
    poseLine += std::to_string(qy)+" ";
    poseLine += std::to_string(qz)+" ";
    poseLine += std::to_string(qw)+"\n";

    return poseLine;
}




/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Polyhuco config file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*

std::string name;
double markerSize;
std::string markerConfig;
std::string markerMap;
std::string detectionMode;
int minId;
int maxId;
std::string commentaries;

*/
// readPolyhucoConfig
polyhucoConfig readPolyhucoConfig(std::string configFileFolder){
    // Config creation
    polyhucoConfig config;
    std::string configFile = configFileFolder+"/config.yml";

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Configuration creation
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    // Raw values
    cv::FileStorage fs; fs.open(configFile, cv::FileStorage::READ);
    config.name = (std::string) fs["name"];
    config.markerSize = (double) fs["markerSize"];
    config.markerConfig = (std::string) fs["markerConfig"];
    config.markerMap = (std::string) fs["markerMap"];
    config.detectionMode = (std::string) fs["detectionMode"];
    std::string idsRange = (std::string) fs["idsRange"];
    config.commentaries = (std::string) fs["commentaries"];
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Configuration processing
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    // Process ids range
    int pos = -1;
    for(int spos = 0; spos < idsRange.length(); spos++){
        if(idsRange[spos] == '-'){
            pos = spos;
            break;
        }
    }
    if(pos != -1){
        config.minId = std::stoi(  idsRange.substr(0, pos)  );
        config.maxId = std::stoi(  idsRange.substr(pos+1, idsRange.length())  );
    }
    // Compute correct access to markerConfig file
    config.markerConfig = configFileFolder+"/"+config.markerConfig;
    // Compute correct access to markerMap file
    config.markerMap = configFileFolder+"/"+config.markerMap;


    // Return config
    return config;
}
// printPolyhucoConfig
void printPolyhucoConfig(polyhucoConfig config){
    std::cout<<BGREEN<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Polyhuco config ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
    std::cout<<YELLOW<<"- Name: "<<RESET<<config.name<<std::endl;
    std::cout<<GREEN<<"- Marker size: "<<RESET<<config.markerSize<<std::endl;
    std::cout<<GREEN<<"- Marker config: "<<RESET<<config.markerConfig<<std::endl;
    std::cout<<GREEN<<"- Marker map: "<<RESET<<config.markerMap<<std::endl;
    std::cout<<GREEN<<"- Detection mode: "<<RESET<<config.detectionMode<<std::endl;
    std::cout<<CYAN<<"- Id min: "<<RESET<<config.minId<<std::endl;
    std::cout<<CYAN<<"- Id max: "<<RESET<<config.maxId<<std::endl;
    std::cout<<MAGENTA<<"- Commentaries: "<<RESET<<config.commentaries<<std::endl;
    std::cout<<BGREEN<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<std::endl;
}

void fiducialObjectConfigGeneration(std::vector<std::string> fo_markers_files, std::string fo_map_path, std::string fo_output_path){
   // Getting files
   std::ofstream ofile(fo_output_path);
   if(!ofile)throw std::runtime_error("Could not open file:"+fo_output_path);
   // Generating output file
   ofile<<"__MARKERS_"<<fo_markers_files.size()<<"__"<<std::endl;
   for(int i = 0; i < fo_markers_files.size(); i++){
        std::ifstream file(fo_markers_files[i]);
        if(!file)throw std::runtime_error("Could not open file:"+fo_markers_files[i]);
        ofile<<"_MARKER_"<<i<<"_BEGIN_"<<std::endl;
        ofile<<file.rdbuf();
        ofile<<"_MARKER_"<<i<<"_END_"<<std::endl;
   }
   ofile<<"__MAP__"<<std::endl;
   std::ifstream file(fo_map_path);
   if(!file)throw std::runtime_error("Could not open file:"+fo_map_path);
   ofile<<file.rdbuf();
}

