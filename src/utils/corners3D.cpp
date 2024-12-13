
std::vector<cv::Point3f> compute3DCornersSchema(std::string config){

    std::string configurationToUse = config;

    // Load 3D corners schema
    cv::FileStorage fs(configurationToUse, cv::FileStorage::READ);
//    cv::FileStorage fs(configurationToUse, cv::FileStorage::READ);
    cv::FileNode corners = fs["marker"]["corners"];
    cv::FileNodeIterator it = corners.begin(), it_end = corners.end(); // Go through the node

    std::vector<cv::Point3f> corners3D;
    for (; it != it_end; ++it){
        // Read each point
        cv::FileNode pt = *it;
        cv::Point3d point;
        cv::FileNodeIterator itPt = pt.begin();
        point.x = *itPt; ++itPt;
        point.y = *itPt;
        corners3D.push_back(point);

        //std::cout<<"Point: "<<point<<std::endl;
    }

    /*
        Set the point (0,0) in the center
    */
    // Move the points to the top left corner
        // Obtain quantity
        float min_x = 0;
        float min_y = 0;
        for(auto c: corners3D){
            if((min_x == 0) || (c.x < min_x)){
                min_x = c.x;
            }
            if((min_y == 0) || (c.y < min_y)){
                min_y = c.y;
            }
        }
        // Move
        std::vector<cv::Point3f> newCorners3D;
        for(auto c: corners3D){
            newCorners3D.push_back(cv::Point3f(c.x - min_x, c.y - min_y, 0));
        }
        corners3D = newCorners3D;
    // Obtain center point now
    float max_x = -1;
    float max_y = -1;
    for(auto c: corners3D){
        if(c.x > max_x){
            max_x = c.x;
        }
        if(c.y > max_y){
            max_y = c.y;
        }
    }
    // Rest quantities to move every point
    newCorners3D.clear();
    for(auto c: corners3D){
        newCorners3D.push_back(cv::Point3f(c.y - max_y/2., c.x - max_x/2., 0));
    }

    return newCorners3D;
}

std::vector<cv::Point3f> getMarkers3DCornersSchema(float size, std::string config){


    std::vector<cv::Point3f> corners3D, newCorners3D;
    corners3D = compute3DCornersSchema(config);


    // Size to use
    float sizeToUse = size;


    // Rest quantities to move every point
    newCorners3D.clear();
    float scale = cv::norm(corners3D[0]-corners3D[1])/sizeToUse;
    for(auto c: corners3D){
        newCorners3D.push_back(c/scale);
    }


    corners3D = newCorners3D;

//    for(auto c: corners3D){
//        std::cout<<c<<std::endl;
//    }

    return corners3D;
}
