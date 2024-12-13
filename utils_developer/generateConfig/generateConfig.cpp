/*~~~~~~~~~~~~~~~
    Includes
~~~~~~~~~~~~~~~*/
// Library includes
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <vector>
#include <cmath> 
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;


// writeToFile
void writeToFile(const std::vector<std::string>& lines, const std::string& filePath) {
    std::ofstream file;

    file.open(filePath, std::ios::app); // Open the file in append mode

    // Check if the file is open
    if (!file) {
        std::cerr << "Failed to open the file.";
        return;
    }

    // Write each line from the vector to the file
    for (const auto& line : lines) {
        file << line << "\n";
    }

    file.close(); // Close the file
}

// readFromFile
std::vector<std::string> readFromFile(const std::string& filePath) {
    std::ifstream file(filePath); // Open the file
    std::vector<std::string> lines;
    std::string line;

    // Check if the file is open
    if (!file) {
        std::cerr << "Failed to open the file.";
        return lines;
    }

    // Read each line from the file into the vector
    while (std::getline(file, line)) {
        lines.push_back(line);
    }

    file.close(); // Close the file

    return lines;
}

// splitString
std::vector<std::string> splitString(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::stringstream ss(str);

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

// Parameters
const cv::String keys =
"{help h usage ? | | Print this message}"
"{@markerParamPaths |<none>| Marker params paths.}"
"{@mapPath |<none>| Map path.}"
"{outputFile |example.yml| Default: example.yml.}"
;

// Main
int main(int argc, char**argv){

    // Reading the parameters
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Developer Fiducial object example application.");
    if(parser.has("help")){
        parser.printMessage();
        return 0;
    }
    if(argc < 3){
        parser.printMessage(); return -1;
    }

    // Getting files
    std::vector<std::string> markerConfigurationsPaths = splitString(parser.get<std::string>("@markerParamPaths"), ',');
    std::string mapPath = parser.get<std::string>("@mapPath");
    std::string outputFile = parser.get<std::string>("outputFile");
    if (std::filesystem::exists(outputFile)) 
        std::filesystem::remove(outputFile);

    // Generating output file
    writeToFile({"__MARKERS_"+std::to_string(markerConfigurationsPaths.size())+"__"}, outputFile);
    for(int i = 0; i < markerConfigurationsPaths.size(); i++){
        std::string markerConfig = markerConfigurationsPaths[i];
        writeToFile({"_MARKER_"+std::to_string(i)+"_BEGIN_"}, outputFile);
        writeToFile(readFromFile(markerConfig), outputFile);
        writeToFile({"_MARKER_"+std::to_string(i)+"_END_"}, outputFile);
    }
    writeToFile({"__MAP__"}, outputFile);
    writeToFile(readFromFile(mapPath), outputFile);

}



