#include <iostream>
#include "cucomarker.h"

using namespace std;
class CmdLineParser{int argc;char** argv;public:CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}   bool operator[](string param)    {int idx = -1;  for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;return (idx != -1);}    string operator()(string param, string defvalue = "-1")    {int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;if (idx == -1)return defvalue;else return (argv[idx + 1]);}};

struct   TimerAvrg{std::vector<double> times;size_t curr=0,n; std::chrono::high_resolution_clock::time_point begin,end;   TimerAvrg(int _n=30){n=_n;times.reserve(n);   }inline void start(){begin= std::chrono::high_resolution_clock::now();    }inline void stop(){end= std::chrono::high_resolution_clock::now();double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;if ( times.size()<n) times.push_back(duration);else{ times[curr]=duration; curr++;if (curr>=times.size()) curr=0;}}double getAvrg(){double sum=0;for(auto t:times) sum+=t;return sum/double(times.size());}};
TimerAvrg Fps;


int main(int argc, char **argv)
{
    CmdLineParser cml(argc, argv);

    cout << "[live:<index>|videoFile] config.yml [-skip] [-c colorDetectionMode: ALL|HSV|GRAY (default ALL)] [-showCandidates]" << endl;

    string TheInputVideo = argv[1];
    cv::VideoCapture TheVideoCapturer;
    int waitTime=10;
    bool isVideo=false;

    ///////////  OPEN VIDEO
    ///
    // read from camera or from  file
    if (TheInputVideo.find("live") != string::npos)
    {
        int vIdx = 0;
        // check if the :idx is here
        char cad[100];
        if (TheInputVideo.find(":") != string::npos)
        {
            std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
            sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
        }
        cout << "Opening camera index " << vIdx << endl;
        TheVideoCapturer.open(vIdx);
        TheVideoCapturer.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        TheVideoCapturer.set(cv::CAP_PROP_FRAME_WIDTH,1920);
        TheVideoCapturer.set(cv::CAP_PROP_FRAME_HEIGHT,1080);
        waitTime = 10;
        isVideo=true;
    }
    else{
        std::cout << "Opening video" << std::endl;
        TheVideoCapturer.open(TheInputVideo);
        if ( TheVideoCapturer.get(cv::CAP_PROP_FRAME_COUNT)>=2) isVideo=true;
        if(cml["-skip"])
            TheVideoCapturer.set(cv::CAP_PROP_POS_FRAMES,stoi(cml("-skip")));
    }
    // check video is open
    if (!TheVideoCapturer.isOpened())
        throw std::runtime_error("Could not open video");

    cv::Mat image;
    TheVideoCapturer >> image;

    cucomarker::MarkerDetector MDetector;
    MDetector.loadConfigFile(argv[2]);
    // MDetector.loadConfigFile("/home/paul/Workspace/Cucomarker/PrivateRepository/CucomarkerReworked/samples/Triangles/TRIANGLE2/config.yml");
    MDetector.setColorDetectionMode(cml("-c", "GRAY"));


    bool showCandidates=cml["-showCandidates"];
    char key = 0;
    int frameId;
    do{

        TheVideoCapturer.retrieve(image);
        cv::resize(image,image,cv::Size(float(image.cols)*0.7,float(image.rows)*0.7));

        frameId = TheVideoCapturer.get(cv::CAP_PROP_POS_FRAMES);

        Fps.start();
        std::vector<cucomarker::Marker> markers=MDetector.detect(image);
        Fps.stop();

        std::cout << "\rFrame "<<frameId<<", time detection=" << Fps.getAvrg()*1000 << " ms, nmarkers=" << markers.size() <<" images resolution="<<image.size() <<std::endl;

        for(auto m:markers) m.draw(image, cv::Scalar(0,0,255));
        if(showCandidates)
            for(auto m:MDetector.getCandidates())
                m.draw(image, cv::Scalar(0,0,255));


        cv::imshow("image", image);


        // key=cv::waitKey();
        key=cv::waitKey(waitTime);

        if (key == 's')
            waitTime = waitTime == 0 ? 10 : 0;
        if (key == 'w')
            cv::imwrite("image.jpg",image);

        if (isVideo)
            if ( TheVideoCapturer.grab()==false) key=27;


    }while(key!=27);

}
