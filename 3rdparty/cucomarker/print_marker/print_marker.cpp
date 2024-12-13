#include <plutosvg.h>
#include <iostream>
#include "svgreader.h"

using namespace std;

class CmdLineParser{int argc;char** argv;public:CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}   bool operator[](string param)    {int idx = -1;  for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;return (idx != -1);}    string operator()(string param, string defvalue = "-1")    {int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;if (idx == -1)return defvalue;else return (argv[idx + 1]);}};


int main(int argc, char** argv){
    try{
        CmdLineParser cml(argc, argv);

        if(argc<5 || cml["-h"])
        {
            std::cerr << "Use: marker_id svg_file outfile(.png) config(.yml) [-e final_marker] [-ncrcbits <int:-1|8|16|32>|-1 auto]  [-dpi <double>|180.0]" <<std::endl;
            return -1;
        }

        int markerId=atoi(argv[1]);
        if(markerId<=0) throw std::runtime_error("marker_id should be higher than 0");

        int finalMarker= stoi(cml("-e",std::to_string(markerId)));
        if(finalMarker<markerId) throw std::runtime_error("finalMarker should be >= marker_id ");

        

        cucomarker::SVGFile svg;
        int ncrcbits=stoi(cml("-ncrcbits","-1"));
        auto maxId=svg.load(argv[2],ncrcbits);
        cout<<"Number of Markers="<<maxId<<" nbits="<<log2(maxId)<<endl;
        double dpi=stod(cml("-dpi","180.0"));


        //prints a single marker
        if(markerId==finalMarker){
            svg.save(markerId,"tmp.svg");
            plutovg_surface_t* surface = plutosvg_load_from_file("tmp.svg", NULL, 0, 0, dpi);
            if(surface == NULL) throw std::runtime_error("Load failed");
            plutovg_surface_write_to_png(surface, argv[3]);
            plutovg_surface_destroy(surface);
            //remove("tmp.svg");
        }
        else{//prints multiple markers
            for(;markerId<=finalMarker;markerId++){
                svg.save(markerId,"tmp.svg");
                plutovg_surface_t* surface = plutosvg_load_from_file("tmp.svg", NULL, 0, 0, dpi);
                if(surface == NULL) throw std::runtime_error("Load failed");
                std::string number=std::to_string(markerId);
                while(number.size()<4) number="0"+number;
                std::string fname=std::string(argv[3])+number+".png";
                cout<<"saving ... "<<fname<<endl;
                plutovg_surface_write_to_png(surface, fname.c_str());
                plutovg_surface_destroy(surface);
            //    remove("tmp.svg");
            }
        }


        svg.exportConfigFile(argv[4]);


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
