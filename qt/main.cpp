#include <QtWidgets/QApplication>
#include <QDesktopWidget>
#include <QtCore/QDebug>

#include <QtGui/QIcon>
#include <QSettings>
#include "fiducialobjectqt.h"

#ifdef WIN32
#include <windows.h>
void freeConsole() {
    FreeConsole();
}
#else
void freeConsole() {}
#endif
using namespace  std;
class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( std::string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( std::string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                    std::string operator()(std::string param,std::string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( std::string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };



int main( int argc, char* argv[] )
{
    CmdLineParser cml(argc,argv);
    QApplication oApp( argc, argv );
    QCoreApplication::setOrganizationName("University of Cordoba");
    QCoreApplication::setApplicationName("FiducialObject");
    if(cml["-settings.clear"]){
        qDebug()<<"Settings clear";
        QSettings set;
        set.clear();
        QApplication::exit(0);
        return 0;
    }

    if(!cml["-debug"])
        freeConsole();

    FiducialObjectQt *wdg=new FiducialObjectQt();
    wdg->resize(800,800);
     wdg->move(QApplication::desktop()->screen()->rect().center() - wdg->rect().center());
     wdg->show();
    oApp.exec();


}
