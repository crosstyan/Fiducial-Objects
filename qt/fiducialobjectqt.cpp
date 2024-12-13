#include <filesystem>
#include <QFileInfo>
#include <QColor>
#include <QPalette>
#include <QTime>
#include <QSettings>
#include <QFileDialog>
#include <QMessageBox>
#include <QDesktopServices>
#include "arucocalibrate.h"
#include <aruco/aruco.h>
#include <plutosvg.h>
#include "cucomarker/print_marker/svgreader.h"
#include <markermapper/markermapper.h>
#include <markermapper/debug.h>
#include "../src/fiducialObject.h"
#include "../src/utils/markerDetectors/cucomarkerMarkerDetector.hpp"
#include "tools/qtgl/glviewer.h"
#include "drawers3d.h"
#include "fiducialobjectqt.h"
#include "ui_fiducialobjectqt.h"
#include <random>

namespace fs = std::filesystem;

/* Auxiliar functions */

// Checking file and folder inputs
enum class FileType {
    FOLDER,
    IMAGE,
    VIDEO,
    OTHER
};
struct FileInfo {
    FileType type;
    std::string name;
    std::string parentDir;
    std::string extension;
};
FileInfo checkFileType(const std::string &path) {
    QFileInfo fileInfo(QString::fromStdString(path));
    std::string fileName = fileInfo.fileName().toStdString();
    std::string parentDir = fileInfo.dir().absolutePath().toStdString();
    std::string extension = fileInfo.suffix().toLower().toStdString(); // convert extension to std::string

    if (fileInfo.isDir()) {
        return {FileType::FOLDER, fileName, parentDir, ""};
    } else if (fileInfo.isFile()) {
        if (extension == "jpg" || extension == "jpeg" || extension == "png" || extension == "bmp" || extension == "tif" || extension == "tiff") {
            return {FileType::IMAGE, fileName, parentDir, extension};
        } else if (extension == "avi" || extension == "mp4" || extension == "mov" || extension == "mkv" || extension == "flv") {
            cv::VideoCapture cap(path);
            if (cap.isOpened()) {
                cap.release();
                return {FileType::VIDEO, fileName, parentDir, extension};
            }
        }
    }

    return {FileType::OTHER, fileName, parentDir, extension};
}


// appendPlainText
void appendPlainText(QTextEdit *messageBox, std::string str){
    QTextCursor cursor = messageBox->textCursor();
    cursor.movePosition(QTextCursor::End);
    messageBox->setTextCursor(cursor);
    messageBox->insertPlainText((str + "\n").c_str());
}

// processImage
bool processImage(cv::Mat &image, fiducialObject::Object FObject, cv::Mat &K, cv::Mat &D, cv::Mat &tvec, cv::Mat &rvec){
    // Processing detection
    auto detection= fiducialObject::Detector::detect({FObject},image,K,D);
    if(detection.size() > 0){
        for(auto &d:detection)
            d.draw(image,K,D);
        // Extracting pose
        if(!detection[0].tvec.empty() and !detection[0].rvec.empty()){
            tvec = detection[0].tvec.clone();
            rvec = detection[0].rvec.clone();
        }
        else{
            // Return
            return false;
        }
        // Return
        return true;
    }
    // Return
    return false;
}

// Function to compute inverse transformation
void computeInverse(cv::Mat &tvec, cv::Mat &rvec) {
    // Convert rvec to rotation matrix
    cv::Mat m33;
    cv::Rodrigues(rvec, m33);
    // Construct 4x4 transformation matrix
    cv::Mat m(cv::Size(4,4), CV_64F);
    m.at<double>(0,0) = m33.at<double>(0,0);
    m.at<double>(0,1) = m33.at<double>(0,1);
    m.at<double>(0,2) = m33.at<double>(0,2);
    m.at<double>(0,3) = tvec.at<double>(0,0);
    m.at<double>(1,0) = m33.at<double>(1,0);
    m.at<double>(1,1) = m33.at<double>(1,1);
    m.at<double>(1,2) = m33.at<double>(1,2);
    m.at<double>(1,3) = tvec.at<double>(1,0);
    m.at<double>(2,0) = m33.at<double>(2,0);
    m.at<double>(2,1) = m33.at<double>(2,1);
    m.at<double>(2,2) = m33.at<double>(2,2);
    m.at<double>(2,3) = tvec.at<double>(2,0);
    m.at<double>(3,0) = 0.0;
    m.at<double>(3,1) = 0.0;
    m.at<double>(3,2) = 0.0;
    m.at<double>(3,3) = 1.0;
    // Invert the transformation matrix
    m = m.inv();
    // Extract the new rotation matrix and translation vector
    tvec = m(cv::Range(0,3), cv::Range(3,4));
    cv::Mat R_new = m(cv::Range(0,3), cv::Range(0,3));
    // Convert the new rotation matrix back to a rotation vector
    cv::Rodrigues(R_new, rvec);
}

// writePoseOnCSV
void writePoseOnCSV(std::string filePath, std::string id, cv::Mat &tvec, cv::Mat &rvec){
    std::fstream file;
    // Open the file for reading and writing
    file.open(filePath, std::ios_base::in | std::ios_base::out);
    bool empty = !file || file.peek() == std::ifstream::traits_type::eof();
    // Close the file after checking if it's empty
    file.close();

    // Reopen the file in append mode
    file.open(filePath, std::ios_base::out | std::ios_base::app);
    // Add headers if the file was empty
    if (empty)
    {
        file << "Identifier,Translation_X,Translation_Y,Translation_Z,Rotation_X,Rotation_Y,Rotation_Z\n";
    }
    // Compute inverse transformation
    computeInverse(tvec, rvec);
    if (file.is_open()) { // If the file is open, write the data to it
        file << id << "," << tvec.at<double>(0,0) << "," << tvec.at<double>(1,0) << "," << tvec.at<double>(2,0) << ","
             << rvec.at<double>(0,0) << "," << rvec.at<double>(1,0) << "," << rvec.at<double>(2,0) << "\n";
        file.flush(); // Flush the buffer to the file
    }
    else {
        std::cout << "Unable to open file";
    }
    // Close the file
    file.close();
}

/* Main functions */

FiducialObjectQt::FiducialObjectQt(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FiducialObjectQt)
{
    ui->setupUi(this);
    setWindowTitle("Custom Fiducial Object");
    setWindowIcon(QIcon ( ":/program_icon.ico" ));
    QSettings settings;
    ui->cc_le_imagesFolder->setText(settings.value( "cc_le_imagesFolder").toString());
    ui->_2_le_svg->setText(settings.value( "_2_le_svg").toString());
    ui->of_le_imagesFolder->setText(settings.value( "of_le_imagesFolder").toString());
    ui->_2_pb_cancel->hide();
    ui->_3_le_imagesFolder->setText(settings.value( "_3_le_imagesFolder").toString());
    ui->_4_le_input->setText(settings.value( "_4_le_input").toString());

}

FiducialObjectQt::~FiducialObjectQt()
{
    delete ui;
}


std::vector<std::string>  FiducialObjectQt::getImagesInFolder(const fs::path& folderPath) {
    auto convertToLowercase=[](const std::string& str)
    {
        std::string lowercaseStr;
        for (char c : str)
        {
            lowercaseStr += std::tolower(c);
        }
        return lowercaseStr;
    };
    std::vector<std::string> images;
    for (const auto& entry : fs::directory_iterator(folderPath)) {
        if (entry.is_regular_file()) {
            const fs::path& filePath = entry.path();
            std::string extension = convertToLowercase(filePath.extension().string());
            if (extension == ".png" || extension == ".jpg" || extension == ".jpeg" || extension == ".gif") {
                images.push_back(filePath.string());
            }
        }
    }
    return images;
};
void FiducialObjectQt::on_cc_pb_imagesfolder_clicked()
{

    QSettings settings;
    QString filepath = QFileDialog::getExistingDirectory( this,tr ( "Select intput file" ),settings.value ( "currDir" ).toString());
    if ( filepath==QString() ) return;
    settings.setValue ( "currDir",filepath );

    ui->cc_le_imagesFolder->setText(filepath);
    settings.setValue ( "cc_le_imagesFolder",filepath );

    //now, count the number of images in the folder
    ui->cc_la_noi2->setText(std::to_string(getImagesInFolder(filepath .toStdString()).size()).c_str());


}


void FiducialObjectQt::on_cc_pb_calibrate_clicked()
{
    try{
        if( ui->cc_le_imagesFolder->text().isEmpty())return;


        ui->pte_messages->clear();
        auto camp=arucoCalibrate(  ui->cc_le_imagesFolder->text().toStdString(),[this](std::string str){
            appendPlainText(ui->pte_messages,(str));
            QCoreApplication::processEvents();
        });
        camp.saveToFile( (fs::path(getOutputDir())/"camera.yml").string() );
        appendPlainText(ui->pte_messages,("DONE!"));
    } catch (const std::exception &ex) {
        QMessageBox::critical(this,tr("Error"),ex.what());
    }
}


void FiducialObjectQt::on__2_pb_svg_clicked()
{
 try{
    QSettings settings;
    QString filepath = QFileDialog::getOpenFileName(this,tr ( "Select SVG File " ),settings.value ( "_2_le_svg" ).toString(),"SVG File (*.svg)");
    if ( filepath==QString() ) return;
    settings.setValue ( "currDir",QFileInfo ( filepath ).absolutePath() );

    cucomarker::SVGFile svg;
    int maxBits=svg.load(filepath.toStdString(),-1);
    ui->_2_le_svg->setText(filepath);
    settings.setValue ( "_2_le_svg",filepath );

 } catch (const std::exception &ex) {
    QMessageBox::critical(this,tr("Error"),ex.what());
 }
}

void FiducialObjectQt::removeFolderContents(const std::string& folderPath) {
     for (const auto& entry : fs::directory_iterator(folderPath)) {
        if (fs::is_regular_file(entry) || fs::is_symlink(entry)) {
            fs::remove(entry.path());
        } else if (fs::is_directory(entry)) {
            removeFolderContents(entry.path().string());
            fs::remove(entry.path());
        }
    }

}
void FiducialObjectQt::on__2_pb_process_clicked()
{
     try{
        if(getOutputDir().empty())throw std::runtime_error("Invalid output dir");
        std::filesystem::create_directories(fs::path(getOutputDir())/"images");
        cucomarker::SVGFile svg;
        double dpi=180;
        auto imgDir=fs::path(getOutputDir())/"images";
        removeFolderContents(imgDir.string());
        std::error_code ec;
        fs::create_directory(imgDir);
        ui->pte_messages->clear();
        cancelGenerateImages=false;
        int maxId=svg.load(ui->_2_le_svg->text().toStdString(),-1);

        svg.exportConfigFile( (fs::path(getOutputDir())/"fo_markers.yml").string());

        ui->_2_pb_cancel->show();

        fs::path tmpsvg=fs::path(getOutputDir())/"tmp.svg";
        for(int markerId=1;markerId<maxId&& !cancelGenerateImages;markerId++ ){

            svg.save(markerId,tmpsvg.generic_string());
            plutovg_surface_t* surface = plutosvg_load_from_file(tmpsvg.string().c_str(), NULL, 0, 0, dpi);
            if(surface == NULL) throw std::runtime_error("Load failed");
            std::string number=std::to_string(markerId);
            while(number.size()<4) number="0"+number;
            std::string fname=number+".png";
            fs::path out=imgDir/fname;
            plutovg_surface_write_to_png(surface, out.string().c_str());
            plutovg_surface_destroy(surface);
            appendPlainText(ui->pte_messages,((out.string()+ " out of "+ std::to_string(maxId)+" images ")));
            QCoreApplication::processEvents();
            //    remove("tmp.svg");
        }
        std::filesystem::remove(tmpsvg,ec);
        ui->_2_pb_cancel->hide();

     } catch (const std::exception &ex) {
        QMessageBox::critical(this,tr("Error"),ex.what());
     }
}
std::string FiducialObjectQt::getOutputDir(){
     fs::path dir=ui->of_le_imagesFolder->text().toStdString();
     if(fs::is_directory(dir)) return dir.string();
     else return {};
}


void FiducialObjectQt::on_of_pb_imagesfolder_clicked()
{
QSettings settings;
QString filepath = QFileDialog::getExistingDirectory( this,tr ( "Select Project Directory" ),settings.value ( "currDir" ).toString());
if ( filepath==QString() ) return;
settings.setValue ( "currDir",filepath );

ui->of_le_imagesFolder->setText(filepath);
settings.setValue ( "of_le_imagesFolder",filepath );

//now, count the number of images in the folder
ui->of_le_imagesFolder->setText( filepath);
}





void FiducialObjectQt::on__2_pb_cancel_clicked()
{
cancelGenerateImages=true;
}


void FiducialObjectQt::on_of_pb_open_clicked()
{
if(getOutputDir().empty())return;
QString folderPath = getOutputDir().c_str();

QUrl folderUrl = QUrl::fromLocalFile(folderPath);
QDesktopServices::openUrl(folderUrl);
}


void FiducialObjectQt::on__3_pb_Process_clicked()
{
try{

 if(getOutputDir().empty())throw std::runtime_error("Invalid output dir");
 if(!fs::is_directory((ui->_3_le_imagesFolder->text().toStdString())))throw std::runtime_error("Please select a valid directory with images of the object from multiple viewpoints");
 if(!fs::is_regular_file(fs::path(getOutputDir())/"camera.yml")) throw std::runtime_error("Missing camera.yml. Calibrate the camera first (Step 1).");
 if(!fs::is_regular_file(fs::path(getOutputDir())/"fo_markers.yml")) throw std::runtime_error("Missing markers (Step 2).");

 aruco::CameraParameters Camera;
 Camera.readFromXMLFile((fs::path(getOutputDir())/"camera.yml").string());


 double mSize=ui->_3_sb_markerSize->value() ;
 auto AMM = aruco_mm::MarkerMapper::create();
 AMM->setParams(Camera,mSize, -1, true);

 // Create detector
 general_marker_detector::GeneralMarkerDetector *frameDetector;

 cucomarker_marker_detector::CucomarkerMarkerDetector *detector = new cucomarker_marker_detector::CucomarkerMarkerDetector(mSize, false, false);
 detector->setParams((fs::path(getOutputDir())/"fo_markers.yml").string());

 detector->setColorDetectionMode("GRAY");
 frameDetector = detector;

 // Insert detector to marker mapper
 AMM->setGeneralMarkerDetector(frameDetector);
 aruco_mm::debug::Debug::setLevel(10);
 aruco_mm::debug::Debug::setPrintFunction([this](std::string msg){
     appendPlainText(ui->pte_messages,(msg));
     QCoreApplication::processEvents();
 });

 cancelGenerateImages=false;
 // Marker Mapper - EXECUTION
 char key=0;
 cv::Mat image,image2;
 std::vector<std::string> files= getImagesInFolder(ui->_3_le_imagesFolder->text().toStdString());
 if(files.size()==0) throw std::runtime_error("No images in the folder:"+ui->_3_le_imagesFolder->text().toStdString());
 //        std::rotate(files.begin(),files.begin()+10,files.end());
 //rotate vector
 int frameidx=0;
 ui->pte_messages->clear();
 for(size_t i=0;i<files.size() ;i++){
        appendPlainText(ui->pte_messages,(std::string("Reading ..."+files[i])));
        QCoreApplication::processEvents();

        if ( files[i].back()=='.') continue; //skip . and ..
        image=cv::imread(files[i]);

        if (image.empty()) continue;

        if (image.rows == Camera.CamSize.width && image.cols == Camera.CamSize.height ){ //auto orient by rotation
                cv::Mat aux;
                cv::transpose(image,aux);
                cv::flip(aux,image,0);
        }

        if( image.rows != Camera.CamSize.height || image.cols != Camera.CamSize.width ){
                appendPlainText(ui->pte_messages,("wont process THIS image because is not of the dimensions of the camera calibration file provided"));
                continue;
        }


        AMM-> process( image ,frameidx++);
        if(cancelGenerateImages) return;


        }



        /*~~~~~~~~~~~~~~~~~~~~~~~~~
            Finish processing
        ~~~~~~~~~~~~~~~~~~~~~~~~~*/



        AMM->optimize();
        AMM->getMarkerMap().saveToFile( (fs::path( getOutputDir())/"fo_map.yml").string());



} catch (const std::exception &ex) {
 QMessageBox::critical(this,tr("Error"),ex.what());
}
}


void FiducialObjectQt::on__3_pb_imagesfolder_clicked()
{


QSettings settings;
QString filepath = QFileDialog::getExistingDirectory( this,tr ( "Select intput file" ),settings.value ( "currDir" ).toString());
if ( filepath==QString() ) return;
settings.setValue ( "currDir",filepath );

ui->_3_le_imagesFolder->setText(filepath);
settings.setValue ( "_3_le_imagesFolder",filepath );

//now, count the number of images in the folder
//ui->_3_le_imagesFolder->setText( getImagesInFolder(filepath .toStdString()).size()).c_str());
}


void FiducialObjectQt::on__3_pb_stop_clicked()
{
cancelGenerateImages=true;

}


void FiducialObjectQt::fiducialObjectConfigGeneration(std::vector<std::string> fo_markers_files, std::string fo_map_path, std::string fo_output_path){
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

void FiducialObjectQt::on_pushButton_clicked()
{
    try{
        fiducialObjectConfigGeneration({(fs::path(getOutputDir())/"fo_markers.yml").string()},(fs::path(getOutputDir())/"fo_map.yml").string(),(fs::path(getOutputDir())/"fobject.yml").string());
        fiducialObject::Object FObject;
        FObject.loadFromFile((fs::path(getOutputDir())/"fobject.yml").string());
        // auto im=cv::imread("/media/salinas/sav/FiducialObjectTutorial/03_3DFiducialObject/IMG_20230531_101409066.jpg");
        auto im=cv::imread("/home/paul/Workspace/FiducialObject/FiducialObjectTutorial/03_3DFiducialObject/IMG_20230531_101409066.jpg");
        auto detection= fiducialObject::Detector::detect({FObject},im);
        for(auto &d:detection)
            d.draw(im);
        // cv::imwrite("/home/salinas/out.jpg",im);
        cv::imwrite("/home/paul/Workspace/FiducialObject/FiducialObjectTutorial/outputs/out.jpg",im);
        FObject.saveToFile("/home/paul/Workspace/FiducialObject/FiducialObjectTutorial/outputs/copied_fobject.yml");
        FObject.scale(10);
        FObject.saveToFile("/home/paul/Workspace/FiducialObject/FiducialObjectTutorial/outputs/scaled_fobject.yml");
    } catch (const std::exception &ex) {
        QMessageBox::critical(this,tr("Error"),ex.what());
    }
}



// MARKER ID DRAWING
void drawNumberIn3DScene(int number, const cv::Point3f& position, qtgl::GlViewer* glWindow) {

    // Create an image with the number text
    cv::Mat numberImage(40, 40, CV_8UC4, cv::Scalar(0, 0, 0, 0));
    std::string text = std::to_string(number);
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1, 2, &baseline);
    int x = (numberImage.cols - textSize.width) / 2;
    int y = (numberImage.rows + textSize.height) / 2;
    cv::putText(numberImage, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255, 255), 2);
//    cv::imshow("Aux", numberImage);
//    cv::waitKey();

    // Auxiliar
    auto line = std::make_shared<GLLine>();
    line->setPoints(position, position*1.01);
    glWindow->insert(line, "LLine_sswdeheiecie"+std::to_string(number));

    // Loop over each pixel in the number image and map it to the 3D scene
    int count = number*100000;
    for (int i = 0; i < numberImage.rows; ++i) {
        for (int j = 0; j < numberImage.cols; ++j) {
            cv::Vec4b pixel = numberImage.at<cv::Vec4b>(i, j);
            cv::Point3f scenePoint = position + cv::Point3f(static_cast<float>(j) / 10000.0f, static_cast<float>(i) / 10000.0f, 0.0f);
            if (pixel[3] != 0) { // Check if the pixel is not transparent

                // Line version
//                auto line = std::make_shared<GLLine>();
//                line->setPoints(scenePoint, scenePoint*1.01);
//                line->color_x=1.f; line->color_y=1.f; line->color_z=1.f;
//                glWindow->insert(line, "LLine_" + std::to_string(count++));
                // Point version
                auto point = std::make_shared<GLPoint>();
                point->setPoint(scenePoint);
                point->color_x=1; point->color_y=1; point->color_z=1;
                glWindow->insert(point, "LLine_" + std::to_string(count++));

                count+=1;
            }
            else{
                bool randomBool = std::rand() % 2 == 0;
                if(randomBool){

                    // Line version
//                    auto line = std::make_shared<GLLine>();
//                    line->setPoints(scenePoint, scenePoint*1.01);
//                    line->color_x=0.8f; line->color_y=0.f; line->color_z=0.2f;
//                    glWindow->insert(line, "LLine_" + std::to_string(count++));
                    // Point version
                    auto point = std::make_shared<GLPoint>();
                    point->setPoint(scenePoint);
                    point->color_x=0.8; point->color_y=0; point->color_z=0.2;
                    glWindow->insert(point, "LLine_" + std::to_string(count++));

                    count+=1;
                }
            }
        }
    }
}


void FiducialObjectQt::on__3_pb_show3d_clicked()
{
    /* Window definition */
    qtgl::GlViewer *_glWindow=new qtgl::GlViewer( {800,800},this);

    /* CUBE */
//    auto cube=std::make_shared<GLCube>();
//    cube->setSize(0.25);
//    _glWindow->insert( cube,"Cube");

    /* Line example */
//    cv::Point3f p1(0.0f, 0.0f, 0.0f);
//    cv::Point3f p2(1.0f, 1.0f, 1.0f);
//    auto line = std::make_shared<GLLine>();
//    line->setPoints(p1, p2);
//    _glWindow->insert(line, "Line");

    /* Draw Fiducial Object */
    fiducialObjectConfigGeneration({(fs::path(getOutputDir())/"fo_markers.yml").string()},(fs::path(getOutputDir())/"fo_map.yml").string(),(fs::path(getOutputDir())/"fobject.yml").string());
    fiducialObject::Object FObject;
    FObject.loadFromFile((fs::path(getOutputDir())/"fobject.yml").string());
    // Drawing lines
        auto drawLinesFromMap = [&](const std::unordered_map<int, std::vector<cv::Point3f>>& markers) {
            int count = 0;
            for (const auto& pair : markers) {
                const auto& points = pair.second;
                for (size_t i = 0; i < points.size() - 1; ++i) {
                    auto line = std::make_shared<GLLine>();
                    line->setPoints(points[i], points[i + 1]);
                    _glWindow->insert(line, "Line_" + std::to_string(pair.first) + "_" + std::to_string(count++));
                }
                // Connect the last point with the first one
                if (!points.empty()) {
                    auto line = std::make_shared<GLLine>();
                    line->setPoints(points.back(), points.front());
                    _glWindow->insert(line, "Line_" + std::to_string(pair.first) + "_" + std::to_string(count++));
                }
                // Draw ID
                cv::Point3f center(0.0f, 0.0f, 0.0f);
                for (const auto& corner : points) {
                    center += corner;
                }
                center /= static_cast<float>(points.size());
                drawNumberIn3DScene(pair.first, center, _glWindow);
            }
        };
        drawLinesFromMap(FObject.markers);

    /* Showing */
    _glWindow->showAxis(0.1);
    _glWindow->show();
}




void FiducialObjectQt::on__4_pb_imagesfolder_clicked()
{
    QSettings settings;
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setDirectory(settings.value("currDir").toString());
    dialog.setWindowTitle(tr("Select input file or directory"));

    if(dialog.exec())
    {
        QString filepath = dialog.selectedFiles().first();
        if(filepath.isEmpty())
            return;

        settings.setValue("currDir",filepath);

        ui->_3_le_imagesFolder->setText(filepath);
        settings.setValue("_4_le_imagesFolder",filepath);
    }
}



void FiducialObjectQt::on__4_pb_Process_clicked()
{

}


void FiducialObjectQt::on__4_open_inputFolder_clicked()
{
    // Folder!
    if(getOutputDir().empty())return;
    QString folderPath = ui->_4_le_input->text().toStdString().c_str();
    FileInfo fileInfo = checkFileType(folderPath.toStdString());
    QUrl folderUrl;
    switch(fileInfo.type){
        case FileType::FOLDER:
            appendPlainText(ui->pte_messages,(std::string(QTime::currentTime().toString("[hh:mm:ss]: ").toStdString()+"Opening folder...")));
            folderUrl = QUrl::fromLocalFile(folderPath);
        break;
        case FileType::IMAGE:
            appendPlainText(ui->pte_messages,(std::string(QTime::currentTime().toString("[hh:mm:ss]: ").toStdString()+"Opening image file...")));
            folderUrl = QUrl::fromLocalFile(fileInfo.parentDir.c_str());
        break;
        case FileType::VIDEO:
            appendPlainText(ui->pte_messages,(std::string(QTime::currentTime().toString("[hh:mm:ss]: ").toStdString()+"Opening video file...")));
            folderUrl = QUrl::fromLocalFile(fileInfo.parentDir.c_str());
        break;
        default:
            appendPlainText(ui->pte_messages,(std::string("Not a valid folder or file!")));
        break;
    }
    QDesktopServices::openUrl(folderUrl);

//    File!
//    if(getOutputDir().empty())return;
//    QString folderPath = (checkFileType(ui->_4_le_input->text().toStdString()).parentDir).c_str();
//    if(!std::filesystem::exists(folderPath.toStdString())) throw std::runtime_error("Container folder does not exist");
//    appendPlainText(ui->pte_messages,(std::string(QTime::currentTime().toString("[hh:mm:ss]: ").toStdString()+"Opening input file container folder...")));
//    QUrl folderUrl = QUrl::fromLocalFile(folderPath);
//    QDesktopServices::openUrl(folderUrl);
}


void FiducialObjectQt::on__4_pb_inputFolder_clicked()
{
    QSettings settings;
    QString filepath = QFileDialog::getExistingDirectory(this, tr("Select input directory"), settings.value("currDir").toString());
    if (filepath.isEmpty())
        return;

    settings.setValue("currDir",filepath);

    // Checking type
    bool isValid = checkFileType(filepath.toStdString()).type == FileType::FOLDER;
    if (isValid) {
        // If the directory is valid, set the text color to green
        QPalette palette = ui->_4_le_input->palette();
        palette.setColor(QPalette::Text, QColor(0, 150, 0));
        ui->_4_le_input->setPalette(palette);
        appendPlainText(ui->pte_messages,(std::string(QTime::currentTime().toString("[hh:mm:ss]: ").toStdString()+"Loaded folder succesfully")));
    } else {
        // If the directory is not valid, set the text color to red
        QPalette palette = ui->_4_le_input->palette();
        palette.setColor(QPalette::Text, QColor(150, 0, 0));
        ui->_4_le_input->setPalette(palette);
        appendPlainText(ui->pte_messages,(std::string(QTime::currentTime().toString("[hh:mm:ss]: ").toStdString()+"ERROR: Not a valid folder")));
    }

    ui->_4_le_input->setText(filepath);
    settings.setValue("_4_le_inputFolder",filepath);
}


void FiducialObjectQt::on__4_pb_inputFile_clicked()
{
    QSettings settings;
    QString filepath = QFileDialog::getOpenFileName(this, tr("Select input file"), settings.value("currDir").toString());
    if (filepath.isEmpty())
        return;

    settings.setValue("currDir",filepath);

    // Checking type
    bool isValid = checkFileType(filepath.toStdString()).type == FileType::IMAGE or checkFileType(filepath.toStdString()).type == FileType::VIDEO;
    if (isValid) {
        // If the directory is valid, set the text color to green
        QPalette palette = ui->_4_le_input->palette();
        palette.setColor(QPalette::Text, QColor(0, 150, 0));
        ui->_4_le_input->setPalette(palette);
        appendPlainText(ui->pte_messages,(std::string(QTime::currentTime().toString("[hh:mm:ss]: ").toStdString()+"Loaded file succesfully")));
    } else {
        // If the directory is not valid, set the text color to red
        QPalette palette = ui->_4_le_input->palette();
        palette.setColor(QPalette::Text, QColor(150, 0, 0));
        ui->_4_le_input->setPalette(palette);
        appendPlainText(ui->pte_messages,(std::string(QTime::currentTime().toString("[hh:mm:ss]: ").toStdString()+"ERROR: Not a valid file")));
    }

    ui->_4_le_input->setText(filepath);
    settings.setValue("_4_le_inputFile",filepath);
}


void FiducialObjectQt::on_pushButton_2_clicked()
{

}



void FiducialObjectQt::on__4_open_inputFile_clicked()
{
//    if(getOutputDir().empty())return;
//    QString folderPath = (checkFileType(ui->_4_le_inputFile->text().toStdString()).parentDir).c_str();
//    if(!std::filesystem::exists(folderPath.toStdString())) throw std::runtime_error("Container folder does not exist");
//    appendPlainText(ui->pte_messages,(std::string(QTime::currentTime().toString("[hh:mm:ss]: ").toStdString()+"Opening input file container folder...")));
//    QUrl folderUrl = QUrl::fromLocalFile(folderPath);
//    QDesktopServices::openUrl(folderUrl);
}




void FiducialObjectQt::on__4_pb_ProcessFolder_clicked()
{
    try {
        // Restart stopProcessing
        stopProcessing = false;
        // Info
        ui->pte_messages->insertHtml(std::string(QTime::currentTime().toString("[hh:mm:ss]: ").toStdString()+"<b>Processing input</b><br>").c_str());
        // Output folder
        FileInfo fileInfo = checkFileType(ui->_4_le_input->text().toStdString());
        std::string outputFolder = getOutputDir()+"/Output__"+fileInfo.name;
            QDir outDir(QString::fromStdString(outputFolder)); // Convert std::string to QString
            if (outDir.exists()) { // Check if directory exists
                outDir.removeRecursively(); // Remove directory if it exists
            }
            outDir.mkpath("."); // Create new directory
        // Camera parameters
        auto fs = cv::FileStorage();
        cv::Mat K, D;
        fs.open(getOutputDir()+"/camera.yml", cv::FileStorage::READ);
        fs["camera_matrix"] >> K;
        fs["distortion_coefficients"] >> D;
        fs.release();
        // Creation of the fiducial object
        fiducialObjectConfigGeneration({(fs::path(getOutputDir())/"fo_markers.yml").string()},(fs::path(getOutputDir())/"fo_map.yml").string(),(fs::path(getOutputDir())/"fobject.yml").string());
        fiducialObject::Object FObject;
        FObject.loadFromFile((fs::path(getOutputDir())/"fobject.yml").string());
        //FObject.scale(2);
        // Openning the path
        std::string path = ui->_4_le_input->text().toStdString();
        if(!(checkFileType(path.c_str()).type == FileType::IMAGE or checkFileType(path.c_str()).type == FileType::VIDEO or checkFileType(path.c_str()).type == FileType::FOLDER)) throw std::runtime_error("Input folder or file not valid");
        // FOLDER
        if(checkFileType(path.c_str()).type == FileType::FOLDER){
            std::string path = ui->_4_le_input->text().toStdString();
            if(path.empty())throw std::runtime_error("Image folder path not valid");
            QDir dir(QString::fromStdString(path)); // Convert std::string to QString
            // Specify the file filters for image file types
            QStringList filters;
            filters << "*.jpg" << "*.png" << "*.jpeg" << "*.bmp";
            // Apply the filters to the QDir object
            dir.setNameFilters(filters);
            // Get the list of image files in the directory
            QFileInfoList list = dir.entryInfoList();
            cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
            cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
            for(int i = 0; i < list.size(); ++i)
            {
                // Stop processing?
                if(stopProcessing) return;
                QCoreApplication::processEvents();
                // Input path
                QFileInfo fileInfo = list.at(i);
                std::string inputImagePath = fileInfo.filePath().toStdString();
                cv::Mat inputImage = cv::imread(inputImagePath);
                appendPlainText(ui->pte_messages,(("Processing image ["+std::to_string(i+1)+"/"+std::to_string(list.size())+"] .../"+checkFileType(inputImagePath).name)));
                // Output path
                std::string outputImagePath = inputImagePath;
                    size_t pos = inputImagePath.find_last_of("/");
                    if (pos != std::string::npos) {
                        outputImagePath = inputImagePath.substr(pos + 1);
                    }
                    outputImagePath = outputFolder+"/"+outputImagePath;
                // Processing
                bool validPose = processImage(inputImage, FObject, K, D, tvec, rvec); if(validPose) writePoseOnCSV(outputFolder+"/poses.csv", ""+checkFileType(inputImagePath).name, tvec, rvec);
                cv::imwrite(outputImagePath,inputImage);
                QCoreApplication::processEvents();
            }
        }
        // IMAGE or VIDEO
        if(checkFileType(path.c_str()).type == FileType::IMAGE){
            // Input path
            std::string inputImagePath = path;
            cv::Mat inputImage = cv::imread(inputImagePath);
            appendPlainText(ui->pte_messages,(("Processing image .../"+checkFileType(inputImagePath).name)));
            // Output path
            std::string outputImagePath = inputImagePath;
                size_t pos = inputImagePath.find_last_of("/");
                if (pos != std::string::npos) {
                    outputImagePath = inputImagePath.substr(pos + 1);
                }
                outputImagePath = outputFolder+"/"+outputImagePath;
            // Processing
            cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
            cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
            bool validPose = processImage(inputImage, FObject, K, D, tvec, rvec); if(validPose) writePoseOnCSV(outputFolder+"/poses.csv", ""+checkFileType(inputImagePath).name, tvec, rvec);
            cv::imwrite(outputImagePath,inputImage);
            QCoreApplication::processEvents();
        }
        else if(checkFileType(path.c_str()).type == FileType::VIDEO){
            // Open video
            cv::VideoCapture cap(path);
            if(!cap.isOpened()) throw std::runtime_error("Cannot open video file");
            cv::Mat frame;
            int processed_frames = 0;
            int numFrames = cap.get(cv::CAP_PROP_FRAME_COUNT);
            // Define the codec and create VideoWriter object
            int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
            int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
            std::string extension = checkFileType(path).extension;
            int fourcc;
            if (extension == "mp4") {
                fourcc = cv::VideoWriter::fourcc('H','2','6','4'); // H264 codec for .mp4
            } else if (extension == "avi") {
                fourcc = cv::VideoWriter::fourcc('M','J','P','G'); // MJPG codec for .avi
            } else {
                // Use MJPG as default for other file types
                fourcc = cv::VideoWriter::fourcc('M','J','P','G');
            }
            cv::VideoWriter video(outputFolder+"/"+checkFileType(path).name, fourcc, 30, cv::Size(frame_width,frame_height));
            // Processing
            cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
            cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
            while(cap.read(frame)) {
                // Stop processing?
                if(stopProcessing) return;
                QCoreApplication::processEvents();
                // Input path
                cv::Mat inputImage = frame;
                appendPlainText(ui->pte_messages,(("Processing frame ["+std::to_string(processed_frames+1)+"/"+std::to_string(numFrames)+"] of .../"+checkFileType(path).name)));
                QCoreApplication::processEvents();
                // Processing
                bool validPose = processImage(inputImage, FObject, K, D, tvec, rvec);
                QCoreApplication::processEvents();
                std::cout<<"Frame: "<<processed_frames+1<<std::endl;
                if(validPose){
                    writePoseOnCSV(outputFolder+"/poses.csv", ""+std::to_string(processed_frames+1), tvec, rvec);
                }
                cv::putText(inputImage, "ID: "+std::to_string(processed_frames+1), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                // Write the frame into the file 'out.avi'
                video.write(inputImage);
                // id
                processed_frames++;
            }
            cap.release();
        }


    } catch (const std::exception &ex) {
        QMessageBox::critical(this,tr("Error"),ex.what());
    }
}

void FiducialObjectQt::on__4_open_inputFolder_2_clicked(){

}

void FiducialObjectQt::on__4_pb_ProcessFile_clicked()
{
    if(getOutputDir().empty())return;
    QString folderPath = (getOutputDir()+"/Output__"+checkFileType(ui->_4_le_input->text().toStdString()).name).c_str();
    if(!std::filesystem::exists(folderPath.toStdString())){
        appendPlainText(ui->pte_messages,(std::string(QTime::currentTime().toString("[hh:mm:ss]: ").toStdString()+"Folder does not exists")));
    }
    else{
        appendPlainText(ui->pte_messages,(std::string(QTime::currentTime().toString("[hh:mm:ss]: ").toStdString()+"Opening output images folder...")));
        QUrl folderUrl = QUrl::fromLocalFile(folderPath);
        QDesktopServices::openUrl(folderUrl);
    }
}


void FiducialObjectQt::on__4_pb_OpenOutput_clicked()
{

}


void FiducialObjectQt::on__4_pb_OpenOutput_2_clicked()
{

}


void FiducialObjectQt::on__4_pb_stopProcessing_clicked()
{
    stopProcessing = true;
}

