#ifndef FIDUCIALOBJECTQT_H
#define FIDUCIALOBJECTQT_H

#include <QWidget>
#include <filesystem>
namespace Ui {
class FiducialObjectQt;
}

class FiducialObjectQt : public QWidget
{
    Q_OBJECT

public:
    explicit FiducialObjectQt(QWidget *parent = nullptr);
    ~FiducialObjectQt();

private slots:
    void on_cc_pb_imagesfolder_clicked();

    void on_cc_pb_calibrate_clicked();

    void on__2_pb_svg_clicked();


    void on_of_pb_imagesfolder_clicked();

    void on__2_pb_process_clicked();

    void on__2_pb_cancel_clicked();

    void on_of_pb_open_clicked();

    void on__3_pb_Process_clicked();


    void on__3_pb_imagesfolder_clicked();

    void on__3_pb_stop_clicked();

    void on_pushButton_clicked();

    void on__3_pb_show3d_clicked();

    void on__4_pb_imagesfolder_clicked();

    void on__4_pb_Process_clicked();

    void on__4_open_inputFolder_clicked();

    void on__4_pb_inputFolder_clicked();

    void on__4_pb_inputFile_clicked();

    void on_pushButton_2_clicked();

    void on__4_open_inputFolder_2_clicked();

    void on__4_open_inputFile_clicked();

    void on__4_pb_ProcessFolder_clicked();

    void on__4_pb_ProcessFile_clicked();

    void on__4_pb_OpenOutput_clicked();

    void on__4_pb_OpenOutput_2_clicked();

    void on__4_pb_stopProcessing_clicked();

private:
    Ui::FiducialObjectQt *ui;
    std::string getOutputDir();
    bool cancelGenerateImages=false;
    bool stopProcessing=false;
    std::vector<std::string>  getImagesInFolder(const std::filesystem::path& folderPath) ;
    void removeFolderContents(const std::string& folderPath) ;
    void fiducialObjectConfigGeneration(std::vector<std::string> fo_markers_files, std::string fo_map_path, std::string fo_output_path);


};

#endif // FIDUCIALOBJECTQT_H
