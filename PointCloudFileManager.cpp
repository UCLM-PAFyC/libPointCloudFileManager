#include <QFileInfo>
#include <QTextStream>
#include <QDir>
#include <QDateTime>
#include <QProcessEnvironment>
#include <QMessageBox>
#include <QProgressDialog>
#include <QApplication>
#include <QtConcurrent>
#include <qtconcurrentmap.h>

#include "ParameterDefinitions.h"
#include "Parameter.h"
#include "ParametersManager.h"
#include "ParametersManagerDialog.h"
#include "CRSTools.h"
//#include "CrsToolsImpl.h"
#include "../libIGDAL/Shapefile.h"
#include <ogrsf_frmts.h>

#include "ProgressExternalProcessDialog.h"

#include "PointCloudFile.h"
#include "PointCloudFileManager.h"
#include "ControlROIs.h"

#include "lasreader.hpp"
#include "laswriter.hpp"

using namespace PCFile;

PointCloudFileManager * PointCloudFileManager::mInstance = 0;

bool PointCloudFileManager::checkInitialize(QString &strError)
{
    if(mPtrCrsTools==NULL)
    {
        strError=QObject::tr("PointCloudFileManager::checkInitialize");
        strError+=QObject::tr("\nCRSTools object has not been established");
        return(false);
    }
    if(mBasePath.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::checkInitialize");
        strError+=QObject::tr("\nBase path has not been established");
        return(false);
    }
    return(true);
}


bool PointCloudFileManager::addPointCloudFilesToPointCloudFile(QString pcfPath,
                                                               int crsEpsgCode,
                                                               bool altitudeIsMsl,
                                                               QVector<QString> &pointCloudFiles,
                                                               QString &strError)
{
    QString strAuxError;
    if(mPtrCrsTools==NULL)
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFilesToPointCloudFile");
        strError+=QObject::tr("\nCrsTools is NULL");
        return(false);
    }
    for(int nf=0;nf<pointCloudFiles.size();nf++)
    {
        if(!QFile::exists(pointCloudFiles.at(nf)))
        {
            strError=QObject::tr("PointCloudFileManager::addPointCloudFilesToPointCloudFile");
            strError+=QObject::tr("\nNot exists Point Cloud File:\n%1").arg(pointCloudFiles.at(nf));
            return(false);
        }
    }
    QString prjFileName=mBasePath+POINTCLOUDFILE_TEMPORAL_PROJECT_FILE;
    if(QFile::exists(prjFileName))
    {
        if(!QFile::remove(prjFileName))
        {
            strError=QObject::tr("PointCloudFileManager::addPointCloudFilesToPointCloudFile");
            strError+=QObject::tr("\nError existing temporal project file:\n%1")
                    .arg(prjFileName);
            return(false);
        }
    }
    QFile file(prjFileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFilesToPointCloudFile");
        strError+=QObject::tr("\nError opening temporal project file:\n%1")
                .arg(prjFileName);
        return(false);
    }
    QTextStream strOut(&file);

    int numberOfProcesses=1;
    strOut<<"Point Cloud File Management Project\n";
    strOut<<"- Number of processes ......................................# ";
    strOut<<QString::number(numberOfProcesses);
    strOut<<"\n";
    strOut<<"- Process (CREATE_PCFM,ADD_PCFILE,ADD_ROI,WRITE_PCFS) ......# ";
//        strOut<<POINTCLOUDDB_PROCESS_ADD_POINTCLOUD_TAG;
    strOut<<POINTCLOUDFILE_PROCESS_ADD_POINT_CLOUD_FILES_TAG;
    strOut<<"\n";
    strOut<<"  - PoinCloudFileManager path ..............................# \"";
    strOut<<pcfPath;
    strOut<<"\"\n";
    strOut<<"  - CRS (libCRS/EPSG/proj4/wkt) ............................# ";
    strOut<<QString::number(crsEpsgCode);//"WGS84;32633;ELLIPSOID_HEIGHTS";
    strOut<<"\n";
    strOut<<"  - Number of files ........................................# ";
    strOut<<QString::number(pointCloudFiles.size());
    strOut<<"\n";
    for(int nf=0;nf<pointCloudFiles.size();nf++)
    {
        QString pointCloudFile=pointCloudFiles.at(nf);
        strOut<<"  - File ...................................................# \"";
        strOut<<pointCloudFile;
        strOut<<"\"\n";
    }
    file.close();
    if(!processProjectFile(prjFileName,
                           strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFilesToPointCloudFile");
        strError+=QObject::tr("\nError calling setFromProjectFile for project file:\n%1\nError:\n%2")
                .arg(prjFileName).arg(strAuxError);
        return(false);
    }
    return(true);
}

bool PointCloudFileManager::createPointCloudFile(QString pcfPath,
                                                 QString projectType,
                                                 double gridSize,
                                                 int crsEpsgCode,
                                                 int verticalCrsEpsgCode,
                                                 QVector<QString> &roisShapefiles,
                                                 QString &strError)
{
    QString strAuxError;
    if(mPtrCrsTools==NULL)
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nCrsTools is NULL");
        return(false);
    }
    QVector<QString> noEmptyRoiShapefiles;
    for(int nRoi=0;nRoi<roisShapefiles.size();nRoi++)
    {
        if(!roisShapefiles.at(nRoi).trimmed().isEmpty())
        {
            if(!QFile::exists(roisShapefiles.at(nRoi)))
            {
                strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
                strError+=QObject::tr("\nNot exists ROI shapefile:\n%1").arg(roisShapefiles.at(nRoi));
                return(false);
            }
            noEmptyRoiShapefiles.push_back(roisShapefiles.at(nRoi).trimmed());
        }
    }
    roisShapefiles.clear();
    roisShapefiles=noEmptyRoiShapefiles;
    noEmptyRoiShapefiles.clear();
    if(!setProjectsParametersManager(projectType,strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError setting parameters for project type: %1\nError:\n%2")
                .arg(projectType).arg(strAuxError);
        return(false);
    }
    QString projectParametersString;
    if(!getProjectParametersString(projectType,
                                   projectParametersString,
                                   strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError recovering parameters string for project type: %1\nError:\n%2")
                .arg(projectType).arg(strAuxError);
        return(false);
    }
//    QDir auxDir=QDir::currentPath();
//    if(auxDir.exists(pcfPath))
//    {
//        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
//        strError+=QObject::tr("\nExists path:\n%1\nremove it before")
//                .arg(pcfPath);
//        return(false);
//    }
    QString prjFileName=mBasePath+POINTCLOUDFILE_TEMPORAL_PROJECT_FILE;
    if(QFile::exists(prjFileName))
    {
        if(!QFile::remove(prjFileName))
        {
            strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
            strError+=QObject::tr("\nError existing temporal project file:\n%1")
                    .arg(prjFileName);
            return(false);
        }
    }
    QFile file(prjFileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError opening temporal project file:\n%1")
                .arg(prjFileName);
        return(false);
    }
    QTextStream strOut(&file);

    int numberOfProcesses=1;
    numberOfProcesses+=roisShapefiles.size();

    strOut<<"Point Cloud File Management Project\n";
    strOut<<"- Number of processes ......................................# ";
    strOut<<QString::number(numberOfProcesses);
    strOut<<"\n";
    strOut<<"- Process (CREATE_PCF,ADD_POINTCLOUD,ADD_ROI,WRITE_PCFS) ...# ";
    strOut<<POINTCLOUDFILE_PROCESS_CREATE_POINT_CLOUD_FILE_TAG;
    strOut<<"\n";
    strOut<<"  - PoinCloudFileManager path ..............................# \"";
    strOut<<pcfPath;
    strOut<<"\"\n";
    strOut<<"  - Database CRS (libCRS/EPSG/proj4/wkt) ...................# ";
    strOut<<QString::number(crsEpsgCode);//"WGS84;32633;ELLIPSOID_HEIGHTS";
    strOut<<"\n";
    strOut<<"    Height type (EPSG,ORTHOMETRIC_HEIGHT,ELLIPSOID_HEIGHT) .# ";
    strOut<<QString::number(verticalCrsEpsgCode);//"WGS84;32633;ELLIPSOID_HEIGHTS";
//    if(altitudeIsMsl)
//    {
//        strOut<<CRSTOOLS_XMLFILE_TAG_ORTHOMETRIC_HEIGHT;
//    }
//    else
//    {
//        strOut<<CRSTOOLS_XMLFILE_TAG_ELLIPSOID_HEIGHT;
//    }
    strOut<<"\n";
    strOut<<"  - Grid size ..............................................# ";
    strOut<<QString::number(gridSize,'f',POINTCLOUDFILE_GRID_SIZE_FIELD_PRECISION);
    strOut<<"\n";
    strOut<<"  - Project Type (Generic,PowerLine,SolarPark) .............# ";
    strOut<<projectType;
    strOut<<"\n";
    strOut<<"  - Project parameters (par1@v@value1@p@par2@v@value2...) ..# ";
    strOut<<projectParametersString;//"Company@v@UCLM@p@Author@v@David Hernandez Lopez";
    strOut<<"\n";
    for(int nRoi=0;nRoi<roisShapefiles.size();nRoi++)
    {
        QString roisShapefile=roisShapefiles.at(nRoi);
        strOut<<"- Process (CREATE_PCF,ADD_POINTCLOUD,ADD_ROI,WRITE_PCFS) ...# ";
        strOut<<POINTCLOUDFILE_PROCESS_ADD_ROI_TAG;
        strOut<<"\n";
        strOut<<"  - PoinCloudFileManager path ..............................# \"";
        strOut<<pcfPath;
        strOut<<"\"\n";
        strOut<<"  - ROI File ...............................................# \"";
        strOut<<roisShapefile;
        strOut<<"\"\n";
        strOut<<"    Field id (None for all features ) ......................# None";
        strOut<<"\n";
        strOut<<"      Field id values (separed by ; ignored si None) .......# ";
        strOut<<"\n";
    }
    file.close();
    if(!processProjectFile(prjFileName,
                           strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError calling setFromProjectFile for project file:\n%1\nError:\n%2")
                .arg(prjFileName).arg(strAuxError);
        return(false);
    }
    return(true);
}

bool PointCloudFileManager::createPointCloudFile(QString pcfPath,
                                                 QString projectType,
                                                 double gridSize,
                                                 int crsEpsgCode,
                                                 bool altitudeIsMsl,
                                                 QVector<QString> &roisShapefiles,
                                                 QString &strError)
{
    QString strAuxError;
    if(mPtrCrsTools==NULL)
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nCrsTools is NULL");
        return(false);
    }
    QVector<QString> noEmptyRoiShapefiles;
    for(int nRoi=0;nRoi<roisShapefiles.size();nRoi++)
    {
        if(!roisShapefiles.at(nRoi).trimmed().isEmpty())
        {
            if(!QFile::exists(roisShapefiles.at(nRoi)))
            {
                strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
                strError+=QObject::tr("\nNot exists ROI shapefile:\n%1").arg(roisShapefiles.at(nRoi));
                return(false);
            }
            noEmptyRoiShapefiles.push_back(roisShapefiles.at(nRoi).trimmed());
        }
    }
    roisShapefiles.clear();
    roisShapefiles=noEmptyRoiShapefiles;
    noEmptyRoiShapefiles.clear();
    if(!setProjectsParametersManager(projectType,strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError setting parameters for project type: %1\nError:\n%2")
                .arg(projectType).arg(strAuxError);
        return(false);
    }
    QString projectParametersString;
    if(!getProjectParametersString(projectType,
                                   projectParametersString,
                                   strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError recovering parameters string for project type: %1\nError:\n%2")
                .arg(projectType).arg(strAuxError);
        return(false);
    }
//    QDir auxDir=QDir::currentPath();
//    if(auxDir.exists(pcfPath))
//    {
//        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
//        strError+=QObject::tr("\nExists path:\n%1\nremove it before")
//                .arg(pcfPath);
//        return(false);
//    }
    QString prjFileName=mBasePath+POINTCLOUDFILE_TEMPORAL_PROJECT_FILE;
    if(QFile::exists(prjFileName))
    {
        if(!QFile::remove(prjFileName))
        {
            strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
            strError+=QObject::tr("\nError existing temporal project file:\n%1")
                    .arg(prjFileName);
            return(false);
        }
    }
    QFile file(prjFileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError opening temporal project file:\n%1")
                .arg(prjFileName);
        return(false);
    }
    QTextStream strOut(&file);

    int numberOfProcesses=1;
    numberOfProcesses+=roisShapefiles.size();

    strOut<<"Point Cloud File Management Project\n";
    strOut<<"- Number of processes ......................................# ";
    strOut<<QString::number(numberOfProcesses);
    strOut<<"\n";
    strOut<<"- Process (CREATE_PCF,ADD_POINTCLOUD,ADD_ROI,WRITE_PCFS) ...# ";
    strOut<<POINTCLOUDFILE_PROCESS_CREATE_POINT_CLOUD_FILE_TAG;
    strOut<<"\n";
    strOut<<"  - PoinCloudFileManager path ..............................# \"";
    strOut<<pcfPath;
    strOut<<"\"\n";
    strOut<<"  - Database CRS (libCRS/EPSG/proj4/wkt) ...................# ";
    strOut<<QString::number(crsEpsgCode);//"WGS84;32633;ELLIPSOID_HEIGHTS";
    strOut<<"\n";
    strOut<<"    Height type (ORTHOMETRIC_HEIGHT,ELLIPSOID_HEIGHT) ......# ";
    if(altitudeIsMsl)
    {
        strOut<<CRSTOOLS_XMLFILE_TAG_ORTHOMETRIC_HEIGHT;
    }
    else
    {
        strOut<<CRSTOOLS_XMLFILE_TAG_ELLIPSOID_HEIGHT;
    }
    strOut<<"\n";
    strOut<<"  - Grid size ..............................................# ";
    strOut<<QString::number(gridSize,'f',POINTCLOUDFILE_GRID_SIZE_FIELD_PRECISION);
    strOut<<"\n";
    strOut<<"  - Project Type (Generic,PowerLine,SolarPark) .............# ";
    strOut<<projectType;
    strOut<<"\n";
    strOut<<"  - Project parameters (par1@v@value1@p@par2@v@value2...) ..# ";
    strOut<<projectParametersString;//"Company@v@UCLM@p@Author@v@David Hernandez Lopez";
    strOut<<"\n";
    for(int nRoi=0;nRoi<roisShapefiles.size();nRoi++)
    {
        QString roisShapefile=roisShapefiles.at(nRoi);
        strOut<<"- Process (CREATE_PCF,ADD_POINTCLOUD,ADD_ROI,WRITE_PCFS) ...# ";
        strOut<<POINTCLOUDFILE_PROCESS_ADD_ROI_TAG;
        strOut<<"\n";
        strOut<<"  - PoinCloudFileManager path ..............................# \"";
        strOut<<pcfPath;
        strOut<<"\"\n";
        strOut<<"  - ROI File ...............................................# \"";
        strOut<<roisShapefile;
        strOut<<"\"\n";
        strOut<<"    Field id (None for all features ) ......................# None";
        strOut<<"\n";
        strOut<<"      Field id values (separed by ; ignored si None) .......# ";
        strOut<<"\n";
    }
    file.close();
    if(!processProjectFile(prjFileName,
                           strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError calling setFromProjectFile for project file:\n%1\nError:\n%2")
                .arg(prjFileName).arg(strAuxError);
        return(false);
    }
    return(true);
}

bool PointCloudFileManager::exportProcessedPointCloudFiles(QString pcfPath,
                                                           QString suffix,
                                                           QString outputPath,
                                                           QString &strError)
{
    QString strAuxError;
    if(mPtrCrsTools==NULL)
    {
        strError=QObject::tr("PointCloudFileManager::exportProcessedPointCloudFiles");
        strError+=QObject::tr("\nCrsTools is NULL");
        return(false);
    }
    QString prjFileName=mBasePath+POINTCLOUDFILE_TEMPORAL_PROJECT_FILE;
    if(QFile::exists(prjFileName))
    {
        if(!QFile::remove(prjFileName))
        {
            strError=QObject::tr("PointCloudFileManager::exportProcessedPointCloudFiles");
            strError+=QObject::tr("\nError existing temporal project file:\n%1")
                    .arg(prjFileName);
            return(false);
        }
    }
    QFile file(prjFileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        strError=QObject::tr("PointCloudFileManager::exportProcessedPointCloudFiles");
        strError+=QObject::tr("\nError opening temporal project file:\n%1")
                .arg(prjFileName);
        return(false);
    }
    QTextStream strOut(&file);

    int numberOfProcesses=1;

    strOut<<"Point Cloud File Management Project\n";
    strOut<<"- Number of processes ......................................# ";
    strOut<<QString::number(numberOfProcesses);
    strOut<<"\n";
    strOut<<"- Process (CREATE_PCF,ADD_POINTCLOUD,ADD_ROI,WRITE_PCFS) ...# ";
    strOut<<POINTCLOUDFILE_PROCESS_WRITE_PCFS_TAG;
    strOut<<"\n";
    strOut<<"  - PoinCloudFileManager path ..............................# \"";
    strOut<<pcfPath;
    strOut<<"\"\n";
    strOut<<"  - Point Cloud Files Suffix ...............................# ";
    strOut<<suffix;
    strOut<<"\n";
    strOut<<"  - Point Cloud Files Output Path (empty for current ) .....# ";
    strOut<<outputPath;
    strOut<<"\n";
    file.close();
    if(!processProjectFile(prjFileName,
                           strAuxError))
    {
        strError=QObject::tr("PointCloudDbManager::exportProcessedPointCloudFiles");
        strError+=QObject::tr("\nError calling setFromProjectFile for project file:\n%1\nError:\n%2")
                .arg(prjFileName).arg(strAuxError);
        return(false);
    }
    return(true);
}

bool PointCloudFileManager::getColorNumberOfBytes(QString pcfPath,
                                                  int &numberOfBytes,
                                                  QString &strError)
{
    QString strAuxError;
    if(!mPtrPcFiles.contains(pcfPath))
    {
        if(!openPointCloudFile(pcfPath,
                               strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::getColorNumberOfBytes");
            strError+=QObject::tr("\nError openning spatialite:\n%1\nError:\n%2")
                    .arg(pcfPath).arg(strAuxError);
            return(false);
        }
    }
    numberOfBytes=mPtrPcFiles[pcfPath]->getColorNumberOfBytes();
    return(true);
}

bool PointCloudFileManager::getProjectCrsEpsgCode(QString pcfPath,
                                                  int &crsEpsgCode,
                                                  QString &strError)
{
    QString strAuxError;
    if(!mPtrPcFiles.contains(pcfPath))
    {
        if(!openPointCloudFile(pcfPath,
                               strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::getProjectCrsEpsgCode");
            strError+=QObject::tr("\nError openning spatialite:\n%1\nError:\n%2")
                    .arg(pcfPath).arg(strAuxError);
            return(false);
        }
    }
    crsEpsgCode=mPtrPcFiles[pcfPath]->getCrsEpsgCode();
    return(true);
}

bool PointCloudFileManager::getGridSizes(QVector<int> &gridSizes,
                                         QString &strError)
{
    gridSizes=mGridSizes;
//    for(int i=0;i<mGridSizes.size();i++)
//    {
//        gridSizes.push_back(qRound(mGridSizes[i]);
//    }
    return(true);
}

bool PointCloudFileManager::getInternalCommands(QVector<QString> &internalCommands,
                                                QString &strError)
{
    if(mInternalCommandsParametersFileName.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::getInternalCommands");
        strError+=QObject::tr("\nInternal tools parameters file name is empty");
        return(false);
    }
    if(!QFile::exists(mInternalCommandsParametersFileName))
    {
        strError=QObject::tr("PointCloudFileManager::getInternalCommands");
        strError+=QObject::tr("\nNot exists Internal tools parameters file name:\n%1")
                .arg(mInternalCommandsParametersFileName);
        return(false);
    }
    internalCommands=mInternalCommands;
    return(true);
}

bool PointCloudFileManager::getLastoolsCommands(QVector<QString> &lastoolsCommands,
                                                QString &strError)
{
    if(mLastoolsCommandsParametersFileName.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::getLastoolsCommands");
        strError+=QObject::tr("\nLastools parameters file name is empty");
        return(false);
    }
    if(!QFile::exists(mLastoolsCommandsParametersFileName))
    {
        strError=QObject::tr("PointCloudFileManager::getLastoolsCommands");
        strError+=QObject::tr("\nNot exists Lastools parameters file name:\n%1")
                .arg(mLastoolsCommandsParametersFileName);
        return(false);
    }
    lastoolsCommands=mLastoolsCommands;
    return(true);
}

bool PointCloudFileManager::getInternalCommandOutputDataFormat(QString &command,
                                                               bool &enableOuputPath,
                                                               bool &enableOutputFile,
                                                               bool &enableSuffix,
                                                               bool &enablePrefix,
                                                               QString &strError)
{
    enableOuputPath=false;
    enableOutputFile=false;
    enableSuffix=false;
    enablePrefix=false;
    if(command.compare(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_ESTIMATE,Qt::CaseInsensitive)==0)
    {
        enableOuputPath=true;
        enableOutputFile=true;
    }
    return(true);
}

bool PointCloudFileManager::getLastoolsCommandsOutputDataFormat(QString &command,
                                                                bool &enableOuputPath,
                                                                bool &enableOutputFile,
                                                                bool &enableSuffix,
                                                                bool &enablePrefix,
                                                                QString &strError)
{
    enableOuputPath=false;
    enableOutputFile=false;
    enableSuffix=false;
    enablePrefix=false;
    if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASTILE,Qt::CaseInsensitive)==0)
    {
        enableOuputPath=true;
        enablePrefix=true;
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASCLIP,Qt::CaseInsensitive)==0)
    {
        enableOuputPath=true;
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LAS2DEM,Qt::CaseInsensitive)==0)
    {
        enableOutputFile=true;
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASBOUNDARY,Qt::CaseInsensitive)==0)
    {
        enableOutputFile=true;
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASHEIGHT,Qt::CaseInsensitive)==0)
    {
        enableOutputFile=true;
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASMERGE,Qt::CaseInsensitive)==0)
    {
        enableOutputFile=true;
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING,Qt::CaseInsensitive)==0)
    {
        enableOutputFile=true;
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING,Qt::CaseInsensitive)==0)
    {
        enableOutputFile=true;
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY,Qt::CaseInsensitive)==0)
    {
        enableOutputFile=true;
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION,Qt::CaseInsensitive)==0)
    {
        enableOutputFile=true;
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_PREPROCESSING,Qt::CaseInsensitive)==0)
    {
        enableOutputFile=true;
//        enableSuffix=false;
    }
    return(true);
}

bool PointCloudFileManager::getLastoolsCommandStrings(QString &command,
                                                      QVector<QString> &inputFiles,
                                                      QString &outputPath,
                                                      QString &outputFile,
                                                      QString &suffix,
                                                      QString &prefix,
                                                      QVector<QString> &lastoolsCommandStrings,
                                                      QString &strError)
{
    lastoolsCommandStrings.clear();
    if(mLastoolsPath.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
        strError+=QObject::tr("\nLastools path is empty");
        return(false);
    }
    QString strAuxError;
    bool enableOuputPath=false;
    bool enableOutputFile=false;
    bool enableSuffix=false;
    bool enablePrefix=false;
    if(!getLastoolsCommandsOutputDataFormat(command,
                                            enableOuputPath,
                                            enableOutputFile,
                                            enableSuffix,
                                            enablePrefix,
                                            strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
        strError+=QObject::tr("\nError recovering output format for command: %1\nError:\n%2")
                .arg(command).arg(strAuxError);
        return(false);
    }
    if(enableOutputFile&&outputFile.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
        strError+=QObject::tr("\nOutput file is empty");
        return(false);
    }
    for(int nif=0;nif<inputFiles.size();nif++)
    {
        QString inputFile=inputFiles[nif];
        if(inputFile.compare(outputFile,Qt::CaseInsensitive)==0)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nOutput file is equal to one input file");
            return(false);
        }
    }
    if(QFile::exists(outputFile))
    {
        if(!QFile::remove(outputFile))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError removing existing output file:\n%1").arg(outputFile);
            return(false);
        }
    }
    if(mPtrLastoolsCommandsParameters==NULL)
    {
        ParametersManager* ptrParametersManager=new ParametersManager();
        if(!ptrParametersManager->loadFromXml(mLastoolsCommandsParametersFileName,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError loading parameters manager from file:\n%1\nError:\n%2")
                    .arg(mLastoolsCommandsParametersFileName).arg(strAuxError);
            delete(ptrParametersManager);
            return(false);
        }
        mPtrLastoolsCommandsParameters=ptrParametersManager;
    }
    if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASTILE,Qt::CaseInsensitive)==0)
    {
        QVector<Parameter *> ptrParameters;
        bool onlyEnabled=true;
        if(!mPtrLastoolsCommandsParameters->getParametersByCommand(command,
                                                                   ptrParameters,
                                                                   onlyEnabled))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            return(false);
        }
        QString parametersString;
        for(int np=0;np<ptrParameters.size();np++)
        {
            Parameter* ptrParameter=ptrParameters[np];
            if(!ptrParameter->isEnabled()) continue;
            QString code=ptrParameter->getCode();
            QString tag=ptrParameter->getTag();
            QString type=ptrParameter->getType();
            if(type.compare(PARAMETER_TYPE_STRING,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_STRING_EN,Qt::CaseInsensitive)==0)
            {
                QString strValue;
                ptrParameter->getValue(strValue);
                strValue=strValue.trimmed();
                if(strValue.isEmpty()) continue;
            }
            parametersString+=" ";
            parametersString+=tag;
            if(type.compare(PARAMETER_TYPE_EMPTY_STRING,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_EMPTY_STRING_EN,Qt::CaseInsensitive)==0)
            {
                continue;
            }
            QString strValue;
            ptrParameter->getValue(strValue);
            parametersString+=" ";
            if(type.compare(PARAMETER_TYPE_DOUBLE,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_DOUBLE_EN,Qt::CaseInsensitive)==0)
            {
                int precision=ptrParameter->getPrintPrecision();
                strValue=QString::number(strValue.toDouble(),'f',precision);
            }
            parametersString+=" ";
            parametersString+=strValue;
        }
        for(int nf=0;nf<inputFiles.size();nf++)
        {
            QString commandString=mLastoolsPath+"/";
            commandString+=command;
            commandString+=" -i ";
            commandString+=inputFiles.at(nf);
            commandString+=parametersString;
            if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASTILE,Qt::CaseInsensitive)==0)
            {
                commandString+=" -odir ";
                commandString+=outputPath;
                if(!prefix.isEmpty())
                {
                    commandString+="/";
                    commandString+=prefix;
                }
            }
            lastoolsCommandStrings.push_back(commandString);
        }
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASCLIP,Qt::CaseInsensitive)==0)
    {
        QVector<Parameter *> ptrParameters;
        bool onlyEnabled=true;
        if(!mPtrLastoolsCommandsParameters->getParametersByCommand(command,
                                                                   ptrParameters,
                                                                   onlyEnabled))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            return(false);
        }
        QString parametersString;
        for(int np=0;np<ptrParameters.size();np++)
        {
            Parameter* ptrParameter=ptrParameters[np];
            if(!ptrParameter->isEnabled()) continue;
            QString code=ptrParameter->getCode();
            QString tag=ptrParameter->getTag();
            QString type=ptrParameter->getType();
            if(type.compare(PARAMETER_TYPE_STRING,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_STRING_EN,Qt::CaseInsensitive)==0)
            {
                QString strValue;
                ptrParameter->getValue(strValue);
                strValue=strValue.trimmed();
                if(strValue.isEmpty()) continue;
            }
            parametersString+=" ";
            parametersString+=tag;
            if(type.compare(PARAMETER_TYPE_EMPTY_STRING,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_EMPTY_STRING_EN,Qt::CaseInsensitive)==0)
            {
                continue;
            }
            QString strValue;
            ptrParameter->getValue(strValue);
            parametersString+=" ";
            if(type.compare(PARAMETER_TYPE_DOUBLE,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_DOUBLE_EN,Qt::CaseInsensitive)==0)
            {
                int precision=ptrParameter->getPrintPrecision();
                strValue=QString::number(strValue.toDouble(),'f',precision);
            }
            parametersString+=" ";
            parametersString+=strValue;
        }
        for(int nf=0;nf<inputFiles.size();nf++)
        {
            QString commandString=mLastoolsPath+"/";
            commandString+=command;
            commandString+=" -i ";
            commandString+=inputFiles.at(nf);
            commandString+=parametersString;
            if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASCLIP,Qt::CaseInsensitive)==0)
            {
                commandString+=" -odir ";
                commandString+=outputPath;
                if(!prefix.isEmpty())
                {
                    commandString+="/";
                    commandString+=prefix;
                }
            }
            lastoolsCommandStrings.push_back(commandString);
        }
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASHEIGHT,Qt::CaseInsensitive)==0)
    {
        QVector<Parameter *> ptrParameters;
        bool onlyEnabled=true;
        if(!mPtrLastoolsCommandsParameters->getParametersByCommand(command,
                                                                   ptrParameters,
                                                                   onlyEnabled))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            return(false);
        }
        QString parametersString;
        for(int np=0;np<ptrParameters.size();np++)
        {
            Parameter* ptrParameter=ptrParameters[np];
            if(!ptrParameter->isEnabled()) continue;
            QString code=ptrParameter->getCode();
            QString tag=ptrParameter->getTag();
            QString type=ptrParameter->getType();
            if(type.compare(PARAMETER_TYPE_STRING,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_STRING_EN,Qt::CaseInsensitive)==0)
            {
                QString strValue;
                ptrParameter->getValue(strValue);
                strValue=strValue.trimmed();
                if(strValue.isEmpty()) continue;
            }
            parametersString+=" ";
            parametersString+=tag;
            if(type.compare(PARAMETER_TYPE_EMPTY_STRING,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_EMPTY_STRING_EN,Qt::CaseInsensitive)==0)
            {
                continue;
            }
            QString strValue;
            ptrParameter->getValue(strValue);
            parametersString+=" ";
            if(type.compare(PARAMETER_TYPE_DOUBLE,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_DOUBLE_EN,Qt::CaseInsensitive)==0)
            {
                int precision=ptrParameter->getPrintPrecision();
                strValue=QString::number(strValue.toDouble(),'f',precision);
            }
            parametersString+=" ";
            parametersString+=strValue;
        }
        for(int nf=0;nf<inputFiles.size();nf++)
        {
            QString commandString=mLastoolsPath+"/";
            commandString+=command;
            commandString+=" -i ";
            commandString+=inputFiles.at(nf);
            commandString+=parametersString;
            if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASHEIGHT,Qt::CaseInsensitive)==0)
            {
                commandString+=" -odir ";
                commandString+=outputPath;
                if(!prefix.isEmpty())
                {
                    commandString+="/";
                    commandString+=prefix;
                }
            }
            lastoolsCommandStrings.push_back(commandString);
        }
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LAS2DEM,Qt::CaseInsensitive)==0)
    {
        if(inputFiles.size()!=1)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nOnly one file input is valid for this command: %1")
                    .arg(command);
            return(false);
        }
        QVector<Parameter *> ptrParameters;
        bool onlyEnabled=true;
        if(!mPtrLastoolsCommandsParameters->getParametersByCommand(command,
                                                                   ptrParameters,
                                                                   onlyEnabled))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            return(false);
        }
//        QString strInputFiles;
//        for(int nf=0;nf<inputFiles.size();nf++)
//        {
//            QString inputFile=inputFiles.at(nf);
//            if(inputFile.contains(" "))
//            {
//                inputFile="\""+inputFile+"\"";
//            }
//            strInputFiles+=" -i ";
//            strInputFiles+=inputFile;
//        }
        QString parametersString;
        for(int np=0;np<ptrParameters.size();np++)
        {
            Parameter* ptrParameter=ptrParameters[np];
            if(!ptrParameter->isEnabled()) continue;
            QString code=ptrParameter->getCode();
            QString tag=ptrParameter->getTag();
            QString type=ptrParameter->getType();
            QString strValue;
            ptrParameter->getValue(strValue);
            strValue=strValue.trimmed();
            if(type.compare(PARAMETER_TYPE_STRING,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_STRING_EN,Qt::CaseInsensitive)==0)
            {
                if(strValue.isEmpty()) continue;
                if(code.compare(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_LAS2DEM_CLASSES,Qt::CaseInsensitive)==0)
                {
                    QStringList strValues=strValue.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
                    QString newStrValue;
                    for(int i=0;i<strValues.size();i++)
                    {
                        QString strAuxValue=strValues.at(i);
                        bool okToInt=false;
                        int intValue=strAuxValue.toInt(&okToInt);
                        if(!okToInt)
                        {
                            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                                    .arg(command);
                            strError+=QObject::tr("\nFor parameter: %1").arg(code);
                            strError+=QObject::tr("\nValue: %1 is not an integer").arg(strAuxValue);
                            return(false);
                        }
                        if(i>0) newStrValue+=" ";
                        newStrValue+=QString::number(intValue);
                    }
                    strValue=newStrValue;
                }
            }
            parametersString+=" ";
            parametersString+=tag;
            if(type.compare(PARAMETER_TYPE_EMPTY_STRING,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_EMPTY_STRING_EN,Qt::CaseInsensitive)==0)
            {
                continue;
            }
            if(type.compare(PARAMETER_TYPE_DOUBLE,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_DOUBLE_EN,Qt::CaseInsensitive)==0)
            {
                int precision=ptrParameter->getPrintPrecision();
                strValue=QString::number(strValue.toDouble(),'f',precision);
            }
//            if(type.compare(PARAMETER_TYPE_VECTOR_INTEGER,Qt::CaseInsensitive)==0
//                    ||type.compare(PARAMETER_TYPE_VECTOR_INTEGER_EN,Qt::CaseInsensitive)==0)
//            {
//                QStringList strValues=strValue.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
//                for(int i=0;i<strValues.size();i++)
//                {
//                    QString strAuxValue=strValues.at(i);
//                    bool okToInt=false;
//                    int intValue=strAuxValue.toInt(&okToInt);
//                    if(!okToInt)
//                    {
//                        strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
//                        strError+=QObject::tr("\nError getting parameters for lastools command: %1")
//                                .arg(command);
//                        strError+=QObject::tr("\nFor parameter: %1").arg(code);
//                        strError+=QObject::tr("\nValue: %1 is not an integer").arg(strAuxValue);
//                        return(false);
//                    }
//                }
//                strValue=strValue.replace(ENUM_CHARACTER_SEPARATOR," ");
//            }
            parametersString+=" ";
            parametersString+=strValue;
        }
        QString inputFile=inputFiles.at(0);
        if(inputFile.contains(" "))
        {
            inputFile="\""+inputFile+"\"";
        }
        if(outputFile.contains(" "))
        {
            outputFile="\""+outputFile+"\"";
        }
        QString commandString=mLastoolsPath+"/";
        commandString+=command;
        commandString+=" -i ";
        commandString+=inputFile;
//        commandString+=strInputFiles;
        commandString+=parametersString;
        commandString+=" -o ";
        commandString+=outputFile;
        lastoolsCommandStrings.push_back(commandString);
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASBOUNDARY,Qt::CaseInsensitive)==0)
    {
        QVector<Parameter *> ptrParameters;
        bool onlyEnabled=true;
        if(!mPtrLastoolsCommandsParameters->getParametersByCommand(command,
                                                                   ptrParameters,
                                                                   onlyEnabled))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            return(false);
        }
        QString strInputFiles;
        for(int nf=0;nf<inputFiles.size();nf++)
        {
            QString inputFile=inputFiles.at(nf);
            if(inputFile.contains(" "))
            {
                inputFile="\""+inputFile+"\"";
            }
            strInputFiles+=" -i ";
            strInputFiles+=inputFile;
        }
        QString parametersString;
        for(int np=0;np<ptrParameters.size();np++)
        {
            Parameter* ptrParameter=ptrParameters[np];
            if(!ptrParameter->isEnabled()) continue;
            QString code=ptrParameter->getCode();
            QString tag=ptrParameter->getTag();
            QString type=ptrParameter->getType();
            if(!ptrParameter->isEnabled())
            {
                continue;
            }
            if(type.compare(PARAMETER_TYPE_STRING,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_STRING_EN,Qt::CaseInsensitive)==0)
            {
                QString strValue;
                ptrParameter->getValue(strValue);
                strValue=strValue.trimmed();
                if(strValue.isEmpty()) continue;
            }
            parametersString+=" ";
            parametersString+=tag;
            if(type.compare(PARAMETER_TYPE_EMPTY_STRING,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_EMPTY_STRING_EN,Qt::CaseInsensitive)==0)
            {
                continue;
            }
            QString strValue;
            ptrParameter->getValue(strValue);
            parametersString+=" ";
            if(type.compare(PARAMETER_TYPE_DOUBLE,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_DOUBLE_EN,Qt::CaseInsensitive)==0)
            {
                int precision=ptrParameter->getPrintPrecision();
                strValue=QString::number(strValue.toDouble(),'f',precision);
            }
            parametersString+=" ";
            parametersString+=strValue;
        }
        QString commandString=mLastoolsPath+"/";
        commandString+=command;
        commandString+=strInputFiles;
        commandString+=parametersString;
        commandString+=" -o ";
        commandString+=outputFile;
        lastoolsCommandStrings.push_back(commandString);
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_LASMERGE,Qt::CaseInsensitive)==0)
    {
        if(inputFiles.size()==1)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nOnly one file input is invalid for this command: %1")
                    .arg(command);
            return(false);
        }
        QString strInputFiles;
        for(int nf=0;nf<inputFiles.size();nf++)
        {
            QString inputFile=inputFiles.at(nf);
            if(inputFile.contains(" "))
            {
                inputFile="\""+inputFile+"\"";
            }
            strInputFiles+=" -i ";
            strInputFiles+=inputFile;
        }
        QVector<Parameter *> ptrParameters;
        bool onlyEnabled=true;
        if(!mPtrLastoolsCommandsParameters->getParametersByCommand(command,
                                                                   ptrParameters,
                                                                   onlyEnabled))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            return(false);
        }
        QString parametersString;
        for(int np=0;np<ptrParameters.size();np++)
        {
            Parameter* ptrParameter=ptrParameters[np];
            if(!ptrParameter->isEnabled()) continue;
            QString code=ptrParameter->getCode();
            QString tag=ptrParameter->getTag();
            QString type=ptrParameter->getType();
            QString strValue;
            ptrParameter->getValue(strValue);
            strValue=strValue.trimmed();
            parametersString+=" ";
            parametersString+=tag;
            if(type.compare(PARAMETER_TYPE_EMPTY_STRING,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_EMPTY_STRING_EN,Qt::CaseInsensitive)==0)
            {
                continue;
            }
            if(type.compare(PARAMETER_TYPE_DOUBLE,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_DOUBLE_EN,Qt::CaseInsensitive)==0)
            {
                int precision=ptrParameter->getPrintPrecision();
                strValue=QString::number(strValue.toDouble(),'f',precision);
            }
            parametersString+=" ";
            parametersString+=strValue;
        }
        if(outputFile.contains(" "))
        {
            outputFile="\""+outputFile+"\"";
        }
        QString commandString=mLastoolsPath+"/";
        commandString+=command;
        commandString+=strInputFiles;
        commandString+=parametersString;
        commandString+=" -o ";
        commandString+=outputFile;
        lastoolsCommandStrings.push_back(commandString);
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING,Qt::CaseInsensitive)==0)
    {
        QString parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_TEMPORAL_PATH;
        Parameter* ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString temporalBasePath;
        ptrParameter->getValue(temporalBasePath);
        QDir auxDir=QDir::currentPath();
        if(!auxDir.exists(temporalBasePath))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nNot exists path: %1").arg(temporalBasePath);
            return(false);
        }
        bool removeOnlyContent=true;
        if(!removeDir(temporalBasePath,removeOnlyContent))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nError removing contents in path: %1").arg(temporalBasePath);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASCLIP_FIRST_SHAPEFILE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasClipFirstShapefile;
        ptrParameter->getValue(lasClipFirstShapefile);
        if(!QFile::exists(lasClipFirstShapefile))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nNot exists file: %1").arg(lasClipFirstShapefile);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASCLIP_SECOND_SHAPEFILE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasClipSecondShapefile;
        ptrParameter->getValue(lasClipSecondShapefile);
        if(!QFile::exists(lasClipSecondShapefile))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nNot exists file: %1").arg(lasClipSecondShapefile);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASTILE_TILE_SIZE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lastileTileSizeStr;
        ptrParameter->getValue(lastileTileSizeStr);
        double dblValue;
        bool okToDouble=false;
        dblValue=lastileTileSizeStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lastileTileSizeStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASTILE_TILE_BUFFER;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lastileBufferStr;
        ptrParameter->getValue(lastileBufferStr);
        okToDouble=false;
        dblValue=lastileBufferStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lastileBufferStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASGROUND_STEP;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasgroundStepStr;
        ptrParameter->getValue(lasgroundStepStr);
        okToDouble=false;
        dblValue=lasgroundStepStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasgroundStepStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASTHIN_AVT;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasthinAvtStr;
        ptrParameter->getValue(lasthinAvtStr);
        okToDouble=false;
        dblValue=lasthinAvtStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasthinAvtStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASTHIN_AMD;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasthinAmdStr;
        ptrParameter->getValue(lasthinAmdStr);
        okToDouble=false;
        dblValue=lasthinAmdStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasthinAmdStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASGROUND_CORES;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasgroundCoresStr;
        ptrParameter->getValue(lasgroundCoresStr);
        bool okToInt=false;
        int intValue=lasgroundCoresStr.toInt(&okToInt);
        if(!okToInt)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(lasgroundCoresStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASTILE_CORES;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lastileCoresStr;
        ptrParameter->getValue(lastileCoresStr);
        okToInt=false;
        intValue=lastileCoresStr.toInt(&okToInt);
        if(!okToInt)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(lastileCoresStr);
            return(false);
        }
        /*
        parameterCode=POINTCLOUDDB_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LAS2DEM_STEP;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        QString las2demStepStr;
        ptrParameter->getValue(las2demStepStr);
        okToDouble=false;
        dblValue=las2demStepStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudDbManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(las2demStepStr);
            return(false);
        }
        parameterCode=POINTCLOUDDB_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LAS2DEM_NODATA;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        QString las2demNodataStr;
        ptrParameter->getValue(las2demNodataStr);
        okToInt=false;
        intValue=lasthinAmdStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudDbManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(las2demNodataStr);
            return(false);
        }
        */
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASCLIP_THIRD_SHAPEFILE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasClipThirdShapefile;
        ptrParameter->getValue(lasClipThirdShapefile);
        if(!QFile::exists(lasClipThirdShapefile))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nNot exists file: %1").arg(lasClipThirdShapefile);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASTHIN_STEP_OUT_POWERLINE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasthinStepOutPowerlineStr;
        ptrParameter->getValue(lasthinStepOutPowerlineStr);
        okToDouble=false;
        dblValue=lasthinStepOutPowerlineStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasthinStepOutPowerlineStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASNOISE_STEP;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasnoiseStepStr;
        ptrParameter->getValue(lasnoiseStepStr);
        okToDouble=false;
        dblValue=lasnoiseStepStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasnoiseStepStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASNOISE_ISOLATED;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasnoiseIsolatedStr;
        ptrParameter->getValue(lasnoiseIsolatedStr);
        okToInt=false;
        intValue=lasnoiseIsolatedStr.toDouble(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(lasnoiseIsolatedStr);
            return(false);
        }
        // Comprobar que no hay ficheros con nombres repetidos
        QVector<QString> inputFileBaseNames;
        for(int nf=0;nf<inputFiles.size();nf++)
        {
            QFileInfo fileInfo(inputFiles.at(nf));
            QString completeBaseName=fileInfo.completeBaseName();
            for(int nfa=0;nfa<inputFileBaseNames.size();nfa++)
            {
                if(inputFileBaseNames[nfa].compare(completeBaseName,Qt::CaseInsensitive)==0)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nRepeated file base name: %1")
                            .arg(completeBaseName);
                    return(false);
                }
            }
            inputFileBaseNames.push_back(completeBaseName);
        }
        QVector<QString> pathToRemove;
        // Paso 0: Crear las carpetas de salidas
        QVector<QString> auxiliaryPaths;
        QString auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_FIRST_CLIP;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_LASNOISE;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_SECOND_CLIP;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_FIRST_UNION;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_SECOND_UNION;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_GROUND;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_TILES;
        auxiliaryPaths.push_back(auxiliaryPath);
        for(int np=0;np<auxiliaryPaths.size();np++)
        {
            QString auxiliaryPath=auxiliaryPaths.at(np);
            if(!auxDir.mkpath(auxiliaryPath))
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError making path: %1").arg(auxiliaryPath);
                return(false);
            }
        }
        // Paso 1: Primer clip
        QString firstClipOutputPath;
        {
            firstClipOutputPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_FIRST_CLIP;
            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASCLIP_FIRST_SHAPEFILE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            for(int nf=0;nf<inputFiles.size();nf++)
            {
                QString inputFile=inputFiles.at(nf);
                QFileInfo fileInfo(inputFile);
                QString firstClipOutputFile=firstClipOutputPath+"/"+fileInfo.completeBaseName()+".laz";
                if(inputFile.contains(" "))
                {
                    inputFile="\""+inputFile+"\"";
                }
                if(firstClipOutputFile.contains(" "))
                {
                    firstClipOutputFile="\""+firstClipOutputFile+"\"";
                }
                QString commandString=mLastoolsPath+"\\";
                commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASCLIP;
                commandString+=" -i ";
                commandString+=inputFile;
                commandString+=" ";
                commandString+=parameterTag;
                commandString+=" ";
                commandString+=lasClipFirstShapefile;
                commandString+=" -v -o ";
                commandString+=firstClipOutputFile;
                lastoolsCommandStrings.push_back(commandString);
            }
        }
        pathToRemove.push_back(firstClipOutputPath);
        // Paso 2: Combinar todos los laz en uno
//        if(inputFiles.size()>1)
        QString firstUnionOutputFileName;
        QString firstUnionOutputPath;
        {
            QString inputString=firstClipOutputPath;
            inputString+="/*.laz";
            if(inputString.contains(" "))
            {
                inputString="\""+inputString+"\"";
            }
            firstUnionOutputPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_FIRST_UNION;
            firstUnionOutputFileName=firstUnionOutputPath+"/";
            firstUnionOutputFileName+=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_FIRST_UNION_OUTPUT_FILE_BASENANE;
            firstUnionOutputFileName+=".laz";
            if(firstUnionOutputFileName.contains(" "))
            {
                firstUnionOutputFileName="\""+firstUnionOutputFileName+"\"";
            }
            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASMERGE;
            commandString+=" -i ";
            commandString+=inputString;
            commandString+=" ";
            commandString+=" -v -o ";
            commandString+=firstUnionOutputFileName;
            lastoolsCommandStrings.push_back(commandString);
        }
        pathToRemove.push_back(firstUnionOutputPath);
        // Paso 3: Eliminar el ruido
        QString lasnoiseOutputFileName;
        QString lasnoiseOutputPath;
        {
            lasnoiseOutputPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_LASNOISE;
            lasnoiseOutputFileName=lasnoiseOutputPath+"/";
            lasnoiseOutputFileName+=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_LANOISE_OUTPUT_FILE_BASENANE;
            lasnoiseOutputFileName+=".laz";
            if(lasnoiseOutputFileName.contains(" "))
            {
                lasnoiseOutputFileName="\""+lasnoiseOutputFileName+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASNOISE;
            commandString+=" -i ";
            commandString+=firstUnionOutputFileName;

            commandString+=" ";
            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASNOISE_STEP;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasnoiseStepStr;

            commandString+=" ";
            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASNOISE_ISOLATED;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasnoiseIsolatedStr;

            commandString+=" -remove_noise -v -olaz -o ";
            commandString+=lasnoiseOutputFileName;
            lastoolsCommandStrings.push_back(commandString);
        }
        pathToRemove.push_back(lasnoiseOutputPath);

        // Paso 4: tileado
        QString tilesOutputPath;
        {
            tilesOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_TILES;
            if(tilesOutputPath.contains(" "))
            {
                tilesOutputPath="\""+tilesOutputPath+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASTILE;
            commandString+=" -i ";
            commandString+=lasnoiseOutputFileName;//firstUnionOutputFileName;

            commandString+=" ";
            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASTILE_CORES;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lastileCoresStr;

            commandString+=" ";
            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASTILE_TILE_BUFFER;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lastileBufferStr;

            commandString+=" ";
            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASTILE_TILE_SIZE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lastileTileSizeStr;

            commandString+=" -flag_as_withheld -v -olaz -odir ";
            commandString+=tilesOutputPath;
            lastoolsCommandStrings.push_back(commandString);

        }
        pathToRemove.push_back(tilesOutputPath);
        // Paso 5: Clasificacion del suelo
        QString groundOutputPath;
        {
            groundOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_GROUND;
            if(groundOutputPath.contains(" "))
            {
                groundOutputPath="\""+groundOutputPath+"\"";
            }

            QString strInput=tilesOutputPath;
            strInput+="\\*.laz";

            QString commandString=mLastoolsPath+"/";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASGROUND;
            commandString+=" -i ";
            commandString+=strInput;

            commandString+=" ";
            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASGROUND_CORES;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasgroundCoresStr;

            commandString+=" ";
            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASGROUND_STEP;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasgroundStepStr;

            commandString+=" -v -olaz -odir ";
            commandString+=groundOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        pathToRemove.push_back(groundOutputPath);
        // Paso 6: Segundo clip
        QString secondClipOutputPath;
        {
            secondClipOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_SECOND_CLIP;
            if(secondClipOutputPath.contains(" "))
            {
                secondClipOutputPath="\""+secondClipOutputPath+"\"";
            }
            QString strInput=groundOutputPath+"\\*.laz";
            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASCLIP_SECOND_SHAPEFILE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            QString commandString=mLastoolsPath+"/";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASCLIP;
            commandString+=" -i ";
            commandString+=strInput;
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasClipSecondShapefile;
            commandString+=" -v -olaz -odir ";
            commandString+=secondClipOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        pathToRemove.push_back(secondClipOutputPath);
        // Paso 7: Segunda union
        QString secondUnionOutputFileName;
        QString secondUnionOutputPath;
        {
            QString inputString;
            inputString=secondClipOutputPath;
            inputString+="\\*.laz";
            if(inputString.contains(" "))
            {
                inputString="\""+inputString+"\"";
            }
            secondUnionOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_SECOND_UNION;
            secondUnionOutputFileName=secondUnionOutputPath+"\\";
            secondUnionOutputFileName+=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_PATH_SECOND_UNION_OUTPUT_FILE_BASENANE;
            secondUnionOutputFileName+=".laz";
            if(secondUnionOutputFileName.contains(" "))
            {
                secondUnionOutputFileName="\""+secondUnionOutputFileName+"\"";
            }
            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASMERGE;
            commandString+=" -i ";
            commandString+=inputString;
            commandString+=" ";
            commandString+=" -v -drop_withheld -o ";
            commandString+=secondUnionOutputFileName;
            lastoolsCommandStrings.push_back(commandString);
        }
        pathToRemove.push_back(secondUnionOutputPath);
        QString dtmOutputFile=outputFile;
        QString originalOutputFile=outputFile;
        QFileInfo outputFileInfo(outputFile);
        // Paso 8: Adelgazar la clase ground
        {
            if(outputFile.contains(" "))
            {
                outputFile="\""+outputFile+"\"";
            }
            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASTHIN;
            commandString+=" -i ";
            commandString+=secondUnionOutputFileName;
            commandString+=" -ignore_class 1";

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASTHIN_AVT;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterValue;
            ptrParameter->getValue(parameterValue);
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=parameterValue;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASTHIN_AMD;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            ptrParameter->getValue(parameterValue);
//            QString parameterTag=ptrParameter->getTag();
//            commandString+=" ";
//            commandString+=parameterTag;
            commandString+=" ";
            commandString+=parameterValue;

            commandString+=" -v -o ";
            commandString+=outputFile;
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 9: Reclasificar a clase 13 los puntos fuera de un buffer de 7.5 m
        // Para quitarle las comillas que se hayan podido añadir
        outputFile=originalOutputFile;
        QString outputFileAux=outputFileInfo.absolutePath()+"/"+outputFileInfo.completeBaseName()+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_OUTPUT_FILE_AUX;
        {
            if(outputFile.contains(" "))
            {
                outputFile="\""+outputFile+"\"";
            }
            if(outputFileAux.contains(" "))
            {
                outputFileAux="\""+outputFileAux+"\"";
            }
            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASCLIP_THIRD_SHAPEFILE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            QString commandString=mLastoolsPath+"/";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASCLIP;
            commandString+=" -i ";
            commandString+=outputFile;
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasClipThirdShapefile;
            commandString+=" -ignore_class 2 -classify 13 -v -o ";
            commandString+=outputFileAux;
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 10: Adelgazar los puntos exteriores
        // Para quitarle las comillas que se hayan podido añadir
        outputFile=originalOutputFile;
        outputFileAux=outputFileInfo.absolutePath()+"/"+outputFileInfo.completeBaseName()+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_OUTPUT_FILE_AUX;
        QString outputFileAux2=outputFileInfo.absolutePath()+"/"+outputFileInfo.completeBaseName()+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_OUTPUT_FILE_AUX2;
        {
//            if(outputFile.contains(" "))
//            {
//                outputFile="\""+outputFile+"\"";
//            }
            if(outputFileAux2.contains(" "))
            {
                outputFileAux2="\""+outputFileAux2+"\"";
            }
            if(outputFileAux.contains(" "))
            {
                outputFileAux="\""+outputFileAux+"\"";
            }
            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LASTHIN_STEP_OUT_POWERLINE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            QString commandString=mLastoolsPath+"/";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASTHIN;
            commandString+=" -i ";
            commandString+=outputFileAux;
            commandString+=" -ignore_class 1 2 -highest ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasthinStepOutPowerlineStr;
            commandString+=" -v -o ";
            commandString+=outputFileAux2;
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 11: Reclasificar del 13 al 1
        // Para quitarle las comillas que se hayan podido añadir
        outputFile=originalOutputFile;
        {
            if(outputFile.contains(" "))
            {
                outputFile="\""+outputFile+"\"";
            }
            QString commandString=mLastoolsPath+"/";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LAS2LAS;
            commandString+=" -i ";
            commandString+=outputFileAux2;
            commandString+=" -change_classification_from_to 13 1";
            commandString+=" -o ";
            commandString+=outputFile;
            lastoolsCommandStrings.push_back(commandString);
        }
        /*
        // Paso 11: Generar DTM
//        // Para quitarle las comillas que se hayan podido añadir
//        outputFile=originalOutputFile;
        {
            QFileInfo dtmFileInfo(dtmOutputFile);
            dtmOutputFile=dtmFileInfo.absolutePath()+"/"+dtmFileInfo.completeBaseName()+POINTCLOUDDB_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LAS2DEM_FILE_SUFFIX;
            if(dtmOutputFile.contains(" "))
            {
                dtmOutputFile="\""+dtmOutputFile+"\"";
            }
            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDDB_LASTOOLS_COMMAND_LAS2DEM;
            commandString+=" -i ";
            commandString+=outputFile;
            commandString+=" -keep_class 2";

            parameterCode=POINTCLOUDDB_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LAS2DEM_STEP;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            QString parameterValue;
            ptrParameter->getValue(parameterValue);
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=parameterValue;

            parameterCode=POINTCLOUDDB_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_LAS2DEM_NODATA;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            ptrParameter->getValue(parameterValue);
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=parameterValue;

            commandString+=" -v -o ";
            commandString+=dtmOutputFile;
            lastoolsCommandStrings.push_back(commandString);
        }
        */
        // Paso 12: remove temporal paths
        for(int np=0;np<pathToRemove.size();np++)
        {
            QString commandString="rd /s /q \"";
            commandString+=pathToRemove[np];
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 13: eliminar ficheros auxiliares
        {
            outputFileAux=outputFileInfo.absolutePath()+"/"+outputFileInfo.completeBaseName()+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_OUTPUT_FILE_AUX;
            QString commandString="del /q \"";
            commandString+=QFileInfo(outputFileAux).filePath().replace("/","\\");
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        {
            outputFileAux2=outputFileInfo.absolutePath()+"/"+outputFileInfo.completeBaseName()+POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_OUTPUT_FILE_AUX2;
            QString commandString="del /q \"";
            commandString+=QFileInfo(outputFileAux2).filePath().replace("/","\\");
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        {
            QString repeatedLasFileName=QFileInfo(originalOutputFile).absolutePath();
            repeatedLasFileName+="/";
            repeatedLasFileName+=QFileInfo(originalOutputFile).completeBaseName();
            repeatedLasFileName+=POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING_OUTPUT_FILE_REPEATED;
            QString commandString="del /q \"";
            commandString+=QFileInfo(repeatedLasFileName).filePath().replace("/","\\");
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_PREPROCESSING,Qt::CaseInsensitive)==0)
    {
        if(inputFiles.size()!=1)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nOnly one file input is valid for this command: %1")
                    .arg(command);
            return(false);
        }
        QVector<Parameter *> ptrParameters;
        bool onlyEnabled=true;
        if(!mPtrLastoolsCommandsParameters->getParametersByCommand(command,
                                                                   ptrParameters,
                                                                   onlyEnabled))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            return(false);
        }
        QString parametersString;
        for(int np=0;np<ptrParameters.size();np++)
        {
            Parameter* ptrParameter=ptrParameters[np];
            QString code=ptrParameter->getCode();
            QString tag=ptrParameter->getTag();
            QString type=ptrParameter->getType();
            parametersString+=" ";
            parametersString+=tag;
            if(type.compare(PARAMETER_TYPE_EMPTY_STRING,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_EMPTY_STRING_EN,Qt::CaseInsensitive)==0)
            {
                continue;
            }
            QString strValue;
            ptrParameter->getValue(strValue);
            parametersString+=" ";
            if(type.compare(PARAMETER_TYPE_DOUBLE,Qt::CaseInsensitive)==0
                    ||type.compare(PARAMETER_TYPE_DOUBLE_EN,Qt::CaseInsensitive)==0)
            {
                int precision=ptrParameter->getPrintPrecision();
                strValue=QString::number(strValue.toDouble(),'f',precision);
            }
            parametersString+=" ";
            parametersString+=strValue;
        }
        QString inputFile=inputFiles.at(0);
        if(inputFile.contains(" "))
        {
            inputFile="\""+inputFile+"\"";
        }
        if(outputFile.contains(" "))
        {
            outputFile="\""+outputFile+"\"";
        }
        QString commandString=mLastoolsPath+"/";
        commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASHEIGHT;
        commandString+=" -i ";
        commandString+=inputFile;
        commandString+=parametersString;
        commandString+=" -replace_z -all_ground_points";
        commandString+=" -o ";
        commandString+=outputFile;
        lastoolsCommandStrings.push_back(commandString);
    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING,Qt::CaseInsensitive)==0)
    {
        int intValue;
        bool okToInt;
        QString parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_TEMPORAL_PATH;
        Parameter* ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString temporalBasePath;
        ptrParameter->getValue(temporalBasePath);
        QDir auxDir=QDir::currentPath();
        if(!auxDir.exists(temporalBasePath))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nNot exists path: %1").arg(temporalBasePath);
            return(false);
        }
        bool removeOnlyContent=true;
        if(!removeDir(temporalBasePath,removeOnlyContent))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nError removing contents in path: %1").arg(temporalBasePath);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASTILE_CORES;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lastileCoresStr;
        ptrParameter->getValue(lastileCoresStr);
        okToInt=false;
        intValue=lastileCoresStr.toInt(&okToInt);
        if(!okToInt)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(lastileCoresStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASTILE_TILE_SIZE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lastileTileSizeStr;
        ptrParameter->getValue(lastileTileSizeStr);
        double dblValue;
        bool okToDouble=false;
        dblValue=lastileTileSizeStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lastileTileSizeStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASTILE_TILE_BUFFER;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lastileBufferStr;
        ptrParameter->getValue(lastileBufferStr);
        okToDouble=false;
        dblValue=lastileBufferStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lastileBufferStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_FIRST_LASTHIN_STEP;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString firstLasthinStepStr;
        ptrParameter->getValue(firstLasthinStepStr);
        okToDouble=false;
        dblValue=firstLasthinStepStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(firstLasthinStepStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_FIRST_LASTHIN_PERCENTILE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString firstLasthinPercentileStr;
        ptrParameter->getValue(firstLasthinPercentileStr);
        QStringList strPercentileValues=firstLasthinPercentileStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        for(int i=0;i<strPercentileValues.size();i++)
        {
            QString strAuxValue=strPercentileValues.at(i);
            okToDouble=false;
            dblValue=strAuxValue.toDouble(&okToDouble);
            if(!okToDouble)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                strError+=QObject::tr("\nValue: %1 is not a double").arg(strAuxValue);
                return(false);
            }
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_FIRST_LASTHIN_CLASSIFY_AS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString firstLasthinClassifyAsStr;
        ptrParameter->getValue(firstLasthinClassifyAsStr);
        okToInt=false;
        intValue=firstLasthinClassifyAsStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(firstLasthinClassifyAsStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASNOISE_CLASSIFY_AS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasnoiseClassifyAsStr;
        ptrParameter->getValue(lasnoiseClassifyAsStr);
        okToInt=false;
        intValue=lasnoiseClassifyAsStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(lasnoiseClassifyAsStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASNOISE_IGNORE_CLASS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasnoiseIgnoreClassStr;
        ptrParameter->getValue(lasnoiseIgnoreClassStr);
        okToInt=false;
        intValue=lasnoiseIgnoreClassStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(lasnoiseIgnoreClassStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASNOISE_ISOLATED;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasnoiseIsolatedStr;
        ptrParameter->getValue(lasnoiseIsolatedStr);
        okToDouble=false;
        dblValue=lasnoiseIsolatedStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasnoiseIsolatedStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASNOISE_STEP_Z;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasnoiseStepZStr;
        ptrParameter->getValue(lasnoiseStepZStr);
        okToDouble=false;
        dblValue=lasnoiseStepZStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasnoiseStepZStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASNOISE_STEP_XY;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasnoiseStepXYStr;
        ptrParameter->getValue(lasnoiseStepXYStr);
        okToDouble=false;
        dblValue=lasnoiseStepXYStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasnoiseStepXYStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_FIRST_LASGROUND_IGNORE_CLASS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString firstLasgroundIgnoreClassStr;
        ptrParameter->getValue(firstLasgroundIgnoreClassStr);
        QStringList strFirstLasgroundIgnoreClassValues=firstLasgroundIgnoreClassStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        for(int i=0;i<strFirstLasgroundIgnoreClassValues.size();i++)
        {
            QString strAuxValue=strFirstLasgroundIgnoreClassValues.at(i);
            okToInt=false;
            intValue=strAuxValue.toInt(&okToInt);
            if(!okToInt)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                strError+=QObject::tr("\nValue: %1 is not an integer").arg(strAuxValue);
                return(false);
            }
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_FIRST_LASHEIGHT_CLASSIFY_BELOW;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString firstLasheightClassifyBelowStr;
        ptrParameter->getValue(firstLasheightClassifyBelowStr);
        QStringList strFirstLasheightBelowValues=firstLasheightClassifyBelowStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        QString firstLasheightBelowHeightStr,firstLasheightBelowClassStr;
        for(int i=0;i<strFirstLasheightBelowValues.size();i++)
        {
            QString strAuxValue=strFirstLasheightBelowValues.at(i);
            if(i==0)
            {
                okToDouble=false;
                dblValue=strAuxValue.toDouble(&okToDouble);
                if(!okToDouble)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                    strError+=QObject::tr("\nValue: %1 is not a double").arg(strAuxValue);
                    return(false);
                }
                firstLasheightBelowHeightStr=strAuxValue;
            }
            if(i==1)
            {
                okToDouble=false;
                intValue=qRound(strAuxValue.toDouble(&okToDouble));
                if(!okToDouble)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                    strError+=QObject::tr("\nValue: %1 is not a double").arg(strAuxValue);
                    return(false);
                }
                firstLasheightBelowClassStr=QString::number(intValue);
            }
        }
        firstLasheightClassifyBelowStr=firstLasheightBelowHeightStr+" "+firstLasheightBelowClassStr;
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_FIRST_LASHEIGHT_CLASSIFY_ABOVE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString firstLasheightClassifyAboveStr;
        ptrParameter->getValue(firstLasheightClassifyAboveStr);
        QStringList strFirstLasheightAboveValues=firstLasheightClassifyAboveStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        QString firstLasheightAboveHeightStr,firstLasheightAboveClassStr;
        for(int i=0;i<strFirstLasheightAboveValues.size();i++)
        {
            QString strAuxValue=strFirstLasheightAboveValues.at(i);
            if(i==0)
            {
                okToDouble=false;
                dblValue=strAuxValue.toDouble(&okToDouble);
                if(!okToDouble)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                    strError+=QObject::tr("\nValue: %1 is not a double").arg(strAuxValue);
                    return(false);
                }
                firstLasheightAboveHeightStr=strAuxValue;
            }
            if(i==1)
            {
                okToDouble=false;
                intValue=qRound(strAuxValue.toDouble(&okToDouble));
                if(!okToDouble)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                    strError+=QObject::tr("\nValue: %1 is not a double").arg(strAuxValue);
                    return(false);
                }
                firstLasheightAboveClassStr=QString::number(intValue);
            }
        }
        firstLasheightClassifyAboveStr=firstLasheightAboveHeightStr+" "+firstLasheightAboveClassStr;
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASTHIN_STEP;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasthinStepStr;
        ptrParameter->getValue(secondLasthinStepStr);
        okToDouble=false;
        dblValue=firstLasthinStepStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(secondLasthinStepStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASTHIN_IGNORE_CLASS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasthinIgnoreClassStr;
        ptrParameter->getValue(secondLasthinIgnoreClassStr);
        QStringList strSecondLasthinIgnoreClassValues=secondLasthinIgnoreClassStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        for(int i=0;i<strSecondLasthinIgnoreClassValues.size();i++)
        {
            QString strAuxValue=strSecondLasthinIgnoreClassValues.at(i);
            okToInt=false;
            intValue=strAuxValue.toDouble(&okToInt);
            if(!okToDouble)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                strError+=QObject::tr("\nValue: %1 is not an integer").arg(strAuxValue);
                return(false);
            }
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASTHIN_CLASSIFY_AS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasthinClassifyAsStr;
        ptrParameter->getValue(secondLasthinClassifyAsStr);
        okToInt=false;
        intValue=secondLasthinClassifyAsStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(secondLasthinClassifyAsStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASGROUND_IGNORE_CLASS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasgroundIgnoreClassStr;
        ptrParameter->getValue(secondLasgroundIgnoreClassStr);
        QStringList strSecondLasgroundIgnoreClassValues=secondLasgroundIgnoreClassStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        for(int i=0;i<strSecondLasgroundIgnoreClassValues.size();i++)
        {
            QString strAuxValue=strSecondLasgroundIgnoreClassValues.at(i);
            okToInt=false;
            intValue=strAuxValue.toDouble(&okToInt);
            if(!okToDouble)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                strError+=QObject::tr("\nValue: %1 is not an integer").arg(strAuxValue);
                return(false);
            }
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASGROUND_BULGE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasgroundBulgeStr;
        ptrParameter->getValue(secondLasgroundBulgeStr);
        okToDouble=false;
        dblValue=secondLasgroundBulgeStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(secondLasgroundBulgeStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASHEIGHT_CLASSIFICATION;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasheightClassificationStr;
        ptrParameter->getValue(secondLasheightClassificationStr);
        okToInt=false;
        intValue=secondLasheightClassificationStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(secondLasheightClassificationStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASHEIGHT_DROP_BELOW;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasheightDropBelowStr;
        ptrParameter->getValue(secondLasheightDropBelowStr);
        okToDouble=false;
        dblValue=secondLasheightDropBelowStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(secondLasheightDropBelowStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LAS2LAS_KEEP_CLASS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString las2lasKeepClassStr;
        ptrParameter->getValue(las2lasKeepClassStr);
        okToInt=false;
        intValue=las2lasKeepClassStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(las2lasKeepClassStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_THIRD_LASTHING_IGNORE_CLASS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString thirdLasthinIgnoreClassStr;
        ptrParameter->getValue(thirdLasthinIgnoreClassStr);
        QStringList strThirdLasthinIgnoreClassValues=thirdLasthinIgnoreClassStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        for(int i=0;i<strThirdLasthinIgnoreClassValues.size();i++)
        {
            QString strAuxValue=strThirdLasthinIgnoreClassValues.at(i);
            okToInt=false;
            intValue=strAuxValue.toDouble(&okToInt);
            if(!okToDouble)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                strError+=QObject::tr("\nValue: %1 is not an integer").arg(strAuxValue);
                return(false);
            }
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_THIRD_LASTHING_ADAPTATIVE_VERTICAL_TOLERANCE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString thirdLasthinAdaptativeVerticalToleranceStr;
        ptrParameter->getValue(thirdLasthinAdaptativeVerticalToleranceStr);
        okToDouble=false;
        dblValue=thirdLasthinAdaptativeVerticalToleranceStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(secondLasheightDropBelowStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_THIRD_LASTHING_ADAPTATIVE_MAXIMUM_DISTANCE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString thirdLasthinAdaptativeMaximunDistanceStr;
        ptrParameter->getValue(thirdLasthinAdaptativeMaximunDistanceStr);
        okToDouble=false;
        dblValue=thirdLasthinAdaptativeMaximunDistanceStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(secondLasheightDropBelowStr);
            return(false);
        }


        // Comprobar que no hay ficheros con nombres repetidos
        QVector<QString> inputFileBaseNames;
        for(int nf=0;nf<inputFiles.size();nf++)
        {
            QFileInfo fileInfo(inputFiles.at(nf));
            QString completeBaseName=fileInfo.completeBaseName();
            for(int nfa=0;nfa<inputFileBaseNames.size();nfa++)
            {
                if(inputFileBaseNames[nfa].compare(completeBaseName,Qt::CaseInsensitive)==0)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nRepeated file base name: %1")
                            .arg(completeBaseName);
                    return(false);
                }
            }
            inputFileBaseNames.push_back(completeBaseName);
        }
        // Paso 0: Crear las carpetas de salidas
        QVector<QString> auxiliaryPaths;
        QString auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_TILES;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_FIRST_LASTHIN;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LASNOISE;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_FIRST_LASGROUND;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_FIRST_LASHEIGHT;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_SECOND_LASTHIN;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_SECOND_LASGROUND;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_SECOND_LASHEIGHT;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LAS2LAS;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LASMERGE;
        auxiliaryPaths.push_back(auxiliaryPath);
        for(int np=0;np<auxiliaryPaths.size();np++)
        {
            QString auxiliaryPath=auxiliaryPaths.at(np);
            if(!auxDir.mkpath(auxiliaryPath))
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError making path: %1").arg(auxiliaryPath);
                return(false);
            }
        }
        QVector<QString> pathToRemove;
        // Paso 1: tile
        QString tilesOutputPath;
        {
            tilesOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_TILES;
            if(tilesOutputPath.contains(" "))
            {
                tilesOutputPath="\""+tilesOutputPath+"\"";
            }
            for(int nf=0;nf<inputFiles.size();nf++)
            {
                QString inputFile=inputFiles.at(nf);
                if(inputFile.contains(" "))
                {
                    inputFile="\""+inputFile+"\"";
                }

                QString commandString=mLastoolsPath+"\\";
                commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASTILE;
                commandString+=" -i ";
                commandString+=inputFile;//firstUnionOutputFileName;

                commandString+=" ";
                parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASTILE_CORES;
                ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
                if(ptrParameter==NULL)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                    return(false);
                }
                QString parameterTag=ptrParameter->getTag();
                commandString+=parameterTag;
                commandString+=" ";
                commandString+=lastileCoresStr;

                commandString+=" ";
                parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASTILE_TILE_BUFFER;
                ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
                if(ptrParameter==NULL)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                    return(false);
                }
                parameterTag=ptrParameter->getTag();
                commandString+=parameterTag;
                commandString+=" ";
                commandString+=lastileBufferStr;

                commandString+=" ";
                parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASTILE_TILE_SIZE;
                ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
                if(ptrParameter==NULL)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                    return(false);
                }
                parameterTag=ptrParameter->getTag();
                commandString+=parameterTag;
                commandString+=" ";
                commandString+=lastileTileSizeStr;

                commandString+=" -flag_as_withheld -v -olaz -odir ";
                commandString+=tilesOutputPath;
                lastoolsCommandStrings.push_back(commandString);
            }
        }
//        pathToRemove.push_back(tilesOutputPath);
        QString firstLasthinOutputPath;
        // Paso 2: first lasthin
        {
            firstLasthinOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_FIRST_LASTHIN;
            if(firstLasthinOutputPath.contains(" "))
            {
                firstLasthinOutputPath="\""+firstLasthinOutputPath+"\"";
            }
            QString firstLasthinInputStr;
            firstLasthinInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_TILES;
            firstLasthinInputStr+="\\*.laz";
            if(firstLasthinInputStr.contains(" "))
            {
                firstLasthinInputStr="\""+firstLasthinInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASTHIN;
            commandString+=" -i ";
            commandString+=firstLasthinInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_FIRST_LASTHIN_STEP;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=firstLasthinStepStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_FIRST_LASTHIN_PERCENTILE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            firstLasthinPercentileStr=firstLasthinPercentileStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=firstLasthinPercentileStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_FIRST_LASTHIN_CLASSIFY_AS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=firstLasthinClassifyAsStr;

            commandString+=" -v -olaz -odir ";
            commandString+=firstLasthinOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        {
            QString commandString="rd /s /q \"";
            commandString+=tilesOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(firstLasthinOutputPath);
        QString lasnoiseOutputPath;
        // Paso 3: lasnoise
        {
            lasnoiseOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LASNOISE;
            if(lasnoiseOutputPath.contains(" "))
            {
                lasnoiseOutputPath="\""+lasnoiseOutputPath+"\"";
            }
            QString lasnoiseInputStr;
            lasnoiseInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_FIRST_LASTHIN;
            lasnoiseInputStr+="\\*.laz";
            if(lasnoiseInputStr.contains(" "))
            {
                lasnoiseInputStr="\""+lasnoiseInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASNOISE;
            commandString+=" -i ";
            commandString+=lasnoiseInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASNOISE_IGNORE_CLASS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasnoiseIgnoreClassStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASNOISE_STEP_XY;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            firstLasthinPercentileStr=firstLasthinPercentileStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasnoiseStepXYStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASNOISE_STEP_Z;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasnoiseStepZStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASNOISE_ISOLATED;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasnoiseIsolatedStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASNOISE_CLASSIFY_AS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasnoiseClassifyAsStr;

            commandString+=" -v -olaz -odir ";
            commandString+=lasnoiseOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(lasnoiseOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=firstLasthinOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
//
        QString firstLasgroundOutputPath;
        // Paso 4: first lasground
        {
            firstLasgroundOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_FIRST_LASGROUND;
            if(firstLasgroundOutputPath.contains(" "))
            {
                firstLasgroundOutputPath="\""+firstLasgroundOutputPath+"\"";
            }
            QString firstLasgroundInputStr;
            firstLasgroundInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LASNOISE;
            firstLasgroundInputStr+="\\*.laz";
            if(firstLasgroundInputStr.contains(" "))
            {
                firstLasgroundInputStr="\""+firstLasgroundInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASGROUND;
            commandString+=" -i ";
            commandString+=firstLasgroundInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_FIRST_LASGROUND_IGNORE_CLASS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            firstLasgroundIgnoreClassStr=firstLasgroundIgnoreClassStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=firstLasgroundIgnoreClassStr;

            commandString+=" -town -ultra_fine -v -olaz -odir ";
            commandString+=firstLasgroundOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(firstLasgroundOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=lasnoiseOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        QString firstLasheightOutputPath;
        // Paso 5: first lasheight
        {
            firstLasheightOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_FIRST_LASHEIGHT;
            if(firstLasheightOutputPath.contains(" "))
            {
                firstLasheightOutputPath="\""+firstLasheightOutputPath+"\"";
            }
            QString firstLasheightInputStr;
            firstLasheightInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_FIRST_LASGROUND;
            firstLasheightInputStr+="\\*.laz";
            if(firstLasheightInputStr.contains(" "))
            {
                firstLasheightInputStr="\""+firstLasheightInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASHEIGHT;
            commandString+=" -i ";
            commandString+=firstLasheightInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_FIRST_LASHEIGHT_CLASSIFY_BELOW;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            firstLasheightClassifyBelowStr=firstLasheightClassifyBelowStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=firstLasheightClassifyBelowStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_FIRST_LASHEIGHT_CLASSIFY_ABOVE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            firstLasheightClassifyAboveStr=firstLasheightClassifyAboveStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=firstLasheightClassifyAboveStr;

            commandString+=" -v -olaz -odir ";
            commandString+=firstLasheightOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(firstLasheightOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=firstLasgroundOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        QString secondLasthinOutputPath;
        // Paso 6: second lasthin
        {
            secondLasthinOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_SECOND_LASTHIN;
            if(secondLasthinOutputPath.contains(" "))
            {
                secondLasthinOutputPath="\""+secondLasthinOutputPath+"\"";
            }
            QString secondLasthinInputStr;
            secondLasthinInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_FIRST_LASHEIGHT;
            secondLasthinInputStr+="\\*.laz";
            if(secondLasthinInputStr.contains(" "))
            {
                secondLasthinInputStr="\""+secondLasthinInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASTHIN;
            commandString+=" -i ";
            commandString+=secondLasthinInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASTHIN_STEP;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasthinStepStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASTHIN_IGNORE_CLASS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            secondLasthinIgnoreClassStr=secondLasthinIgnoreClassStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasthinIgnoreClassStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASTHIN_CLASSIFY_AS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasthinClassifyAsStr;

            commandString+=" -lowest -v -olaz -odir ";
            commandString+=secondLasthinOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(secondLasthinOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=firstLasheightOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        QString secondLasgroundOutputPath;
        // Paso 7: second lasground
        {
            secondLasgroundOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_SECOND_LASGROUND;
            if(secondLasgroundOutputPath.contains(" "))
            {
                secondLasgroundOutputPath="\""+secondLasgroundOutputPath+"\"";
            }
            QString secondLasgroundInputStr;
            secondLasgroundInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_SECOND_LASTHIN;
            secondLasgroundInputStr+="\\*.laz";
            if(secondLasgroundInputStr.contains(" "))
            {
                secondLasgroundInputStr="\""+secondLasgroundInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASGROUND;
            commandString+=" -i ";
            commandString+=secondLasgroundInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASGROUND_IGNORE_CLASS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            secondLasgroundIgnoreClassStr=secondLasgroundIgnoreClassStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasgroundIgnoreClassStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASGROUND_BULGE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasgroundBulgeStr;

            commandString+=" -town -extra_fine -v -olaz -odir ";
            commandString+=secondLasgroundOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(secondLasgroundOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=secondLasthinOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        QString secondLasheightOutputPath;
        // Paso 8: second lasheight
        {
            secondLasheightOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_SECOND_LASHEIGHT;
            if(secondLasheightOutputPath.contains(" "))
            {
                secondLasheightOutputPath="\""+secondLasheightOutputPath+"\"";
            }
            QString secondLasheightInputStr;
            secondLasheightInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_SECOND_LASGROUND;
            secondLasheightInputStr+="\\*.laz";
            if(secondLasheightInputStr.contains(" "))
            {
                secondLasheightInputStr="\""+secondLasheightInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASHEIGHT;
            commandString+=" -i ";
            commandString+=secondLasheightInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASHEIGHT_CLASSIFICATION;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasheightClassificationStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LASHEIGHT_DROP_BELOW;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasheightDropBelowStr;

            commandString+=" -v -olaz -odir ";
            commandString+=secondLasheightOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(secondLasheightOutputPath);
        QString las2lasOutputPath;
        // Paso 9: las2las
        {
            las2lasOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LAS2LAS;
            if(las2lasOutputPath.contains(" "))
            {
                las2lasOutputPath="\""+las2lasOutputPath+"\"";
            }
            QString las2lasInputStr;
            las2lasInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_SECOND_LASGROUND;
            las2lasInputStr+="\\*.laz";
            if(las2lasInputStr.contains(" "))
            {
                las2lasInputStr="\""+las2lasInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LAS2LAS;
            commandString+=" -i ";
            commandString+=las2lasInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_SECOND_LAS2LAS_KEEP_CLASS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=las2lasKeepClassStr;

            commandString+=" -v -olaz -odir ";
            commandString+=las2lasOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(las2lasOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=secondLasgroundOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 10: first lasmerge
        {
            QString firstLasmergeOutuputFilename=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LASMERGE;
            firstLasmergeOutuputFilename+="\\";
            firstLasmergeOutuputFilename+=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASMERGE_GROUND_FILENAME;
            if(firstLasmergeOutuputFilename.contains(" "))
            {
                firstLasmergeOutuputFilename="\""+firstLasmergeOutuputFilename+"\"";
            }
            QString firstLasmergeInputStr;
            firstLasmergeInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LAS2LAS;
            firstLasmergeInputStr+="\\*.laz";
            if(firstLasmergeInputStr.contains(" "))
            {
                firstLasmergeInputStr="\""+firstLasmergeInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASMERGE;
            commandString+=" -i ";
            commandString+=firstLasmergeInputStr;

            commandString+=" -drop_withheld -v -o ";
            commandString+=firstLasmergeOutuputFilename;
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 11: second lasmerge
        QString secondLasmergeOutuputPath;
        {
            secondLasmergeOutuputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LASMERGE;
            QString secondLasmergeOutuputFilename=secondLasmergeOutuputPath+"\\";
            secondLasmergeOutuputFilename+=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASMERGE_OBJECTS_FILENAME;
            if(secondLasmergeOutuputFilename.contains(" "))
            {
                secondLasmergeOutuputFilename="\""+secondLasmergeOutuputFilename+"\"";
            }
            QString secondLasmergeInputStr;
            secondLasmergeInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_SECOND_LASHEIGHT;
            secondLasmergeInputStr+="\\*.laz";
            if(secondLasmergeInputStr.contains(" "))
            {
                secondLasmergeInputStr="\""+secondLasmergeInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASMERGE;
            commandString+=" -i ";
            commandString+=secondLasmergeInputStr;

            commandString+=" -drop_withheld -v -o ";
            commandString+=secondLasmergeOutuputFilename;
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 12: third lasmerge
        {
            QString commandString="rd /s /q \"";
            commandString+=secondLasheightOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        {
            QString commandString="rd /s /q \"";
            commandString+=las2lasOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        QString thirdLasmergeOutuputFilename;
        {
            thirdLasmergeOutuputFilename=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LASMERGE;
            thirdLasmergeOutuputFilename+="\\";
            thirdLasmergeOutuputFilename+=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_LASMERGE_TEMP_FILENAME;
            if(thirdLasmergeOutuputFilename.contains(" "))
            {
                thirdLasmergeOutuputFilename="\""+thirdLasmergeOutuputFilename+"\"";
            }
            QString thirdLasmergeInputStr;
            thirdLasmergeInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LASMERGE;
            thirdLasmergeInputStr+="\\*.laz";
            if(thirdLasmergeInputStr.contains(" "))
            {
                thirdLasmergeInputStr="\""+thirdLasmergeInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASMERGE;
            commandString+=" -i ";
            commandString+=thirdLasmergeInputStr;

            commandString+=" -v -o ";
            commandString+=thirdLasmergeOutuputFilename;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(las2lasOutputPath);
        QString originalOutputFile=outputFile;
        QFileInfo outputFileInfo(outputFile);
        // Paso 13: third lasthin
        {
            if(outputFile.contains(" "))
            {
                outputFile="\""+outputFile+"\"";
            }

            QString thirdLasthinInputStr=thirdLasmergeOutuputFilename;
//            thirdLasthinInputStr=temporalBasePath+"\\"+POINTCLOUDDB_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LASMERGE;
//            thirdLasthinInputStr+="\\*.laz";
//            if(thirdLasthinInputStr.contains(" "))
//            {
//                thirdLasthinInputStr="\""+thirdLasthinInputStr+"\"";
//            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASTHIN;
            commandString+=" -i ";
            commandString+=thirdLasthinInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_THIRD_LASTHING_IGNORE_CLASS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            thirdLasthinIgnoreClassStr=thirdLasthinIgnoreClassStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=thirdLasthinIgnoreClassStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_THIRD_LASTHING_ADAPTATIVE_VERTICAL_TOLERANCE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=thirdLasthinAdaptativeVerticalToleranceStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_THIRD_LASTHING_ADAPTATIVE_MAXIMUM_DISTANCE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
//            QString parameterTag=ptrParameter->getTag();
//            commandString+=" ";
//            commandString+=parameterTag;
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            commandString+=" ";
            commandString+=thirdLasthinAdaptativeMaximunDistanceStr;

            commandString+=" -v -o ";
            commandString+=outputFile;
            lastoolsCommandStrings.push_back(commandString);
        }
        {
            QString commandString="rd /s /q \"";
            commandString+=secondLasmergeOutuputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 14: remove temporal paths
//        for(int np=0;np<pathToRemove.size();np++)
//        {
//            QString commandString="rd /s /q \"";
//            commandString+=pathToRemove[np];
//            commandString+="\"";
//            lastoolsCommandStrings.push_back(commandString);
//        }

    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY,Qt::CaseInsensitive)==0)
    {
        int intValue;
        bool okToInt;
        QString parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_TEMPORAL_PATH;
        Parameter* ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString temporalBasePath;
        ptrParameter->getValue(temporalBasePath);
        QDir auxDir=QDir::currentPath();
        if(!auxDir.exists(temporalBasePath))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nNot exists path: %1").arg(temporalBasePath);
            return(false);
        }
        bool removeOnlyContent=true;
        if(!removeDir(temporalBasePath,removeOnlyContent))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nError removing contents in path: %1").arg(temporalBasePath);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASTILE_CORES;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lastileCoresStr;
        ptrParameter->getValue(lastileCoresStr);
        okToInt=false;
        intValue=lastileCoresStr.toInt(&okToInt);
        if(!okToInt)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(lastileCoresStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASTILE_TILE_SIZE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lastileTileSizeStr;
        ptrParameter->getValue(lastileTileSizeStr);
        double dblValue;
        bool okToDouble=false;
        dblValue=lastileTileSizeStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lastileTileSizeStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASTILE_TILE_BUFFER;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lastileBufferStr;
        ptrParameter->getValue(lastileBufferStr);
        okToDouble=false;
        dblValue=lastileBufferStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lastileBufferStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_FIRST_LASTHIN_STEP;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString firstLasthinStepStr;
        ptrParameter->getValue(firstLasthinStepStr);
        okToDouble=false;
        dblValue=firstLasthinStepStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(firstLasthinStepStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_FIRST_LASTHIN_PERCENTILE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString firstLasthinPercentileStr;
        ptrParameter->getValue(firstLasthinPercentileStr);
        QStringList strPercentileValues=firstLasthinPercentileStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        for(int i=0;i<strPercentileValues.size();i++)
        {
            QString strAuxValue=strPercentileValues.at(i);
            okToDouble=false;
            dblValue=strAuxValue.toDouble(&okToDouble);
            if(!okToDouble)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                strError+=QObject::tr("\nValue: %1 is not a double").arg(strAuxValue);
                return(false);
            }
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_FIRST_LASTHIN_CLASSIFY_AS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString firstLasthinClassifyAsStr;
        ptrParameter->getValue(firstLasthinClassifyAsStr);
        okToInt=false;
        intValue=firstLasthinClassifyAsStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(firstLasthinClassifyAsStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASNOISE_CLASSIFY_AS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasnoiseClassifyAsStr;
        ptrParameter->getValue(lasnoiseClassifyAsStr);
        okToInt=false;
        intValue=lasnoiseClassifyAsStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(lasnoiseClassifyAsStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASNOISE_IGNORE_CLASS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasnoiseIgnoreClassStr;
        ptrParameter->getValue(lasnoiseIgnoreClassStr);
        okToInt=false;
        intValue=lasnoiseIgnoreClassStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(lasnoiseIgnoreClassStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASNOISE_ISOLATED;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasnoiseIsolatedStr;
        ptrParameter->getValue(lasnoiseIsolatedStr);
        okToDouble=false;
        dblValue=lasnoiseIsolatedStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasnoiseIsolatedStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASNOISE_STEP_Z;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasnoiseStepZStr;
        ptrParameter->getValue(lasnoiseStepZStr);
        okToDouble=false;
        dblValue=lasnoiseStepZStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasnoiseStepZStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASNOISE_STEP_XY;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasnoiseStepXYStr;
        ptrParameter->getValue(lasnoiseStepXYStr);
        okToDouble=false;
        dblValue=lasnoiseStepXYStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasnoiseStepXYStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_FIRST_LASGROUND_IGNORE_CLASS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString firstLasgroundIgnoreClassStr;
        ptrParameter->getValue(firstLasgroundIgnoreClassStr);
        QStringList strFirstLasgroundIgnoreClassValues=firstLasgroundIgnoreClassStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        for(int i=0;i<strFirstLasgroundIgnoreClassValues.size();i++)
        {
            QString strAuxValue=strFirstLasgroundIgnoreClassValues.at(i);
            okToInt=false;
            intValue=strAuxValue.toInt(&okToInt);
            if(!okToInt)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                strError+=QObject::tr("\nValue: %1 is not an integer").arg(strAuxValue);
                return(false);
            }
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_FIRST_LASHEIGHT_CLASSIFY_BELOW;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString firstLasheightClassifyBelowStr;
        ptrParameter->getValue(firstLasheightClassifyBelowStr);
        QStringList strFirstLasheightBelowValues=firstLasheightClassifyBelowStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        QString firstLasheightBelowHeightStr,firstLasheightBelowClassStr;
        for(int i=0;i<strFirstLasheightBelowValues.size();i++)
        {
            QString strAuxValue=strFirstLasheightBelowValues.at(i);
            if(i==0)
            {
                okToDouble=false;
                dblValue=strAuxValue.toDouble(&okToDouble);
                if(!okToDouble)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                    strError+=QObject::tr("\nValue: %1 is not a double").arg(strAuxValue);
                    return(false);
                }
                firstLasheightBelowHeightStr=strAuxValue;
            }
            if(i==1)
            {
                okToDouble=false;
                intValue=qRound(strAuxValue.toDouble(&okToDouble));
                if(!okToDouble)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                    strError+=QObject::tr("\nValue: %1 is not a double").arg(strAuxValue);
                    return(false);
                }
                firstLasheightBelowClassStr=QString::number(intValue);
            }
        }
        firstLasheightClassifyBelowStr=firstLasheightBelowHeightStr+" "+firstLasheightBelowClassStr;
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_FIRST_LASHEIGHT_CLASSIFY_ABOVE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString firstLasheightClassifyAboveStr;
        ptrParameter->getValue(firstLasheightClassifyAboveStr);
        QStringList strFirstLasheightAboveValues=firstLasheightClassifyAboveStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        QString firstLasheightAboveHeightStr,firstLasheightAboveClassStr;
        for(int i=0;i<strFirstLasheightAboveValues.size();i++)
        {
            QString strAuxValue=strFirstLasheightAboveValues.at(i);
            if(i==0)
            {
                okToDouble=false;
                dblValue=strAuxValue.toDouble(&okToDouble);
                if(!okToDouble)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                    strError+=QObject::tr("\nValue: %1 is not a double").arg(strAuxValue);
                    return(false);
                }
                firstLasheightAboveHeightStr=strAuxValue;
            }
            if(i==1)
            {
                okToDouble=false;
                intValue=qRound(strAuxValue.toDouble(&okToDouble));
                if(!okToDouble)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                    strError+=QObject::tr("\nValue: %1 is not a double").arg(strAuxValue);
                    return(false);
                }
                firstLasheightAboveClassStr=QString::number(intValue);
            }
        }
        firstLasheightClassifyAboveStr=firstLasheightAboveHeightStr+" "+firstLasheightAboveClassStr;
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASTHIN_STEP;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasthinStepStr;
        ptrParameter->getValue(secondLasthinStepStr);
        okToDouble=false;
        dblValue=firstLasthinStepStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(secondLasthinStepStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASTHIN_IGNORE_CLASS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasthinIgnoreClassStr;
        ptrParameter->getValue(secondLasthinIgnoreClassStr);
        QStringList strSecondLasthinIgnoreClassValues=secondLasthinIgnoreClassStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        for(int i=0;i<strSecondLasthinIgnoreClassValues.size();i++)
        {
            QString strAuxValue=strSecondLasthinIgnoreClassValues.at(i);
            okToInt=false;
            intValue=strAuxValue.toDouble(&okToInt);
            if(!okToDouble)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                strError+=QObject::tr("\nValue: %1 is not an integer").arg(strAuxValue);
                return(false);
            }
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASTHIN_CLASSIFY_AS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasthinClassifyAsStr;
        ptrParameter->getValue(secondLasthinClassifyAsStr);
        okToInt=false;
        intValue=secondLasthinClassifyAsStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(secondLasthinClassifyAsStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASGROUND_IGNORE_CLASS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasgroundIgnoreClassStr;
        ptrParameter->getValue(secondLasgroundIgnoreClassStr);
        QStringList strSecondLasgroundIgnoreClassValues=secondLasgroundIgnoreClassStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        for(int i=0;i<strSecondLasgroundIgnoreClassValues.size();i++)
        {
            QString strAuxValue=strSecondLasgroundIgnoreClassValues.at(i);
            okToInt=false;
            intValue=strAuxValue.toDouble(&okToInt);
            if(!okToDouble)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                strError+=QObject::tr("\nValue: %1 is not an integer").arg(strAuxValue);
                return(false);
            }
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASGROUND_BULGE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasgroundBulgeStr;
        ptrParameter->getValue(secondLasgroundBulgeStr);
        okToDouble=false;
        dblValue=secondLasgroundBulgeStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(secondLasgroundBulgeStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASHEIGHT_CLASSIFICATION;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasheightClassificationStr;
        ptrParameter->getValue(secondLasheightClassificationStr);
        okToInt=false;
        intValue=secondLasheightClassificationStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(secondLasheightClassificationStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASHEIGHT_DROP_BELOW;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString secondLasheightDropBelowStr;
        ptrParameter->getValue(secondLasheightDropBelowStr);
        okToDouble=false;
        dblValue=secondLasheightDropBelowStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(secondLasheightDropBelowStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LAS2LAS_KEEP_CLASS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString las2lasKeepClassStr;
        ptrParameter->getValue(las2lasKeepClassStr);
        okToInt=false;
        intValue=las2lasKeepClassStr.toInt(&okToInt);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(las2lasKeepClassStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_THIRD_LASTHING_IGNORE_CLASS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString thirdLasthinIgnoreClassStr;
        ptrParameter->getValue(thirdLasthinIgnoreClassStr);
        QStringList strThirdLasthinIgnoreClassValues=thirdLasthinIgnoreClassStr.split(ENUM_CHARACTER_SEPARATOR,QString::SkipEmptyParts);
        for(int i=0;i<strThirdLasthinIgnoreClassValues.size();i++)
        {
            QString strAuxValue=strThirdLasthinIgnoreClassValues.at(i);
            okToInt=false;
            intValue=strAuxValue.toDouble(&okToInt);
            if(!okToDouble)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
                strError+=QObject::tr("\nValue: %1 is not an integer").arg(strAuxValue);
                return(false);
            }
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_THIRD_LASTHING_ADAPTATIVE_VERTICAL_TOLERANCE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString thirdLasthinAdaptativeVerticalToleranceStr;
        ptrParameter->getValue(thirdLasthinAdaptativeVerticalToleranceStr);
        okToDouble=false;
        dblValue=thirdLasthinAdaptativeVerticalToleranceStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(secondLasheightDropBelowStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_THIRD_LASTHING_ADAPTATIVE_MAXIMUM_DISTANCE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString thirdLasthinAdaptativeMaximunDistanceStr;
        ptrParameter->getValue(thirdLasthinAdaptativeMaximunDistanceStr);
        okToDouble=false;
        dblValue=thirdLasthinAdaptativeMaximunDistanceStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(secondLasheightDropBelowStr);
            return(false);
        }


        // Comprobar que no hay ficheros con nombres repetidos
        QVector<QString> inputFileBaseNames;
        for(int nf=0;nf<inputFiles.size();nf++)
        {
            QFileInfo fileInfo(inputFiles.at(nf));
            QString completeBaseName=fileInfo.completeBaseName();
            for(int nfa=0;nfa<inputFileBaseNames.size();nfa++)
            {
                if(inputFileBaseNames[nfa].compare(completeBaseName,Qt::CaseInsensitive)==0)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nRepeated file base name: %1")
                            .arg(completeBaseName);
                    return(false);
                }
            }
            inputFileBaseNames.push_back(completeBaseName);
        }
        // Paso 0: Crear las carpetas de salidas
        QVector<QString> auxiliaryPaths;
        QString auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_TILES;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_FIRST_LASTHIN;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_LASNOISE;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_FIRST_LASGROUND;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_FIRST_LASHEIGHT;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_SECOND_LASTHIN;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_SECOND_LASGROUND;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_SECOND_LASHEIGHT;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_LAS2LAS;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_LASMERGE;
        auxiliaryPaths.push_back(auxiliaryPath);
        for(int np=0;np<auxiliaryPaths.size();np++)
        {
            QString auxiliaryPath=auxiliaryPaths.at(np);
            if(!auxDir.mkpath(auxiliaryPath))
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError making path: %1").arg(auxiliaryPath);
                return(false);
            }
        }
        QVector<QString> pathToRemove;
        // Paso 1: tile
        QString tilesOutputPath;
        {
            tilesOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_TILES;
            if(tilesOutputPath.contains(" "))
            {
                tilesOutputPath="\""+tilesOutputPath+"\"";
            }
            for(int nf=0;nf<inputFiles.size();nf++)
            {
                QString inputFile=inputFiles.at(nf);
                if(inputFile.contains(" "))
                {
                    inputFile="\""+inputFile+"\"";
                }

                QString commandString=mLastoolsPath+"\\";
                commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASTILE;
                commandString+=" -i ";
                commandString+=inputFile;//firstUnionOutputFileName;

                commandString+=" ";
                parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASTILE_CORES;
                ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
                if(ptrParameter==NULL)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                    return(false);
                }
                QString parameterTag=ptrParameter->getTag();
                commandString+=parameterTag;
                commandString+=" ";
                commandString+=lastileCoresStr;

                commandString+=" ";
                parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASTILE_TILE_BUFFER;
                ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
                if(ptrParameter==NULL)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                    return(false);
                }
                parameterTag=ptrParameter->getTag();
                commandString+=parameterTag;
                commandString+=" ";
                commandString+=lastileBufferStr;

                commandString+=" ";
                parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASTILE_TILE_SIZE;
                ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
                if(ptrParameter==NULL)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                    return(false);
                }
                parameterTag=ptrParameter->getTag();
                commandString+=parameterTag;
                commandString+=" ";
                commandString+=lastileTileSizeStr;

                commandString+=" -flag_as_withheld -v -olaz -odir ";
                commandString+=tilesOutputPath;
                lastoolsCommandStrings.push_back(commandString);
            }
        }
//        pathToRemove.push_back(tilesOutputPath);
        QString firstLasthinOutputPath;
        // Paso 2: first lasthin
        {
            firstLasthinOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_FIRST_LASTHIN;
            if(firstLasthinOutputPath.contains(" "))
            {
                firstLasthinOutputPath="\""+firstLasthinOutputPath+"\"";
            }
            QString firstLasthinInputStr;
            firstLasthinInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_TILES;
            firstLasthinInputStr+="\\*.laz";
            if(firstLasthinInputStr.contains(" "))
            {
                firstLasthinInputStr="\""+firstLasthinInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASTHIN;
            commandString+=" -i ";
            commandString+=firstLasthinInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_FIRST_LASTHIN_STEP;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=firstLasthinStepStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_FIRST_LASTHIN_PERCENTILE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            firstLasthinPercentileStr=firstLasthinPercentileStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=firstLasthinPercentileStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_FIRST_LASTHIN_CLASSIFY_AS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=firstLasthinClassifyAsStr;

            commandString+=" -v -olaz -odir ";
            commandString+=firstLasthinOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        {
            QString commandString="rd /s /q \"";
            commandString+=tilesOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(firstLasthinOutputPath);
        QString lasnoiseOutputPath;
        // Paso 3: lasnoise
        {
            lasnoiseOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_LASNOISE;
            if(lasnoiseOutputPath.contains(" "))
            {
                lasnoiseOutputPath="\""+lasnoiseOutputPath+"\"";
            }
            QString lasnoiseInputStr;
            lasnoiseInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_FIRST_LASTHIN;
            lasnoiseInputStr+="\\*.laz";
            if(lasnoiseInputStr.contains(" "))
            {
                lasnoiseInputStr="\""+lasnoiseInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASNOISE;
            commandString+=" -i ";
            commandString+=lasnoiseInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASNOISE_IGNORE_CLASS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasnoiseIgnoreClassStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASNOISE_STEP_XY;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            firstLasthinPercentileStr=firstLasthinPercentileStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasnoiseStepXYStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASNOISE_STEP_Z;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasnoiseStepZStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASNOISE_ISOLATED;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasnoiseIsolatedStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASNOISE_CLASSIFY_AS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasnoiseClassifyAsStr;

            commandString+=" -v -olaz -odir ";
            commandString+=lasnoiseOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(lasnoiseOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=firstLasthinOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
//
        QString firstLasgroundOutputPath;
        // Paso 4: first lasground
        {
            firstLasgroundOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_FIRST_LASGROUND;
            if(firstLasgroundOutputPath.contains(" "))
            {
                firstLasgroundOutputPath="\""+firstLasgroundOutputPath+"\"";
            }
            QString firstLasgroundInputStr;
            firstLasgroundInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_LASNOISE;
            firstLasgroundInputStr+="\\*.laz";
            if(firstLasgroundInputStr.contains(" "))
            {
                firstLasgroundInputStr="\""+firstLasgroundInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASGROUND;
            commandString+=" -i ";
            commandString+=firstLasgroundInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_FIRST_LASGROUND_IGNORE_CLASS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            firstLasgroundIgnoreClassStr=firstLasgroundIgnoreClassStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=firstLasgroundIgnoreClassStr;

            commandString+=" -town -ultra_fine -v -olaz -odir ";
            commandString+=firstLasgroundOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(firstLasgroundOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=lasnoiseOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        QString firstLasheightOutputPath;
        // Paso 5: first lasheight
        {
            firstLasheightOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_FIRST_LASHEIGHT;
            if(firstLasheightOutputPath.contains(" "))
            {
                firstLasheightOutputPath="\""+firstLasheightOutputPath+"\"";
            }
            QString firstLasheightInputStr;
            firstLasheightInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_FIRST_LASGROUND;
            firstLasheightInputStr+="\\*.laz";
            if(firstLasheightInputStr.contains(" "))
            {
                firstLasheightInputStr="\""+firstLasheightInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASHEIGHT;
            commandString+=" -i ";
            commandString+=firstLasheightInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_FIRST_LASHEIGHT_CLASSIFY_BELOW;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            firstLasheightClassifyBelowStr=firstLasheightClassifyBelowStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=firstLasheightClassifyBelowStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_FIRST_LASHEIGHT_CLASSIFY_ABOVE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            firstLasheightClassifyAboveStr=firstLasheightClassifyAboveStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=firstLasheightClassifyAboveStr;

            commandString+=" -v -olaz -odir ";
            commandString+=firstLasheightOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(firstLasheightOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=firstLasgroundOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        QString secondLasthinOutputPath;
        // Paso 6: second lasthin
        {
            secondLasthinOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_SECOND_LASTHIN;
            if(secondLasthinOutputPath.contains(" "))
            {
                secondLasthinOutputPath="\""+secondLasthinOutputPath+"\"";
            }
            QString secondLasthinInputStr;
            secondLasthinInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_FIRST_LASHEIGHT;
            secondLasthinInputStr+="\\*.laz";
            if(secondLasthinInputStr.contains(" "))
            {
                secondLasthinInputStr="\""+secondLasthinInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASTHIN;
            commandString+=" -i ";
            commandString+=secondLasthinInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASTHIN_STEP;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasthinStepStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASTHIN_IGNORE_CLASS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            secondLasthinIgnoreClassStr=secondLasthinIgnoreClassStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasthinIgnoreClassStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASTHIN_CLASSIFY_AS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasthinClassifyAsStr;

            commandString+=" -lowest -v -olaz -odir ";
            commandString+=secondLasthinOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(secondLasthinOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=firstLasheightOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        QString secondLasgroundOutputPath;
        // Paso 7: second lasground
        {
            secondLasgroundOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_SECOND_LASGROUND;
            if(secondLasgroundOutputPath.contains(" "))
            {
                secondLasgroundOutputPath="\""+secondLasgroundOutputPath+"\"";
            }
            QString secondLasgroundInputStr;
            secondLasgroundInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_SECOND_LASTHIN;
            secondLasgroundInputStr+="\\*.laz";
            if(secondLasgroundInputStr.contains(" "))
            {
                secondLasgroundInputStr="\""+secondLasgroundInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASGROUND;
            commandString+=" -i ";
            commandString+=secondLasgroundInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASGROUND_IGNORE_CLASS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            secondLasgroundIgnoreClassStr=secondLasgroundIgnoreClassStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasgroundIgnoreClassStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASGROUND_BULGE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasgroundBulgeStr;

            commandString+=" -town -extra_fine -v -olaz -odir ";
            commandString+=secondLasgroundOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(secondLasgroundOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=secondLasthinOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        QString secondLasheightOutputPath;
        // Paso 8: second lasheight
        {
            secondLasheightOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_SECOND_LASHEIGHT;
            if(secondLasheightOutputPath.contains(" "))
            {
                secondLasheightOutputPath="\""+secondLasheightOutputPath+"\"";
            }
            QString secondLasheightInputStr;
            secondLasheightInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_SECOND_LASGROUND;
            secondLasheightInputStr+="\\*.laz";
            if(secondLasheightInputStr.contains(" "))
            {
                secondLasheightInputStr="\""+secondLasheightInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASHEIGHT;
            commandString+=" -i ";
            commandString+=secondLasheightInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASHEIGHT_CLASSIFICATION;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasheightClassificationStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LASHEIGHT_DROP_BELOW;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=secondLasheightDropBelowStr;

            commandString+=" -v -olaz -odir ";
            commandString+=secondLasheightOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(secondLasheightOutputPath);
        QString las2lasOutputPath;
        // Paso 9: las2las
        {
            las2lasOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_LAS2LAS;
            if(las2lasOutputPath.contains(" "))
            {
                las2lasOutputPath="\""+las2lasOutputPath+"\"";
            }
            QString las2lasInputStr;
            las2lasInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_SECOND_LASGROUND;
            las2lasInputStr+="\\*.laz";
            if(las2lasInputStr.contains(" "))
            {
                las2lasInputStr="\""+las2lasInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LAS2LAS;
            commandString+=" -i ";
            commandString+=las2lasInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_SECOND_LAS2LAS_KEEP_CLASS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=las2lasKeepClassStr;

            commandString+=" -v -olaz -odir ";
            commandString+=las2lasOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(las2lasOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=secondLasgroundOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 10: first lasmerge
        {
            QString firstLasmergeOutuputFilename=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_LASMERGE;
            firstLasmergeOutuputFilename+="\\";
            firstLasmergeOutuputFilename+=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASMERGE_GROUND_FILENAME;
            if(firstLasmergeOutuputFilename.contains(" "))
            {
                firstLasmergeOutuputFilename="\""+firstLasmergeOutuputFilename+"\"";
            }
            QString firstLasmergeInputStr;
            firstLasmergeInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_LAS2LAS;
            firstLasmergeInputStr+="\\*.laz";
            if(firstLasmergeInputStr.contains(" "))
            {
                firstLasmergeInputStr="\""+firstLasmergeInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASMERGE;
            commandString+=" -i ";
            commandString+=firstLasmergeInputStr;

            commandString+=" -drop_withheld -v -o ";
            commandString+=firstLasmergeOutuputFilename;
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 11: second lasmerge
        QString secondLasmergeOutuputPath;
        {
            secondLasmergeOutuputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_LASMERGE;
            QString secondLasmergeOutuputFilename=secondLasmergeOutuputPath+"\\";
            secondLasmergeOutuputFilename+=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASMERGE_OBJECTS_FILENAME;
            if(secondLasmergeOutuputFilename.contains(" "))
            {
                secondLasmergeOutuputFilename="\""+secondLasmergeOutuputFilename+"\"";
            }
            QString secondLasmergeInputStr;
            secondLasmergeInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_SECOND_LASHEIGHT;
            secondLasmergeInputStr+="\\*.laz";
            if(secondLasmergeInputStr.contains(" "))
            {
                secondLasmergeInputStr="\""+secondLasmergeInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASMERGE;
            commandString+=" -i ";
            commandString+=secondLasmergeInputStr;

            commandString+=" -drop_withheld -v -o ";
            commandString+=secondLasmergeOutuputFilename;
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 12: third lasmerge
        {
            QString commandString="rd /s /q \"";
            commandString+=secondLasheightOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        {
            QString commandString="rd /s /q \"";
            commandString+=las2lasOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        QString thirdLasmergeOutuputFilename;
        {
            thirdLasmergeOutuputFilename=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_LASMERGE;
            thirdLasmergeOutuputFilename+="\\";
            thirdLasmergeOutuputFilename+=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_LASMERGE_TEMP_FILENAME;
            if(thirdLasmergeOutuputFilename.contains(" "))
            {
                thirdLasmergeOutuputFilename="\""+thirdLasmergeOutuputFilename+"\"";
            }
            QString thirdLasmergeInputStr;
            thirdLasmergeInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_PATH_LASMERGE;
            thirdLasmergeInputStr+="\\*.laz";
            if(thirdLasmergeInputStr.contains(" "))
            {
                thirdLasmergeInputStr="\""+thirdLasmergeInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASMERGE;
            commandString+=" -i ";
            commandString+=thirdLasmergeInputStr;

            commandString+=" -v -o ";
            commandString+=thirdLasmergeOutuputFilename;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(las2lasOutputPath);
        QString originalOutputFile=outputFile;
        QFileInfo outputFileInfo(outputFile);
        // Paso 13: third lasthin
        {
            if(outputFile.contains(" "))
            {
                outputFile="\""+outputFile+"\"";
            }

            QString thirdLasthinInputStr=thirdLasmergeOutuputFilename;
//            thirdLasthinInputStr=temporalBasePath+"\\"+POINTCLOUDDB_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING_PATH_LASMERGE;
//            thirdLasthinInputStr+="\\*.laz";
//            if(thirdLasthinInputStr.contains(" "))
//            {
//                thirdLasthinInputStr="\""+thirdLasthinInputStr+"\"";
//            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASTHIN;
            commandString+=" -i ";
            commandString+=thirdLasthinInputStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_THIRD_LASTHING_IGNORE_CLASS;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            thirdLasthinIgnoreClassStr=thirdLasthinIgnoreClassStr.replace(ENUM_CHARACTER_SEPARATOR," ");
            QString parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=thirdLasthinIgnoreClassStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_THIRD_LASTHING_ADAPTATIVE_VERTICAL_TOLERANCE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=thirdLasthinAdaptativeVerticalToleranceStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY_THIRD_LASTHING_ADAPTATIVE_MAXIMUM_DISTANCE;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
//            QString parameterTag=ptrParameter->getTag();
//            commandString+=" ";
//            commandString+=parameterTag;
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            commandString+=" ";
            commandString+=thirdLasthinAdaptativeMaximunDistanceStr;

            commandString+=" -v -o ";
            commandString+=outputFile;
            lastoolsCommandStrings.push_back(commandString);
        }
        {
            QString commandString="rd /s /q \"";
            commandString+=secondLasmergeOutuputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 14: remove temporal paths
//        for(int np=0;np<pathToRemove.size();np++)
//        {
//            QString commandString="rd /s /q \"";
//            commandString+=pathToRemove[np];
//            commandString+="\"";
//            lastoolsCommandStrings.push_back(commandString);
//        }

    }
    else if(command.compare(POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION,Qt::CaseInsensitive)==0)
    {
        int intValue;
        bool okToInt;
        QString parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_TEMPORAL_PATH;
        Parameter* ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString temporalBasePath;
        ptrParameter->getValue(temporalBasePath);
        QDir auxDir=QDir::currentPath();
        if(!auxDir.exists(temporalBasePath))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nNot exists path: %1").arg(temporalBasePath);
            return(false);
        }
        bool removeOnlyContent=true;
        if(!removeDir(temporalBasePath,removeOnlyContent))
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nError removing contents in path: %1").arg(temporalBasePath);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASTILE_CORES;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lastileCoresStr;
        ptrParameter->getValue(lastileCoresStr);
        okToInt=false;
        intValue=lastileCoresStr.toInt(&okToInt);
        if(!okToInt)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(lastileCoresStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASTILE_TILE_SIZE;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lastileTileSizeStr;
        ptrParameter->getValue(lastileTileSizeStr);
        double dblValue;
        bool okToDouble=false;
        dblValue=lastileTileSizeStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lastileTileSizeStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASTILE_TILE_BUFFER;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lastileBufferStr;
        ptrParameter->getValue(lastileBufferStr);
        okToDouble=false;
        dblValue=lastileBufferStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lastileBufferStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_CORES;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasclassifyCoresStr;
        ptrParameter->getValue(lasclassifyCoresStr);
        okToInt=false;
        intValue=lasclassifyCoresStr.toInt(&okToInt);
        if(!okToInt)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not an integer").arg(lasclassifyCoresStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_STEP;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasclassifyStepStr;
        ptrParameter->getValue(lasclassifyStepStr);
        okToDouble=false;
        dblValue=lasclassifyStepStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasclassifyStepStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_PLANAR;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasclassifyPlanarStr;
        ptrParameter->getValue(lasclassifyPlanarStr);
        okToDouble=false;
        dblValue=lasclassifyPlanarStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasclassifyPlanarStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_RUGED;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasclassifyRugedStr;
        ptrParameter->getValue(lasclassifyRugedStr);
        okToDouble=false;
        dblValue=lasclassifyRugedStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasclassifyRugedStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_GROUNDOFFSET;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        QString lasclassifyGroundOffsetStr;
        ptrParameter->getValue(lasclassifyGroundOffsetStr);
        okToDouble=false;
        dblValue=lasclassifyGroundOffsetStr.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nFor parameter: %1").arg(parameterCode);
            strError+=QObject::tr("\nValue: %1 is not a double").arg(lasclassifyGroundOffsetStr);
            return(false);
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_SMALLBUILDINGS;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        bool smallBuildings=false;
        if(ptrParameter->isEnabled())
        {
            smallBuildings=true;
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_SMALLTREES;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        bool smallTrees=false;
        if(ptrParameter->isEnabled())
        {
            smallTrees=true;
        }
        parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_KEEPOVERHANG;
        ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
            strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                    .arg(command);
            strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
            return(false);
        }
        bool keepOverhang=false;
        if(ptrParameter->isEnabled())
        {
            keepOverhang=true;
        }

        // Comprobar que no hay ficheros con nombres repetidos
        QVector<QString> inputFileBaseNames;
        for(int nf=0;nf<inputFiles.size();nf++)
        {
            QFileInfo fileInfo(inputFiles.at(nf));
            QString completeBaseName=fileInfo.completeBaseName();
            for(int nfa=0;nfa<inputFileBaseNames.size();nfa++)
            {
                if(inputFileBaseNames[nfa].compare(completeBaseName,Qt::CaseInsensitive)==0)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nRepeated file base name: %1")
                            .arg(completeBaseName);
                    return(false);
                }
            }
            inputFileBaseNames.push_back(completeBaseName);
        }
        // Paso 0: Crear las carpetas de salidas
        QVector<QString> auxiliaryPaths;
        QString auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_PATH_LASTILES;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_PATH_LASHEIGHT;
        auxiliaryPaths.push_back(auxiliaryPath);
        auxiliaryPath=temporalBasePath+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_PATH_LASCLASSIFY;
        auxiliaryPaths.push_back(auxiliaryPath);
        for(int np=0;np<auxiliaryPaths.size();np++)
        {
            QString auxiliaryPath=auxiliaryPaths.at(np);
            if(!auxDir.mkpath(auxiliaryPath))
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError making path: %1").arg(auxiliaryPath);
                return(false);
            }
        }
        QVector<QString> pathToRemove;
        // Paso 1: tile
        QString tilesOutputPath;
        {
            tilesOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_PATH_LASTILES;
            if(tilesOutputPath.contains(" "))
            {
                tilesOutputPath="\""+tilesOutputPath+"\"";
            }
            for(int nf=0;nf<inputFiles.size();nf++)
            {
                QString inputFile=inputFiles.at(nf);
                if(inputFile.contains(" "))
                {
                    inputFile="\""+inputFile+"\"";
                }

                QString commandString=mLastoolsPath+"\\";
                commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASTILE;
                commandString+=" -i ";
                commandString+=inputFile;//firstUnionOutputFileName;

                commandString+=" ";
                parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASTILE_CORES;
                ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
                if(ptrParameter==NULL)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                    return(false);
                }
                QString parameterTag=ptrParameter->getTag();
                commandString+=parameterTag;
                commandString+=" ";
                commandString+=lastileCoresStr;

                commandString+=" ";
                parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASTILE_TILE_BUFFER;
                ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
                if(ptrParameter==NULL)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                    return(false);
                }
                parameterTag=ptrParameter->getTag();
                commandString+=parameterTag;
                commandString+=" ";
                commandString+=lastileBufferStr;

                commandString+=" ";
                parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASTILE_TILE_SIZE;
                ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
                if(ptrParameter==NULL)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                    return(false);
                }
                parameterTag=ptrParameter->getTag();
                commandString+=parameterTag;
                commandString+=" ";
                commandString+=lastileTileSizeStr;

                commandString+=" -flag_as_withheld -v -olaz -odir ";
                commandString+=tilesOutputPath;
                lastoolsCommandStrings.push_back(commandString);
            }
        }
//        pathToRemove.push_back(tilesOutputPath);
        QString lasheightOutputPath;
        // Paso 2: lasheight
        {
            lasheightOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_PATH_LASHEIGHT;
            if(lasheightOutputPath.contains(" "))
            {
                lasheightOutputPath="\""+lasheightOutputPath+"\"";
            }
            QString lasheightInputStr;
            lasheightInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_PATH_LASTILES;
            lasheightInputStr+="\\*.laz";
            if(lasheightInputStr.contains(" "))
            {
                lasheightInputStr="\""+lasheightInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASHEIGHT;
            commandString+=" -i ";
            commandString+=lasheightInputStr;
            commandString+=" -v -olaz -odir ";
            commandString+=lasheightOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        {
            QString commandString="rd /s /q \"";
            commandString+=tilesOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(firstLasthinOutputPath);
        QString lasclassifyOutputPath;
        // Paso 3: lasclassify
        {
            lasclassifyOutputPath=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_PATH_LASCLASSIFY;
            if(lasclassifyOutputPath.contains(" "))
            {
                lasclassifyOutputPath="\""+lasclassifyOutputPath+"\"";
            }
            QString lasclassifyInputStr;
            lasclassifyInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_PATH_LASHEIGHT;
            lasclassifyInputStr+="\\*.laz";
            if(lasclassifyInputStr.contains(" "))
            {
                lasclassifyInputStr="\""+lasclassifyInputStr+"\"";
            }

            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASCLASSIFY;
            commandString+=" -i ";
            commandString+=lasclassifyInputStr;
            QString parameterTag;
            if(smallBuildings)
            {
                parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_SMALLBUILDINGS;
                ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
                if(ptrParameter==NULL)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                    return(false);
                }
                parameterTag=ptrParameter->getTag();
                commandString+=" ";
                commandString+=parameterTag;
            }
            if(smallTrees)
            {
                parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_SMALLTREES;
                ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
                if(ptrParameter==NULL)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                    return(false);
                }
                parameterTag=ptrParameter->getTag();
                commandString+=" ";
                commandString+=parameterTag;
            }
            if(keepOverhang)
            {
                parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_KEEPOVERHANG;
                ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
                if(ptrParameter==NULL)
                {
                    strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                    strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                            .arg(command);
                    strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                    return(false);
                }
                parameterTag=ptrParameter->getTag();
                commandString+=" ";
                commandString+=parameterTag;
            }
            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_CORES;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasclassifyCoresStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_GROUNDOFFSET;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasclassifyGroundOffsetStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_STEP;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasclassifyStepStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_PLANAR;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            parameterTag=ptrParameter->getTag();
            commandString+=" ";
            commandString+=parameterTag;
            commandString+=" ";
            commandString+=lasclassifyPlanarStr;

            parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_LASCLASSIFY_RUGED;
            ptrParameter=mPtrLastoolsCommandsParameters->getParameter(parameterCode);
            if(ptrParameter==NULL)
            {
                strError=QObject::tr("PointCloudFileManager::getLastoolsCommandStrings");
                strError+=QObject::tr("\nError getting parameters for lastools command: %1")
                        .arg(command);
                strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
                return(false);
            }
            if(ptrParameter->isEnabled())
            {
                parameterTag=ptrParameter->getTag();
                commandString+=" ";
                commandString+=parameterTag;
                commandString+=" ";
                commandString+=lasclassifyRugedStr;
            }

            commandString+=" -v -olaz -odir ";
            commandString+=lasclassifyOutputPath;
            lastoolsCommandStrings.push_back(commandString);
        }
        //        pathToRemove.push_back(lasnoiseOutputPath);
        {
            QString commandString="rd /s /q \"";
            commandString+=lasheightOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
        // Paso 4: lasmerge
        QString originalOutputFile=outputFile;
        {
            QFileInfo outputFileInfo(outputFile);
            {
                if(outputFile.contains(" "))
                {
                    outputFile="\""+outputFile+"\"";
                }
            }
            QString lasmergeInputStr;
            lasmergeInputStr=temporalBasePath+"\\"+POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION_PATH_LASCLASSIFY;
            lasmergeInputStr+="\\*.laz";
            if(lasmergeInputStr.contains(" "))
            {
                lasmergeInputStr="\""+lasmergeInputStr+"\"";
            }
            QString commandString=mLastoolsPath+"\\";
            commandString+=POINTCLOUDFILE_LASTOOLS_COMMAND_LASMERGE;
            commandString+=" -i ";
            commandString+=lasmergeInputStr;

            commandString+=" -drop_withheld -v -o ";
            commandString+=outputFile;
            lastoolsCommandStrings.push_back(commandString);
        }
        {
            QString commandString="rd /s /q \"";
            commandString+=lasclassifyOutputPath;
            commandString+="\"";
            lastoolsCommandStrings.push_back(commandString);
        }
    }
    return(true);
}

bool PointCloudFileManager::getMaximumDensity(QString pcfPath,
                                              double &maximumDensity,
                                              QString &strError)
{
    QString strAuxError;
    if(!mPtrPcFiles.contains(pcfPath))
    {
        if(!openPointCloudFile(pcfPath,
                               strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::getMaximumDensity");
            strError+=QObject::tr("\nError openning spatialite:\n%1\nError:\n%2")
                    .arg(pcfPath).arg(strAuxError);
            return(false);
        }
    }
    maximumDensity=mPtrPcFiles[pcfPath]->getMaximumDensity();
    return(true);
}

bool PointCloudFileManager::getNeighbors(QString pcfPath,
                                         QVector<double> point,
                                         int pointCrsEpsgCode,
                                         QString pointCrsProj4String,
                                         double searchRadius2d,
                                         int numberOfNeighbors,
                                         QVector<Point> &points,
                                         QVector<double> &distances,
                                         QString &strError)
{
    QString strAuxError;
    if(!mPtrPcFiles.contains(pcfPath))
    {
        if(!openPointCloudFile(pcfPath,
                               strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::getNeighbors");
            strError+=QObject::tr("\nError openning spatialite:\n%1\nError:\n%2")
                    .arg(pcfPath).arg(strAuxError);
            return(false);
        }
    }
    return(mPtrPcFiles[pcfPath]->getNeighbors(point,
                                              pointCrsEpsgCode,
                                              pointCrsProj4String,
                                              searchRadius2d,
                                              numberOfNeighbors,
                                              points,
                                              distances,
                                              strError));
    return(true);
}

bool PointCloudFileManager::getPointsFromWktGeometry(QString pcfPath,
                                                     QString wktGeometry,
                                                     int geometryCrsEpsgCode,
                                                     QString geometryCrsProj4String,
                                                     QMap<int, QMap<int, QString> > &tilesTableName,
                                                     QMap<int, QMap<int, QMap<int, QVector<Point> > > > &pointsByTileByFileId,
                                                     QMap<int, QMap<QString, bool> > &existsFieldsByFileId,
                                                     QVector<QString> &ignoreTilesTableName,
                                                     bool tilesFullGeometry,
                                                     QString &strError)
{
    QString strAuxError;
    if(!mPtrPcFiles.contains(pcfPath))
    {
        if(!openPointCloudFile(pcfPath,
                               strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::getPointsFromWktGeometry");
            strError+=QObject::tr("\nError openning spatialite:\n%1\nError:\n%2")
                    .arg(pcfPath).arg(strAuxError);
            return(false);
        }
    }
    return(mPtrPcFiles[pcfPath]->getPointsFromWktGeometry(wktGeometry,
                                                          geometryCrsEpsgCode,
                                                          geometryCrsProj4String,
                                                          tilesTableName,
                                                          pointsByTileByFileId,
                                                          existsFieldsByFileId,
                                                          ignoreTilesTableName,
                                                          tilesFullGeometry,
                                                          strError));
    return(true);
}

bool PointCloudFileManager::getProjectTypes(QVector<QString> &projectTypes,
                                            QString &strError)
{
    projectTypes=mProjectTypes;
    return(true);
}

bool PointCloudFileManager::getReachedMaximumNumberOfPoints(QString pcfPath,
                                                            bool &reachedMaximumNumberOfPoints,
                                                            QString &strError)
{
    reachedMaximumNumberOfPoints=false;
    if(mPtrPcFiles.contains(pcfPath))
    {
        QString strAuxError;
        if(!mPtrPcFiles[pcfPath]->getReachedMaximumNumberOfPoints(reachedMaximumNumberOfPoints,
                                                                  strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::getReachedMaximumNumberOfPoints");
            strError+=QObject::tr("\nError recovering reached maximun number of points for project file:\n%1\nError:\n%2")
                    .arg(pcfPath).arg(strAuxError);
            return(false);
        }
    }
    return(true);
}

bool PointCloudFileManager::getROIsWktGeometry(QString pcfPath,
                                               QMap<QString, QString> &values,
                                               QString &strError)
{
    QString strAuxError;
    if(!mPtrPcFiles.contains(pcfPath))
    {
        if(!openPointCloudFile(pcfPath,
                               strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::getROIsWktGeometry");
            strError+=QObject::tr("\nError openning spatialite:\n%1\nError:\n%2")
                    .arg(pcfPath).arg(strAuxError);
            return(false);
        }
    }
    return(mPtrPcFiles[pcfPath]->getROIsWktGeometry(values,strError));
}

bool PointCloudFileManager::getTilesNamesFromWktGeometry(QString pcfPath,
                                                         QString wktGeometry,
                                                         int geometryCrsEpsgCode,
                                                         QString geometryCrsProj4String,
                                                         QMap<int, QMap<int, QString> > &tilesTableName,
                                                         QString &strError)
{
    QString strAuxError;
    if(!mPtrPcFiles.contains(pcfPath))
    {
        if(!openPointCloudFile(pcfPath,
                               strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::getROIsWktGeometry");
            strError+=QObject::tr("\nError openning spatialite:\n%1\nError:\n%2")
                    .arg(pcfPath).arg(strAuxError);
            return(false);
        }
    }
    return(mPtrPcFiles[pcfPath]->getTilesNamesFromWktGeometry(wktGeometry,geometryCrsEpsgCode,
                                                                   geometryCrsProj4String,
                                                                   tilesTableName,strError));
}

bool PointCloudFileManager::getTilesWktGeometry(QString pcfPath,
                                                QMap<QString, QString> &values,
                                                QString &strError)
{
    QString strAuxError;
    if(!mPtrPcFiles.contains(pcfPath))
    {
        if(!openPointCloudFile(pcfPath,
                               strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::getTilesWktGeometry");
            strError+=QObject::tr("\nError openning spatialite:\n%1\nError:\n%2")
                    .arg(pcfPath).arg(strAuxError);
            return(false);
        }
    }
    return(mPtrPcFiles[pcfPath]->getTilesWktGeometry(values,strError));
}

bool PointCloudFileManager::getVerticalCRSs(int epsgCode,
                                            QVector<int> &verticalCRSs,
                                            QString &strError)
{
    verticalCRSs.clear();
    strError.clear();
    mPtrCrsTools->getVerticalCrsEpsgCodesFromCrsEpsgCode(epsgCode,
                                                         verticalCRSs);
    return(true);
}

bool PointCloudFileManager::initializeCrsTools(QString& strError)
{
    if(mPtrCrsTools!=NULL)
    {
        return(true);
    }
    QProcessEnvironment enVar=QProcessEnvironment::systemEnvironment();
    QString projPath=enVar.value(CRSTOOLS_ENVIRONMENT_VARIABLE_PROJ_LIB);
    if(projPath.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::initializeCrsTools");
        strError+=QObject::tr("\nIt has not defined the environment variable: %1").arg(CRSTOOLS_ENVIRONMENT_VARIABLE_PROJ_LIB);
        return(false);
    }
    QString gdalDataPath=enVar.value(CRSTOOLS_ENVIRONMENT_VARIABLE_GDAL_DATA);
    if(gdalDataPath.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::initializeCrsTools");
        strError+=QObject::tr("\nIt has not defined the environment variable: %1").arg(CRSTOOLS_ENVIRONMENT_VARIABLE_GDAL_DATA);
        return(false);
    }
    mPtrCrsTools=libCRS::CRSTools::getInstance();
//    QString trsFilePath=mBasePath;
    QString strAuxError;
    QSettings* ptrSettings=NULL;
    if(!mPtrCrsTools->initialize(ptrSettings,strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::initializeCrsTools");
        strError+=QObject::tr("\nError opening CrsTools:\n%1").arg(strAuxError);
        return(false);
    }
    return(true);
}

bool PointCloudFileManager::openPointCloudFile(QString pcPath,
                                               QString &strError)
{
    if(!mPtrPcFiles.contains(pcPath))
    {
        QString strAuxError;
        PointCloudFile* ptrPcf=new PointCloudFile(mPtrCrsTools,
                                                  this);
        if(!ptrPcf->setFromPath(pcPath,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::openPointCloudSpatialiteDb");
            strError+=QObject::tr("\nError setting point cloud from file:\n%1\nError:\n%2")
                    .arg(pcPath).arg(strAuxError);
            delete(ptrPcf);
            return(false);
        }
//        else
//        {
//            delete(ptrPcDb);
//        }
        if(!mTempPath.isEmpty())
        {
            if(!ptrPcf->setTempPath(mTempPath,strAuxError))
            {
                strError=QObject::tr("PointCloudFileManager::openPointCloudSpatialiteDb");
                strError+=QObject::tr("\nError setting temporal path to point cloud file:\n: %1")
                        .arg(strAuxError);
                delete(ptrPcf);
                return(false);
            }
        }
        if(!mOutputPath.isEmpty())
        {
            if(!ptrPcf->setOutputPath(mOutputPath,strAuxError))
            {
                strError=QObject::tr("PointCloudFileManager::openPointCloudSpatialiteDb");
                strError+=QObject::tr("\nError setting output path to point cloud file:\n: %1")
                        .arg(strAuxError);
                delete(ptrPcf);
                return(false);
            }
        }
        mPtrPcFiles[pcPath]=ptrPcf;
    }
    return(true);
}

bool PointCloudFileManager::processInternalCommand(QString &command,
                                                   QVector<QString> &inputFiles,
                                                   QString &outputPath,
                                                   QString &outputFile,
                                                   QString &suffix,
                                                   QString &prefix,
                                                   QString &strError)
{
    if(!mInternalCommands.contains(command))
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommand");
        strError+=QObject::tr("\nInvalid Internal command: %1").arg(command);
        return(false);
    }
    if(command.compare(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_ESTIMATE,Qt::CaseInsensitive)==0)
    {
        return(processInternalCommandVegetationGrowthEstimate(command,
                                                              inputFiles,
                                                              outputPath,
                                                              outputFile,
                                                              suffix,
                                                              prefix,
                                                              strError));
    }
    return(true);
}

bool PointCloudFileManager::processInternalCommandVegetationGrowthEstimate(QString &command,
                                                                           QVector<QString> &inputFiles,
                                                                           QString &outputPath,
                                                                           QString &outputFileName,
                                                                           QString &suffix,
                                                                           QString &prefix,
                                                                           QString &strError)
{
    outputFileName=outputFileName.trimmed();
    outputPath=outputPath.trimmed();
    QWidget* ptrWidget=new QWidget();
    QString strAuxError;
    if(mInternalCommandsParametersFileName.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nInternal commands parameters file name is empty");
        return(false);
    }
    if(!QFile::exists(mInternalCommandsParametersFileName))
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nNot exists Internal commands parameters file name:\n%1")
                .arg(mLastoolsCommandsParametersFileName);
        return(false);
    }
    if(mPtrInternalCommandsParameters==NULL)
    {
        ParametersManager* ptrParametersManager=new ParametersManager();
        if(!ptrParametersManager->loadFromXml(mInternalCommandsParametersFileName,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
            strError+=QObject::tr("\nError loading parameters manager from file:\n%1\nError:\n%2")
                    .arg(mInternalCommandsParametersFileName).arg(strAuxError);
            delete(ptrParametersManager);
            return(false);
        }
        mPtrInternalCommandsParameters=ptrParametersManager;
    }
    QString strValue,parameterCode,reportFileName;
    if(outputPath.isEmpty()&&outputFileName.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nOutput path and output file are empty");
        return(false);
    }
    if(outputPath.isEmpty())
    {
        QFileInfo outputPathInfo(outputFileName);
        outputPath=outputPathInfo.absolutePath();
    }
    reportFileName=outputPath+"/";
    reportFileName+=POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_REPORT_FILE_BASENAME;
    reportFileName+=POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_REPORT_FILE_EXTENSION;
    QString modelFileName=outputFileName;
    if(modelFileName.isEmpty())
    {
        modelFileName=outputPath+"/";
        modelFileName+=POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_BASENAME;
        modelFileName+=POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_EXTENSION;
    }
    if(QFile::exists(reportFileName))
    {
        if(!QFile::remove(reportFileName))
        {
            strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
            strError+=QObject::tr("\nError removing report file:\n%1")
                    .arg(reportFileName);
            return(false);
        }
    }
    bool okToInt=false;
    bool okToDouble=false;
    Parameter* ptrParameter=NULL;
    parameterCode=POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_USE_MULTI_PROCESS;
    ptrParameter=mPtrInternalCommandsParameters->getParameter(parameterCode);
    if(ptrParameter==NULL)
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nParameter: %1 not found in file:\n%2")
                .arg(parameterCode).arg(mPtrInternalCommandsParameters->getFileName());
        return(false);
    }
    ptrParameter->getValue(strValue);
    bool useMultiProcess=true;
    if(strValue.compare("true",Qt::CaseInsensitive)!=0
            &&strValue.compare("false",Qt::CaseInsensitive)!=0)
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nParameter: %1 is not true or false, is:").arg(parameterCode).arg(strValue);
        return(false);
    }
    if(strValue.compare("false",Qt::CaseInsensitive)==0)
    {
        useMultiProcess=false;
    }
    parameterCode=POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_ESTIMATE_EPSG_CODE;
    ptrParameter=mPtrInternalCommandsParameters->getParameter(parameterCode);
    if(ptrParameter==NULL)
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nError getting parameters for internal command: %1")
                .arg(command);
        strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
        return(false);
    }
    ptrParameter->getValue(strValue);
    int pcfsEpsgCode=strValue.trimmed().toInt(&okToInt);
    if(!okToInt)
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nError getting parameters for internal command: %1")
                .arg(command);
        strError+=QObject::tr("\nParameter: %1 is not an integer").arg(parameterCode);
        return(false);
    }
    parameterCode=POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_ESTIMATE_SPATIAL_RESOLUTION;
    ptrParameter=mPtrInternalCommandsParameters->getParameter(parameterCode);
    if(ptrParameter==NULL)
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nError getting parameters for internal command: %1")
                .arg(command);
        strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
        return(false);
    }
    ptrParameter->getValue(strValue);
    mPICVGESpatialResolution=strValue.trimmed().toInt(&okToInt);
    if(!okToInt)
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nError getting parameters for internal command: %1")
                .arg(command);
        strError+=QObject::tr("\nParameter: %1 is not an integer").arg(parameterCode);
        return(false);
    }
    parameterCode=POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_UPDATE_EXISTING_MODEL;
    ptrParameter=mPtrInternalCommandsParameters->getParameter(parameterCode);
    if(ptrParameter==NULL)
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nError getting parameters for internal command: %1")
                .arg(command);
        strError+=QObject::tr("\nNot exists parameter: %1").arg(parameterCode);
        return(false);
    }
    ptrParameter->getValue(strValue);
    bool updateExistingModel=false;
    strValue=strValue.trimmed();
    if(strValue.compare("true",Qt::CaseInsensitive)==0)
    {
        updateExistingModel=true;
    }
    QMap<int,QVector<double> > vegetationGrowthModel;
    if(QFile::exists(modelFileName))
    {
        if(!updateExistingModel)
        {
            if(!QFile::remove(modelFileName))
            {
                strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
                strError+=QObject::tr("\nError removing model file:\n%1")
                        .arg(modelFileName);
                return(false);
            }
        }
        else
        {
            if(!readVegetationGrowthModel(modelFileName,
                                          vegetationGrowthModel,
                                          strAuxError))
            {
                strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
                strError+=QObject::tr("\nError reading vegetation growth model from file:\n%1")
                        .arg(modelFileName);
                strError+=QObject::tr("\nError:\n%1")
                        .arg(strAuxError);
                return(false);
            }
        }
    }

    QMap<int,QVector<quint16> > vegetationGrowthModelValues;
    QVector<int> stretchs;
    stretchs.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_1_STRETCH_UPPER);
    stretchs.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_2_STRETCH_UPPER);
    stretchs.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_3_STRETCH_UPPER);
    stretchs.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_4_STRETCH_UPPER);
    stretchs.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_5_STRETCH_UPPER);
    stretchs.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_6_STRETCH_UPPER);
    if(vegetationGrowthModel.isEmpty())
    {
        for(int ns=0;ns<stretchs.size();ns++)
        {
            QVector<double> aux(5); // numberOfValues,mean,std,percentil05,percentil95
            vegetationGrowthModel[stretchs[ns]]=aux;
            QVector<quint16> aux16;
            vegetationGrowthModelValues[stretchs[ns]]=aux16;
        }
    }
    QString pcfsCrsDescription;
    if(!mPtrCrsTools->appendUserCrs(pcfsEpsgCode,
                                    pcfsCrsDescription,
                                    strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nInvalid CRS From EPSG code: %1").arg(QString::number(pcfsEpsgCode));
        return(false);
    }
    QFile reportFile(reportFileName);
    if (!reportFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nError opening report file:\n%1")
                .arg(reportFileName);
        return(false);
    }
    QTextStream outReport(&reportFile);
    outReport<<"VEGETATION GROWTH ESTIMATION FROM POINT CLOUDS REPORT"<<"\n";
    outReport<<"- Number of input files ................: "<<QString::number(inputFiles.size())<<"\n";
    QMap<int,QVector<int> > filePosByYear;
    for(int nf=0;nf<inputFiles.size();nf++)
    {
        QString inputFileName=inputFiles.at(nf);
        QFileInfo inputFileInfo(inputFileName);
        QString inputBaseName=inputFileInfo.baseName();
        QStringList strCandidates=inputBaseName.split("_");
        bool existsYear=false;
        for(int nsc=0;nsc<strCandidates.size();nsc++)
        {
            QString strCandidate=strCandidates.at(nsc).trimmed();
            if(strCandidate.size()==4)
            {
                okToInt=false;
                int year=strCandidate.toInt(&okToInt);
                if(okToInt)
                {
                    if(year>=POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MIN_YEAR)
                    {
                        if(existsYear)
                        {
                            strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
                            strError+=QObject::tr("\nFor file:\n%1").arg(inputFileName);
                            strError+=QObject::tr("\nthere are more than one string candidate for year");
                            reportFile.close();
                            return(false);
                        }
                        if(!filePosByYear.contains(year))
                        {
                            QVector<int> aux;
                            filePosByYear[year]=aux;
                        }
                        filePosByYear[year].push_back(nf);
                        existsYear=true;
                    }
                }
            }
        }
        if(!existsYear)
        {
            strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
            strError+=QObject::tr("\nFor file:\n%1").arg(inputFileName);
            strError+=QObject::tr("\nthere are one string candidate for year");
            reportFile.close();
            return(false);
        }
    }
    QVector<int> years;
    QMap<int,QVector<int> >::const_iterator iterFilePosByYear=filePosByYear.begin();
    outReport<<"- Files by year: "<<"\n";
    while(iterFilePosByYear!=filePosByYear.end())
    {
        int year=iterFilePosByYear.key();
        years.push_back(year);
        outReport<<"  - Year ...............................: "<<QString::number(year)<<"\n";
        for(int nf=0;nf<iterFilePosByYear.value().size();nf++)
        {
            QString fileName=inputFiles[iterFilePosByYear.value()[nf]];
            outReport<<"    - File .............................: "<<fileName<<"\n";
        }
        iterFilePosByYear++;
    }
    double maxCoordinatesLenght=(pow(2,16)-1.)*mPICVGESpatialResolution;
    int numberOfProcesses=inputFiles.size();
    mPICVGEBoundingBoxesByFilePos.clear();
    mPICVGEMaxHeightsByTileXYByFilePos.clear();
    if(!useMultiProcess)
    {
        QProgressDialog* ptrProgress=NULL;
        if(ptrWidget!=NULL)
        {
            QString title=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
            QString msgGlobal=QObject::tr("Reading point cloud files");
            msgGlobal+=QObject::tr("\nNumber of files to read: %1").arg(QString::number(numberOfProcesses));
            ptrProgress=new QProgressDialog(title, "Abort",0,numberOfProcesses, ptrWidget);
            ptrProgress->setWindowModality(Qt::WindowModal);
            ptrProgress->setLabelText(msgGlobal);
            ptrProgress->show();
            qApp->processEvents();
        }
        for(int nf=0;nf<inputFiles.size();nf++)
        {
            QString inputFileName=inputFiles.at(nf);
            if(ptrWidget!=NULL)
            {
                ptrProgress->setValue(nf+1);
                qApp->processEvents();
            }
            std::string stdFileName=inputFileName.toStdString();
            const char* charFileName=stdFileName.c_str();
            LASreadOpener lasreadopener;
            lasreadopener.set_file_name(charFileName);
            if (!lasreadopener.active())
            {
                strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
                strError+=QObject::tr("\nError opening file:\n%1").arg(inputFileName);
                reportFile.close();
                return(false);
            }

            LASreader* lasreader = lasreadopener.open();
            LASheader* lasheader = &lasreader->header;
            int numberOfPoints=lasreader->npoints;
            double fileMinX=lasheader->min_x;
            double fileMinY=lasheader->min_y;
            double fileMaxX=lasheader->max_x;
            double fileMaxY=lasheader->max_y;
            QVector<double> boundingBox(4);
            boundingBox[0]=fileMinX;
            boundingBox[1]=fileMinY;
            boundingBox[2]=fileMaxX;
            boundingBox[3]=fileMaxY;
            mPICVGEBoundingBoxesByFilePos[nf]=boundingBox;
            QProgressDialog* ptrFileProgress=NULL;
            int pointsStep=POINTCLOUDFILE_NUMBER_OF_POINTS_TO_PROCESS_BY_STEP;
            int numberOfSteps=ceil((double)numberOfPoints/(double)pointsStep);
            if(ptrWidget!=NULL)
            {
                QString title=QObject::tr("Adding Point Cloud File:");
                QString msgGlobal=inputFileName;
                msgGlobal+="\n";
                msgGlobal+=QString::number(numberOfPoints,10);
                msgGlobal+=" points";
                ptrFileProgress=new QProgressDialog(title, "Abort",0,numberOfSteps, ptrWidget);
                ptrFileProgress->setWindowModality(Qt::WindowModal);
                ptrFileProgress->setLabelText(msgGlobal);
                ptrFileProgress->show();
                qApp->processEvents();
            }
            int pointPosition=-1;
            int step=0;
            int numberOfProcessedPoints=0;
            int numberOfProcessedPointsInStep=0;
            int numberOfPointsToProcessInStep=numberOfPoints;
            if(POINTCLOUDFILE_NUMBER_OF_POINTS_TO_PROCESS_BY_STEP<numberOfPointsToProcessInStep)
            {
                numberOfPointsToProcessInStep=POINTCLOUDFILE_NUMBER_OF_POINTS_TO_PROCESS_BY_STEP;
            }
            while(lasreader->read_point()&&numberOfPointsToProcessInStep>0)
            {
                U8 pointClass=lasreader->point.get_classification();
                if(pointClass==4||pointClass==5)
                {
                    double x=lasreader->point.get_X()*lasheader->x_scale_factor+lasheader->x_offset;
                    double y=lasreader->point.get_Y()*lasheader->y_scale_factor+lasheader->y_offset;
                    double z=lasreader->point.get_Z()*lasheader->z_scale_factor+lasheader->z_offset;
                    quint16 posX=floor((x-fileMinX)/mPICVGESpatialResolution);
                    quint16 posY=floor((y-fileMinY)/mPICVGESpatialResolution);
                    quint16 height=qRound(z*1000.);
                    if(posX>maxCoordinatesLenght
                            ||posY>maxCoordinatesLenght)
                    {
                        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
                        strError+=QObject::tr("\nIn file:\n%1").arg(inputFileName);
                        strError+=QObject::tr("\ncoordinates increments out of 16 bits domain for point: (%1,%2)")
                                .arg(QString::number(x,'f',3)).arg(QString::number(y,'f',3));
                        lasreader->close();
                        delete lasreader;
                        reportFile.close();
                        return(false);
                    }
                    if(!mPICVGEMaxHeightsByTileXYByFilePos[nf].contains(posX))
                    {
                        mPICVGEMaxHeightsByTileXYByFilePos[nf][posX][posY]=height;
                    }
                    else if(!mPICVGEMaxHeightsByTileXYByFilePos[nf][posX].contains(posY))
                    {
                        mPICVGEMaxHeightsByTileXYByFilePos[nf][posX][posY]=height;
                    }
                    else
                    {
                        if(height>mPICVGEMaxHeightsByTileXYByFilePos[nf][posX][posY])
                        {
                            mPICVGEMaxHeightsByTileXYByFilePos[nf][posX][posY]=height;
                        }
                    }
                }
                numberOfProcessedPoints++;
                numberOfProcessedPointsInStep++;
                if(numberOfProcessedPointsInStep==numberOfPointsToProcessInStep)
                {
                    step++;
                    numberOfPointsToProcessInStep=numberOfPoints-numberOfProcessedPoints;
                    if(POINTCLOUDFILE_NUMBER_OF_POINTS_TO_PROCESS_BY_STEP<numberOfPointsToProcessInStep)
                    {
                        numberOfPointsToProcessInStep=POINTCLOUDFILE_NUMBER_OF_POINTS_TO_PROCESS_BY_STEP;
                    }
                    numberOfProcessedPointsInStep=0;
                    if(ptrWidget!=NULL)
                    {
                        ptrFileProgress->setValue(step);
                        qApp->processEvents();
                    }
                }
            }
            if(ptrWidget!=NULL)
            {
                ptrFileProgress->setValue(numberOfProcesses);
                qApp->processEvents();
                ptrFileProgress->close();
                delete(ptrFileProgress);
            }
            lasreader->close();
            delete lasreader;
        }
        if(ptrWidget!=NULL)
        {
            ptrProgress->setValue(numberOfProcesses);
            qApp->processEvents();
            ptrProgress->close();
            delete(ptrProgress);
        }
    }
    else
    {
        mInputFiles=inputFiles;
        if(mPtrMpProgressDialog!=NULL)
        {
            delete(mPtrMpProgressDialog);
        }
        if(ptrWidget!=NULL)
            mPtrMpProgressDialog=new QProgressDialog(ptrWidget);
        else
            mPtrMpProgressDialog=new QProgressDialog();
        //        mNumberOfSqlsInTransaction=0;
        mNumberOfFilesToProcess=mInputFiles.size();
        QString dialogText=QObject::tr("Reading point cloud files");
        dialogText+=QObject::tr("\nNumber of point cloud files to read:%1").arg(mNumberOfFilesToProcess);
        dialogText+=QObject::tr("\n... progressing using %1 threads").arg(QThread::idealThreadCount());
        //                mPtrMpProgressDialog->setWindowTitle(title);
        QVector<int> posFiles;
        mNumberOfPointsToProcessByFileName.clear();
        for(int nf=0;nf<mNumberOfFilesToProcess;nf++)
        {
            QString inputFileName=mInputFiles[nf];
//            dialogText+=QObject::tr("\nPoints to process %1 in file: %2")
//                    .arg("All").arg(inputFileName);
            mNumberOfPointsToProcessByFileName[inputFileName]=-1;
            posFiles.push_back(nf);
        }
        mPtrMpProgressDialog->setLabelText(dialogText);
        mPtrMpProgressDialog->setModal(true);
        QFutureWatcher<void> futureWatcher;
        QObject::connect(&futureWatcher, SIGNAL(finished()), mPtrMpProgressDialog, SLOT(reset()));
        QObject::connect(mPtrMpProgressDialog, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));
        QObject::connect(&futureWatcher, SIGNAL(progressRangeChanged(int,int)), mPtrMpProgressDialog, SLOT(setRange(int,int)));
        QObject::connect(&futureWatcher, SIGNAL(progressValueChanged(int)), mPtrMpProgressDialog, SLOT(setValue(int)));
        //                futureWatcher.setFuture(QtConcurrent::map(fieldsValuesToRetrieve, mpLoadPhotovoltaicPanelsFromDb));
        futureWatcher.setFuture(QtConcurrent::map(posFiles,
                                                  [this](int& data)
        {mpProcessInternalCommandVegetationGrowthEstimate(data);}));
        // Display the dialog and start the event loop.
        mStrErrorMpProgressDialog="";
        mPtrMpProgressDialog->exec();
        futureWatcher.waitForFinished();
        delete(mPtrMpProgressDialog);
        mPtrMpProgressDialog=NULL;
        if(!mStrErrorMpProgressDialog.isEmpty())
        {
            strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
            strError+=QObject::tr("\nError reading point cloud files");
            strError+=QObject::tr("\nError:\n%1").arg(mStrErrorMpProgressDialog);
            reportFile.close();
            return(false);
        }
    }

//    QVector<double> aux(3); // mean,std,numberOfValues
//    vegetationGrowthModel[strectchs[ns]]=aux;
//    QMap<int,QVector<double> > boundingBoxesByFilePos;
//    QMap<int,QMap<quint16,QMap<quint16,quint16> > > maxHeightsByTileXYByFilePos;
//    QVector<QString> inputFiles;
    for(int fny=0;fny<(years.size()-1);fny++)
    {
        int firstYear=years[fny];
        QVector<int> firstYearFilesPos=filePosByYear[firstYear];
        for(int sny=fny+1;sny<years.size();sny++)
        {
            int secondYear=years[sny];
            QVector<int> secondYearFilesPos=filePosByYear[secondYear];
            for(int nfyf=0;nfyf<firstYearFilesPos.size();nfyf++)
            {
                int firstFilePos=firstYearFilesPos[nfyf];
                double fMinX=mPICVGEBoundingBoxesByFilePos[firstFilePos][0];
                double fMinY=mPICVGEBoundingBoxesByFilePos[firstFilePos][1];
                double fMaxX=mPICVGEBoundingBoxesByFilePos[firstFilePos][2];
                double fMaxY=mPICVGEBoundingBoxesByFilePos[firstFilePos][3];
                QMap<quint16,QMap<quint16,quint16> > firstMaxHeightsByTileXY=mPICVGEMaxHeightsByTileXYByFilePos[firstFilePos];
                for(int nsyf=0;nsyf<secondYearFilesPos.size();nsyf++)
                {
                    int secondFilePos=secondYearFilesPos[nsyf];
                    double sMinX=mPICVGEBoundingBoxesByFilePos[secondFilePos][0];
                    double sMinY=mPICVGEBoundingBoxesByFilePos[secondFilePos][1];
                    double sMaxX=mPICVGEBoundingBoxesByFilePos[secondFilePos][2];
                    double sMaxY=mPICVGEBoundingBoxesByFilePos[secondFilePos][3];
                    if(sMaxX<=fMinX||sMinX>=fMaxX
                            ||sMaxY<=fMinY||sMinY>=fMaxY)
                    {
                        continue;
                    }
                    QMap<quint16,QMap<quint16,quint16> > secondMaxHeightsByTileXY=mPICVGEMaxHeightsByTileXYByFilePos[secondFilePos];
                    QMap<quint16,QMap<quint16,quint16> >::const_iterator iterShTileX=secondMaxHeightsByTileXY.begin();
                    while(iterShTileX!=secondMaxHeightsByTileXY.end())
                    {
                        quint16 sPosX=iterShTileX.key();
                        QMap<quint16,quint16> secondMaxHeightsByTileY=iterShTileX.value();
                        QMap<quint16,quint16>::const_iterator iterShTileY=secondMaxHeightsByTileY.begin();
                        while(iterShTileY!=secondMaxHeightsByTileY.end())
                        {
                            quint16 sPosY=iterShTileY.key();
                            double x=sPosX*mPICVGESpatialResolution+sMinX+mPICVGESpatialResolution/2.0;
                            double y=sPosY*mPICVGESpatialResolution+sMinY+mPICVGESpatialResolution/2.0;
                            quint16 fPosX=floor((x-fMinX)/mPICVGESpatialResolution);
                            quint16 fPosY=floor((y-fMinY)/mPICVGESpatialResolution);
                            if(firstMaxHeightsByTileXY.contains(fPosX))
                            {
                                if(firstMaxHeightsByTileXY[fPosX].contains(fPosY))
                                {
                                    double fHeight=firstMaxHeightsByTileXY[fPosX][fPosY]/1000.;
                                    double sHeight=secondMaxHeightsByTileXY[sPosX][sPosY]/1000.;
                                    if(sHeight>fHeight)
                                    {
                                        double annualGrowthRate=(sHeight-fHeight)/(double(secondYear-firstYear));
                                        quint16 value=(quint16)ceil(annualGrowthRate*1000.);
                                        int pos=-1;
                                        while(fHeight>stretchs[pos+1]) // hasta 2 m de altura es el primer crecimiento
                                        {
                                            pos++;
                                        }
                                        vegetationGrowthModelValues[pos].push_back(value);
                                    }
                                }
                            }
                            iterShTileY++;
                        }
                        iterShTileX++;
                    }
                }

            }

        }
    }
    mPICVGEBoundingBoxesByFilePos.clear();
    mPICVGEMaxHeightsByTileXYByFilePos.clear();
    //    outReport<<"- Number of input files ................: "<<QString::number(inputFiles.size())<<"\n";
//    QMap<int,QVector<double> > vegetationGrowthModel;
//    QVector<double> aux(6); // numberOfValues,percentil,mean,std,min,max,
//    QMap<int,QVector<quint16> > vegetationGrowthModelValues;
//    QVector<int> strectchs;
    outReport<<"- Model calculated for annual growth rates by strech from input files:"<<"\n";
    outReport<<"  - Stretchs (m)........................: "<<QString::number(stretchs.size())<<" values [";
    for(int ns=0;ns<stretchs.size();ns++)
    {
        outReport<<QString::number(stretchs[ns]);
        if(ns<(stretchs.size()-1)) outReport<<",";
    }
    outReport<<"]\n";
    for(int ns=0;ns<stretchs.size();ns++)
    {
        double meanValue=0.;
        double stdValue=-1.;
        double percentil95Value=0;
        double percentil05Value=0;
        bool existsValues=false;
        int numberOfUsedValues=0;
        if(vegetationGrowthModelValues[ns].size()>=POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_MINIMUM_NUMBER_OF_VALUES_FOR_STATISTICS)
        {
            existsValues=true;
            std::vector<double> dblValues;
            for(int nv=0;nv<vegetationGrowthModelValues[ns].size();nv++)
            {
                double value=vegetationGrowthModelValues[ns][nv]/1000.;
                stdValue+=pow(meanValue-value,2.0);
                dblValues.push_back(value);
            }
            std::sort(dblValues.begin(), dblValues.end());
            int posPercentil95=ceil(vegetationGrowthModelValues[ns].size()*POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_STRETCH_PERCENTIL/100.);
            percentil95Value=dblValues[posPercentil95];
            int posPercentil05=floor(vegetationGrowthModelValues[ns].size()*(1.-POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_STRETCH_PERCENTIL/100.));
            percentil05Value=dblValues[posPercentil05];
            numberOfUsedValues=posPercentil95-posPercentil05;
            for(int nv=posPercentil05;nv<posPercentil95;nv++)
            {
                double value=dblValues[nv];
                meanValue+=value;
            }
            meanValue/=numberOfUsedValues;
            stdValue=0.;
            for(int nv=posPercentil05;nv<posPercentil95;nv++)
            {
                double value=dblValues[nv];
                stdValue+=pow(meanValue-value,2.0);
                dblValues.push_back(value);
            }
            stdValue=sqrt(stdValue/(numberOfUsedValues-1));
            //            std::sort(v.begin(), v.end(), sort_using_greater_than); // de mas a menos
        }
        outReport<<"  - Stretch (m).........................: "<<QString::number(stretchs[ns])<<"\n";
        if(existsValues)
        {
//            outReport<<"    - Number of values .................: "<<QString::number(vegetationGrowthModelValues[ns].size())<<"\n";
            outReport<<"    - Number of values .................: "<<QString::number(numberOfUsedValues)<<"\n";
            outReport<<"    - Mean value (m)....................: "<<QString::number(meanValue,'f',2)<<"\n";
            outReport<<"    - Std value (m).....................: "<<QString::number(stdValue,'f',2)<<"\n";
            outReport<<"    - Percentil value  "<< QString::number(100-POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_STRETCH_PERCENTIL);
            outReport<<"% (m)...........: "<<QString::number(percentil05Value,'f',2)<<"\n";
            outReport<<"    - Percentil value "<< QString::number(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_STRETCH_PERCENTIL);
            outReport<<"% (m)...........: "<<QString::number(percentil95Value,'f',2)<<"\n";
            if(stdValue>0)
            if(vegetationGrowthModel[stretchs[ns]][0]==0)
            {
                vegetationGrowthModel[stretchs[ns]][0]=vegetationGrowthModelValues[ns].size();
                vegetationGrowthModel[stretchs[ns]][1]=meanValue;
                vegetationGrowthModel[stretchs[ns]][2]=stdValue;
                vegetationGrowthModel[stretchs[ns]][3]=percentil05Value;
                vegetationGrowthModel[stretchs[ns]][4]=percentil95Value;
            }
            else
            {
                int previousNumberOfValues=vegetationGrowthModel[stretchs[ns]][0];
                double previousMeanValue=vegetationGrowthModel[stretchs[ns]][1];
                double previousStdValue=vegetationGrowthModel[stretchs[ns]][2];
                double previousPercentile05Value=vegetationGrowthModel[stretchs[ns]][3];
                double previousPercentile95Value=vegetationGrowthModel[stretchs[ns]][4];
                int finalNumberOfValues=previousNumberOfValues+numberOfUsedValues;
                double finalPercentil05Value=(previousPercentile05Value*previousNumberOfValues+percentil05Value*numberOfUsedValues)/(previousNumberOfValues+numberOfUsedValues);
                double finalPercentil95Value=(previousPercentile95Value*previousNumberOfValues+percentil95Value*numberOfUsedValues)/(previousNumberOfValues+numberOfUsedValues);
                double finalMeanValue=(previousMeanValue*previousNumberOfValues+meanValue*numberOfUsedValues)/(previousNumberOfValues+numberOfUsedValues);
                double finalStdValue=(previousStdValue*previousNumberOfValues+stdValue*numberOfUsedValues)/(previousNumberOfValues+numberOfUsedValues);
                vegetationGrowthModel[stretchs[ns]][0]=finalNumberOfValues;
                vegetationGrowthModel[stretchs[ns]][1]=finalMeanValue;
                vegetationGrowthModel[stretchs[ns]][2]=finalStdValue;
                vegetationGrowthModel[stretchs[ns]][3]=finalPercentil05Value;
                vegetationGrowthModel[stretchs[ns]][4]=finalPercentil95Value;
            }
        }
        else
        {
            outReport<<"    - There are no values"<<"\n";
        }
    }
    reportFile.close();
    if(QFile::exists(modelFileName))
    {
//        outReport<<"- Model combining the calculated with the existing one:"<<"\n";
        if(!QFile::remove(modelFileName))
        {
            strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
            strError+=QObject::tr("\nError removing model file:\n%1").arg(modelFileName);
            return(false);
        }
    }
    QFile modelFile(modelFileName);
    if (!modelFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        strError=QObject::tr("PointCloudFileManager::processInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nError opening model file:\n%1")
                .arg(modelFileName);
        return(false);
    }
    QTextStream outModel(&modelFile);
    outModel<<"VEGETATION GROWTH ESTIMATION FROM POINT CLOUDS MODEL"<<"\n";
    for(int ns=0;ns<stretchs.size();ns++)
    {
        outModel<<"- Stretch ............................";
        outModel<<POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR<<" ";
        outModel<<QString::number(stretchs[ns])<<"\n";
        outModel<<"  - Number of values .................";
        outModel<<POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR<<" ";
        outModel<<QString::number(vegetationGrowthModel[stretchs[ns]][0])<<"\n";
        if(vegetationGrowthModel[stretchs[ns]][0]==0) continue;
        outModel<<"  - Mean value (m)....................";
        outModel<<POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR<<" ";
        outModel<<QString::number(vegetationGrowthModel[stretchs[ns]][1],'f',2)<<"\n";
        outModel<<"  - Std value (m).....................";
        outModel<<POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR<<" ";
        outModel<<QString::number(vegetationGrowthModel[stretchs[ns]][2],'f',2)<<"\n";
        outModel<<"  - Percentil value  "<< QString::number(100-POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_STRETCH_PERCENTIL);
        outModel<<"% (m)...........";
        outModel<<POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR<<" ";
        outModel<<QString::number(vegetationGrowthModel[stretchs[ns]][3],'f',2)<<"\n";
        outModel<<"  - Percentil value "<< QString::number(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_STRETCH_PERCENTIL);
        outModel<<"% (m)...........";
        outModel<<POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR<<" ";
        outModel<<QString::number(vegetationGrowthModel[stretchs[ns]][4],'f',2)<<"\n";
    }
    modelFile.close();
    return(true);
}

bool PointCloudFileManager::processReclassificationConfusionMatrixReport(QString &pcfPath,
                                                                         QString &outputFileName,
                                                                         QVector<int> &selectedClasses,
                                                                         QString &strError)
{
    QString strAuxError;
    if(mPtrCrsTools==NULL)
    {
        strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReport");
        strError+=QObject::tr("\nCrsTools is NULL");
        return(false);
    }
    if(selectedClasses.size()<1)
    {
        strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReport");
        strError+=QObject::tr("\nNo selected classes for point cloud path:|n%1").arg(pcfPath);
        return(false);
    }
    QString strSelectedClasses;
    for(int i=0;i<selectedClasses.size();i++)
    {
        strSelectedClasses+=QString::number(selectedClasses[i]);
        if(i<(selectedClasses.size()-1))
        {
            strSelectedClasses+=";";
        }
    }
    QString prjFileName=mBasePath+POINTCLOUDFILE_TEMPORAL_PROJECT_FILE;
    if(QFile::exists(prjFileName))
    {
        if(!QFile::remove(prjFileName))
        {
            strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReport");
            strError+=QObject::tr("\nError existing temporal project file:\n%1")
                    .arg(prjFileName);
            return(false);
        }
    }
    QFile file(prjFileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReport");
        strError+=QObject::tr("\nError opening temporal project file:\n%1")
                .arg(prjFileName);
        return(false);
    }
    QTextStream strOut(&file);

    int numberOfProcesses=1;

    strOut<<"Point Cloud File Management Project\n";
    strOut<<"- Number of processes ......................................# ";
    strOut<<QString::number(numberOfProcesses);
    strOut<<"\n";
    strOut<<"- Process (CREATE_PCF,ADD_POINTCLOUD,ADD_ROI,WRITE_PCFS) ...# ";
    strOut<<POINTCLOUDFILE_PROCESS_PROCESS_RECLASSIFICATION_CONFUSION_MATRIX_REPORT;
    strOut<<"\n";
    strOut<<"  - Point cloud path .......................................# \"";
    strOut<<pcfPath;
    strOut<<"\"\n";
    strOut<<"  - Output file name .......................................# ";
    strOut<<outputFileName;
    strOut<<"\n";
    strOut<<"  - Selected classes (; separated values) ..................#";
    strOut<<strSelectedClasses;
    strOut<<"\n";
    file.close();
    if(!processProjectFile(prjFileName,
                           strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReport");
        strError+=QObject::tr("\nError calling setFromProjectFile for project file:\n%1\nError:\n%2")
                .arg(prjFileName).arg(strAuxError);
        return(false);
    }
    return(true);
}

void PointCloudFileManager::on_ProgressExternalProcessDialog_closed()
{
    mPtrProgressExternalProcessDialog->setAutoCloseWhenFinish(false);
    disconnect(mPtrProgressExternalProcessDialog, SIGNAL(dialog_closed()),this,SLOT(on_ProgressExternalProcessDialog_closed()));
    if(mPtrProgressExternalProcessDialog->getState()==PROCESSDIALOG_STATE_ERROR)
    {
        QString title=mProgressExternalProcessTitle;
        QString msg=QObject::tr("Error executing:\n");
        msg+=mStrExecution;
        QMessageBox::information(new QWidget(),title,msg);
        return;
    }
    if(mPtrProgressExternalProcessDialog->getState()==PROCESSDIALOG_STATE_CANCELED)
    {
        QString title=mProgressExternalProcessTitle;
        QString msg=QObject::tr("Process was canceled:\n");
        msg+=mStrExecution;
        QMessageBox::information(new QWidget(),title,msg);
        return;
    }
    QString msg="Successful process completed";
    QDateTime finalDateTime=QDateTime::currentDateTime();
    int initialSeconds=(int)mInitialDateTime.toTime_t();
    int finalSeconds=(int)finalDateTime.toTime_t();
    int totalDurationSeconds=finalSeconds-initialSeconds;
    double dblTotalDurationSeconds=(double)totalDurationSeconds;
    int durationDays=(int)floor(dblTotalDurationSeconds/60.0/60.0/24.0);
    int durationHours=(int)floor((dblTotalDurationSeconds-durationDays*60.0*60.0*24.0)/60.0/60.0);
    int durationMinutes=(int)floor((dblTotalDurationSeconds-durationDays*60.0*60.0*24.0-durationHours*60.0*60.0)/60.0);
    int durationSeconds=dblTotalDurationSeconds-durationDays*60.0*60.0*24.0-durationHours*60.0*60.0-durationMinutes*60.0;
    {
        QString msgTtime="- Process time:\n";
        msgTtime+="  - Start time of the process ......................: ";
        msgTtime+=mInitialDateTime.toString("yyyy/MM/dd - hh/mm/ss.zzz");
        msgTtime+="\n";
        msgTtime+="  - End time of the process ........................: ";
        msgTtime+=finalDateTime.toString("yyyy/MM/dd - hh/mm/ss.zzz");
        msgTtime+="\n";
        msgTtime+="  - Number of total seconds ........................: ";
        msgTtime+=QString::number(dblTotalDurationSeconds,'f',3);
        msgTtime+="\n";
        msgTtime+="    - Number of days ...............................: ";
        msgTtime+=QString::number(durationDays);
        msgTtime+="\n";
        msgTtime+="    - Number of hours ..............................: ";
        msgTtime+=QString::number(durationHours);
        msgTtime+="\n";
        msgTtime+="    - Number of minutes ............................: ";
        msgTtime+=QString::number(durationMinutes);
        msgTtime+="\n";
        msgTtime+="    - Number of seconds ............................: ";
        msgTtime+=QString::number(durationSeconds,'f',3);
        msgTtime+="\n";
        msg+=msgTtime;
    }
    QString title=mProgressExternalProcessTitle;
    QMessageBox::information(new QWidget(),title,msg);
    return;
}

bool PointCloudFileManager::addPointCloudFile(QString fileName,
                                              QTextStream &in,
                                              QString &strError)
{
    QString pcPath,dbCrsDescription,dbCrsProj4String;
    QString pointCloudFileName,pointCloudCrsDescription,pointCloudCrsProj4String;
    int pointCloudCrsEpsgCode;
    int dbCrsEpsgCode;
    double gridSize;

    int intValue,nline=0;
    double dblValue;
    bool okToInt,okToDouble;
    QString strLine,strValue;
    QStringList strList;
    QStringList strAuxList;
    QDir currentDir=QDir::current();

    QString msg,strAuxError;

    // Reading database file name
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    pcPath=strList.at(1).trimmed();
    if(pcPath.startsWith("\"")&&pcPath.endsWith("\""))
    {
        pcPath=pcPath.remove("\"");
    }
    if(!currentDir.exists(pcPath))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nNot exists point cloud path:\n%1\nYou must create it before").arg(pcPath);
        return(false);
    }
    if(!openPointCloudFile(pcPath,
                           strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError setting point cloud from path:\n%1\nError:\n%2")
                .arg(pcPath).arg(strAuxError);
        return(false);
    }
    dbCrsDescription=mPtrPcFiles[pcPath]->getCrsDescription();
    dbCrsEpsgCode=mPtrPcFiles[pcPath]->getCrsEpsgCode();
    if(!mPtrCrsTools->appendUserCrs(dbCrsEpsgCode,
                                    dbCrsProj4String,
                                    strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError adding CRS from PROJ4:\n%1\nfrom file:\n%2\nError:\n%3")
                .arg(dbCrsProj4String).arg(pcPath).arg(strAuxError);
        return(false);
    }

    /*
    // DEPURACION
    QString wktGeometry="POLYGON((481525.12825687526 4301653.6647398835,481702.64052925183 4301653.6647398835,481702.64052925183 4301542.959537572,481525.12825687526 4301542.959537572,481525.12825687526 4301653.6647398835))";
    int geometryCrsEpsgCode=25830;
    QString geometryCrsProj4="+proj=utm +zone=30 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";
    wktGeometry="POLYGON((-3.212947443852194 38.86366822705954,-3.210898444917243 38.86366822705954,-3.210898444917243 38.86239037264084,-3.212947443852194 38.86239037264084,-3.212947443852194 38.86366822705954))";
    geometryCrsEpsgCode=4326;
    geometryCrsProj4="+proj=longlat +datum=WGS84 +no_defs";
    QVector<QString> tilesTableNames;
    if(!getTilesTableNamesFromWktGeometry(dbFileName,
                                          wktGeometry,
                                          geometryCrsEpsgCode,
                                          geometryCrsProj4,
                                          tilesTableNames,
                                          strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError adding CRS from PROJ4:\n%1\nfrom file:\n%2\nError:\n%3")
                .arg(dbCrsProj4String).arg(dbFileName).arg(strAuxError);
        return(false);
    }
    int yo=1;
    yo++;
    // DEPURACION
    */


//    dbCrsProj4String=mPtrPcSpDbs[dbFileName]->getCrsProj4String();
//    if(!mPtrCrsTools->appendUserCrs(dbCrsProj4String,
//                                    dbCrsDescription,
//                                    strAuxError))
//    {
//        strError=QObject::tr("PointCloudFileManager::addROI");
//        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
//        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
//        strError+=QObject::tr("\nError adding CRS from PROJ4:\n%1\nfrom file:\n%2\nError:\n%3")
//                .arg(dbCrsProj4String).arg(dbFileName).arg(strAuxError);
//        return(false);
//    }

    // Reading input file
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    pointCloudFileName=strList.at(1).trimmed();
    if(pointCloudFileName.startsWith("\"")&&pointCloudFileName.endsWith("\""))
    {
        pointCloudFileName=pointCloudFileName.remove("\"");
    }
    if(!QFile::exists(pointCloudFileName))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nNot exists shapefile:\n%1").arg(pointCloudFileName);
        return(false);
    }

    // Reading input CRS
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    okToInt=false;
    strValue=strList.at(1).trimmed();
    int epsgCode=strValue.toInt(&okToInt);
    pointCloudCrsEpsgCode=-1;
    // epsg code
    if(okToInt)
    {
        if(!mPtrCrsTools->appendUserCrs(epsgCode,
                                        pointCloudCrsDescription,
                                        strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nInvalid CRS From EPSG code: %1").arg(QString::number(epsgCode));
            return(false);
        }
        pointCloudCrsEpsgCode=epsgCode;
    }
    else
    {
        // libCRS string
        if(!mPtrCrsTools->isValidCrs(strValue,strAuxError))
        {
            // proj4
            if(!mPtrCrsTools->appendUserCrs(strValue,
                                            pointCloudCrsDescription,
                                            strAuxError))
            {
                // wkt
                if(!mPtrCrsTools->appendUserCrsFromWkt(strValue,
                                               pointCloudCrsDescription,
                                               strAuxError))
                {
                    int yo=1;
                }
            }
        }
        else
        {
            pointCloudCrsDescription=strValue;
        }
    }
    if(!mPtrCrsTools->isValidCrs(pointCloudCrsDescription,strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nInvalid CRS from string: %1\nError:\n%2")
                .arg(strValue).arg(strAuxError);
        return(false);
    }
    if(!mPtrCrsTools->getProj4String(pointCloudCrsDescription,pointCloudCrsProj4String,strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError recovering PROJ4 string from CRS: %1\nError:\n%2")
                .arg(dbCrsDescription).arg(strAuxError);
        return(false);
    }
    if(pointCloudCrsEpsgCode==-1)
    {
        if(!mPtrCrsTools->getCrsEpsgCode(pointCloudCrsDescription,pointCloudCrsEpsgCode,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nError recovering EPSG code from CRS: %1\nError:\n%2")
                    .arg(dbCrsDescription).arg(strAuxError);
            return(false);
        }
    }
    bool updateHeader=true;
    if(!mPtrPcFiles[pcPath]->addPointCloudFile(pointCloudFileName,
                                               pointCloudCrsDescription,
                                               pointCloudCrsProj4String,
                                               pointCloudCrsEpsgCode,
                                               updateHeader,
                                               strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
        strError+=QObject::tr("\nError adding point cloud file:\n%1\nError:\n%2")
                .arg(pointCloudFileName).arg(strAuxError);
        return(false);
    }
    return(true);
}

bool PointCloudFileManager::addPointCloudFiles(QString fileName,
                                               QTextStream &in,
                                               QString &strError)
{
    QString pcPath,dbCrsDescription,dbCrsProj4String;
    QString pointCloudFileName,pointCloudCrsDescription,pointCloudCrsProj4String;
    int pointCloudCrsEpsgCode;
    int dbCrsEpsgCode;
    double gridSize;

    int intValue,nline=0;
    double dblValue;
    bool okToInt,okToDouble;
    QString strLine,strValue;
    QStringList strList;
    QStringList strAuxList;
    QDir currentDir=QDir::current();

    QString msg,strAuxError;

    // Reading database file name
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    pcPath=strList.at(1).trimmed();
    if(pcPath.startsWith("\"")&&pcPath.endsWith("\""))
    {
        pcPath=pcPath.remove("\"");
    }
    if(!currentDir.exists(pcPath))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nNot exists point cloud path:\n%1\nYou must create it before").arg(pcPath);
        return(false);
    }
    if(!openPointCloudFile(pcPath,
                           strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError setting point cloud from path:\n%1\nError:\n%2")
                .arg(pcPath).arg(strAuxError);
        return(false);
    }
    dbCrsDescription=mPtrPcFiles[pcPath]->getCrsDescription();
    dbCrsEpsgCode=mPtrPcFiles[pcPath]->getCrsEpsgCode();
    if(!mPtrCrsTools->appendUserCrs(dbCrsEpsgCode,
                                    dbCrsProj4String,
                                    strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError adding CRS from PROJ4:\n%1\nfrom file:\n%2\nError:\n%3")
                .arg(dbCrsProj4String).arg(pcPath).arg(strAuxError);
        return(false);
    }

    /*
    // DEPURACION
    QString wktGeometry="POLYGON((481525.12825687526 4301653.6647398835,481702.64052925183 4301653.6647398835,481702.64052925183 4301542.959537572,481525.12825687526 4301542.959537572,481525.12825687526 4301653.6647398835))";
    int geometryCrsEpsgCode=25830;
    QString geometryCrsProj4="+proj=utm +zone=30 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";
    wktGeometry="POLYGON((-3.212947443852194 38.86366822705954,-3.210898444917243 38.86366822705954,-3.210898444917243 38.86239037264084,-3.212947443852194 38.86239037264084,-3.212947443852194 38.86366822705954))";
    geometryCrsEpsgCode=4326;
    geometryCrsProj4="+proj=longlat +datum=WGS84 +no_defs";
    QVector<QString> tilesTableNames;
    if(!getTilesTableNamesFromWktGeometry(dbFileName,
                                          wktGeometry,
                                          geometryCrsEpsgCode,
                                          geometryCrsProj4,
                                          tilesTableNames,
                                          strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError adding CRS from PROJ4:\n%1\nfrom file:\n%2\nError:\n%3")
                .arg(dbCrsProj4String).arg(dbFileName).arg(strAuxError);
        return(false);
    }
    int yo=1;
    yo++;
    // DEPURACION
    */


//    dbCrsProj4String=mPtrPcSpDbs[dbFileName]->getCrsProj4String();
//    if(!mPtrCrsTools->appendUserCrs(dbCrsProj4String,
//                                    dbCrsDescription,
//                                    strAuxError))
//    {
//        strError=QObject::tr("PointCloudFileManager::addROI");
//        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
//        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
//        strError+=QObject::tr("\nError adding CRS from PROJ4:\n%1\nfrom file:\n%2\nError:\n%3")
//                .arg(dbCrsProj4String).arg(dbFileName).arg(strAuxError);
//        return(false);
//    }

    // Reading input CRS
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    okToInt=false;
    strValue=strList.at(1).trimmed();
    int epsgCode=strValue.toInt(&okToInt);
    pointCloudCrsEpsgCode=-1;
    // epsg code
    if(okToInt)
    {
        if(!mPtrCrsTools->appendUserCrs(epsgCode,
                                        pointCloudCrsDescription,
                                        strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nInvalid CRS From EPSG code: %1").arg(QString::number(epsgCode));
            return(false);
        }
        pointCloudCrsEpsgCode=epsgCode;
    }
    else
    {
        // libCRS string
        if(!mPtrCrsTools->isValidCrs(strValue,strAuxError))
        {
            // proj4
            if(!mPtrCrsTools->appendUserCrs(strValue,
                                            pointCloudCrsDescription,
                                            strAuxError))
            {
                // wkt
                if(!mPtrCrsTools->appendUserCrsFromWkt(strValue,
                                               pointCloudCrsDescription,
                                               strAuxError))
                {
                    int yo=1;
                }
            }
        }
        else
        {
            pointCloudCrsDescription=strValue;
        }
    }
    if(!mPtrCrsTools->isValidCrs(pointCloudCrsDescription,strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nInvalid CRS from string: %1\nError:\n%2")
                .arg(strValue).arg(strAuxError);
        return(false);
    }
    if(!mPtrCrsTools->getProj4String(pointCloudCrsDescription,pointCloudCrsProj4String,strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError recovering PROJ4 string from CRS: %1\nError:\n%2")
                .arg(dbCrsDescription).arg(strAuxError);
        return(false);
    }
    if(pointCloudCrsEpsgCode==-1)
    {
        if(!mPtrCrsTools->getCrsEpsgCode(pointCloudCrsDescription,pointCloudCrsEpsgCode,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nError recovering EPSG code from CRS: %1\nError:\n%2")
                    .arg(dbCrsDescription).arg(strAuxError);
            return(false);
        }
    }

    // Reading number of files
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    okToInt=false;
    strValue=strList.at(1).trimmed();
    int numberOfFiles=strValue.toInt(&okToInt);
    if(!okToInt)
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nInvalid number of files: %1").arg(strValue);
        return(false);
    }

    // Reading files
    QVector<QString> pointCloudFileNames;
    for(int nf=0;nf<numberOfFiles;nf++)
    {
        // Reading input file
        nline++;
        strLine=in.readLine();
        strLine=strLine.trimmed();
        strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        if(strList.size()!=2)
        {
            strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
            return(false);
        }
        pointCloudFileName=strList.at(1).trimmed();
        if(pointCloudFileName.startsWith("\"")&&pointCloudFileName.endsWith("\""))
        {
            pointCloudFileName=pointCloudFileName.remove("\"");
        }
        if(!QFile::exists(pointCloudFileName))
        {
            strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nNot exists shapefile:\n%1").arg(pointCloudFileName);
            return(false);
        }
        pointCloudFileNames.push_back(pointCloudFileName);
    }

    bool updateHeader=true;
    if(!mPtrPcFiles[pcPath]->addPointCloudFiles(pointCloudFileNames,
                                                pointCloudCrsDescription,
                                                pointCloudCrsProj4String,
                                                pointCloudCrsEpsgCode,
                                                updateHeader,
                                                strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addPointCloudFiles");
        strError+=QObject::tr("\nError adding point cloud files:\n%1")
                .arg(strAuxError);
        return(false);
    }
    return(true);
}

bool PointCloudFileManager::addROI(QString fileName,
                                   QTextStream &in,
                                   QString &strError)
{
    QString pcPath,dbCrsDescription,dbCrsProj4String;
    QString shapeFileName,shapeFileCrsDescription,fieldId;
    QStringList fieldIdValues;
    int dbCrsEpsgCode;
    double gridSize;

    int intValue,nline=0;
    double dblValue;
    bool okToInt,okToDouble;
    QString strLine,strValue;
    QStringList strList;
    QStringList strAuxList;
    QDir currentDir=QDir::current();

    QString msg,strAuxError;

    // Reading database file name
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::addROI");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    pcPath=strList.at(1).trimmed();
    if(pcPath.startsWith("\"")&&pcPath.endsWith("\""))
    {
        pcPath=pcPath.remove("\"");
    }
    if(!currentDir.exists(pcPath))
    {
        strError=QObject::tr("PointCloudFileManager::addROI");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nNot exists point cloud path:\n%1\nYou must create it before").arg(pcPath);
        return(false);
    }
    if(!openPointCloudFile(pcPath,
                           strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addROI");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError setting point cloud file from file:\n%1\nError:\n%2")
                .arg(pcPath).arg(strAuxError);
        return(false);
    }
    dbCrsDescription=mPtrPcFiles[pcPath]->getCrsDescription();
    dbCrsEpsgCode=mPtrPcFiles[pcPath]->getCrsEpsgCode();
    if(!mPtrCrsTools->appendUserCrs(dbCrsEpsgCode,
                                    dbCrsProj4String,
                                    strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addROI");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError adding CRS from PROJ4:\n%1\nfrom file:\n%2\nError:\n%3")
                .arg(dbCrsProj4String).arg(pcPath).arg(strAuxError);
        return(false);
    }

//    dbCrsProj4String=mPtrPcSpDbs[dbFileName]->getCrsProj4String();
//    if(!mPtrCrsTools->appendUserCrs(dbCrsProj4String,
//                                    dbCrsDescription,
//                                    strAuxError))
//    {
//        strError=QObject::tr("PointCloudDbManager::addROI");
//        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
//        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
//        strError+=QObject::tr("\nError adding CRS from PROJ4:\n%1\nfrom file:\n%2\nError:\n%3")
//                .arg(dbCrsProj4String).arg(dbFileName).arg(strAuxError);
//        return(false);
//    }

    // Reading input file
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::addROI");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    shapeFileName=strList.at(1).trimmed();
    if(shapeFileName.startsWith("\"")&&shapeFileName.endsWith("\""))
    {
        shapeFileName=shapeFileName.remove("\"");
    }
    if(!QFile::exists(shapeFileName))
    {
        strError=QObject::tr("PointCloudFileManager::addROI");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nNot exists shapefile:\n%1").arg(shapeFileName);
        return(false);
    }
    IGDAL::Shapefile* ptrShapefile=new IGDAL::Shapefile();
    if(!ptrShapefile->setFromFile(shapeFileName,
                                  strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addROI");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError loading shapefile:\n%1\nError:\n%2")
                .arg(shapeFileName).arg(strAuxError);
        return(false);
    }
    QString shapeFileCrsProj4String=ptrShapefile->getCrsDescription();
    if(!mPtrCrsTools->appendUserCrs(shapeFileCrsProj4String,
                                    shapeFileCrsDescription,
                                    strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::addROI");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError recovering CRS description from PROJ4 string: %1 for shapefile:\n%2\nError:\n%3")
                .arg(shapeFileCrsProj4String).arg(shapeFileName).arg(strAuxError);
        return(false);
    }

    OGRwkbGeometryType geometryTypeShapefile=ptrShapefile->getGeometryType();
    if(geometryTypeShapefile!=wkbPolygon
            &&geometryTypeShapefile!=wkbMultiPolygon
            &&geometryTypeShapefile!=wkbPolygon25D
            &&geometryTypeShapefile!=wkbMultiPolygon25D
            &&geometryTypeShapefile!=wkbPolygonZM
            &&geometryTypeShapefile!=wkbMultiPolygonZM
            &&geometryTypeShapefile!=wkbPolygonM
            &&geometryTypeShapefile!=wkbMultiPolygonM)
    {
        strError=QObject::tr("PointCloudFileManager::addROI");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nGeometry type no POLYGON or MULTIPOLYGON in shapefile:\n%1\nError:\n%2")
                .arg(shapeFileName).arg(strAuxError);
        return(false);
    }

    // Reading fieldId
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::addROI");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    fieldId=strList.at(1).trimmed();
    if(fieldId.compare(POINTCLOUDFILE_ROIS_SHAPEFILE_FIELD_ID_NONE,Qt::CaseInsensitive)==0)
    {
        // Ignore line fieldId values
        nline++;
        strLine=in.readLine();
    }
    else
    {
        if(!ptrShapefile->getExistsFieldName(fieldId))
        {
            strError=QObject::tr("PointCloudFileManager::addROI");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nNot exists field id: %1 in shapefile:\n%2")
                    .arg(fieldId).arg(shapeFileName);
            return(false);
        }

        // Reading fieldId values
        nline++;
        strLine=in.readLine();
        strLine=strLine.trimmed();
        strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        if(strList.size()!=2)
        {
            strError=QObject::tr("PointCloudFileManager::addROI");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
            return(false);
        }
        fieldIdValues=strList.at(1).trimmed().split(POINTCLOUDFILE_ROIS_SHAPEFILE_FIELD_ID_VALUES_STRING_SEPARATOR);
        if(fieldIdValues.size()==0)
        {
            strError=QObject::tr("PointCloudFileManager::addROI");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nThere are not fieldId values separated by %1").arg(POINTCLOUDFILE_ROIS_SHAPEFILE_FIELD_ID_VALUES_STRING_SEPARATOR);
            return(false);
        }
    }
    QMap<QString,OGRGeometry* > ptrROIGeometryByRoiId; // roiId:fieldName;fieldValue;shapefile
    if(fieldIdValues.size()>0)
    {
        for(int nfidv=0;nfidv<fieldIdValues.size();nfidv++)
        {
            QString fieldIdValue=fieldIdValues.at(nfidv);
            QStringList fieldsName,fieldsValues;
            fieldsName.append(fieldId);
            fieldsValues.append(fieldIdValue);
            QVector<OGRGeometry*> ptrClonedGeometries;
            if(!ptrShapefile->getFeatureGeometry(fieldsName,
                                                 fieldsValues,
                                                 ptrClonedGeometries,
                                                 strAuxError))
            {
                strError=QObject::tr("PointCloudFileManager::addROI");
                strError+=QObject::tr("\nError reading file: %1").arg(fileName);
                strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
                strError+=QObject::tr("\nError recovering features for fielid: %1 value: %2 for shapefile:\n%3")
                        .arg(fieldId).arg(fieldIdValue).arg(shapeFileName);
                return(false);
            }
            if(ptrClonedGeometries.size()>1)
            {
                strError=QObject::tr("PointCloudFileManager::addROI");
                strError+=QObject::tr("\nError reading file: %1").arg(fileName);
                strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
                strError+=QObject::tr("\nThere are several features for field id: %1 value: %2 for shapefile:\n%3")
                        .arg(fieldId).arg(fieldIdValue).arg(shapeFileName);
                return(false);
            }
            QString roiId=fieldId+POINTCLOUDFILE_ROI_ID_STRING_SEPARATOR+fieldIdValue+POINTCLOUDFILE_ROI_ID_STRING_SEPARATOR+shapeFileName;
            OGRGeometry* ptrGeometry=ptrClonedGeometries[0];
            int coordinateDimension=ptrGeometry->getCoordinateDimension();
            if(coordinateDimension==3)
            {
                ptrGeometry->flattenTo2D();
            }
            coordinateDimension=ptrGeometry->getCoordinateDimension();
            ptrROIGeometryByRoiId[roiId]=ptrGeometry;
//            ptrROIGeometryByRoiId[roiId]=ptrClonedGeometries[0];
//            ptrROIsGeometriesByFieldIdValue[fieldIdValue]=ptrClonedGeometries;
        }
    }
    else
    {
        QVector<OGRGeometry*> ptrClonedGeometries;
        if(!ptrShapefile->getAllFeaturesGeometry(ptrClonedGeometries,
                                             strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::setFromProjectFile");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nError recovering all features for shapefile:\n%1").arg(shapeFileName);
            return(false);
        }
        for(int nf=0;nf<ptrClonedGeometries.size();nf++)
        {
            QString fieldId=POINTCLOUDFILE_ROIS_SHAPEFILE_FIELD_ID_NONE;
            QString fieldIdValue=POINTCLOUDFILE_ROIS_SHAPEFILE_FIELD_ID_NONE+QString::number(nf+1);
            QString roiId=fieldId+POINTCLOUDFILE_ROI_ID_STRING_SEPARATOR+fieldIdValue+POINTCLOUDFILE_ROI_ID_STRING_SEPARATOR+shapeFileName;
            OGRGeometry* ptrGeometry=ptrClonedGeometries[nf];
            int coordinateDimension=ptrGeometry->getCoordinateDimension();
            if(coordinateDimension==3)
            {
                ptrGeometry->flattenTo2D();
            }
            coordinateDimension=ptrGeometry->getCoordinateDimension();
            ptrROIGeometryByRoiId[roiId]=ptrGeometry;
//            ptrROIGeometryByRoiId[roiId]=ptrClonedGeometries[nf];
        }
    }
    delete(ptrShapefile);
    if(dbCrsProj4String!=shapeFileCrsDescription)
    {
        QMap<QString,OGRGeometry* >::iterator iter=ptrROIGeometryByRoiId.begin();
        while(iter!=ptrROIGeometryByRoiId.end())
        {
            QString fieldIdValue=iter.key();
            OGRGeometry* ptrGeometry=iter.value();
            if(!mPtrCrsTools->crsOperation(shapeFileCrsDescription,
                                           dbCrsProj4String,
                                           &ptrGeometry,
                                           strAuxError))
            {
                strError=QObject::tr("PointCloudFileManager::addROI");
                strError+=QObject::tr("\nError reading file: %1").arg(fileName);
                strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
                strError+=QObject::tr("\nError in geometry transform for shapefile:\n%1\nError:\n%2")
                        .arg(shapeFileName).arg(strAuxError);
                return(false);
            }
            OGREnvelope* ptrTargetEnvelope=new OGREnvelope();
            ptrGeometry->getEnvelope(ptrTargetEnvelope);
            int yo=1;
            iter++;
        }
    }
    if(!mPtrPcFiles[pcPath]->addROIs(ptrROIGeometryByRoiId,
                                         strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::setFromProjectFile");
        strError+=QObject::tr("\nError adding ROIs for shapefile:\n%1\nError:\n%2")
                .arg(shapeFileName).arg(strAuxError);
        return(false);
    }
    // fieldId

//    roisShapeFileNames.push_back(shapeFileName);
//    crsDescriptionByROIShapefile[shapeFileName]=shapeFileCrsDescription;
//    ptrROIsGeometriesByFieldIdValueByROIShapefile[shapeFileName]=ptrROIsGeometriesByFieldIdValue;

//    freeOGRGeometries(ptrROIsGeometriesByFieldIdValueByROIShapefile);
    return(true);
}

bool PointCloudFileManager::createPointCloudFile(QString fileName,
                                                 QTextStream &in,
                                                 QString &strError)
{
    QString pcPath,dbCrsDescription,dbCrsProj4String;
    int crsEpsgCode;
    double gridSize;

    int intValue,nline=0;
    double dblValue;
    bool okToInt,okToDouble;
    QString strLine,strValue;
    QStringList strList;
    QStringList strAuxList;
    QDir currentDir=QDir::current();

    QString msg,strAuxError;

    // Reading path
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    pcPath=strList.at(1).trimmed();
    if(pcPath.startsWith("\"")&&pcPath.endsWith("\""))
    {
        pcPath=pcPath.remove("\"");
    }
    if(currentDir.exists(pcPath)
            &&QDir(pcPath).entryInfoList(QDir::NoDotAndDotDot|QDir::AllEntries).count() != 0)
    {
//        QFile::remove(dbFileName);
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nExists not empty point cloud path:\n%1\nYou must remove it before").arg(pcPath);
        return(false);
    }

    // Reading database CRS
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    okToInt=false;
    strValue=strList.at(1).trimmed();
    int epsgCode=strValue.toInt(&okToInt);
    crsEpsgCode=-1;
    // epsg code
    if(okToInt)
    {
        if(!mPtrCrsTools->appendUserCrs(epsgCode,
                                        dbCrsDescription,
                                        strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nInvalid CRS From EPSG code: %1").arg(QString::number(epsgCode));
            return(false);
        }
        crsEpsgCode=epsgCode;
    }
    else
    {
        // libCRS string
        if(!mPtrCrsTools->isValidCrs(strValue,strAuxError))
        {
            // proj4
            if(!mPtrCrsTools->appendUserCrs(strValue,
                                            dbCrsDescription,
                                            strAuxError))
            {
                // wkt
                if(!mPtrCrsTools->appendUserCrsFromWkt(strValue,
                                               dbCrsDescription,
                                               strAuxError))
                {
                    int yo=1;
                }
            }
        }
        else
        {
            dbCrsDescription=strValue;
        }
    }
    if(!mPtrCrsTools->isValidCrs(dbCrsDescription,strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nInvalid CRS from string: %1\nError:\n%2")
                .arg(strValue).arg(strAuxError);
        return(false);
    }
    if(!mPtrCrsTools->getProj4String(dbCrsDescription,dbCrsProj4String,strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError recovering PROJ4 string from CRS: %1\nError:\n%2")
                .arg(dbCrsDescription).arg(strAuxError);
        return(false);
    }
    if(crsEpsgCode==-1)
    {
        if(!mPtrCrsTools->getCrsEpsgCode(dbCrsDescription,crsEpsgCode,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nError recovering EPSG code from CRS: %1\nError:\n%2")
                    .arg(dbCrsDescription).arg(strAuxError);
            return(false);
        }
    }

    // Reading Height type
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    strValue=strList.at(1).trimmed();
    int verticalCrsEpsgCode=strValue.toInt(&okToInt);
    // epsg code
    QString heightType;
    if(!okToInt)
    {
        if(strValue.compare(CRSTOOLS_XMLFILE_TAG_ORTHOMETRIC_HEIGHT,Qt::CaseInsensitive)!=0
                &&strValue.compare(CRSTOOLS_XMLFILE_TAG_ELLIPSOID_HEIGHT,Qt::CaseInsensitive)!=0)
        {
            strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nInvalide height type: %1, must be %2 or %3")
                    .arg(strValue).arg(CRSTOOLS_XMLFILE_TAG_ORTHOMETRIC_HEIGHT).arg(CRSTOOLS_XMLFILE_TAG_ORTHOMETRIC_HEIGHT);
            return(false);
        }
        heightType=strValue;
    }

    // Reading grid size
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    okToDouble=false;
    strValue=strList.at(1).trimmed();
    gridSize=strValue.toDouble(&okToDouble);
    if(!okToDouble)
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nGrid size is not a double value: %1")
                .arg(strValue);
        return(false);
    }
    int intGridSize=qRound(gridSize);
    if(intGridSize>mMaxGridSize)
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nGrid size: %1 is greather than maximum valid: %2")
                .arg(strValue).arg(QString::number(mMaxGridSize));
        return(false);
    }

    // Reading project type
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    QString projectType=strList.at(1).trimmed().toLower();
    if(!mProjectTypes.contains(projectType))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nInvalid project type %1").arg(projectType);
        return(false);
    }

    // Reading project parameters
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    QString projectParametersString=strList.at(1).trimmed();

    if(!validateProjectParametersString(projectType,projectParametersString,strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nInvalid project parameters string:\n%1\nfor project type: %2\nError:\n%3")
                .arg(projectParametersString).arg(projectType).arg(strAuxError);
        return(false);
    }

//    if(!selectProjectParameters(projectType,strAuxError))
//    {
//        strError=QObject::tr("PointCloudDbManager::createDb");
//        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
//        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
//        strError+=QObject::tr("\nInvalid project parameters string:\n%1\nfor project type: %2\nError:\n%3")
//                .arg(projectParametersString).arg(projectType).arg(strAuxError);
//        return(false);
//    }

    PointCloudFile* ptrPcFile=new PointCloudFile(mPtrCrsTools,
                                                 this);
    if(heightType.isEmpty())
    {
        if(!ptrPcFile->create(pcPath,
                              crsEpsgCode,
                              verticalCrsEpsgCode,
                              gridSize,
                              projectType,
                              projectParametersString,
                              strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nError creating object:\n: %1")
                    .arg(strAuxError);
            return(false);
        }
    }
    else
    {
        if(!ptrPcFile->create(pcPath,
                              dbCrsDescription,dbCrsProj4String,crsEpsgCode,heightType,
                              gridSize,
                              projectType,
                              projectParametersString,
                              strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nError creating object:\n: %1")
                    .arg(strAuxError);
            return(false);
        }
    }
    if(!ptrPcFile->setTempPath(mTempPath,strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::createPointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError setting temporal path:\n%1\nto database:\n: %2\nError:\n%3")
                .arg(mTempPath).arg(pcPath).arg(strAuxError);
        delete(ptrPcFile);
        return(false);
    }
    mPtrPcFiles[pcPath]=ptrPcFile;
    return(true);
}

bool PointCloudFileManager::getProjectParametersString(QString projectType,
                                                       QString &projectParametersString,
                                                       QString &strError)
{
    projectParametersString.clear();
    QString strAuxError;
    if(!mProjectTypes.contains(projectType))
    {
        strError=QObject::tr("PointCloudFileManager::getProjectParametersString");
        strError+=QObject::tr("\nInvalid project type: %1").arg(projectType);
        return(false);
    }
    if(mPtrProjectsParametersManagerByProjectType[projectType]==NULL)
    {
        strError=QObject::tr("PointCloudFileManager::getProjectParametersString");
        strError+=QObject::tr("\nThere are no parameters for project type: %1")
                .arg(projectType);
        return(false);
    }
    QVector<Parameter *> ptrParameters;
    ParametersManager* ptrParametersManager=mPtrProjectsParametersManagerByProjectType[projectType];
    QString command=POINTCLOUDFILE_PROJECT_PARAMETERS_TAG;
    bool onlyEnabled=true;
    if(!ptrParametersManager->getParametersByCommand(command,
                                                     ptrParameters,
                                                     onlyEnabled))
    {
        strError=QObject::tr("PointCloudFileManager::getProjectParametersString");
        strError+=QObject::tr("\nError getting parameters for project type: %1")
                .arg(projectType);
        return(false);
    }
    for(int np=0;np<ptrParameters.size();np++)
    {
        if(np>0)
        {
            projectParametersString+=POINTCLOUDFILE_PROJECT_PARAMETERS_FILE_PARAMETERS_STRING_SEPARATOR;
        }
        Parameter* ptrParameter=ptrParameters[np];
        QString parameterCode=ptrParameter->getCode();
        QString value;
        ptrParameter->getValue(value);
        QString strParameter=parameterCode+POINTCLOUDFILE_PROJECT_PARAMETERS_FILE_PARAMETER_VALUE_STRING_SEPARATOR+value;
        projectParametersString+=strParameter;
    }
    return(true);
}

bool PointCloudFileManager::removeDir(QString dirName,
                                      bool onlyContent)
{
    bool result = true;
    QDir dir(dirName);
    if (dir.exists(dirName))
    {
        Q_FOREACH(QFileInfo info, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::System | QDir::Hidden  | QDir::AllDirs | QDir::Files, QDir::DirsFirst)) {
            if (info.isDir())
            {
                result = removeDir(info.absoluteFilePath());
            }
            else
            {
                result = QFile::remove(info.absoluteFilePath());
            }
            if (!result)
            {
                return result;
            }
        }
        if(!onlyContent)
        {
            result = dir.rmdir(dirName);
        }
    }
    return result;
}

bool PointCloudFileManager::runProcessList(QVector<QString> &processList,
                                           QString title,
                                           QString &strError,
                                           QWidget *ptrWidget)
{
    QString processFileName=mBasePath+POINTCLOUDFILE_PROCESS_LIST_FILE;
    if(QFile::exists(processFileName))
    {
        if(!QFile::remove(processFileName))
        {
            strError=QObject::tr("PointCloudFileManager::runProcessList");
            strError+=QObject::tr("\nError existing temporal process list file:\n%1")
                    .arg(processFileName);
            return(false);
        }
    }
    QDir baseDir(mBasePath);
    QFile file(processFileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        strError=QObject::tr("PointCloudFileManager::runProcessList");
        strError+=QObject::tr("\nError opening temporal process list file:\n%1")
                .arg(processFileName);
        return(false);
    }
    QTextStream strOut(&file);
    strOut<<"setlocal"<<"\n";
//    QString strDrive=baseDir.rootPath();
//    strOut<<strDrive<<"\n";
    strOut<<"cd \""<<baseDir.absolutePath()<<"\"\n";
    for(int np=0;np<processList.size();np++)
    {
        strOut<<processList.at(np)<<"\n";
    }
    strOut<<"endlocal"<<"\n";
    file.close();

    QStringList parameters;
    mStrExecution=processFileName;
    if(mPtrProgressExternalProcessDialog==NULL)
    {
        if(ptrWidget==NULL)
        {
            ptrWidget=new QWidget();
        }
        mPtrProgressExternalProcessDialog=new ProcessTools::ProgressExternalProcessDialog(true,ptrWidget);
        mPtrProgressExternalProcessDialog->setAutoCloseWhenFinish(false);
    }
    mPtrProgressExternalProcessDialog->setDialogTitle(title);
//    connect(mPtrProgressExternalProcessDialog, SIGNAL(dialog_closed()),this,SLOT(on_ProgressExternalProcessDialog_closed()));

    mInitialDateTime=QDateTime::currentDateTime();
    mProgressExternalProcessTitle=title;
    mPtrProgressExternalProcessDialog->runExternalProcess(mStrExecution,parameters,mBasePath);

    return(true);
}

bool PointCloudFileManager::selectInternalCommandParameters(QString internalCommand,
                                                            QString &strError)
{
    QString strAuxError;
    if(!mInternalCommands.contains(internalCommand))
    {
        strError=QObject::tr("PointCloudFileManager::selectInternalCommandParameters");
        strError+=QObject::tr("\nInvalid Internal command: %1").arg(internalCommand);
        return(false);
    }
    if(mInternalCommandsParametersFileName.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::selectInternalCommandParameters");
        strError+=QObject::tr("\nInternal commands parameters file name is empty");
        return(false);
    }
    if(!QFile::exists(mInternalCommandsParametersFileName))
    {
        strError=QObject::tr("PointCloudFileManager::selectInternalCommandParameters");
        strError+=QObject::tr("\nNot exists Internal commands parameters file name:\n%1")
                .arg(mLastoolsCommandsParametersFileName);
        return(false);
    }
    if(mPtrInternalCommandsParameters==NULL)
    {
        ParametersManager* ptrParametersManager=new ParametersManager();
        if(!ptrParametersManager->loadFromXml(mInternalCommandsParametersFileName,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::selectInternalCommandParameters");
            strError+=QObject::tr("\nError loading parameters manager from file:\n%1\nError:\n%2")
                    .arg(mInternalCommandsParametersFileName).arg(strAuxError);
            delete(ptrParametersManager);
            return(false);
        }
        mPtrInternalCommandsParameters=ptrParametersManager;
    }
    ParametersManagerDialog parameterDialog(mPtrInternalCommandsParameters,
                                            internalCommand);
    return(true);
}

bool PointCloudFileManager::processProjectFile(QString &fileName,
                                             QString &strError)
{
    QWidget* ptrWidget=new QWidget();
    QString strAuxError;
    if(!checkInitialize(strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::processProjectFile");
        strError+=QObject::tr("\nPointCloudDbManager is not initialized:\n%1").arg(strAuxError);
        return(false);
    }
    if(fileName.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::processProjectFile");
        strError+=QObject::tr("\nInput file name is empty");
        return(false);
    }
    if(!QFile::exists(fileName))
    {
        strError=QObject::tr("PointCloudFileManager::processProjectFile");
        strError+=QObject::tr("\nNot exists input file: \n%1").arg(fileName);
        return(false);
    }
    QFile fileInput(fileName);
    if (!fileInput.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        strError=QObject::tr("PointCloudFileManager::processProjectFile");
        strError+=QObject::tr("\nError opening file: \n%1").arg(fileName);
        return(false);
    }
    QFileInfo fileInputInfo(fileName);
    QString projectFileBaseName=fileInputInfo.completeBaseName();
    QTextStream in(&fileInput);

    int intValue,nline=0;
    double dblValue;
    bool okToInt,okToDouble;
    QString strLine,strValue;
    QStringList strList;
    QStringList strAuxList;
    QDir currentDir=QDir::current();

    QString msg;

    // Ignore header
    nline++;
    strLine=in.readLine();

    // Reading number of processes
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::processProjectFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        fileInput.close();
        return(false);
    }
    strValue=strList.at(1).trimmed();
    okToInt=false;
    int numberOfProcesses=strValue.toInt(&okToInt);
    if(!okToInt)
    {
        strError=QObject::tr("PointCloudFileManager::processProjectFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nNumber of processes is not an integer: %1").arg(strValue);
        fileInput.close();
        return(false);
    }
    QProgressDialog* ptrProgress=NULL;
    if(ptrWidget!=NULL)
    {
        QString title=QObject::tr("PointCloudFileManager::processProjectFile");
        QString msgGlobal=QObject::tr("Number of processes: %1").arg(QString::number(numberOfProcesses));
        ptrProgress=new QProgressDialog(title, "Abort",0,numberOfProcesses, ptrWidget);
        ptrProgress->setWindowModality(Qt::WindowModal);
        ptrProgress->setLabelText(msgGlobal);
        ptrProgress->show();
        qApp->processEvents();
    }
    for(int np=0;np<numberOfProcesses;np++)
    {
        // Reading process type
        nline++;
        strLine=in.readLine();
        strLine=strLine.trimmed();
        strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        if(strList.size()!=2)
        {
            strError=QObject::tr("PointCloudFileManager::processProjectFile");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
            fileInput.close();
            if(ptrWidget!=NULL)
            {
                ptrProgress->setValue(numberOfProcesses);
                qApp->processEvents();
                ptrProgress->close();
                delete(ptrProgress);
            }
            return(false);
        }
        QString processType=strList.at(1).trimmed();
        if(ptrWidget!=NULL)
        {
            ptrProgress->setValue(np+1);
            qApp->processEvents();
        }
        if(processType.compare(POINTCLOUDFILE_PROCESS_CREATE_POINT_CLOUD_FILE_TAG)==0)
        {
            if(!createPointCloudFile(fileName,in,strAuxError))
            {
                strError=QObject::tr("PointCloudFileManager::processProjectFile");
                strError+=QObject::tr("\nError reading file: %1").arg(fileName);
                strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
                strError+=QObject::tr("\nError in process create point cloud file:\n%1").arg(strAuxError);
                fileInput.close();
                if(ptrWidget!=NULL)
                {
                    ptrProgress->setValue(numberOfProcesses);
                    qApp->processEvents();
                    ptrProgress->close();
                    delete(ptrProgress);
                }
                return(false);
            }
        }
        else if(processType.compare(POINTCLOUDFILE_PROCESS_ADD_POINT_CLOUD_FILE_TAG)==0)
        {
            if(!addPointCloudFile(fileName,in,strAuxError))
            {
                strError=QObject::tr("PointCloudFileManager::processProjectFile");
                strError+=QObject::tr("\nError reading file: %1").arg(fileName);
                strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
                strError+=QObject::tr("\nError adding point cloud file:\n%1").arg(strAuxError);
                fileInput.close();
                if(ptrWidget!=NULL)
                {
                    ptrProgress->setValue(numberOfProcesses);
                    qApp->processEvents();
                    ptrProgress->close();
                    delete(ptrProgress);
                }
                return(false);
            }
        }
        else if(processType.compare(POINTCLOUDFILE_PROCESS_ADD_POINT_CLOUD_FILES_TAG)==0)
        {
            if(!addPointCloudFiles(fileName,in,strAuxError))
            {
                strError=QObject::tr("PointCloudFileManager::processProjectFile");
                strError+=QObject::tr("\nError reading file: %1").arg(fileName);
                strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
                strError+=QObject::tr("\nError adding point cloud file:\n%1").arg(strAuxError);
                fileInput.close();
                if(ptrWidget!=NULL)
                {
                    ptrProgress->setValue(numberOfProcesses);
                    qApp->processEvents();
                    ptrProgress->close();
                    delete(ptrProgress);
                }
                return(false);
            }
        }
        else if(processType.compare(POINTCLOUDFILE_PROCESS_ADD_ROI_TAG)==0)
        {
            if(!addROI(fileName,in,strAuxError))
            {
                strError=QObject::tr("PointCloudFileManager::processProjectFile");
                strError+=QObject::tr("\nError reading file: %1").arg(fileName);
                strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
                strError+=QObject::tr("\nError adding ROI:\n%1").arg(strAuxError);
                fileInput.close();
                if(ptrWidget!=NULL)
                {
                    ptrProgress->setValue(numberOfProcesses);
                    qApp->processEvents();
                    ptrProgress->close();
                    delete(ptrProgress);
                }
                return(false);
            }
        }
        else if(processType.compare(POINTCLOUDFILE_PROCESS_WRITE_PCFS_TAG)==0)
        {
            if(!writePointCloudFile(fileName,in,strAuxError))
            {
                strError=QObject::tr("PointCloudFileManager::processProjectFile");
                strError+=QObject::tr("\nError reading file: %1").arg(fileName);
                strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
                strError+=QObject::tr("\nError in process create database:\n%1").arg(strAuxError);
                fileInput.close();
                if(ptrWidget!=NULL)
                {
                    ptrProgress->setValue(numberOfProcesses);
                    qApp->processEvents();
                    ptrProgress->close();
                    delete(ptrProgress);
                }
                return(false);
            }
        }
        else if(processType.compare(POINTCLOUDFILE_PROCESS_PROCESS_RECLASSIFICATION_CONFUSION_MATRIX_REPORT)==0)
        {
            if(!processReclassificationConfusionMatrixReport(fileName,in,strAuxError))
            {
                strError=QObject::tr("PointCloudFileManager::processProjectFile");
                strError+=QObject::tr("\nError reading file: %1").arg(fileName);
                strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
                strError+=QObject::tr("\nError in process create database:\n%1").arg(strAuxError);
                fileInput.close();
                if(ptrWidget!=NULL)
                {
                    ptrProgress->setValue(numberOfProcesses);
                    qApp->processEvents();
                    ptrProgress->close();
                    delete(ptrProgress);
                }
                return(false);
            }
        }
    }
    if(ptrWidget!=NULL)
    {
        ptrProgress->setValue(numberOfProcesses);
        qApp->processEvents();
        ptrProgress->close();
        delete(ptrProgress);
    }
    fileInput.close();
    return(true);
}

bool PointCloudFileManager::processReclassificationConfusionMatrixReport(QString fileName,
                                                                         QTextStream &in,
                                                                         QString &strError)
{
    QString pcPath,dbCrsDescription,dbCrsProj4String;
    QString outputFileName;
    int dbCrsEpsgCode;

    int intValue,nline=0;
    double dblValue;
    bool okToInt,okToDouble;
    QString strLine,strValue;
    QStringList strList;
    QStringList strAuxList;
    QDir currentDir=QDir::current();

    QString msg,strAuxError;

    // Reading database file name
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReportFromSpatialiteDb");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    pcPath=strList.at(1).trimmed();
    if(pcPath.startsWith("\"")&&pcPath.endsWith("\""))
    {
        pcPath=pcPath.remove("\"");
    }
    if(!currentDir.exists(pcPath))
    {
//        QFile::remove(dbFileName);
        strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReportFromSpatialiteDb");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nNot exists point cloud path:\n%1\nYou must remove it before").arg(pcPath);
        return(false);
    }
    if(!openPointCloudFile(pcPath,
                           strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReportFromSpatialiteDb");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError setting point cloud from path:\n%1\nError:\n%2")
                .arg(pcPath).arg(strAuxError);
        return(false);
    }
    dbCrsDescription=mPtrPcFiles[pcPath]->getCrsDescription();
    dbCrsEpsgCode=mPtrPcFiles[pcPath]->getCrsEpsgCode();
    if(!mPtrCrsTools->appendUserCrs(dbCrsEpsgCode,
                                    dbCrsProj4String,
                                    strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReportFromSpatialiteDb");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError adding CRS from PROJ4:\n%1\nfrom paht:\n%2\nError:\n%3")
                .arg(dbCrsProj4String).arg(pcPath).arg(strAuxError);
        return(false);
    }

//    dbCrsProj4String=mPtrPcSpDbs[dbFileName]->getCrsProj4String();
//    if(!mPtrCrsTools->appendUserCrs(dbCrsProj4String,
//                                    dbCrsDescription,
//                                    strAuxError))
//    {
//        strError=QObject::tr("PointCloudDbManager::addROI");
//        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
//        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
//        strError+=QObject::tr("\nError adding CRS from PROJ4:\n%1\nfrom file:\n%2\nError:\n%3")
//                .arg(dbCrsProj4String).arg(dbFileName).arg(strAuxError);
//        return(false);
//    }

    // Reading outputFileName
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReportFromSpatialiteDb");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    outputFileName=strList.at(1).trimmed();
    if(outputFileName.startsWith("\"")&&outputFileName.endsWith("\""))
    {
        outputFileName=outputFileName.remove("\"");
    }
    if(QFile::exists(outputFileName))
    {
        if(!QFile::remove(outputFileName))
        {
            strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReportFromSpatialiteDb");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nError removing existing output file:\n %1").arg(outputFileName);
            return(false);
        }
    }

    // Reading classes
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()>2)
    {
        strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReportFromSpatialiteDb");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not one or two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    QStringList strClasses=strList.at(1).trimmed().split(";");
    QVector<int> selectedClasses;
    for(int i=0;i<strClasses.size();i++)
    {
        bool okToInt=false;
        int classValue=strClasses.at(i).toInt(&okToInt);
        if(!okToInt)
        {
            strError=QObject::tr("PointCloudFileManager::processReclassificationConfusionMatrixReportFromSpatialiteDb");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nNo integer value for class: %1").arg(strClasses.at(i));
            return(false);
        }
        if(!selectedClasses.contains(classValue))
        {
            selectedClasses.push_back(classValue);
        }
    }
    if(!mPtrPcFiles[pcPath]->processReclassificationConfusionMatrixReport(outputFileName,
                                                                          selectedClasses,
                                                                          strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::writePointCloudFilesFromSpatialiteDb");
        strError+=QObject::tr("\nError in process for point cloud path:\n%1\nError:\n%2")
                .arg(pcPath).arg(strAuxError);
        return(false);
    }
    return(true);
}

bool PointCloudFileManager::readVegetationGrowthModel(QString &fileName,
                                                      QMap<int, QVector<double> > &vegetationGrowthModel,
                                                      QString &strError)
{
    vegetationGrowthModel.clear();
    if(fileName.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
        strError+=QObject::tr("\nInput file name is empty");
        return(false);
    }
    if(!QFile::exists(fileName))
    {
        strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
        strError+=QObject::tr("\nNot exists input file: \n%1").arg(fileName);
        return(false);
    }
    QFile fileInput(fileName);
    if (!fileInput.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
        strError+=QObject::tr("\nError opening file: \n%1").arg(fileName);
        return(false);
    }
    QFileInfo fileInputInfo(fileName);
    QString projectFileBaseName=fileInputInfo.completeBaseName();

    QTextStream in(&fileInput);
    QVector<int> stretchs;
    stretchs.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_1_STRETCH_UPPER);
    stretchs.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_2_STRETCH_UPPER);
    stretchs.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_3_STRETCH_UPPER);
    stretchs.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_4_STRETCH_UPPER);
    stretchs.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_5_STRETCH_UPPER);
    stretchs.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_6_STRETCH_UPPER);

    int intValue,nline=0;
    double dblValue;
    bool okToInt,okToDouble;
    QString strLine,strValue;
    QStringList strList;
    QStringList strAuxList;
    QDir currentDir=QDir::current();

    QString msg;

    // Ignore header
    nline++;
    strLine=in.readLine();

    for(int ns=0;ns<stretchs.size();ns++)
    {
        // Reading stretch, integer
        nline++;
        strLine=in.readLine();
        strLine=strLine.trimmed();
        strList=strLine.split(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR);
        if(strList.size()!=2)
        {
            strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR);
            fileInput.close();
            return(false);
        }
        strValue=strList.at(1).trimmed();
        okToInt=false;
        int stretch=strValue.toInt(&okToInt);
        if(!okToInt)
        {
            strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nStretch is not an integer: %1").arg(strValue);
            fileInput.close();
            return(false);
        }

        // Reading number of values
        nline++;
        strLine=in.readLine();
        strLine=strLine.trimmed();
        strList=strLine.split(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR);
        if(strList.size()!=2)
        {
            strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR);
            fileInput.close();
            return(false);
        }
        strValue=strList.at(1).trimmed();
        okToInt=false;
        int numberOfValues=strValue.toInt(&okToInt);
        if(!okToInt)
        {
            strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nNumber of values is not an integer: %1").arg(strValue);
            fileInput.close();
            return(false);
        }
        if(numberOfValues==0)
        {
            QVector<double> values(5);
            vegetationGrowthModel[stretch]=values;
            continue;
        }

        // Reading mean value
        nline++;
        strLine=in.readLine();
        strLine=strLine.trimmed();
        strList=strLine.split(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR);
        if(strList.size()!=2)
        {
            strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR);
            fileInput.close();
            return(false);
        }
        strValue=strList.at(1).trimmed();
        okToDouble=false;
        double meanValue=strValue.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nMean value is not a double: %1").arg(strValue);
            fileInput.close();
            return(false);
        }

        // Reading std value
        nline++;
        strLine=in.readLine();
        strLine=strLine.trimmed();
        strList=strLine.split(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR);
        if(strList.size()!=2)
        {
            strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR);
            fileInput.close();
            return(false);
        }
        strValue=strList.at(1).trimmed();
        okToDouble=false;
        double stdValue=strValue.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nStandard deviation value is not a double: %1").arg(strValue);
            fileInput.close();
            return(false);
        }

        // Reading percentile 05% value
        nline++;
        strLine=in.readLine();
        strLine=strLine.trimmed();
        strList=strLine.split(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR);
        if(strList.size()!=2)
        {
            strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR);
            fileInput.close();
            return(false);
        }
        strValue=strList.at(1).trimmed();
        okToDouble=false;
        double percentile05Value=strValue.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nPercentil 95% value is not a double: %1").arg(strValue);
            fileInput.close();
            return(false);
        }

        // Reading percentile 95% value
        nline++;
        strLine=in.readLine();
        strLine=strLine.trimmed();
        strList=strLine.split(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR);
        if(strList.size()!=2)
        {
            strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_MODEL_FILE_STRING_SEPARATOR);
            fileInput.close();
            return(false);
        }
        strValue=strList.at(1).trimmed();
        okToDouble=false;
        double percentile95Value=strValue.toDouble(&okToDouble);
        if(!okToDouble)
        {
            strError=QObject::tr("PointCloudFileManager::readVegetationGrowthModel");
            strError+=QObject::tr("\nError reading file: %1").arg(fileName);
            strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
            strError+=QObject::tr("\nPercentil 95% value is not a double: %1").arg(strValue);
            fileInput.close();
            return(false);
        }

        QVector<double> values(5);
        values[0]=numberOfValues;
        values[1]=meanValue;
        values[2]=stdValue;
        values[3]=percentile05Value;
        values[4]=percentile95Value;
        vegetationGrowthModel[stretch]=values;
    }
    fileInput.close();
    return(true);
}

bool PointCloudFileManager::selectLastoolsCommandParameters(QString lastoolscommand,
                                                            QString &strError)
{
    QString strAuxError;
    if(!mLastoolsCommands.contains(lastoolscommand))
    {
        strError=QObject::tr("PointCloudFileManager::selectLastoolsCommandParameters");
        strError+=QObject::tr("\nInvalid lastools command: %1").arg(lastoolscommand);
        return(false);
    }
    if(mLastoolsCommandsParametersFileName.isEmpty())
    {
        strError=QObject::tr("PointCloudFileManager::selectLastoolsCommandParameters");
        strError+=QObject::tr("\nLastools parameters file name is empty");
        return(false);
    }
    if(!QFile::exists(mLastoolsCommandsParametersFileName))
    {
        strError=QObject::tr("PointCloudFileManager::selectLastoolsCommandParameters");
        strError+=QObject::tr("\nNot exists Lastools parameters file name:\n%1")
                .arg(mLastoolsCommandsParametersFileName);
        return(false);
    }
    if(mPtrLastoolsCommandsParameters==NULL)
    {
        ParametersManager* ptrParametersManager=new ParametersManager();
        if(!ptrParametersManager->loadFromXml(mLastoolsCommandsParametersFileName,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::selectLastoolsCommandParameters");
            strError+=QObject::tr("\nError loading parameters manager from file:\n%1\nError:\n%2")
                    .arg(mLastoolsCommandsParametersFileName).arg(strAuxError);
            delete(ptrParametersManager);
            return(false);
        }
        mPtrLastoolsCommandsParameters=ptrParametersManager;
    }
    ParametersManagerDialog parameterDialog(mPtrLastoolsCommandsParameters,
                                            lastoolscommand);
    return(true);
}

bool PointCloudFileManager::selectProjectParameters(QString projectType,
                                                    QString &strError)
{
    QString strAuxError;
    if(!mProjectTypes.contains(projectType))
    {
        strError=QObject::tr("PointCloudFileManager::selectProjectParameters");
        strError+=QObject::tr("\nInvalid project type: %1").arg(projectType);
        return(false);
    }
    QString projectParametersFileName=mProjectParametersFileByProjectType[projectType];
    if(!QFile::exists(projectParametersFileName))
    {
        strError=QObject::tr("PointCloudFileManager::selectProjectParameters");
        strError+=QObject::tr("\nNot exist parameters file:\n%1").arg(projectParametersFileName);
        return(false);
    }
    if(mPtrProjectsParametersManagerByProjectType[projectType]==NULL)
    {
        ParametersManager* ptrParametersManager=new ParametersManager();
        if(!ptrParametersManager->loadFromXml(projectParametersFileName,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::selectProjectParameters");
            strError+=QObject::tr("\nError loading parameters manager from file:\n%1\nError:\n%2")
                    .arg(projectParametersFileName).arg(strAuxError);
            delete(ptrParametersManager);
            return(false);
        }
        mPtrProjectsParametersManagerByProjectType[projectType]=ptrParametersManager;
    }
    ParametersManagerDialog parameterDialog(mPtrProjectsParametersManagerByProjectType[projectType],
                                            POINTCLOUDFILE_PROJECT_PARAMETERS_TAG);
    return(true);
}

bool PointCloudFileManager::setBasePath(QString basePath,
                                        QString& strError)
{
    mBasePath=basePath;
    QString generic=QString::fromLatin1(POINTCLOUDFILE_PROJECT_TYPE_GENERIC_TAG).toLower();
    QString powerline=QString::fromLatin1(POINTCLOUDFILE_PROJECT_TYPE_POWERLINE_TAG).toLower();
    QString solarpark=QString::fromLatin1(POINTCLOUDFILE_PROJECT_TYPE_SOLARPARK_TAG).toLower();
    mProjectParametersFileByProjectType[generic]=mBasePath+"/"+POINTCLOUDFILE_PROJECT_TYPE_GENERIC_PARAMETERS_FILE_NAME;
    mProjectParametersFileByProjectType[powerline]=mBasePath+"/"+POINTCLOUDFILE_PROJECT_TYPE_POWERLINE_PARAMETERS_FILE_NAME;
    mProjectParametersFileByProjectType[solarpark]=mBasePath+"/"+POINTCLOUDFILE_PROJECT_TYPE_SOLARPARK_PARAMETERS_FILE_NAME;
    mLastoolsCommandsParametersFileName=mBasePath+"/"+POINTCLOUDFILE_LASTOOLS_PARAMETERS_FILE_NAME;
    mLastoolsCommands.clear();
    mLastoolsCommands.push_back(POINTCLOUDFILE_LASTOOLS_COMMAND_LASBOUNDARY);
    mLastoolsCommands.push_back(POINTCLOUDFILE_LASTOOLS_COMMAND_LASCLIP);
    mLastoolsCommands.push_back(POINTCLOUDFILE_LASTOOLS_COMMAND_LASHEIGHT);
    mLastoolsCommands.push_back(POINTCLOUDFILE_LASTOOLS_COMMAND_LASMERGE);
    mLastoolsCommands.push_back(POINTCLOUDFILE_LASTOOLS_COMMAND_LASTILE);
    mLastoolsCommands.push_back(POINTCLOUDFILE_LASTOOLS_COMMAND_LAS2DEM);
//    mLastoolsCommands.push_back(POINTCLOUDFILE_LASTOOLS_COMMAND_POWERLINE_PREPROCESSING);
//    mLastoolsCommands.push_back(POINTCLOUDFILE_LASTOOLS_COMMAND_SOLARPARK_PREPROCESSING);
    mLastoolsCommands.push_back(POINTCLOUDFILE_LASTOOLS_COMMAND_GROUND_FROM_PHOTOGRAMMETRY);
    mLastoolsCommands.push_back(POINTCLOUDFILE_LASTOOLS_COMMAND_BUILDING_HIGHVEGETATION);
    mLastoolsCommands.push_back(POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_PREPROCESSING);
    mInternalCommandsParametersFileName=mBasePath+"/"+POINTCLOUDFILE_INTERNALTOOLS_PARAMETERS_FILE_NAME;
    mInternalCommands.clear();
    mInternalCommands.push_back(POINTCLOUDFILE_INTERNALTOOLS_COMMAND_VEGETATION_GROWTH_ESTIMATE);
//    mInternalCommands.push_back();
//    mInternalCommands.push_back();
//    mInternalCommands.push_back();
    if(mTempPath.isEmpty())
    {
        QString tempPath=mBasePath+POINTCLOUDFILE_PROCESS_DEFAULT_TEMP_PATH;
        QString strAuxError;
        if(!setTempPath(tempPath,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::setBasePath");
            strError+=QObject::tr("\nError setting temporal path:\n%1\nError:\n%2")
                    .arg(tempPath).arg(strAuxError);
            return(false);
        }
    }
    if(mGeoidFilessLastoolsPath.isEmpty())
    {
        QString geoidsLastoolsPath=mBasePath+POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_PATH;
        QString strAuxError;
        if(!setGeoidFilesLastoolsPath(geoidsLastoolsPath,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::setBasePath");
            strError+=QObject::tr("\nError setting temporal path:\n%1\nError:\n%2")
                    .arg(geoidsLastoolsPath).arg(strAuxError);
            return(false);
        }
    }
    return(true);
}

bool PointCloudFileManager::setGeoidFilesLastoolsPath(QString value,
                                                QString &strError)
{
    QDir currentDir=QDir::currentPath();
    if(!currentDir.exists(value))
    {
        strError=QObject::tr("PointCloudFileManager::setGeoidsLastoolsPath");
        strError+=QObject::tr("\nNot exists path:\n%1").arg(value);
        return(false);
    }
    QVector<QString> files;
    files.push_back(value+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_FILE_SPAIN_EGM08REDNAD_25830);
    files.push_back(value+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_FILE_SPAIN_EGM08REDNAD_25829);
    files.push_back(value+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_FILE_SPAIN_EGM08REDNAD_25831);
    files.push_back(value+"/"+POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_FILE_SPAIN_EGM08REDNAD_CANARIAS_4083);
    for(int nf=0;nf<files.size();nf++)
    {
        QString fileName=files[nf];
        if(!QFile::exists(fileName))
        {
            strError=QObject::tr("PointCloudFileManager::setGeoidsLastoolsPath");
            strError+=QObject::tr("\nNot exists file:\n%1").arg(fileName);
            return(false);
        }
    }
    if(!QFile::exists(mLastoolsCommandsParametersFileName))
    {
        strError=QObject::tr("PointCloudFileManager::setGeoidsLastoolsPath");
        strError+=QObject::tr("\nNot exists Lastools parameters file name:\n%1")
                .arg(mLastoolsCommandsParametersFileName);
        return(false);
    }
    QString strAuxError;
    if(mPtrLastoolsCommandsParameters==NULL)
    {
        ParametersManager* ptrParametersManager=new ParametersManager();
        if(!ptrParametersManager->loadFromXml(mLastoolsCommandsParametersFileName,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::setGeoidsLastoolsPath");
            strError+=QObject::tr("\nError loading parameters manager from file:\n%1\nError:\n%2")
                    .arg(mLastoolsCommandsParametersFileName).arg(strAuxError);
            delete(ptrParametersManager);
            return(false);
        }
        mPtrLastoolsCommandsParameters=ptrParametersManager;
        QString parameterCode=POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOID_FILE;
        Parameter* ptrParameter=ptrParametersManager->getParameter(parameterCode);
        QString strValue;
        ptrParameter->getStrValue(strValue);
        bool validGeoidFile=false;
        QString fileNameWithPath=value+"/";
        if(strValue.contains(POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_FILE_SPAIN_EGM08REDNAD_25830))
        {
            fileNameWithPath+=POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_FILE_SPAIN_EGM08REDNAD_25830;
            validGeoidFile=true;
        }
        if(strValue.contains(POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_FILE_SPAIN_EGM08REDNAD_25831))
        {
            fileNameWithPath+=POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_FILE_SPAIN_EGM08REDNAD_25831;
            validGeoidFile=true;
        }
        if(strValue.contains(POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_FILE_SPAIN_EGM08REDNAD_25829))
        {
            fileNameWithPath+=POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_FILE_SPAIN_EGM08REDNAD_25829;
            validGeoidFile=true;
        }
        if(strValue.contains(POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_FILE_SPAIN_EGM08REDNAD_CANARIAS_4083))
        {
            fileNameWithPath+=POINTCLOUDFILE_LASTOOLS_COMMAND_E2OHC_GEOIDS_FILE_SPAIN_EGM08REDNAD_CANARIAS_4083;
            validGeoidFile=true;
        }
        if(!validGeoidFile)
        {
            strError=QObject::tr("PointCloudFileManager::setGeoidsLastoolsPath");
            strError+=QObject::tr("\nInvalid file:\n%1\nfor parameter code: %2\nin parameters file:\n%3")
                    .arg(strValue).arg(parameterCode).arg(mLastoolsCommandsParametersFileName);
            delete(ptrParametersManager);
            mPtrLastoolsCommandsParameters=NULL;
            return(false);
        }
        ptrParameter->getStrValue(strValue);
        ptrParameter->setValue(fileNameWithPath);
        ptrParameter->getStrValue(strValue);
        if(!ptrParametersManager->saveAsXml(NULL))
        {
            strError=QObject::tr("PointCloudFileManager::setGeoidsLastoolsPath");
            strError+=QObject::tr("\nError savingg parameters manager to file:\n%1\nError:\n%2")
                    .arg(mLastoolsCommandsParametersFileName).arg(strAuxError);
            delete(ptrParametersManager);
            mPtrLastoolsCommandsParameters=NULL;
            return(false);
        }
        ptrParameter->getStrValue(strValue);
    }

//    QString strAuxError;
//    QMap<QString,PointCloudDb::PointCloudSpatialiteDb*>::iterator iter=mPtrPcSpDbs.begin();
//    while(iter!=mPtrPcSpDbs.end())
//    {
//        if(!iter.value()->setTempPath(value,strAuxError))
//        {
//            strError=QObject::tr("PointCloudDbManager::setTempPath");
//            strError+=QObject::tr("\nError setting temporal path to database:\n: %1")
//                    .arg(strAuxError);
//            return(false);
//        }
//        iter++;
//    }
    mGeoidFilessLastoolsPath=value;
    return(true);
}

bool PointCloudFileManager::setControlROIs(ControlROIs *ptrControlROIs,
                                           QString &strError)
{
    mPtrControlROIs=ptrControlROIs;
    return(true);
}

bool PointCloudFileManager::setLastoolsPath(QString path,
                                            QString &strError)
{
    QDir currentDir=QDir::currentPath();
    if(!currentDir.exists(path))
    {
        strError=QObject::tr("PointCloudFileManager::setLastoolsPath");
        strError+=QObject::tr("\nNot exists lastools path:\n%1").arg(path);
        return(false);
    }
    mLastoolsPath=path;
    return(true);
}

bool PointCloudFileManager::setMultiProcess(bool useMultiProcess,
                                            QString &strError)
{
    mUseMultiProcess=useMultiProcess;
    return(true);
}

bool PointCloudFileManager::setOutputPath(QString value,
                                          QString &strError)
{
    QDir currentDir=QDir::currentPath();
    if(!currentDir.exists(value))
    {
        if(!currentDir.mkpath(value))
        {
            strError=QObject::tr("PointCloudFileManager::setOutputPath");
            strError+=QObject::tr("\nError making path:\n%1").arg(value);
            return(false);
        }
    }
    QString strAuxError;
    QMap<QString,PCFile::PointCloudFile*>::iterator iter=mPtrPcFiles.begin();
    while(iter!=mPtrPcFiles.end())
    {
        if(!iter.value()->setOutputPath(value,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::setOutputPath");
            strError+=QObject::tr("\nError setting temporal path to database:\n: %1")
                    .arg(strAuxError);
            return(false);
        }
        iter++;
    }
    mOutputPath=value;
    return(true);
}

bool PointCloudFileManager::setProjectsParametersManager(QString projectType,
                                                       QString &strError)
{
    QString strAuxError;
    if(!mProjectTypes.contains(projectType))
    {
        strError=QObject::tr("PointCloudFileManager::setProjectsParametersManager");
        strError+=QObject::tr("\nInvalid project type: %1").arg(projectType);
        return(false);
    }
    if(mPtrProjectsParametersManagerByProjectType[projectType]!=NULL)
    {
        return(true);
    }
    QString projectParametersFileName=mProjectParametersFileByProjectType[projectType];
    if(!QFile::exists(projectParametersFileName))
    {
        strError=QObject::tr("PointCloudFileManager::setProjectsParametersManager");
        strError+=QObject::tr("\nNot exist parameters file:\n%1").arg(projectParametersFileName);
        return(false);
    }

    ParametersManager* ptrParametersManager=new ParametersManager();
    if(!ptrParametersManager->loadFromXml(projectParametersFileName,strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::setProjectsParametersManager");
        strError+=QObject::tr("\nError loading parameters manager from file:\n%1").arg(projectParametersFileName);
        delete(ptrParametersManager);
        return(false);
    }
    mPtrProjectsParametersManagerByProjectType[projectType]=ptrParametersManager;
    return(true);
}

void PointCloudFileManager::setProjectTypes()
{
    if(!mProjectTypes.isEmpty())
    {
        return;
    }
    QString generic=QString::fromLatin1(POINTCLOUDFILE_PROJECT_TYPE_GENERIC_TAG).toLower();
//    QString powerline=QString::fromLatin1(POINTCLOUDFILE_PROJECT_TYPE_POWERLINE_TAG).toLower();
//    QString solarpark=QString::fromLatin1(POINTCLOUDFILE_PROJECT_TYPE_SOLARPARK_TAG).toLower();
    mProjectTypes.push_back(generic);
//    mProjectTypes.push_back(powerline);
//    mProjectTypes.push_back(solarpark);
    mProjectParametersFileByProjectType[generic]=mBasePath+"/"+POINTCLOUDFILE_PROJECT_TYPE_GENERIC_PARAMETERS_FILE_NAME;
//    mProjectParametersFileByProjectType[powerline]=mBasePath+"/"+POINTCLOUDFILE_PROJECT_TYPE_POWERLINE_PARAMETERS_FILE_NAME;
//    mProjectParametersFileByProjectType[solarpark]=mBasePath+"/"+POINTCLOUDFILE_PROJECT_TYPE_SOLARPARK_PARAMETERS_FILE_NAME;
    mPtrProjectsParametersManagerByProjectType[generic]=NULL;
//    mPtrProjectsParametersManagerByProjectType[powerline]=NULL;
//    mPtrProjectsParametersManagerByProjectType[solarpark]=NULL;
}

bool PointCloudFileManager::setTempPath(QString value,
                                        QString &strError)
{
    QDir currentDir=QDir::currentPath();
    if(!currentDir.exists(value))
    {
        if(!currentDir.mkpath(value))
        {
            strError=QObject::tr("PointCloudFileManager::setTempPath");
            strError+=QObject::tr("\nError making path:\n%1").arg(value);
            return(false);
        }
    }
    QString strAuxError;
    QMap<QString,PCFile::PointCloudFile*>::iterator iter=mPtrPcFiles.begin();
    while(iter!=mPtrPcFiles.end())
    {
        if(!iter.value()->setTempPath(value,strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::setTempPath");
            strError+=QObject::tr("\nError setting temporal path to database:\n: %1")
                    .arg(strAuxError);
            return(false);
        }
        iter++;
    }
    mTempPath=value;
    return(true);
}

bool PointCloudFileManager::updateNotEdited2dToolsPoints(QString pcfPath,
                                                         QMap<int, QMap<int, QVector<int> > > &pointFileIdByTile,
                                                         QMap<int, QMap<int, QVector<int> > > &pointPositionByTile,
                                                         QMap<int, QMap<int, QVector<quint8> > > &pointClassNewByTile,
                                                         QMap<int, QMap<int, QVector<quint8> > > &pointClassByTile,
                                                         QString &strError)
{
    QString strAuxError;
    if(!mPtrPcFiles.contains(pcfPath))
    {
        if(!openPointCloudFile(pcfPath,
                               strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::updatePoints");
            strError+=QObject::tr("\nError openning spatialite:\n%1\nError:\n%2")
                    .arg(pcfPath).arg(strAuxError);
            return(false);
        }
    }
    return(mPtrPcFiles[pcfPath]->updateNotEdited2dToolsPoints(pcfPath,
                                              pointFileIdByTile,
                                              pointPositionByTile,
                                              pointClassNewByTile,
                                              pointClassByTile,
                                              strError));
}

bool PointCloudFileManager::updatePoints(QString pcfPath,
                                         QString strAction,
                                         quint8 classValue,
                                         QMap<int, QMap<int, QVector<int> > > &pointFileIdByTile,
                                         QMap<int, QMap<int, QVector<int> > > &pointPositionByTile,
//                                         QMap<int, QMap<int, QVector<quint8> > > &pointClassNewByTile,
//                                         QMap<int, QMap<int, QVector<quint8> > > &pointClassByTile,
                                         QMap<quint8, bool> &lockedClasses,
                                         QString &strError)
{
    QString strAuxError;
    if(!mPtrPcFiles.contains(pcfPath))
    {
        if(!openPointCloudFile(pcfPath,
                               strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::updatePoints");
            strError+=QObject::tr("\nError openning spatialite:\n%1\nError:\n%2")
                    .arg(pcfPath).arg(strAuxError);
            return(false);
        }
    }
    return(mPtrPcFiles[pcfPath]->updatePoints(strAction,
                                              classValue,
                                              pointFileIdByTile,
                                              pointPositionByTile,
//                                              pointClassNewByTile,
//                                              pointClassByTile,
                                              lockedClasses,
                                              strError));
}

bool PointCloudFileManager::validateProjectParametersString(QString projectType,
                                                            QString projectParametersString,
                                                            QString &strError)
{
    QString strAuxError;
    if(!mProjectTypes.contains(projectType))
    {
        strError=QObject::tr("PointCloudFileManager::validateProjectParametersString");
        strError+=QObject::tr("\nInvalid project type: %1").arg(projectType);
        return(false);
    }
    if(mPtrProjectsParametersManagerByProjectType[projectType]==NULL)
    {
        if(!setProjectsParametersManager(projectType,
                                         strAuxError))
        {
            strError=QObject::tr("PointCloudFileManager::validateProjectParametersString");
            strError+=QObject::tr("\nError setting project parameters manager for project type: %1\nError:\n%2")
                    .arg(projectType).arg(strAuxError);
            return(false);
        }
    }
    QStringList projectParametersList=projectParametersString.split(POINTCLOUDFILE_PROJECT_PARAMETERS_FILE_PARAMETERS_STRING_SEPARATOR);
    for(int np=0;np<projectParametersList.size();np++)
    {
        QString projectParameterString=projectParametersList.at(np);
        QStringList projectParameterList=projectParameterString.split(POINTCLOUDFILE_PROJECT_PARAMETERS_FILE_PARAMETER_VALUE_STRING_SEPARATOR);
        if(projectParameterList.size()!=2)
        {
            strError=QObject::tr("PointCloudFileManager::validateProjectParametersString");
            strError+=QObject::tr("\nInvalid parameter: %1 \nfor project type: %2")
                    .arg(projectParameterString).arg(projectType);
            return(false);
        }
        QString parameterCode=projectParameterList.at(0);
        QString parameterValue=projectParameterList.at(1);
        Parameter* ptrParameter=mPtrProjectsParametersManagerByProjectType[projectType]->getParameter(parameterCode);
        if(ptrParameter==NULL)
        {
            strError=QObject::tr("PointCloudFileManager::validateProjectParametersString");
            strError+=QObject::tr("\nInvalid parameter: %1 \nfor project type: %2")
                    .arg(projectParameterString).arg(projectType);
            return(false);
        }
    }
    return(true);
}

bool PointCloudFileManager::writePointCloudFile(QString fileName,
                                                QTextStream &in,
                                                QString &strError)
{
    QString pcPath,dbCrsDescription,dbCrsProj4String;
    QString suffix,outputPath;
    int dbCrsEpsgCode;

    int intValue,nline=0;
    double dblValue;
    bool okToInt,okToDouble;
    QString strLine,strValue;
    QStringList strList;
    QStringList strAuxList;
    QDir currentDir=QDir::current();

    QString msg,strAuxError;

    // Reading database file name
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudFileManager::writePointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    pcPath=strList.at(1).trimmed();
    if(pcPath.startsWith("\"")&&pcPath.endsWith("\""))
    {
        pcPath=pcPath.remove("\"");
    }
    if(!currentDir.exists(pcPath))
    {
        strError=QObject::tr("PointCloudFileManager::writePointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nNot exists point cloud path:\n%1\nYou must create it before").arg(pcPath);
        return(false);
    }
    if(!openPointCloudFile(pcPath,
                           strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::writePointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError setting point cloud from path:\n%1\nError:\n%2")
                .arg(pcPath).arg(strAuxError);
        return(false);
    }
    dbCrsDescription=mPtrPcFiles[pcPath]->getCrsDescription();
    dbCrsEpsgCode=mPtrPcFiles[pcPath]->getCrsEpsgCode();
    if(!mPtrCrsTools->appendUserCrs(dbCrsEpsgCode,
                                    dbCrsProj4String,
                                    strAuxError))
    {
        strError=QObject::tr("PointCloudFileManager::writePointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nError adding CRS from PROJ4:\n%1\nfrom path:\n%2\nError:\n%3")
                .arg(dbCrsProj4String).arg(pcPath).arg(strAuxError);
        return(false);
    }

//    dbCrsProj4String=mPtrPcSpDbs[dbFileName]->getCrsProj4String();
//    if(!mPtrCrsTools->appendUserCrs(dbCrsProj4String,
//                                    dbCrsDescription,
//                                    strAuxError))
//    {
//        strError=QObject::tr("PointCloudDbManager::addROI");
//        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
//        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
//        strError+=QObject::tr("\nError adding CRS from PROJ4:\n%1\nfrom file:\n%2\nError:\n%3")
//                .arg(dbCrsProj4String).arg(dbFileName).arg(strAuxError);
//        return(false);
//    }

    // Reading suffix
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()!=2)
    {
        strError=QObject::tr("PointCloudDbManager::writePointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    suffix=strList.at(1).trimmed();

    // Reading outputPath
    nline++;
    strLine=in.readLine();
    strLine=strLine.trimmed();
    strList=strLine.split(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
    if(strList.size()>2)
    {
        strError=QObject::tr("PointCloudDbManager::writePointCloudFile");
        strError+=QObject::tr("\nError reading file: %1").arg(fileName);
        strError+=QObject::tr("\nError reading line: %1").arg(QString::number(nline));
        strError+=QObject::tr("\nThere are not one or two fields separated by %1").arg(POINTCLOUDFILE_PROJECT_STRING_SEPARATOR);
        return(false);
    }
    if(strList.size()==2)
    {
        outputPath=strList.at(1).trimmed();
        if(outputPath.startsWith("\"")&&outputPath.endsWith("\""))
        {
            outputPath=outputPath.remove("\"");
        }
    }
    if(!mPtrPcFiles[pcPath]->writePointCloudFiles(suffix,
                                                  outputPath,
                                                  strAuxError))
    {
        strError=QObject::tr("PointCloudDbManager::writePointCloudFile");
        strError+=QObject::tr("\nError writting point cloud files for path:\n%1\nError:\n%2")
                .arg(pcPath).arg(strAuxError);
        return(false);
    }
    return(true);
}

void PointCloudFileManager::mpProcessInternalCommandVegetationGrowthEstimate(int nf)
{
    QString strError,strAuxError;
    QString inputFileName=mInputFiles.at(nf);
    std::string stdFileName=inputFileName.toStdString();
    const char* charFileName=stdFileName.c_str();
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(charFileName);
    if (!lasreadopener.active())
    {
        strError=QObject::tr("PointCloudFileManager::mpProcessInternalCommandVegetationGrowthEstimate");
        strError+=QObject::tr("\nError opening file:\n%1").arg(inputFileName);
        mStrErrorMpProgressDialog=strError;
        emit(mPtrMpProgressDialog->canceled());
        return;
    }

    double maxCoordinatesLenght=(pow(2,16)-1.)*mPICVGESpatialResolution;
    LASreader* lasreader = lasreadopener.open();
    LASheader* lasheader = &lasreader->header;
    int numberOfPoints=lasreader->npoints;
    double fileMinX=lasheader->min_x;
    double fileMinY=lasheader->min_y;
    double fileMaxX=lasheader->max_x;
    double fileMaxY=lasheader->max_y;
    QVector<double> boundingBox(4);
    boundingBox[0]=fileMinX;
    boundingBox[1]=fileMinY;
    boundingBox[2]=fileMaxX;
    boundingBox[3]=fileMaxY;
    int pointsStep=POINTCLOUDFILE_NUMBER_OF_POINTS_TO_PROCESS_BY_STEP;
    int numberOfSteps=ceil((double)numberOfPoints/(double)pointsStep);
    int pointPosition=-1;
    int step=0;
    int numberOfProcessedPoints=0;
    int numberOfProcessedPointsInStep=0;
    int numberOfPointsToProcessInStep=numberOfPoints;
    if(POINTCLOUDFILE_NUMBER_OF_POINTS_TO_PROCESS_BY_STEP<numberOfPointsToProcessInStep)
    {
        numberOfPointsToProcessInStep=POINTCLOUDFILE_NUMBER_OF_POINTS_TO_PROCESS_BY_STEP;
    }
    while(lasreader->read_point()&&numberOfPointsToProcessInStep>0)
    {
        U8 pointClass=lasreader->point.get_classification();
        if(pointClass==4||pointClass==5)
        {
            double x=lasreader->point.get_X()*lasheader->x_scale_factor+lasheader->x_offset;
            double y=lasreader->point.get_Y()*lasheader->y_scale_factor+lasheader->y_offset;
            double z=lasreader->point.get_Z()*lasheader->z_scale_factor+lasheader->z_offset;
            quint16 posX=floor((x-fileMinX)/mPICVGESpatialResolution);
            quint16 posY=floor((y-fileMinY)/mPICVGESpatialResolution);
            quint16 height=qRound(z*1000.);
            if(posX>maxCoordinatesLenght
                    ||posY>maxCoordinatesLenght)
            {
                strError=QObject::tr("PointCloudFileManager::mpProcessInternalCommandVegetationGrowthEstimate");
                strError+=QObject::tr("\nIn file:\n%1").arg(inputFileName);
                strError+=QObject::tr("\ncoordinates increments out of 16 bits domain for point: (%1,%2)")
                        .arg(QString::number(x,'f',3)).arg(QString::number(y,'f',3));
                lasreader->close();
                mStrErrorMpProgressDialog=strError;
                emit(mPtrMpProgressDialog->canceled());
                return;
            }
            mMutex.lock();
            if(!mPICVGEMaxHeightsByTileXYByFilePos[nf].contains(posX))
            {
                mPICVGEMaxHeightsByTileXYByFilePos[nf][posX][posY]=height;
            }
            else if(!mPICVGEMaxHeightsByTileXYByFilePos[nf][posX].contains(posY))
            {
                mPICVGEMaxHeightsByTileXYByFilePos[nf][posX][posY]=height;
            }
            else
            {
                if(height>mPICVGEMaxHeightsByTileXYByFilePos[nf][posX][posY])
                {
                    mPICVGEMaxHeightsByTileXYByFilePos[nf][posX][posY]=height;
                }
            }
            mMutex.unlock();
        }
        numberOfProcessedPoints++;
        numberOfProcessedPointsInStep++;
        if(numberOfProcessedPointsInStep==numberOfPointsToProcessInStep)
        {
            step++;
            numberOfPointsToProcessInStep=numberOfPoints-numberOfProcessedPoints;
            if(POINTCLOUDFILE_NUMBER_OF_POINTS_TO_PROCESS_BY_STEP<numberOfPointsToProcessInStep)
            {
                numberOfPointsToProcessInStep=POINTCLOUDFILE_NUMBER_OF_POINTS_TO_PROCESS_BY_STEP;
            }
            numberOfProcessedPointsInStep=0;
            int numberOfPointsToProcess=numberOfPoints-numberOfProcessedPoints;
            mMutex.lock();
            mNumberOfPointsToProcessByFileName[inputFileName]=numberOfPointsToProcess;
//            QString dialogText=QObject::tr("Reading point cloud files");
//            dialogText+=QObject::tr("\nNumber of point cloud files to read:%1").arg(mNumberOfFilesToProcess);
//            dialogText+=QObject::tr("\n... progressing using %1 threads").arg(QThread::idealThreadCount());
//    //                mPtrMpProgressDialog->setWindowTitle(title);
//            QMap<QString,int>::iterator iter=mNumberOfPointsToProcessByFileName.begin();
//            while(iter!=mNumberOfPointsToProcessByFileName.end())
//            {
//                QString auxInputFileName=iter.key();
//                int auxNumberOfPointsToProcess=iter.value();
//                QString strNumberOfPointsToProcess="All";
//                if(auxNumberOfPointsToProcess!=-1)
//                {
//                    strNumberOfPointsToProcess=QString::number(auxNumberOfPointsToProcess);
//                }
//                dialogText+=QObject::tr("\nPoints to process %1 in file: %2")
//                        .arg(strNumberOfPointsToProcess).arg(auxInputFileName);
//                iter++;
//            }
//            mPtrMpProgressDialog->setLabelText(dialogText);
            mMutex.unlock();
        }
    }
    lasreader->close();
    delete lasreader;
    mMutex.lock();
    mPICVGEBoundingBoxesByFilePos[nf]=boundingBox;
    mMutex.unlock();
    return;
}
