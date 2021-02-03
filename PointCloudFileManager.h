#ifndef LIBPOINTCLOUDFILEMANAGER_H
#define LIBPOINTCLOUDFILEMANAGER_H

#include <QVector>
#include <QMap>
#include <QObject>
#include <QTextStream>
#include <QDateTime>
#include <QMutex>
#include <QProgressDialog>

#include "PointCloudFileDefinitions.h"
#include "Point.h"

#include "libPointCloudFileManager_global.h"

//#include "cpl_conv.h"

class ParametersManager;

namespace ProcessTools{
class ProgressExternalProcessDialog;
}

namespace libCRS{
    class CRSTools;
}

namespace PCFile{
class PointCloudFile;
class LIBPOINTCLOUDFILEMANAGERSHARED_EXPORT PointCloudFileManager : public QObject
{
    Q_OBJECT

public:
    inline PointCloudFileManager(QObject *parent = 0)
    {
        mPtrCrsTools=NULL;
        mPtrLastoolsCommandsParameters=NULL;
        mPtrInternalCommandsParameters=NULL;
        mPtrProgressExternalProcessDialog=NULL;
        mPtrMpProgressDialog=NULL;
//        mGridSizes.push_back(POINTCLOUDFILE_PROJECT_GRID_SIZE_1);
        mGridSizes.push_back(POINTCLOUDFILE_PROJECT_GRID_SIZE_5);
        mGridSizes.push_back(POINTCLOUDFILE_PROJECT_GRID_SIZE_10);
        mGridSizes.push_back(POINTCLOUDFILE_PROJECT_GRID_SIZE_20);
        mGridSizes.push_back(POINTCLOUDFILE_PROJECT_GRID_SIZE_50);
//        mGridSizes.push_back(POINTCLOUDFILE_PROJECT_GRID_SIZE_100); // falla
//        mGridSizes.push_back(POINTCLOUDFILE_PROJECT_GRID_SIZE_200);
        mMaxGridSize=mGridSizes[mGridSizes.size()-1];
        mUseMultiProcess=false;
        setProjectTypes();
    };
    static inline PointCloudFileManager * getInstance(void )
    {
        if ( mInstance == 0 ) mInstance = new PointCloudFileManager;
            return mInstance;
    };
    bool addPointCloudFilesToPointCloudFile(QString pcfPath,
                                            int crsEpsgCode,
                                            bool altitudeIsMsl,
                                            QVector<QString> &pointCloudFiles,
                                            QString& strError);
    bool createPointCloudFile(QString pcfPath,
                              QString projectType,
                              double gridSize,
                              int crsEpsgCode,
                              bool altitudeIsMsl,
                              QVector<QString> &roisShapefiles,
                              QString& strError);
    bool exportProcessedPointCloudFiles(QString pcfPath,
                                        QString suffix,
                                        QString outputPath,
                                        QString& strError);
    QString getBasePath(){return(mBasePath);};
    bool getColorNumberOfBytes(QString pcfPath,
                               int& numberOfBytes,
                               QString& strError);
    bool getProjectCrsEpsgCode(QString dbFileName,
                               int& crsEpsgCode,
                               QString& strError);
    bool getGridSizes(QVector<int> &gridSizes,
                      QString& strError);
    bool getInternalCommands(QVector<QString>& internalCommands,
                             QString& strError);
    bool getLastoolsCommands(QVector<QString>& lastoolsCommands,
                             QString& strError);
    bool getInternalCommandOutputDataFormat(QString& command,
                                            bool& enableOuputPath,
                                            bool& enableOutputFile,
                                            bool& enableSuffix,
                                            bool& enablePrefix,
                                            QString& strError);
    bool getLastoolsCommandsOutputDataFormat(QString& command,
                                             bool& enableOuputPath,
                                             bool& enableOutputFile,
                                             bool& enableSuffix,
                                             bool& enablePrefix,
                                             QString& strError);
    bool getLastoolsCommandStrings(QString& command,
                                   QVector<QString>& inputFiles,
                                   QString& outputPath,
                                   QString& outputFile,
                                   QString& suffix,
                                   QString& prefix,
                                   QVector<QString> &lastoolsCommandStrings,
                                   QString& strError);
    bool getMaximumDensity(QString pcfPath,
                           double &maximumDensity,
                           QString& strError);
    bool getMultiProcess(){return(mUseMultiProcess);};
    bool getPointsFromWktGeometry(QString pcfPath,
                                  QString wktGeometry,
                                  int geometryCrsEpsgCode,
                                  QString geometryCrsProj4String,
                                  QMap<int,QMap<int,QString> >& tilesTableName,
                                  QMap<int, QMap<int, QMap<int, QVector<PCFile::Point> > > > &pointsByTileByFileId, //[fileId][tileX][tileY]
                                  QMap<int,QMap<QString,bool> >& existsFieldsByFileId,
                                  QVector<QString>& ignoreTilesTableName,
                                  bool tilesFullGeometry,
                                  QString& strError);
    bool getProjectTypes(QVector<QString>& projectTypes,
                         QString& strError);
    bool getROIsWktGeometry(QString pcfPath,
                            QMap<QString, QString> &values,
                            QString& strError);
    bool getTilesNamesFromWktGeometry(QString pcfPath,
                                           QString wktGeometry,
                                           int geometryCrsEpsgCode,
                                           QString geometryCrsProj4String,
                                           QMap<int,QMap<int,QString> >& tilesTableName,
                                           QString& strError);
    bool getTilesWktGeometry(QString pcfPath,
                             QMap<QString, QString> &values,
                             QString& strError);
    bool initializeCrsTools(QString &strError);
    bool openPointCloudFile(QString pcPath,
                            QString& strError);
    bool processInternalCommand(QString& command,
                                QVector<QString>& inputFiles,
                                QString& outputPath,
                                QString& outputFile,
                                QString& suffix,
                                QString& prefix,
                                QString& strError);
    bool processInternalCommandVegetationGrowthEstimate(QString& command,
                                                        QVector<QString>& inputFiles,
                                                        QString& outputPath,
                                                        QString& outputFileName,
                                                        QString& suffix,
                                                        QString& prefix,
                                                        QString& strError);
    bool processReclassificationConfusionMatrixReport(QString& pcfPath,
                                                      QString& outputFileName,
                                                      QVector<int>& selectedClasses,
                                                      QString& strError);
    bool processProjectFile(QString& fileName,
                            QString& strError);
    bool runProcessList(QVector<QString>& processList,
                        QString title,
                        QString& strError,
                        QWidget* ptrWidget=NULL);
    bool selectInternalCommandParameters(QString internalCommand,
                                         QString& strError);
    bool selectLastoolsCommandParameters(QString lastoolscommand,
                                         QString& strError);
    bool selectProjectParameters(QString projectType,
                          QString& strError);
    bool setBasePath(QString basePath, QString &strError);
    void setCrsTools(libCRS::CRSTools* ptrCrsTools){mPtrCrsTools=ptrCrsTools;}
    bool setGeoidFilesLastoolsPath(QString value,
                               QString& strError);
    bool setLastoolsPath(QString path,
                         QString& strError);
    bool setMultiProcess(bool useMultiProcess,
                         QString& strError);
    bool setOutputPath(QString value,
                       QString& strError);
    bool setTempPath(QString value,
                     QString& strError);
    bool updatePoints(QString pcfPath,
                      QString strAction,
                      quint8 classValue,
                      QMap<int, QMap<int, QVector<int> > > &pointFileIdByTile,
                      QMap<int, QMap<int, QVector<int> > > &pointPositionByTile,
//                      QMap<int, QMap<int, QVector<quint8> > > &pointClassNewByTile,
//                      QMap<int, QMap<int, QVector<quint8> > > &pointClassByTile,
                      QMap<quint8,bool>& lockedClasses,
                      QString& strError);
private slots:
    void on_ProgressExternalProcessDialog_closed();

private:
    bool addPointCloudFile(QString fileName,
                           QTextStream& in,
                           QString& strError);
    bool addPointCloudFiles(QString fileName,
                            QTextStream& in,
                            QString& strError);
    bool addROI(QString fileName,
                QTextStream& in,
                QString& strError);
    bool checkInitialize(QString& strError);
    bool createPointCloudFile(QString fileName,
                              QTextStream& in,
                              QString& strError);
    bool getProjectParametersString(QString projectType,
                                    QString& projectParametersString,
                                    QString& strError);
    bool processReclassificationConfusionMatrixReport(QString fileName,
                                                      QTextStream& in,
                                                      QString& strError);
    bool readVegetationGrowthModel(QString& fileName,
                                   QMap<int,QVector<double> >& vegetationGrowthModel,
                                   QString& strError);
    bool removeDir(QString dirName,
                   bool onlyContent=false);
    bool setProjectsParametersManager(QString projectType,
                                      QString& strError);
    void setProjectTypes();
    bool validateProjectParametersString(QString projectType,
                                         QString projectParametersString,
                                         QString& strError);
    bool writePointCloudFile(QString fileName,
                             QTextStream& in,
                             QString& strError);

    void mpProcessInternalCommandVegetationGrowthEstimate(int nf);

    static PointCloudFileManager * mInstance;
    libCRS::CRSTools* mPtrCrsTools;
    QMap<QString,PCFile::PointCloudFile*> mPtrPcFiles;
    QString mBasePath;
    QVector<QString> mProjectTypes;
    QMap<QString,QString> mProjectParametersFileByProjectType;
    QMap<QString,ParametersManager*> mPtrProjectsParametersManagerByProjectType;
    QString mLastoolsPath;
    QString mLastoolsCommandsParametersFileName;
    QVector<QString> mLastoolsCommands;
    ParametersManager* mPtrLastoolsCommandsParameters;
    QString mInternalCommandsParametersFileName;
    QVector<QString> mInternalCommands;
    ParametersManager* mPtrInternalCommandsParameters;

    ProcessTools::ProgressExternalProcessDialog* mPtrProgressExternalProcessDialog;
    QString mStrExecution;
    QDateTime mInitialDateTime;
    QString mProgressExternalProcessTitle;

    QString mTempPath;
    QString mOutputPath;
    QString mGeoidFilessLastoolsPath;
    QVector<int> mGridSizes;
    int mMaxGridSize;
    bool mUseMultiProcess;

    QMutex mMutex;
    int mNumberOfFilesToProcess;
    QMap<QString,int> mNumberOfPointsToProcessByFileName;
    QVector<QString> mInputFiles;
    QProgressDialog* mPtrMpProgressDialog;
    QString mStrErrorMpProgressDialog;

    int mPICVGESpatialResolution;
    QMap<int,QVector<double> > mPICVGEBoundingBoxesByFilePos;
    QMap<int,QMap<quint16,QMap<quint16,quint16> > > mPICVGEMaxHeightsByTileXYByFilePos;
};
}
#endif // LIBPOINTCLOUDFILEMANAGER_H
