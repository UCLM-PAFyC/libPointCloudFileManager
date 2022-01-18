#ifndef POINTCLOUDFILE_H
#define POINTCLOUDFILE_H

#include "PointCloudFileDefinitions.h"

#include "libPointCloudFileManager_global.h"

#include <QString>
#include <QMap>
#include <QDateTime>

//#include <QWaitCondition>
#include <QMutex>
//#include <QtConcurrentRun>

#include <ogrsf_frmts.h>
#include <gdal_utils.h>
#include <gdal_priv.h>

#include "quazip.h"
#include "JlCompress.h"

class QProgressDialog;

class OGRGeometry;
//class QuaZip;

namespace libCRS{
    class CRSTools;
};

namespace PCFile{
class Point;
class PointCloudFileManager;
class LIBPOINTCLOUDFILEMANAGERSHARED_EXPORT PointCloudFile
{
public:
    PointCloudFile(libCRS::CRSTools* ptrCrsTools,
                   PointCloudFileManager* ptrPCFManager,
                   bool useMultiProcess=true);
    ~PointCloudFile();
    bool addPointCloudFile(QString inputFileName,
                           QString pointCloudCrsDescription,
                           int pointCloudCrsEpsgCode,
                           int pointCloudVerticalCrsEpsgCode,
                           bool updateHeader,
                           QString& strError);
    bool addPointCloudFile(QString inputFileName,
                           QString pointCloudCrsDescription,
                           QString pointCloudCrsProj4String,
                           int pointCloudCrsEpsgCode,
                           bool updateHeader,
                           QString& strError);
    bool addPointCloudFiles(QVector<QString>& inputFileNames,
                            QString pointCloudCrsDescription,
                            int pointCloudCrsEpsgCode,
                            int pointCloudVerticalCrsEpsgCode,
                            bool updateHeader,
                            QString& strError);
    bool addPointCloudFiles(QVector<QString>& inputFileNames,
                            QString pointCloudCrsDescription,
                            QString pointCloudCrsProj4String,
                            int pointCloudCrsEpsgCode,
                            bool updateHeader,
                            QString& strError);
    bool addROIs(QMap<QString,OGRGeometry*> ptrROIsGeometryByRoiId,
                 QString& strError);
    bool create(QString path,
                int crsEpsgCode,
                int verticalCrsEpsgCode,
                double gridSize,
                QString projectType,
                QString projectParametersString,
                QString& strError);
    bool create(QString path,
                QString dbCrsDescription,
                QString dbCrsProj4String,
                int dbCrsEpsgCode,
                QString heightType,
                double gridSize,
                QString projectType,
                QString projectParametersString,
                QString& strError);
    int getColorNumberOfBytes(){return(mNumberOfColorBytes);};
    QString getCrsDescription(){return(mCrsDescription);};
    int getCrsEpsgCode(){return(mSRID);};
    int getVerticalCrsEpsgCode(){return(mVerticalCrsEpsgCode);};
    QString getCrsProj4String(){return(mCrsProj4String);};
    QString getPath(){return(mPath);};
    double getGridSize(){return(mGridSize);};
    QString getHeightType(){return(mHeightType);};
    double getMaximumDensity(){return(mMaximumDensity);};
    bool getNeighbors(QVector<double> point, // 2d o 3d
                      int pointCrsEpsgCode,
                      QString pointCrsProj4String,
                      double searchRadius2d, // <=0 se usa 100 por la distancia para densidad media
                      int numberOfNeighbors, // <=0 se devuelven todos
                      QVector<PCFile::Point>& points, // ordenado de cercano a lejano
                      QVector<int>& tilesX,
                      QVector<int>& tilesY,
                      QVector<double>& distances, // ordenado de cercano a lejano
                      QVector<int>& fileIdPoints,
                      QMap<int,QMap<QString,bool> >& existsFieldsByFileId,
                      QString& strError);
    bool getPointsFromWktGeometry(QString wktGeometry,
                                  int geometryCrsEpsgCode,
                                  QString geometryCrsProj4String,
                                  QMap<int,QMap<int,QString> >& tilesTableName,
                                  QMap<int, QMap<int, QMap<int, QVector<PCFile::Point> > > > &pointsByTileByFileId, //[fileId][tileX][tileY]
                                  QMap<int,QMap<QString,bool> >& existsFieldsByFileId,
                                  QVector<QString>& ignoreTilesTableName,
                                  bool tilesFullGeometry,
                                  QString& strError);
    QString getProjectType(){return(mProjectType);};
    bool getReachedMaximumNumberOfPoints(bool& reachedMaximumNumberOfPoints,
                                         QString& strError);
    bool getROIsWktGeometry(QMap<QString, QString> &values,
                            QString& strError);
    bool getTilesNamesFromWktGeometry(QString wktGeometry,
                                           int geometryCrsEpsgCode,
                                           QString geometryCrsProj4String,
                                           QMap<int,QMap<int,QString> >& tilesTableName,
                                           QVector<QString> &ignoreTilesTableName,
                                           bool tilesFullGeometry,
                                           OGRGeometry **ptrPtrGeometry,
                                           QMap<int,QMap<int,bool> >& tilesOverlaps,
                                           QString& strError);
    bool getTilesNamesFromGeometry(QMap<int, QMap<int, QString> > &tilesTableName,
                                   QVector<QString> &ignoreTilesTableName,
                                   OGRGeometry* ptrGeometry,
                                   QMap<int, QMap<int, bool> > &tilesOverlaps,
                                   QString &strError);
    bool getTilesNamesFromWktGeometry(QString wktGeometry,
                                           int geometryCrsEpsgCode,
                                           QString geometryCrsProj4String,
                                           QMap<int,QMap<int,QString> >& tilesTableName,
                                           QString& strError);
    bool getTilesWktGeometry(QMap<QString, QString> &values,
                             QString& strError);
    bool processReclassificationConfusionMatrixReport(QString& fileName,
                                                      QVector<int>& classes,
                                                      QString& strError);
    bool setFromPath(QString path,
                     QString& strError);
    bool setOutputPath(QString value,
                       QString& strError);
    bool setTempPath(QString value,
                     QString& strError);
    bool updateNotEdited2dToolsPoints(QString pcfPath,
                                      QMap<int, QMap<int, QVector<int> > > &pointFileIdByTile,
                                      QMap<int, QMap<int, QVector<int> > > &pointPositionByTile,
                                      QMap<int, QMap<int, QVector<quint8> > > &pointClassNewByTile,
                                      QMap<int, QMap<int, QVector<quint8> > > &pointClassByTile,
                                      QString& strError);
    bool updatePoints(QString strAction,
                      quint8 classValue,
                      QMap<int, QMap<int, QVector<int> > > &pointFileIdByTile,
                      QMap<int, QMap<int, QVector<int> > > &pointPositionByTile,
//                      QMap<int, QMap<int, QVector<quint8> > > &pointClassNewByTile,
//                      QMap<int, QMap<int, QVector<quint8> > > &pointClassByTile,
                      QMap<quint8,bool>& lockedClasses,
                      QString& strError);
    bool writePointCloudFiles(QString suffix,
                              QString outputPath,
                              QString& strError);
private:
    bool addTilesFromBoundingBox(int minX,
                                 int minY,
                                 int maxX,
                                 int maxY,
//                                 QString inputFileName,
                                 QMap<int, QMap<int, int> > &tilesNumberOfPoints,
                                 QWidget *ptrWidget,
                                 QString &strError);
    bool addTileTable(int tileX,
                      int tileY,
                      bool& added,
                      QString &strError);
    void clear();
    bool readHeader(QString& strError);
    bool removeDir(QString dirName,
                   bool onlyContent=false);
    bool removeTile(int tileX,
                    int tileY,
                    QString& strError);
    bool removeTile(int tileX,
                    int tileY,
                    int fileIndex,
                    QString& strError);
    bool writeHeader(QString& strError);

    void mpAddPointCloudFile(QString inputFileName);
    void mpAddTilesGeometry(int tilePos);
    void mpGetTilesWktGeometry(int tilePos);
    void mpGetTilesFromWktGeometry(int tilePos);
    void mpGetTilesNamesFromWktGeometry(int tilePos);
    void mpGetPointsFromWktGeometryByTilePosition(int tilePos);

    QString mTempPath;
    QString mOutputPath;
    libCRS::CRSTools* mPtrCrsTools;
    qint32 mLastDateJd;
    qint32 mReferenceDateJd;
    qint32 mSRID;
    QString mHeightType; // puede ser el codigo epsg de un crs vertical
    QString mCrsDescription; // si es compuesto EPSG:25830+5782
    QString mCrsProj4String; // si hay crs vertical es wkt y no proj4
    int mVerticalCrsEpsgCode; // -1 o el codigo si es compuesto, persiste en mHeightType
    qreal mGridSize;
    qreal mMaximumDensity; // ptos/m2
    QString mProjectType;
    QMap<QString,OGRGeometry*> mPtrROIs;
    QMap<QString,QString> mROIsWkt;
    OGRGeometry* mPtrROIsUnion;
    qint32 mNumberOfPointsInMemory;
    qint32 mMaximumNumberOfPointsInMemory;
    QMap<QString,QString> mParameterValueByCode;
    QMap<QString,bool> mStoredFields;
    int mNumberOfColorBytes;
    QString mPath;
    QString mHeaderFileName;
    QMap<QString,int> mFilesIndex;
    QMap<int,QString> mFileByIndex;
    QMap<int,QString> mZipFilePathPointsByIndex; // ruta para descomprimir el fichero comprimido
    QMap<int,QString> mZipFilePointsByIndex; // fichero comprimido
    QMap<int,QString> mClassesFileByIndex; // fichero de clases
    int mNewFilesIndex;
//    QMap<int,OGRGeometry*> mFilePtrGeometryByIndex;
    QMap<int,QMap<int,QString> > mTilesName;
    QMap<int,QMap<int,int> > mTilesNumberOfPoints;
    QMap<int,QMap<int,bool> > mTilesContainedInROIs;
    QMap<int,QMap<int,bool> > mTilesOverlapsWithROIs;
    QMap<int,QMap<int,OGRGeometry*> > mTilesGeometry;
    QMap<QString,QString> mTilessWkt;
    QMap<int,QMap<int,QVector<int> > > mTilesByFileIndex;
    PointCloudFileManager* mPtrPCFManager;
    double mMinimumFc;
    double mMinimumSc;
    double mMinimumTc;

//    QString mZipFileName;
//    QuaZip* mPtrZipFile;

//    bool mUseMultiProcess;
    QProgressDialog* mPtrMpProgressDialog;
    QString mStrErrorMpProgressDialog;
    QMutex mMutex;
    OGRGeometry* mMpPtrGeometry;
    QMap<int, QMap<int, QString> > mMpTilesTableName;
    QMap<int, QMap<int, bool> > mTilesOverlaps;
    QVector<QString> mMpIgnoreTilesTableName;
    QVector<int> mTilesXToProcess;
    QVector<int> mTilesYToProcess;
    QuaZip mZipFilePoints;
    QMap<int,QMap<int,int> > mMpTilesNumberOfPointsInFile;
    QMap<int,QMap<int,QVector<PCFile::Point> > > mPointsByTile;
    QMap<int,QMap<int,QVector<quint8> > > mTilesPointsClass;
    QMap<int,QMap<int,QMap<int,quint8> > > mTilesPointsClassNewByPos; // se guarda vacío
    QMap<int,QMap<int,int> > mTilesNop;
    bool mTilesFullGeometry;
    bool mExistsColor;
    bool mExistsGpsTime;
    bool mExistsUserData;
    bool mExistsIntensity;
    bool mExistsSourceId;
    bool mExistsNir;
    bool mExistsReturn;
    bool mExistsReturns;
    QString mClassesFileName;
    QString mZipFileNamePoints;

    int mNumberOfFilesToProcess;
    QMap<QString,int> mNumberOfPointsToProcessByFileName;
    bool mUpdateHeader;
    int mMaximumNumberOfPoints; // POINTCLOUDFILE_WITHOUT_MAXIMUM_NUMBER_OF_POINTS_LIMITS
    int mNumberOfPoints;
};
}
#endif // POINTCLOUDFILE_H
