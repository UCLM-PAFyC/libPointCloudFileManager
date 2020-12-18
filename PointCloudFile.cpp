#include <QFile>
#include <QFileInfo>
#include <QProgressDialog>
#include <QApplication>
#include <QDir>
#include <QMessageBox>
#include <QDateTime>
#include <QTextStream>
#include <QDataStream>
#include <QtConcurrent>
#include <qtconcurrentmap.h>
#include <QProgressDialog>

#include <ogrsf_frmts.h>
#include <gdal_utils.h>
#include <gdal_priv.h>

#include "lasreader.hpp"
#include "laswriter.hpp"

//#include "quazip.h"
//#include "JlCompress.h"

#include "CRSTools.h"
#include "../libIGDAL/Shapefile.h"
#include "PointCloudFileDefinitions.h"

#include "PointCloudFileManager.h"
#include "PointCloudFile.h"

using namespace PCFile;

PointCloudFile::PointCloudFile(libCRS::CRSTools* ptrCrsTools,
                               PointCloudFileManager* ptrPCFManager,
                               bool useMultiProcess)
{
    mPtrCrsTools=ptrCrsTools;
    mPtrPCFManager=ptrPCFManager;
    mSRID=POINTCLOUDFILE_SRID_NO_VALUE;
    mGridSize=POINTCLOUDFILE_NO_DOUBLE_VALUE;
    mPtrROIsUnion=NULL;
    mReferenceDateJd=QDate::fromString("1970:01:13","yyyy:MM:dd").toJulianDay();
    mNumberOfPointsInMemory=0;
    mMaximumDensity=0.;
    mStoredFields[POINTCLOUDFILE_PARAMETER_COLOR]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_GPS_TIME]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_USER_DATA]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_INTENSITY]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_SOURCE_ID]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_NIR]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_RETURN]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_RETURNS]=false;
    mNumberOfColorBytes=1;
    mNewFilesIndex=0;
    mMinimumFc=POINTCLOUDFILE_NO_DOUBLE_MINIMUM_VALUE;
    mMinimumSc=POINTCLOUDFILE_NO_DOUBLE_MINIMUM_VALUE;
    mMinimumTc=POINTCLOUDFILE_NO_DOUBLE_MINIMUM_VALUE;
    mUseMultiProcess=useMultiProcess;
    mPtrMpProgressDialog=NULL;
    mMpPtrGeometry=NULL;
}

PointCloudFile::~PointCloudFile()
{
    clear();
    QDir auxDir=QDir::currentPath();
    QMap<int,QString>::const_iterator iterZipFilePaths=mZipFilePathPointsByIndex.begin();
    while(iterZipFilePaths!=mZipFilePathPointsByIndex.end())
    {
        QString zipFilePath=iterZipFilePaths.value();
        if(auxDir.exists(zipFilePath))
        {
            if(!removeDir(zipFilePath))
            {
                int yo=1;
            }
        }
        iterZipFilePaths++;
    }

}

bool PointCloudFile::addPointCloudFile(QString inputFileName,
                                       QString pointCloudCrsDescription,
                                       QString pointCloudCrsProj4String,
                                       int pointCloudCrsEpsgCode,
                                       bool updateHeader,
                                       QString &strError)
{
    if(pointCloudCrsEpsgCode!=mSRID)
    {
        strError=QObject::tr("PointCloudFile::addPointCloudFile");
        strError+=QObject::tr("\nFile CRS EPSG: %1 and project CRS EPSG: %2 are different")
                .arg(QString::number(pointCloudCrsEpsgCode)).arg(QString::number(mSRID));
        strError+=QObject::tr("\nReproject File to project CRS before");
        return(false);
    }
    if(mTempPath.isEmpty())
    {
        strError=QObject::tr("PointCloudFile::addPointCloudFile");
        strError+=QObject::tr("\nTemporal path is empty");
        return(false);
    }
    QWidget* ptrWidget=new QWidget();
//    if(mFilePtrGeometryByIndex.contains(inputFileName))
//    {
//        strError=QObject::tr("PointCloudFile::addPointCloudFile");
//        strError+=QObject::tr("\nExists point cloud file:\n%1").arg(inputFileName);
//        return(false);
//    }
    QString strAuxError;
    double minX=1000000000.0;
    double minY=1000000000.0;
    double minZ=1000000000.0;
    double maxX=-1000000000.0;
    double maxY=-1000000000.0;
    double maxZ=-1000000000.0;

    QMap<int,QMap<int,int> > tilesNumberOfPoints; // nuevos para este fichero
    if(mPtrROIsUnion!=NULL)
    {
        OGREnvelope* ptrROIsEnvelope=new OGREnvelope();
        mPtrROIsUnion->getEnvelope(ptrROIsEnvelope);
        minX=ptrROIsEnvelope->MinX;
        minY=ptrROIsEnvelope->MinY;
        maxX=ptrROIsEnvelope->MaxX;
        maxY=ptrROIsEnvelope->MaxY;
        delete(ptrROIsEnvelope);
        // se crean entradas en: mTilesTableName y mTilesTableGeometry
        if(!addTilesFromBoundingBox(minX,minY,maxX,maxY,
//                                    inputFileName,
                                    tilesNumberOfPoints,
                                    ptrWidget,strAuxError))
        {
            strError=QObject::tr("PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError adding tiles from ROIs for file:\n%1\nError:\n%2")
                    .arg(inputFileName).arg(strAuxError);
            return(false);
        }
        if(tilesNumberOfPoints.size()==0) // no hay tiles para las ROIs
        {
            return(true);
        }
    }

    QFileInfo inputFileInfo(inputFileName);
    QString inputFileBaseName=inputFileInfo.completeBaseName();
    QString tilesPointsFileZipFileName=mPath+"/"+inputFileBaseName+"."+POINTCLOUDFILE_DHL_SUFFIX;
    if(QFile::exists(tilesPointsFileZipFileName))
    {
        if(!QFile::remove(tilesPointsFileZipFileName))
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError removing existing file:\n%1")
                    .arg(tilesPointsFileZipFileName);
            return(false);
        }
    }

    QString tilesPointsFileZipFilePath=mPath+"/"+inputFileBaseName;
    QDir currentDir=QDir::currentPath();
    if(currentDir.exists(tilesPointsFileZipFilePath))
    {
        if(!removeDir(tilesPointsFileZipFilePath))
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError removing existing dir:\n%1")
                    .arg(tilesPointsFileZipFilePath);
            return(false);
        }
    }
    if(!currentDir.mkpath(tilesPointsFileZipFilePath))
    {
        strError=QObject::tr("\PointCloudFile::addPointCloudFile");
        strError+=QObject::tr("\nError making dir:\n%1")
                .arg(tilesPointsFileZipFilePath);
        return(false);
    }

    QString pointsClassFileName=mPath+"/"+inputFileBaseName+"."+POINTCLOUDFILE_PCS_SUFFIX;
    if(QFile::exists(pointsClassFileName))
    {
        if(!QFile::remove(pointsClassFileName))
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError removing existing file:\n%1")
                    .arg(pointsClassFileName);
            return(false);
        }
    }
//    QuaZip* ptrTilesPointsFileZip= new QuaZip(tilesPointsFileZipFileName);
//    if(!ptrTilesPointsFileZip->open(QuaZip::mdCreate))
//    {
//        int qazErrorCode=ptrTilesPointsFileZip->getZipError();
//        strError=QObject::tr("\PointCloudFile::addPointCloudFile");
//        strError+=QObject::tr("\nError creating file:\n%1\nError code:\n%2")
//                .arg(tilesPointsFileZipFileName).arg(QString::number(qazErrorCode));
//        return(false);
//    }
    std::string stdFileName=inputFileName.toStdString();
    const char* charFileName=stdFileName.c_str();

    bool storeColor=mStoredFields[POINTCLOUDFILE_PARAMETER_COLOR];
    bool storeGpsTime=mStoredFields[POINTCLOUDFILE_PARAMETER_GPS_TIME];
    bool storeUserData=mStoredFields[POINTCLOUDFILE_PARAMETER_USER_DATA];
    bool storeIntensity=mStoredFields[POINTCLOUDFILE_PARAMETER_INTENSITY];
    bool storeSourceId=mStoredFields[POINTCLOUDFILE_PARAMETER_SOURCE_ID];
    bool storeNir=mStoredFields[POINTCLOUDFILE_PARAMETER_NIR];
    bool storeReturn=mStoredFields[POINTCLOUDFILE_PARAMETER_RETURN];
    bool storeReturns=mStoredFields[POINTCLOUDFILE_PARAMETER_RETURNS];
    bool existsColor=false;
    bool existsGpsTime=false;
    bool existsUserData=false;
    bool existsIntensity=false;
    bool existsSourceId=false;
    bool existsNir=false;
    bool existsReturn=false;
    bool existsReturns=false;
    if(storeColor||storeGpsTime||storeUserData
            ||storeIntensity||storeSourceId||storeNir
            ||storeReturn||storeReturns)
    {
        LASreadOpener lasreadopener;
        lasreadopener.set_file_name(charFileName);
        if (!lasreadopener.active())
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError opening file:\n%1").arg(inputFileName);
            return(false);
        }
        LASreader* lasreader = lasreadopener.open();
        LASheader* lasheader = &lasreader->header;
        int variablesToFind=0;
        if(storeColor) variablesToFind++;
        if(storeGpsTime) variablesToFind++;
        if(storeUserData) variablesToFind++;
        if(storeIntensity) variablesToFind++;
        if(storeSourceId) variablesToFind++;
        if(storeNir) variablesToFind++;
        if(storeReturn) variablesToFind++;
        if(storeReturns) variablesToFind++;
        while(lasreader->read_point())
        {
            if(storeColor&&!existsColor)
            {
                if(lasreader->point.have_rgb)
                {
                    existsColor=true;
                    variablesToFind--;
                }
            }
            if(storeGpsTime&&!existsGpsTime)
            {
                if(lasreader->point.have_gps_time)
                {
                    existsGpsTime=true;
                    variablesToFind--;
                }
            }
            if(storeNir&&!existsNir)
            {
                if(lasreader->point.have_nir)
                {
                    existsNir=true;
                    variablesToFind--;
                }
            }
            if(storeIntensity&&!existsIntensity)
            {
                if(lasreader->point.get_intensity()!=0)
                {
                    existsIntensity=true;
                    variablesToFind--;
                }
            }
            if(storeUserData&&!existsUserData)
            {
                if(lasreader->point.get_user_data()!=0)
                {
                    existsUserData=true;
                    variablesToFind--;
                }
            }
            if(storeSourceId&&!existsSourceId)
            {
                if(lasreader->point.get_point_source_ID()!=0)
                {
                    existsSourceId=true;
                    variablesToFind--;
                }
            }
            if(storeReturn&&!existsReturn)
            {
                if(lasreader->point.get_return_number()!=0)
                {
                    existsReturn=true;
                    variablesToFind--;
                }
            }
            if(storeReturns&&!existsReturns)
            {
                if(lasreader->point.get_number_of_returns()!=0)
                {
                    existsReturns=true;
                    variablesToFind--;
                }
            }
            if(variablesToFind==0) break;
        }
        lasreader->close();
        delete lasreader;
    }

    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(charFileName);
    if (!lasreadopener.active())
    {
        strError=QObject::tr("\PointCloudFile::addPointCloudFile");
        strError+=QObject::tr("\nError opening file:\n%1").arg(inputFileName);
        return(false);
    }

    LASreader* lasreader = lasreadopener.open();
    LASheader* lasheader = &lasreader->header;
    int numberOfPoints=lasreader->npoints;
    double fileMinX=lasheader->min_x;
    double fileMinY=lasheader->min_y;
    double fileMaxX=lasheader->max_x;
    double fileMaxY=lasheader->max_y;
    double fileMinZ=lasheader->min_z;
    double fileMaxZ=lasheader->max_z;
    if(mPtrROIsUnion==NULL)
    {
        minX=fileMinX;
        minY=fileMinY;
        maxX=fileMaxX;
        maxY=fileMaxY;
        if(!addTilesFromBoundingBox(minX,minY,maxX,maxY,
//                                    inputFileName,
                                    tilesNumberOfPoints,
                                    ptrWidget,strAuxError))
        {
            strError=QObject::tr("PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError adding tiles from file:\n%1\nError:\n%2")
                    .arg(inputFileName).arg(strAuxError);
            return(false);
        }
        if(tilesNumberOfPoints.size()==0) // no hay tiles, ¿fichero sin puntos?
        {
            return(true);
        }
    }
    QProgressDialog* ptrProgress=NULL;
    int pointsByStep=POINTCLOUDFILE_NUMBER_OF_POINTS_TO_PROCESS_BY_STEP;
    int numberOfSteps=ceil((double)numberOfPoints/(double)pointsByStep);
    if(numberOfSteps>1
            &&ptrWidget!=NULL)
    {
        QString title=QObject::tr("Adding Point Cloud File:");
        QString msgGlobal=inputFileName;
        msgGlobal+="\n";
        msgGlobal+=QString::number(numberOfPoints,10);
        msgGlobal+=" points";
        ptrProgress=new QProgressDialog(title, "Abort",0,numberOfSteps, ptrWidget);
        ptrProgress->setWindowModality(Qt::WindowModal);
        ptrProgress->setLabelText(msgGlobal);
        ptrProgress->show();
        qApp->processEvents();
    }
    int pointPosition=-1;
    int step=0;
    int numberOfProcessedPoints=0;
    int numberOfProcessedPointsInStep=0;
    U8 pointDataFormat=lasheader->point_data_format;
    QMap<int,QMap<int,QString> > tilesPointsFileNames;
    QMap<int,QMap<int,QVector<quint8> > > tilesPointsClass;
    QMap<int,QMap<int,QMap<int,quint8> > > tilesPointsClassNewByPos; // se guarda vacío
//    QMap<int,QMap<int,QuaZipFile*> > tilesPtrPointsFiles;
    QMap<int,QMap<int,QFile*> > tilesPtrPointsFiles;
    QMap<int,QMap<int,QDataStream*> > tilesPtrPointsDataStreams;
    QMap<int,QMap<int,int> > tilesNop;
    while(lasreader->read_point())
    {
        double x=lasreader->point.get_X()*lasheader->x_scale_factor+lasheader->x_offset;
        double y=lasreader->point.get_Y()*lasheader->y_scale_factor+lasheader->y_offset;
        pointPosition++;
        numberOfProcessedPoints++;
        numberOfProcessedPointsInStep++;
        if(numberOfProcessedPointsInStep==pointsByStep)
        {
            step++;
            numberOfProcessedPointsInStep=0;
            if(numberOfSteps>1
                    &&ptrWidget!=NULL)
            {
                ptrProgress->setValue(step);
                qApp->processEvents();
            }
        }
        bool includedPoint=true;
        int tileX=qRound(floor(floor(x)/mGridSize)*mGridSize);
        int tileY=qRound(floor(floor(y)/mGridSize)*mGridSize);
        if(!mTilesName.contains(tileX))
        {
            includedPoint=false;
        }
        else if(!mTilesName[tileX].contains(tileY))
        {
            includedPoint=false;
        }
        if(!includedPoint)
        {
            continue;
        }
        if(mPtrROIsUnion!=NULL)
        {
            if(mTilesOverlapsWithROIs[tileX][tileY])
            {
                OGRGeometry* ptrPoint=NULL;
                ptrPoint=OGRGeometryFactory::createGeometry(wkbPoint);
                ((OGRPoint*)ptrPoint)->setX(x);
                ((OGRPoint*)ptrPoint)->setY(y);
                if(!mPtrROIsUnion->Contains(ptrPoint))
                {
                    includedPoint=false;
                }
                OGRGeometryFactory::destroyGeometry(ptrPoint);
            }
        }
        if(!includedPoint)
        {
            continue;
        }
        QString tileTableName="tile_"+QString::number(tileX)+"_"+QString::number(tileY);
        quint8 pointClass=lasreader->point.get_classification();
        quint16 ix=qRound((x-tileX)*1000.);
        quint16 iy=qRound((y-tileY)*1000.);
        double z=lasreader->point.get_Z()*lasheader->z_scale_factor+lasheader->z_offset;
        if(z<POINTCLOUDFILE_HEIGHT_MINIMUM_VALID_VALUE
                ||z>POINTCLOUDFILE_HEIGHT_MAXIMUM_VALID_VALUE)
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nIn file:\n%1").arg(inputFileName);
            strError+=QObject::tr("\nfor point x=%1, y=%2")
                    .arg(QString::number(x,'f',3))
                    .arg(QString::number(y,'f',3));
            strError+=QObject::tr("\nz: %1 out of valid domain: [%2,%3]")
                    .arg(QString::number(z,'f',3))
                    .arg(QString::number(POINTCLOUDFILE_HEIGHT_MINIMUM_VALID_VALUE,'f',3))
                    .arg(QString::number(POINTCLOUDFILE_HEIGHT_MAXIMUM_VALID_VALUE,'f',3));
            return(false);
        }
        double zt=z-POINTCLOUDFILE_HEIGHT_MINIMUM_VALID_VALUE;
        quint8 z_pc=zt*1000.0-floor(zt*10.)*100.;
        double nz=floor(zt*10.);
        quint8 z_pa=floor(nz/256.);
        quint8 z_pb=nz-z_pa*256.;
//        double zc=(z_pa*256.0+z_pb)/10.+z_pc/1000.+POINTCLOUDFILE_HEIGHT_MINIMUM_VALID_VALUE;
        QDataStream* tilePtrPointsDataStream=NULL;
        bool existsTileFile=true;
        if(!tilesPointsFileNames.contains(tileX))
        {
            existsTileFile=false;
        }
        else if(!tilesPointsFileNames[tileX].contains(tileY))
        {
            existsTileFile=false;
        }
        if(!existsTileFile)
        {
//            QString tilePointsFileName=tileTableName;
//            QuaZipFile* ptrTilePointsFile=new QuaZipFile(ptrTilesPointsFileZip);
//            if(!ptrTilePointsFile->open(QIODevice::WriteOnly,
//                                        QuaZipNewInfo(tilePointsFileName)))
//            {
//                QString qazError=ptrTilePointsFile->getZipError();
//                strError=QObject::tr("\PointCloudFile::addPointCloudFile");
//                strError+=QObject::tr("\nIn file:\n%1\nError creating file:\n%2\nError:\n%3")
//                        .arg(tilesPointsFileZipFileName).arg(tilePointsFileName).arg(qazError);
//                return(false);
//            }
            QString tilePointsFileName=tilesPointsFileZipFilePath+"/"+tileTableName;
            QFile* ptrTilePointsFile=new QFile(tilePointsFileName);
            if(!ptrTilePointsFile->open(QIODevice::WriteOnly))
            {
                strError=QObject::tr("\PointCloudFile::addPointCloudFile");
                strError+=QObject::tr("\nError creating file:\n%1")
                        .arg(tilePointsFileName);
                return(false);
            }
            tilePtrPointsDataStream=new QDataStream(ptrTilePointsFile);
            tilesPointsFileNames[tileX][tileY]=tilePointsFileName;
            tilesPtrPointsFiles[tileX][tileY]=ptrTilePointsFile;
            tilesPtrPointsDataStreams[tileX][tileY]=tilePtrPointsDataStream;
            QVector<quint8> auxClasses;
            tilesPointsClass[tileX][tileY]=auxClasses;
            tilesNop[tileX][tileY]=0;
        }
        else
        {
            tilePtrPointsDataStream=tilesPtrPointsDataStreams[tileX][tileY];
        }
        (*tilePtrPointsDataStream)<<ix<<iy<<z_pa<<z_pb<<z_pc;
        if(existsColor)
        {
            quint16 color_r=lasreader->point.get_R();
            quint16 color_g=lasreader->point.get_G();
            quint16 color_b=lasreader->point.get_B();
            if(mNumberOfColorBytes==1)
            {
                quint8 r=floor(double(color_r)/256.0);
                if(r>255) r=255;
                quint8 g=floor(double(color_g)/256.0);
                if(g>255) g=255;
                quint8 b=floor(double(color_b)/256.0);
                if(b>255) b=255;
                (*tilePtrPointsDataStream)<<r;
                (*tilePtrPointsDataStream)<<g;
                (*tilePtrPointsDataStream)<<b;
            }
            else
            {
                (*tilePtrPointsDataStream)<<color_r;
                (*tilePtrPointsDataStream)<<color_g;
                (*tilePtrPointsDataStream)<<color_b;
            }
        }
        if(existsGpsTime)
        {
            double gpsTime=lasreader->point.get_gps_time();
            int dayOfWeek=floor(gpsTime/24./60./60.);
            gpsTime-=(dayOfWeek*24.*60.*60.);
            int hours=floor(gpsTime/60./60.);
            gpsTime-=(hours*60.*60.);
            double dblMinutes=gpsTime/60.;
            int ms=qRound((dblMinutes*60.)*pow(10.,6.));
            quint8 msb1=floor(ms/256./256./256.);
            quint8 msb2=floor((ms-msb1*256.*256.*256.)/256./256.);
            quint8 msb3=floor((ms-msb1*256.*256.*256.-msb2*256.*256.)/256.);
            quint8 gpsTimeDow=dayOfWeek; // 3 bits, 0-7
            quint8 gpsHour=hours; // 5 bits, 0-23
            quint8 gpsDowHourPackit;
            gpsDowHourPackit = (gpsTimeDow << 3) | gpsHour;
            (*tilePtrPointsDataStream)<<gpsDowHourPackit;
            (*tilePtrPointsDataStream)<<msb1;
            (*tilePtrPointsDataStream)<<msb2;
            (*tilePtrPointsDataStream)<<msb3;
        }
        if(existsUserData)
        {
            quint8 userData=lasreader->point.get_user_data();
            (*tilePtrPointsDataStream)<<userData;
        }
        if(existsIntensity)
        {
            quint16 intensity=lasreader->point.get_intensity();
            (*tilePtrPointsDataStream)<<intensity;
        }
        if(existsSourceId)
        {
            quint16 sourceId=lasreader->point.get_point_source_ID();
            (*tilePtrPointsDataStream)<<sourceId;
        }
        if(existsNir)
        {
            quint16 nir=lasreader->point.get_NIR();
            if(mNumberOfColorBytes==1)
            {
                quint8 ir=floor(double(nir)/256.0);
                if(ir>255) ir=255;
                (*tilePtrPointsDataStream)<<ir;
            }
            else
            {
                (*tilePtrPointsDataStream)<<nir;
            }
        }
        if(existsReturn)
        {
            quint8 returnNumber=lasreader->point.get_return_number();
            (*tilePtrPointsDataStream)<<returnNumber;
        }
        if(existsReturns)
        {
            quint8 numberOfReturns=lasreader->point.get_number_of_returns();
            (*tilePtrPointsDataStream)<<numberOfReturns;
        }
        double minX=floor(x);
        double minY=floor(y);
        double minZ=floor(z);
        if(minX<mMinimumFc) mMinimumFc=minX;
        if(minY<mMinimumSc) mMinimumSc=minY;
        if(minZ<mMinimumTc) mMinimumTc=minZ;
        tilesPointsClass[tileX][tileY].push_back(pointClass);
        tilesNumberOfPoints[tileX][tileY]=tilesNumberOfPoints[tileX][tileY]+1;
        tilesNop[tileX][tileY]=tilesNop[tileX][tileY]+1;
    }
    if(numberOfSteps>1
            &&ptrWidget!=NULL)
    {
        ptrProgress->setValue(numberOfSteps);
        qApp->processEvents();
        ptrProgress->close();
        delete(ptrProgress);
    }
    lasreader->close();
    delete lasreader;
    if(tilesPointsClass.size()==0)
    {
        if(!removeDir(tilesPointsFileZipFilePath))
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError removing directory:\n%1")
                    .arg(tilesPointsFileZipFilePath);
            return(false);
        }
        return(true);
    }
    int fileIndex=-1;
    if(mFilesIndex.contains(inputFileName))
    {
        fileIndex=mFilesIndex[inputFileName];
    }
    else
    {
        /*
        QString wktGeometry="POLYGON((";
        wktGeometry+=QString::number(floor(fileMinX),'f',0);
        wktGeometry+=" ";
        wktGeometry+=QString::number(floor(fileMinY),'f',0);
        wktGeometry+=",";
        wktGeometry+=QString::number(floor(fileMinX),'f',0);
        wktGeometry+=" ";
        wktGeometry+=QString::number(floor(fileMaxY),'f',0);
        wktGeometry+=",";
        wktGeometry+=QString::number(floor(fileMaxX),'f',0);
        wktGeometry+=" ";
        wktGeometry+=QString::number(floor(fileMaxY),'f',0);
        wktGeometry+=",";
        wktGeometry+=QString::number(floor(fileMaxX),'f',0);
        wktGeometry+=" ";
        wktGeometry+=QString::number(floor(fileMinY),'f',0);
        wktGeometry+=",";
        wktGeometry+=QString::number(floor(fileMinX),'f',0);
        wktGeometry+=" ";
        wktGeometry+=QString::number(floor(fileMinY),'f',0);
        wktGeometry+="))";
        QByteArray byteArrayWktGeometry = wktGeometry.toUtf8();
        char *charsWktGeometry = byteArrayWktGeometry.data();
        OGRGeometry* ptrGeometry;
        ptrGeometry=OGRGeometryFactory::createGeometry(wkbPolygon);
        if(OGRERR_NONE!=ptrGeometry->importFromWkt(&charsWktGeometry))
        {
            strError=QObject::tr("PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError making geometry from WKT: %1").arg(wktGeometry);
            lasreader->close();
            return(false);
        }
        */
        fileIndex=mNewFilesIndex;
        mNewFilesIndex++;
//        mFilePtrGeometryByIndex[fileIndex]=ptrGeometry;
        mFilesIndex[inputFileName]=fileIndex;
        mFileByIndex[fileIndex]=inputFileName;
        QMap<int,QVector<int> > aux;
        mTilesByFileIndex[fileIndex]=aux;
    }

//    QMap<int,QMap<int,QuaZipFile*> >::iterator iterTileXZf=tilesPtrPointsFiles.begin();
    QMap<int,QMap<int,QFile*> >::iterator iterTileXZf=tilesPtrPointsFiles.begin();
    while(iterTileXZf!=tilesPtrPointsFiles.end())
    {
        int tileX=iterTileXZf.key();
//        QMap<int,QuaZipFile*>::iterator iterTileYZf=iterTileXZf.value().begin();
        QMap<int,QFile*>::iterator iterTileYZf=iterTileXZf.value().begin();
        while(iterTileYZf!=iterTileXZf.value().end())
        {
            int tileY=iterTileYZf.key();
//            QuaZipFile* ptrFile=iterTileYZf.value();
            QFile* ptrFile=iterTileYZf.value();
            ptrFile->close();
//            int qazErrorCode=ptrFile->getZipError();
//            if(UNZ_OK!=qazErrorCode)
//            {
//                int numberOfPoints=tilesPointsClass[tileX][tileY].size();
//                QString tileFileName=tilesPointsFileNames[tileX][tileY];
//                strError=QObject::tr("\PointCloudFile::addPointCloudFile");
//                strError+=QObject::tr("\nIn file:\n%1\nError closing file:\n%2\nError code:\n%3")
//                        .arg(tilesPointsFileZipFileName).arg(tileFileName).arg(QString::number(qazErrorCode));
//                return(false);
//            }
            delete(tilesPtrPointsDataStreams[tileX][tileY]);
            tilesPtrPointsDataStreams[tileX][tileY]=NULL;
            delete(ptrFile);
            tilesPtrPointsFiles[tileX][tileY]=NULL;
            iterTileYZf++;
        }
        iterTileXZf++;
    }
    if(!JlCompress::compressDir(tilesPointsFileZipFileName,
                                tilesPointsFileZipFilePath))
    {
        strError=QObject::tr("\PointCloudFile::addPointCloudFile");
        strError+=QObject::tr("\nError compressing directory:\n%1")
                .arg(tilesPointsFileZipFilePath);
        return(false);
    }
    if(!removeDir(tilesPointsFileZipFilePath))
    {
        strError=QObject::tr("\PointCloudFile::addPointCloudFile");
        strError+=QObject::tr("\nError removing directory:\n%1")
                .arg(tilesPointsFileZipFilePath);
        return(false);
    }
//    ptrTilesPointsFileZip->close();
//    int qazErrorCode=ptrTilesPointsFileZip->getZipError();
//    if(UNZ_OK!=qazErrorCode)
//    {
//        strError=QObject::tr("\PointCloudFile::addPointCloudFile");
//        strError+=QObject::tr("\nError closing file:\n%1\nError:\n%2")
//                .arg(tilesPointsFileZipFileName).arg(QString::number(qazErrorCode));
//        return(false);
//    }
    QFile pointsClassFile(pointsClassFileName);
    pointsClassFile.open(QIODevice::WriteOnly);
    QDataStream outPointsClass(&pointsClassFile);   // we will serialize the data into the file
    QMap<QString,bool> exitsFields;
    exitsFields[POINTCLOUDFILE_PARAMETER_COLOR]=existsColor;
    exitsFields[POINTCLOUDFILE_PARAMETER_GPS_TIME]=existsGpsTime;
    exitsFields[POINTCLOUDFILE_PARAMETER_USER_DATA]=existsUserData;
    exitsFields[POINTCLOUDFILE_PARAMETER_INTENSITY]=existsIntensity;
    exitsFields[POINTCLOUDFILE_PARAMETER_SOURCE_ID]=existsSourceId;
    exitsFields[POINTCLOUDFILE_PARAMETER_NIR]=existsNir;
    exitsFields[POINTCLOUDFILE_PARAMETER_RETURN]=existsReturn;
    exitsFields[POINTCLOUDFILE_PARAMETER_RETURNS]=existsReturns;
    outPointsClass<<tilesNop;
    outPointsClass<<exitsFields;
    outPointsClass<<tilesPointsClass;
    outPointsClass<<tilesPointsClassNewByPos;
    pointsClassFile.close();
    QMap<int,QMap<int,int> >::const_iterator iterTileX=tilesNumberOfPoints.begin();
    while(iterTileX!=tilesNumberOfPoints.end())
    {
        int tileX=iterTileX.key();
        QMap<int,int>::const_iterator iterTileY=iterTileX.value().begin();
        while(iterTileY!=iterTileX.value().end())
        {
            int tileY=iterTileY.key();
            int numberOfPoints=iterTileY.value();
            if(numberOfPoints==0)
            {
                if(!removeTile(iterTileX.key(),iterTileY.key(),fileIndex,strAuxError))
                {
                    strError=QObject::tr("PointCloudFile::addPointCloudFile");
                    strError+=QObject::tr("Error removing tile: %1 %2\nfor file:\n%3\nError:\n%4")
                            .arg(QString::number(iterTileX.key()))
                            .arg(QString::number(iterTileY.key()))
                            .arg(inputFileName).arg(strAuxError);
                    return(false);
                }
            }
            else
            {
                mTilesNumberOfPoints[tileX][tileY]=mTilesNumberOfPoints[tileX][tileY]+numberOfPoints;
                double tileDensity=((double)numberOfPoints)/pow(mGridSize,2.0);
                if(tileDensity>mMaximumDensity)
                {
                    mMaximumDensity=tileDensity;
                }
                if(!mTilesByFileIndex[fileIndex].contains(tileX))
                {
                    QVector<int> aux;
                    mTilesByFileIndex[fileIndex][tileX]=aux;
                }
                if(mTilesByFileIndex[fileIndex][tileX].indexOf(tileY)==-1)
                {
                    mTilesByFileIndex[fileIndex][tileX].push_back(tileY);
                }
            }
            iterTileY++;
        }
        iterTileX++;
    }
    if(mTilesByFileIndex[fileIndex].size()==0)
    {
        mTilesByFileIndex.remove(fileIndex);
        QString fileName=mFileByIndex[fileIndex];
        mFilesIndex.remove(fileName);
        mFileByIndex.remove(fileIndex);
        /*
        OGRGeometry* ptrGeometry=mFilePtrGeometryByIndex[fileIndex];
        if(ptrGeometry!=NULL)
        {
            OGRGeometryFactory::destroyGeometry(ptrGeometry);
            mFilePtrGeometryByIndex[fileIndex]=NULL;
        }
        mFilePtrGeometryByIndex.remove(fileIndex);
        */
    }
    mZipFilePathPointsByIndex[fileIndex]=tilesPointsFileZipFilePath;
    mZipFilePointsByIndex[fileIndex]=tilesPointsFileZipFileName;
    mClassesFileByIndex[fileIndex]=pointsClassFileName;
    if(updateHeader)
    {
        if(!writeHeader(strAuxError))
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError writing header after add file:\n%1").arg(inputFileName);
            return(false);
        }
    }
    return(true);
}

bool PointCloudFile::addPointCloudFiles(QVector<QString> &inputFileNames,
                                        QString pointCloudCrsDescription,
                                        QString pointCloudCrsProj4String,
                                        int pointCloudCrsEpsgCode,
                                        bool updateHeader,
                                        QString &strError)
{
    QString strAuxError;
    if(!mUseMultiProcess||inputFileNames.size()==1)
    {
        for(int nf=0;nf<inputFileNames.size();nf++)
        {
            QString inputFileName=inputFileNames[nf];
            if(!addPointCloudFile(inputFileName,
                                  pointCloudCrsDescription,
                                  pointCloudCrsProj4String,
                                  pointCloudCrsEpsgCode,
                                  updateHeader,
                                  strAuxError))
            {
                strError=QObject::tr("PointCloudFile::addPointCloudFiles");
                strError+=QObject::tr("\nError adding file:\n%1\nError:\n%2")
                        .arg(inputFileName).arg(strAuxError);
                return(false);
            }
        }
        return(true);
    }
    if(pointCloudCrsEpsgCode!=mSRID)
    {
        strError=QObject::tr("PointCloudFile::addPointCloudFiles");
        strError+=QObject::tr("\nFile CRS EPSG: %1 and project CRS EPSG: %2 are different")
                .arg(QString::number(pointCloudCrsEpsgCode)).arg(QString::number(mSRID));
        strError+=QObject::tr("\nReproject File to project CRS before");
        return(false);
    }
    if(mTempPath.isEmpty())
    {
        strError=QObject::tr("PointCloudFile::addPointCloudFiles");
        strError+=QObject::tr("\nTemporal path is empty");
        return(false);
    }
    QWidget* ptrWidget=new QWidget();
//    if(mFilePtrGeometryByIndex.contains(inputFileName))
//    {
//        strError=QObject::tr("PointCloudFile::addPointCloudFiles");
//        strError+=QObject::tr("\nExists point cloud file:\n%1").arg(inputFileName);
//        return(false);
//    }

    double minX=1000000000.0;
    double minY=1000000000.0;
    double minZ=1000000000.0;
    double maxX=-1000000000.0;
    double maxY=-1000000000.0;
    double maxZ=-1000000000.0;

    QMap<int,QMap<int,int> > tilesNumberOfPoints; // nuevos para este fichero
    if(mPtrROIsUnion!=NULL)
    {
        OGREnvelope* ptrROIsEnvelope=new OGREnvelope();
        mPtrROIsUnion->getEnvelope(ptrROIsEnvelope);
        minX=ptrROIsEnvelope->MinX;
        minY=ptrROIsEnvelope->MinY;
        maxX=ptrROIsEnvelope->MaxX;
        maxY=ptrROIsEnvelope->MaxY;
        delete(ptrROIsEnvelope);
        // se crean entradas en: mTilesTableName y mTilesTableGeometry
        if(!addTilesFromBoundingBox(minX,minY,maxX,maxY,
//                                    inputFileName,
                                    tilesNumberOfPoints,
                                    ptrWidget,strAuxError))
        {
            strError=QObject::tr("PointCloudFile::addPointCloudFiles");
            strError+=QObject::tr("\nError adding tiles from ROIs:\n%1")
                    .arg(strAuxError);
            return(false);
        }
        if(tilesNumberOfPoints.size()==0) // no hay tiles para las ROIs
        {
            return(true);
        }
    }
    else
    {
        for(int nf=0;nf<inputFileNames.size();nf++)
        {
            QString inputFileName=inputFileNames[nf];
            std::string stdFileName=inputFileName.toStdString();
            const char* charFileName=stdFileName.c_str();
            LASreadOpener lasreadopener;
            lasreadopener.set_file_name(charFileName);
            if (!lasreadopener.active())
            {
                strError=QObject::tr("\PointCloudFile::addPointCloudFile");
                strError+=QObject::tr("\nError opening file:\n%1").arg(inputFileName);
                return(false);
            }

            LASreader* lasreader = lasreadopener.open();
            LASheader* lasheader = &lasreader->header;
            int numberOfPoints=lasreader->npoints;
            double fileMinX=lasheader->min_x;
            double fileMinY=lasheader->min_y;
            double fileMaxX=lasheader->max_x;
            double fileMaxY=lasheader->max_y;
            double fileMinZ=lasheader->min_z;
            double fileMaxZ=lasheader->max_z;
            minX=fileMinX;
            minY=fileMinY;
            maxX=fileMaxX;
            maxY=fileMaxY;
            if(!addTilesFromBoundingBox(minX,minY,maxX,maxY,
//                                        inputFileName,
                                        tilesNumberOfPoints,
                                        ptrWidget,strAuxError))
            {
                strError=QObject::tr("PointCloudFile::addPointCloudFile");
                strError+=QObject::tr("\nError adding tiles from file:\n%1\nError:\n%2")
                        .arg(inputFileName).arg(strAuxError);
                return(false);
            }
//            if(tilesNumberOfPoints.size()==0) // no hay tiles, ¿fichero sin puntos?
//            {
//                return(true);
//            }
        }
    }

    mUpdateHeader=updateHeader;
    if(mPtrMpProgressDialog!=NULL)
    {
        delete(mPtrMpProgressDialog);
    }
    if(ptrWidget!=NULL)
        mPtrMpProgressDialog=new QProgressDialog(ptrWidget);
    else
        mPtrMpProgressDialog=new QProgressDialog();
    //        mNumberOfSqlsInTransaction=0;
    mNumberOfFilesToProcess=inputFileNames.size();
    QString dialogText=QObject::tr("Adding point cloud files");
    dialogText+=QObject::tr("\nNumber of point cloud files to process:%1").arg(mNumberOfFilesToProcess);
    dialogText+=QObject::tr("\n... progressing using %1 threads").arg(QThread::idealThreadCount());
    //                mPtrMpProgressDialog->setWindowTitle(title);
    mNumberOfPointsToProcessByFileName.clear();
    for(int nf=0;nf<mNumberOfFilesToProcess;nf++)
    {
        QString inputFileName=inputFileNames[nf];
        dialogText+=QObject::tr("\nPoints to process %1 in file: %2")
                .arg("All").arg(inputFileName);
        mNumberOfPointsToProcessByFileName[inputFileName]=-1;
    }
    mPtrMpProgressDialog->setLabelText(dialogText);
    mPtrMpProgressDialog->setModal(true);
    QFutureWatcher<void> futureWatcher;
    QObject::connect(&futureWatcher, SIGNAL(finished()), mPtrMpProgressDialog, SLOT(reset()));
    QObject::connect(mPtrMpProgressDialog, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));
    QObject::connect(&futureWatcher, SIGNAL(progressRangeChanged(int,int)), mPtrMpProgressDialog, SLOT(setRange(int,int)));
    QObject::connect(&futureWatcher, SIGNAL(progressValueChanged(int)), mPtrMpProgressDialog, SLOT(setValue(int)));
    //                futureWatcher.setFuture(QtConcurrent::map(fieldsValuesToRetrieve, mpLoadPhotovoltaicPanelsFromDb));
    futureWatcher.setFuture(QtConcurrent::map(inputFileNames,
                                              [this](QString& data)
    {mpAddPointCloudFile(data);}));
    // Display the dialog and start the event loop.
    mStrErrorMpProgressDialog="";
    mPtrMpProgressDialog->exec();
    futureWatcher.waitForFinished();
    delete(mPtrMpProgressDialog);
    mPtrMpProgressDialog=NULL;
    if(!mStrErrorMpProgressDialog.isEmpty())
    {
        strError=QObject::tr("PointCloudFile::addPointCloudFiles");
        strError+=QObject::tr("\nError adding point cloud files");
        strError+=QObject::tr("\nError:\n%1").arg(mStrErrorMpProgressDialog);
        return(false);
    }
    QVector<int> numberOfTilesToRemoveTileXs;
    QVector<int> numberOfTilesToRemoveTileYs;
    QMap<int,QMap<int,int> >::const_iterator iterTileX=mTilesNumberOfPoints.begin();
    while(iterTileX!=mTilesNumberOfPoints.end())
    {
        int tileX=iterTileX.key();
        QMap<int,int>::const_iterator iterTileY=iterTileX.value().begin();
        while(iterTileY!=iterTileX.value().end())
        {
            int tileY=iterTileY.key();
            int numberOfPoints=iterTileY.value();
            if(numberOfPoints==0)
            {
                if(!removeTile(iterTileX.key(),iterTileY.key(),strAuxError))
                {
                    strError=QObject::tr("PointCloudFile::addPointCloudFiles");
                    strError+=QObject::tr("Error removing tile: %1 %2\n\nError:\n%3")
                            .arg(QString::number(iterTileX.key()))
                            .arg(QString::number(iterTileY.key()))
                            .arg(strAuxError);
                    return(false);
                }
                numberOfTilesToRemoveTileXs.push_back(tileX);
                numberOfTilesToRemoveTileYs.push_back(tileY);
            }
            else
            {
                double tileDensity=((double)numberOfPoints)/pow(mGridSize,2.0);
                if(tileDensity>mMaximumDensity)
                {
                    mMaximumDensity=tileDensity;
                }
            }
            iterTileY++;
        }
        iterTileX++;
    }
    for(int ntr=0;ntr<numberOfTilesToRemoveTileXs.size();ntr++)
    {
        int tileX=numberOfTilesToRemoveTileXs[ntr];
        int tileY=numberOfTilesToRemoveTileYs[ntr];
        mTilesNumberOfPoints[tileX].remove(tileY);
        if(mTilesNumberOfPoints[tileX].size()==0)
        {
            mTilesNumberOfPoints.remove(tileX);
        }
    }
    return(true);
}

bool PointCloudFile::addROIs(QMap<QString, OGRGeometry *> ptrROIsGeometryByRoiId,
                             QString &strError)
{
    QMap<QString, OGRGeometry *>::const_iterator iter=ptrROIsGeometryByRoiId.begin();
    while(iter!=ptrROIsGeometryByRoiId.end())
    {
        QString roiId=iter.key();
        if(mPtrROIs.contains(roiId))
        {
            strError=QObject::tr("PointCloudFile::addROIs");
            strError+=QObject::tr("\nExists ROI id:\n%1").arg(roiId);
            return(false);
        }
        QStringList strValues=roiId.split(POINTCLOUDFILE_ROI_ID_STRING_SEPARATOR);
        if(strValues.size()!=3)
        {
            strError=QObject::tr("PointCloudFile::addROIs");
            strError+=QObject::tr("\nThere are no three values in ROI id:\n%1").arg(roiId);
            return(false);
        }
        QString fieldId=strValues.at(0);
        QString fieldIdValue=strValues.at(1);
        QString shapefile=strValues.at(2);
        OGRGeometry* ptrGeometry=iter.value();
        char* ptrWKT;
        if(OGRERR_NONE!=ptrGeometry->exportToWkt(&ptrWKT))
        {
            strError=QObject::tr("PointCloudFile::addROIs");
            strError+=QObject::tr("\nError exporting geometry to wkt for ROI id:\n%1").arg(roiId);
            return(false);
        }
        QString roiWkt=QString::fromLatin1(ptrWKT);

        if(mPtrROIsUnion==NULL)
        {
            mPtrROIsUnion=ptrGeometry->Union(ptrGeometry);
        }
        else
        {
            mPtrROIsUnion=mPtrROIsUnion->Union(ptrGeometry);
        }
        OGREnvelope* ptrEnvelope=new OGREnvelope();
        mPtrROIsUnion->getEnvelope(ptrEnvelope);
        double minimumFc=floor(ptrEnvelope->MinX); // aunque sea negativo uso floor
        double minimumSc=floor(ptrEnvelope->MaxX);
        delete(ptrEnvelope);
        if(minimumFc<mMinimumFc) mMinimumFc=minimumFc;
        if(minimumSc<mMinimumSc) mMinimumSc=minimumSc;
        mPtrROIs[roiId]=ptrGeometry;
        iter++;
    }
    QString strAuxError;
    if(!writeHeader(strAuxError))
    {
        strError=QObject::tr("PointCloudFile::addROIs");
        strError+=QObject::tr("\nError writing:\n%1").arg(strAuxError);
        return(false);
    }
    return(true);
}

bool PointCloudFile::addTilesFromBoundingBox(int minX,
                                             int minY,
                                             int maxX,
                                             int maxY,
//                                             QString inputFileName,
                                             QMap<int, QMap<int, int> > &tilesNumberOfPoints,
                                             QWidget *ptrWidget,
                                             QString &strError)
{
    tilesNumberOfPoints.clear();
    QString strAuxError;
    QProgressDialog* ptrProgress=NULL;
    int tilesByStep=POINTCLOUDFILE_NUMBER_OF_TILES_TO_PROCESS_BY_STEP;
    int numberOfTiles=(floor((maxX-minX)/mGridSize)+1)*(floor((maxY-minY)/mGridSize)+1);
    int numberOfSteps=ceil((double)numberOfTiles/(double)tilesByStep);
    if(numberOfSteps>1
            &&ptrWidget!=NULL)
    {
        QString title=QObject::tr("Adding tiles:");
//        QString msgGlobal=inputFileName;
//        msgGlobal+="\n";
        QString msgGlobal=QString::number(numberOfTiles,10);
        msgGlobal+=" number of tiles";
        ptrProgress=new QProgressDialog(title, "Abort",0,numberOfSteps, ptrWidget);
        ptrProgress->setWindowModality(Qt::WindowModal);
        ptrProgress->setLabelText(msgGlobal);
        ptrProgress->show();
        qApp->processEvents();
    }
    int step=0;
    int numberOfProcessedTiless=0;
    int numberOfProcessedTilesInStep=0;
    int tileX=qRound(floor(floor(minX)/mGridSize)*mGridSize);
    while(tileX<=(maxX+mGridSize))
    {
        int tileY=qRound(floor(floor(minY)/mGridSize)*mGridSize);
        while(tileY<=(maxY+mGridSize))
        {
            bool existsTile=false;
            if(mTilesName.contains(tileX))
            {
                if(mTilesName[tileX].contains(tileY))
                {
                    existsTile=true;
                }
            }
            if(!existsTile)
            {
                bool addedTile=false;
                if(!addTileTable(tileX,tileY,addedTile,strAuxError))
                {
                    strError=QObject::tr("PointCloudFile::addTilesFromBoundingBox");
                    strError+=QObject::tr("Error creating Tile table: %1 % 2\nError:\n%3")
                            .arg(QString::number(tileX)).arg(QString::number(tileY)).arg(strAuxError);
                    return(false);
                }
                if(addedTile)
                {
                    tilesNumberOfPoints[tileX][tileY]=0;
                }
            }
            tileY+=mGridSize;
            numberOfProcessedTilesInStep++;
            numberOfProcessedTiless++;
            if(numberOfProcessedTilesInStep==POINTCLOUDFILE_NUMBER_OF_TILES_TO_PROCESS_BY_STEP)
            {
                step++;
                if(numberOfSteps>1
                        &&ptrWidget!=NULL)
                {
                    ptrProgress->setValue(step);
                    qApp->processEvents();
                }
                numberOfProcessedTilesInStep=0;
            }
        }
        tileX+=mGridSize;
    }
    if(numberOfSteps>1
            &&ptrWidget!=NULL)
    {
        ptrProgress->setValue(numberOfSteps);
        qApp->processEvents();
        ptrProgress->close();
        delete(ptrProgress);
    }
    return(true);
}

bool PointCloudFile::addTileTable(int tileX,
                                  int tileY,
                                  bool& added,
                                  QString &strError)
{
    bool existsTile=false;
    if(mTilesName.contains(tileX))
    {
        if(mTilesName[tileX].contains(tileY))
        {
            existsTile=true;
        }
    }
    if(existsTile)
    {
        return(true);
    }

    QString tileTableName="tile_"+QString::number(tileX)+"_"+QString::number(tileY);
    QString wktGeometry="POLYGON((";
    wktGeometry+=QString::number(tileX);
    wktGeometry+=" ";
    wktGeometry+=QString::number(tileY);
    wktGeometry+=",";
    wktGeometry+=QString::number(tileX);
    wktGeometry+=" ";
    wktGeometry+=QString::number(qRound(tileY+mGridSize),'f',0);
    wktGeometry+=",";
    wktGeometry+=QString::number(qRound(tileX+mGridSize),'f',0);
    wktGeometry+=" ";
    wktGeometry+=QString::number(qRound(tileY+mGridSize),'f',0);
    wktGeometry+=",";
    wktGeometry+=QString::number(qRound(tileX+mGridSize),'f',0);
    wktGeometry+=" ";
    wktGeometry+=QString::number(tileY);
    wktGeometry+=",";
    wktGeometry+=QString::number(tileX);
    wktGeometry+=" ";
    wktGeometry+=QString::number(tileY);
    wktGeometry+="))";
    QByteArray byteArrayWktGeometry = wktGeometry.toUtf8();
    char *charsWktGeometry = byteArrayWktGeometry.data();
    OGRGeometry* ptrGeometry;
    ptrGeometry=OGRGeometryFactory::createGeometry(wkbPolygon);
    if(OGRERR_NONE!=ptrGeometry->importFromWkt(&charsWktGeometry))
    {
        strError=QObject::tr("PointCloudFile::addTileTable");
        strError+=QObject::tr("\nError making geometry from WKT: %1").arg(wktGeometry);
        return(false);
    }
    added=true;
    if(mPtrROIsUnion!=NULL)
    {
        added=false;
        if(mPtrROIsUnion->Contains(ptrGeometry))
        {
            added=true;
            mTilesContainedInROIs[tileX][tileY]=true;
            mTilesOverlapsWithROIs[tileX][tileY]=false;
        }
        else if(mPtrROIsUnion->Overlaps(ptrGeometry))
        {
            added=true;
            mTilesContainedInROIs[tileX][tileY]=false;
            mTilesOverlapsWithROIs[tileX][tileY]=true;
        }
    }
    if(!added)
    {
        OGRGeometryFactory::destroyGeometry(ptrGeometry);
        ptrGeometry=NULL;
        return(true);
    }
    mTilesName[tileX][tileY]=tileTableName;
    mTilesGeometry[tileX][tileY]=ptrGeometry;
    mTilesNumberOfPoints[tileX][tileY]=0;
    return(true);
}

bool PointCloudFile::create(QString path,
                            QString dbCrsDescription,
                            QString dbCrsProj4String,
                            int dbCrsEpsgCode,
                            QString heightType,
                            double gridSize,
                            QString projectType,
                            QString projectParametersString,
                            QString &strError)
{
    QDir currentDir=QDir::currentPath();
    if(currentDir.exists(path)
            &&QDir(path).entryInfoList(QDir::NoDotAndDotDot|QDir::AllEntries).count() != 0)
    {
        strError=QObject::tr("PointCloudFile::create");
        strError+=QObject::tr("\nYou must select an empty path:\n%1").arg(path);
        return(false);
    }
    if(!currentDir.mkpath(path))
    {
        strError=QObject::tr("PointCloudFile::create");
        strError+=QObject::tr("\nError making path:\n%1").arg(path);
        return(false);
    }
    mPath=path;
    mSRID=dbCrsEpsgCode;
    mCrsDescription=dbCrsDescription;
    mCrsProj4String=dbCrsProj4String;
    mHeightType=heightType;
    mGridSize=gridSize;
    mProjectType=projectType;
    mParameterValueByCode.clear();
    QStringList projectParametersList=projectParametersString.split(POINTCLOUDFILE_PROJECT_PARAMETERS_FILE_PARAMETERS_STRING_SEPARATOR);
    for(int np=0;np<projectParametersList.size();np++)
    {
        QString projectParameterString=projectParametersList.at(np);
        QStringList projectParameterList=projectParameterString.split(POINTCLOUDFILE_PROJECT_PARAMETERS_FILE_PARAMETER_VALUE_STRING_SEPARATOR);
        if(projectParameterList.size()!=2)
        {
            strError=QObject::tr("PointCloudFile::create");
            strError+=QObject::tr("\nInvalid parameters string:\n%1")
                    .arg(projectParameterString);
            return(false);
        }
        QString parameterCode=projectParameterList.at(0).trimmed();
        QString parameterValue=projectParameterList.at(1).trimmed().toLower();
        mParameterValueByCode[parameterCode]=parameterValue;
        if(parameterCode.compare(POINTCLOUDFILE_PARAMETER_COLOR_BYTES,Qt::CaseInsensitive)==0)
        {
            bool okToInt=false;
            int intValue=parameterValue.toInt(&okToInt);
            if(!okToInt)
            {
                strError=QObject::tr("PointCloudFile::create");
                strError+=QObject::tr("\nParameter %1 value is not an integer: %2")
                        .arg(parameterCode).arg(parameterValue);
                return(false);
            }
            mNumberOfColorBytes=intValue;
            continue;
        }
        QMap<QString,bool>::const_iterator iterStoredFields=mStoredFields.begin();
        while(iterStoredFields!=mStoredFields.end())
        {
            QString storeField=iterStoredFields.key();
            if(parameterCode.compare(storeField,Qt::CaseInsensitive)==0)
            {
                if(parameterValue.compare("false",Qt::CaseInsensitive)==0
                    ||parameterValue.compare("falso",Qt::CaseInsensitive)==0)
                {
                    mStoredFields[storeField]=false;
                }
                else if(parameterValue.compare("true",Qt::CaseInsensitive)==0
                        ||parameterValue.compare("verdadero",Qt::CaseInsensitive)==0)
                {
                    mStoredFields[storeField]=true;
                }
                break;
            }
            iterStoredFields++;
        }
    }
    QString strAuxError;
    if(!writeHeader(strAuxError))
    {
        strError=QObject::tr("PointCloudFile::create");
        strError+=QObject::tr("\nError:\n%1").arg(strAuxError);
        return(false);
    }
    return(true);
}

bool PointCloudFile::getPointsFromWktGeometry(QString wktGeometry,
                                              int geometryCrsEpsgCode,
                                              QString geometryCrsProj4String,
                                              QMap<int,QMap<int,QString> >& tilesTableName,
                                              QMap<int,QMap<int,QMap<int,QVector<PCFile::Point> > > >& pointsByTileByFileId,
                                              QMap<int, QMap<QString, bool> > &existsFieldsByFileId,
                                              QVector<QString> &ignoreTilesTableName,
                                              bool tilesFullGeometry,
                                              QString &strError)
{
    QWidget* ptrWidget=new QWidget();
    QProgressDialog* ptrProgress=NULL;
    mTilesFullGeometry=tilesFullGeometry;
    tilesTableName.clear();
    pointsByTileByFileId.clear();
    existsFieldsByFileId.clear();
    mMpIgnoreTilesTableName.clear();
    mMpIgnoreTilesTableName=ignoreTilesTableName;
    QVector<QString> tilesTableNames;
    QString strAuxError;
    QMap<int,QMap<int,bool> > tilesOverlaps;
    if(mMpPtrGeometry!=NULL)
    {
        OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
        mMpPtrGeometry=NULL;
    }
    QByteArray byteArrayWktGeometry = wktGeometry.toUtf8();
    char *charsWktGeometry = byteArrayWktGeometry.data();
    bool validGeometry=false;
    if(wktGeometry.toLower().contains("polygon"))
    {
        mMpPtrGeometry=OGRGeometryFactory::createGeometry(wkbPolygon);
        validGeometry=true;
    }
    if(!validGeometry)
    {
        strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
        strError+=QObject::tr("\nNot valid geometry from WKT: %1").arg(wktGeometry);
        OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
        mMpPtrGeometry=NULL;
        return(false);
    }
    if(OGRERR_NONE!=mMpPtrGeometry->importFromWkt(&charsWktGeometry))
    {
        strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
        strError+=QObject::tr("\nError making geometry from WKT: %1").arg(wktGeometry);
        OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
        mMpPtrGeometry=NULL;
        return(false);
    }
    if(geometryCrsEpsgCode!=-1)
    {
        if(geometryCrsEpsgCode!=mSRID)
        {
            QString geometryCrsDescription;
            if(!mPtrCrsTools->appendUserCrs(geometryCrsEpsgCode,
                                            geometryCrsDescription,
                                            strAuxError))
            {
                if(!mPtrCrsTools->appendUserCrs(geometryCrsProj4String,//proj4
                                                geometryCrsDescription,
                                                strAuxError))
                {
                    strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                    strError+=QObject::tr("\nInvalid CRS From EPSG code: %1 and PROJ4:\n%2")
                            .arg(QString::number(geometryCrsEpsgCode)).arg(geometryCrsProj4String);
                    OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
                    mMpPtrGeometry=NULL;
                    return(false);
                }
            }
            if(!mPtrCrsTools->crsOperation(geometryCrsDescription,
                                           mCrsDescription,
                                           &mMpPtrGeometry,
                                           strAuxError))
            {
                strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                strError+=QObject::tr("\nError in CRS operation:\n%1").arg(strAuxError);
                OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
                mMpPtrGeometry=NULL;
                return(false);
            }
        }
    }
    else
    {
        QString geometryCrsDescription;
        if(!mPtrCrsTools->appendUserCrs(geometryCrsProj4String,//proj4
                                        geometryCrsDescription,
                                        strAuxError))
        {
            strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
            strError+=QObject::tr("\nInvalid CRS From PROJ4:\n%1").arg(geometryCrsProj4String);
            OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
            mMpPtrGeometry=NULL;
            return(false);
        }
        if(!mPtrCrsTools->crsOperation(geometryCrsDescription,
                                       mCrsDescription,
                                       &mMpPtrGeometry,
                                       strAuxError))
        {
            strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
            strError+=QObject::tr("\nError in CRS operation:\n%1").arg(strAuxError);
            OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
            mMpPtrGeometry=NULL;
            return(false);
        }
    }
    if(!getTilesNamesFromGeometry(tilesTableName,
                                  ignoreTilesTableName,
                                  mMpPtrGeometry,
                                  tilesOverlaps,
                                  strAuxError))
    {
        strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
        strError+=QObject::tr("\nError recovering tiles from wkt:\n%1\nError:\n%2")
                .arg(wktGeometry).arg(strAuxError);
        OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
        mMpPtrGeometry=NULL;
        return(false);
    }

    int numberOfFilesAndTileToProcess=0;
    QMap<int,QMap<int,QVector<int> > > tilesByFileIndex;
    QMap<int,QMap<int,int> > tilesNumberOfPoints;
    QMap<int,QMap<int,QString> >::const_iterator iterX=tilesTableName.begin();
    while(iterX!=tilesTableName.end())
    {
        int tileX=iterX.key();
        QMap<int,QString>::const_iterator iterY=iterX.value().begin();
        while(iterY!=iterX.value().end())
        {
            int tileY=iterY.key();
            tilesNumberOfPoints[tileX][tileY]=0;
            QString tileTableName=iterY.value();
            QMap<int,QMap<int,QVector<int> > >::const_iterator iterFiles=mTilesByFileIndex.begin();
            while(iterFiles!=mTilesByFileIndex.end())
            {
                int fileIndex=iterFiles.key();
                QMap<int,QVector<int> > tilesInFile=iterFiles.value();
                if(tilesInFile.contains(tileX))
                {
                    if(tilesInFile[tileX].indexOf(tileY)!=-1)
                    {
                        numberOfFilesAndTileToProcess++;
                        if(!tilesByFileIndex.contains(fileIndex))
                        {
                            QMap<int,QVector<int> > aux;
                            tilesByFileIndex[fileIndex]=aux;
                        }
                        if(!tilesByFileIndex[fileIndex].contains(tileX))
                        {
                            QVector<int> aux;
                            tilesByFileIndex[fileIndex][tileX]=aux;
                        }
                        if(tilesByFileIndex[fileIndex][tileX].indexOf(tileY)==-1)
                        {
                            tilesByFileIndex[fileIndex][tileX].push_back(tileY);
                        }
                    }
                }
                iterFiles++;
            }
            iterY++;
        }
        iterX++;
    }
    int numberOfSteps=numberOfFilesAndTileToProcess;
    QDir auxDir=QDir::currentPath();
    if(ptrWidget!=NULL)
    {
        QString title=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
        QString msgGlobal=QObject::tr("Recovering points from %1 files and tiles")
                .arg(QString::number(numberOfSteps));
        ptrProgress=new QProgressDialog(title, "Abort",0,numberOfSteps, ptrWidget);
        ptrProgress->setWindowModality(Qt::WindowModal);
        ptrProgress->setLabelText(msgGlobal);
        ptrProgress->show();
        qApp->processEvents();
    }
    int step=0;
    QMap<int,QMap<int,QVector<int> > >::Iterator iterFiles=tilesByFileIndex.begin();
    while(iterFiles!=tilesByFileIndex.end())
    {
        mPointsByTile.clear();
        int fileIndex=iterFiles.key();
        if(!mClassesFileByIndex.contains(fileIndex))
        {
            strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
            strError+=QObject::tr("\nThere is no classes file for index: %1")
                    .arg(QString::number(fileIndex));
            if(ptrWidget!=NULL)
            {
                ptrProgress->close();
                delete(ptrProgress);
            }
            OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
            mMpPtrGeometry=NULL;
            return(false);
        }
        mClassesFileName=mClassesFileByIndex[fileIndex];
        QFile pointsClassFile(mClassesFileName);
        if (!pointsClassFile.open(QIODevice::ReadOnly))
        {
            strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
            strError+=QObject::tr("\nError opening file:\n%1").arg(mClassesFileName);
            if(ptrWidget!=NULL)
            {
                ptrProgress->close();
                delete(ptrProgress);
            }
            OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
            mMpPtrGeometry=NULL;
            return(false);
        }
        QDataStream inPointsClass(&pointsClassFile);
        QMap<QString,bool> existsFields;
        mTilesPointsClass.clear();
        mTilesPointsClassNewByPos.clear(); // se guarda vacío
        mTilesNop.clear();
        inPointsClass>>mTilesNop;
        inPointsClass>>existsFields;
        inPointsClass>>mTilesPointsClass;
        inPointsClass>>mTilesPointsClassNewByPos;
        pointsClassFile.close();
        mExistsColor=existsFields[POINTCLOUDFILE_PARAMETER_COLOR];
        mExistsGpsTime=existsFields[POINTCLOUDFILE_PARAMETER_GPS_TIME];
        mExistsUserData=existsFields[POINTCLOUDFILE_PARAMETER_USER_DATA];
        mExistsIntensity=existsFields[POINTCLOUDFILE_PARAMETER_INTENSITY];
        mExistsSourceId=existsFields[POINTCLOUDFILE_PARAMETER_SOURCE_ID];
        mExistsNir=existsFields[POINTCLOUDFILE_PARAMETER_NIR];
        mExistsReturn=existsFields[POINTCLOUDFILE_PARAMETER_RETURN];
        mExistsReturns=existsFields[POINTCLOUDFILE_PARAMETER_RETURNS];
        existsFieldsByFileId[fileIndex]=existsFields;
        if(!mZipFilePointsByIndex.contains(fileIndex))
        {
            strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
            strError+=QObject::tr("\nThere is no points file for index: %1")
                    .arg(QString::number(fileIndex));
            if(ptrWidget!=NULL)
            {
                ptrProgress->close();
                delete(ptrProgress);
            }
            OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
            mMpPtrGeometry=NULL;
            return(false);
        }
        mZipFileNamePoints=mZipFilePointsByIndex[fileIndex];
        QString zipFilePointsPath=mZipFilePathPointsByIndex[fileIndex];
        mZipFilePoints.setZipName(mZipFileNamePoints);
//        QuaZip mZipFilePoints(zipFileNamePoints);
        if(!mZipFilePoints.open(QuaZip::mdUnzip))
        {
            strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
            strError+=QObject::tr("\nError opening file:\n%1\nError:\n%2")
                    .arg(mZipFileNamePoints).arg(QString::number(mZipFilePoints.getZipError()));
            if(ptrWidget!=NULL)
            {
                ptrProgress->close();
                delete(ptrProgress);
            }
            OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
            mMpPtrGeometry=NULL;
            return(false);
        }
        bool useMultiProcess=mUseMultiProcess;
//        useMultiProcess=false;
        if(!useMultiProcess)
        {
            QMap<int,QVector<int> >::const_iterator iterTileX=iterFiles.value().begin();
            while(iterTileX!=iterFiles.value().end())
            {
                int tileX=iterTileX.key();
                if(!mTilesPointsClass.contains(tileX))
                {
                    strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                    strError+=QObject::tr("\nNot exists tile x: %1 in classes  file:\n%2")
                            .arg(QString::number(tileX)).arg(mClassesFileName);
                    if(ptrWidget!=NULL)
                    {
                        ptrProgress->close();
                        delete(ptrProgress);
                    }
                    OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
                    mMpPtrGeometry=NULL;
                    mZipFilePoints.close();
                    return(false);
                }
                for(int i=0;i<iterTileX.value().size();i++)
                {
                    int tileY=iterTileX.value()[i];
                    if(!mTilesPointsClass[tileX].contains(tileY))
                    {
                        strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                        strError+=QObject::tr("\nNot exists tile y: %1 for tile x: %2 in classes  file:\n%3")
                                .arg(QString::number(tileY))
                                .arg(QString::number(tileX)).arg(mClassesFileName);
                        if(ptrWidget!=NULL)
                        {
                            ptrProgress->close();
                            delete(ptrProgress);
                        }
                        OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
                        mMpPtrGeometry=NULL;
                        mZipFilePoints.close();
                        return(false);
                    }
                    step++;
                    if(ptrWidget!=NULL)
                    {
                        ptrProgress->setValue(step);
                        qApp->processEvents();
                    }
                    int numberOfPoints=mTilesNop[tileX][tileY];
                    QString tileTableName=mTilesName[tileX][tileY];
                    if(!mZipFilePoints.setCurrentFile(tileTableName))
                    {
                        strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                        strError+=QObject::tr("\nNot exists: %1 in file:\n%2\nError:\n%3")
                                .arg(tileTableName).arg(mZipFileNamePoints)
                                .arg(QString::number(mZipFilePoints.getZipError()));
                        if(ptrWidget!=NULL)
                        {
                            ptrProgress->close();
                            delete(ptrProgress);
                        }
                        OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
                        mMpPtrGeometry=NULL;
                        mZipFilePoints.close();
                        return(false);
                    }
                    QuaZipFile inPointsFile(&mZipFilePoints);
                    if (!inPointsFile.open(QIODevice::ReadOnly))
                    {
                        strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                        strError+=QObject::tr("\nError opening: %1 in file:\n%2\nError:\n%3")
                                .arg(tileTableName).arg(mZipFileNamePoints)
                                .arg(QString::number(mZipFilePoints.getZipError()));
                        if(ptrWidget!=NULL)
                        {
                            ptrProgress->close();
                            delete(ptrProgress);
                        }
                        OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
                        mMpPtrGeometry=NULL;
                        mZipFilePoints.close();
                        return(false);
                    }
                    QDataStream inPoints(&inPointsFile);
                    int pos=0;
                    QVector<PCFile::Point> pointsInTile(numberOfPoints);
                    int numberOfRealPoints=0; // porque puede haber puntos fuera del wkt
                    while(!inPoints.atEnd())
                    {
                        quint16 ix,iy;
                        quint8 z_pc,z_pa,z_pb;
                        inPoints>>ix>>iy>>z_pa>>z_pb>>z_pc;
                        PCFile::Point pto;
                        pto.setPositionInTile(pos);
                        if(pos>mTilesPointsClass[tileX][tileY].size())
                        {
                            strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                            strError+=QObject::tr("\nNot exists position: %1 in tile X: %2 tile Y: %3 in classes  file:\n%4")
                                    .arg(QString::number(pos)).arg(QString::number(tileX))
                                    .arg(QString::number(tileY)).arg(mClassesFileName);
                            if(ptrWidget!=NULL)
                            {
                                ptrProgress->close();
                                delete(ptrProgress);
                            }
                            OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
                            mMpPtrGeometry=NULL;
                            mZipFilePoints.close();
                            return(false);
                        }
                        quint8 ptoClass=mTilesPointsClass[tileX][tileY][pos];
                        pto.setClass(ptoClass);
                        quint8 ptoClassNew=ptoClass;
                        if(mTilesPointsClassNewByPos.contains(tileX))
                        {
                            if(mTilesPointsClassNewByPos[tileX].contains(tileY))
                            {
                                if(mTilesPointsClassNewByPos[tileX][tileY].contains(pos))
                                {
                                    ptoClassNew=mTilesPointsClassNewByPos[tileX][tileY][pos];
                                }
                            }
                        }
                        pto.setClassNew(ptoClassNew);
                        pto.setCoordinates(ix,iy,z_pa,z_pb,z_pc);
                        if(mExistsColor)
                        {
                            if(mNumberOfColorBytes==1)
                            {
                                quint8 color_r,color_g,color_b;
                                inPoints>>color_r>>color_g>>color_b;
                                pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_RED,color_r);
                                pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_GREEN,color_g);
                                pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_BLUE,color_b);
                            }
                            else
                            {
                                quint16 color_r,color_g,color_b;
                                inPoints>>color_r>>color_g>>color_b;
                                pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_RED,color_r);
                                pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_GREEN,color_g);
                                pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_BLUE,color_b);
                            }
                        }
                        if(mExistsGpsTime)
                        {
                            quint8 gpsDowHourPackit;
                            quint8 msb1,msb2,msb3;
                            inPoints>>gpsDowHourPackit>>msb1>>msb2>>msb3;
                            pto.setGpsTime(gpsDowHourPackit,msb1,msb2,msb3);
                        }
                        if(mExistsUserData)
                        {
                            quint8 userData;
                            inPoints>>userData;
                            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_USER_DATA,userData);
                        }
                        if(mExistsIntensity)
                        {
                            quint16 intensity;
                            inPoints>>intensity;
                            pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_INTENSITY,intensity);
                        }
                        if(mExistsSourceId)
                        {
                            quint16 sourceId;
                            inPoints>>sourceId;
                            pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_SOURCE_ID,sourceId);
                        }
                        if(mExistsNir)
                        {
                            if(mNumberOfColorBytes==1)
                            {
                                quint8 nir;
                                inPoints>>nir;
                                pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_NIR,nir);
                            }
                            else
                            {
                                quint16 nir;
                                inPoints>>nir;
                                pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_NIR,nir);
                            }
                        }
                        if(mExistsReturn)
                        {
                            quint8 returnNumber;
                            inPoints>>returnNumber;
                            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_RETURN,returnNumber);
                        }
                        if(mExistsReturns)
                        {
                            quint8 numberOfReturns;
                            inPoints>>numberOfReturns;
                            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_RETURNS,numberOfReturns);
                        }
                        pos++;
                        if(!tilesFullGeometry)
                        {
                            if(tilesOverlaps[tileX][tileY])
                            {
                                OGRGeometry* ptrPoint=NULL;
                                ptrPoint=OGRGeometryFactory::createGeometry(wkbPoint);
                                double x=tileX+ix/1000.;
                                double y=tileY+iy/1000.;
                                ((OGRPoint*)ptrPoint)->setX(x);
                                ((OGRPoint*)ptrPoint)->setY(y);
                                if(!mMpPtrGeometry->Contains(ptrPoint))
                                {
                                    OGRGeometryFactory::destroyGeometry(ptrPoint);
                                    continue;
                                }
                                OGRGeometryFactory::destroyGeometry(ptrPoint);
                            }
                        }
                        pointsInTile[numberOfRealPoints]=pto;
                        numberOfRealPoints++;
                    }
                    inPointsFile.close();
                    if(numberOfRealPoints<numberOfPoints)
                    {
                        pointsInTile.resize(numberOfRealPoints);
                    }
                    if(pointsInTile.size()>0)
                    {
                        mPointsByTile[tileX][tileY]=pointsInTile;
                    }
                    if(numberOfRealPoints>0)
                    {
                        tilesNumberOfPoints[tileX][tileY]=tilesNumberOfPoints[tileX][tileY]+numberOfRealPoints;
                    }
                }
                iterTileX++;
            }
        }
        else
        {
            QWidget* ptrWidget=new QWidget();
            mTilesXToProcess.clear();
            mTilesYToProcess.clear();
            QVector<int> tilesPosition;
            QMap<int,QVector<int> >::const_iterator iterTileX=iterFiles.value().begin();
            while(iterTileX!=iterFiles.value().end())
            {
                int tileX=iterTileX.key();
                if(!mTilesPointsClass.contains(tileX))
                {
                    strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                    strError+=QObject::tr("\nNot exists tile x: %1 in classes  file:\n%2")
                            .arg(QString::number(tileX)).arg(mClassesFileName);
                    if(ptrWidget!=NULL)
                    {
                        ptrProgress->close();
                        delete(ptrProgress);
                    }
                    OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
                    mMpPtrGeometry=NULL;
                    return(false);
                }
                for(int i=0;i<iterTileX.value().size();i++)
                {
                    int tileY=iterTileX.value()[i];
                    if(!mTilesPointsClass[tileX].contains(tileY))
                    {
                        strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                        strError+=QObject::tr("\nNot exists tile y: %1 for tile x: %2 in classes  file:\n%3")
                                .arg(QString::number(tileY))
                                .arg(QString::number(tileX)).arg(mClassesFileName);
                        if(ptrWidget!=NULL)
                        {
                            ptrProgress->close();
                            delete(ptrProgress);
                        }
                        OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
                        mMpPtrGeometry=NULL;
                        return(false);
                    }
                    mTilesXToProcess.push_back(tileX);
                    mTilesYToProcess.push_back(tileY);
                    tilesPosition.push_back(tilesPosition.size());
                }
                iterTileX++;
            }
            mMpTilesNumberOfPointsInFile.clear();

            if(mPtrMpProgressDialog!=NULL)
            {
                delete(mPtrMpProgressDialog);
            }
            if(ptrWidget!=NULL)
                mPtrMpProgressDialog=new QProgressDialog(ptrWidget);
            else
                mPtrMpProgressDialog=new QProgressDialog();
            QString dialogText=QObject::tr("Getting points from tiles in classes file:\n%1")
                    .arg(mClassesFileName);
            dialogText+=QObject::tr("\nNumber of tiles to process:%1").arg(tilesPosition.size());
            dialogText+=QObject::tr("\n... progressing using %1 threads").arg(QThread::idealThreadCount());
            mPtrMpProgressDialog->setLabelText(dialogText);
            mPtrMpProgressDialog->setModal(true);
            QFutureWatcher<void> futureWatcher;
            QObject::connect(&futureWatcher, SIGNAL(finished()), mPtrMpProgressDialog, SLOT(reset()));
            QObject::connect(mPtrMpProgressDialog, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));
            QObject::connect(&futureWatcher, SIGNAL(progressRangeChanged(int,int)), mPtrMpProgressDialog, SLOT(setRange(int,int)));
            QObject::connect(&futureWatcher, SIGNAL(progressValueChanged(int)), mPtrMpProgressDialog, SLOT(setValue(int)));
            //                futureWatcher.setFuture(QtConcurrent::map(fieldsValuesToRetrieve, mpLoadPhotovoltaicPanelsFromDb));
            futureWatcher.setFuture(QtConcurrent::map(tilesPosition,
                                                      [this](int& data)
            {mpGetPointsFromWktGeometryByTilePosition(data);}));
            mStrErrorMpProgressDialog="";
            mPtrMpProgressDialog->exec();
            futureWatcher.waitForFinished();
            delete(mPtrMpProgressDialog);
            mPtrMpProgressDialog=NULL;
            if(!mStrErrorMpProgressDialog.isEmpty())
            {
                strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                strError+=QObject::tr("\nError adding tiles geometry");
                strError+=QObject::tr("\nError:\n%1").arg(mStrErrorMpProgressDialog);
                OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
                mMpPtrGeometry=NULL;
                return(false);
            }
            mTilesXToProcess.clear();
            mTilesYToProcess.clear();
            pointsByTileByFileId[fileIndex]=mPointsByTile;
            {
                QMap<int,QMap<int,int> >::iterator iterX=mMpTilesNumberOfPointsInFile.begin();
                while(iterX!=mMpTilesNumberOfPointsInFile.end())
                {
                    int tileX=iterX.key();
                    QMap<int,int>::iterator iterY=iterX.value().begin();
                    while(iterY!=iterX.value().end())
                    {
                        int tileY=iterY.key();
                        int numberOfPreviousPoints=0;
                        if(tilesNumberOfPoints.contains(tileX))
                        {
                            if(tilesNumberOfPoints[tileX].contains(tileY))
                            {
                                numberOfPreviousPoints=tilesNumberOfPoints[tileX][tileY];
                            }
                        }
                        int numberOfPoints=mMpTilesNumberOfPointsInFile[tileX][tileY];
                        tilesNumberOfPoints[tileX][tileY]=numberOfPreviousPoints+numberOfPoints;
                        iterY++;
                    }
                    iterX++;
                }
            }
        }
        mZipFilePoints.close();
        iterFiles++;
    }
    OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
    mMpPtrGeometry=NULL;
    if(ptrWidget!=NULL)
    {
        ptrProgress->close();
        delete(ptrProgress);
    }
    {
        QMap<int,QMap<int,int> >::iterator iterX=tilesNumberOfPoints.begin();
        while(iterX!=tilesNumberOfPoints.end())
        {
            int tileX=iterX.key();
            QMap<int,int>::iterator iterY=iterX.value().begin();
            while(iterY!=iterX.value().end())
            {
                int tileY=iterY.key();
                if(iterY.value()==0)
                {
                    tilesTableName[tileX].remove(tileY);
                    if(tilesTableName[tileX].size()==0)
                    {
                        tilesTableName.remove(tileX);
                    }
                }
                iterY++;
            }
            iterX++;
        }
    }
    return(true);
}

/*
bool PointCloudFile::getPointsFromWktGeometry(QString wktGeometry,
                                              int geometryCrsEpsgCode,
                                              QString geometryCrsProj4String,
                                              QMap<int,QMap<int,QString> >& tilesTableName,
                                              QMap<int,QMap<int,QMap<int,QVector<PCFile::Point> > > >& pointsByTileByFileId,
                                              QMap<int, QMap<QString, bool> > &existsFieldsByFileId,
                                              QVector<QString> &ignoreTilesTableName,
                                              bool tilesFullGeometry,
                                              QString &strError)
{
    QWidget* ptrWidget=new QWidget();
    QProgressDialog* ptrProgress=NULL;
    tilesTableName.clear();
    pointsByTileByFileId.clear();
    existsFieldsByFileId.clear();
    QVector<QString> tilesTableNames;
    QString strAuxError;
    QMap<int,QMap<int,bool> > tilesOverlaps;
    OGRGeometry* ptrGeometry=NULL;
    if(!getTilesNamesFromWktGeometry(wktGeometry,
                                     geometryCrsEpsgCode,
                                     geometryCrsProj4String,
                                     tilesTableName,
                                     ignoreTilesTableName,
                                     tilesFullGeometry,
                                     &ptrGeometry,
                                     tilesOverlaps,
                                     strAuxError))
    {
        strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
        strError+=QObject::tr("\nError recovering tiles from wkt:\n%1\nError:\n%2")
                .arg(wktGeometry).arg(strAuxError);
        return(false);
    }
    int numberOfFilesAndTileToProcess=0;
    QMap<int,QMap<int,QVector<int> > > tilesByFileIndex;
    QMap<int,QMap<int,QString> >::const_iterator iterX=tilesTableName.begin();
    while(iterX!=tilesTableName.end())
    {
        int tileX=iterX.key();
        QMap<int,QString>::const_iterator iterY=iterX.value().begin();
        while(iterY!=iterX.value().end())
        {
            int tileY=iterY.key();
            QString tileTableName=iterY.value();
            QMap<int,QMap<int,QVector<int> > >::const_iterator iterFiles=mTilesByFileIndex.begin();
            while(iterFiles!=mTilesByFileIndex.end())
            {
                int fileIndex=iterFiles.key();
                QMap<int,QVector<int> > tilesInFile=iterFiles.value();
                if(tilesInFile.contains(tileX))
                {
                    if(tilesInFile[tileX].indexOf(tileY)!=-1)
                    {
                        numberOfFilesAndTileToProcess++;
                        if(!tilesByFileIndex.contains(fileIndex))
                        {
                            QMap<int,QVector<int> > aux;
                            tilesByFileIndex[fileIndex]=aux;
                        }
                        if(!tilesByFileIndex[fileIndex].contains(tileX))
                        {
                            QVector<int> aux;
                            tilesByFileIndex[fileIndex][tileX]=aux;
                        }
                        if(tilesByFileIndex[fileIndex][tileX].indexOf(tileY)==-1)
                        {
                            tilesByFileIndex[fileIndex][tileX].push_back(tileY);
                        }
                    }
                }
                iterFiles++;
            }
            iterY++;
        }
        iterX++;
    }
    int numberOfSteps=numberOfFilesAndTileToProcess;
    QDir auxDir=QDir::currentPath();
    if(ptrWidget!=NULL)
    {
        QString title=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
        QString msgGlobal=QObject::tr("Recovering points from %1 files and tiles")
                .arg(QString::number(numberOfSteps));
        ptrProgress=new QProgressDialog(title, "Abort",0,numberOfSteps, ptrWidget);
        ptrProgress->setWindowModality(Qt::WindowModal);
        ptrProgress->setLabelText(msgGlobal);
        ptrProgress->show();
        qApp->processEvents();
    }
    int step=0;
    QMap<int,QMap<int,QVector<int> > >::const_iterator iterFiles=tilesByFileIndex.begin();
    while(iterFiles!=tilesByFileIndex.end())
    {
        QMap<int,QMap<int,QVector<PCFile::Point> > > pointsByTile;
        int fileIndex=iterFiles.key();
        if(!mClassesFileByIndex.contains(fileIndex))
        {
            strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
            strError+=QObject::tr("\nThere is no classes file for index: %1")
                    .arg(QString::number(fileIndex));
            if(ptrWidget!=NULL)
            {
                ptrProgress->close();
                delete(ptrProgress);
            }
            return(false);
        }
        QString classesFileName=mClassesFileByIndex[fileIndex];
        QFile pointsClassFile(classesFileName);
        if (!pointsClassFile.open(QIODevice::ReadOnly))
        {
            strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
            strError+=QObject::tr("\nError opening file:\n%1").arg(classesFileName);
            if(ptrWidget!=NULL)
            {
                ptrProgress->close();
                delete(ptrProgress);
            }
            return(false);
        }
        QDataStream inPointsClass(&pointsClassFile);
        QMap<QString,bool> existsFields;
        QMap<int,QMap<int,QVector<quint8> > > tilesPointsClass;
        QMap<int,QMap<int,QMap<int,quint8> > > tilesPointsClassNewByPos; // se guarda vacío
        QMap<int,QMap<int,int> > tilesNop;
        inPointsClass>>tilesNop;
        inPointsClass>>existsFields;
        inPointsClass>>tilesPointsClass;
        inPointsClass>>tilesPointsClassNewByPos;
        pointsClassFile.close();
        bool existsColor=existsFields[POINTCLOUDFILE_PARAMETER_COLOR];
        bool existsGpsTime=existsFields[POINTCLOUDFILE_PARAMETER_GPS_TIME];
        bool existsUserData=existsFields[POINTCLOUDFILE_PARAMETER_USER_DATA];
        bool existsIntensity=existsFields[POINTCLOUDFILE_PARAMETER_INTENSITY];
        bool existsSourceId=existsFields[POINTCLOUDFILE_PARAMETER_SOURCE_ID];
        bool existsNir=existsFields[POINTCLOUDFILE_PARAMETER_NIR];
        bool existsReturn=existsFields[POINTCLOUDFILE_PARAMETER_RETURN];
        bool existsReturns=existsFields[POINTCLOUDFILE_PARAMETER_RETURNS];
        if(!mZipFilePointsByIndex.contains(fileIndex))
        {
            strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
            strError+=QObject::tr("\nThere is no points file for index: %1")
                    .arg(QString::number(fileIndex));
            if(ptrWidget!=NULL)
            {
                ptrProgress->close();
                delete(ptrProgress);
            }
            return(false);
        }
        QString zipFileNamePoints=mZipFilePointsByIndex[fileIndex];
        QString zipFilePointsPath=mZipFilePathPointsByIndex[fileIndex];
        QuaZip zipFilePoints(zipFileNamePoints);
        if(!zipFilePoints.open(QuaZip::mdUnzip))
        {
            strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
            strError+=QObject::tr("\nError opening file:\n%1\nError:\n%2")
                    .arg(zipFileNamePoints).arg(QString::number(zipFilePoints.getZipError()));
            if(ptrWidget!=NULL)
            {
                ptrProgress->close();
                delete(ptrProgress);
            }
            return(false);
        }
        QMap<int,QVector<int> >::const_iterator iterTileX=iterFiles.value().begin();
        while(iterTileX!=iterFiles.value().end())
        {
            int tileX=iterTileX.key();
            if(!tilesPointsClass.contains(tileX))
            {
                strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                strError+=QObject::tr("\nNot exists tile x: %1 in classes  file:\n%2")
                        .arg(QString::number(tileX)).arg(classesFileName);
                if(ptrWidget!=NULL)
                {
                    ptrProgress->close();
                    delete(ptrProgress);
                }
                return(false);
            }
            for(int i=0;i<iterTileX.value().size();i++)
            {
                int tileY=iterTileX.value()[i];
                if(!tilesPointsClass[tileX].contains(tileY))
                {
                    strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                    strError+=QObject::tr("\nNot exists tile y: %1 for tile x: %2 in classes  file:\n%3")
                            .arg(QString::number(tileY))
                            .arg(QString::number(tileX)).arg(classesFileName);
                    if(ptrWidget!=NULL)
                    {
                        ptrProgress->close();
                        delete(ptrProgress);
                    }
                    return(false);
                }
                step++;
                if(ptrWidget!=NULL)
                {
                    ptrProgress->setValue(step);
                    qApp->processEvents();
                }
                int numberOfPoints=tilesNop[tileX][tileY];
                QString tileTableName=mTilesName[tileX][tileY];
                if(!zipFilePoints.setCurrentFile(tileTableName))
                {
                    strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                    strError+=QObject::tr("\nNot exists: %1 in file:\n%2\nError:\n%3")
                            .arg(tileTableName).arg(zipFileNamePoints)
                            .arg(QString::number(zipFilePoints.getZipError()));
                    if(ptrWidget!=NULL)
                    {
                        ptrProgress->close();
                        delete(ptrProgress);
                    }
                    return(false);
                }
                QuaZipFile inPointsFile(&zipFilePoints);
                if (!inPointsFile.open(QIODevice::ReadOnly))
                {
                    strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                    strError+=QObject::tr("\nError opening: %1 in file:\n%2\nError:\n%3")
                            .arg(tileTableName).arg(zipFileNamePoints)
                            .arg(QString::number(zipFilePoints.getZipError()));
                    if(ptrWidget!=NULL)
                    {
                        ptrProgress->close();
                        delete(ptrProgress);
                    }
                    return(false);
                }
                QDataStream inPoints(&inPointsFile);
                int pos=0;
                QVector<PCFile::Point> pointsInTile(numberOfPoints);
                int numberOfRealPoints=0; // porque puede haber puntos fuera del wkt
                while(!inPoints.atEnd())
                {
                    quint16 ix,iy;
                    quint8 z_pc,z_pa,z_pb;
                    inPoints>>ix>>iy>>z_pa>>z_pb>>z_pc;
                    if(!tilesFullGeometry)
                    {
                        if(tilesOverlaps[tileX][tileY])
                        {
                            OGRGeometry* ptrPoint=NULL;
                            ptrPoint=OGRGeometryFactory::createGeometry(wkbPoint);
                            double x=tileX+ix/1000.;
                            double y=tileY+iy/1000.;
                            ((OGRPoint*)ptrPoint)->setX(x);
                            ((OGRPoint*)ptrPoint)->setY(y);
                            if(!ptrGeometry->Contains(ptrPoint))
                            {
                                OGRGeometryFactory::destroyGeometry(ptrPoint);
                                continue;
                            }
                            OGRGeometryFactory::destroyGeometry(ptrPoint);
                        }
                    }
                    PCFile::Point pto;
                    pto.setPositionInTile(pos);
                    if(pos>tilesPointsClass[tileX][tileY].size())
                    {
                        strError=QObject::tr("PointCloudFile::getPointsFromWktGeometry");
                        strError+=QObject::tr("\nNot exists position: %1 in tile X: %2 tile Y: %3 in classes  file:\n%4")
                                .arg(QString::number(pos)).arg(QString::number(tileX))
                                .arg(QString::number(tileY)).arg(classesFileName);
                        if(ptrWidget!=NULL)
                        {
                            ptrProgress->close();
                            delete(ptrProgress);
                        }
                        return(false);
                    }
                    quint8 ptoClass=tilesPointsClass[tileX][tileY][pos];
                    pto.setClass(ptoClass);
                    quint8 ptoClassNew=ptoClass;
                    if(tilesPointsClassNewByPos.contains(tileX))
                    {
                        if(tilesPointsClassNewByPos[tileX].contains(tileY))
                        {
                            if(tilesPointsClassNewByPos[tileX][tileY].contains(pos))
                            {
                                ptoClassNew=tilesPointsClassNewByPos[tileX][tileY][pos];
                            }
                        }
                    }
                    pto.setClassNew(ptoClassNew);
                    pto.setCoordinates(ix,iy,z_pa,z_pb,z_pc);
                    if(existsColor)
                    {
                        if(mNumberOfColorBytes==1)
                        {
                            quint8 color_r,color_g,color_b;
                            inPoints>>color_r>>color_g>>color_b;
                            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_RED,color_r);
                            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_GREEN,color_g);
                            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_BLUE,color_b);
                        }
                        else
                        {
                            quint16 color_r,color_g,color_b;
                            inPoints>>color_r>>color_g>>color_b;
                            pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_RED,color_r);
                            pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_GREEN,color_g);
                            pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_BLUE,color_b);
                        }
                    }
                    if(existsGpsTime)
                    {
                        quint8 gpsDowHourPackit;
                        quint8 msb1,msb2,msb3;
                        inPoints>>gpsDowHourPackit>>msb1>>msb2>>msb3;
                        pto.setGpsTime(gpsDowHourPackit,msb1,msb2,msb3);
                    }
                    if(existsUserData)
                    {
                        quint8 userData;
                        inPoints>>userData;
                        pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_USER_DATA,userData);
                    }
                    if(existsIntensity)
                    {
                        quint16 intensity;
                        inPoints>>intensity;
                        pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_INTENSITY,intensity);
                    }
                    if(existsSourceId)
                    {
                        quint16 sourceId;
                        inPoints>>sourceId;
                        pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_SOURCE_ID,sourceId);
                    }
                    if(existsNir)
                    {
                        if(mNumberOfColorBytes==1)
                        {
                            quint8 nir;
                            inPoints>>nir;
                            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_NIR,nir);
                        }
                        else
                        {
                            quint16 nir;
                            inPoints>>nir;
                            pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_NIR,nir);
                        }
                    }
                    if(existsReturn)
                    {
                        quint8 returnNumber;
                        inPoints>>returnNumber;
                        pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_RETURN,returnNumber);
                    }
                    if(existsReturns)
                    {
                        quint8 numberOfReturns;
                        inPoints>>numberOfReturns;
                        pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_RETURNS,numberOfReturns);
                    }
                    pointsInTile[numberOfRealPoints]=pto;
                    numberOfRealPoints++;
                    pos++;
                }
                inPointsFile.close();
                if(numberOfRealPoints<numberOfPoints)
                {
                    pointsInTile.resize(numberOfRealPoints);
                }
                if(pointsInTile.size()>0)
                {
                    pointsByTile[tileX][tileY]=pointsInTile;
                }
            }
            iterTileX++;
        }
        zipFilePoints.close();
        if(pointsByTile.size()>0)
        {
            pointsByTileByFileId[fileIndex]=pointsByTile;
            existsFieldsByFileId[fileIndex]=existsFields;
        }
        iterFiles++;
    }
    if(!tilesFullGeometry)
    {
        OGRGeometryFactory::destroyGeometry(ptrGeometry);
    }
    if(ptrWidget!=NULL)
    {
        ptrProgress->close();
        delete(ptrProgress);
    }
    return(true);
}
*/

bool PointCloudFile::getROIsWktGeometry(QMap<QString,QString> &values,
                                        QString &strError)
{
    values.clear();
    if(mPtrROIs.size()>0
            &&(mROIsWkt.size()!=mPtrROIs.size()))
    {
        QMap<QString,OGRGeometry*>::const_iterator iterPtrRois=mPtrROIs.begin();
        while(iterPtrRois!=mPtrROIs.end())
        {
            QString roiId=iterPtrRois.key();
            if(!mROIsWkt.contains(roiId))
            {
                OGRGeometry* ptrGeometry=iterPtrRois.value();
                char* ptrWKT;
                if(OGRERR_NONE!=ptrGeometry->exportToWkt(&ptrWKT))
                {
                    strError=QObject::tr("PointCloudFile::getWktROIs");
                    strError+=QObject::tr("\nError exporting geometry to wkt for ROI id:\n%1").arg(roiId);
                    return(false);
                }
                QString roiWkt=QString::fromLatin1(ptrWKT);
                mROIsWkt[roiId]=roiWkt;
            }
            iterPtrRois++;
        }
    }
    values=mROIsWkt;
    return(true);
}

bool PointCloudFile::getTilesNamesFromWktGeometry(QString wktGeometry,
                                                       int geometryCrsEpsgCode,
                                                       QString geometryCrsProj4String,
                                                       QMap<int, QMap<int, QString> > &tilesTableName,
                                                       QVector<QString> &ignoreTilesTableName,
                                                       bool tilesFullGeometry,
                                                       OGRGeometry** ptrGeometry,
                                                       QMap<int, QMap<int, bool> > &tilesOverlaps,
                                                       QString &strError)
{
    tilesTableName.clear();
    tilesOverlaps.clear();
    QByteArray byteArrayWktGeometry = wktGeometry.toUtf8();
    char *charsWktGeometry = byteArrayWktGeometry.data();
//    OGRGeometry* ptrGeometry;
    bool validGeometry=false;
    QString strAuxError;
    if(wktGeometry.toLower().contains("polygon"))
    {
        (*ptrGeometry)=OGRGeometryFactory::createGeometry(wkbPolygon);
        validGeometry=true;
    }
    if(!validGeometry)
    {
        strError=QObject::tr("PointCloudFile::getTilesTableNamesFromWktGeometry");
        strError+=QObject::tr("\nNot valid geometry from WKT: %1").arg(wktGeometry);
        return(false);
    }
    if(OGRERR_NONE!=(*ptrGeometry)->importFromWkt(&charsWktGeometry))
    {
        strError=QObject::tr("PointCloudFile::getTilesTableNamesFromWktGeometry");
        strError+=QObject::tr("\nError making geometry from WKT: %1").arg(wktGeometry);
        return(false);
    }
    if(geometryCrsEpsgCode!=-1)
    {
        if(geometryCrsEpsgCode!=mSRID)
        {
            QString geometryCrsDescription;
            if(!mPtrCrsTools->appendUserCrs(geometryCrsEpsgCode,
                                            geometryCrsDescription,
                                            strAuxError))
            {
                if(!mPtrCrsTools->appendUserCrs(geometryCrsProj4String,//proj4
                                                geometryCrsDescription,
                                                strAuxError))
                {
                    strError=QObject::tr("PointCloudFile::getTilesTableNamesFromWktGeometry");
                    strError+=QObject::tr("\nInvalid CRS From EPSG code: %1 and PROJ4:\n%2")
                            .arg(QString::number(geometryCrsEpsgCode)).arg(geometryCrsProj4String);
                    return(false);
                }
            }
            if(!mPtrCrsTools->crsOperation(geometryCrsDescription,
                                           mCrsDescription,
                                           ptrGeometry,
                                           strAuxError))
            {
                strError=QObject::tr("PointCloudFile::getTilesTableNamesFromWktGeometry");
                strError+=QObject::tr("\nError in CRS operation:\n%1").arg(strAuxError);
                return(false);
            }
        }
    }
    else
    {
        QString geometryCrsDescription;
        if(!mPtrCrsTools->appendUserCrs(geometryCrsProj4String,//proj4
                                        geometryCrsDescription,
                                        strAuxError))
        {
            strError=QObject::tr("PointCloudFile::getTilesTableNamesFromWktGeometry");
            strError+=QObject::tr("\nInvalid CRS From PROJ4:\n%1").arg(geometryCrsProj4String);
            return(false);
        }
        if(!mPtrCrsTools->crsOperation(geometryCrsDescription,
                                       mCrsDescription,
                                       ptrGeometry,
                                       strAuxError))
        {
            strError=QObject::tr("PointCloudFile::getTilesTableNamesFromWktGeometry");
            strError+=QObject::tr("\nError in CRS operation:\n%1").arg(strAuxError);
            return(false);
        }
    }
    OGRwkbGeometryType geometryType=(*ptrGeometry)->getGeometryType();
    QMap<int,QMap<int,OGRGeometry*> >::const_iterator iter1=mTilesGeometry.begin();
    while(iter1!=mTilesGeometry.end())
    {
        int tileX=iter1.key();
        QMap<int,OGRGeometry*>::const_iterator iter2=iter1.value().begin();
        while(iter2!=iter1.value().end())
        {
            if(geometryType==wkbPolygon
                    ||geometryType==wkbMultiPolygon
                    ||geometryType==wkbPolygon25D
                    ||geometryType==wkbMultiPolygon25D
                    ||geometryType==wkbPolygonM
                    ||geometryType==wkbMultiPolygonM
                    ||geometryType==wkbPolygonZM
                    ||geometryType==wkbMultiPolygonZM)
            {
                int tileY=iter2.key();
                QString tileTableName=mTilesName[tileX][tileY];
                if(ignoreTilesTableName.indexOf(tileTableName)!=-1)
                {
                    iter2++;
                    continue;
                }
                if((*ptrGeometry)->Overlaps(iter2.value()))
                {
                    tilesTableName[tileX][tileY]=tileTableName;
                    tilesOverlaps[tileX][tileY]=true;
                }
                else if((*ptrGeometry)->Contains(iter2.value()))
                {
                    tilesTableName[tileX][tileY]=tileTableName;
                    tilesOverlaps[tileX][tileY]=false;
                }
                else if((*ptrGeometry)->Within(iter2.value()))
                {
                    tilesTableName[tileX][tileY]=tileTableName;
                    tilesOverlaps[tileX][tileY]=false;
                }
            }
            iter2++;
        }
        iter1++;
    }
    if(tilesFullGeometry)
    {
        OGRGeometryFactory::destroyGeometry((*ptrGeometry));
        (*ptrGeometry)=NULL;
    }
//    else
//    {
//        ptrPtrGeometry=&ptrGeometry;
//    }
    return(true);
}

bool PointCloudFile::getTilesNamesFromGeometry(QMap<int, QMap<int, QString> > &tilesTableName,
                                               QVector<QString> &ignoreTilesTableName,
                                               OGRGeometry* ptrGeometry,
                                               QMap<int, QMap<int, bool> > &tilesOverlaps,
                                               QString &strError)
{
    tilesTableName.clear();
    tilesOverlaps.clear();
    if(!mUseMultiProcess)
    {
        OGRwkbGeometryType geometryType=ptrGeometry->getGeometryType();
        QMap<int,QMap<int,OGRGeometry*> >::const_iterator iter1=mTilesGeometry.begin();
        while(iter1!=mTilesGeometry.end())
        {
            int tileX=iter1.key();
            QMap<int,OGRGeometry*>::const_iterator iter2=iter1.value().begin();
            while(iter2!=iter1.value().end())
            {
                if(geometryType==wkbPolygon
                        ||geometryType==wkbMultiPolygon
                        ||geometryType==wkbPolygon25D
                        ||geometryType==wkbMultiPolygon25D
                        ||geometryType==wkbPolygonM
                        ||geometryType==wkbMultiPolygonM
                        ||geometryType==wkbPolygonZM
                        ||geometryType==wkbMultiPolygonZM)
                {
                    int tileY=iter2.key();
                    QString tileTableName=mTilesName[tileX][tileY];
                    if(ignoreTilesTableName.indexOf(tileTableName)!=-1)
                    {
                        iter2++;
                        continue;
                    }
                    if(ptrGeometry->Overlaps(iter2.value()))
                    {
                        tilesTableName[tileX][tileY]=tileTableName;
                        tilesOverlaps[tileX][tileY]=true;
                    }
                    else if(ptrGeometry->Contains(iter2.value()))
                    {
                        tilesTableName[tileX][tileY]=tileTableName;
                        tilesOverlaps[tileX][tileY]=false;
                    }
                    else if(ptrGeometry->Within(iter2.value()))
                    {
                        tilesTableName[tileX][tileY]=tileTableName;
                        tilesOverlaps[tileX][tileY]=false;
                    }
                }
                iter2++;
            }
            iter1++;
        }
    }
    else
    {
        QWidget* ptrWidget=new QWidget();
        mMpTilesTableName.clear();
        mTilesOverlaps.clear();
        mTilesXToProcess.clear();
        mTilesYToProcess.clear();
        QVector<int> tilesPosition;
        QMap<int,QMap<int,OGRGeometry*> >::const_iterator  iterTilesX=mTilesGeometry.begin();
        while(iterTilesX!=mTilesGeometry.end())
        {
            int tileX=iterTilesX.key();
            QMap<int,OGRGeometry*>::const_iterator iterTilesY=iterTilesX.value().begin();
            while(iterTilesY!=iterTilesX.value().end())
            {
                int tileY=iterTilesY.key();
                mTilesXToProcess.push_back(tileX);
                mTilesYToProcess.push_back(tileY);
                tilesPosition.push_back(tilesPosition.size());
                iterTilesY++;
            }
            iterTilesX++;
        }
        if(mPtrMpProgressDialog!=NULL)
        {
            delete(mPtrMpProgressDialog);
        }
        if(ptrWidget!=NULL)
            mPtrMpProgressDialog=new QProgressDialog(ptrWidget);
        else
            mPtrMpProgressDialog=new QProgressDialog();
        QString dialogText=QObject::tr("Getting tiles names from WKT geometry");
        dialogText+=QObject::tr("\nNumber of tiles to process:%1").arg(tilesPosition.size());
        dialogText+=QObject::tr("\n... progressing using %1 threads").arg(QThread::idealThreadCount());
        mPtrMpProgressDialog->setLabelText(dialogText);
        mPtrMpProgressDialog->setModal(true);
        QFutureWatcher<void> futureWatcher;
        QObject::connect(&futureWatcher, SIGNAL(finished()), mPtrMpProgressDialog, SLOT(reset()));
        QObject::connect(mPtrMpProgressDialog, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));
        QObject::connect(&futureWatcher, SIGNAL(progressRangeChanged(int,int)), mPtrMpProgressDialog, SLOT(setRange(int,int)));
        QObject::connect(&futureWatcher, SIGNAL(progressValueChanged(int)), mPtrMpProgressDialog, SLOT(setValue(int)));
        //                futureWatcher.setFuture(QtConcurrent::map(fieldsValuesToRetrieve, mpLoadPhotovoltaicPanelsFromDb));
        futureWatcher.setFuture(QtConcurrent::map(tilesPosition,
                                                  [this](int& data)
        {mpGetTilesNamesFromWktGeometry(data);}));
        mStrErrorMpProgressDialog="";
        mPtrMpProgressDialog->exec();
        futureWatcher.waitForFinished();
        delete(mPtrMpProgressDialog);
        mPtrMpProgressDialog=NULL;
        if(!mStrErrorMpProgressDialog.isEmpty())
        {
            strError=QObject::tr("PointCloudFile::getTilesNamesFromGeometry");
            strError+=QObject::tr("\nError adding tiles geometry");
            strError+=QObject::tr("\nError:\n%1").arg(mStrErrorMpProgressDialog);
            return(false);
        }
        {
            QMap<int, QMap<int, QString> >::iterator iterTilesX=mMpTilesTableName.begin();
            while(iterTilesX!=mMpTilesTableName.end())
            {
                 QMap<int, QString>::iterator iterTilesY=iterTilesX.value().begin();
                 while(iterTilesY!=iterTilesX.value().end())
                 {
                     tilesTableName[iterTilesX.key()][iterTilesY.key()]=iterTilesY.value();
                     iterTilesY++;
                 }
                 iterTilesX++;
            }
        }
        {
            QMap<int, QMap<int, bool> >::iterator iterTilesX=mTilesOverlaps.begin();
            while(iterTilesX!=mTilesOverlaps.end())
            {
                 QMap<int, bool>::iterator iterTilesY=iterTilesX.value().begin();
                 while(iterTilesY!=iterTilesX.value().end())
                 {
                     tilesOverlaps[iterTilesX.key()][iterTilesY.key()]=iterTilesY.value();
                     iterTilesY++;
                 }
                 iterTilesX++;
            }
        }
        mMpTilesTableName.clear();
        mTilesOverlaps.clear();
    }
    return(true);
}

bool PointCloudFile::getTilesNamesFromWktGeometry(QString wktGeometry,
                                                  int geometryCrsEpsgCode,
                                                  QString geometryCrsProj4String,
                                                  QMap<int, QMap<int, QString> > &tilesTableName,
                                                  QString &strError)
{
    tilesTableName.clear();
    QByteArray byteArrayWktGeometry = wktGeometry.toUtf8();
    char *charsWktGeometry = byteArrayWktGeometry.data();
    if(mMpPtrGeometry!=NULL)
    {
        OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
        mMpPtrGeometry=NULL;
    }
    bool validGeometry=false;
    QString strAuxError;
    if(wktGeometry.toLower().contains("polygon"))
    {
        mMpPtrGeometry=OGRGeometryFactory::createGeometry(wkbPolygon);
        validGeometry=true;
    }
    if(!validGeometry)
    {
        strError=QObject::tr("PointCloudFile::getTilesFromWktGeometry");
        strError+=QObject::tr("\nNot valid geometry from WKT: %1").arg(wktGeometry);
        return(false);
    }
    if(OGRERR_NONE!=mMpPtrGeometry->importFromWkt(&charsWktGeometry))
    {
        strError=QObject::tr("PointCloudFile::getTilesFromWktGeometry");
        strError+=QObject::tr("\nError making geometry from WKT: %1").arg(wktGeometry);
        OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
        mMpPtrGeometry=NULL;
        return(false);
    }
    if(geometryCrsEpsgCode!=-1)
    {
        if(geometryCrsEpsgCode!=mSRID)
        {
            QString geometryCrsDescription;
            if(!mPtrCrsTools->appendUserCrs(geometryCrsEpsgCode,
                                            geometryCrsDescription,
                                            strAuxError))
            {
                if(!mPtrCrsTools->appendUserCrs(geometryCrsProj4String,//proj4
                                                geometryCrsDescription,
                                                strAuxError))
                {
                    strError=QObject::tr("PointCloudFile::getTilesFromWktGeometry");
                    strError+=QObject::tr("\nInvalid CRS From EPSG code: %1 and PROJ4:\n%2")
                            .arg(QString::number(geometryCrsEpsgCode)).arg(geometryCrsProj4String);
                    OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
                    mMpPtrGeometry=NULL;
                    return(false);
                }
            }
            if(!mPtrCrsTools->crsOperation(geometryCrsDescription,
                                           mCrsDescription,
                                           &mMpPtrGeometry,
                                           strAuxError))
            {
                strError=QObject::tr("PointCloudFile::getTilesFromWktGeometry");
                strError+=QObject::tr("\nError in CRS operation:\n%1").arg(strAuxError);
                OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
                mMpPtrGeometry=NULL;
                return(false);
            }
        }
    }
    else
    {
        QString geometryCrsDescription;
        if(!mPtrCrsTools->appendUserCrs(geometryCrsProj4String,//proj4
                                        geometryCrsDescription,
                                        strAuxError))
        {
            strError=QObject::tr("PointCloudFile::getTilesFromWktGeometry");
            strError+=QObject::tr("\nInvalid CRS From PROJ4:\n%1").arg(geometryCrsProj4String);
            OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
            mMpPtrGeometry=NULL;
            return(false);
        }
        if(!mPtrCrsTools->crsOperation(geometryCrsDescription,
                                       mCrsDescription,
                                       &mMpPtrGeometry,
                                       strAuxError))
        {
            strError=QObject::tr("PointCloudFile::getTilesFromWktGeometry");
            strError+=QObject::tr("\nError in CRS operation:\n%1").arg(strAuxError);
            OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
            mMpPtrGeometry=NULL;
            return(false);
        }
    }
    if(!mUseMultiProcess)
    {
        OGRwkbGeometryType geometryType=mMpPtrGeometry->getGeometryType();
        QMap<int,QMap<int,OGRGeometry*> >::const_iterator iter1=mTilesGeometry.begin();
        while(iter1!=mTilesGeometry.end())
        {
            int tileX=iter1.key();
            QMap<int,OGRGeometry*>::const_iterator iter2=iter1.value().begin();
            while(iter2!=iter1.value().end())
            {
                if(geometryType==wkbPolygon
                        ||geometryType==wkbMultiPolygon
                        ||geometryType==wkbPolygon25D
                        ||geometryType==wkbMultiPolygon25D
                        ||geometryType==wkbPolygonM
                        ||geometryType==wkbMultiPolygonM
                        ||geometryType==wkbPolygonZM
                        ||geometryType==wkbMultiPolygonZM)
                {
                    int tileY=iter2.key();
                    QString tileTableName=mTilesName[tileX][tileY];
                    if(mMpPtrGeometry->Overlaps(iter2.value()))
                    {
                        tilesTableName[tileX][tileY]=tileTableName;
                    }
                    else if(mMpPtrGeometry->Contains(iter2.value()))
                    {
                        tilesTableName[tileX][tileY]=tileTableName;
                    }
                    else if(mMpPtrGeometry->Within(iter2.value()))
                    {
                        tilesTableName[tileX][tileY]=tileTableName;
                    }
                }
                iter2++;
            }
            iter1++;
        }
    }
    else
    {
        QWidget* ptrWidget=new QWidget();
        mMpTilesTableName.clear();
        mTilesXToProcess.clear();
        mTilesYToProcess.clear();
        QVector<int> tilesPosition;
        QMap<int,QMap<int,OGRGeometry*> >::const_iterator  iterTilesX=mTilesGeometry.begin();
        while(iterTilesX!=mTilesGeometry.end())
        {
            int tileX=iterTilesX.key();
            QMap<int,OGRGeometry*>::const_iterator iterTilesY=iterTilesX.value().begin();
            while(iterTilesY!=iterTilesX.value().end())
            {
                int tileY=iterTilesY.key();
                mTilesXToProcess.push_back(tileX);
                mTilesYToProcess.push_back(tileY);
                tilesPosition.push_back(tilesPosition.size());
                iterTilesY++;
            }
            iterTilesX++;
        }
        if(mPtrMpProgressDialog!=NULL)
        {
            delete(mPtrMpProgressDialog);
        }
        if(ptrWidget!=NULL)
            mPtrMpProgressDialog=new QProgressDialog(ptrWidget);
        else
            mPtrMpProgressDialog=new QProgressDialog();
        QString dialogText=QObject::tr("Getting tiles from WKT geometry");
        dialogText+=QObject::tr("\nNumber of tiles to process:%1").arg(tilesPosition.size());
        dialogText+=QObject::tr("\n... progressing using %1 threads").arg(QThread::idealThreadCount());
        mPtrMpProgressDialog->setLabelText(dialogText);
        mPtrMpProgressDialog->setModal(true);
        QFutureWatcher<void> futureWatcher;
        QObject::connect(&futureWatcher, SIGNAL(finished()), mPtrMpProgressDialog, SLOT(reset()));
        QObject::connect(mPtrMpProgressDialog, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));
        QObject::connect(&futureWatcher, SIGNAL(progressRangeChanged(int,int)), mPtrMpProgressDialog, SLOT(setRange(int,int)));
        QObject::connect(&futureWatcher, SIGNAL(progressValueChanged(int)), mPtrMpProgressDialog, SLOT(setValue(int)));
        //                futureWatcher.setFuture(QtConcurrent::map(fieldsValuesToRetrieve, mpLoadPhotovoltaicPanelsFromDb));
        futureWatcher.setFuture(QtConcurrent::map(tilesPosition,
                                                  [this](int& data)
        {mpGetTilesFromWktGeometry(data);}));
        mStrErrorMpProgressDialog="";
        mPtrMpProgressDialog->exec();
        futureWatcher.waitForFinished();
        delete(mPtrMpProgressDialog);
        mPtrMpProgressDialog=NULL;
        if(!mStrErrorMpProgressDialog.isEmpty())
        {
            strError=QObject::tr("PointCloudFile::getTilesFromWktGeometry");
            strError+=QObject::tr("\nError adding tiles geometry");
            strError+=QObject::tr("\nError:\n%1").arg(mStrErrorMpProgressDialog);
            OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
            mMpPtrGeometry=NULL;
            return(false);
        }
        {
            QMap<int, QMap<int, QString> >::iterator iterTilesX=mMpTilesTableName.begin();
            while(iterTilesX!=mMpTilesTableName.end())
            {
                 QMap<int, QString>::iterator iterTilesY=iterTilesX.value().begin();
                 while(iterTilesY!=iterTilesX.value().end())
                 {
                     tilesTableName[iterTilesX.key()][iterTilesY.key()]=iterTilesY.value();
                     iterTilesY++;
                 }
                 iterTilesX++;
            }
        }
        mMpTilesTableName.clear();
    }
    OGRGeometryFactory::destroyGeometry(mMpPtrGeometry);
    mMpPtrGeometry=NULL;
    return(true);
}

bool PointCloudFile::getTilesWktGeometry(QMap<QString, QString> &values,
                                         QString &strError)
{
    values.clear();
    bool useMultiProcess=mUseMultiProcess;
    useMultiProcess=false;
    if(!useMultiProcess)
    {
        QMap<int,QMap<int,QString> >::const_iterator iterTileX=mTilesName.begin();
        while(iterTileX!=mTilesName.end())
        {
            int tileX=iterTileX.key();
            QMap<int,QString>::const_iterator iterTileY=iterTileX.value().begin();
            while(iterTileY!=iterTileX.value().end())
            {
                int tileY=iterTileY.key();
                QString tileTableName=iterTileY.value();
                if(!mTilessWkt.contains(tileTableName))
                {
                    if(!mTilesGeometry.contains(tileX))
                    {
                        strError=QObject::tr("PointCloudFile::getTilesWktGeometry");
                        strError+=QObject::tr("\nNot exists geometry for tile:(%1,%2)")
                                .arg(QString::number(tileX)).arg(QString::number(tileY));
                        return(false);
                    }
                    if(!mTilesGeometry[tileX].contains(tileY))
                    {
                        strError=QObject::tr("PointCloudFile::getTilesWktGeometry");
                        strError+=QObject::tr("\nNot exists geometry for tile:(%1,%2)")
                                .arg(QString::number(tileX)).arg(QString::number(tileY));
                        return(false);
                    }
                    OGRGeometry* ptrGeometry=mTilesGeometry[tileX][tileY];
                    char* ptrWKT;
                    if(OGRERR_NONE!=ptrGeometry->exportToWkt(&ptrWKT))
                    {
                        strError=QObject::tr("PointCloudFile::getTilesWktGeometry");
                        strError+=QObject::tr("\nError exporting to WKT geometry for tile:(%1,%2)")
                                .arg(QString::number(tileX)).arg(QString::number(tileY));
                        return(false);
                    }
                    QString tileWkt=QString::fromLatin1(ptrWKT);
                    mTilessWkt[tileTableName]=tileWkt;
                }
                iterTileY++;
            }
            iterTileX++;
        }
    }
    else
    {
        QWidget* ptrWidget=new QWidget();
        mTilesXToProcess.clear();
        mTilesYToProcess.clear();
        QVector<int> tilesPosition;
        QMap<int,QMap<int,QString> >::const_iterator  iterTilesX=mTilesName.begin();
        while(iterTilesX!=mTilesName.end())
        {
            int tileX=iterTilesX.key();
            QMap<int,QString>::const_iterator iterTilesY=iterTilesX.value().begin();
            while(iterTilesY!=iterTilesX.value().end())
            {
                int tileY=iterTilesY.key();
                QString tileTableName=iterTilesY.value();
                if(!mTilessWkt.contains(tileTableName))
                {
                    if(!mTilesGeometry.contains(tileX))
                    {
                        strError=QObject::tr("PointCloudFile::getTilesWktGeometry");
                        strError+=QObject::tr("\nNot exists geometry for tile:(%1,%2)")
                                .arg(QString::number(tileX)).arg(QString::number(tileY));
                        return(false);
                    }
                    if(!mTilesGeometry[tileX].contains(tileY))
                    {
                        strError=QObject::tr("PointCloudFile::getTilesWktGeometry");
                        strError+=QObject::tr("\nNot exists geometry for tile:(%1,%2)")
                                .arg(QString::number(tileX)).arg(QString::number(tileY));
                        return(false);
                    }
                    mTilesXToProcess.push_back(tileX);
                    mTilesYToProcess.push_back(tileY);
                    tilesPosition.push_back(tilesPosition.size());
                }
                iterTilesY++;
            }
            iterTilesX++;
        }
        if(mPtrMpProgressDialog!=NULL)
        {
            delete(mPtrMpProgressDialog);
        }
        if(ptrWidget!=NULL)
            mPtrMpProgressDialog=new QProgressDialog(ptrWidget);
        else
            mPtrMpProgressDialog=new QProgressDialog();
        QString dialogText=QObject::tr("Getting tiles WKT geometry");
        dialogText+=QObject::tr("\nNumber of tiles to process:%1").arg(tilesPosition.size());
        dialogText+=QObject::tr("\n... progressing using %1 threads").arg(QThread::idealThreadCount());
        mPtrMpProgressDialog->setLabelText(dialogText);
        mPtrMpProgressDialog->setModal(true);
        QFutureWatcher<void> futureWatcher;
        QObject::connect(&futureWatcher, SIGNAL(finished()), mPtrMpProgressDialog, SLOT(reset()));
        QObject::connect(mPtrMpProgressDialog, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));
        QObject::connect(&futureWatcher, SIGNAL(progressRangeChanged(int,int)), mPtrMpProgressDialog, SLOT(setRange(int,int)));
        QObject::connect(&futureWatcher, SIGNAL(progressValueChanged(int)), mPtrMpProgressDialog, SLOT(setValue(int)));
        //                futureWatcher.setFuture(QtConcurrent::map(fieldsValuesToRetrieve, mpLoadPhotovoltaicPanelsFromDb));
        futureWatcher.setFuture(QtConcurrent::map(tilesPosition,
                                                  [this](int& data)
        {mpGetTilesWktGeometry(data);}));
        mStrErrorMpProgressDialog="";
        mPtrMpProgressDialog->exec();
        futureWatcher.waitForFinished();
        delete(mPtrMpProgressDialog);
        mPtrMpProgressDialog=NULL;
        if(!mStrErrorMpProgressDialog.isEmpty())
        {
            strError=QObject::tr("PointCloudFile::getTilesWktGeometry");
            strError+=QObject::tr("\nError adding tiles geometry");
            strError+=QObject::tr("\nError:\n%1").arg(mStrErrorMpProgressDialog);
            return(false);
        }
    }
    values=mTilessWkt;
    return(true);
}

bool PointCloudFile::processReclassificationConfusionMatrixReport(QString &fileName,
                                                                  QVector<int> &classes,
                                                                  QString &strError)
{
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        strError=QObject::tr("PointCloudFile::processReclassificationConfusionMatrixReport");
        strError+=QObject::tr("\nError opening output file:\n").arg(fileName);
        return(false);
    }
    QTextStream out(&file);
    out<<"Reclassification confusion matrix report for Point Cloud path: "<<mPath<<"\n";
    out<<"=============================================================\n";
    out<<"Analyzed classes: ";
    for(int i=0;i<classes.size();i++)
    {
        out<<QString::number(classes.at(i)).rightJustified(3);
    }
    out<<"\n";
    QMap<QString,QMap<int,QMap<int,int> > > changesByClassByFile;
    QMap<QString,QMap<int,int> > pointsByClassByFile;
    QMap<QString,QMap<int,int> > pointsRemovedByClassByFile;
    QMap<QString,QMap<int,int> > pointsToAnotherClassesByClassByFile;
    QMap<QString,QMap<int,int> > pointsFromAnotherClassesByClassByFile;
    QMap<QString,int>::const_iterator iterFiles=mFilesIndex.begin();
    while(iterFiles!=mFilesIndex.end())
    {
        int fileId=iterFiles.value();
        QString pointCloudFileName=iterFiles.key();
        for(int i=0;i<classes.size();i++)
        {
            QMap<int,int> aux;
            for(int j=0;j<classes.size();j++)
            {
                aux[classes[j]]=0;
            }
            changesByClassByFile[pointCloudFileName][classes[i]]=aux;
            pointsByClassByFile[pointCloudFileName][classes[i]]=0;
            pointsRemovedByClassByFile[pointCloudFileName][classes[i]]=0;
            pointsToAnotherClassesByClassByFile[pointCloudFileName][classes[i]]=0;
            pointsFromAnotherClassesByClassByFile[pointCloudFileName][classes[i]]=0;
        }
        iterFiles++;
    }
    QWidget* ptrWidget=new QWidget();
    QString strAuxError;
    QProgressDialog* ptrProgress=NULL;
    int numberOfFiles=mFilesIndex.size();
    if(ptrWidget!=NULL)
    {
        QString title=QObject::tr("Reporting reclassification confusion matrix for point cloud: ");
        QString msgGlobal=mPath;
        msgGlobal+="\n";
        msgGlobal+=QString::number(numberOfFiles,10);
        msgGlobal+=" number of files";
        ptrProgress=new QProgressDialog(title, "Abort",0,numberOfFiles, ptrWidget);
        ptrProgress->setWindowModality(Qt::WindowModal);
        ptrProgress->setLabelText(msgGlobal);
        ptrProgress->show();
        qApp->processEvents();
    }
    iterFiles=mFilesIndex.begin();
    int nf=0;
    while(iterFiles!=mFilesIndex.end())
    {
        nf++;
        int fileId=iterFiles.value();
        QString pointCloudFileName=iterFiles.key();
        if(ptrWidget!=NULL)
        {
            ptrProgress->setValue(nf);
            qApp->processEvents();
        }
        QString classesFileName=mClassesFileByIndex[fileId];
        QFile pointsClassFile(classesFileName);
        if (!pointsClassFile.open(QIODevice::ReadOnly))
        {
            strError=QObject::tr("PointCloudFile::processReclassificationConfusionMatrixReport");
            strError+=QObject::tr("\nError opening file:\n%1").arg(classesFileName);
            if(ptrWidget!=NULL)
            {
                ptrProgress->setValue(numberOfFiles);
                qApp->processEvents();
                ptrProgress->close();
                delete(ptrProgress);
            }
            return(false);
        }
        QDataStream inPointsClass(&pointsClassFile);
        QMap<QString,bool> existsFields;
        QMap<int,QMap<int,QVector<quint8> > > tilesPointsClass;
        QMap<int,QMap<int,QMap<int,quint8> > > tilesPointsClassNewByPos; // se guarda vacío
        QMap<int,QMap<int,int> > tilesNop;
        inPointsClass>>tilesNop;
        inPointsClass>>existsFields;
        inPointsClass>>tilesPointsClass;
        inPointsClass>>tilesPointsClassNewByPos;
        pointsClassFile.close();
        QMap<int,QMap<int,QVector<quint8> > >::const_iterator iterTileX=tilesPointsClass.begin(); // se guarda vacío
        while(iterTileX!=tilesPointsClass.end())
        {
            int tileX=iterTileX.key();
            QMap<int,QVector<quint8> >::const_iterator iterTileY=iterTileX.value().begin();
            while(iterTileY!=iterTileX.value().end())
            {
                int tileY=iterTileY.key();
                QVector<quint8> pointsPositionInTile=iterTileY.value();
                for(int pointPositionInTile=0;pointPositionInTile<pointsPositionInTile.size();pointPositionInTile++)
                {
                    quint8 classOriginal=pointsPositionInTile[pointPositionInTile];
                    quint8 classNew=pointsPositionInTile[pointPositionInTile];
                    int removed=0;
                    if(tilesPointsClassNewByPos.contains(tileX))
                    {
                        if(tilesPointsClassNewByPos[tileX].contains(tileY))
                        {
                            if(tilesPointsClassNewByPos[tileX][tileY].contains(pointPositionInTile))
                            {
                                classNew=tilesPointsClassNewByPos[tileX][tileY][pointPositionInTile];
                                if(classNew==POINTCLOUDFILE_CLASS_NUMBER_REMOVE) removed=1;
                            }
                        }
                    }
                    if(!classes.contains(classOriginal))
                    {
                        if(classes.contains(classNew))
                        {
                            pointsFromAnotherClassesByClassByFile[pointCloudFileName][classNew]=pointsFromAnotherClassesByClassByFile[pointCloudFileName][classNew]+1;
                        }
                        continue;
                    }
                    pointsByClassByFile[pointCloudFileName][classOriginal]=pointsByClassByFile[pointCloudFileName][classOriginal]+1;
                    if(removed==1)
                    {
                        pointsRemovedByClassByFile[pointCloudFileName][classOriginal]=pointsRemovedByClassByFile[pointCloudFileName][classOriginal]+1;
                    }
                    else
                    {
                        if(!classes.contains(classNew))
                        {
                            pointsToAnotherClassesByClassByFile[pointCloudFileName][classOriginal]=pointsToAnotherClassesByClassByFile[pointCloudFileName][classOriginal]+1;
                            continue;
                        }
                        changesByClassByFile[pointCloudFileName][classOriginal][classNew]=changesByClassByFile[pointCloudFileName][classOriginal][classNew]+1;
                    }
                }
                iterTileY++;
            }
            iterTileX++;
        }
        iterFiles++;
    }
    if(ptrWidget!=NULL)
    {
        ptrProgress->setValue(numberOfFiles);
        qApp->processEvents();
        ptrProgress->close();
        delete(ptrProgress);
    }
    iterFiles=mFilesIndex.begin();
    while(iterFiles!=mFilesIndex.end())
    {
        int fileId=iterFiles.value();
        QString pointCloudFileName=iterFiles.key();
        out<<"- Point cloud file .....................: "<<pointCloudFileName<<"\n";
        out<<"  Points numbers confusion matrix ......:\n";
        out<<QString("").rightJustified(12);
        out<<QString("N.Points").rightJustified(12);
        for(int i=0;i<classes.size();i++)
        {
            QString classTitle="Class "+QString::number(classes[i]).rightJustified(2);
            out<<classTitle.rightJustified(12);
        }
        out<<QString("To Another").rightJustified(14);
        out<<QString("From Another").rightJustified(14);
        out<<QString("Removed").rightJustified(12);
        out<<"\n";
        for(int i=0;i<classes.size();i++)
        {
            QString classTitle="Class "+QString::number(classes[i]).rightJustified(2);
            out<<classTitle.rightJustified(12);
            out<<QString::number(pointsByClassByFile[pointCloudFileName][classes[i]]).rightJustified(12);
            for(int j=0;j<classes.size();j++)
            {
                out<<QString::number(changesByClassByFile[pointCloudFileName][classes[i]][classes[j]]).rightJustified(12);
            }
            out<<QString::number(pointsToAnotherClassesByClassByFile[pointCloudFileName][classes[i]]).rightJustified(14);
            out<<QString::number(pointsFromAnotherClassesByClassByFile[pointCloudFileName][classes[i]]).rightJustified(14);
            out<<QString::number(pointsRemovedByClassByFile[pointCloudFileName][classes[i]]).rightJustified(12);
            out<<"\n";
        }
        out<<"\n";
        out<<"  Percentages points confusion matrix ..:\n";
        out<<QString("").rightJustified(12);
        out<<QString("N.Points").rightJustified(12);
        for(int i=0;i<classes.size();i++)
        {
            QString classTitle="Class "+QString::number(classes[i]).rightJustified(2);
            out<<classTitle.rightJustified(12);
        }
        out<<QString("To Another").rightJustified(14);
        out<<QString("From Another").rightJustified(14);
        out<<QString("Removed").rightJustified(12);
        out<<"\n";
        for(int i=0;i<classes.size();i++)
        {
            QString classTitle="Class "+QString::number(classes[i]).rightJustified(2);
            out<<classTitle.rightJustified(12);
            out<<QString::number(pointsByClassByFile[pointCloudFileName][classes[i]]).rightJustified(12);
            double percentage=0.0;
            for(int j=0;j<classes.size();j++)
            {
                if(pointsByClassByFile[pointCloudFileName][classes[i]]>0)
                {
                    percentage=100.0*changesByClassByFile[pointCloudFileName][classes[i]][classes[j]]/pointsByClassByFile[pointCloudFileName][classes[i]];
                    out<<QString::number(percentage,'f',2).rightJustified(12);
                }
                else
                {
                    out<<QString("----").rightJustified(12);
                }
            }
            if(pointsByClassByFile[pointCloudFileName][classes[i]]>0)
            {
                percentage=100.0*pointsToAnotherClassesByClassByFile[pointCloudFileName][classes[i]]/pointsByClassByFile[pointCloudFileName][classes[i]];
                out<<QString::number(percentage,'f',2).rightJustified(14);
            }
            else
            {
                out<<QString("----").rightJustified(14);
            }
            out<<QString::number(pointsFromAnotherClassesByClassByFile[pointCloudFileName][classes[i]]).rightJustified(14);
            if(pointsByClassByFile[pointCloudFileName][classes[i]]>0)
            {
                percentage=100.0*pointsRemovedByClassByFile[pointCloudFileName][classes[i]]/pointsByClassByFile[pointCloudFileName][classes[i]];
                out<<QString::number(percentage,'f',2).rightJustified(12);
            }
            else
            {
                out<<QString("----").rightJustified(12);
            }
            out<<"\n";
        }
        out<<"\n";
        iterFiles++;
    }
    out<<"- Results for all files:"<<"\n";
    out<<"  Points numbers confusion matrix ......:\n";
    out<<QString("").rightJustified(12);
    out<<QString("N.Points").rightJustified(12);
    for(int i=0;i<classes.size();i++)
    {
        QString classTitle="Class "+QString::number(classes[i]).rightJustified(2);
        out<<classTitle.rightJustified(12);
    }
    out<<QString("To Another").rightJustified(14);
    out<<QString("From Another").rightJustified(14);
    out<<QString("Removed").rightJustified(12);
    out<<"\n";
    for(int i=0;i<classes.size();i++)
    {
        QString classTitle="Class "+QString::number(classes[i]).rightJustified(2);
        out<<classTitle.rightJustified(12);
        int numberOfPoints=0;
        iterFiles=mFilesIndex.begin();
        while(iterFiles!=mFilesIndex.end())
        {
            QString pointCloudFileName=iterFiles.key();
            numberOfPoints+=pointsByClassByFile[pointCloudFileName][classes[i]];
            iterFiles++;
        }
        out<<QString::number(numberOfPoints).rightJustified(12);
        for(int j=0;j<classes.size();j++)
        {
            int numberOfChangedPoints=0;
            iterFiles=mFilesIndex.begin();
            while(iterFiles!=mFilesIndex.end())
            {
                QString pointCloudFileName=iterFiles.key();
                numberOfChangedPoints+=changesByClassByFile[pointCloudFileName][classes[i]][classes[j]];
                iterFiles++;
            }
            out<<QString::number(numberOfChangedPoints).rightJustified(12);
        }
        int numberOfPointsToAnotherClass=0;
        iterFiles=mFilesIndex.begin();
        while(iterFiles!=mFilesIndex.end())
        {
            QString pointCloudFileName=iterFiles.key();
            numberOfPointsToAnotherClass+=pointsToAnotherClassesByClassByFile[pointCloudFileName][classes[i]];
            iterFiles++;
        }
        out<<QString::number(numberOfPointsToAnotherClass).rightJustified(14);
        int numberOfPointsFromAnotherClass=0;
        iterFiles=mFilesIndex.begin();
        while(iterFiles!=mFilesIndex.end())
        {
            QString pointCloudFileName=iterFiles.key();
            numberOfPointsFromAnotherClass+=pointsFromAnotherClassesByClassByFile[pointCloudFileName][classes[i]];
            iterFiles++;
        }
        out<<QString::number(numberOfPointsFromAnotherClass).rightJustified(14);
        int numberOfRemovedPoints=0;
        iterFiles=mFilesIndex.begin();
        while(iterFiles!=mFilesIndex.end())
        {
            QString pointCloudFileName=iterFiles.key();
            numberOfRemovedPoints+=pointsRemovedByClassByFile[pointCloudFileName][classes[i]];
            iterFiles++;
        }
        out<<QString::number(numberOfRemovedPoints).rightJustified(12);
        out<<"\n";
    }
    out<<"  Percentages points confusion matrix ..:\n";
    out<<QString("").rightJustified(12);
    out<<QString("N.Points").rightJustified(12);
    for(int i=0;i<classes.size();i++)
    {
        QString classTitle="Class "+QString::number(classes[i]).rightJustified(2);
        out<<classTitle.rightJustified(12);
    }
    out<<QString("To Another").rightJustified(14);
    out<<QString("From Another").rightJustified(14);
    out<<QString("Removed").rightJustified(12);
    out<<"\n";
    for(int i=0;i<classes.size();i++)
    {
        QString classTitle="Class "+QString::number(classes[i]).rightJustified(2);
        out<<classTitle.rightJustified(12);
        int numberOfPoints=0;
        iterFiles=mFilesIndex.begin();
        while(iterFiles!=mFilesIndex.end())
        {
            QString pointCloudFileName=iterFiles.key();
            numberOfPoints+=pointsByClassByFile[pointCloudFileName][classes[i]];
            iterFiles++;
        }
        out<<QString::number(numberOfPoints).rightJustified(12);
        double percentage=0.0;
        for(int j=0;j<classes.size();j++)
        {
            int numberOfChangedPoints=0;
            iterFiles=mFilesIndex.begin();
            while(iterFiles!=mFilesIndex.end())
            {
                QString pointCloudFileName=iterFiles.key();
                numberOfChangedPoints+=changesByClassByFile[pointCloudFileName][classes[i]][classes[j]];
                iterFiles++;
            }
            if(numberOfPoints>0)
            {
                percentage=100.0*numberOfChangedPoints/numberOfPoints;
                out<<QString::number(percentage,'f',2).rightJustified(12);
            }
            else
            {
                out<<QString("----").rightJustified(12);
            }
        }
        int numberOfPointsToAnotherClass=0;
        iterFiles=mFilesIndex.begin();
        while(iterFiles!=mFilesIndex.end())
        {
            QString pointCloudFileName=iterFiles.key();
            numberOfPointsToAnotherClass+=pointsToAnotherClassesByClassByFile[pointCloudFileName][classes[i]];
            iterFiles++;
        }
        if(numberOfPoints>0)
        {
            percentage=100.0*numberOfPointsToAnotherClass/numberOfPoints;
            out<<QString::number(percentage,'f',2).rightJustified(14);
        }
        else
        {
            out<<QString("----").rightJustified(14);
        }
        int numberOfPointsFromAnotherClass=0;
        iterFiles=mFilesIndex.begin();
        while(iterFiles!=mFilesIndex.end())
        {
            QString pointCloudFileName=iterFiles.key();
            numberOfPointsFromAnotherClass+=pointsFromAnotherClassesByClassByFile[pointCloudFileName][classes[i]];
            iterFiles++;
        }
        if(numberOfPoints>0)
        {
            percentage=100.0*numberOfPointsFromAnotherClass/numberOfPoints;
            out<<QString::number(percentage,'f',2).rightJustified(14);
        }
        else
        {
            out<<QString("----").rightJustified(14);
        }
        int numberOfRemovedPoints=0;
        iterFiles=mFilesIndex.begin();
        while(iterFiles!=mFilesIndex.end())
        {
            QString pointCloudFileName=iterFiles.key();
            numberOfRemovedPoints+=pointsRemovedByClassByFile[pointCloudFileName][classes[i]];
            iterFiles++;
        }
        if(numberOfPoints>0)
        {
            percentage=100.0*numberOfRemovedPoints/numberOfPoints;
            out<<QString::number(percentage,'f',2).rightJustified(12);
        }
        else
        {
            out<<QString("----").rightJustified(12);
        }
        out<<"\n";
    }
    file.close();
    return(true);
}

bool PointCloudFile::setFromPath(QString path,
                                 QString &strError)
{
    clear();
    QDir currentDir=QDir::currentPath();
    if(!currentDir.exists(path))
    {
        strError=QObject::tr("PointCloudFile::setFromPath");
        strError+=QObject::tr("\nNot exists path:\n%1")
                .arg(path);
        clear();
        return(false);
    }
    mPath=path;
    QString strAuxError;
    if(!readHeader(strAuxError))
    {
        strError=QObject::tr("PointCloudFile::setFromPath");
        strError+=QObject::tr("\nError setting from path:\n%1\nError:\n%2")
                .arg(path).arg(strAuxError);
        clear();
        return(false);
    }
    return(true);
}

bool PointCloudFile::readHeader(QString &strError)
{
    QString headerFileName=mPath+"/"+POINTCLOUDFILE_MANAGER_FILE_NAME;
    if(!QFile::exists(headerFileName))
    {
        strError=QObject::tr("PointCloudFile::readHeader");
        strError+=QObject::tr("\nHeader file not exists:\n%1").arg(headerFileName);
        return(false);
    }
    QFile headerFile(headerFileName);
    headerFile.open(QIODevice::ReadOnly);
    mParameterValueByCode.clear();
    QDataStream headerIn(&headerFile);
    headerIn>>mSRID;
    headerIn>>mCrsDescription;
    headerIn>>mCrsProj4String;
    headerIn>>mHeightType;
    headerIn>>mGridSize;
    headerIn>>mProjectType;
    headerIn>>mParameterValueByCode;

    headerIn>>mMinimumFc;
    headerIn>>mMinimumSc;
    headerIn>>mMinimumTc;
    headerIn>>mMaximumDensity;
    headerIn>>mFilesIndex;
    headerIn>>mFileByIndex;
    headerIn>>mNewFilesIndex;
    headerIn>>mTilesName;
    headerIn>>mTilesNumberOfPoints;
    headerIn>>mTilesContainedInROIs;
    headerIn>>mTilesOverlapsWithROIs;
    headerIn>>mTilesByFileIndex;

    quint16 numberOfROIs=0;
    headerIn>>numberOfROIs;
    if(numberOfROIs>0)
    {
        for(int nroi=0;nroi<=numberOfROIs;nroi++)
        {
            QString roiId,roiWkt;
            headerIn>>roiId;
            headerIn>>roiWkt;
            roiWkt=roiWkt.toUpper();
            QByteArray byteArrayWktGeometry = roiWkt.toUtf8();
            char *charsWktGeometry = byteArrayWktGeometry.data();
            OGRGeometry* ptrGeometry;
            if(roiWkt.contains("MULTI"))
            {
                ptrGeometry=OGRGeometryFactory::createGeometry(wkbMultiPolygon);
            }
            else
            {
                ptrGeometry=OGRGeometryFactory::createGeometry(wkbPolygon);
            }
            if(OGRERR_NONE!=ptrGeometry->importFromWkt(&charsWktGeometry))
            {
                strError=QObject::tr("PointCloudFile::readHeader");
                strError+=QObject::tr("\nIn path:\n%1").arg(mPath);
                strError+=QObject::tr("\nIn file:\n%1").arg(headerFileName);
                strError+=QObject::tr("\nError importing ROI geometry from WKT:\n%1")
                        .arg(roiWkt);
                return(false);
            }
            if(nroi<numberOfROIs)
            {
                mPtrROIs[roiId]=ptrGeometry;
            }
            else // a continuacion figura la union
            {
                mPtrROIsUnion=ptrGeometry;
            }
        }
    }
    headerFile.close();
    QMap<QString,QString>::const_iterator iterPvbc=mParameterValueByCode.begin();
    while(iterPvbc!=mParameterValueByCode.end())
    {
        QString parameterCode=iterPvbc.key();
        QString parameterValue=iterPvbc.value();
        if(parameterCode.compare(POINTCLOUDFILE_PARAMETER_COLOR_BYTES,Qt::CaseInsensitive)==0)
        {
            bool okToInt=false;
            int intValue=parameterValue.toInt(&okToInt);
            if(!okToInt)
            {
                strError=QObject::tr("PointCloudFile::readHeader");
                strError+=QObject::tr("\nParameter %1 value is not an integer: %2")
                        .arg(parameterCode).arg(parameterValue);
                return(false);
            }
            mNumberOfColorBytes=intValue;
            iterPvbc++;
            continue;
        }
        QMap<QString,bool>::const_iterator iterStoredFields=mStoredFields.begin();
        while(iterStoredFields!=mStoredFields.end())
        {
            QString storeField=iterStoredFields.key();
            if(parameterCode.compare(storeField,Qt::CaseInsensitive)==0)
            {
                if(parameterValue.compare("false",Qt::CaseInsensitive)==0
                    ||parameterValue.compare("falso",Qt::CaseInsensitive)==0)
                {
                    mStoredFields[storeField]=false;
                }
                else if(parameterValue.compare("true",Qt::CaseInsensitive)==0
                        ||parameterValue.compare("verdadero",Qt::CaseInsensitive)==0)
                {
                    mStoredFields[storeField]=true;
                }
                break;
            }
            iterStoredFields++;
        }
        iterPvbc++;
    }   
    if(!mUseMultiProcess)
    {
        QMap<int,QMap<int,QString> >::const_iterator  iterTilesX=mTilesName.begin();
        while(iterTilesX!=mTilesName.end())
        {
            int tileX=iterTilesX.key();
            QMap<int,QString>::const_iterator iterTilesY=iterTilesX.value().begin();
            while(iterTilesY!=iterTilesX.value().end())
            {
                int tileY=iterTilesY.key();
                QString wktGeometry="POLYGON((";
                wktGeometry+=QString::number(tileX);
                wktGeometry+=" ";
                wktGeometry+=QString::number(tileY);
                wktGeometry+=",";
                wktGeometry+=QString::number(tileX);
                wktGeometry+=" ";
                wktGeometry+=QString::number(qRound(tileY+mGridSize),'f',0);
                wktGeometry+=",";
                wktGeometry+=QString::number(qRound(tileX+mGridSize),'f',0);
                wktGeometry+=" ";
                wktGeometry+=QString::number(qRound(tileY+mGridSize),'f',0);
                wktGeometry+=",";
                wktGeometry+=QString::number(qRound(tileX+mGridSize),'f',0);
                wktGeometry+=" ";
                wktGeometry+=QString::number(tileY);
                wktGeometry+=",";
                wktGeometry+=QString::number(tileX);
                wktGeometry+=" ";
                wktGeometry+=QString::number(tileY);
                wktGeometry+="))";
                QByteArray byteArrayWktGeometry = wktGeometry.toUtf8();
                char *charsWktGeometry = byteArrayWktGeometry.data();
                OGRGeometry* ptrGeometry;
                ptrGeometry=OGRGeometryFactory::createGeometry(wkbPolygon);
                if(OGRERR_NONE!=ptrGeometry->importFromWkt(&charsWktGeometry))
                {
                    strError=QObject::tr("PointCloudFile::readHeader");
                    strError+=QObject::tr("\nError making geometry from WKT: %1").arg(wktGeometry);
                    return(false);
                }
                mTilesGeometry[tileX][tileY]=ptrGeometry;
                iterTilesY++;
            }
            iterTilesX++;
        }
    }
    else
    {
        QWidget* ptrWidget=new QWidget();
        mTilesXToProcess.clear();
        mTilesYToProcess.clear();
        QVector<int> tilesPosition;
        QVector<int> tilesX;
        QVector<int> tilesY;
        QMap<int,QMap<int,QString> >::const_iterator  iterTilesX=mTilesName.begin();
        while(iterTilesX!=mTilesName.end())
        {
            int tileX=iterTilesX.key();
            QMap<int,QString>::const_iterator iterTilesY=iterTilesX.value().begin();
            while(iterTilesY!=iterTilesX.value().end())
            {
                int tileY=iterTilesY.key();
                mTilesXToProcess.push_back(tileX);
                mTilesYToProcess.push_back(tileY);
                tilesPosition.push_back(tilesPosition.size());
                iterTilesY++;
            }
            iterTilesX++;
        }
        if(mPtrMpProgressDialog!=NULL)
        {
            delete(mPtrMpProgressDialog);
        }
        if(ptrWidget!=NULL)
            mPtrMpProgressDialog=new QProgressDialog(ptrWidget);
        else
            mPtrMpProgressDialog=new QProgressDialog();

        QString dialogText=QObject::tr("Adding tiles geometry");
        dialogText+=QObject::tr("\nNumber of tiles to process:%1").arg(tilesPosition.size());
        dialogText+=QObject::tr("\n... progressing using %1 threads").arg(QThread::idealThreadCount());
        mPtrMpProgressDialog->setLabelText(dialogText);
        mPtrMpProgressDialog->setModal(true);
        QFutureWatcher<void> futureWatcher;
        QObject::connect(&futureWatcher, SIGNAL(finished()), mPtrMpProgressDialog, SLOT(reset()));
        QObject::connect(mPtrMpProgressDialog, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));
        QObject::connect(&futureWatcher, SIGNAL(progressRangeChanged(int,int)), mPtrMpProgressDialog, SLOT(setRange(int,int)));
        QObject::connect(&futureWatcher, SIGNAL(progressValueChanged(int)), mPtrMpProgressDialog, SLOT(setValue(int)));
        //                futureWatcher.setFuture(QtConcurrent::map(fieldsValuesToRetrieve, mpLoadPhotovoltaicPanelsFromDb));
        futureWatcher.setFuture(QtConcurrent::map(tilesPosition,
                                                  [this](int& data)
        {mpAddTilesGeometry(data);}));
        mStrErrorMpProgressDialog="";
        mPtrMpProgressDialog->exec();
        futureWatcher.waitForFinished();
        delete(mPtrMpProgressDialog);
        mPtrMpProgressDialog=NULL;
        if(!mStrErrorMpProgressDialog.isEmpty())
        {
            strError=QObject::tr("PointCloudFile::readHeader");
            strError+=QObject::tr("\nError adding tiles geometry");
            strError+=QObject::tr("\nError:\n%1").arg(mStrErrorMpProgressDialog);
            return(false);
        }
    }
    QMap<int,QString>::const_iterator iterFiles=mFileByIndex.begin();
    while(iterFiles!=mFileByIndex.end())
    {
        int fileIndex=iterFiles.key();
        QString inputFileName=iterFiles.value();
        if(!QFile::exists(inputFileName))
        {
            strError=QObject::tr("\PointCloudFile::readHeader");
            strError+=QObject::tr("\nFile not found:\n%1")
                    .arg(inputFileName);
            return(false);
        }
        QFileInfo inputFileInfo(inputFileName);
        QString inputFileBaseName=inputFileInfo.completeBaseName();
        QString tilesPointsFileZipFileName=mPath+"/"+inputFileBaseName+"."+POINTCLOUDFILE_DHL_SUFFIX;
        if(!QFile::exists(tilesPointsFileZipFileName))
        {
            strError=QObject::tr("\PointCloudFile::readHeader");
            strError+=QObject::tr("\nFile not found:\n%1")
                    .arg(tilesPointsFileZipFileName);
            return(false);
        }
        QString pointsClassFileName=mPath+"/"+inputFileBaseName+"."+POINTCLOUDFILE_PCS_SUFFIX;
        if(!QFile::exists(pointsClassFileName))
        {
            strError=QObject::tr("\PointCloudFile::readHeader");
            strError+=QObject::tr("\nFile not found:\n%1")
                    .arg(pointsClassFileName);
            return(false);
        }
        QString tilesPointsFileZipFilePath=mPath+"/"+inputFileBaseName;
        mZipFilePathPointsByIndex[fileIndex]=tilesPointsFileZipFilePath;
        mZipFilePointsByIndex[fileIndex]=tilesPointsFileZipFileName;
        mClassesFileByIndex[fileIndex]=pointsClassFileName;
        iterFiles++;
    }

    return(true);
}

bool PointCloudFile::removeDir(QString dirName,
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

bool PointCloudFile::removeTile(int tileX,
                                int tileY,
                                QString &strError)
{
    if(!mTilesName.contains(tileX))
    {
        return(true);
    }
    else if(!mTilesName[tileX].contains(tileY))
    {
        return(true);
    }
    if(mTilesNumberOfPoints[tileX][tileY]==0)
    {
        OGRGeometry* ptrGeometry=mTilesGeometry[tileX][tileY];
        if(ptrGeometry!=NULL)
        {
            OGRGeometryFactory::destroyGeometry(ptrGeometry);
            mTilesGeometry[tileX][tileY]=NULL;
            mTilesGeometry[tileX].remove(tileY);
        }
        mTilesGeometry[tileX].remove(tileY);
        mTilesName[tileX].remove(tileY);
        mTilesContainedInROIs[tileX].remove(tileY);
        mTilesOverlapsWithROIs[tileX].remove(tileY);
        mTilesNumberOfPoints[tileX].remove(tileY);
        if(mTilesGeometry[tileX].size()==0)
        {
            mTilesGeometry.remove(tileX);
        }
        if(mTilesName[tileX].size()==0)
        {
            mTilesName.remove(tileX);
        }
        if(mTilesContainedInROIs[tileX].size()==0)
        {
            mTilesContainedInROIs.remove(tileX);
        }
        if(mTilesOverlapsWithROIs[tileX].size()==0)
        {
            mTilesOverlapsWithROIs.remove(tileX);
        }
    }
    return(true);
}

bool PointCloudFile::removeTile(int tileX,
                                int tileY,
                                int fileIndex,
                                QString &strError)
{
    if(!mTilesName.contains(tileX))
    {
        return(true);
    }
    else if(!mTilesName[tileX].contains(tileY))
    {
        return(true);
    }
    if(mTilesNumberOfPoints[tileX][tileY]==0)
    {
        OGRGeometry* ptrGeometry=mTilesGeometry[tileX][tileY];
        if(ptrGeometry!=NULL)
        {
            OGRGeometryFactory::destroyGeometry(ptrGeometry);
            mTilesGeometry[tileX][tileY]=NULL;
            mTilesGeometry[tileX].remove(tileY);
        }
        mTilesGeometry[tileX].remove(tileY);
        mTilesName[tileX].remove(tileY);
        mTilesContainedInROIs[tileX].remove(tileY);
        mTilesOverlapsWithROIs[tileX].remove(tileY);
        mTilesNumberOfPoints[tileX].remove(tileY);
        if(mTilesGeometry[tileX].size()==0)
        {
            mTilesGeometry.remove(tileX);
        }
        if(mTilesName[tileX].size()==0)
        {
            mTilesName.remove(tileX);
        }
        if(mTilesContainedInROIs[tileX].size()==0)
        {
            mTilesContainedInROIs.remove(tileX);
        }
        if(mTilesOverlapsWithROIs[tileX].size()==0)
        {
            mTilesOverlapsWithROIs.remove(tileX);
        }
    }
    QMap<int,QVector<int> > tilesFile=mTilesByFileIndex[fileIndex];
    if(tilesFile.contains(tileX))
    {
        int pos=tilesFile[tileX].indexOf(tileY);
        if(pos!=-1)
        {
            mTilesByFileIndex[fileIndex][tileX].remove(pos);
            if(mTilesByFileIndex[fileIndex][tileX].size()==0)
            {
                mTilesByFileIndex[fileIndex].remove(tileX);
            }
        }
    }
    return(true);
}

void PointCloudFile::clear()
{
    mSRID=POINTCLOUDFILE_SRID_NO_VALUE;
    mCrsDescription.clear();
    mCrsProj4String.clear();
    mGridSize=POINTCLOUDFILE_NO_DOUBLE_VALUE;
//    mStorePointsData=false;
    mNumberOfPointsInMemory=0;
    mMaximumDensity=0;
//        mPointCloudFiles.clear();
//    mROIsId.clear();
//    mFilesId.clear();
    mTilesContainedInROIs.clear();
    mTilesOverlapsWithROIs.clear();
//    mTilesTableNameByFileId.clear();
    mTempPath.clear();
    mParameterValueByCode.clear();
    mStoredFields[POINTCLOUDFILE_PARAMETER_COLOR]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_GPS_TIME]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_USER_DATA]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_INTENSITY]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_SOURCE_ID]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_NIR]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_RETURN]=false;
    mStoredFields[POINTCLOUDFILE_PARAMETER_RETURNS]=false;
    mNumberOfColorBytes=1;
    mNewFilesIndex=0;
    mTilesNumberOfPoints.clear();
    mTilesByFileIndex.clear();
//    mTileTableNameByDbId.clear();
//    mTilesTableMinX.clear();
//    mTilesTableMaxX.clear();
//    mTilesTableMinY.clear();
//    mTilesTableMaxY.clear();
    if(mPtrROIsUnion!=NULL)
    {
        OGRGeometryFactory::destroyGeometry(mPtrROIsUnion);
        mPtrROIsUnion=NULL;
    }
    QMap<QString,OGRGeometry*>::iterator iterROIs=mPtrROIs.begin();
    while(iterROIs!=mPtrROIs.end())
    {
        OGRGeometry* ptrGeometry=iterROIs.value();
        if(ptrGeometry!=NULL)
        {
            OGRGeometryFactory::destroyGeometry(ptrGeometry);
            mPtrROIs[iterROIs.key()]=NULL;
        }
        iterROIs++;
    }
    mROIsWkt.clear();
    mTilessWkt.clear();
    QMap<int,QMap<int,OGRGeometry*> >::iterator iterTilesColumn=mTilesGeometry.begin();
    while(iterTilesColumn!=mTilesGeometry.end())
    {
        QMap<int,OGRGeometry*>::iterator iterTilesRow=iterTilesColumn.value().begin();
        while(iterTilesRow!=iterTilesColumn.value().end())
        {
            OGRGeometry* ptrGeometry=iterTilesRow.value();
            if(ptrGeometry!=NULL)
            {
                OGRGeometryFactory::destroyGeometry(ptrGeometry);
                mTilesGeometry[iterTilesColumn.key()][iterTilesRow.key()]=NULL;
            }
            iterTilesRow++;
        }
        iterTilesColumn++;
    }
    /*
    QMap<int,OGRGeometry*>::iterator iterFiles=mFilePtrGeometryByIndex.begin();
    while(iterFiles!=mFilePtrGeometryByIndex.end())
    {
        OGRGeometry* ptrGeometry=iterFiles.value();
        if(ptrGeometry!=NULL)
        {
            OGRGeometryFactory::destroyGeometry(ptrGeometry);
            mFilePtrGeometryByIndex[iterFiles.key()]=NULL;
        }
        iterFiles++;
    }
    */
    mFilesIndex.clear();
    mFileByIndex.clear();
    mZipFilePointsByIndex.clear();
    mZipFilePathPointsByIndex.clear();
    mClassesFileByIndex.clear();
}

bool PointCloudFile::writeHeader(QString &strError)
{
//    QuaZipFile headerFile(mPtrZipFile);
//    QString headerFileName=POINTCLOUDFILE_HEADER_FILE_NAME;
//    headerFile.open(QIODevice::WriteOnly, QuaZipNewInfo(headerFileName));
    mHeaderFileName=mPath+"/"+POINTCLOUDFILE_MANAGER_FILE_NAME;
    QFile headerFile(mHeaderFileName);
    headerFile.open(QIODevice::WriteOnly);
    QDataStream headerOut(&headerFile);
    headerOut<<mSRID;
    headerOut<<mCrsDescription;
    headerOut<<mCrsProj4String;
    headerOut<<mHeightType;
    headerOut<<mGridSize;
    headerOut<<mProjectType;
    headerOut<<mParameterValueByCode;

    headerOut<<mMinimumFc;
    headerOut<<mMinimumSc;
    headerOut<<mMinimumTc;
    headerOut<<mMaximumDensity;
    headerOut<<mFilesIndex;
    headerOut<<mFileByIndex;
    headerOut<<mNewFilesIndex;
    headerOut<<mTilesName;
    headerOut<<mTilesNumberOfPoints;
    headerOut<<mTilesContainedInROIs;
    headerOut<<mTilesOverlapsWithROIs;
    headerOut<<mTilesByFileIndex;

    quint16 numberOfROIs=mPtrROIs.size();
    headerOut<<numberOfROIs;
    if(numberOfROIs>0)
    {
        QMap<QString,OGRGeometry*>::const_iterator iterPtrRois=mPtrROIs.begin();
        while(iterPtrRois!=mPtrROIs.end())
        {
            QString roiId=iterPtrRois.key();
            OGRGeometry* ptrGeometry=iterPtrRois.value();
            char* ptrWKT;
            if(OGRERR_NONE!=ptrGeometry->exportToWkt(&ptrWKT))
            {
                strError=QObject::tr("PointCloudFile::writeHeader");
                strError+=QObject::tr("\nError exporting geometry to wkt for ROI id:\n%1").arg(roiId);
                return(false);
            }
            QString roiWkt=QString::fromLatin1(ptrWKT);
            headerOut<<roiId<<roiWkt;
            iterPtrRois++;
        }
        char* ptrUnionWKT;
        if(OGRERR_NONE!=mPtrROIsUnion->exportToWkt(&ptrUnionWKT))
        {
            strError=QObject::tr("PointCloudFile::writeHeader");
            strError+=QObject::tr("\nError exporting geometry to wkt for ROI union");
            return(false);
        }
        QString roiUnionWkt=QString::fromLatin1(ptrUnionWKT);
        QString roiUnionId=POINTCLOUDFILE_PROCESS_ROI_UNION_ID;
        headerOut<<roiUnionId<<roiUnionWkt;
    }
    headerFile.close();
    return(true);
}

void PointCloudFile::mpAddPointCloudFile(QString inputFileName)
{
    QString strError,strAuxError;
    QMap<int,QMap<int,int> > tilesNumberOfPoints; // nuevos para este fichero
    double minX=1000000000.0;
    double minY=1000000000.0;
    double minZ=1000000000.0;
    double maxX=-1000000000.0;
    double maxY=-1000000000.0;
    double maxZ=-1000000000.0;
    QFileInfo inputFileInfo(inputFileName);
    QString inputFileBaseName=inputFileInfo.completeBaseName();
    QString tilesPointsFileZipFileName=mPath+"/"+inputFileBaseName+"."+POINTCLOUDFILE_DHL_SUFFIX;
    if(QFile::exists(tilesPointsFileZipFileName))
    {
        if(!QFile::remove(tilesPointsFileZipFileName))
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError removing existing file:\n%1")
                    .arg(tilesPointsFileZipFileName);
            mStrErrorMpProgressDialog=strError;
            emit(mPtrMpProgressDialog->canceled());
            return;
        }
    }

    QString tilesPointsFileZipFilePath=mPath+"/"+inputFileBaseName;
    QDir currentDir=QDir::currentPath();
    if(currentDir.exists(tilesPointsFileZipFilePath))
    {
        if(!removeDir(tilesPointsFileZipFilePath))
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError removing existing dir:\n%1")
                    .arg(tilesPointsFileZipFilePath);
            mStrErrorMpProgressDialog=strError;
            emit(mPtrMpProgressDialog->canceled());
            return;
        }
    }
    if(!currentDir.mkpath(tilesPointsFileZipFilePath))
    {
        strError=QObject::tr("\PointCloudFile::addPointCloudFile");
        strError+=QObject::tr("\nError making dir:\n%1")
                .arg(tilesPointsFileZipFilePath);
        mStrErrorMpProgressDialog=strError;
        emit(mPtrMpProgressDialog->canceled());
        return;
    }

    QString pointsClassFileName=mPath+"/"+inputFileBaseName+"."+POINTCLOUDFILE_PCS_SUFFIX;
    if(QFile::exists(pointsClassFileName))
    {
        if(!QFile::remove(pointsClassFileName))
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError removing existing file:\n%1")
                    .arg(pointsClassFileName);
            mStrErrorMpProgressDialog=strError;
            emit(mPtrMpProgressDialog->canceled());
            return;
        }
    }
//    QuaZip* ptrTilesPointsFileZip= new QuaZip(tilesPointsFileZipFileName);
//    if(!ptrTilesPointsFileZip->open(QuaZip::mdCreate))
//    {
//        int qazErrorCode=ptrTilesPointsFileZip->getZipError();
//        strError=QObject::tr("\PointCloudFile::addPointCloudFile");
//        strError+=QObject::tr("\nError creating file:\n%1\nError code:\n%2")
//                .arg(tilesPointsFileZipFileName).arg(QString::number(qazErrorCode));
//        return(false);
//    }
    std::string stdFileName=inputFileName.toStdString();
    const char* charFileName=stdFileName.c_str();

    bool storeColor=mStoredFields[POINTCLOUDFILE_PARAMETER_COLOR];
    bool storeGpsTime=mStoredFields[POINTCLOUDFILE_PARAMETER_GPS_TIME];
    bool storeUserData=mStoredFields[POINTCLOUDFILE_PARAMETER_USER_DATA];
    bool storeIntensity=mStoredFields[POINTCLOUDFILE_PARAMETER_INTENSITY];
    bool storeSourceId=mStoredFields[POINTCLOUDFILE_PARAMETER_SOURCE_ID];
    bool storeNir=mStoredFields[POINTCLOUDFILE_PARAMETER_NIR];
    bool storeReturn=mStoredFields[POINTCLOUDFILE_PARAMETER_RETURN];
    bool storeReturns=mStoredFields[POINTCLOUDFILE_PARAMETER_RETURNS];
    bool existsColor=false;
    bool existsGpsTime=false;
    bool existsUserData=false;
    bool existsIntensity=false;
    bool existsSourceId=false;
    bool existsNir=false;
    bool existsReturn=false;
    bool existsReturns=false;
    if(storeColor||storeGpsTime||storeUserData
            ||storeIntensity||storeSourceId||storeNir
            ||storeReturn||storeReturns)
    {
        LASreadOpener lasreadopener;
        lasreadopener.set_file_name(charFileName);
        if (!lasreadopener.active())
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError opening file:\n%1").arg(inputFileName);
            mStrErrorMpProgressDialog=strError;
            emit(mPtrMpProgressDialog->canceled());
            return;
        }
        LASreader* lasreader = lasreadopener.open();
        LASheader* lasheader = &lasreader->header;
        int variablesToFind=0;
        if(storeColor) variablesToFind++;
        if(storeGpsTime) variablesToFind++;
        if(storeUserData) variablesToFind++;
        if(storeIntensity) variablesToFind++;
        if(storeSourceId) variablesToFind++;
        if(storeNir) variablesToFind++;
        if(storeReturn) variablesToFind++;
        if(storeReturns) variablesToFind++;
        while(lasreader->read_point())
        {
            if(storeColor&&!existsColor)
            {
                if(lasreader->point.have_rgb)
                {
                    existsColor=true;
                    variablesToFind--;
                }
            }
            if(storeGpsTime&&!existsGpsTime)
            {
                if(lasreader->point.have_gps_time)
                {
                    existsGpsTime=true;
                    variablesToFind--;
                }
            }
            if(storeNir&&!existsNir)
            {
                if(lasreader->point.have_nir)
                {
                    existsNir=true;
                    variablesToFind--;
                }
            }
            if(storeIntensity&&!existsIntensity)
            {
                if(lasreader->point.get_intensity()!=0)
                {
                    existsIntensity=true;
                    variablesToFind--;
                }
            }
            if(storeUserData&&!existsUserData)
            {
                if(lasreader->point.get_user_data()!=0)
                {
                    existsUserData=true;
                    variablesToFind--;
                }
            }
            if(storeSourceId&&!existsSourceId)
            {
                if(lasreader->point.get_point_source_ID()!=0)
                {
                    existsSourceId=true;
                    variablesToFind--;
                }
            }
            if(storeReturn&&!existsReturn)
            {
                if(lasreader->point.get_return_number()!=0)
                {
                    existsReturn=true;
                    variablesToFind--;
                }
            }
            if(storeReturns&&!existsReturns)
            {
                if(lasreader->point.get_number_of_returns()!=0)
                {
                    existsReturns=true;
                    variablesToFind--;
                }
            }
            if(variablesToFind==0) break;
        }
        lasreader->close();
        delete lasreader;
    }

    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(charFileName);
    if (!lasreadopener.active())
    {
        strError=QObject::tr("\PointCloudFile::addPointCloudFile");
        strError+=QObject::tr("\nError opening file:\n%1").arg(inputFileName);
        mStrErrorMpProgressDialog=strError;
        emit(mPtrMpProgressDialog->canceled());
        return;
    }

    LASreader* lasreader = lasreadopener.open();
    LASheader* lasheader = &lasreader->header;
    int numberOfPoints=lasreader->npoints;
    double fileMinX=lasheader->min_x;
    double fileMinY=lasheader->min_y;
    double fileMaxX=lasheader->max_x;
    double fileMaxY=lasheader->max_y;
    double fileMinZ=lasheader->min_z;
    double fileMaxZ=lasheader->max_z;

    int pointsByStep=POINTCLOUDFILE_NUMBER_OF_POINTS_TO_PROCESS_BY_STEP;
    int pointPosition=-1;
    int step=0;
    int numberOfProcessedPoints=0;
    int numberOfProcessedPointsInStep=0;
    U8 pointDataFormat=lasheader->point_data_format;
    QMap<int,QMap<int,QString> > tilesPointsFileNames;
    QMap<int,QMap<int,QVector<quint8> > > tilesPointsClass;
    QMap<int,QMap<int,QMap<int,quint8> > > tilesPointsClassNewByPos; // se guarda vacío
//    QMap<int,QMap<int,QuaZipFile*> > tilesPtrPointsFiles;
    QMap<int,QMap<int,QFile*> > tilesPtrPointsFiles;
    QMap<int,QMap<int,QDataStream*> > tilesPtrPointsDataStreams;
    QMap<int,QMap<int,int> > tilesNop;
    while(lasreader->read_point())
    {
        double x=lasreader->point.get_X()*lasheader->x_scale_factor+lasheader->x_offset;
        double y=lasreader->point.get_Y()*lasheader->y_scale_factor+lasheader->y_offset;
        pointPosition++;
        numberOfProcessedPoints++;
        numberOfProcessedPointsInStep++;
        if(numberOfProcessedPointsInStep==pointsByStep)
        {
            mMutex.lock();
            int numberOfPointsToProcess=numberOfPoints-numberOfProcessedPoints;
            mNumberOfPointsToProcessByFileName[inputFileName]=numberOfPointsToProcess;
            QString dialogText=QObject::tr("Adding point cloud files");
            dialogText+=QObject::tr("\nNumber of point cloud files to process:%1").arg(mNumberOfFilesToProcess);
            dialogText+=QObject::tr("\n... progressing using %1 threads").arg(QThread::idealThreadCount());
    //                mPtrMpProgressDialog->setWindowTitle(title);
            QMap<QString,int>::iterator iter=mNumberOfPointsToProcessByFileName.begin();
            while(iter!=mNumberOfPointsToProcessByFileName.end())
            {
                QString auxInputFileName=iter.key();
                int auxNumberOfPointsToProcess=iter.value();
                QString strNumberOfPointsToProcess="All";
                if(auxNumberOfPointsToProcess!=-1)
                {
                    strNumberOfPointsToProcess=QString::number(auxNumberOfPointsToProcess);
                }
                dialogText+=QObject::tr("\nPoints to process %1 in file: %2")
                        .arg(strNumberOfPointsToProcess).arg(auxInputFileName);
                iter++;
            }
            mPtrMpProgressDialog->setLabelText(dialogText);
            mMutex.unlock();
            numberOfProcessedPointsInStep=0;
        }
        bool includedPoint=true;
        int tileX=qRound(floor(floor(x)/mGridSize)*mGridSize);
        int tileY=qRound(floor(floor(y)/mGridSize)*mGridSize);
        if(!mTilesName.contains(tileX))
        {
            includedPoint=false;
        }
        else if(!mTilesName[tileX].contains(tileY))
        {
            includedPoint=false;
        }
        if(!includedPoint)
        {
            continue;
        }
        if(mPtrROIsUnion!=NULL)
        {
            if(mTilesOverlapsWithROIs[tileX][tileY])
            {
                OGRGeometry* ptrPoint=NULL;
                ptrPoint=OGRGeometryFactory::createGeometry(wkbPoint);
                ((OGRPoint*)ptrPoint)->setX(x);
                ((OGRPoint*)ptrPoint)->setY(y);
                if(!mPtrROIsUnion->Contains(ptrPoint))
                {
                    includedPoint=false;
                }
                OGRGeometryFactory::destroyGeometry(ptrPoint);
            }
        }
        if(!includedPoint)
        {
            continue;
        }
        QString tileTableName="tile_"+QString::number(tileX)+"_"+QString::number(tileY);
        quint8 pointClass=lasreader->point.get_classification();
        quint16 ix=qRound((x-tileX)*1000.);
        quint16 iy=qRound((y-tileY)*1000.);
        double z=lasreader->point.get_Z()*lasheader->z_scale_factor+lasheader->z_offset;
        if(z<POINTCLOUDFILE_HEIGHT_MINIMUM_VALID_VALUE
                ||z>POINTCLOUDFILE_HEIGHT_MAXIMUM_VALID_VALUE)
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nIn file:\n%1").arg(inputFileName);
            strError+=QObject::tr("\nfor point x=%1, y=%2")
                    .arg(QString::number(x,'f',3))
                    .arg(QString::number(y,'f',3));
            strError+=QObject::tr("\nz: %1 out of valid domain: [%2,%3]")
                    .arg(QString::number(z,'f',3))
                    .arg(QString::number(POINTCLOUDFILE_HEIGHT_MINIMUM_VALID_VALUE,'f',3))
                    .arg(QString::number(POINTCLOUDFILE_HEIGHT_MAXIMUM_VALID_VALUE,'f',3));
            mStrErrorMpProgressDialog=strError;
            emit(mPtrMpProgressDialog->canceled());
            return;
        }
        double zt=z-POINTCLOUDFILE_HEIGHT_MINIMUM_VALID_VALUE;
        quint8 z_pc=zt*1000.0-floor(zt*10.)*100.;
        double nz=floor(zt*10.);
        quint8 z_pa=floor(nz/256.);
        quint8 z_pb=nz-z_pa*256.;
//        double zc=(z_pa*256.0+z_pb)/10.+z_pc/1000.+POINTCLOUDFILE_HEIGHT_MINIMUM_VALID_VALUE;
        QDataStream* tilePtrPointsDataStream=NULL;
        bool existsTileFile=true;
        if(!tilesPointsFileNames.contains(tileX))
        {
            existsTileFile=false;
        }
        else if(!tilesPointsFileNames[tileX].contains(tileY))
        {
            existsTileFile=false;
        }
        if(!existsTileFile)
        {
//            QString tilePointsFileName=tileTableName;
//            QuaZipFile* ptrTilePointsFile=new QuaZipFile(ptrTilesPointsFileZip);
//            if(!ptrTilePointsFile->open(QIODevice::WriteOnly,
//                                        QuaZipNewInfo(tilePointsFileName)))
//            {
//                QString qazError=ptrTilePointsFile->getZipError();
//                strError=QObject::tr("\PointCloudFile::addPointCloudFile");
//                strError+=QObject::tr("\nIn file:\n%1\nError creating file:\n%2\nError:\n%3")
//                        .arg(tilesPointsFileZipFileName).arg(tilePointsFileName).arg(qazError);
//                return(false);
//            }
            QString tilePointsFileName=tilesPointsFileZipFilePath+"/"+tileTableName;
            QFile* ptrTilePointsFile=new QFile(tilePointsFileName);
            if(!ptrTilePointsFile->open(QIODevice::WriteOnly))
            {
                strError=QObject::tr("\PointCloudFile::addPointCloudFile");
                strError+=QObject::tr("\nError creating file:\n%1")
                        .arg(tilePointsFileName);
                mStrErrorMpProgressDialog=strError;
                emit(mPtrMpProgressDialog->canceled());
                return;
            }
            tilePtrPointsDataStream=new QDataStream(ptrTilePointsFile);
            tilesPointsFileNames[tileX][tileY]=tilePointsFileName;
            tilesPtrPointsFiles[tileX][tileY]=ptrTilePointsFile;
            tilesPtrPointsDataStreams[tileX][tileY]=tilePtrPointsDataStream;
            QVector<quint8> auxClasses;
            tilesPointsClass[tileX][tileY]=auxClasses;
            tilesNop[tileX][tileY]=0;
        }
        else
        {
            tilePtrPointsDataStream=tilesPtrPointsDataStreams[tileX][tileY];
        }
        (*tilePtrPointsDataStream)<<ix<<iy<<z_pa<<z_pb<<z_pc;
        if(existsColor)
        {
            quint16 color_r=lasreader->point.get_R();
            quint16 color_g=lasreader->point.get_G();
            quint16 color_b=lasreader->point.get_B();
            if(mNumberOfColorBytes==1)
            {
                quint8 r=floor(double(color_r)/256.0);
                if(r>255) r=255;
                quint8 g=floor(double(color_g)/256.0);
                if(g>255) g=255;
                quint8 b=floor(double(color_b)/256.0);
                if(b>255) b=255;
                (*tilePtrPointsDataStream)<<r;
                (*tilePtrPointsDataStream)<<g;
                (*tilePtrPointsDataStream)<<b;
            }
            else
            {
                (*tilePtrPointsDataStream)<<color_r;
                (*tilePtrPointsDataStream)<<color_g;
                (*tilePtrPointsDataStream)<<color_b;
            }
        }
        if(existsGpsTime)
        {
            double gpsTime=lasreader->point.get_gps_time();
            int dayOfWeek=floor(gpsTime/24./60./60.);
            gpsTime-=(dayOfWeek*24.*60.*60.);
            int hours=floor(gpsTime/60./60.);
            gpsTime-=(hours*60.*60.);
            double dblMinutes=gpsTime/60.;
            int ms=qRound((dblMinutes*60.)*pow(10.,6.));
            quint8 msb1=floor(ms/256./256./256.);
            quint8 msb2=floor((ms-msb1*256.*256.*256.)/256./256.);
            quint8 msb3=floor((ms-msb1*256.*256.*256.-msb2*256.*256.)/256.);
            quint8 gpsTimeDow=dayOfWeek; // 3 bits, 0-7
            quint8 gpsHour=hours; // 5 bits, 0-23
            quint8 gpsDowHourPackit;
            gpsDowHourPackit = (gpsTimeDow << 3) | gpsHour;
            (*tilePtrPointsDataStream)<<gpsDowHourPackit;
            (*tilePtrPointsDataStream)<<msb1;
            (*tilePtrPointsDataStream)<<msb2;
            (*tilePtrPointsDataStream)<<msb3;
        }
        if(existsUserData)
        {
            quint8 userData=lasreader->point.get_user_data();
            (*tilePtrPointsDataStream)<<userData;
        }
        if(existsIntensity)
        {
            quint16 intensity=lasreader->point.get_intensity();
            (*tilePtrPointsDataStream)<<intensity;
        }
        if(existsSourceId)
        {
            quint16 sourceId=lasreader->point.get_point_source_ID();
            (*tilePtrPointsDataStream)<<sourceId;
        }
        if(existsNir)
        {
            quint16 nir=lasreader->point.get_NIR();
            if(mNumberOfColorBytes==1)
            {
                quint8 ir=floor(double(nir)/256.0);
                if(ir>255) ir=255;
                (*tilePtrPointsDataStream)<<ir;
            }
            else
            {
                (*tilePtrPointsDataStream)<<nir;
            }
        }
        if(existsReturn)
        {
            quint8 returnNumber=lasreader->point.get_return_number();
            (*tilePtrPointsDataStream)<<returnNumber;
        }
        if(existsReturns)
        {
            quint8 numberOfReturns=lasreader->point.get_number_of_returns();
            (*tilePtrPointsDataStream)<<numberOfReturns;
        }
        double minX=floor(x);
        double minY=floor(y);
        double minZ=floor(z);
        mMutex.lock();
        if(minX<mMinimumFc) mMinimumFc=minX;
        if(minY<mMinimumSc) mMinimumSc=minY;
        if(minZ<mMinimumTc) mMinimumTc=minZ;
        mMutex.unlock();
        tilesPointsClass[tileX][tileY].push_back(pointClass);
        tilesNumberOfPoints[tileX][tileY]=tilesNumberOfPoints[tileX][tileY]+1;
        tilesNop[tileX][tileY]=tilesNop[tileX][tileY]+1;
    }
    lasreader->close();
    delete lasreader;
    if(tilesPointsClass.size()==0)
    {
        if(!removeDir(tilesPointsFileZipFilePath))
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError removing directory:\n%1")
                    .arg(tilesPointsFileZipFilePath);
            mStrErrorMpProgressDialog=strError;
            emit(mPtrMpProgressDialog->canceled());
            return;
        }
        return;
    }
    int fileIndex=-1;
    if(mFilesIndex.contains(inputFileName))
    {
        fileIndex=mFilesIndex[inputFileName];
    }
    else
    {
        /*
        QString wktGeometry="POLYGON((";
        wktGeometry+=QString::number(floor(fileMinX),'f',0);
        wktGeometry+=" ";
        wktGeometry+=QString::number(floor(fileMinY),'f',0);
        wktGeometry+=",";
        wktGeometry+=QString::number(floor(fileMinX),'f',0);
        wktGeometry+=" ";
        wktGeometry+=QString::number(floor(fileMaxY),'f',0);
        wktGeometry+=",";
        wktGeometry+=QString::number(floor(fileMaxX),'f',0);
        wktGeometry+=" ";
        wktGeometry+=QString::number(floor(fileMaxY),'f',0);
        wktGeometry+=",";
        wktGeometry+=QString::number(floor(fileMaxX),'f',0);
        wktGeometry+=" ";
        wktGeometry+=QString::number(floor(fileMinY),'f',0);
        wktGeometry+=",";
        wktGeometry+=QString::number(floor(fileMinX),'f',0);
        wktGeometry+=" ";
        wktGeometry+=QString::number(floor(fileMinY),'f',0);
        wktGeometry+="))";
        QByteArray byteArrayWktGeometry = wktGeometry.toUtf8();
        char *charsWktGeometry = byteArrayWktGeometry.data();
        OGRGeometry* ptrGeometry;
        ptrGeometry=OGRGeometryFactory::createGeometry(wkbPolygon);
        if(OGRERR_NONE!=ptrGeometry->importFromWkt(&charsWktGeometry))
        {
            strError=QObject::tr("PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError making geometry from WKT: %1").arg(wktGeometry);
            lasreader->close();
            return(false);
        }
        */
        mMutex.lock();
        fileIndex=mNewFilesIndex;
        mNewFilesIndex++;
//        mFilePtrGeometryByIndex[fileIndex]=ptrGeometry;
        mFilesIndex[inputFileName]=fileIndex;
        mFileByIndex[fileIndex]=inputFileName;
        QMap<int,QVector<int> > aux;
        mTilesByFileIndex[fileIndex]=aux;
        mMutex.unlock();
    }

//    QMap<int,QMap<int,QuaZipFile*> >::iterator iterTileXZf=tilesPtrPointsFiles.begin();
    QMap<int,QMap<int,QFile*> >::iterator iterTileXZf=tilesPtrPointsFiles.begin();
    while(iterTileXZf!=tilesPtrPointsFiles.end())
    {
        int tileX=iterTileXZf.key();
//        QMap<int,QuaZipFile*>::iterator iterTileYZf=iterTileXZf.value().begin();
        QMap<int,QFile*>::iterator iterTileYZf=iterTileXZf.value().begin();
        while(iterTileYZf!=iterTileXZf.value().end())
        {
            int tileY=iterTileYZf.key();
//            QuaZipFile* ptrFile=iterTileYZf.value();
            QFile* ptrFile=iterTileYZf.value();
            ptrFile->close();
//            int qazErrorCode=ptrFile->getZipError();
//            if(UNZ_OK!=qazErrorCode)
//            {
//                int numberOfPoints=tilesPointsClass[tileX][tileY].size();
//                QString tileFileName=tilesPointsFileNames[tileX][tileY];
//                strError=QObject::tr("\PointCloudFile::addPointCloudFile");
//                strError+=QObject::tr("\nIn file:\n%1\nError closing file:\n%2\nError code:\n%3")
//                        .arg(tilesPointsFileZipFileName).arg(tileFileName).arg(QString::number(qazErrorCode));
//                return(false);
//            }
            delete(tilesPtrPointsDataStreams[tileX][tileY]);
            tilesPtrPointsDataStreams[tileX][tileY]=NULL;
            delete(ptrFile);
            tilesPtrPointsFiles[tileX][tileY]=NULL;
            iterTileYZf++;
        }
        iterTileXZf++;
    }
    if(!JlCompress::compressDir(tilesPointsFileZipFileName,
                                tilesPointsFileZipFilePath))
    {
        strError=QObject::tr("\PointCloudFile::addPointCloudFile");
        strError+=QObject::tr("\nError compressing directory:\n%1")
                .arg(tilesPointsFileZipFilePath);
        mStrErrorMpProgressDialog=strError;
        emit(mPtrMpProgressDialog->canceled());
        return;
    }
    if(!removeDir(tilesPointsFileZipFilePath))
    {
        strError=QObject::tr("\PointCloudFile::addPointCloudFile");
        strError+=QObject::tr("\nError removing directory:\n%1")
                .arg(tilesPointsFileZipFilePath);
        mStrErrorMpProgressDialog=strError;
        emit(mPtrMpProgressDialog->canceled());
        return;
    }
//    ptrTilesPointsFileZip->close();
//    int qazErrorCode=ptrTilesPointsFileZip->getZipError();
//    if(UNZ_OK!=qazErrorCode)
//    {
//        strError=QObject::tr("\PointCloudFile::addPointCloudFile");
//        strError+=QObject::tr("\nError closing file:\n%1\nError:\n%2")
//                .arg(tilesPointsFileZipFileName).arg(QString::number(qazErrorCode));
//        return(false);
//    }
    QFile pointsClassFile(pointsClassFileName);
    pointsClassFile.open(QIODevice::WriteOnly);
    QDataStream outPointsClass(&pointsClassFile);   // we will serialize the data into the file
    QMap<QString,bool> exitsFields;
    exitsFields[POINTCLOUDFILE_PARAMETER_COLOR]=existsColor;
    exitsFields[POINTCLOUDFILE_PARAMETER_GPS_TIME]=existsGpsTime;
    exitsFields[POINTCLOUDFILE_PARAMETER_USER_DATA]=existsUserData;
    exitsFields[POINTCLOUDFILE_PARAMETER_INTENSITY]=existsIntensity;
    exitsFields[POINTCLOUDFILE_PARAMETER_SOURCE_ID]=existsSourceId;
    exitsFields[POINTCLOUDFILE_PARAMETER_NIR]=existsNir;
    exitsFields[POINTCLOUDFILE_PARAMETER_RETURN]=existsReturn;
    exitsFields[POINTCLOUDFILE_PARAMETER_RETURNS]=existsReturns;
    outPointsClass<<tilesNop;
    outPointsClass<<exitsFields;
    outPointsClass<<tilesPointsClass;
    outPointsClass<<tilesPointsClassNewByPos;
    pointsClassFile.close();
    mMutex.lock();
    QMap<int,QMap<int,int> >::const_iterator iterTileX=tilesNumberOfPoints.begin();
    while(iterTileX!=tilesNumberOfPoints.end())
    {
        int tileX=iterTileX.key();
        QMap<int,int>::const_iterator iterTileY=iterTileX.value().begin();
        while(iterTileY!=iterTileX.value().end())
        {
            int tileY=iterTileY.key();
            int numberOfPoints=iterTileY.value();
            if(numberOfPoints>0)
            {
                mTilesNumberOfPoints[tileX][tileY]=mTilesNumberOfPoints[tileX][tileY]+numberOfPoints;
                if(!mTilesByFileIndex[fileIndex].contains(tileX))
                {
                    QVector<int> aux;
                    mTilesByFileIndex[fileIndex][tileX]=aux;
                }
                if(mTilesByFileIndex[fileIndex][tileX].indexOf(tileY)==-1)
                {
                    mTilesByFileIndex[fileIndex][tileX].push_back(tileY);
                }
            }
            iterTileY++;
        }
        iterTileX++;
    }
    if(mTilesByFileIndex[fileIndex].size()==0)
    {
        mTilesByFileIndex.remove(fileIndex);
        QString fileName=mFileByIndex[fileIndex];
        mFilesIndex.remove(fileName);
        mFileByIndex.remove(fileIndex);
        /*
        OGRGeometry* ptrGeometry=mFilePtrGeometryByIndex[fileIndex];
        if(ptrGeometry!=NULL)
        {
            OGRGeometryFactory::destroyGeometry(ptrGeometry);
            mFilePtrGeometryByIndex[fileIndex]=NULL;
        }
        mFilePtrGeometryByIndex.remove(fileIndex);
        */
    }
    mZipFilePathPointsByIndex[fileIndex]=tilesPointsFileZipFilePath;
    mZipFilePointsByIndex[fileIndex]=tilesPointsFileZipFileName;
    mClassesFileByIndex[fileIndex]=pointsClassFileName;
    if(mUpdateHeader)
    {
        if(!writeHeader(strAuxError))
        {
            strError=QObject::tr("\PointCloudFile::addPointCloudFile");
            strError+=QObject::tr("\nError writing header after add file:\n%1").arg(inputFileName);
            mStrErrorMpProgressDialog=strError;
            mMutex.unlock();
            emit(mPtrMpProgressDialog->canceled());
            return;
        }
    }
    mMutex.unlock();
    return;
}

void PointCloudFile::mpAddTilesGeometry(int tilePos)
{
    QString strError;
    int tileX=mTilesXToProcess[tilePos];
    int tileY=mTilesYToProcess[tilePos];
    if(!mTilesName.contains(tileX)) return;
    if(!mTilesName[tileX].contains(tileY)) return;
    QString wktGeometry="POLYGON((";
    wktGeometry+=QString::number(tileX);
    wktGeometry+=" ";
    wktGeometry+=QString::number(tileY);
    wktGeometry+=",";
    wktGeometry+=QString::number(tileX);
    wktGeometry+=" ";
    wktGeometry+=QString::number(qRound(tileY+mGridSize),'f',0);
    wktGeometry+=",";
    wktGeometry+=QString::number(qRound(tileX+mGridSize),'f',0);
    wktGeometry+=" ";
    wktGeometry+=QString::number(qRound(tileY+mGridSize),'f',0);
    wktGeometry+=",";
    wktGeometry+=QString::number(qRound(tileX+mGridSize),'f',0);
    wktGeometry+=" ";
    wktGeometry+=QString::number(tileY);
    wktGeometry+=",";
    wktGeometry+=QString::number(tileX);
    wktGeometry+=" ";
    wktGeometry+=QString::number(tileY);
    wktGeometry+="))";
    QByteArray byteArrayWktGeometry = wktGeometry.toUtf8();
    char *charsWktGeometry = byteArrayWktGeometry.data();
    OGRGeometry* ptrGeometry;
    ptrGeometry=OGRGeometryFactory::createGeometry(wkbPolygon);
    if(OGRERR_NONE!=ptrGeometry->importFromWkt(&charsWktGeometry))
    {
        strError=QObject::tr("PointCloudFile::mpAddTilesGeometry");
        strError+=QObject::tr("\nError making geometry from WKT: %1").arg(wktGeometry);
        mStrErrorMpProgressDialog=strError;
        emit(mPtrMpProgressDialog->canceled());
        return;
    }
    mMutex.lock();
    mTilesGeometry[tileX][tileY]=ptrGeometry;
    mMutex.unlock();
    return;
}

void PointCloudFile::mpGetTilesWktGeometry(int tilePos)
{
    QString strError;
    int tileX=mTilesXToProcess[tilePos];
    int tileY=mTilesYToProcess[tilePos];
    if(!mTilesName.contains(tileX)) return;
    if(!mTilesName[tileX].contains(tileY)) return;
    QString tileTableName=mTilesName[tileX][tileY];
    if(!mTilessWkt.contains(tileTableName))
    {
        if(!mTilesGeometry.contains(tileX))
        {
            strError=QObject::tr("PointCloudFile::mpGetTilesXWktGeometry");
            strError+=QObject::tr("\nNot exists geometry for tile:(%1,%2)")
                    .arg(QString::number(tileX)).arg(QString::number(tileY));
            mStrErrorMpProgressDialog=strError;
            emit(mPtrMpProgressDialog->canceled());
            return;
        }
        if(!mTilesGeometry[tileX].contains(tileY))
        {
            strError=QObject::tr("PointCloudFile::mpGetTilesXWktGeometry");
            strError+=QObject::tr("\nNot exists geometry for tile:(%1,%2)")
                    .arg(QString::number(tileX)).arg(QString::number(tileY));
            mStrErrorMpProgressDialog=strError;
            emit(mPtrMpProgressDialog->canceled());
            return;
        }
        mMutex.lock();
        OGRGeometry* ptrGeometry=mTilesGeometry[tileX][tileY];
        char* ptrWKT;
        if(OGRERR_NONE!=ptrGeometry->exportToWkt(&ptrWKT))
        {
            strError=QObject::tr("PointCloudFile::mpGetTilesXWktGeometry");
            strError+=QObject::tr("\nError exporting to WKT geometry for tile:(%1,%2)")
                    .arg(QString::number(tileX)).arg(QString::number(tileY));
            mStrErrorMpProgressDialog=strError;
            mMutex.unlock();
            emit(mPtrMpProgressDialog->canceled());
            return;
        }
        QString tileWkt=QString::fromLatin1(ptrWKT);
        mTilessWkt[tileTableName]=tileWkt;
        mMutex.unlock();
    }
    return;
}

void PointCloudFile::mpGetTilesFromWktGeometry(int tilePos)
{
    QString strError;
    int tileX=mTilesXToProcess[tilePos];
    int tileY=mTilesYToProcess[tilePos];
    if(!mTilesGeometry.contains(tileX)) return;
    if(!mTilesGeometry[tileX].contains(tileY)) return;
    OGRwkbGeometryType geometryType=mMpPtrGeometry->getGeometryType();
    if(geometryType==wkbPolygon
            ||geometryType==wkbMultiPolygon
            ||geometryType==wkbPolygon25D
            ||geometryType==wkbMultiPolygon25D
            ||geometryType==wkbPolygonM
            ||geometryType==wkbMultiPolygonM
            ||geometryType==wkbPolygonZM
            ||geometryType==wkbMultiPolygonZM)
    {
        QString tileTableName=mTilesName[tileX][tileY];
        OGRGeometry* ptrTileGeometry=mTilesGeometry[tileX][tileY];
        mMutex.lock();
        if(mMpPtrGeometry->Overlaps(ptrTileGeometry))
        {
            mMpTilesTableName[tileX][tileY]=tileTableName;
        }
        else if(mMpPtrGeometry->Contains(ptrTileGeometry))
        {
            mMpTilesTableName[tileX][tileY]=tileTableName;
        }
        else if(mMpPtrGeometry->Within(ptrTileGeometry))
        {
            mMpTilesTableName[tileX][tileY]=tileTableName;
        }
        mMutex.unlock();
    }
    return;
}

void PointCloudFile::mpGetTilesNamesFromWktGeometry(int tilePos)
{
    QString strError;
    int tileX=mTilesXToProcess[tilePos];
    int tileY=mTilesYToProcess[tilePos];
    if(!mTilesGeometry.contains(tileX)) return;
    if(!mTilesGeometry[tileX].contains(tileY)) return;
    OGRwkbGeometryType geometryType=mMpPtrGeometry->getGeometryType();
    if(geometryType==wkbPolygon
            ||geometryType==wkbMultiPolygon
            ||geometryType==wkbPolygon25D
            ||geometryType==wkbMultiPolygon25D
            ||geometryType==wkbPolygonM
            ||geometryType==wkbMultiPolygonM
            ||geometryType==wkbPolygonZM
            ||geometryType==wkbMultiPolygonZM)
    {
        QString tileTableName=mTilesName[tileX][tileY];
        OGRGeometry* ptrTileGeometry=mTilesGeometry[tileX][tileY];
        mMutex.lock();
        if(mMpPtrGeometry->Overlaps(ptrTileGeometry))
        {
            mMpTilesTableName[tileX][tileY]=tileTableName;
            mTilesOverlaps[tileX][tileY]=true;
        }
        else if(mMpPtrGeometry->Contains(ptrTileGeometry))
        {
            mMpTilesTableName[tileX][tileY]=tileTableName;
            mTilesOverlaps[tileX][tileY]=false;
        }
        else if(mMpPtrGeometry->Within(ptrTileGeometry))
        {
            mMpTilesTableName[tileX][tileY]=tileTableName;
            mTilesOverlaps[tileX][tileY]=false;
        }
        mMutex.unlock();
    }
    return;
}

void PointCloudFile::mpGetPointsFromWktGeometryByTilePosition(int tilePos)
{
    QString strError;
    int tileX=mTilesXToProcess[tilePos];
    int tileY=mTilesYToProcess[tilePos];
    int numberOfPoints=mTilesNop[tileX][tileY];
    QString tileTableName=mTilesName[tileX][tileY];
    QuaZip zipFilePointsTile;
    zipFilePointsTile.setZipName(mZipFileNamePoints);
//        QuaZip mZipFilePoints(zipFileNamePoints);
    if(!zipFilePointsTile.open(QuaZip::mdUnzip))
    {
        strError=QObject::tr("PointCloudFile::mpGetPointsFromWktGeometryByTilePosition");
        strError+=QObject::tr("\nError opening file:\n%1\nError:\n%2")
                .arg(mZipFileNamePoints).arg(QString::number(zipFilePointsTile.getZipError()));
        mStrErrorMpProgressDialog=strError;
        emit(mPtrMpProgressDialog->canceled());
        return;
    }
    if(!zipFilePointsTile.setCurrentFile(tileTableName))
    {
        strError=QObject::tr("PointCloudFile::mpGetPointsFromWktGeometryByTilePosition");
        strError+=QObject::tr("\nNot exists: %1 in file:\n%2\nError:\n%3")
                .arg(tileTableName).arg(mZipFileNamePoints)
                .arg(QString::number(zipFilePointsTile.getZipError()));
        mStrErrorMpProgressDialog=strError;
        emit(mPtrMpProgressDialog->canceled());
        return;
    }
    QuaZipFile inPointsFile(&zipFilePointsTile);
    if (!inPointsFile.open(QIODevice::ReadOnly))
    {
        strError=QObject::tr("PointCloudFile::mpGetPointsFromWktGeometryByTilePosition");
        strError+=QObject::tr("\nError opening: %1 in file:\n%2\nError:\n%3")
                .arg(tileTableName).arg(mZipFileNamePoints)
                .arg(QString::number(mZipFilePoints.getZipError()));
        mStrErrorMpProgressDialog=strError;
        emit(mPtrMpProgressDialog->canceled());
        return;
    }
    QDataStream inPoints(&inPointsFile);
    int pos=0;
    QVector<PCFile::Point> pointsInTile(numberOfPoints);
    int numberOfRealPoints=0; // porque puede haber puntos fuera del wkt
    while(!inPoints.atEnd())
    {
        quint16 ix,iy;
        quint8 z_pc,z_pa,z_pb;
        inPoints>>ix>>iy>>z_pa>>z_pb>>z_pc;
        PCFile::Point pto;
        pto.setPositionInTile(pos);
        if(pos>mTilesPointsClass[tileX][tileY].size())
        {
            strError=QObject::tr("PointCloudFile::mpGetPointsFromWktGeometryByTilePosition");
            strError+=QObject::tr("\nNot exists position: %1 in tile X: %2 tile Y: %3 in classes file:\n%4")
                    .arg(QString::number(pos)).arg(QString::number(tileX))
                    .arg(QString::number(tileY)).arg(mClassesFileName);
            mStrErrorMpProgressDialog=strError;
            emit(mPtrMpProgressDialog->canceled());
            return;
        }
        quint8 ptoClass=mTilesPointsClass[tileX][tileY][pos];
        pto.setClass(ptoClass);
        quint8 ptoClassNew=ptoClass;
        if(mTilesPointsClassNewByPos.contains(tileX))
        {
            if(mTilesPointsClassNewByPos[tileX].contains(tileY))
            {
                if(mTilesPointsClassNewByPos[tileX][tileY].contains(pos))
                {
                    ptoClassNew=mTilesPointsClassNewByPos[tileX][tileY][pos];
                }
            }
        }
        pto.setClassNew(ptoClassNew);
        pto.setCoordinates(ix,iy,z_pa,z_pb,z_pc);
        if(mExistsColor)
        {
            if(mNumberOfColorBytes==1)
            {
                quint8 color_r,color_g,color_b;
                inPoints>>color_r>>color_g>>color_b;
                pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_RED,color_r);
                pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_GREEN,color_g);
                pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_BLUE,color_b);
            }
            else
            {
                quint16 color_r,color_g,color_b;
                inPoints>>color_r>>color_g>>color_b;
                pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_RED,color_r);
                pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_GREEN,color_g);
                pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_BLUE,color_b);
            }
        }
        if(mExistsGpsTime)
        {
            quint8 gpsDowHourPackit;
            quint8 msb1,msb2,msb3;
            inPoints>>gpsDowHourPackit>>msb1>>msb2>>msb3;
            pto.setGpsTime(gpsDowHourPackit,msb1,msb2,msb3);
        }
        if(mExistsUserData)
        {
            quint8 userData;
            inPoints>>userData;
            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_USER_DATA,userData);
        }
        if(mExistsIntensity)
        {
            quint16 intensity;
            inPoints>>intensity;
            pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_INTENSITY,intensity);
        }
        if(mExistsSourceId)
        {
            quint16 sourceId;
            inPoints>>sourceId;
            pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_SOURCE_ID,sourceId);
        }
        if(mExistsNir)
        {
            if(mNumberOfColorBytes==1)
            {
                quint8 nir;
                inPoints>>nir;
                pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_NIR,nir);
            }
            else
            {
                quint16 nir;
                inPoints>>nir;
                pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_NIR,nir);
            }
        }
        if(mExistsReturn)
        {
            quint8 returnNumber;
            inPoints>>returnNumber;
            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_RETURN,returnNumber);
        }
        if(mExistsReturns)
        {
            quint8 numberOfReturns;
            inPoints>>numberOfReturns;
            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_RETURNS,numberOfReturns);
        }
        pos++;
        if(!mTilesFullGeometry)
        {
            if(mTilesOverlaps[tileX][tileY])
            {
                OGRGeometry* ptrPoint=NULL;
                ptrPoint=OGRGeometryFactory::createGeometry(wkbPoint);
                double x=tileX+ix/1000.;
                double y=tileY+iy/1000.;
                ((OGRPoint*)ptrPoint)->setX(x);
                ((OGRPoint*)ptrPoint)->setY(y);
                if(!mMpPtrGeometry->Contains(ptrPoint))
                {
                    OGRGeometryFactory::destroyGeometry(ptrPoint);
                    continue;
                }
                OGRGeometryFactory::destroyGeometry(ptrPoint);
            }
        }
        pointsInTile[numberOfRealPoints]=pto;
        numberOfRealPoints++;
    }
    inPointsFile.close();
    if(numberOfRealPoints<numberOfPoints)
    {
        pointsInTile.resize(numberOfRealPoints);
    }
    zipFilePointsTile.close();
    mMutex.lock();
    if(pointsInTile.size()>0)
    {
        mPointsByTile[tileX][tileY]=pointsInTile;
    }
    if(numberOfRealPoints>0)
    {
        mMpTilesNumberOfPointsInFile[tileX][tileY]=mMpTilesNumberOfPointsInFile[tileX][tileY]+numberOfRealPoints;
    }
    mMutex.unlock();
    return;
}

bool PointCloudFile::setOutputPath(QString value,
                                   QString &strError)
{
    QDir currentDir=QDir::currentPath();
    if(!currentDir.exists(value))
    {
        if(!currentDir.mkpath(value))
        {
            strError=QObject::tr("PointCloudFile::setOutputPath");
            strError+=QObject::tr("\nError making path:\n%1").arg(value);
            return(false);
        }
    }
    mOutputPath=value;
    return(true);
}

bool PointCloudFile::setTempPath(QString value,
                                 QString &strError)
{
    QDir currentDir=QDir::currentPath();
    if(!currentDir.exists(value))
    {
        if(!currentDir.mkpath(value))
        {
            strError=QObject::tr("PointCloudFile::setTempPath");
            strError+=QObject::tr("\nError making path:\n%1").arg(value);
            return(false);
        }
    }
    mTempPath=value;
    return(true);
}

bool PointCloudFile::updatePoints(QString strAction,
                                  quint8 classValue,
                                  QMap<int, QMap<int, QVector<int> > > &pointFileIdByTile,
                                  QMap<int, QMap<int, QVector<int> > > &pointPositionByTile,
//                                  QMap<int, QMap<int, QVector<quint8> > > &pointClassNewByTile,
//                                  QMap<int, QMap<int, QVector<quint8> > > &pointClassByTile,
                                  QMap<quint8, bool> &lockedClasses,
                                  QString &strError)
{
    if(strAction.compare(POINTCLOUDFILE_ACTION_CHANGE_CLASS,Qt::CaseInsensitive)!=0
            &&strAction.compare(POINTCLOUDFILE_ACTION_RECOVER_ORIGINAL_CLASS,Qt::CaseInsensitive)!=0
            &&strAction.compare(POINTCLOUDFILE_ACTION_DELETE,Qt::CaseInsensitive)!=0
            &&strAction.compare(POINTCLOUDFILE_ACTION_RECOVER_DELETED,Qt::CaseInsensitive)!=0)
    {
        strError=QObject::tr("PointCloudFile::updatePoints");
        strError+=QObject::tr("\nInvalid action: %1").arg(strAction);
        return(false);
    }
    if(strAction.compare(POINTCLOUDFILE_ACTION_CHANGE_CLASS,Qt::CaseInsensitive)==0
        &&classValue==POINTCLOUDFILE_ACTION_ALL_CLASSES_VALUE)
    {
        strError=QObject::tr("PointCloudFile::updatePoints");
        strError+=QObject::tr("\nInvalid action: %1 for All Classes").arg(strAction);
        return(false);
    }
    QWidget* ptrWidget=new QWidget();
    QProgressDialog* ptrProgress=NULL;
    QMap<int,QMap<int,QMap<int,QVector<int> > > > pointsIndexByTilesByFileIndex;
    QMap<int, QMap<int, QVector<int> > >::const_iterator iterTileX=pointFileIdByTile.begin();
    while(iterTileX!=pointFileIdByTile.end())
    {
        int tileX=iterTileX.key();
        QMap<int, QVector<int> >::const_iterator iterTileY=iterTileX.value().begin();
        while(iterTileY!=iterTileX.value().end())
        {
            int tileY=iterTileY.key();
            QVector<int> filesIndex=iterTileY.value();
            for(int i=0;i<filesIndex.size();i++)
            {
                int fileIndex=filesIndex[i];
                if(!pointsIndexByTilesByFileIndex.contains(fileIndex))
                {
                    QMap<int,QMap<int,QVector<int> > > aux;
                    pointsIndexByTilesByFileIndex[fileIndex]=aux;
                }
                else if(!pointsIndexByTilesByFileIndex[fileIndex].contains(tileX))
                {
                    QMap<int,QVector<int> > aux;
                    pointsIndexByTilesByFileIndex[fileIndex][tileX]=aux;
                }
                pointsIndexByTilesByFileIndex[fileIndex][tileX][tileY].push_back(i);
            }
            iterTileY++;
        }
        iterTileX++;
    }
    int numberOfSteps=pointsIndexByTilesByFileIndex.size();
    QDir auxDir=QDir::currentPath();
    if(ptrWidget!=NULL)
    {
        QString title=QObject::tr("PointCloudFile::updatePoints");
        QString msgGlobal=QObject::tr("Updating points from %1 files and tiles")
                .arg(QString::number(numberOfSteps));
        ptrProgress=new QProgressDialog(title, "Abort",0,numberOfSteps, ptrWidget);
        ptrProgress->setWindowModality(Qt::WindowModal);
        ptrProgress->setLabelText(msgGlobal);
        ptrProgress->show();
        qApp->processEvents();
    }
    int step=0;
    QMap<int,QMap<int,QMap<int,QVector<int> > > >::const_iterator iterFiles=pointsIndexByTilesByFileIndex.begin();
    while(iterFiles!=pointsIndexByTilesByFileIndex.end())
    {
        step++;
        if(ptrWidget!=NULL)
        {
            ptrProgress->setValue(step);
            qApp->processEvents();
        }
        int fileIndex=iterFiles.key();
        if(!mClassesFileByIndex.contains(fileIndex))
        {
            strError=QObject::tr("PointCloudFile::updatePoints");
            strError+=QObject::tr("\nThere is no classes file for index: %1")
                    .arg(QString::number(fileIndex));
            if(ptrWidget!=NULL)
            {
                ptrProgress->setValue(step);
                qApp->processEvents();
            }
            return(false);
        }
        QString pointsClassFileName=mClassesFileByIndex[fileIndex];
        QFile pointsClassFile(pointsClassFileName);
        if (!pointsClassFile.open(QIODevice::ReadOnly))
        {
            strError=QObject::tr("PointCloudFile::updatePoints");
            strError+=QObject::tr("\nError opening file:\n%1").arg(pointsClassFileName);
            if(ptrWidget!=NULL)
            {
                ptrProgress->setValue(step);
                qApp->processEvents();
            }
            return(false);
        }
        QDataStream inPointsClass(&pointsClassFile);
        QMap<QString,bool> existsFields;
        QMap<int,QMap<int,QVector<quint8> > > tilesPointsClass;
        QMap<int,QMap<int,QMap<int,quint8> > > tilesPointsClassNewByPos; // se guarda vacío
        QMap<int,QMap<int,int> > tilesNop;
        inPointsClass>>tilesNop;
        inPointsClass>>existsFields;
        inPointsClass>>tilesPointsClass;
        inPointsClass>>tilesPointsClassNewByPos;
        pointsClassFile.close();
        QMap<int,QMap<int,QVector<int> > > pointsIndexByTiles=iterFiles.value();
        QMap<int,QMap<int,QVector<int> > >::const_iterator iterTileX=pointsIndexByTiles.begin();
        bool existsChanges=false;
        while(iterTileX!=pointsIndexByTiles.end())
        {
            int tileX=iterTileX.key();
            QMap<int,QVector<int> >::const_iterator iterTileY=iterTileX.value().begin();
            while(iterTileY!=iterTileX.value().end())
            {
                int tileY=iterTileY.key();
                QVector<int> pointsIndex=iterTileY.value();
                for(int npi=0;npi<pointsIndex.size();npi++)
                {
                    int pointIndex=pointsIndex[npi];
//                    if(!pointClassByTile.contains(tileX)
//                            ||!pointPositionByTile.contains(tileX)
//                            ||!pointClassNewByTile.contains(tileX))
                    if(!pointPositionByTile.contains(tileX)
                            ||!tilesPointsClass.contains(tileX))
                    {
                        strError=QObject::tr("PointCloudFile::updatePoints");
                        strError+=QObject::tr("\nFor file index: %1, not exists tile X: %2 tile Y: %3 point index: %4")
                                .arg(QString::number(fileIndex)).arg(QString::number(tileX))
                                .arg(QString::number(tileY).arg(QString::number(pointIndex)));
                        if(ptrWidget!=NULL)
                        {
                            ptrProgress->setValue(step);
                            qApp->processEvents();
                        }
                        return(false);
                    }
//                    if(!pointClassByTile[tileX].contains(tileY)
//                            ||!pointPositionByTile[tileX].contains(tileY)
//                            ||!pointClassNewByTile[tileX].contains(tileY))
                    if(!pointPositionByTile[tileX].contains(tileY)
                            ||!tilesPointsClass[tileX].contains(tileY))
                    {
                        strError=QObject::tr("PointCloudFile::updatePoints");
                        strError+=QObject::tr("\nFor file index: %1, not exists tile X: %2 tile Y: %3 point index: %4")
                                .arg(QString::number(fileIndex)).arg(QString::number(tileX))
                                .arg(QString::number(tileY).arg(QString::number(pointIndex)));
                        if(ptrWidget!=NULL)
                        {
                            ptrProgress->setValue(step);
                            qApp->processEvents();
                        }
                        return(false);
                    }
//                    if(pointClassByTile[tileX][tileY].size()<(pointIndex+1)
//                            ||!pointPositionByTile[tileX][tileY].size()<(pointIndex+1)
//                            ||!pointClassNewByTile[tileX][tileY].size()<(pointIndex+1))
                    if(pointIndex>(pointPositionByTile[tileX][tileY].size()-1))
                    {
                        strError=QObject::tr("PointCloudFile::updatePoints");
                        strError+=QObject::tr("\nFor file index: %1, not exists tile X: %2 tile Y: %3 point index: %4")
                                .arg(QString::number(fileIndex)).arg(QString::number(tileX))
                                .arg(QString::number(tileY).arg(QString::number(pointIndex)));
                        if(ptrWidget!=NULL)
                        {
                            ptrProgress->setValue(step);
                            qApp->processEvents();
                        }
                        return(false);
                    }
                    int pointPositionInTile=pointPositionByTile[tileX][tileY][pointIndex];
                    if(pointPositionInTile>(tilesPointsClass[tileX][tileY].size()-1))
                    {
                        strError=QObject::tr("PointCloudFile::updatePoints");
                        strError+=QObject::tr("\nFor file index: %1, not exists tile X: %2 tile Y: %3 point index: %4")
                                .arg(QString::number(fileIndex)).arg(QString::number(tileX))
                                .arg(QString::number(tileY).arg(QString::number(pointIndex)));
                        if(ptrWidget!=NULL)
                        {
                            ptrProgress->setValue(step);
                            qApp->processEvents();
                        }
                        return(false);
                    }
//                    quint8 pointClass=pointClassByTile[tileX][tileY][pointIndex];
                    quint8 pointClass=tilesPointsClass[tileX][tileY][pointPositionInTile];
                    quint8 pointClassNew=pointClass;
                    if(tilesPointsClassNewByPos.contains(tileX))
                    {
                        if(tilesPointsClassNewByPos[tileX].contains(tileY))
                        {
                            if(tilesPointsClassNewByPos[tileX][tileY].contains(pointPositionInTile))
                            {
                                pointClassNew=tilesPointsClassNewByPos[tileX][tileY][pointPositionInTile];
                            }
                        }
                    }
                    if(lockedClasses.contains(pointClassNew))
                    {
                        if(lockedClasses[pointClassNew]) continue;
                    }
//                    quint8 pointClassNew=pointClassNewByTile[tileX][tileY][pointIndex];
                    if(strAction.compare(POINTCLOUDFILE_ACTION_CHANGE_CLASS,Qt::CaseInsensitive)==0)
                    {
                        if(pointClassNew==POINTCLOUDFILE_CLASS_NUMBER_REMOVE) continue;
                        if(pointClassNew!=classValue)
                        {
                            tilesPointsClassNewByPos[tileX][tileY][pointPositionInTile]=classValue;
                            if(!existsChanges) existsChanges=true;
                        }
                    }
                    else if(strAction.compare(POINTCLOUDFILE_ACTION_RECOVER_ORIGINAL_CLASS,Qt::CaseInsensitive)==0)
                    {
                        if(pointClassNew==POINTCLOUDFILE_CLASS_NUMBER_REMOVE) continue;
                        if(classValue!=POINTCLOUDFILE_ACTION_ALL_CLASSES_VALUE)
                        {
                            if(classValue!=pointClassNew) continue;
                        }
                        if(pointClassNew!=pointClass)
                        {
                            tilesPointsClassNewByPos[tileX][tileY][pointPositionInTile]=pointClass;
                            if(!existsChanges) existsChanges=true;
                        }
                    }
                    else if(strAction.compare(POINTCLOUDFILE_ACTION_DELETE,Qt::CaseInsensitive)==0)
                    {
                        if(pointClassNew==POINTCLOUDFILE_CLASS_NUMBER_REMOVE) continue;
                        if(classValue!=POINTCLOUDFILE_ACTION_ALL_CLASSES_VALUE)
                        {
                            if(classValue!=pointClassNew) continue;
                        }
                        tilesPointsClassNewByPos[tileX][tileY][pointPositionInTile]=POINTCLOUDFILE_CLASS_NUMBER_REMOVE;
                        if(!existsChanges) existsChanges=true;
                    }
                    else if(strAction.compare(POINTCLOUDFILE_ACTION_RECOVER_DELETED,Qt::CaseInsensitive)==0)
                    {
                        if(pointClassNew!=POINTCLOUDFILE_CLASS_NUMBER_REMOVE) continue;
                        if(classValue!=POINTCLOUDFILE_ACTION_ALL_CLASSES_VALUE)
                        {
                            if(classValue!=pointClass) continue;
                        }
                        tilesPointsClassNewByPos[tileX][tileY][pointPositionInTile]=pointClass;
                        if(!existsChanges) existsChanges=true;
                    }
                }
                iterTileY++;
            }
            iterTileX++;
        }
        if(existsChanges)
        {
            pointsClassFile.open(QIODevice::WriteOnly);
            QDataStream outPointsClass(&pointsClassFile);   // we will serialize the data into the file
            outPointsClass<<tilesNop;
            outPointsClass<<existsFields;
            outPointsClass<<tilesPointsClass;
            outPointsClass<<tilesPointsClassNewByPos;
            pointsClassFile.close();
        }
        iterFiles++;
    }
    if(ptrWidget!=NULL)
    {
        ptrProgress->close();
        delete(ptrProgress);
    }
    return(true);
}

bool PointCloudFile::writePointCloudFiles(QString suffix,
                                          QString outputPath,
                                          QString &strError)
{
    if(suffix.isEmpty()&&outputPath.isEmpty())
    {
        strError=QObject::tr("PointCloudFile::writePointCloudFiles");
        strError+=QObject::tr("\nSuffix and output path are empty");
        return(false);
    }
    QDir currentDir=QDir::currentPath();
    if(!outputPath.isEmpty())
    {
        if(!currentDir.exists(outputPath))
        {
            strError=QObject::tr("PointCloudFile::writePointCloudFiles");
            strError+=QObject::tr("\nNot exists output path:\n").arg(outputPath);
            return(false);
        }
    }
    QWidget* ptrWidget=new QWidget();
    QString strAuxError;
    int numberOfSqlsInTransaction=0;
    numberOfSqlsInTransaction=0;
    QProgressDialog* ptrProgress=NULL;
    int filesByStep=POINTCLOUDFILE_NUMBER_OF_FILES_TO_WRITE_PROCESS_BY_STEP;
    int numberOfFiles=mFilesIndex.size();
    int numberOfSteps=ceil(numberOfFiles/filesByStep);
    if(ptrWidget!=NULL)
    {
        QString title=QObject::tr("Writting files for point cloud: ");
        QString msgGlobal=mPath;
        msgGlobal+="\n";
        msgGlobal+=QString::number(numberOfFiles,10);
        msgGlobal+=" number of files";
        ptrProgress=new QProgressDialog(title, "Abort",0,numberOfSteps, ptrWidget);
        ptrProgress->setWindowModality(Qt::WindowModal);
        ptrProgress->setLabelText(msgGlobal);
        ptrProgress->show();
        qApp->processEvents();
    }
    int step=0;
    int numberOfProcessedFiless=0;
    int numberOfProcessedFilesInStep=0;
    QMap<QString,int>::const_iterator iterFiles=mFilesIndex.begin();
    while(iterFiles!=mFilesIndex.end())
    {
        int fileIndex=iterFiles.value();
        QString inputPointCloudFileName=iterFiles.key();
        if(!QFile::exists(inputPointCloudFileName))
        {
            strError=QObject::tr("PointCloudFile::writePointCloudFiles");
            strError+=QObject::tr("\nNot exists point cloud file:\n").arg(inputPointCloudFileName);
            if(ptrWidget!=NULL)
            {
                ptrProgress->setValue(numberOfSteps);
                qApp->processEvents();
                ptrProgress->close();
                delete(ptrProgress);
            }
            return(false);
        }
        QFileInfo inputPointCloudFileInfo(inputPointCloudFileName);
        QString outputPointCloudFileName;
        if(!outputPath.isEmpty())
        {
            outputPointCloudFileName=outputPath;
        }
        else
        {
            outputPointCloudFileName=inputPointCloudFileInfo.absolutePath();
        }
        outputPointCloudFileName+="/";
        if(suffix.isEmpty())
        {
            outputPointCloudFileName+=inputPointCloudFileInfo.fileName();
        }
        else
        {
            outputPointCloudFileName+=inputPointCloudFileInfo.baseName();
            outputPointCloudFileName+=suffix;
            outputPointCloudFileName+=".";
            outputPointCloudFileName+=inputPointCloudFileInfo.completeSuffix();
        }
        if(QFile::exists(outputPointCloudFileName))
        {
            if(!QFile::remove(outputPointCloudFileName))
            {
                strError=QObject::tr("PointCloudFile::writePointCloudFiles");
                strError+=QObject::tr("Error removing existing output file:\n%1").arg(outputPointCloudFileName);
                if(ptrWidget!=NULL)
                {
                    ptrProgress->setValue(numberOfSteps);
                    qApp->processEvents();
                    ptrProgress->close();
                    delete(ptrProgress);
                }
                return(false);
            }
        }
        numberOfProcessedFilesInStep++;
        numberOfProcessedFiless++;
        if(numberOfProcessedFilesInStep==POINTCLOUDFILE_NUMBER_OF_FILES_TO_WRITE_PROCESS_BY_STEP)
        {
            step++;
            if(ptrWidget!=NULL)
            {
                ptrProgress->setValue(step);
                qApp->processEvents();
            }
            numberOfProcessedFilesInStep=0;
        }
        QString classesFileName=mClassesFileByIndex[fileIndex];
        QFile pointsClassFile(classesFileName);
        if (!pointsClassFile.open(QIODevice::ReadOnly))
        {
            strError=QObject::tr("PointCloudFile::writePointCloudFiles");
            strError+=QObject::tr("\nError opening file:\n%1").arg(classesFileName);
            if(ptrWidget!=NULL)
            {
                ptrProgress->setValue(numberOfSteps);
                qApp->processEvents();
                ptrProgress->close();
                delete(ptrProgress);
            }
            return(false);
        }
        QDataStream inPointsClass(&pointsClassFile);
        QMap<QString,bool> existsFields;
        QMap<int,QMap<int,QVector<quint8> > > tilesPointsClass;
        QMap<int,QMap<int,QMap<int,quint8> > > tilesPointsClassNewByPos; // se guarda vacío
        QMap<int,QMap<int,int> > tilesNop;
        inPointsClass>>tilesNop;
        inPointsClass>>existsFields;
        inPointsClass>>tilesPointsClass;
        inPointsClass>>tilesPointsClassNewByPos;
        pointsClassFile.close();
        bool existsColor=existsFields[POINTCLOUDFILE_PARAMETER_COLOR];
        bool existsGpsTime=existsFields[POINTCLOUDFILE_PARAMETER_GPS_TIME];
        bool existsUserData=existsFields[POINTCLOUDFILE_PARAMETER_USER_DATA];
        bool existsIntensity=existsFields[POINTCLOUDFILE_PARAMETER_INTENSITY];
        bool existsSourceId=existsFields[POINTCLOUDFILE_PARAMETER_SOURCE_ID];
        bool existsNir=existsFields[POINTCLOUDFILE_PARAMETER_NIR];
        bool existsReturn=existsFields[POINTCLOUDFILE_PARAMETER_RETURN];
        bool existsReturns=existsFields[POINTCLOUDFILE_PARAMETER_RETURNS];
        bool existsChanges=false;
        QMap<int,QMap<int,QMap<int,quint8> > > pointsClassNewByPosInTileByTile;
        QMap<int,QMap<int,QMap<int,quint8> > >::const_iterator iterTileXPointsClassNew=tilesPointsClassNewByPos.begin();
        while(iterTileXPointsClassNew!=tilesPointsClassNewByPos.end())
        {
            int tileX=iterTileXPointsClassNew.key();
            QMap<int,QMap<int,quint8> >::const_iterator iterTileYPointsClassNew=iterTileXPointsClassNew.value().begin();
            while(iterTileYPointsClassNew!=iterTileXPointsClassNew.value().end())
            {
                int tileY=iterTileYPointsClassNew.key();
                QMap<int,quint8>::const_iterator iterPositionPointsClassNew=iterTileYPointsClassNew.value().begin();
                while(iterPositionPointsClassNew!=iterTileYPointsClassNew.value().end())
                {
                    int posInTile=iterPositionPointsClassNew.key();
                    if(!tilesPointsClass.contains(tileX))
                    {
                        iterPositionPointsClassNew++;
                        continue;
                    }
                    if(!tilesPointsClass[tileX].contains(tileY))
                    {
                        iterPositionPointsClassNew++;
                        continue;
                    }
                    if(posInTile>(tilesPointsClass[tileX][tileY].size()-1))
                    {
                        iterPositionPointsClassNew++;
                        continue;
                    }
                    quint8 classOriginal=tilesPointsClass[tileX][tileY][posInTile];
                    quint8 classNew=iterPositionPointsClassNew.value();
                    if(classNew!=classOriginal)
                    {
                        pointsClassNewByPosInTileByTile[tileX][tileY][posInTile]=classNew;
                        if(!existsChanges) existsChanges=true;
                    }
                    iterPositionPointsClassNew++;
                }
                iterTileYPointsClassNew++;
            }
            iterTileXPointsClassNew++;
        }
        if(!existsChanges)
        {
            if(!outputPath.isEmpty())
            {
                if(!QFile::copy(inputPointCloudFileName,
                                outputPointCloudFileName))
                {
                    strError=QObject::tr("PointCloudFile::writePointCloudFiles");
                    strError+=QObject::tr("Error copying witout changes input file:\n%1").arg(inputPointCloudFileName);
                    strError+=QObject::tr("\nto output file:\n%1").arg(outputPointCloudFileName);
                    if(ptrWidget!=NULL)
                    {
                        ptrProgress->setValue(numberOfSteps);
                        qApp->processEvents();
                        ptrProgress->close();
                        delete(ptrProgress);
                    }
                    return(false);
                }
            }
            iterFiles++;
            continue;
        }
        QString zipFileNamePoints=mZipFilePointsByIndex[fileIndex];
        QString zipFilePointsPath=mZipFilePathPointsByIndex[fileIndex];
        QuaZip zipFilePoints(zipFileNamePoints);
        if(!zipFilePoints.open(QuaZip::mdUnzip))
        {
            strError=QObject::tr("PointCloudFile::writePointCloudFiles");
            strError+=QObject::tr("\nError opening file:\n%1\nError:\n%2")
                    .arg(zipFileNamePoints).arg(QString::number(zipFilePoints.getZipError()));
            if(ptrWidget!=NULL)
            {
                ptrProgress->close();
                delete(ptrProgress);
            }
            return(false);
        }
        QMap<int,QMap<int,QMap<int,QMap<int,QVector<quint8> > > > > pointsClassesNewByCoorInTileByTile;
        QMap<int,QMap<int,QMap<int,QMap<int,QVector<double> > > > > pointsAltitudesByCoorInTileByTile;
        QMap<int,QMap<int,QMap<int,quint8> > >::const_iterator iterTileX2=pointsClassNewByPosInTileByTile.begin();
        while(iterTileX2!=pointsClassNewByPosInTileByTile.end())
        {
            int tileX=iterTileX2.key();
            QMap<int,QMap<int,quint8> >::const_iterator iterTileY2=iterTileX2.value().begin();
            while(iterTileY2!=iterTileX2.value().end())
            {
                int tileY=iterTileY2.key();
                QMap<int,quint8> pointsClassNewByPosInTile=iterTileY2.value();
                QString tileTableName=mTilesName[tileX][tileY];
                if(!zipFilePoints.setCurrentFile(tileTableName))
                {
                    strError=QObject::tr("PointCloudFile::writePointCloudFiles");
                    strError+=QObject::tr("\nNot exists: %1 in file:\n%2\nError:\n%3")
                            .arg(tileTableName).arg(zipFileNamePoints)
                            .arg(QString::number(zipFilePoints.getZipError()));
                    if(ptrWidget!=NULL)
                    {
                        ptrProgress->close();
                        delete(ptrProgress);
                    }
                    return(false);
                }
                QuaZipFile inPointsFile(&zipFilePoints);
                if (!inPointsFile.open(QIODevice::ReadOnly))
                {
                    strError=QObject::tr("PointCloudFile::writePointCloudFiles");
                    strError+=QObject::tr("\nError opening: %1 in file:\n%2\nError:\n%3")
                            .arg(tileTableName).arg(zipFileNamePoints)
                            .arg(QString::number(zipFilePoints.getZipError()));
                    if(ptrWidget!=NULL)
                    {
                        ptrProgress->close();
                        delete(ptrProgress);
                    }
                    return(false);
                }
                QDataStream inPoints(&inPointsFile);
                int pos=0;
                while(!inPoints.atEnd())
                {
                    quint16 ix,iy;
                    quint8 z_pc,z_pa,z_pb;
                    inPoints>>ix>>iy>>z_pa>>z_pb>>z_pc;
                    PCFile::Point pto;
                    pto.setPositionInTile(pos);
                    if(pos>tilesPointsClass[tileX][tileY].size())
                    {
                        strError=QObject::tr("PointCloudFile::writePointCloudFiles");
                        strError+=QObject::tr("\nNot exists position: %1 in tile X: %2 tile Y: %3 in classes  file:\n%4")
                                .arg(QString::number(pos)).arg(QString::number(tileX))
                                .arg(QString::number(tileY)).arg(classesFileName);
                        if(ptrWidget!=NULL)
                        {
                            ptrProgress->close();
                            delete(ptrProgress);
                        }
                        return(false);
                    }
                    quint8 ptoClass=tilesPointsClass[tileX][tileY][pos];
                    pto.setClass(ptoClass);
                    quint8 ptoClassNew=ptoClass;
                    if(tilesPointsClassNewByPos.contains(tileX))
                    {
                        if(tilesPointsClassNewByPos[tileX].contains(tileY))
                        {
                            if(tilesPointsClassNewByPos[tileX][tileY].contains(pos))
                            {
                                ptoClassNew=tilesPointsClassNewByPos[tileX][tileY][pos];
                            }
                        }
                    }
                    pto.setClassNew(ptoClassNew);
                    pto.setCoordinates(ix,iy,z_pa,z_pb,z_pc);
                    if(existsColor)
                    {
                        if(mNumberOfColorBytes==1)
                        {
                            quint8 color_r,color_g,color_b;
                            inPoints>>color_r>>color_g>>color_b;
                            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_RED,color_r);
                            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_GREEN,color_g);
                            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_BLUE,color_b);
                        }
                        else
                        {
                            quint16 color_r,color_g,color_b;
                            inPoints>>color_r>>color_g>>color_b;
                            pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_RED,color_r);
                            pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_GREEN,color_g);
                            pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_COLOR_BLUE,color_b);
                        }
                    }
                    if(existsGpsTime)
                    {
                        quint8 gpsDowHourPackit;
                        quint8 msb1,msb2,msb3;
                        inPoints>>gpsDowHourPackit>>msb1>>msb2>>msb3;
                        pto.setGpsTime(gpsDowHourPackit,msb1,msb2,msb3);
                    }
                    if(existsUserData)
                    {
                        quint8 userData;
                        inPoints>>userData;
                        pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_USER_DATA,userData);
                    }
                    if(existsIntensity)
                    {
                        quint16 intensity;
                        inPoints>>intensity;
                        pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_INTENSITY,intensity);
                    }
                    if(existsSourceId)
                    {
                        quint16 sourceId;
                        inPoints>>sourceId;
                        pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_SOURCE_ID,sourceId);
                    }
                    if(existsNir)
                    {
                        if(mNumberOfColorBytes==1)
                        {
                            quint8 nir;
                            inPoints>>nir;
                            pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_NIR,nir);
                        }
                        else
                        {
                            quint16 nir;
                            inPoints>>nir;
                            pto.set16BitsValue(POINTCLOUDFILE_PARAMETER_NIR,nir);
                        }
                    }
                    if(existsReturn)
                    {
                        quint8 returnNumber;
                        inPoints>>returnNumber;
                        pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_RETURN,returnNumber);
                    }
                    if(existsReturns)
                    {
                        quint8 numberOfReturns;
                        inPoints>>numberOfReturns;
                        pto.set8BitsValue(POINTCLOUDFILE_PARAMETER_RETURNS,numberOfReturns);
                    }
                    if(pointsClassNewByPosInTile.contains(pos))
                    {
                        quint8 classNew=pointsClassNewByPosInTile[pos];
                        bool isNew=false;
                        quint16 ix=pto.getIx();
                        quint16 iy=pto.getIy();
                        double z=pto.getZ();
                        if(!pointsClassesNewByCoorInTileByTile.contains(tileX)) isNew=true;
                        else if(!pointsClassesNewByCoorInTileByTile[tileX].contains(tileY)) isNew=true;
                        else if(!pointsClassesNewByCoorInTileByTile[tileX].contains(tileY)) isNew=true;
                        else if(!pointsClassesNewByCoorInTileByTile[tileX][tileY].contains(ix)) isNew=true;
                        else if(!pointsClassesNewByCoorInTileByTile[tileX][tileY][ix].contains(iy)) isNew=true;
                        if(isNew)
                        {
                            QVector<quint8> aux1;
                            pointsClassesNewByCoorInTileByTile[tileX][tileY][ix][iy]=aux1;
                            QVector<double> aux2;
                            pointsAltitudesByCoorInTileByTile[tileX][tileY][ix][iy]=aux2;
                        }
                        pointsClassesNewByCoorInTileByTile[tileX][tileY][ix][iy].push_back(classNew);
                        pointsAltitudesByCoorInTileByTile[tileX][tileY][ix][iy].push_back(z);
                    }
                    pos++;
                }
                inPointsFile.close();
                iterTileY2++;
            }
            iterTileX2++;
        }

        std::string stdInputFileName=inputPointCloudFileName.toStdString();
        const char* charInputFileName=stdInputFileName.c_str();

        std::string stdOutputFileName=outputPointCloudFileName.toStdString();
        const char* charOutputFileName=stdOutputFileName.c_str();

        int numberOfPoints;
        LASreadOpener lasreadopener;
        lasreadopener.set_file_name(charInputFileName);
        if (!lasreadopener.active())
        {
            strError=QObject::tr("PointCloudSpatialiteDb::writePointCloudFiles");
            strError+=QObject::tr("Error opening file:\n%1").arg(inputPointCloudFileName);
            if(ptrWidget!=NULL)
            {
                ptrProgress->setValue(numberOfSteps);
                qApp->processEvents();
                ptrProgress->close();
                delete(ptrProgress);
            }
            return(false);
        }
        LASreader* lasreader = lasreadopener.open();
        LASheader* lasheader = &lasreader->header;
        LASwriteOpener laswriteopener;
        laswriteopener.set_file_name(charOutputFileName);
        LASheader outputLasheader=lasreader->header;
        LASwriter* laswriter = laswriteopener.open(&outputLasheader);
        numberOfPoints=lasreader->npoints;
        QProgressDialog* ptrWritePointCloudFileProgress=NULL;
        int pointsStep=POINTCLOUDFILE_NUMBER_OF_POINTS_TO_WRITE_PROCESS_BY_STEP;
        int wPclNumberOfSteps=ceil((double)numberOfPoints/(double)pointsStep);
        if(ptrWidget!=NULL)
        {
            QString title=QObject::tr("Point Cloud File write operation");
            QString msgGlobal=" ... writting points ";
            msgGlobal+=QString::number(numberOfPoints,10);
            msgGlobal+=" points";
            ptrWritePointCloudFileProgress=new QProgressDialog(title, "Abort",0,wPclNumberOfSteps, ptrWidget);
            ptrWritePointCloudFileProgress->setWindowModality(Qt::WindowModal);
            ptrWritePointCloudFileProgress->setLabelText(msgGlobal);
            ptrWritePointCloudFileProgress->show();
            qApp->processEvents();
        }
        int wPclStep=0;
        int numberOfProcessedPoints=0;
        int numberOfProcessedPointsInStep=0;
        int numberOfPointsToProcessInStep=numberOfPoints;
        if(POINTCLOUDFILE_NUMBER_OF_POINTS_TO_WRITE_PROCESS_BY_STEP<numberOfPointsToProcessInStep)
        {
            numberOfPointsToProcessInStep=POINTCLOUDFILE_NUMBER_OF_POINTS_TO_WRITE_PROCESS_BY_STEP;
        }
        int pointIndex=-1;
        while(lasreader->read_point()&&numberOfPointsToProcessInStep>0)
        {
            pointIndex++;
            bool writePoint=true;
            double x=lasreader->point.get_X()*lasheader->x_scale_factor+lasheader->x_offset;
            double y=lasreader->point.get_Y()*lasheader->y_scale_factor+lasheader->y_offset;
            int tileX=qRound(floor(floor(x)/mGridSize)*mGridSize);
            int tileY=qRound(floor(floor(y)/mGridSize)*mGridSize);
            if(pointsClassesNewByCoorInTileByTile.contains(tileX))
            {
                if(pointsClassesNewByCoorInTileByTile[tileX].contains(tileY))
                {
                    quint16 ix=qRound((x-tileX)*1000.);
                    if(pointsClassesNewByCoorInTileByTile[tileX][tileY].contains(ix))
                    {
                        quint16 iy=qRound((y-tileY)*1000.);
                        if(pointsClassesNewByCoorInTileByTile[tileX][tileY][ix].contains(iy))
                        {
                            double z=lasreader->point.get_Z()*lasheader->z_scale_factor+lasheader->z_offset;
                            QVector<double> pointsClassesNewZ=pointsAltitudesByCoorInTileByTile[tileX][tileY][ix][iy];
                            for(int npz=0;npz<pointsClassesNewZ.size();npz++)
                            {
                                double pointClassNewZ=pointsClassesNewZ[npz];
                                // estoy suponiendo que no pueden existir dos puntos a menos de 1 mm en 3D
                                if(fabs(z-pointClassNewZ)<=0.001)
                                {
                                    int classNew=pointsClassesNewByCoorInTileByTile[tileX][tileY][ix][iy][npz];
                                    if(classNew==POINTCLOUDFILE_CLASS_NUMBER_REMOVE)
                                    {
                                        writePoint=false;
                                    }
                                    else
                                    {
                                        lasreader->point.set_classification(classNew);
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            if(writePoint)
            {
                laswriter->write_point(&lasreader->point);
                laswriter->update_inventory(&lasreader->point);
            }
            numberOfProcessedPoints++;
            numberOfProcessedPointsInStep++;
            if(numberOfProcessedPointsInStep==numberOfPointsToProcessInStep)
            {
                wPclStep++;
                numberOfPointsToProcessInStep=numberOfPoints-numberOfProcessedPoints;
                if(POINTCLOUDFILE_NUMBER_OF_POINTS_TO_WRITE_PROCESS_BY_STEP<numberOfPointsToProcessInStep)
                {
                    numberOfPointsToProcessInStep=POINTCLOUDFILE_NUMBER_OF_POINTS_TO_WRITE_PROCESS_BY_STEP;
                }
                numberOfProcessedPointsInStep=0;
                if(ptrWidget!=NULL)
                {
                    ptrWritePointCloudFileProgress->setValue(wPclStep);
                    qApp->processEvents();
                }
            }
        }
        laswriter->update_header(&outputLasheader, TRUE);
        laswriter->close();
        lasreader->close();
//            delete lasreader;
        if(ptrWidget!=NULL)
        {
            ptrWritePointCloudFileProgress->setValue(wPclNumberOfSteps);
            qApp->processEvents();
            ptrWritePointCloudFileProgress->close();
            delete(ptrWritePointCloudFileProgress);
        }
        iterFiles++;
    }
    if(ptrWidget!=NULL)
    {
        ptrProgress->setValue(numberOfSteps);
        qApp->processEvents();
        ptrProgress->close();
        delete(ptrProgress);
    }
    return(true);
}
