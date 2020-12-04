#ifndef POINT_H
#define POINT_H


#include "libPointCloudFileManager_global.h"

#include <QString>
#include <QMap>

namespace PCFile{

class LIBPOINTCLOUDFILEMANAGERSHARED_EXPORT Point
{
public:
    Point(){mClass=0;mClassNew=0;mFc=0;mSc=0;mZpa=0;mZpb=0;mZpc=0;
           mGpsMsb1=0;mGpsMsb2=0;mGpsMsb3=0;};
    quint8 getClass(){return(mClass);};
    quint8 getClassNew(){return(mClassNew);};
    quint16 getIx(){return(mFc);};
    quint16 getIy(){return(mSc);};
    double getGpsTime();
    int getPositionInTile(){return(mPositionInTile);};
    double getZ();
    void get8BitsValues(QMap<QString,quint8>& values){values=m8BitsValues;};
    void get16BitsValues(QMap<QString,quint16>& values){values=m16BitsValues;};
    void setCoordinates(quint16 fc,quint16 sc,quint8 zpa,quint8 zpb,quint8 zpc);
    void setClass(quint8 value){mClass=value;};
    void setClassNew(quint8 value){mClassNew=value;};
    void setGpsTime(quint8 gpsDowHourPackit,
                    quint8 gpsMsb1,
                    quint8 gpsMsb2,
                    quint8 gpsMsb3){
        mGpsDowHourPackit=gpsDowHourPackit;mGpsMsb1=gpsMsb1;
        mGpsMsb2=gpsMsb2;mGpsMsb3=gpsMsb3;};
    void set16BitsValue(QString tag,quint16 value){m16BitsValues[tag]=value;};
    void set8BitsValue(QString tag,quint8 value){m8BitsValues[tag]=value;};
    void setPositionInTile(int positionInTile){mPositionInTile=positionInTile;};
    void setRealValue(QString tag,qreal value){mRealValues[tag]=value;};
//    ~Point();
private:
    int mPositionInTile;
    quint16 mFc;
    quint16 mSc;
    quint8 mZpa,mZpb,mZpc;
    quint8 mGpsDowHourPackit,mGpsMsb1,mGpsMsb2,mGpsMsb3;
    // double zc=(z_pa*256.0+z_pb)/10.+z_pc/1000.+POINTCLOUDFILE_HEIGHT_MINIMUM_VALID_VALUE;
    quint8 mClass,mClassNew;
    QMap<QString,quint16> m16BitsValues;
    QMap<QString,quint8> m8BitsValues;
    QMap<QString,qreal> mRealValues;
};
}
#endif // POINT_H
