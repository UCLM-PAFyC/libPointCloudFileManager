#include "PointCloudFileDefinitions.h"
#include "Point.h"

using namespace PCFile;


double Point::getGpsTime()
{
    quint8 h = ((mGpsDowHourPackit >> 0) & 0x1F);
    quint8 dow = ((mGpsDowHourPackit >> 3) & 0x07);
    int ms=mGpsMsb1*256*256*256+mGpsMsb2*256*256+mGpsMsb3*256;
    double gpsTime=h*60.*60.+dow*24.*60.*60.+ms/pow(10.,6.);
    return(gpsTime);
}

double Point::getZ()
{
    double zc=(mZpa*256.0+mZpb)/10.+mZpc/1000.+POINTCLOUDFILE_HEIGHT_MINIMUM_VALID_VALUE;
    return(zc);
}

void Point::setCoordinates(quint16 fc, quint16 sc, quint8 zpa, quint8 zpb, quint8 zpc)
{
    mFc=fc;mSc=sc;mZpa=zpa;mZpb=zpb;mZpc=zpc;
//    double z=getZ();
//    if(z<0)
//    {
//        int yo=1;
//        yo++;
//    }
}
