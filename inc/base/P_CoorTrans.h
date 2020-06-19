/**
 *   P_CoorTrans.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PCOORTRANS_H_H_
#define __PCOORTRANS_H_H_
#include "P_Types.h"

namespace Position
{
#define D2R(X) (X) * 0.01745329251994329547437168059786927187815308570862
#define R2D(X) (X) / 0.01745329251994329547437168059786927187815308570862

const double PI64 = 3.1415926535897932384626433832795028841971693993751; //Pi

struct Datum
{
    double r_max;
    double r_min;
    double e2;
};
extern Datum WGS84Datum;

//! 坐标转换类(静态类)
class PCoorTrans
{
public:

    //! 围绕x轴旋转
    static inline Mat RotateByX(double degree)
    {
        double sin = sinf(D2R(degree));
        double cos = cosf(D2R(degree));

        return (Mat_<MATTYPE>(3,3) << 1, 0   , 0   ,
                                      0, cos ,-sin ,
                                      0, sin , cos );
    }
    //! 围绕y轴旋转
    static inline Mat RotateByY(double degree)
    {
        double sin = sinf(D2R(degree));
        double cos = cosf(D2R(degree));

        return (Mat_<MATTYPE>(3,3) << cos , 0 , sin ,
                                      0   , 1 , 0   ,
                                      -sin, 0 , cos );
    }
    //! 围绕z轴旋转
    static inline Mat RotateByZ(double degree)
    {
        double sin = sinf(D2R(degree));
        double cos = cosf(D2R(degree));

        return (Mat_<MATTYPE>(3,3) << cos, -sin, 0 ,
                                      sin, cos , 0 ,
                                      0  , 0   , 1 );
    }

    //! 大地坐标->地心地固空间直角坐标系
    static XYZCoordinate BLH_to_XYZ(const BLHCoordinate &blh, Datum dt = WGS84Datum)
    {
#if 0
        double a = dt.r_max;
        double e = dt.e2;
        double r_lat = D2R(blh.lat);
        double r_lon = D2R(blh.lon);
        double slat  = sin(r_lat);
        double clat  = cos(r_lat);
        double N = a / sqrt(1 - e * slat * slat);
        double WGS84_X = (N + blh.alt) * clat * cos(r_lon);
        double WGS84_Y = (N + blh.alt) * clat * sin(r_lon);
        double WGS84_Z = (N * (1 - e) + blh.alt) * slat;
#else
        double clat = cos(D2R(blh.lat));
        double slat = sin(D2R(blh.lat));
        double clon = cos(D2R(blh.lon));
        double slon = sin(D2R(blh.lon));

        double a2   = dt.r_max * dt.r_max;
        double b2   = dt.r_min * dt.r_min;

        double L    = 1.0/sqrt(a2 * clat * clat + b2 * slat * slat);

        double WGS84_X = (a2 * L + blh.alt) * clat * clon;
        double WGS84_Y = (a2 * L + blh.alt) * clat * slon;
        double WGS84_Z = (b2 * L + blh.alt) * slat;
#endif
        return {WGS84_X, WGS84_Y, WGS84_Z};
    }
    //! 地心地固坐标系->大地坐标系
    static BLHCoordinate XYZ_to_BLH(const XYZCoordinate &pt, Datum dt = WGS84Datum)
    {
#if 0
        double f, f1, f2;
        double p, zw, nnq;
        double bb, l, h;

        double a = dt.r_max;
        double eq = dt.e2;
        f = PI64 * 50 / 180;
        double x, y, z;
        x = pt.x;
        y = pt.y;
        z = pt.z;

        double ddp = x * x + y * y;

        p = z / sqrt(ddp);
        do
        {
            zw = a / sqrt(1 - eq * sin(f) * sin(f));
            nnq = 1 - eq * zw / (sqrt(ddp) / cos(f));
            f1 = atan(p / nnq);
            f2 = f;
            f = f1;
        } while (!(abs(f2 - f1) < 10E-10));
        bb = R2D(f);
        l = R2D(atan(y / x));
        if (l < 0)
            l += 180.0;
        h = sqrt(ddp) / cos(f1) - a / sqrt(1 - eq * sin(f1) * sin(f1));
#else
        double a2 = dt.r_max * dt.r_max;
        double b = sqrt( a2 * (1 - dt.e2));
        double ep = sqrt((a2 - b * b) /(b * b));
        double p = hypot(pt.x,pt.y);
        double th = atan2(dt.r_max * pt.z,b * p);
        double l = atan2(pt.y,pt.x);
        double bb = atan2((pt.z + ep * ep * b * pow(sin(th),3)), (p - dt.e2 * dt.r_max * pow(cos(th),3)));
        double N = dt.r_max / sqrt(1 - dt.e2 * sin(bb) * sin(bb));
        double h     = p / cos(bb) - N;
        bb = R2D(bb);
        l = R2D(l);
#endif
        return {bb, l, h};
    }

    //! 计算 xyz坐标系转enu坐标系 旋转矩阵
    static cv::Mat XYZ_to_ENU( double B,  double L)
    {
        double r_L = D2R(L);
        double r_B = D2R(B);

        cv::Mat XYZ2ENU = (cv::Mat_<double>(3, 3) << -sin(r_L), cos(r_L), 0,
                           -sin(r_B) * cos(r_L), -sin(r_L) * sin(r_B), cos(r_B),
                           cos(r_L) * cos(r_B), cos(r_B) * sin(r_L), sin(r_B));
        return XYZ2ENU;
    }

    //! 计算 imu坐标系 转enu坐标系
    static cv::Mat IMU_to_ENU( double yaw,  double pitch,  double roll)
    {
        //绕y轴
        cv::Mat rollR = RotateByY(roll);
        //绕x轴
        cv::Mat pitchR = RotateByX(pitch);
        //绕z轴
        cv::Mat yawR =  RotateByZ(yaw);
        return yawR * pitchR * rollR;
    }
    
    //! 计算enu坐标系到imu坐标系
    static cv::Mat ENU_to_IMU(double yaw, double pitch, double roll)
    {
        return IMU_to_ENU(yaw,pitch,roll).inv();
    }

    //! 经纬度转高斯投影坐标（B 维度  L 精度   H 高程)
    static Point3d BLH_to_GaussPrj(const BLHCoordinate &BLH, Datum datum = WGS84Datum) //,double lon)
    {
        int ProjNo, ZoneWide; ////带宽
        double longitude0, X0, xval, yval;
        double a, e2, ee, NN, T, C, A, M, b, l; //, h;
        b = BLH.lat;
        l = BLH.lon;
        //    h = BLH.z;
        ZoneWide = 3; //3度带宽
        a = datum.r_max;
        ProjNo = (int)((l - 1.5) / ZoneWide + 1);
        longitude0 = ProjNo * ZoneWide; //中央经线

        longitude0 = D2R(longitude0);
        l = D2R(l); //经度转换为弧度
        b = D2R(b); //纬度转换为弧度
        e2 = datum.e2;
        ee = e2 * (1.0 - e2);
        NN = a / sqrt(1.0 - e2 * sin(b) * sin(b));
        T = tan(b) * tan(b);
        C = ee * cos(b) * cos(b);
        A = (l - longitude0) * cos(b);

        M = a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * b - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * sin(2 * b) + (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * sin(4 * b) - (35 * e2 * e2 * e2 / 3072) * sin(6 * b));
        xval = NN * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ee) * A * A * A * A * A / 120);
        yval = M + NN * tan(b) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * ee) * A * A * A * A * A * A / 720);
        X0 = 500000L;
        xval = xval + X0;
        return Point3d(yval, xval, BLH.alt);
    }

    //! 高斯投影由大地平面坐标(Unit:Metres)反算经纬度(Unit:DD)
    static BLHCoordinate GaussPrj_to_BLH(Point3d XYZ, double lon, Datum datum = WGS84Datum)
    {
        int ProjNo, ZoneWide; ////带宽
        double l, b, longitude0, X0, xval, yval;
        double e1, e2, a, ee, NN, T, C, M, D, R, u, fai;
        a = datum.r_max; //54年北京坐标系参数
        ZoneWide = 3;    //3度带宽

        ProjNo = (int)(XYZ.y / 1000000L);                          //查找带号
        longitude0 = (int)((lon - 1.5) / ZoneWide + 1) * ZoneWide; //中央经线

        longitude0 = D2R(longitude0); //中央经线
        X0 = ProjNo * 1000000L + 500000L;
        yval = XYZ.y - X0; //带内大地坐标
        xval = XYZ.x;
        e2 = datum.e2;
        e1 = (1.0 - sqrt(1 - e2)) / (1.0 + sqrt(1 - e2));
        ee = e2 / (1 - e2);
        M = xval;
        u = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));
        fai = u + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * u) + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * u) + (151 * e1 * e1 * e1 / 96) * sin(6 * u) + (1097 * e1 * e1 * e1 * e1 / 512) * sin(8 * u);
        C = ee * cos(fai) * cos(fai);
        T = tan(fai) * tan(fai);
        NN = a / sqrt(1.0 - e2 * sin(fai) * sin(fai));

        R = a * (1 - e2) / sqrt((1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)));
        D = yval / NN;
        //计算经度(Longitude) 纬度(lat)
        l = longitude0 + (D - (1 + 2 * T + C) * D * D * D / 6 + (5 - 2 * C + 28 * T - 3 * C * C + 8 * ee + 24 * T * T) * D * D * D * D * D / 120) / cos(fai);
        b = fai - (NN * tan(fai) / R) * (D * D / 2 - (5 + 3 * T + 10 * C - 4 * C * C - 9 * ee) * D * D * D * D / 24 + (61 + 90 * T + 298 * C + 45 * T * T - 256 * ee - 3 * C * C) * D * D * D * D * D * D / 720);
        //转换为度 DD
        l = R2D(l);
        b = R2D(b);
        return BLHCoordinate{b, l, XYZ.z};
    }
    //! 经纬度转mercator投影
    static inline Point3d BLH_to_Mercator(const BLHCoordinate &blh)
    {
        Point3d mercator;

        mercator.x = D2R(blh.lon) * WGS84Datum.r_max;

        mercator.y = log(tan(D2R(blh.lat) * 0.5 + 0.25 * PI64)) * WGS84Datum.r_max;

        mercator.z = 0; // blh.altitude;

        return mercator;
    }
    //! mercator投影转经纬度
    static inline BLHCoordinate Mercator_to_BLH(const Point3d &mercator)
    {
        BLHCoordinate blh;

        blh.lon = R2D(mercator.x / WGS84Datum.r_max);

        blh.alt = 0; //blh.altitude;

        blh.lat = R2D(2 * (atan(exp(mercator.y / WGS84Datum.r_max))) - 0.5 * PI64);

        return blh;
    }


    // Checks if a matrix is a valid rotation matrix.
    static bool inline IsRotationMatrix(const Mat &R)
    {
        Mat Rt;
        transpose(R, Rt);
        Mat shouldBeIdentity = Rt * R;
        Mat I = Mat::eye(3,3, shouldBeIdentity.type());
        return  norm(I, shouldBeIdentity) < 1e-6;
    }

    // Calculates rotation matrix to euler angles
    // The result is the same as MATLAB except the order
    // of the euler angles ( x and z are swapped ).
    static Vec3f RotationMatrixToEulerAngles(const Mat &R)
    {
    
        assert(IsRotationMatrix(R));
        
        float sy = sqrt(R.at<MATTYPE>(0,0) * R.at<MATTYPE>(0,0) +  R.at<MATTYPE>(1,0) * R.at<MATTYPE>(1,0) );
    
        bool singular = sy < 1e-6; // If
    
        float x, y, z;
        if (!singular)
        {
            x = atan2(R.at<MATTYPE>(2,1) , R.at<MATTYPE>(2,2));
            y = atan2(-R.at<MATTYPE>(2,0), sy);
            z = atan2(R.at<MATTYPE>(1,0), R.at<MATTYPE>(0,0));
        }
        else
        {
            x = atan2(-R.at<MATTYPE>(1,2), R.at<MATTYPE>(1,1));
            y = atan2(-R.at<MATTYPE>(2,0), sy);
            z = 0;
        }
        return Vec3f(x, y, z);
    }

    // Calculates rotation matrix given euler angles.
    static Mat EulerAnglesToRotationMatrix(const Vec3f &theta)
    {
        // Calculate rotation about x axis
        Mat R_x = (Mat_<MATTYPE>(3,3) <<
                1,       0,              0,
                0,       cos(theta[0]),   -sin(theta[0]),
                0,       sin(theta[0]),   cos(theta[0])
                );
        
        // Calculate rotation about y axis
        Mat R_y = (Mat_<MATTYPE>(3,3) <<
                cos(theta[1]),    0,      sin(theta[1]),
                0,               1,      0,
                -sin(theta[1]),   0,      cos(theta[1])
                );
        
        // Calculate rotation about z axis
        Mat R_z = (Mat_<MATTYPE>(3,3) <<
                cos(theta[2]),    -sin(theta[2]),      0,
                sin(theta[2]),    cos(theta[2]),       0,
                0,               0,                  1);
        
        
        // Combined rotation matrix
        Mat R = R_z * R_y * R_x;
        
        return R;
    }
};

} // namespace Position

#endif