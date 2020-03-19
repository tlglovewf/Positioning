/**
 *   P_Utils.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PUTILS_H_H_
#define __PUTILS_H_H_

#include <ctime>

#include "P_Types.h"
#include "P_CoorTrans.h"
#include "P_Writer.h"
namespace Position
{

#define GPS_LEAP_TIME 18.0 //GPS 闰秒
//工具类
class PUtils
{
public:
    /****************************************************
     * ****************字符串相关*************************
     * **************************************************/
    static StringVector SplitString(const string &str, const string &delim)
    {
        std::vector<std::string> tokens;
        size_t prev = 0, pos = 0;
        do
        {
            pos = str.find(delim, prev);
            if (pos == std::string::npos) pos = str.length();
            std::string token = str.substr(prev, pos-prev);
            if (!token.empty()) tokens.push_back(token);
            prev = pos + delim.length();
        }
        while (pos < str.length() && prev < str.length());
        return tokens;
    }

    /****************************************************
     **********************时间处理***********************
     ****************************************************/

    /* 获取拉格朗日插值
     *
     */
    static double inline GetLxValue(double x, double x1, double x2, double y1, double y2)
    {
        double dx = x1 - x2;

        return y1 * (x - x2) / dx + y2 * (x - x1) / -dx;
    }

    /* "hhmmsszzzzzz"   ->    天秒
     * "031953179985"
     */
    static inline double HMS2DaySec(const std::string &str)
    {
        int h;
        int m;
        int s;
        int z;
        sscanf(str.c_str(), "%2d%2d%2d%6d", &h, &m, &s, &z);

        double daysec = h * 3600 + m * 60 + s + z / 1.0e6 + GPS_LEAP_TIME;

        return daysec;
    }

    /* 周秒 -> 天秒
     *
     */
    static inline double WeekSec2DaySec(double wktime)
    {
        const double daysec = 86400;

        int iday = wktime / daysec;

        double dtime = wktime - iday * daysec;

        return dtime;
    }

    /* 天秒 -> 时分秒
     *
     */
    static std::string DaySec2HMS(double dtime)
    {
        dtime -= GPS_LEAP_TIME;

        int zz = (dtime - (int)dtime) * 1e6;

        int hh = (int)dtime / 3600;

        int _m = (int)dtime - hh * 3600;

        int mm = _m / 60;

        int ss = _m - mm * 60;

        std::string s;
        std::stringstream str;
        if (hh < 10)
            str << 0;
        str << hh;
        str << mm;
        str << ss;
        str << zz;

        str >> s;

        return s;
    }

    /*
    *  周秒 -> 天秒
    */
    static double Wktime2Daytime(double wktime)
    {
        //天秒
        static const int daysec = 3600 * 24;
        double msec = wktime - static_cast<int>(wktime);
        int sttime = (static_cast<int>(wktime) % daysec);
        return (sttime + msec);
    }

    /* 从位姿获取R和t
     *
     */
    static void GetRtFromPose(const PoseData &predata,
                              const PoseData &curdata,
                              const Mat cam2imuR,
                              const Mat cam2imuT,
                              Mat &R, Mat &t);

    /* 通过帧间R、t推算绝对坐标
     *
     */
    static void CalcPoseFromRT(const PoseData &origin,
                               const Mat &R, const Mat &t,
                               const Mat &cam2imuR,
                               const Mat &cam2imuT,
                               BLHCoordinate &blh,
                               const PoseData &realdst);

    /****************************************************
     **********************视觉相关***********************
     ****************************************************/
    //图像像素 相似度计算
    static void CheckPixelSimilarity(const Mat &img1, const Mat &img2);

    //直方图 相似度计算
    static void CheckHistSimilarity(const Mat &img1, const Mat &img2);

    //直方图均衡
    static void ImageHistEqualized(const Mat &img, Mat &outimg);
    //绘制直方图
    static void HistDraw(const Mat &img);

    //平行合并保存
    static void CombineSave(const Mat &img1, const Mat &img2, const std::string &out)
    {
        assert(img1.size() == img2.size());
        Mat outimg;
        cv::hconcat(img1, img2, outimg);
        PROMT_V("Save image to ", outimg);
        imwrite(out,outimg);
    }

    //获取反对称矩阵
    static inline cv::Mat antisymMat(const cv::Mat &t)
    {
        cv::Mat t_x = (cv::Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
                       t.at<double>(2, 0), 0, -t.at<double>(0, 0),
                       -t.at<double>(1, 0), t.at<double>(0, 0), 0);
        return t_x;
    }
    //计算本质矩阵
    static inline cv::Mat ComputeEssentialMat(const cv::Mat &R, const cv::Mat &T)
    {
        return antisymMat(T) * R;
    }
    //计算基础矩阵K1 K2 两个相机的内参矩阵  单目K1=K2
    static inline cv::Mat ComputeFundamentalMat(const cv::Mat &E12, const cv::Mat &K1, const cv::Mat &K2)
    {
        return (K2.inv()).t() * E12 * K1.inv();
    }
     //三角化 P1 K[I|0] P2 K[R|t]    x3D 4x1
    static void Triangulate(const cv::Point2f &kp1, const cv::Point2f &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
    {
        cv::Mat A(4,4,MATCVTYPE);

        A.row(0) = kp1.x*P1.row(2)-P1.row(0);
        A.row(1) = kp1.y*P1.row(2)-P1.row(1);
        A.row(2) = kp2.x*P2.row(2)-P2.row(0);
        A.row(3) = kp2.y*P2.row(2)-P2.row(1);

        cv::Mat u,w,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        x3D = vt.row(3).t();
        x3D = x3D.rowRange(0,3)/x3D.at<MATTYPE>(3);
    }
    //从位置矩阵总取R和t
    static void GetRTFromFramePose(const cv::Mat &pose, cv::Mat &R, cv::Mat &t)
    {
        if(pose.empty())
            return;
        R = pose.rowRange(0,3).colRange(0,3).clone();
        t = pose.rowRange(0,3).col(3);
    }
    //单目 从R t中反推F
    static inline cv::Mat ComputeFFromRT(const cv::Mat &R,const cv::Mat &t,const cv::Mat &K)
    {
        cv::Mat E = ComputeEssentialMat(R,t);
        return ComputeFundamentalMat(E,K,K);
    }
    //根据基础矩阵 创建 ax + by + c = 0 极线
    static void CreateEpiline(const cv::Mat &F, cv::Point2f pt, double &a, double &b, double &c)
    {
        std::vector<cv::Point2f> selPoints1;
        selPoints1.push_back(pt);
        std::vector<cv::Vec3f> epline;

        cv::computeCorrespondEpilines(cv::Mat(selPoints1), 1, F, epline);

        a = epline[0][0];
        b = epline[0][1];
        c = epline[0][2];
    }

    //像素坐标系->图像坐标系
    static inline cv::Point2d Pixel2Cam(const cv::Point2d &p, const cv::Mat &K)
    {
        return cv::Point2d(
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
    }

    //y = (-a*x - c) / b   计算y值
    static inline double ComputeY(float x, float a, float b, float c)
    {
        assert(fabs(b) > 1e-6);
        return (-c - a * x) / b;
    }

    /*
     * 叉乘
     */
    inline static float segmentCross(
        const Point2f &v,
        const Point2f &vStart,
        const Point2f &vEnd)
    {
        return (vStart.x - v.x) * (vEnd.y - v.y) - (vEnd.x - v.x) * (vStart.y - v.y);
    }

#define YD_EPS_REAL32 1e-6
    /**
     * 判断线段与线段是否相交(跨立试验）
     * @param vStart1,vEnd1 线段1起终点
     * @param vStart2,vEnd2 线段2起终点
     * @return 是否相交
     */
    static bool IsIntersect(const Point2f &vStart1, const Point2f &vEnd1,
                            const Point2f &vStart2, const Point2f &vEnd2)
    {
        float leftS, leftE;
        leftS = segmentCross(vStart1, vStart2, vEnd2);
        leftE = segmentCross(vEnd1, vStart2, vEnd2);
        if (leftS * leftE > YD_EPS_REAL32)
        {
            return false; // vStart1, vEnd1在另一条直线的同侧
        }

        leftS = segmentCross(vStart2, vStart1, vEnd1);
        leftE = segmentCross(vEnd2, vStart1, vEnd1);
        if (leftS * leftE > YD_EPS_REAL32)
        {
            return false; // vStart2, vEnd2在另一条直线的同侧
        }

        return true;
    }

    /**
     * 判断直线是否与矩形相交
     * @param bg,ed 线段起终点
     * @param rect  矩形
     */
    static bool IsIntersect(const Point2f &bg, const Point2f &ed, const Rect2f &rect)
    {
        if (rect.contains(bg) || rect.contains(ed))
        { //先判断直线起终点是否在矩形内
            return true;
        }
        else
        { //再判断直线是否与矩形对角线相交
            return IsIntersect(bg, ed, Point2f(rect.x, rect.y),
                               Point2f(rect.x + rect.width, rect.y + rect.height)) ||
                   IsIntersect(bg, ed, Point2f(rect.x + rect.width, rect.y),
                               Point2f(rect.x, rect.y + rect.height));
        }
    }

    /**
	* 获取文件名No
	* @param  name  名
	* @return rect  序号
	*/
    static inline std::string GetNameNo(const std::string &name)
    {
        int index = name.find_last_of('-');

        std::string tmp = name.substr(++index);

        tmp.erase(tmp.find_last_of('.'), 4);

        index = tmp.find_first_not_of('0');

        return tmp.substr(index);
    }

    /* get gauss project error
    */
    static inline cv::Point3d CalcGaussErr(const BLHCoordinate &lf, const BLHCoordinate &rg)
    {
        return PCoorTrans::BLH_to_GaussPrj(rg) - PCoorTrans::BLH_to_GaussPrj(lf);
    }

    /* normalize vector
    */
    static inline Point3d Normalize(const Point3d &pt)
    {
        return pt / cv::norm(pt);
    }

     /* get item BLH from two frame gps 
     */
     static cv::Mat CalcTransBLH(const PoseData &lft, const PoseData &rgt)
     {
        Point3d lft_xyz = PCoorTrans::BLH_to_XYZ(lft.pos);
        Point3d rgt_xyz = PCoorTrans::BLH_to_XYZ(rgt.pos);

        Point3d yaw = rgt_xyz - lft_xyz;

        yaw = Normalize(yaw); //z

        Point3d up = Normalize(lft_xyz); //y

        Point3d pitch = yaw.cross(up); //x

        up = yaw.cross(pitch);

        up = Normalize(up);

        cv::Mat trans = (Mat_<double>(4,4) << pitch.x,up.x,yaw.x,lft_xyz.x,
                                              pitch.y,up.y,yaw.y,lft_xyz.y,
                                              pitch.z,up.z,yaw.z,lft_xyz.z,
                                              0,0,0,1);


        return trans;
     }
     
};

//及时
class Time_Interval
{
public:
    /*  开始
     */
    inline void start()
    {
        _t = clock();
    }
    /*  结束
     */
    inline float end()
    {
        return (clock() - _t) / (float)CLOCKS_PER_SEC;
    }
    /* 输出
	 */
    inline void prompt(const std::string &str)
    {
        std::cout << str.c_str() << end() << "s" << std::endl;
    }

protected:
    time_t _t;
};
} // namespace Position

#endif