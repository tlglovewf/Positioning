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

#include <regex>

#if (defined __APPLE__) || (defined __unix__)  || (defined LINUX)
#include "dirent.h"
#endif

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
        #if 0 //old version
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
        #else
        try
        {
            regex re{delim};
            return vector<string>{
                    sregex_token_iterator(str.begin(), str.end(), re, -1),
                    sregex_token_iterator()
               };      
        }
        catch(const std::exception& e)
        {
            cout<<"error:"<<e.what()<<std::endl;
        }
        #endif
        return tokens;
    }

    /* 取first sec 中间字符
     */
    inline static std::string GetCenterStr(const std::string &img,const std::string &first,const std::string &sec)
    {
        size_t m = img.find_last_of(first);
        size_t n =  img.find(sec);
        
        if(n != string::npos)
        {
            return img.substr(m + 1,n - m - 1);
        }

        return std::string();
    }

#define JPGSTR "jpg"
#define PNGSTR "png"

    /* 后缀
    */
    static bool IsPicSuffix(const char *pName,size_t len)
    {
        const size_t suffix = 3;
        assert(pName);
        if(len < suffix)
        {
            return false;
        }
        const char *pSuffix =  &pName[len - suffix];

        return !strcasecmp(pSuffix, JPGSTR) | !strcasecmp(pSuffix, PNGSTR);
    }

    /* 替换后缀
    */
    static void ReplaceFileSuffix(string &name,const string &pfix, const string &efix)
    {
        assert(pfix.size() == efix.size());
        size_t n = name.find_first_of(pfix);
        if(string::npos == n)
        {
           name = name + "." + efix;
        }
        else
        {
            name.replace(n,efix.size(),efix);
        }
    }

#undef JPGSTR
#undef PNGSTR
    /* 扫描目录下文件名
    */
    static int LoadPathNames( const std::string &dirpath, Position::StringVector &files )
    {
        int index = 0;
        
#if (defined __APPLE__) || (defined __unix__) || (defined LINUX)
        DIR *dp;
        struct dirent *dirp;
        if((dp = opendir(dirpath.c_str())) == NULL)
        {
            assert(NULL);
        }
        
        while((dirp = readdir(dp)) != NULL)
        {
            if(IsPicSuffix(dirp->d_name,strlen(dirp->d_name)))
            {
                files.emplace_back(dirp->d_name);
                ++index;
            }
        }
        closedir(dp);
        sort(files.begin(),files.end());
#else
        //add other platforms
#endif
        return index;
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

    //计算内点率 
    //status  基础矩阵或者单应矩阵 ransac计算出的掩码数组
    static float ComputeInlierRate(const U8Vector &status)
    {
        int inlier = std::count_if(status.begin(), status.end(),[](u8 n)->bool{
                return n > 0;});
        return inlier / (float)status.size();
    }

    //绘制匹配图
    //status  单应矩阵或者基础矩阵 ransac计算的掩码数组
    static Mat DrawFeatureMatch(const Mat &img1, const Mat &img2,
                      const KeyPtVector &pt1s, const KeyPtVector &pt2s,
                      const MatchVector &matches,
                      const U8Vector &status = U8Vector());

    //绘制网格线段
    static void DrawGridLine(Mat &img ,int width, int height)
    {
        assert(!img.empty());
        assert(img.cols > width);
        assert(img.rows > height);

        int nw = img.cols / width;
        int nh = img.rows / height;

        auto color = CV_RGB(255,0,0);
        int thickness = 1;
        for(int i = 1; i < nw; ++i)
        {
           const float xw = i * width;
           line(img,cv::Point2f( xw ,0),cv::Point2f( xw, img.rows),color,thickness);
        }
        for(int i = 1; i < nh; ++i)
        {
            const float yh = i * height;
            line(img,cv::Point2f(0,yh),cv::Point2f(img.cols,yh),color,thickness);
        }
    }

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
        cv::Mat t_x = (cv::Mat_<MATTYPE>(3, 3) << 0, -t.at<MATTYPE>(2, 0), t.at<MATTYPE>(1, 0),
                                                  t.at<MATTYPE>(2, 0), 0, -t.at<MATTYPE>(0, 0),
                                                 -t.at<MATTYPE>(1, 0), t.at<MATTYPE>(0, 0), 0);
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
    //同名点量测
    static Mat CorPtsMeasure(const CameraParam &camera, const Mat &R, const Mat &t, const Point2f &pt1, const Point2f &pt2)
    {
        if(R.empty() || t.empty()  || camera.K.empty())
            return Mat();
        cv::Mat P1(3,4,MATCVTYPE,cv::Scalar(0));
        camera.K.copyTo(P1.rowRange(0,3).colRange(0,3));

        // Camera 2 Projection Matrix K[R|t]
        cv::Mat P2(3,4,MATCVTYPE);
        R.copyTo(P2.rowRange(0,3).colRange(0,3));
        t.copyTo(P2.rowRange(0,3).col(3));
        P2 = camera.K * P2;
        Mat rst;
        PUtils::Triangulate(pt1,pt2,P1,P2,rst);
        return rst;
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

    //根据两帧位姿态,计算速度
    static inline cv::Mat computeVelocity(const cv::Mat &prepose, const cv::Mat &curpose)
    {
       return curpose * prepose.inv();
    }

    // 设直线方程为ax+by+c=0,点坐标为(m,n)
    // 则垂足为((b*b*m-a*b*n-a*c)/(a*a+b*b),(a*a*n-a*b*m-b*c)/(a*a+b*b)) 
    static inline Point2f GetFootPoint(MATTYPE a, MATTYPE b, MATTYPE c, const Point2f &pt)
    {
        MATTYPE x = (b * b * pt.x - a * b * pt.y - a * c) / (a * a + b * b);
        MATTYPE y = (a * a * pt.y - a * b * pt.x - b * c) / (a * a + b * b);

        return Point2f(x,y);
    }

    //根据基础矩阵 推算直线 ax + by + c = 0
    static inline void CalcEpiline(const Mat &F21,const Point2f &prept,MATTYPE &a,MATTYPE &b,MATTYPE &c)
    {
            const MATTYPE f11 = F21.at<MATTYPE>(0,0);
            const MATTYPE f12 = F21.at<MATTYPE>(0,1);
            const MATTYPE f13 = F21.at<MATTYPE>(0,2);
            const MATTYPE f21 = F21.at<MATTYPE>(1,0);
            const MATTYPE f22 = F21.at<MATTYPE>(1,1);
            const MATTYPE f23 = F21.at<MATTYPE>(1,2);
            const MATTYPE f31 = F21.at<MATTYPE>(2,0);
            const MATTYPE f32 = F21.at<MATTYPE>(2,1);
            const MATTYPE f33 = F21.at<MATTYPE>(2,2);

            a = f11 * prept.x + f12 * prept.y + f13;
            b = f21 * prept.x + f22 * prept.y + f23;
            c = f31 * prept.x + f32 * prept.y + f33;
    }

    //根据基础矩阵 创建 ax + by + c = 0 极线
    // static void CreateEpiline(const cv::Mat &F, cv::Point2f pt, double &a, double &b, double &c)
    // {
    //     std::vector<cv::Point2f> selPoints1;
    //     selPoints1.push_back(pt);
    //     std::vector<cv::Vec3f> epline;

    //     cv::computeCorrespondEpilines(cv::Mat(selPoints1), 1, F, epline);

    //     a = epline[0][0];
    //     b = epline[0][1];
    //     c = epline[0][2];
    // }

    //通过R,t 计算极线（内参越准,计算结果越准）
    static inline EpLine ComputeEpLine(const cv::Mat &R, const cv::Mat &t,const CameraParam &cam, const cv::Point2f &pt)
    {
        assert(!R.empty() && !t.empty());
        assert(!cam.K.empty());

        cv::Mat F = PUtils::ComputeFFromRT(R,t,cam.K);

        double a,b,c;
        PUtils::CalcEpiline(F,pt,a,b,c);

        PROMT_V("ep line",a, b, c);
        return EpLine(a,b,c);
    }

    //反投 依赖Frame的姿态 rpy
    static cv::Point2f backProject(const FrameData &frame,const BLHCoordinate &blh,const CameraParam &camera)
    {
        //calc target xyz coordinate
	    Point3d  xyz = PCoorTrans::BLH_to_XYZ(blh);

        //calc cam xyz coordinate
	    Point3d  cam = PCoorTrans::BLH_to_XYZ(frame._pos.pos);

        Point3d dt = xyz - cam;

        //trans pt -> mat
        Mat tmp = (Mat_<MATTYPE>(3,1) << dt.x ,dt.y,dt.z);

        //xyz -> enu
	    cv::Mat XYZ2Enu1 = PCoorTrans::XYZ_to_ENU(frame._pos.pos.lat, frame._pos.pos.lon);

        //计算imu到enu 转换矩阵
	    cv::Mat Rimu2Enu1 = PCoorTrans::IMU_to_ENU(-frame._pos._yaw, frame._pos._pitch, frame._pos._roll);

        //xyz->enu->imu
        Mat rst = Rimu2Enu1.t() * XYZ2Enu1 * tmp;

        //imu-> cam
        //imu与cam不重合存在转换值时
        if(!camera.TCam2Imu.empty())
        {
            rst = rst - camera.TCam2Imu;
            rst =  camera.RCam2Imu.t() *  rst ;
        }
      

        Mat t = camera.K * rst ;

        Point2f pt;
        pt.x = t.at<MATTYPE>(0,0) / t.at<MATTYPE>(2,0);
        pt.y = t.at<MATTYPE>(1,0) / t.at<MATTYPE>(2,0);

        return pt;
    }


    //绘制关键点
    static Mat DrawKeyPoints(const Mat &img, const KeyPtVector &keys)
    {
        Mat keypoint_img;
        drawKeypoints(img,keys,keypoint_img,CV_RGB(0,0,255),DrawMatchesFlags::DEFAULT);
        putText(keypoint_img,"key size:" + std::to_string(keys.size()),cv::Point2f(50,50),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255,0,0), 2, CV_AA);
        return keypoint_img;
    }

    //绘制极线 ax + by + c = 0    
    //pt 另一帧像素坐标    img 当前帧图像
    static void DrawEpiLine(MATTYPE a, MATTYPE b, MATTYPE c, const Point2f &pt, Mat &img);

    //像素坐标系->图像坐标系
    static inline cv::Point2d Pixel2Cam(const cv::Point2d &p, const cv::Mat &K)
    {
        return cv::Point2d(
            (p.x - K.at<MATTYPE>(0, 2)) / K.at<MATTYPE>(0, 0),
            (p.y - K.at<MATTYPE>(1, 2)) / K.at<MATTYPE>(1, 1));
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

     //计算质心
    //P:[t1,t2,t3,.....]
    static void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
    {
        // 将二维数组转化为向量
        // src：输入矩阵
        // dst：通过处理输入矩阵的所有行/列而得到的单行/列向量
        // dim：矩阵被简化后的维数索引.0意味着矩阵被处理成一行,1意味着矩阵被处理成为一列,-1时维数将根据输出向量的大小自动选择.
        // CV_REDUCE_SUM： 输出是矩阵的所有行/列的和
        cv::reduce(P,C,1,cv::REDUCE_SUM);//将二维数组转化为向量，第3个参数表示转为1列。这里即是把P的每行数据t1,t2,t3,....相加
        C = C/P.cols;//相加后除以总列数得到平均数，即是质心

        for(int i=0; i<P.cols; i++)
        {
            Pr.col(i)=P.col(i)-C;
        }
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

        cv::Mat trans = (Mat_<MATTYPE>(4,4) << pitch.x,up.x,yaw.x,lft_xyz.x,
                                              pitch.y,up.y,yaw.y,lft_xyz.y,
                                              pitch.z,up.z,yaw.z,lft_xyz.z,
                                              0,0,0,1);


        return trans;
     }

    /* --------------------------------------------------- */
    /* ------------------ common functions --------------- */
    /* --------------------------------------------------- */
     /*
      * make vector items unique(vts must suppport operator <)
     */
     template<typename T>
     static inline void MakeVectorItemsUnique( std::vector<T> &vts )
     {
         std::sort(vts.begin(),vts.end());
         vts.erase(std::unique(vts.begin(),vts.end()),vts.end());
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
    inline void prompt(const std::string &str,bool isreset = false)
    {
        std::cout << str.c_str() << end() << "s" << std::endl;
        if(isreset)
            start();
    }

protected:
    time_t _t;
};
} // namespace Position

#endif