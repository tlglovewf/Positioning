/**
 *   P_Types.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PTYPES_H_H_
#define __PTYPES_H_H_

#include <set>
#include <map>
#include <cmath>
#include <vector>
#include <memory>
#include <string>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>



using namespace std;
using namespace cv;

namespace Position
{

#define HIGHTPRE 1


#if  HIGHTPRE  
    #define MATTYPE   double     
    #define MATCVTYPE CV_64F
#else
    #define MATTYPE   float     
    #define MATCVTYPE CV_32F
#endif

//log info marco define 
#define _TOSTRING(a)              #a
#define _RECORD_FILE(a)             __FILE__   "(" _TOSTRING(a) ") : "
#define _RECORD_LOG(a)              _RECORD_FILE(__LINE__) _TOSTRING(a)
#define _RECORD_TIME_LOG()          _RECORD_FILE(__LINE__) __TIME__
#define _DATE_LOG_()                __DATE__
#define _DATE_TIME_LOG()            __TIMESTAMP__


#define WRONGDATA -1000 //错误值
#define OUTPUTRESULT 1

#define RELEASEPT(X)  if(X){delete X; X = NULL;}
#define RELEASEPTS(X) if(X){delete [] X;X = NULL;}

#define  CHITH  5.991  //卡方阈值 

//base type define 
typedef unsigned char u8;
typedef unsigned int  u32;
typedef unsigned long u64;

//! 经纬度
struct BLHCoordinate
{
    double lon;

    double lat;

    double alt;

    BLHCoordinate(double _lat, double _lon, double _alt) : lon(_lon),lat(_lat), alt(_alt) {}

    BLHCoordinate() : alt(WRONGDATA) {}

    //有效性
    static inline bool isValid(const BLHCoordinate &blh)
    {
        return blh.alt > WRONGDATA;
    }
};

typedef Point3d XYZCoordinate;

//! 相机结构体
struct CameraParam
{
    Mat K;           //相机内参
    Mat D;           //畸变参数
    Mat RCam2Imu;    //相机->IMU坐标系 旋转矩阵
    Mat TCam2Imu;    //相机->IMU坐标系 平移矩阵
    u8 fps;          //相机帧率
    u8 rgb;          //相机颜色值(1rgb 0gray)
    CameraParam():fps(0),rgb(0){}
    //add more
};

//! 姿态结构体
struct PoseData {
    double _t;          //时间
    
    BLHCoordinate pos;  //位置
    
    double _pitch;      //俯仰角
    double _roll;       //翻滚角
    double _yaw;        //航偏角

    PoseData()
    { memset(this,0,sizeof(*this));}
};

//! imu 原始数据
struct ImuRawData
{
    double _t;
    double _gyro_x; //陀螺仪加速度
    double _gyro_y;
    double _gyro_z;

    double _acc_x;  //线加速度
    double _acc_y;
    double _acc_z;  

    ImuRawData(double t, double gx,double gy, double gz,
                          double ax,double ay, double az):_t(t),_gyro_x(gx),_gyro_y(gy),_gyro_z(gz),
                                                          _acc_x(ax),_acc_y(ay),_acc_z(az)
                          {

                          }
    ImuRawData()
    {
        memset(this,0,sizeof(*this));
    }
};

//! 信息索引 图片索引，目标在图片中索引
typedef std::pair<int,int> InfoIndex;
struct BatchItem;
struct TrackerItem
{
    int             id      ;                   //目标id
    int             clsid   ;                   //小类别id
    InfoIndex       stno    ;                   //第一帧出现的索引
    InfoIndex       edno    ;                   //最后一帧出现的索引
    BLHCoordinate   blh     ;                   //经纬度
    int             maxsize ;                   //出现的帧数
    std::shared_ptr<BatchItem> batch  ;         //关联处理的批次,只有可能关联到一个批次
    TrackerItem(){memset(this,0,sizeof(*this));}
};
typedef std::vector<TrackerItem>        TrackerItemVector;
typedef TrackerItemVector::iterator     TrackerItemVIter; 

//! 目标结构体
struct TargetData {
  
    int           _type;     //类型
    
    Rect2f        _box;      //目标包围框
    
    BLHCoordinate _pos;      //无值
    
    //! 中心点
    inline Point2f center()const
    {
        return (_box.tl() + _box.br()) / 2;
    }
    
    //! 有效性
    static inline bool isValid(const TargetData &target)
    {
		return (target._type > WRONGDATA) || BLHCoordinate::isValid(target._pos) ;
    }
	TargetData() :_type(WRONGDATA){}
};

//! 结果
struct ResultData
{
    int           _type;        //类型

    BLHCoordinate _pos;         //坐标
};

typedef std::vector<TargetData> TargetVector;
typedef TargetVector::iterator  TargetVIter;
typedef std::vector<ResultData> ResultVector;
typedef ResultVector::iterator  ResultVIter;
//! 帧数据
struct FrameData {

    PoseData        _pos;     //姿态
    
	std::string     _name;     //路径

    Mat             _img;      //图像
    
    TargetVector    _targets;  //目标集
    
    //! 插入
    void push_back(const TargetData &target)
    {
        _targets.emplace_back(target);
    }
};

//! ax + by + c = 0
struct EpLine
{
    double a;
    double b;
    double c;
    EpLine():a(0),b(0),c(0){}
    EpLine(double _a,double _b,double _c):a(_a),b(_b),c(_c){}
};

//! 批处理值
struct BatchItem
{
    std::string _btname;
    int _n;
    std::vector<FrameData*>  _fmsdata;
    std::vector<Mat>         _poses;
    inline bool isvaild()const
    {
        return _fmsdata.size() == _poses.size();
    }
    BatchItem(const std::string &btn,int n):_btname(btn),_n(n)
    {
        _fmsdata.reserve(n);
        _poses.reserve(n);
    }
};
//! 图片序号组
struct ImagePair {
    size_t pre, cur;
};

typedef std::vector<cv::Point2f>                PtVector;
typedef std::vector<cv::Point3f>                Pt3Vector;

typedef std::vector<cv::DMatch>                 MatchVector;
typedef std::vector<MatchVector>                MatchVectors;
typedef std::vector<MatchVectors>               MatchMatrix;

typedef std::vector<cv::KeyPoint>               KeyPtVector;


//! 特征提取信息
struct FeatureInfo
{
    string      _name;
    KeyPtVector _keys;
    Mat         _des;
    FeatureInfo(const std::string &name):_name(name){}
    FeatureInfo(const std::string &name,
                const KeyPtVector &keys,
                const Mat         &des):_name(name),_keys(keys),_des(des){}
};

 //! 姿态估计组
struct InputPair
{
    const KeyPtVector &_query;
    const KeyPtVector &_train;
    const MatchVector &_match;
    InputPair(const KeyPtVector &query,
              const KeyPtVector &train,
              const MatchVector &match):_query(query),_train(train),_match(match){}
};

//! 姿态结果
struct PoseResult
{
    Mat         _R;
    Mat         _t;
    Pt3Vector   _vpts;
    MatchVector _match;
};


typedef vector<ImagePair>                   ImagePairVector;
typedef vector<std::shared_ptr<BatchItem> > PrjBatchVector;
typedef PrjBatchVector::iterator            PrjBatchVIter;

typedef std::vector<PoseData>               PoseVector;
typedef PoseVector::iterator                PoseVIter;

typedef std::vector<ImuRawData>             IMURawVector;
typedef IMURawVector::iterator              IMURawVIter;


typedef std::vector<ResultData>             RstVector;
typedef RstVector::iterator                 RstVIter;

typedef std::vector<FrameData*>             FrameDataPtrVector;
typedef FrameDataPtrVector::iterator        FrameDataPtrVIter;

class   IFrame;           
typedef std::vector<IFrame*>                FrameVector;
typedef FrameVector::iterator               FrameVIter;

class   IKeyFrame;
typedef std::vector<IKeyFrame*>             KeyFrameVector;
typedef KeyFrameVector::iterator            KeyFrameVter;

typedef std::set<IKeyFrame*>                KeyFmSet;
typedef KeyFmSet::iterator                  KeyFmSetIter;

typedef std::pair<IKeyFrame*,int>           KeyFramePair;
typedef std::map<IKeyFrame*,int>            KeyFrameMap;  //<keyframe,index>
typedef KeyFrameMap::iterator               KeyFrameMapIter;

typedef std::set<IFrame*>                   FrameSet;
typedef FrameSet::iterator                  FrameSetIter;


class IMapPoint;
typedef std::vector<IMapPoint*>             MapPtVector;
typedef MapPtVector::iterator               MapPtVIter;

typedef std::set<IMapPoint*>                MapPtSet;
typedef MapPtSet::iterator                  MapPtSetIter;

typedef std::vector<size_t>                 SzVector;
typedef std::vector<u8>                     U8Vector;
typedef std::vector<int>                    IntVector;
typedef std::vector<bool>                   BolVector;
typedef std::vector<float>                  FloatVector;

typedef pair<int,int>                       MatchPair;
typedef std::vector<MatchPair>              MatchPairs;

typedef vector<string>                      StringVector;


#define NameMap(T)                          std::map<std::string, std::shared_ptr<T> > 


} // namespace Position

#endif