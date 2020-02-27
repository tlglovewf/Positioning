/**
 *   p_inteface.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PINTERFACE_H_H_
#define __PINTERFACE_H_H_
#include "P_Types.h"
namespace Position
{
#define DISABLEDCP(X)  X(const X&); \
                        X& operator=(const X&);

#define DISABLEDC(X)   X();\
                      ~X();
    // base class 
    class IBase
    {
    public:
        IBase(){}
        virtual ~IBase(){}
        void release()
        {
            delete this;
        }
    };

    //配置属性
    class IConfigParam : public IBase
    {
    public:
        // 获取值
        virtual void* data() = 0; 
    };
#define GETCFGVALUE(v,type) (*reinterpret_cast<type*>(v->data()))
#define SETCFGVALUE(v,d)    (GETVALUE(v,decltype(d))) = d
    // config interface
    class IConfig : public IBase
    {
    public:
        // 加载配置文件
        virtual void load(const std::string &path) = 0;

        // 获取值
        virtual IConfigParam* operator[](const std::string &name) = 0;
        
    };

    //地图点
    class IMapPoint : public IBase
    {
    public:
        //获取位置(世界坐标)
        virtual const Mat getPose()const = 0;
        //设置位置(世界坐标)
        virtual void setPose(const cv::Mat &pose) = 0;
        //获取序号
        virtual u64 index()const = 0;
        //观察点
        virtual int observations()const = 0;
        //添加观察者 index(特征点序号)
        virtual void addObservation(IFrame *frame,int index) = 0;
        //移除观察者
        virtual void rmObservation(IFrame *frame) = 0;
        //获取观察帧列表
        virtual const FrameMap& getObservation()const = 0;
        //是否在帧中
        virtual bool isInFrame(IFrame *pFrame) = 0;
        //设置坏点
        virtual void setBadFlag() = 0;
        //返回是否为坏点
        virtual bool isBad()const = 0;

    };
    //帧对象
    class IFrame : public IBase
    {
    public:
        //获取数据
        virtual FrameData getData()const = 0;
        //获取关键点
        virtual const KeyPtVector& getKeys()const = 0;
        //获取特征点数量
        virtual int getKeySize()const = 0;
        //获取描述子
        virtual const Mat& getDescript()const = 0;
        //获取位置(世界坐标)
        virtual const Mat getPose()const = 0;
        //设置位置(世界坐标)
        virtual void setPose(const cv::Mat &pose) = 0;
        //获取地图点
        virtual const MapPtVector& getPoints() = 0;
        //帧序号
        virtual int index()const = 0;
        //添加地图点
        virtual void addMapPoint(IMapPoint *pt, int index) = 0;
        //是否已有对应地图点
        virtual bool hasMapPoint(int index) = 0;
        //移除地图点
        virtual void rmMapPoint(IMapPoint *pt) = 0;
        virtual void rmMapPoint(int index) = 0;
        //是否为坏点
        virtual bool isBad()const = 0;
        //设为坏帧
        virtual void setBadFlag() = 0;
    };

    // data interface
    class IData : public IBase
    {
    public:
        //预处理数据
        virtual bool loadDatas() = 0;

        //第一个元素
        virtual FrameDataVIter begin() = 0;

        //最后一个元素
        virtual FrameDataVIter end() = 0;

        // 获取相机参数 default(0)  left    1 right 
        virtual CameraParam getCamera(int index = 0) = 0;
    };

    // serialization interface
    class IWriter : public IBase
    {
    public:
    };

    // visual interface
    class IViewer : public IBase
    {
    public:
    };

    //检测对象
    class IDetector : public IBase
    {
    public:
        //识别
        virtual TargetVector detect(const Mat &img) = 0;
    };

    //检查
    class IChecker : public IBase
    {
    public:
        //检查
        virtual bool check(const FrameData &frame) = 0;
    };

    //特征接口
    class IFeature : public IBase
    {
    public:
        //计算特征点
        virtual bool detect(const FrameData &frame,KeyPtVector &keys, Mat &descript) = 0;
        //返回sigma参数(主要用于优化 信息矩阵)
        virtual const FloatVector& getSigma2() const = 0;
    };

    //特征跟踪接口
    class IFeatureMatcher : public IBase
    {
    public:
        
        //匹配  返回匹配对
        virtual MatchVector match(IFrame *preframe, IFrame *curframe, int windowsize) = 0;

    };

    //位姿推算
    class IPoseEstimation
    {
    public:
        //设置相机参数
        virtual void setCamera(const CameraParam &cam) = 0;
        //设置帧
        virtual void setFrames( IFrame *pre, IFrame *cur) = 0;
        //推算位姿
        virtual bool estimate(cv::Mat &R, cv::Mat &t, MatchVector &matches, Pt3Vector &vPts, BolVector &bTriangle) = 0;
    };

    //定位算法
    class IPositioning : public IBase
    {
    public:
        //设置相机参数
        virtual void setParams(const CameraParam &cam) = 0;
        //添加单张
        virtual void addFrame(IFrame *pframe) = 0;
        //添加多张
        virtual void addFrames(const FrameVector &framedatas) = 0;
        //处理
        virtual void position() = 0;
    };
} // namespace Position

#endif