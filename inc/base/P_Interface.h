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
    // base class 
    class IBase
    {
    public:
        IBase(){}
        virtual ~IBase(){}
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
        virtual void setParams( IFrame *pre, IFrame *cur, const MatchVector &matches) = 0;
        //推算位姿
        virtual bool estimate(cv::Mat &R, cv::Mat &t) = 0;
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