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
#define CFGVALUE(v,type) (*reinterpret_cast<type*>(v->data()))
#define GETCFGVALUE(v,str,type) CFGVALUE((*v)[#str],type)
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
         //设置世界位姿
        virtual void setWorldPos(const cv::Mat &Pos) = 0;
        //获取位姿
        virtual cv::Mat getWorldPos() = 0;
        //获取序号
        virtual u64 index()const = 0;
        //观察点
        virtual int observations() = 0;
        //添加观察者 index(特征点序号)
        virtual void addObservation(IKeyFrame *frame,int index) = 0;
        //移除观察者
        virtual void rmObservation(IKeyFrame *frame) = 0;
        //获取观察帧列表
        virtual KeyFrameMap getObservations() = 0;
        //是否在帧中
        virtual bool isInFrame(IKeyFrame *pFrame) = 0;
        //获取点在帧中的序号
        virtual int getIndexInKeyFrame(IKeyFrame *pFrame) = 0;
        //设置坏点
        virtual void setBadFlag() = 0;
        //返回是否为坏点
        virtual bool isBad() = 0;
        //返回向量
        virtual cv::Mat normal() = 0;
    };
    //位姿节点
    class IPoseNode : public IBase
    {
    public:
         //获取位置(R|t 矩阵)
        virtual Mat getPose() = 0;
        //设置位置(R|t 矩阵)
        virtual void setPose(const cv::Mat &pose) = 0;
        //序号
        virtual u64 index()const = 0;
    };

    //帧对象
    class IFrame : public IPoseNode
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
        //获取中心点
        virtual const Mat& getCameraCenter()const = 0;
    };

    //关键帧
    class IKeyFrame : public IPoseNode
    {
    public:
        //强制类型转换
        virtual operator IFrame*()const = 0;
        //帧目标
        virtual TargetVector& getTargets() = 0;

        //获取旋转 平移分量
        virtual Mat getRotation() = 0;
        virtual Mat getTranslation() = 0;

        //更新下一帧
        virtual void updateNext(IKeyFrame *next) = 0;
        //更新上一帧
        virtual void updatePrev(IKeyFrame *pre) = 0;
        //获取到下一帧
        virtual IKeyFrame* getNext() = 0;
        //获取上一帧
        virtual IKeyFrame* getPrev() = 0;
        //是否为坏点
        virtual bool isBad() = 0;
        //设为坏帧
        virtual void setBadFlag() = 0;
           //添加地图点
        virtual void addMapPoint(IMapPoint *pt, int index) = 0;
        //是否已有对应地图点
        virtual bool hasMapPoint(int index) = 0;
        //移除地图点
        virtual void rmMapPoint(IMapPoint *pt) = 0;
        virtual void rmMapPoint(int index) = 0;
         //获取地图点
        virtual const MapPtVector& getPoints() = 0;
    };
    #define IFRAME(K) (static_cast<Position::IFrame*>(*(K)))
    //地图
    class IMap : public IBase
    {
    public:
        //创建关键帧
        virtual IKeyFrame* createKeyFrame(IFrame *frame) = 0;
        //创建地图点
        virtual IMapPoint* createMapPoint(const cv::Mat &pose) = 0;
        virtual IMapPoint* createMapPoint(const cv::Point3f &pose) = 0;
        //加入/移除关键帧
        virtual void addKeyFrame(IKeyFrame *pKF) = 0;
        virtual void rmKeyFrame(IKeyFrame *pKF) = 0;
        //加入/移除地图点
        virtual void addMapPoint(IMapPoint *pMp) = 0;
        virtual void rmMapPoint(IMapPoint *pMp) = 0;
        //清空
        virtual void clear() = 0;

        //获取所有地图点
        virtual MapPtVector getAllMapPts() = 0;

        //获取所有帧
        virtual KeyFrameVector getAllFrames() = 0;

        //设最近点关联地图点
        virtual void setReferenceMapPoints(const MapPtVector &vpMPs) = 0;
        //获取最近帧关联点
        virtual MapPtVector getReferenceMapPoints() = 0;
        //最大帧号
        virtual u64 getMaxKFid() = 0;

        //获取帧，点计数
        virtual u64 frameCount() = 0;
        virtual u64 mapptCount() = 0;

        //用于多线程 地图更新锁
        virtual std::mutex& mapUpdateMutex() = 0;

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
        virtual const CameraParam& getCamera(int index = 0)const = 0;

        //根据图像名取时间(天秒)
        virtual double getTimeFromName(const std::string &name) = 0;
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
        //初始化
        virtual bool init() = 0;
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

    //跟踪状态
    enum eTrackStatus
    {   
        eTrackPrepare,
        eTrackReady,
        eTrackOk,
        eTrackLost
    };

    //帧跟踪接口
    class ITrajProcesser : public IBase
    {
    public:
        //处理
        virtual bool process(const FrameDataVector &framedatas) = 0;
        //跟踪
        virtual cv::Mat track(const FrameData &data) = 0;
        //状态
        virtual eTrackStatus status()const = 0;
        //重置
        virtual void reset() = 0;
        //当前帧
        virtual IKeyFrame* current()const = 0;
        //上一帧
        virtual IKeyFrame* last()const = 0;
    };

     //优化基类
    class IOptimizer : public IBase
    {
    public:
        //单例
        static IOptimizer* getSingleton();
        
        //单张位姿优化
        virtual int frameOptimization(IKeyFrame *pFrame, const FloatVector &sigma2) = 0;
        
        //ba 优化
        virtual void bundleAdjustment(const KeyFrameVector &keyframes,const MapPtVector &mappts, const FloatVector &sigma2,int nIterations = 5,
                                        bool *pbStopFlag = NULL,int nIndex = 0,bool bRobust = true) = 0;

        //设置相机参数
        virtual void setCamera(const CameraParam &mCam) = 0;

        //设置特征提取类
        virtual void setFeature(const std::shared_ptr<IFeature> &feature) = 0;
    };

    //块匹配接口
    class IBlockMatcher : public IBase
    {
    public:
         /*
         * 块评分
         * @param cur 当前帧
         * @param pt  像素点
         * @return    评分
         */
        virtual double score(const Mat &cur,const Point2f &pt) = 0;
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
        virtual bool estimate(cv::Mat &R, cv::Mat &t, MatchVector &matches, Pt3Vector &vPts) = 0;
    };

    //定位算法
    class IPositioning : public IBase
    {
    public:
        //定位关键帧中目标
        virtual void position(IKeyFrame *frame) = 0;
        //定位关键点
        virtual void position(IKeyFrame *frame, const Point2f &prept) = 0;
        //获取极线
        virtual EpLine computeEpLine(const cv::Mat &R, const cv::Mat &t,const cv::Point2f &pt) = 0;
        //极线匹配(基于目标包围盒)
        virtual TargetData eplineMatch(const EpLine &epline,const TargetData &item, const TargetVector &targets) = 0;
        //极线匹配(基于块)
        virtual cv::Point2f eplineMatch(const EpLine &epline, const FrameData &preframe, const FrameData &curframe,const cv::Point2f &pt) = 0;
        //反投
        virtual cv::Point2f backProject(const FrameData &frame,const BLHCoordinate &blh, cv::Mat &outimg) = 0;

    };
} // namespace Position

#endif