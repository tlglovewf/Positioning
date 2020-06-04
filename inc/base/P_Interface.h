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
#define SETCFGVALUE(v,str,d)    (GETCFGVALUE(v,str,decltype(d))) = d
    // config interface
    class IConfig : public IBase
    {
    public:
        // 加载配置文件
        virtual void load(const std::string &path) = 0;

        // 获取值
        virtual IConfigParam* operator[](const std::string &name) = 0;
        
        //获取单例
        static std::shared_ptr<IConfig>& Instance();

        //设置单例
        static void SetInstance(const std::shared_ptr<IConfig> &pinstance);
    };

#define SETGLOBALCONFIG(X) Position::IConfig::SetInstance(X);
#define GETGLOBALCONFIG()  Position::IConfig::Instance()

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
        //观察帧数量
        virtual int observations() = 0;
        //添加观察者 index(特征点序号)
        virtual void addObservation(IKeyFrame *frame,int index) = 0;
        //移除观察者
        virtual void rmObservation(IKeyFrame *frame) = 0;
        //获取观察帧列表
        virtual const KeyFrameMap& getObservations() = 0;
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
        virtual FrameData* getData()const = 0;
        //获取关键点
        virtual const KeyPtVector& getKeys()const = 0;
        //获取外点
        virtual u8& outlier(int index)  = 0;
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

         //获取关键点
        virtual const KeyPtVector& getKeys()const = 0;

        //获取旋转 平移分量
        virtual Mat getRotation() = 0;
        virtual Mat getTranslation() = 0;
        //获取光心位置
        virtual const Mat& getCameraCenter() = 0;
        //获取数据
        virtual FrameData* getData()const = 0;
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
        virtual const MapPtVector& getWorldPoints() = 0;
    };
    #define IFRAME(K) (static_cast<Position::IFrame*>(*(K)))
    //地图
    class IMap : public IBase
    {
    public:
        //sort
        static void SortFrames(KeyFrameVector &frames); 
        static IKeyFrame* CreateKeyFrame(const std::shared_ptr<IMap> &pmap, FrameData *data, const Mat &pose);

        //创建关键帧
        virtual IKeyFrame* createKeyFrame(IFrame *frame) = 0;
        //创建地图点
        virtual IMapPoint* createMapPoint(const cv::Mat &pose)     = 0;
        virtual IMapPoint* createMapPoint(const cv::Point3f &pose) = 0;

        //加入/移除关键帧
        virtual void addKeyFrame(IKeyFrame *pKF) = 0;
        virtual void rmKeyFrame(IKeyFrame *pKF)  = 0;

        //加入/移除地图点
        virtual void addMapPoint(IMapPoint *pMp) = 0;
        virtual void rmMapPoint(IMapPoint *pMp)  = 0;

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

        //获取点、帧数量
        virtual u64 mapPointsInMap() = 0;
        virtual u64 keyFrameInMap() = 0;

        //用于多线程 地图更新锁
        virtual std::mutex& mapUpdateMutex() = 0;

        //当前帧
        virtual IKeyFrame* currentKeyFrame() = 0;
    };

    
    // data interface
    class IFrameData : public IBase
    {
    public:
        //预处理数据        
        virtual bool loadDatas() = 0;

        //第一个元素
        virtual FrameDataPtrVIter begin() = 0;

        //最后一个元素
        virtual FrameDataPtrVIter end() = 0;

        //数据总量
        virtual size_t size()const = 0;

        // 获取相机参数 default(0)  left    1 right 
        virtual const CameraParam& getCamera(int index = 0)const = 0;

        //根据图像名取时间(天秒)
        virtual double getTimeFromName(const std::string &name) = 0;
    };

    //database interface
    class IDatabase : public IBase
    {
    public:
        //打开数据库
        virtual bool open(const std::string &path) = 0;
        //关闭数据库
        virtual void close() = 0;
        //执行
        virtual bool exec(const std::string &str, char **&result, int &col, int &row) = 0;
        //释放资源
        virtual void releaseDatas(char **&result) = 0;
    };

    // serialization interface
    class ISerialization : public IBase
    {
    public:
        //设置地图
        virtual void setMap(const std::shared_ptr<IMap> &pmap) = 0;
        //加载地图
        virtual void loadMap(const std::string &path)  = 0;
        //保存地图
        virtual void saveMap(const std::string &path) = 0;
    };


    //帧批组生成器
    class IBatchesGenerator : public IBase
    {
    public:
        //生成批处理对象
        virtual PrjBatchVector generate(const std::shared_ptr<IFrameData> &pdata,TrackerItemVector &trackers) = 0;
    };


      //project batch file
    class IProjList : public IBase
    {
    public:
        //设置生成器
        virtual void setBatcherGenerator(const std::shared_ptr<IBatchesGenerator> &pGtor) = 0;

        //加载项目列表
        virtual void loadPrjList(const std::string &path) = 0;

        //获取项目列表
        virtual PrjBatchVector& getPrjList() = 0;

        //目标信息
        virtual TrackerItemVector& trackInfos() = 0;

        //从文件加载
        virtual void load(const std::string &path) = 0;

        //存储到文件
        virtual void save(const std::string &path) = 0;
    };

       //帧目标定位器
    class ITargetPositioner : public IBase
    {
    public:
        //定位 frame序列 必须是排序好的
        virtual bool position(KeyFrameVector &frame) = 0;       
    };

    //视觉定位器
    class IVisualPositioner : public IBase
    {
    public:
        //定位目标
        virtual bool position(TrackerItem &item) = 0;
    
    protected:
        //选择用于量测的帧
        virtual void selectFrame(const TrackerItem &item,int &idx1, int &idex2) = 0;
    };
    
    // visual interface
    class IViewer : public IBase
    {
    public:
        //设置显示地图
        virtual void setMap(const std::shared_ptr<IMap> &pMap) = 0;
        //初始化
        virtual void init() = 0;
        //绘制一次
        virtual bool renderOnce() = 0;
        //绘制循环
        virtual void renderLoop() = 0;
        //绘制状态
        virtual bool isRender()const = 0;
    };
    
    //检查
    template<typename TYPE>
    class IChecker : public IBase
    {
    public:
        //检查
        virtual bool check(const TYPE &item) = 0;
    };

    //过滤器
    template<typename TYPE>
    class IFilter : public IBase
    {
    public:
        //过滤
        virtual void doFilter(TYPE &t) = 0;
    };

    //特征接口
    class IFeature : public IBase
    {
    public:
        //计算特征点
        virtual bool detect(const FrameData &frame,KeyPtVector &keys, Mat &descript) = 0;
        //获取名称
        virtual std::string name()const = 0;
        //返回sigma参数(主要用于优化 信息矩阵)
        virtual const FloatVector& getSigma2() const = 0;
    };

    //特征跟踪接口
    class IFeatureMatcher : public IBase
    {
    public:
        
        //匹配  返回匹配对
        //@param preframe   前帧
        //@param curframe   后帧
        //@param windowsize 搜索过滤范围(像素)
        virtual MatchVector match(IFrame *preframe, IFrame *curframe, int windowsize) = 0;

        //获取名称
        virtual std::string name()const = 0;
    };

    //跟踪状态
    enum eTrackStatus
    {   
        eTrackNoImage = 0,
        eTrackNoReady    ,
        eTrackOk         ,
        eTrackLost
    };

    //帧跟踪接口
    class ITrajProcesser : public IBase
    {
    public:
        //设置可视接口
        virtual void setViewer(const std::shared_ptr<IViewer> &viewer) = 0;
        //获取地图
        virtual const std::shared_ptr<IMap>& getMap() = 0;
        //处理
        virtual bool process(const FrameDataPtrVector &framedatas) = 0;
        //跟踪
        virtual cv::Mat track(FrameData *data) = 0;
        //状态
        virtual eTrackStatus status()const = 0;
        //重置
        virtual void reset() = 0;
        //结束
        virtual void over() = 0;
        //等待处理
        virtual void wait() = 0;
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
    class IPoseSolver
    {
    public:
        //设置相机参数
        virtual void setCamera(const CameraParam &cam) = 0;
        //设置帧
        virtual void setFrames( IFrame *pre, IFrame *cur) = 0;
        //推算位姿
        virtual bool estimate(cv::Mat &R, cv::Mat &t, MatchVector &matches, Pt3Vector &vPts) = 0;
    };


    //gps融合
    class IGpsFusion : public IBase
    {
    public:
        //融合
        virtual bool fuse(const std::shared_ptr<IMap> &pmap, const CameraParam &cam) = 0;
    };

} // namespace Position

#endif