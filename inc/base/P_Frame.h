/**
 *   P_Frame.h
 *   
 *   add by tu li gen   2020.2.13
 * 
 */
#ifndef __PFRAME_H_H_
#define __PFRAME_H_H_
#include "P_Interface.h"

namespace Position
{
    class PMap;
    class PMapPoint;

    //帧对象
    class PFrame : public IFrame
    {
    public:
        friend class FrameGrid;
        //构造函数 retainimg(是否保存图片资源 默认释放)
        PFrame(const FrameData &data,std::shared_ptr<IFeature> pFeature,bool retainimg = false);
        
         //获取数据
        virtual FrameData getData()const 
        {
            return mData;
        }
         //获取关键点
        virtual const KeyPtVector& getKeys()const
        {
            return mKeypts;
        }
         //获取特征点数量
        virtual int getKeySize()const 
        {
            return mN;
        }
        //获取描述子
        virtual const Mat& getDescript()const 
        {
            return mDescript;
        }
        
        //获取位置(世界坐标)
        virtual const Mat& getPose()const 
        {
            return mPose;
        }
        //设置位置(世界坐标)
        virtual void setPose(const cv::Mat &pose) 
        {
            mPose = pose;
        }

        //获取地图点
        virtual const MapPtVector& getPoints() 
        {
            return mPts;
        }

        //帧序号
        virtual int index()const 
        {
            return mIndex;
        }
         //添加地图点
        virtual void addMapPoint( IMapPoint *pt, int index)
        {
            assert(index > -1 && index < mPts.size());
            mPts[index] = pt;
            mPts[index]->addObservation(this,index);
        }
        //是否有对应特特征点
        virtual bool hasMapPoint(int index) 
        {
            assert(index > -1 && index < mPts.size());
            return (NULL != mPts[index]);
        }
        //移除地图点
        virtual void rmMapPoint( IMapPoint *pt )
        {
           if(NULL == pt)
                return;
           pt->rmObservation(this);
           MapPtVIter it =  find(mPts.begin(),mPts.end(),pt);
           (*it) = NULL;
        }
        virtual void rmMapPoint(int index) 
        {
            assert(index > -1 && index < mPts.size());
            assert(mPts[index]);
            mPts[index]->rmObservation(this);
            mPts[index] = NULL;
        }

        //重置静态数据
        static void resetStaticParams()
        {
            s_nIndexCount = 0;
        }
        //是否为坏点
        virtual bool isBad()const
        {
            return mbBad;
        }
        //设为坏帧
        virtual void setBadFlag()
        {
            mbBad = true;
        }

    protected:
        int                         mN;
        bool                        mbBad;
        u64                         mIndex;


        FrameData                   mData;
        KeyPtVector                 mKeypts;
        MapPtVector                 mPts;
        Mat                         mDescript;
        Mat                         mPose;
        std::shared_ptr<IFeature>   mFeature;
        FloatVector                 mvInvLevelSigma2;
        vector<vector<SzVector> >   mGrid;

        static u64                  s_nIndexCount;

    private:
        DISABLEDCP(PFrame)
    };
    
      //关键帧
    class PKeyFrame : public IKeyFrame
    {
    public:
        //构造
        PKeyFrame(IFrame *pframe,PMap *pMap):mpFrame(pframe),mpMap(pMap){assert(pframe);}

        //设置位置
        virtual void setPose(const cv::Mat &pose)
        {
            mpFrame->setPose(pose);
        }
        //获取位置(世界坐标)
        virtual const Mat& getPose()const
        {
            return mpFrame->getPose();
        }
        //重载强转
        virtual operator IFrame*()const
        {
            return mpFrame;
        }
        //是否为坏点
        bool isBad()const
        {
            return mpFrame->isBad();
        }
        //设为坏帧
        void setBadFlag()
        {
            mpFrame->setBadFlag();
        }
        //序号
        int index()const
        {
            assert(mpFrame);
            return mpFrame->index();
        }

         //获取数据
        virtual FrameData getData()const 
        {
            return mpFrame->getData();
        }
        //获取关键点
        virtual const KeyPtVector& getKeys()const 
        {
            return mpFrame->getKeys();
        }
        //获取特征点数量
        virtual int getKeySize()const 
        {
            return mpFrame->getKeySize();
        }
        //获取描述子
        virtual const Mat& getDescript()const 
        {
            return mpFrame->getDescript();
        }
        //获取地图点
        virtual const MapPtVector& getPoints() 
        {
            return mpFrame->getPoints();
        }
        //添加地图点
        virtual void addMapPoint(IMapPoint *pt, int index) 
        {
            mpFrame->addMapPoint(pt,index);
        }
        //是否已有对应地图点
        virtual bool hasMapPoint(int index) 
        {
           return mpFrame->hasMapPoint(index);
        }
        //移除地图点
        virtual void rmMapPoint(IMapPoint *pt) 
        {
            mpFrame->rmMapPoint(pt);
        }
        virtual void rmMapPoint(int index) 
        {
            mpFrame->rmMapPoint(index);
        }


    protected:
        IFrame *mpFrame;
        PMap   *mpMap;

    private:
        DISABLEDCP(PKeyFrame)
    };

#define FRAMEPT(K)（(IFrame*)*K)

    //frame grid 划分
    class FrameGrid
    {
    public:
        //根据坐标 查询特征点序号
        static SzVector getFrameFeaturesInArea(IFrame *pframe,
                                               float x,float y,float r,
                                               int minLevel,int maxLevel);

        //初始化配置参数
        static void initParams(float width, float height);

        //将特征点分grid
        static void assignFeaturesToGrid(IFrame *frame);

            //判斷關鍵點在哪個grid
        static inline bool PosInGrid( const cv::KeyPoint &kp, int &posX, int &posY)
        {
            posX = round((kp.pt.x) * mfGridElementWidthInv);
            posY = round((kp.pt.y) * mfGridElementWidthInv);

            if( posX < 0 || posX >= FRAME_GRID_COLS ||
                posY < 0 || posY >= FRAME_GRID_ROWS)
                return false;
            else
                return true;
        }
    private:

        static int  FRAME_GRID_ROWS; 
        static int  FRAME_GRID_COLS;
    
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;
    };     
}

#endif