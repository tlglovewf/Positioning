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
        friend class FrameHelper;

        //构造函数 retainimg(是否保存图片资源 默认释放)
        PFrame(const FrameData &data,const std::shared_ptr<IFeature> &pFeature,int index, int cameraIndex = 0);
        ~PFrame();
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
        virtual Mat getPose() 
        {
            return mPose;
        }
        //设置位置(世界坐标)
        virtual void setPose(const cv::Mat &pose) 
        {
            mPose = pose;
            UpdatePoseMatrices();
        }

        //帧序号
        virtual u64 index()const 
        {
            return mIndex;
        }

        //获取中心点
        virtual const Mat& getCameraCenter()const 
        {
            return mOw;
        }
    protected:
         // Computes rotation, translation and camera center matrices from the camera pose.
        void UpdatePoseMatrices()
        {
            cv::Mat rot = mPose.rowRange(0,3).colRange(0,3);
            cv::Mat t   = mPose.rowRange(0,3).col(3);
            mOw = -rot.t() * t;
        }
    protected:
        int                         mN;
        int                         mCamIdx;
        u64                         mIndex;


        FrameData                   mData;
        KeyPtVector                 mKeypts;
        Mat                         mDescript;
        Mat                         mPose;
        Mat                         mOw;
        std::shared_ptr<IFeature>   mFeature;
        vector<vector<SzVector> >   mGrid;

    private:
        DISABLEDCP(PFrame)
    };
    
      //关键帧
    class PKeyFrame : public IKeyFrame
    {

    protected:
          //构造
        PKeyFrame(IFrame *pframe,IKeyFrame *prev,PMap *pMap);
        ~PKeyFrame();

    public:
      
        friend class PMap;

        //设置位置
        virtual void setPose(const cv::Mat &pose)
        {
            mpFrame->setPose(pose);
        }
        //获取位置(世界坐标)
        virtual Mat getPose()
        {
            return mpFrame->getPose();
        }
        //重载强转
        virtual operator IFrame*()const
        {
            return mpFrame;
        }
        //是否为坏点
        bool isBad()
        {
            return mbBad;
        }
        //设为坏帧
        void setBadFlag()
        {
            mbBad = true;
        }
        //序号
        u64 index()const
        {
            assert(mpFrame);
            return mpFrame->index();
        }

        //获取地图点
        virtual const MapPtVector& getPoints() 
        {
            return mPts;
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
         //帧目标
        virtual TargetVector& getTargets() 
        {
            return mTargets;
        }

         //更新下一帧
        virtual void updateNext(IKeyFrame *next) 
        {
            mpNext = next;
        }
        //更新上一帧
        virtual void updatePrev(IKeyFrame *pre) 
        {
            mpPre = pre;
        }
         //获取到下一帧
        virtual IKeyFrame* getNext() 
        {
            return mpNext;
        }

        //获取上一帧
        virtual IKeyFrame* getPrev() 
        {
            return mpPre;
        }
         //获取旋转 平移分量
        virtual Mat getRotation()
        {
            return mpFrame->getPose().rowRange(0,3).colRange(0,3);
        }
        virtual Mat getTranslation() 
        {
            return mpFrame->getPose().rowRange(0,3).col(3);
        }
    protected:
        IFrame      *mpFrame;
        IKeyFrame   *mpNext;
        IKeyFrame   *mpPre;
        PMap        *mpMap;
        bool        mbBad;

        MapPtVector                 mPts;
        TargetVector                mTargets;

    private:
        DISABLEDCP(PKeyFrame)
    };

    //frame 帮助类
    class FrameHelper
    {
    public:
        //根据坐标 查询特征点序号
        static SzVector getFrameFeaturesInArea(IFrame *pframe,
                                               float x,float y,float r,
                                               int minLevel,int maxLevel);

        //初始化配置参数
        static void initParams(float width, float height,CameraParam *pcam, int index = 0);

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

       //获取相机参数
       static void getCameraParams(double &fx,double &fy,double &cx, double &cy)
       {
           assert(mInit);
           fx = mFx;
           fy = mFy;
           cx = mCx;
           cy = mCy;
       }

    private:
        static bool mInit;

        static int  FRAME_GRID_ROWS; 
        static int  FRAME_GRID_COLS;
    
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;

         //相机内参
        static double mFx;
        static double mFy;
        static double mCx;
        static double mCy;

        DISABLEDC(FrameHelper)
    };     
}

#endif