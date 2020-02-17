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
    //帧对象
    class PFrame : public IFrame
    {
    public:
        friend class FrameGrid;
        //构造函数
        PFrame(const FrameData &data,std::shared_ptr<IFeature> pFeature);
        
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
        
    protected:
        int                         mN;
        FrameData                   mData;
        KeyPtVector                 mKeypts;
        Mat                         mDescript;
        std::shared_ptr<IFeature>   mFeature;
        vector<vector<SzVector> >   mGrid;
    };

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