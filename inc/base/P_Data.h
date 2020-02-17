/**
 *   P_Data.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PDATA_H_H_
#define __PDATA_H_H_
#include "P_Interface.h"

namespace Position
{

    //数据对象
    class PData : public IData
    {
    public:
        PData(std::shared_ptr<IConfig> pcfg);
        //预处理数据
        virtual bool loadDatas()
        {
            assert(NULL);
        }

        //第一个元素
        virtual FrameVIter begin() 
        {
            return mFrameDatas.begin();
        }

        //最后一个元素
        virtual FrameVIter end() 
        {
            return mFrameDatas.end();
        }

        // 获取相机参数 default(0)  left    1 right 
        virtual CameraParam getCamera(int index = 0) 
        {
            return mCamera;
        }
    protected:
        //清理
        void clear()
        {
            mFrameDatas.clear();
        }

    protected:
        std::shared_ptr<IConfig>    mpCfg;
        std::unique_ptr<IChecker>   mpChecker;

        FrameVector                 mFrameDatas;
        CameraParam                 mCamera;
    };

    //维亚数据处理
    class WeiyaData : public PData
    {
    public:
        WeiyaData(std::shared_ptr<IConfig> pcfg);
        //预处理数据
        virtual bool loadDatas();

    protected:
    };



    // add more ..
} // namespace Position

#endif