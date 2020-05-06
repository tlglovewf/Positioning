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
        PData(const std::shared_ptr<IConfig> &pcfg):mpCfg(pcfg)
        {
            //add more
        }
        //预处理数据
        virtual bool loadDatas()
        {
            assert(NULL);
            return false;
        }

        //第一个元素
        virtual FrameDataVIter begin() 
        {
            return mFrameDatas.begin();
        }

        //最后一个元素
        virtual FrameDataVIter end() 
        {
            return mFrameDatas.end();
        }
        //数据总量
        virtual size_t size()const 
        {
            return mFrameDatas.size();
        }
        // 获取相机参数 default(0)  left    1 right 
        virtual const CameraParam& getCamera(int index = 0)const
        {
            return mCamera;
        }

        //根据图像名取时间(天秒)
        virtual double getTimeFromName(const std::string &name) 
        {
            assert(NULL);
            return 0.0;
        }

    protected:
        //清理
        void clear()
        {
            mFrameDatas.clear();
        }

    protected:
        std::shared_ptr<IConfig>    mpCfg;

        FrameDataVector             mFrameDatas;
        CameraParam                 mCamera;
    };
    // add more ..
} // namespace Position

#endif