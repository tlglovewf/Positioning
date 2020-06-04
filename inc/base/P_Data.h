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
    class PFrameData : public IFrameData
    {
    public:
        PFrameData(const std::shared_ptr<IConfig> &pcfg):mpCfg(pcfg)
        {
            //add more
        }

        ~PFrameData()
        {
           clear();
        }
        //预处理数据
        virtual bool loadDatas()
        {
            assert(NULL);
            return false;
        }

        //第一个元素
        virtual FrameDataPtrVIter begin() 
        {
            return mFrameDatas.begin();
        }

        //最后一个元素
        virtual FrameDataPtrVIter end() 
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
            for(size_t i = 0; i < mFrameDatas.size(); ++i)
            {
                delete mFrameDatas[i];
            }
            mFrameDatas.shrink_to_fit();
        }

    protected:
        std::shared_ptr<IConfig>    mpCfg;

        FrameDataPtrVector          mFrameDatas;
        CameraParam                 mCamera;
    };
    // add more ..
} // namespace Position

#endif