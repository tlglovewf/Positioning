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
    //维亚数据处理
    class WeiyaData : public IData
    {
    public:
        //预处理数据
        virtual void preprocess(const std::string &imgpath, const std::string &pstpath);

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

    protected:

        void clear()
        {
            mFrameDatas.clear();
        }

    protected:
        FrameVector mFrameDatas;
    };



    // add more ..
} // namespace Position

#endif