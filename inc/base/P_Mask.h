/**
 *   P_Mask.h
 *   
 *   add by tu li gen   2020.6.3
 *   图片掩码接口
 */
#ifndef __P_MASK_H_H_
#define __P_MASK_H_H_
// #include "P_SemanticGraph.h"
#include "P_IOHelper.h"

namespace Position
{
    //图像掩码
    class ImageMask
    {
    public:
        ImageMask() = default;
        ImageMask(const Size &size)
        {
            init(size);
        }
        //初始化
        void init(const Size &size)
        {
            mMask = cv::Mat::ones(size.height, size.width, CV_8UC1);
            mMask.setTo(255);
        }
        //运算符重载
        Mat operator()()
        {
            return mMask;
        }
        //通过语义图设置掩码
        void setMask(const string &sempath, const string &segim)
        {
            // assert(!mMask.empty());
            // string str = segim;
            // Position::PUtils::ReplaceFileSuffix(str, "jpg", SemanticGraph::Instance()->defaultsuffix);
            // cv::Mat seg = imread(sempath + str);
            // if (seg.empty())
            // {
            //     LOG_ERROR_F("%s SemanticGraph Gernator Failed..", (sempath + str).c_str());
            //     return;
            // }

            // if (seg.size() == mMask.size())
            // {
            //     LOG_ERROR_F("%s Size Not Same.", str.c_str());
            //     return;
            // }

            // for (size_t i = 0; i < seg.rows; i++)
            //     for (size_t j = 0; j < seg.cols; j++)
            //     {
            //         Point pt = Point(j, i);
            //         if (SemanticGraph::Instance()->isDynamicObj(pt, seg))
            //         {
            //             mMask.at<uchar>(pt) = 0;
            //         }
            //     }
        }
        //通过范围设置掩码
        void setMask(const Rect &rect)
        {
            assert(!mMask.empty());
            if ((mMask.size().width >= rect.width) &&
                (mMask.size().height >= rect.height))
            {
                mMask(rect).setTo(0);
            }
        }
        //获取全局掩码
        static ImageMask &GlobalMask() 
        {
            static ImageMask global;
            return global;
        }
        //检查
        bool check(const Point2f& pt)
        {
            return mMask.at<uchar>(pt) != 0;
        }
    protected:
        Mat mMask;
    };
} // namespace Position

#define INITGLOBALMASK(SZ) Position::ImageMask::GlobalMask().init(SZ);
#define CHECKMASK(PT)      Position::ImageMask::GlobalMask().check(PT)
#define SETGLOBALMASK(M)   Position::ImageMask::GlobalMask().setMask(M);

#endif