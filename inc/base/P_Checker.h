/**
 *   P_Checker.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PCHECKER_H_H__
#define __PCHECKER_H_H__
#include "P_Interface.h"

namespace Position
{
    //检查基类
    class PChecker : public IChecker
    {
    public:
        //检查
        virtual bool check(const FrameData &frame) ;
        virtual bool check(const std::string &str) ;
    };
    //路径有效性检测
    class PathChecker
    {
    public:
        static bool check(const std::string &path) ;
    };
    
#define PATHCHECK(path) PathChecker::check(path)



    /*
     *  结果位置检查
     */
    class ResultPosCheck : public IResultCheck
    {
    public:
        /*
         * 检查结果
         * @param item  跟踪对象
         * @param index 关联帧序号
         * @param blh   检查的结果 
         */
        virtual bool check(const TrackerItem &item,int index, const BLHCoordinate &blh);
    };


    /*
     * 结果检查策略
     */
    class ResultCheckStrategy
    {
    public:
        ResultCheckStrategy();

        static ResultCheckStrategy* Instance()
        {
            static ResultCheckStrategy inst;
            return &inst;
        }

        /*
         * 添加检测策略
         */
        void addStrategy(const shared_ptr<IResultCheck> &rstcheck)
        {
            mRstChks.emplace_back(rstcheck);
        }

        /*
         * 检查结果
         * @param item  跟踪对象
         * @param index 关联帧序号
         * @param blh   检查的结果 
         */
        bool check(const TrackerItem &item,int index, const BLHCoordinate &blh);
    protected:
        std::vector<shared_ptr<IResultCheck> > mRstChks;
    };
}

#endif