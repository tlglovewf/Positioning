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
    //路径有效性检测
    class PathChecker
    {
    public:
        static bool check(const std::string &path) ;
    };
    //文件权限查询
    class FilePermissionChecker
    {
    public:
        static bool checkRead (const std::string &path);
        static bool checkWrite(const std::string &path);
    };

#define PATHCHECK(path)  Position::PathChecker::check(path)
#define READCHECK(path)  Position::FilePermissionChecker::checkRead(path)
#define WRITECHECK(path) Position::FilePermissionChecker::checkWrite(path)


    /*
     *  帧检查
     */
    class FrameChecker : public IChecker<FrameData>
    {
     public:
        //检查
        virtual bool check(const FrameData &item);
    };


    //结果验证参数
    struct RstCheckParams
    {
        const TrackerItem &_itm;    //验证的目标
        int                _idx;    //验证的帧序号
        BLHCoordinate      _rst;    //待验证结果
        RstCheckParams(const TrackerItem &item,int idx, const BLHCoordinate &rst):_itm(item),_idx(idx),_rst(rst){}
    };
    /*
     *  结果位置检查
     */
    class ResultPosChecker : public IChecker<RstCheckParams>
    {
    public:

        /*
         * 检查结果
         * @param item  跟踪对象
         * @param index 关联帧序号
         * @param blh   检查的结果 
         */
        virtual bool check(const RstCheckParams &blh);
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
        void addStrategy(const shared_ptr<IChecker<RstCheckParams> > &rstcheck)
        {
            mRstChks.emplace_back(rstcheck);
        }

        /*
         * 检查结果
         * @param item  跟踪对象
         * @param index 关联帧序号
         * @param blh   检查的结果 
         */
        bool check(const RstCheckParams &params);
    protected:
        std::vector<shared_ptr<IChecker<RstCheckParams> > > mRstChks;
    };
}

#endif