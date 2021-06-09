#include "P_Checker.h"
#include "P_IOHelper.h"
#include "P_Utils.h"
// #if (defined __APPLE__) || (defined __unix__) || (defined LINUX)
// #include "dirent.h"
// #endif
#include <unistd.h>

namespace Position
{
    bool PathChecker::check(const std::string &str)
    {
        //add more
        if(str.empty() || 
           (access( str.c_str(), 0 ) == -1))
            return false;

        return true;
    }

        //文件权限查询
    bool FilePermissionChecker::checkRead (const std::string &path)
    {
        if(!PATHCHECK(path))
            return false;
        return access(path.c_str(),4) != -1;
    }
    bool FilePermissionChecker::checkWrite(const std::string &path)
    {
        if(!PATHCHECK(path))
            return false;
        return access(path.c_str(),2) != -1;
    }

    bool ResultPosChecker::check(const RstCheckParams &params)
    {
        assert(params._itm.batch);
        assert(BLHCoordinate::isValid(params._rst));
        assert(params._itm.batch->_fmsdata.size() > params._idx);

        double error1 = cv::norm(PUtils::CalcGaussErr(params._itm.blh,params._rst));
        double error2 = cv::norm(PUtils::CalcGaussErr(params._itm.batch->_fmsdata[params._idx]->_pos.pos,params._rst));
        //暂定  横纵绝对值之和 > 10m为计算无效
        const float er = 10.0;
        if( error1 > er ||
            error2 > er )
        {
            return false;
        }
        else
        {
            return true;
        }
        return true;
    }


    ResultCheckStrategy::ResultCheckStrategy()
    {
        mRstChks.emplace_back(new ResultPosChecker());
    }

    /*
     * 检查
     */
    bool ResultCheckStrategy::check(const RstCheckParams &params)
    {
        //没有检查实例时,默认永真
        if(mRstChks.empty())
            return true;

        for(auto rst : mRstChks)
        {//遍历检查实例
            if(!rst->check(params))
            {  
                return false;
            }
        }

        return true;
    }

    bool FrameChecker::check(const FrameData &item)
    {
        PROMTD_S("check frame");
    }


}