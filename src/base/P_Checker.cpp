#include "P_Checker.h"
#include "P_Writer.h"
#include "P_Utils.h"
namespace Position
{
    bool PChecker::check(const FrameData &frame)
    {
        if(frame._pos._t < 0     ||
           frame._pos.pos.lon < 0|| 
           frame._img.empty())
           {
               PROMT_S("Frame error !");
               PROMT_S(frame._name);
               return false; 
           }
           else
           {
               return true;
           }
    }
    bool PChecker::check(const std::string &str)
    {
        return true;
    }

    bool PathChecker::check(const std::string &str)
    {
        //add more 
        return true;
    }


    bool ResultPosCheck::check(const TrackerItem &item,int index,const BLHCoordinate &blh)
    {
        assert(item.batch);
        assert(BLHCoordinate::isValid(item.blh));
        assert(item.batch->_fmsdata.size() > index);

        double error1 = cv::norm(PUtils::CalcGaussErr(item.blh,blh));
        double error2 = cv::norm(PUtils::CalcGaussErr(item.batch->_fmsdata[index]->_pos.pos,blh));
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
    }


    ResultCheckStrategy::ResultCheckStrategy()
    {
        mRstChks.emplace_back(new ResultPosCheck());
    }

    /*
     * 检查
     */
    bool ResultCheckStrategy::check(const TrackerItem &item,int index, const BLHCoordinate &blh)
    {
        //没有检查实例时,默认永真
        if(mRstChks.empty())
            return true;

        for(auto rst : mRstChks)
        {//遍历检查实例
            if(!rst->check(item,index,blh))
            {
                return false;
            }
        }

        return true;
    }
}