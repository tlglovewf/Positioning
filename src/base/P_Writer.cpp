#include "P_Writer.h"
namespace Position
{
    //设置地图
    void MapDefaultSer::setMap(const std::shared_ptr<IMap> &pmap) 
    {
        mpMap = pmap;
    }

    //加载地图
    void MapDefaultSer::loadMap(const std::string &path, bool frame /* = true */,bool mpts /* = true */) 
    {
        if( mpMap && !path.empty() )
        {
            if(frame)
            {

            }

            if(mpts)
            {

            }
        }
    }

    //保存地图
    void MapDefaultSer::saveMap(const std::string &path,bool frame /* = true */,bool mpts /* = true */) 
    {
        if( mpMap && !path.empty() )
        {
            if(frame)
            {

            }

            if(mpts)
            {
                
            }
        }
    }
}