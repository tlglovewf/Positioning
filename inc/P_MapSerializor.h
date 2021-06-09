/**
 *   P_MapSerializor.h
 *   
 *   add by tu li gen   2020.2.15
 * 
 */
#ifndef __MAPSERIALIZOR_H_H_
#define __MAPSERIALIZOR_H_H_
#include "P_Interface.h"
#include "P_IOHelper.h"
namespace  Position
{
      /*
     *  持久化 轨迹枚举
     */
    enum eMapTraceSerType
    {
        eDefaultTraceSer
    };

    /*
    *  持久化 地图点
    */
    enum eMapPointSerType
    {
        eDefaultPtSer
    };

     /*
     *  持久化 地图管理类
     */
    class MapSerManager
    {
    public:
        MapSerManager();
        //! 单例
        static MapSerManager* Instance()
        {
            static MapSerManager mgr;
            return &mgr;
        }

        //! 设置地图
        void setMap(const std::shared_ptr<Position::IMap> &pmap)
        {
            mpTracSer->setMap(pmap);
            mpPtSer->setMap(pmap);
        }

        //! 设置持久化类型
        void SetSerType(eMapTraceSerType trtype, eMapPointSerType epttype);

        //! 获取轨迹持久化指针
        ISerialization* tracSerPtr()
        {
            return  mpTracSer;
        }
        //! 获取地图点持久化指针
        ISerialization* mpPtSerPtr()
        {
            return mpPtSer;
        }

        //! 显示地图
        void displayMap(const std::shared_ptr<IConfig> &pCfg, const std::string &trac,const std::string &mpts);

        //! 融合地图  secMap -> baseMap
        void combineMap(std::shared_ptr<IMap> &baseMap, const std::shared_ptr<IMap> &secMap);
    protected:
        ISerialization *mpTracSer;
        ISerialization *mpPtSer;
    };

#define   SETMAPSERIALIZATIONMAP(MAP)  Position::MapSerManager::Instance()->setMap(MAP);
#define   SAVEMAPFRAME(path)           Position::MapSerManager::Instance()->tracSerPtr()->saveMap(path);
#define   SAVEMAPPOINTS(path)          Position::MapSerManager::Instance()->mpPtSerPtr()->saveMap(path);
#define   LOADMAPFRAME(path)           Position::MapSerManager::Instance()->tracSerPtr()->loadMap(path);
#define   LOADMAPPOINTS(path)          Position::MapSerManager::Instance()->mpPtSerPtr()->loadMap(path);
#define   DISPLAYMAP(CFG,FMS,MPTS)     Position::MapSerManager::Instance()->displayMap(CFG,FMS,MPTS);
} // namespace  Position

#endif