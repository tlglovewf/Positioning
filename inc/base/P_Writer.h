/**
 *   P_Writer.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PWRITER_H_H_
#define __PWRITER_H_H_


#include "P_Interface.h"
#include <fstream>
#include <iostream>
#include "log4cpp/Category.hh"

namespace Position
{
    //全局字符输出类
    class PStaticWriter
    {
    public:
        /* write real trace
        */
        static inline bool WriteRealTrace(ofstream &os, const BLHCoordinate &blh, const std::string &filename)
        {
            os.precision(15);
            os << blh.lat << "," << blh.lon << "," << filename.c_str() << endl;
            return true;
        }

        /* write estimate trace
        */
        static inline bool WriteEstTrace(ofstream &os, const BLHCoordinate &blh, const cv::Point3d &res, const std::string &filename)
        {
            os.precision(15);
            os << blh.lat << "," << blh.lon << "," << res.x << "," << res.y << "," << res.z << "," << filename.c_str() << endl;
            return true;
        }


        /* print console info.
        */
        static void Promt(const std::string &str)
        {
            cout << str.c_str() << endl;
        }
        template<typename... Args>
        static void Promt(const std::string &str,Args ...args)
        {
            cout << str.c_str() << " : ";
            promt_vs(args...);
        }

        /* print console info.
        */
        static void PromtDebug(const std::string &str)
        {
            cout << str.c_str() << endl;
        }
        template<typename... Args>
        static void PromtDebug(const std::string &str,Args ...args)
        {
            cout << str.c_str() << " : ";
            promt_vs(args...);
        }

    protected:
        static void promt_vs()
        {
            cout << endl;
        }
        template<typename V,typename... Args>
        static void promt_vs(V v,Args ...args)
        {
            cout << v << " ";
            promt_vs(args...);
        }
    };

    // serialization interface
    class PMapSer : public ISerialization
    {
    public:
        //设置地图
        virtual void setMap(const std::shared_ptr<IMap> &pmap)
        {
            assert(pmap);
            mpMap = pmap;
        }
        //加载地图
        virtual void loadMap(const std::string &path)  
        {
            assert(NULL);
        }
        //保存地图
        virtual void saveMap(const std::string &path) 
        {
            assert(NULL);
        }
        //合并地图
        virtual void combineMap(const std::string &path1,const std::string &path2, const std::string &outpath) 
        {
            assert(NULL);
        }
    protected:
        std::shared_ptr<IMap>    mpMap;
    };

    class DefaultWRNode
    {
    public:
        //打开文件
        bool open(const std::string &path,std::ios::openmode type)
        {
            mfile.open(path, type);
            return mfile.is_open();
        }

    protected:
        //写项目
        void writeItem(const std::string &name, const cv::Mat &pose);

    protected:
        fstream mfile;
    };

    //轨迹持久化
    class PMapTraceSer : public PMapSer,public DefaultWRNode
    {
    public:
         //加载地图
        virtual void loadMap(const std::string &path);
        //保存地图
        virtual void saveMap(const std::string &path);
    };

    //地图点云持久化
    class PMapPointsSer : public PMapSer,public DefaultWRNode
    {
    public:
         //加载地图
        virtual void loadMap(const std::string &path);
        //保存地图
        virtual void saveMap(const std::string &path);
    };


    //log 打印类
    class PLogManager
    {
    public:
        //从log配置文件加载
        PLogManager(const std::string &settingPath);
        //从工程配置加载
        PLogManager(const shared_ptr<IConfig> &pcfg);
        ~PLogManager();
        //获取log 输出类, 默认就是root
        log4cpp::Category& instacne(const std::string&  str = "");
    public:
        static PLogManager* s_Mgr;
    };

} // namespace Position

//输出
#define PROMT_S(X)      Position::PStaticWriter::Promt(X);
#define PROMT_V(X,...)  Position::PStaticWriter::Promt(X,__VA_ARGS__);

//仅为调试
#define PROMTD_S(X)     Position::PStaticWriter::PromtDebug(X);
#define PROMTD_V(X,...) Position::PStaticWriter::PromtDebug(X,__VA_ARGS__);


#define LOG_INITIALIZE(F)  Position::PLogManager::s_Mgr = new Position::PLogManager(F);

#define LOG_PROMT()       (Position::PLogManager::s_Mgr->instacne()).

  

#define LOG_DEBUG(X)    LOG_PROMT()debug(X);
#define LOG_INFO(X)     LOG_PROMT()info (X);
#define LOG_WARNING(X)  LOG_PROMT()warn (X);
#define LOG_ERROR(X)    LOG_PROMT()error(X);
#define LOG_CRIT(X)     LOG_PROMT()crit (X);

#define LOG_DEBUG_F(F,...)    LOG_PROMT()debug(F,__VA_ARGS__);
#define LOG_INFO_F(F,...)     LOG_PROMT()info (F,__VA_ARGS__);
#define LOG_WARNING_F(F,...)  LOG_PROMT()warn (F,__VA_ARGS__);
#define LOG_ERROR_F(F,...)    LOG_PROMT()error(F,__VA_ARGS__);
#define LOG_CRIT_F(F,...)     LOG_PROMT()crit (F,__VA_ARGS__);

#endif