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
        //合并地图
        virtual void combineMap(const std::string &path1,const std::string &path2, const std::string &outpath);
    };

    //地图点云持久化
    class PMapPointsSer : public PMapSer,public DefaultWRNode
    {
    public:
         //加载地图
        virtual void loadMap(const std::string &path);
        //保存地图
        virtual void saveMap(const std::string &path);
        //合并地图
        virtual void combineMap(const std::string &path1,const std::string &path2, const std::string &outpath);
    };

} // namespace Position

//输出
#define PROMT_S(X)      Position::PStaticWriter::Promt(X);
#define PROMT_V(X,...)  Position::PStaticWriter::Promt(X,__VA_ARGS__);

//仅为调试
#define PROMTD_S(X)     Position::PStaticWriter::PromtDebug(X);
#define PROMTD_V(X,...) Position::PStaticWriter::PromtDebug(X,__VA_ARGS__);

#endif