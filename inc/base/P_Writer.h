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
class PWriter
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
} // namespace Position

//输出
#define PROMT_S(X)   Position::PWriter::Promt(X);
#define PROMT_V(X,...) Position::PWriter::Promt(X,__VA_ARGS__);

//仅为调试
#define PROMTD_S(X)   Position::PWriter::PromtDebug(X);
#define PROMTD_V(X,...) Position::PWriter::PromtDebug(X,__VA_ARGS__);

#endif