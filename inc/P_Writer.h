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
class P_Writer
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
    template<typename T>
    static void Promt(const std::string &str,T t)
    {
        cout << str.c_str() << " : " << t << endl;
    }
};
} // namespace Position

#define PROMT_S(X)   Position::P_Writer::Promt(X);
#define PROMT_V(X,V) Position::P_Writer::Promt(X,V);

#endif