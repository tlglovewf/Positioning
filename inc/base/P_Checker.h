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

}

#endif