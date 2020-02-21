
/**
 *   P_Optimizer.h
 *   
 *   add by tu li gen   2020.2.19
 * 
 */
#ifndef __POPTIMIZER_H_H_
#define __POPTIMIZER_H_H_
#include "P_Interface.h"
#include "P_Map.h"
namespace POSITION
{
    //优化基类
    class IOptimizer : public IBase
    {
    public:
    
    protected:
    };

    //g2o 优化类
    class G2oOptimizer : public IOptimizer
    {
    public:


    };

}

#endif