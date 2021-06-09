/**
 *   P_GpsFusion.h
 *   
 *   add by tu ligen   2020.4.14
 * 
 */

#include "P_Interface.h"

namespace Position
{
    //gps 融合
    class GpsFunsion : public IGpsFusion
    {
    public:
         //融合
        virtual bool fuse(const std::shared_ptr<IMap> &pmap, const CameraParam &cam);
    };
}