#include "P_Checker.h"
#include "P_Writer.h"
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
}