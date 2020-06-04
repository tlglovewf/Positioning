#include "P_Filter.h"
namespace Position
{
    void FrameDataFilter::doFilter(FrameDataPtrVector &t)
    {
        FrameDataPtrVector temp;
        size_t len = t.size() - 1;
        if(len < 1)
        {
            return;
        }
        FrameData* pLast = t[0];
        FrameDataPtrVIter it = t.begin() + 1;
        FrameDataPtrVIter ed = t.end();

        for(;it != ed; )
        {
            FrameData *pCur = *it;
           
            double dtlon = fabs( pCur->_pos.pos.lon - pLast->_pos.pos.lon);
            double dtlat = fabs( pCur->_pos.pos.lat - pLast->_pos.pos.lat);
            double total = dtlon + dtlat;

            //小数点后5位为米,
            if(total < 1.5e-5)
            {//不足距离的帧剔除
                it = t.erase(it);
                delete pCur;
            }
            else
            {
                ++it;
            }
        }
    }
}