#ifndef _ORBKEYFRAMEDATABASE_H_
#define _ORBKEYFRAMEDATABASE_H_

#include <vector>
#include <list>
#include <set>

#include "P_ORBKeyFrame.h"
#include "P_ORBFrame.h"
#include "P_ORBVocabulary.h"

#include<mutex>


namespace Position
{

    class ORBKeyFrameDatabase
    {
    public:

      ORBKeyFrameDatabase( const std::shared_ptr<ORBVocabulary>& pvoc);

      void add(ORBKeyFrame* pKF);

      void erase(ORBKeyFrame* pKF);

      void clear();

      // Loop Detection
      std::vector<ORBKeyFrame *> DetectLoopCandidates(ORBKeyFrame* pKF, float minScore);

      // Relocalization
      std::vector<ORBKeyFrame*> DetectRelocalizationCandidates(ORBFrame* F);

    protected:

      // Associated vocabulary
      std::shared_ptr<ORBVocabulary> mpVoc;

      // Inverted file
      std::vector<list<ORBKeyFrame*> > mvInvertedFile;

      // Mutex
      std::mutex mMutex;
    };

}

#endif //_ORBKEYFRAMEDATABASE_H_
