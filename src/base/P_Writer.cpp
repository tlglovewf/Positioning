#include "P_Writer.h"
#include "P_Converter.h"
#include "P_Utils.h"
#include "P_Frame.h"

namespace Position
{
#define HEADTRACSTR "Frame Trace"
#define HEADMPSTR   "Map Points"

#define BEGINFILEREGION(PATH,MD)  try                               \
                                  {                                 \
                                     if(open(PATH,std::ios::MD))    \
                                     {

#define ENDFILEREGION()              }                              \
                                  }                                 \
                                  catch(const std::exception& e)    \
                                  {                                 \
                                      std::cerr << e.what() << '\n';\
                                  }


    void DefaultWRNode::writeItem(const std::string &name, const cv::Mat &pose)
    {
        std::string str = PConverter::toString(pose);
        if(!name.empty())
        {
            str = name + ":" + str;
        }
        mfile << str << endl;
    }

    //加载地图轨迹
    void PMapTraceSer::loadMap(const std::string &path) 
    {
        assert(mpMap);
        assert(!path.empty());
        BEGINFILEREGION(path,in)

        string head;
        getline(mfile,head);
        if(head == HEADTRACSTR)
        {
            while(!mfile.eof())
            {//遍历文件 创建关键帧
                string line;
                getline(mfile,line);
                if(line.empty())
                    continue;
                StringVector svs = PUtils::SplitString(line,":");
                assert(svs.size() == 2);
                FrameData data;
                data._name = svs[0];
                IFrame *pf = new PFrame(data);
                pf->setPose(PConverter::str2CVMat(svs[1]));
                mpMap->createKeyFrame(pf);
            }
        }
        else
        {
            PROMT_S("It's not a avilable trace file")
        }
        ENDFILEREGION()
    }

    //保存地图轨迹
    void PMapTraceSer::saveMap(const std::string &path) 
    {
        assert(mpMap);
        assert(!path.empty());
        BEGINFILEREGION(path,out)

        const KeyFrameVector &keyfms = mpMap->getAllFrames();
        if(keyfms.empty())
            return;
        //write head 
        mfile << HEADTRACSTR << endl;
        for(size_t i = 0; i < keyfms.size(); ++i)
        {
            const std::string &name = keyfms[i]->getData()._name;
            const Mat& pose = keyfms[i]->getPose();
            writeItem(name,pose);
        }

        ENDFILEREGION()
    }

    //合并地图
    void PMapTraceSer::combineMap(const std::string &path1,const std::string &path2, const std::string &outpath) 
    {
        assert(NULL);
    }

     //加载地图轨迹
    void PMapPointsSer::loadMap(const std::string &path) 
    {
        assert(mpMap);
        assert(!path.empty());
        BEGINFILEREGION(path,in)

        string head;
        getline(mfile,head);
        if(head == HEADMPSTR)
        {
            while(!mfile.eof())
            {//遍历文件 创建关键帧
                string line;
                getline(mfile,line);
                if(line.empty())
                    continue;

                cv::Mat pt = PConverter::str2CVMat(line,true);
                if(pt.empty())
                    continue;
                mpMap->createMapPoint(pt);
            }
        }
        else
        {
            PROMT_S("It's not a avilable mappoints file")
        }
        

        ENDFILEREGION()
    }

    //保存地图轨迹
    void PMapPointsSer::saveMap(const std::string &path) 
    {
        assert(mpMap);
        assert(!path.empty());
        BEGINFILEREGION(path,out)

        const MapPtVector  &pts = mpMap->getAllMapPts();
        if(pts.empty())
            return;
        //write head 
        mfile << HEADMPSTR << endl;
        for(size_t i = 0; i < pts.size(); ++i)
        {
            const Mat &pt = pts[i]->getWorldPos();
            writeItem("",pt);
        }

        ENDFILEREGION()
    }

    //合并地图
    void PMapPointsSer::combineMap(const std::string &path1,const std::string &path2, const std::string &outpath) 
    {
        assert(NULL);
    }
}