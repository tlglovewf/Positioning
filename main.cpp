#include "P_Controller.h"
#include "P_Factory.h"
#include "P_Detector.h"
#include "P_Writer.h"

#include "P_MultiVisionTrajProcesser.h"

#include "project/hdproject.h"
#include "project/weiyaproject.h"


#include "P_Map.h"
#include "P_Factory.h"

using namespace std;
using namespace cv;

#define WEIYA           0  //是否为weiya数据
#define USECONTROLLER   1  //是否启用定位框架



//动态物体
class DynamicObjectInfor
{
public:
    typedef map<std::string, cv::Vec3b> Item;
    typedef Item::const_iterator        ItemIter;
    //单例
    static DynamicObjectInfor* Instance()
    {
        static DynamicObjectInfor instance;
        return &instance;
    }
    //加载
    void load(const std::string &path)
    {
        if(path.empty())
        {
            PROMT_S("Path error~  Semantics Config File Loaded failed.")
        }
        try
        {
            ifstream segfile;
            segfile.open(path);

            if(segfile.is_open())
            {
                PROMT_S("Begin to load semantics");
                while(!segfile.eof())
                {
                    std::string str;
                    getline(segfile,str);
                    trimString(str);//去首尾空格
                    if(str.empty() || str[0] == '#')
                    {
                        continue;
                    }
                    else
                    {
                        int s = str.find_first_of(":");
                        int v = str.find_first_of("#");//剔除注释
                        string name = str.substr(0,s);
                        string result = str.substr(s+1,(v - s)-1);
                        trimString(result);
                        int r,g,b;
                        sscanf( result.c_str(), "%d, %d, %d",&b,&g,&r);
                        cv::Vec3b vv(r,g,b);
                        mObjs.insert(std::make_pair(name,vv));
                        PROMT_V(name.c_str(),vv);
                    }
                }
            }
            segfile.close();
            PROMT_S("End load semantics");
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    // //
    // void rmFeatureByDynamicObj(const Mat &segImg,Position::MatchVector &matches)
    // {

    // }

    // bool isFeatureDynamicObj(IFrame *pframe, const Point2f &pt)
    // {

    // }

    //[] 运算符
    cv::Vec3b operator[](const std::string &name)const
    {
        ItemIter it = mObjs.find(name);
        if(it != mObjs.end())
        {
            return it->second;
        }
        else
        {
            return cv::Vec3b();
        }
    }

    //开始
    ItemIter begin()const
    {
        return mObjs.begin();
    }

    //结束
    ItemIter end()const
    {
        return mObjs.end();
    }

protected:
    //剔除前后空格
    void trimString(std::string & str )
    {
        if(str.empty())
            return;
        int s = str.find_first_not_of(" ");
        int e = str.find_last_not_of(" ");

        if( s == string::npos || 
            e == string::npos)
            return;

        str = str.substr(s,e-s+1);
    }
    
protected:
    map<std::string, cv::Vec3b> mObjs;
};


void MapDisplay(const std::shared_ptr<Position::IConfig> &pCfg)
{
    string outpath = GETCFGVALUE(pCfg,OutPath,string) + "/";

    Position::MapSerManager::Instance()->displayMap(pCfg,outpath + "trac.txt",outpath + "mpts.txt");
}



int main(void)
{  

#if WEIYA
    std::shared_ptr<Position::IConfig> pCfg = std::make_shared<WeiyaConfig>("../config_weiya.yaml"); 
    std::shared_ptr<Position::IData> pData(new WeiyaData(pCfg));
#else
    std::shared_ptr<Position::IConfig> pCfg = std::make_shared<HdConfig>("../config_hd.yaml"); 
    std::shared_ptr<Position::IData> pData(new HdData(pCfg));
#endif

    std::shared_ptr<Position::IDetector> pdetecter = std::make_shared<Position::SSDDetector >();

    MapDisplay(pCfg);

    return 0;
#if USECONTROLLER
    std::unique_ptr<PositionController> system(new PositionController(pdetecter,pData,pCfg));

    system->run();

    return 0;
#else
    //multi vision situation test 
    pData->loadDatas();
    const string imgpath = GETCFGVALUE(pCfg,ImgPath ,string) + "/";
    const string outpath = GETCFGVALUE(pCfg,OutPath ,string) + "/";
    std::shared_ptr<Position::ITrajProcesser> pTraj(Position::PFactory::CreateTrajProcesser(Position::eMultiVision,pCfg,pData));
    std::shared_ptr<Position::IMap> map = pTraj->getMap();
    Position::FrameDataVector datas;
    Position::FrameDataVIter iter = pData->begin();
    Position::FrameDataVIter ed   = pData->end();
    //插入帧数据
    for(;iter != ed ; ++iter)
    {
        Mat img = imread(imgpath + iter->_name ,IMREAD_UNCHANGED);
        if(img.empty())
            continue;
        iter->_img = img;
        datas.push_back(*iter);
    }
    //处理帧数据
    pTraj->process(datas);

    //可视化帧数据
    std::unique_ptr<Position::IViewer> pv(Position::PFactory::CreateViewer(Position::eVPangolin,pCfg));
    pv->setMap(map);
    pv->renderLoop();
#endif
    return 0;
}