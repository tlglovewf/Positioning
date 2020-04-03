#include "P_Controller.h"
#include "P_Factory.h"
#include "P_Detector.h"
#include "P_Writer.h"

#include "P_MultiVisionTrajProcesser.h"

#include "project/hdproject.h"
#include "project/weiyaproject.h"


#include "P_Map.h"
#include "P_Frame.h"

#include "P_Utils.h"
#include "P_Converter.h"
#include "fstream"



using namespace std;
using namespace cv;

#define SAVEMATCHIMG    1  //是否存储同名点匹配文件
#define WEIYA           0  //是否为weiya数据
#define USECONTROLLER   0  //是否启用定位框架



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


Position::IFrame* CreateFrame(const std::string &imgpath,const std::shared_ptr<Position::IFeature> &feature)
{
    cout << imgpath.c_str() << endl;
    if(imgpath.empty())
    {
        return NULL;
    }
    else
    {
        Position::FrameData data;
        data._name = imgpath;
        data._img  = imread(imgpath,IMREAD_UNCHANGED);
        if(data._img.channels() > 1)
        {//按 rgb 处理
            cv::cvtColor(data._img,data._img,CV_RGB2GRAY);
        }
        static int fmindex = 0;
        Position::IFrame *frame = new Position::PFrame(data,feature,fmindex++);
        Position::FrameHelper::assignFeaturesToGrid(frame);
        return frame;
    }
}

std::shared_ptr<Position::IConfig> sCfg =  std::make_shared<HdConfig>("../config_hd.yaml"); 
const string imgpath = GETCFGVALUE(sCfg,ImgPath ,string) + "/";
const string outpath = GETCFGVALUE(sCfg,OutPath ,string) + "/";
const int sbg = GETCFGVALUE(sCfg,StNo,int);
const int sed = GETCFGVALUE(sCfg,EdNo,int);

//获取图片特征信息
Position::FrameVector GetImageFeatures(bool bsave)
{
    std::shared_ptr<Position::IFeature> pFeature(Position::PFactory::CreateFeature(Position::eFeatureOrb,sCfg));

    Position::FrameVector  frames;
    frames.reserve(1 + sed - sbg);
    Position::StringVector filenms;

    Position::PUtils::LoadPathNames(imgpath,filenms);

    assert(!filenms.empty());
    int index = 0;
    for( const string &nm : filenms)
    {
        if(++index < sbg)
            continue;
        if(index > sed)
            break;

        Position::IFrame* pf = CreateFrame(imgpath + nm,pFeature);
        assert(pf);

        const Mat &img = pf->getData()._img;
        frames.emplace_back(pf);
        if(bsave)
        {
            std::string svname = outpath + "keypt_" + nm;
            Mat keyimg;
            cv::drawKeypoints(img,pf->getKeys(),keyimg);
            const string text = "key size :" + std::to_string(pf->getKeySize());
            putText(keyimg, text , Point(50, 50), CV_FONT_HERSHEY_COMPLEX, 2, Scalar(0, 0, 255), 3, CV_AA);
            imwrite(svname,keyimg);
        }
    }
    return frames;
}

void GetFeatureMatch(Position::FrameVector &frames,bool bsave)
{
    if(frames.size() < 2)
        return;
    std::shared_ptr<Position::IFeatureMatcher> pMat(Position::PFactory::CreateFeatureMatcher(Position::eFMDefault,GETCFGVALUE(sCfg,MatchRatio,float)));

    std::string prenm = Position::PUtils::GetCenterStr(frames[0]->getData()._name,"_",".jpg");
    Position::IFrame *preF = frames[0];
    for(int i = 1; i < frames.size();++i)
    {

        Position::MatchVector matches = pMat->match(preF ,frames[i],GETCFGVALUE(sCfg,SearchRadius,int));

        if(matches.empty())
            continue;
        if(bsave)
        {
            std::string cn = Position::PUtils::GetCenterStr(frames[i]->getData()._name,"_",".jpg") ;
            std::string svname = outpath + prenm + "_" + cn + ".jpg";
            Mat keyimg;
            cv::drawMatches(preF->getData()._img,preF->getKeys(),
                         frames[i]->getData()._img,frames[i]->getKeys(),matches,keyimg);
            const string text = "match :" + std::to_string(matches.size());
            putText(keyimg, text , Point(50, 50), CV_FONT_HERSHEY_COMPLEX, 2, Scalar(0, 0, 255), 3, CV_AA);
            imwrite(svname,keyimg);
            prenm = cn;
        }
        preF = frames[i];
    }
}



class MapSer
{
public:

    //打开文件
    bool open(const std::string &path,std::ios::openmode type)
    {
        mfile.open(path, type);
        return mfile.is_open();
    }

    void read(const std::string &path)
    {
        if(open(path,std::ios::in))
        {
            string str;
            getline(mfile,str);
            readLine(str);
            mfile.close();
        }
    }

    void write(const std::string &path)
    {
        if(open(path,std::ios::out))
        {
            Mat mat = (Mat_<double>(9,1) << 1,2,3,4,5,6,7,7,8);

            mfile << "test:" << Position::PConverter::toString(mat) << endl;

            mfile.close();
        }
    }

public:
    //读取行
    void readLine(const std::string &line)
    {
        Position::StringVector strs = Position::PUtils::SplitString(line,":");
        cout << strs[0].c_str() << endl;
        cout << strs[1].c_str() << endl;
    }
    //写入行
    void writeLine(const std::string &name, cv::Mat &pose)
    {
        const std::string line = name + ":" + Position::PConverter::toString(pose);
        // mfile << name + " " + pose
        cout << line.c_str() << endl;
    }
protected:
    fstream mfile;
};

int main(void)
{  
    // MapSer().readLine("test:[1,2,3]");
     
    // Mat mat = (Mat_<double>(9,1) << 1,2,3,4,5,6,7,7,8);

    MapSer().write("/Users/TLG/Documents/Positioning/output/test.txt");
    MapSer().read("test.txt");

    // cout << str2mat(mat2str(mat).c_str() ) << endl;




    return 0;

    Position::CameraParam camera;
    Position::FrameHelper::initParams(GETCFGVALUE(sCfg,ImgWd,int),GETCFGVALUE(sCfg,ImgHg,int),&camera);

    Position::FrameVector frames = GetImageFeatures(true);

    GetFeatureMatch(frames,true);

    return 0;
}