/**
 *   P_SemanticGraph.h
 *   
 *   add by tu li gen   2020.4.9
 * 
 */
#ifndef _SEMANTICGRAPH_H_H_
#define _SEMANTICGRAPH_H_H_H
#include "P_Types.h"
#include "P_Utils.h"
//动态物体
class SemanticGraph
{
public:
    typedef map<std::string, cv::Vec3b> Item;
    typedef Item::const_iterator        ItemIter;

    const std::string defaultsuffix = "png";

    //是否可用
    inline bool isEnabled()const
    {
        return !mObjs.empty();
    }

    //设置语义路径
    inline void setSemanticPath(const std::string &path)
    {
        mPath = path;
    }

    //单例
    static SemanticGraph* Instance()
    {
        static SemanticGraph instance;
        return &instance;
    }
    //加载
    void loadObjInfos(const std::string &path)
    {
        if(path.empty())
        {
            PROMT_S("Semantic image path is error!!!")
        }
        try
        {
            ifstream segfile;
            segfile.open(path);

            if(segfile.is_open())
            {
                cout << "Begin to load Semantic object infos" << endl;
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
                    }
                }
            }
            segfile.close();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    //是否为动态物体
    bool isDyobj(const Point2f &pt, const std::string &name)
    {
        if(!isEnabled())
            return false;

        if(mCurSem.first != name)
        {
            mCurSem.first = name;
             mCurSem.second = imread(mPath+name);
             if( mCurSem.second.empty())
             {
                 string str = name;
                 Position::PUtils::ReplaceFileSuffix(str,"jpg","png");
                  mCurSem.second = imread(mPath + str);
                  if( mCurSem.second.empty())
                  {
                      PROMT_S("NO SEM IMAGE");
                      return false;
                  }
             }
        }
        else if(mCurSem.second.empty())
        {
            return false;
        }
        else
        {
            ;
        }
        assert(!mCurSem.second.empty());

        return isDynamicObj(pt,mCurSem.second);   
    }

    //是否为动态物体
    bool isDynamicObj(const Point2f &pt,const Mat &seimg)
    {
       cv::Vec3b clr = seimg.at<Vec3b>(pt);

       ItemIter it = mObjs.begin();
       ItemIter ed = mObjs.end();

       for(; it != ed ;++it)
       {
           if(it->second == clr)
           {
               return true;
           }
               
       }
       return false;
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
    map<std::string, cv::Mat>   mSemImg;
    pair<std::string,cv::Mat>   mCurSem;
    std::string                 mPath;
};

#endif