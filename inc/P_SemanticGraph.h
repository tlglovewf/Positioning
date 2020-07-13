/**
 *   P_SemanticGraph.h
 *   
 *   add by tu li gen   2020.4.9
 * 
 */
#ifndef _SEMANTICGRAPH_H_H_
#define _SEMANTICGRAPH_H_H_
#include "P_Types.h"
#include "P_Utils.h"
#include "P_Checker.h"
#include "P_IOHelper.h"
#include "P_Mask.h"
namespace Position
{
    //动态物体
    class SemanticGraph
    {
    public:
        typedef map<std::string, cv::Vec3b> Item;
        typedef Item::const_iterator ItemIter;

        const std::string defaultsuffix = "png";

        //是否可用
        inline bool isEnabled() const
        {
            return !mObjs.empty();
        }

        //设置语义路径
        inline void setSemanticPath(const std::string &path)
        {
            mPath = path;
        }

        //单例
        static SemanticGraph *Instance()
        {
            static SemanticGraph instance;
            return &instance;
        }
        //加载
        void loadObjInfos(const std::string &path)
        {
            if (!PATHCHECK(path))
            {
                LOG_ERROR("Semantice image path error!!!")
            }
            try
            {
                ifstream segfile;
                segfile.open(path);

                if (segfile.is_open())
                {
                    LOG_INFO("Begin to load Semantice object infos ...");
                    while (!segfile.eof())
                    {
                        std::string str;
                        getline(segfile, str);
                        trimString(str); //去首尾空格
                        if (str.empty() || str[0] == '#')
                        {
                            continue;
                        }
                        else
                        {
                            int s = str.find_first_of(":");
                            int v = str.find_first_of("#"); //剔除注释
                            string name = str.substr(0, s);
                            string result = str.substr(s + 1, (v - s) - 1);
                            trimString(result);
                            int r, g, b;
                            sscanf(result.c_str(), "%d, %d, %d", &r, &g, &b);
                            //opencv default bgr
                            cv::Vec3b vv(b, g, r);
                            mObjs.insert(std::make_pair(name, vv));
                        }
                    }
                    LOG_INFO("End.");
                }
                segfile.close();
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }

        //是否为动态物体
        bool isDyobj(const Point2f &pt, const std::string &name)
        {
            if(!CHECKMASK(pt))
                return true;

            if (!isEnabled())
                return false;

            if (mCurSem.first != name)
            {
                mCurSem.first = name;
                string str = name;
                Position::PUtils::ReplaceFileSuffix(str, "jpg", defaultsuffix);
                mCurSem.second = imread(mPath + str);
            }
            else if (mCurSem.second.empty())
            {
                // PROMT_S("CUR SEM IMAGE EMPTY");
                return false;
            }
            else
            {
                ;
            }
            return isDynamicObj(pt, mCurSem.second);
        }

        //是否为动态物体
        bool isDynamicObj(const Point2f &pt, const Mat &seimg)
        {
            if (seimg.empty())
            {
                PROMTD_S("NO SEM IMAGE");
                return false;
            }
            cv::Vec3b clr = seimg.at<Vec3b>(pt);

            ItemIter it = mObjs.begin();
            ItemIter ed = mObjs.end();

            for (; it != ed; ++it)
            {
                if (it->second == clr)
                {
                    return true;
                }
            }
            return false;
        }
    //! 根据语义色彩生成区域图
    Mat GenerateObjArea(const Mat &img, const Vec3b &clr)
    {
        Mat oimg(img.rows,img.cols,CV_8U);
        for(size_t i = 0; i < img.cols; ++i)
        {
            for(size_t j = 0; j < img.rows; ++j)
            {
                if(img.at<Vec3b>(j,i) == clr)
                {
                    oimg.at<uchar>(j,i) = 255;
                }
                else
                {
                    oimg.at<uchar>(j,i) = 0;
                }
            }
        }
        return oimg;
    }
    
    protected:
        //剔除前后空格
        void trimString(std::string &str)
        {
            if (str.empty())
                return;
            int s = str.find_first_not_of(" ");
            int e = str.find_last_not_of(" ");

            if (s == string::npos ||
                e == string::npos)
                return;

            str = str.substr(s, e - s + 1);
        }

    protected:
        map<std::string, cv::Vec3b> mObjs;
        map<std::string, cv::Mat> mSemImg;
        pair<std::string, cv::Mat> mCurSem;
        std::string mPath;
    };
 
} // namespace Position
#define SETDYOSETTING(F)   Position::SemanticGraph::Instance()->loadObjInfos(F)
#define SETSEMANTICPATH(P) Position::SemanticGraph::Instance()->setSemanticPath(P)
#define CHECKDYO(P,F)      Position::SemanticGraph::Instance()->isDyobj(P,F)       
#endif