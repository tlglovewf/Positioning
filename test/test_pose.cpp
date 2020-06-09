#include "test.h"
#include "P_Factory.h"
#include "project/newhwproject.h"


template<typename T>
class Test
{
public:
    virtual T* create() = 0;
};

class BaseA
{
public:
    virtual void display()const{
        assert(NULL);
    }
};

class TestA : public BaseA
{
public:
      virtual void display()const{
       cout << "test a" << endl;
    }
};

class B : public Test<TestA>
{
public:
     virtual TestA* create(){return new TestA();}
};


// class Task
// {
// public:
//     Task()
// };



TESTBEGIN()
    std::shared_ptr<Position::IConfig>          pcfg(new ImgAutoConfig("../config/config_new.yaml"));
    SETGLOBALCONFIG(pcfg);

    std::shared_ptr<Position::IFeature>         pfeature(GETFEATURE(Sift));
    std::shared_ptr<Position::IFeatureMatcher>  pMatcher(GETFEATUREMATCHER(Knn));
    std::shared_ptr<Position::IFrameData>       pdata(new NewHwProjectData(pcfg));
    Position::FrameData frame;
    pdata->loadDatas();
    std::string path = GETCFGVALUE(pcfg,ImgPath,string) + "/";
    cout << path.c_str() << endl;
    frame._img = imread(path + (*pdata->begin())->_name);
    // pfeature->detect()
    Position::KeyPtVector keys;
    Mat des;
    pfeature->detect(frame,keys,des);
    cout << keys.size() << endl;
    Position::MatchVector matches;
    

TESTEND()