#include "test.h"
#include "P_Interface.h"
#include "P_IOHelper.h"
#include "P_Checker.h"
#include "P_CoorTrans.h"
#include "P_Utils.h"
#include "project/newhwproject.h"
#include "Thirdparty/sqlite3/sqlite3.h"
#include "P_SemanticGraph.h"
#include "P_Mask.h"
#include "P_ORBFeature.h"
template <typename T>
inline void display(T items)
{
    for (auto i : items)
    {
        cout << i << " ";
    }
    cout << endl;
}

#define _TOSTR(a) #a
#define NMB_STR(a) _TOSTR(a)
#define FILE_LOG() "{" __FILE__ "}"
#define LINE_LOG() "{" NMB_STR(__LINE__) "}"
#define PRINTMESSAGE(X) FILE_LOG() \
LINE_LOG() ":" X

#define FM_T_V "%d" \
               "%s"
#define FM_T_S "%s"

#include <regex>

typedef unsigned int U4;
typedef int L4;
// typedef

// int (*callback)(void *, int, char **, char **)


template<typename T>
class IFactory
{
public:
    // template<size_t Index>
    // using type = typename std::tuple_element<Index,std::tuple<Args...> >::type;

    virtual string name()const = 0;
};

class IFeatureFactory : public IFactory<Position::IFeature>
{
public:

};

#define IFEATUREFACTORYDECLARE(F,D) \
        class I##F##Factory : public IFeatureFactory<F>\
        {\
        public:\
            virtual shared_ptr<F> create(const D &d){assert(NULL);}\
            virtual string name()const {return #F;}\
        };



TESTBEGIN()
   
//add more code
    // ORBFeatureFactory orbf;
    // shared_ptr<IFeature> pf = orbf.create();


TESTEND()