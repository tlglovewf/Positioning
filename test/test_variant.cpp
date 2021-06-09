#include "test.h"
#include "iostream"
#include "string"
#include "variant/optional.hpp"
using namespace std;

TESTBEGIN()

    mapbox::util::variant<int,string,float> item(10);
     
    cout << item.get<int>() << endl;

    cout << item.is<string>() << endl;

    
TESTEND()