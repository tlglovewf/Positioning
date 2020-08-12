#include "test.h"
#include "iostream"
#include "string"
#include "Thirdparty/filesystem/include/ghc/filesystem.hpp"
using namespace std;

TESTBEGIN()

    ghc::filesystem::path path("/media/tu/Work/GitHub/Positioning/output/pose.txt");
    cout << path.is_absolute() << endl;
    ghc::filesystem::create_directory("test");

    cout << ghc::filesystem::file_size("pose.txt");
TESTEND()