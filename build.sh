COMTYPE=Release

#build thirdparty lib
cd Thirdparty 
#DBoW2
echo "build dbow2..."
cd DBoW2
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=$COMTYPE .. 
make -j4
#copy so to main lib
cp ../lib/* ../../../lib/

#g2o
echo "build g2o ..."
cd ../../g2o
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=$COMTYPE .. 
make -j4
#copy so to main lib
cp ../lib/* ../../../lib/

#geographiclib
echo "build geographiclib ..."
cd ../../GeographicLib
mkdir build 
cd build
cmake -DCMAKE_BUILD_TYPE=$COMTYPE .. 
make -j4
#copy so to main lib
cp ../lib/* ../../../lib/

#sqlite3
echo "build sqlite3"
cd ../../sqlite3
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=$COMTYPE ..
make -j4
#copy .a to main lib
cp ../lib/*.a ../../../lib/

#log4cpp
echo "build log4cpp ..."
cd ../../log4cpp
mkdir build 
cd build
cmake -DCMAKE_BUILD_TYPE=$COMTYPE .. 
make -j4
#copy so to main lib
cp ../lib/* ../../../lib/
cp ./include/log4cpp/config.h ../include/log4cpp/

#build positioning
echo "build positioning ..."
cd ../../../
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=$COMTYPE .. 
make -j4
#../output/Positioning
