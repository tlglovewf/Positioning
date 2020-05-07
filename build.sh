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

#build positioning
echo "build positioning ..."
cd ../../../
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=$COMTYPE .. 
make -j4
#../output/Positioning
