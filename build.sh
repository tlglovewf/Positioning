#build thirdparty lib
cd Thirdparty 
#DBoW2
cd DBoW2
mkdir build
cd build
cmake ..
make -j4
#g2o
cd ../../g2o
mkdir build && cd build
cmake ..
make -j4
#geographiclib
cd ../../GeographicLib
mkdir build && cd build
cmake ..
make -j4

#build positioning
#cd ../../../
#mkdir build
#cd build
#cmake ..
#make -j
#../output/Positioning
