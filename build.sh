echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j


echo "Configuring and building Thirdparty/Sophus ..."

cd ../../Sophus
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j


echo "Uncompress vocabulary ..."

cd ../../../Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..


echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
