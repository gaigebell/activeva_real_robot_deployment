cmake_minimum_required(VERSION 3.10)
project(d1_bridge CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# TODO(现场): 按实际安装路径调整（与 d1_sdk/CMakeLists.txt 一致的依赖链）
#   - unitree_sdk2 需已 sudo make install 到 /usr/local
#   - ddscxx / iceoryx 头文件路径按机器实际安装位置
set(D1_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../d1_sdk" CACHE PATH "d1_sdk 源码目录（msg/*.hpp）")

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
include_directories(${D1_SDK_DIR}/src)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

add_executable(d1_bridge main.cpp)
