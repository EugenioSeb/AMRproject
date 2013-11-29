cmake_minimum_required(VERSION 2.8)
project(remoteApi)

add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-D__linux)

add_library(remoteApi v_repConst.h extApiPlatform.h extApiPlatform.c extApi.h extApi.c v_repLib.h extApiInternal.h)
target_link_libraries(remoteApi pthread)
