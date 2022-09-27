cmake_minimum_required(VERSION 2.8.3)
project(nlopt)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED)

include(ExternalProject)

set(VERSION 2.4.2)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
    LIBRARIES nlopt_wrap
)

set(NLOPT_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/nlopt_install)
set(NLOPT_INSTALL_DIR ${NLOPT_INSTALL_PREFIX}/${CMAKE_INSTALL_PREFIX})
configure_file(make_install_nlopt.sh.in ${CMAKE_BINARY_DIR}/make_install_nlopt.sh)

ExternalProject_Add(nlopt_src
  DOWNLOAD_COMMAND wget https://github.com/ethz-asl/thirdparty_library_binaries/raw/master/nlopt-${VERSION}.tar.gz
  PATCH_COMMAND tar -xzf ../nlopt-${VERSION}.tar.gz && rm -rf ../nlopt_src-build/nlopt-${VERSION} && mv nlopt-${VERSION} ../nlopt_src-build/
  CONFIGURE_COMMAND nlopt-${VERSION}/configure --with-cxx --without-matlab --with-pic --prefix=${CMAKE_INSTALL_PREFIX}
  BUILD_COMMAND make
  INSTALL_COMMAND ${CMAKE_BINARY_DIR}/make_install_nlopt.sh
)

add_library(nlopt_wrap src/wrap_lib.cc)
if(APPLE)
  target_link_libraries(nlopt_wrap -Wl,-all_load ${NLOPT_INSTALL_DIR}/lib/libnlopt_cxx.a -Wl,-noall_load)
else()
  target_link_libraries(nlopt_wrap -Wl,--whole-archive ${NLOPT_INSTALL_DIR}/lib/libnlopt_cxx.a -Wl,--no-whole-archive)
endif()
add_dependencies(nlopt_wrap nlopt_src)

install(TARGETS nlopt_wrap
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY ${NLOPT_INSTALL_DIR}/
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)
