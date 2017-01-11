#--------------------------------------------------------
# The CMakeLists.txt for:        magfield & test_magfield
# Author(s):                                 Nick Rypkema
#--------------------------------------------------------

cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUNIX -D_LINUX=1 -DCMDLINE=1")

# magfield library
SET(SRC
  magfield.cpp
)

ADD_LIBRARY(magfield SHARED ${SRC})

# test magfield library
SET(SRC
  test_magfield.cpp
)

ADD_EXECUTABLE(test_magfield ${SRC})

TARGET_LINK_LIBRARIES(test_magfield
   magfield
   m
   pthread)
