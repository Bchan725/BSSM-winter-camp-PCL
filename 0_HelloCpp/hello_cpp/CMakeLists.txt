cmake_minimum_required(VERSION 3.8)
project(hello_cpp)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 라이브러리 생성
add_library(hello_cpp_lib STATIC
  src/hello_cpp.cpp
)

# include 디렉토리 설정
target_include_directories(hello_cpp_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 실행 파일 생성
add_executable(hello_cpp
  src/main.cpp
)

# 실행 파일에 라이브러리 링크
target_link_libraries(hello_cpp
  hello_cpp_lib
)

# 실행 파일에도 include 디렉토리 설정
target_include_directories(hello_cpp
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
