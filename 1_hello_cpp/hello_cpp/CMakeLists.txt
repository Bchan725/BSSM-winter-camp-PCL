cmake_minimum_required(VERSION 3.10)

project(hello_cpp)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 실행 파일 생성
add_executable(hello_cpp
    src/main.cpp
    src/person.cpp
)

# include 디렉토리 추가
target_include_directories(hello_cpp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

