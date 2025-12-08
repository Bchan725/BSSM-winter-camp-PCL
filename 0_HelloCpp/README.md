# Hello Cpp 

---

본 문서는 Cpp 사용 방법을 익히기 위해 제작된 문서입니다. 


## Colcon 을 사용하여 Cpp 프로젝트 생성 방법

```bash
ros2 pkg create --build-type ament_cmake {PROJECT_NAME}
```


## 보통의 Cpp 프로젝트 폴더 구조 

```bash
$pwd: hello_cpp/
.
├── CMakeLists.txt
├── include
│   └── hello_cpp
├── package.xml
└── src
```
