# 최소 필요한 CMake 버전을 지정합니다.
cmake_minimum_required(VERSION 3.8)
project(mobile_robot_description)

# C 표준을 C99로 설정합니다 (선택 사항)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++ 표준을 C++14로 설정합니다 (선택 사항)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# GCC 또는 Clang을 사용하는 경우 컴파일러 경고를 활성화합니다 (선택 사항)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성을 찾습니다: ament_cmake 및 xacro
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# 설치 구성: 'launch', 'rviz', 'world', 및 'urdf' 디렉토리를 설치 경로로 복사합니다.
install(
  DIRECTORY launch rviz world urdf gazebo
  DESTINATION share/${PROJECT_NAME}
)

# 테스트 빌드를 위해 빌드하는 경우, ament_lint_auto를 사용하여 코드 린팅을 수행합니다 (선택 사항)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 모든 소스 파일에 저작권 정보가 없는 경우에 저작권 린팅을 건너뛰려면 (필요한 경우 주석 해제)
  #set(ament_cmake_copyright_FOUND TRUE)
  # cpplint를 건너뛰려면 (git 리포지토리가 아닐 때 주석 해제)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament 패키지를 생성합니다.
ament_package()
