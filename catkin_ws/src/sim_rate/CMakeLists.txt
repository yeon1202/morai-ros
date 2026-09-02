cmake_minimum_required(VERSION 3.0.2)
project(sim_rate)

# 파이썬 노드 둘뿐이라 빌드할 C++ 이 없다.
find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/sim_rate_estimator.py
  scripts/wheel_speed_scaler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
