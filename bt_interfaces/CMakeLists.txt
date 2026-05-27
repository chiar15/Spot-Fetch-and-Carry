cmake_minimum_required(VERSION 3.8)
project(bt_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies - SOLO quelle usate nel .srv
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)  # ← SOLO questo è usato nel tuo srv

# Genera interfacce - DEPENDENCIES esatte del srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Detection.srv"
  DEPENDENCIES builtin_interfaces  # ← Corretto: solo quello usato
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
