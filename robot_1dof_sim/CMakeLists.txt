cmake_minimum_required(VERSION 3.8)
project(robot_1dof_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# 1. DEPENDENCIAS DE COMPILACIÓN
# ==============================================================================
find_package(ament_cmake REQUIRED)

# ==============================================================================
# 2. INSTALACIÓN DE ARCHIVOS (¡CRÍTICO PARA ROS 2!)
# ==============================================================================
# Esta instrucción le dice a Colcon que tome estas carpetas de nuestro espacio 
# de trabajo (src) y las copie al directorio de instalación global de ROS 2.
# Sin esto, el comando 'ros2 launch' nunca encontrará el URDF ni los YAML.

install(DIRECTORY
  urdf
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# 3. SECCIÓN DE PRUEBAS (LINTERS)
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Genera los archivos finales del paquete
ament_package()