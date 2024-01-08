# Establece la versión mínima de CMake requerida
cmake_minimum_required(VERSION 3.8)
# Define el nombre del proyecto y el lenguaje de programación
project(burger1_robot LANGUAGES CXX)

# Agrega opciones de compilación específicas para los compiladores GNU y Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Define las dependencias del paquete
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Busca las dependencias del paquete utilizando ament_cmake
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Compila la biblioteca compartida burger1_robot a partir del archivo burger1_robot_system.cpp
add_library(
  burger1_robot
  SHARED
  src/burger1_robot_system.cpp
)

# Configura características de compilación, directorios de inclusión y dependencias para la biblioteca
target_compile_features(burger1_robot PUBLIC cxx_std_17)
target_include_directories(burger1_robot PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/burger1_robot>
)
ament_target_dependencies(
  burger1_robot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Configura las macros de visibilidad para usar dllexport en lugar de dllimport durante la construcción de la biblioteca
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_BUILDING_DLL")

# Exporta la descripción del plugin para la interfaz de hardware
pluginlib_export_plugin_description_file(hardware_interface burger1_robot.xml)


# Instala archivos en directorios específicos para su uso posterior
install(
  DIRECTORY include/
  DESTINATION include/burger1_robot
)
install(
  DIRECTORY config/
  DESTINATION share/burger1_robot
)
install(TARGETS burger1_robot
  EXPORT burger1_robot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Configura pruebas si la opción de construcción de pruebas está habilitada
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Exporta metas del paquete, sus dependencias y otros detalles para el sistema de construcción de ROS 2
ament_export_targets(burger1_robot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
