#ifndef ROS2_CONTROL__BURGER1_ROBOT_SYSTEM_HPP_
#define ROS2_CONTROL__BURGER1_ROBOT_HPP_

// Incluyendo los encabezados necesarios
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "burger1_robot/visibility_control.h"

namespace burger1_robot
{
// Clase que implementa la interfaz del sistema de hardware para un robot DiffBot
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  ROS2_CONTROL_PUBLIC
  // Método llamado durante la inicialización del hardware
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_PUBLIC
  // Método para exportar las interfaces de estado del hardware
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_PUBLIC
  // Método para exportar las interfaces de comando del hardware
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_PUBLIC
  // Método llamado cuando el sistema se activa
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_PUBLIC
  // Método llamado cuando el sistema se desactiva
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_PUBLIC
  // Método para leer datos del hardware
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROS2_CONTROL_PUBLIC
  // Método para escribir comandos en el hardware
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parámetros para la simulación del robot DiffBot
  double hw_start_sec_;
  double hw_stop_sec_;

  // Almacenar comandos para el robot simulado
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

}  // namespace ros2_control

#endif  // ROS2_CONTROL__DIFFBOT_SYSTEM_HPP_
