// Inclusión de encabezados
#include "burger1_robot/burger1_robot_system.hpp"

// Inclusión de bibliotecas estándar
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

// Inclusión de encabezados de ROS 2
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace burger1_robot
{
  // Implementación de la clase DiffBotSystemHardware que hereda de hardware_interface::SystemInterface
  hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
    const hardware_interface::HardwareInfo & info)
  {
    // Verificación y configuración inicial
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Configuración de parámetros de ejemplo (por motivos ilustrativos)
    hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);

    // Inicialización de vectores de estados y comandos
    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // Verificación de interfaces para cada articulación
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
      // Verificación de interfaces de comandos
      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("DiffBotSystemHardware"),
          "Invalid command interfaces for joint '%s'.", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // Verificación de interfaces de estados
      if (joint.state_interfaces.size() != 2 ||
          joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("DiffBotSystemHardware"),
          "Invalid state interfaces for joint '%s'.", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Exportación de interfaces de estados
  std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    return state_interfaces;
  }

  // Exportación de interfaces de comandos
  std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    return command_interfaces;
  }

  // Activación del hardware
  hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Tareas de activación (por motivos ilustrativos)
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

    for (auto i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
    }

    // Configuración de valores predeterminados
    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
      if (std::isnan(hw_positions_[i]))
      {
        hw_positions_[i] = 0;
        hw_velocities_[i] = 0;
        hw_commands_[i] = 0;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Desactivación del hardware
  hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Tareas de desactivación (por motivos ilustrativos)
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");

    for (auto i = 0; i < hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Lectura de datos del hardware
  hardware_interface::return_type DiffBotSystemHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
  {
    // Lectura de datos simulada (por motivos ilustrativos)
    for (std::size_t i = 0; i < hw_velocities_.size(); i++)
    {
      hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

      RCLCPP_INFO(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
        hw_velocities_[i], info_.joints[i].name.c_str());
    }

    return hardware_interface::return_type::OK;
  }

  // Escritura de datos en el hardware
  hardware_interface::return_type DiffBotSystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Escritura de datos simulada (por motivos ilustrativos)
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");

    for (auto i = 0u; i < hw_commands_.size(); i++)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'!",
        hw_commands_[i], info_.joints[i].name.c_str());

      hw_velocities_[i] = hw_commands_[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully written!");

    return hardware_interface::return_type::OK;
  }

}  // namespace ros2_control

// Exportación de la clase como un plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  burger1_robot::DiffBotSystemHardware, hardware_interface::SystemInterface)
