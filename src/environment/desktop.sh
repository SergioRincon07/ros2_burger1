#!/bin/bash

# Este es un comentario
echo "Iniciando construcción en la Desktop ..."

# Configurar el entorno si es necesario
# (puedes agregar comandos para instalar dependencias, configurar variables de entorno, etc.)

# Construir paquetes específicos para la Desktop Pi
colcon build --packages-select burger1_description robot_control_manager
source install/setup.bash

echo "Construcción en la Desktop completada."
