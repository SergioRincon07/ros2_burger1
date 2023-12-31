#!/bin/bash

# Este es un comentario
echo "Iniciando construcción en la Raspberry Pi..."

# Configurar el entorno si es necesario
# (puedes agregar comandos para instalar dependencias, configurar variables de entorno, etc.)

# Construir paquetes específicos para la Raspberry Pi
colcon build --packages-select burger1_robot
source install/setup.bash

echo "Construcción en la Raspberry Pi completada."
