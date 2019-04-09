# Compilar con Kdevelop

1. En una consola ejecutar `kdevelop`
2. En Project, click en Open / Import Project...
3. Seleccionar el CMakeLists.txt de la carpeta src, click en siguiente
4. Cambiar src por robot_toolkit
5. En Extra Arguments poner: `-DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_INSTALL_PREFIX=../install`
6. Con el boton de build comprobar que la compilacion sea exitosa. 

# Uso de `robot_toolkit.sh`

Usando el bash `robot_toolkit.sh`  usted podra:

1. Compilar el codigo fuente en la maquina virtual `virtual_nao`
2. Instalar los binarios sobre el robot
3. Ejecutar el robot_toolkit en el robot
4. Detener el robot_toolkit en el robot

Para mayor informacion ejecute `./robot_toolkit.sh --help`