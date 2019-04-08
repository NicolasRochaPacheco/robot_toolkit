# Compilar con Kdevelop

1. En una consola ejecutar `kdevelop`
2. En Project, click en Open / Import Project...
3. Seleccionar el CMakeLists.txt de la carpeta src, click en siguiente
4. Cambiar src por robot_toolkit
5. En Extra Arguments poner: `-DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_INSTALL_PREFIX=../install`
6. Con el boton de build comprobar que la compilacion sea exitosa. 

