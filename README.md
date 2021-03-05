# MonaEngine
Motor desarrollado en C++ para el proceso de titulación en la carrera de Ingeniería en Computación de la Universidad de Chile.
## Dependencias
MonaEngine depende de una lista de diferentes librerías, estas estan en el directorio `thirdParty/`
 - [ImGui](https://github.com/ocornut/imgui) Para gui de depuración
 - [GLFW](https://www.glfw.org/) para la creación y manejo de ventanas e input.
 - [glad](https://glad.dav1d.de/) para cargar las funciones de OpenGL 
 - [glm](https://glm.g-truc.net/0.9.9/index.html) para algebra lineal.
 - [spdlog](https://github.com/gabime/spdlog) para Logging
 - [stb\_image](https://github.com/nothings/stb) para cargar imagenes desde disco
 - [dr_wav](https://mackron.github.io/dr_wav.html) para cargar archivos .wav desde disco
 - [assimp](https://github.com/assimp/assimp) para cargar mallas, animaciones y esqueletos.
 - [OpenAL-Soft](https://github.com/kcat/openal-soft) para la implementacion del sistema de audio.
 - [BulletPhysics](https://github.com/bulletphysics/bullet3) para la implementacion de sistema de fisica.
 
## Software/Libreria externas necesarias
Para generar el projecto,solución o makefiles se necesita [CMake 3.15+](https://cmake.org/), y de OpenGL4.5 para funcionar.

## Generando la solución o makefiles
En windows basta con ejecutar el comando:
cmake -G "Visual Studio 16 2019" -A x64 
Dentro del directorio del repositorio.


##Posibles problemas
El motor se desarrolló principalmente en windows y se probó en dos computadores con linux, en uno de estos fue necesario instalar
algunas librerías con los siguientes comandos:
 - sudo apt-get install xorg-dev libglu1-mesa-dev
 - sudo apt-get install libx11-dev


## Assets usados

