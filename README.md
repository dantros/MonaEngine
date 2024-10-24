# MonaEngine
3D Game Engine implementado en C++ con fines académicos. Este motor ha sido probado en Windows y Ubuntu.

## Screenshots
![examples/Breakout.cpp](screenshots/breakout.png "breakout")
![examples/AnimationAudioExample.cpp](screenshots/animation_audio.png "AnimatedExample")

Ejemplos y sus descripciones disponibles en [examples](examples/README.md)

## Setup

Este repositorio utiliza submódulos git para algunas de sus dependencias, por lo que luego de clonar el repositorio, es necesario ejecutar en el directorio raíz.
```
git submodule update --init --recursive
```

Para generar el projecto, solución o makefiles se necesita [CMake 3.20+](https://cmake.org/), y de OpenGL4.5 para funcionar.

### Construcción vía CMakePresets

Es posible utilizar 'CMakePresets.json' para generar la solución de Visual Studio que compilará el motor.
```
cmake --preset vs-debug-examples
```
Esto compilará la biblioteca estática MonaEngine y los códigos de ejemplo. Si los códigos de ejemplo no son de interés, se puede utilziar el preset `vs-debug`. Para linux existen los presets `linux-debug` y `linux-debug-examples` los cuales generarán los Makefiles correspondientes. Revisar otros presets y detalles en el archivo `CMakePresets.json`.

Estos presets generarán una carpeta llamada `build` paralela la carpeta MonaEngine donde se encuentra el código fuente. En el nuevo directorio `build` se debe abrir `MonaEngine.sln` con Visual Studio (Windows), o ejecutar `make` para procesar los Makefiles (Linux).

### Construcción vía CMake directo

Por supuesto, también es posible utilizar directamente `cmake` o `cmake-gui`.

En windows basta con ejecutar el comando: **cmake -G "Visual Studio 17 2022" -A x64** , dentro del directorio del repositorio. Es importante
mencionar que la version de VS cambiara dependediendo de la version instalada en el computador donde se desea compilar el motor.

En linux basta ejecutar el comando **cmake . -DCMAKE_BUILD_TYPE=BUILDTYPE**, dentro del directorio del repositorio. BUILDTYPE puede ser
DEBUG o RELEASE.

Una vez generada la solición o los makefiles, basta con seguir el flujo tipico de Visual Studio o Linux.

### Posibles problemas

El motor se desarrolló principalmente en windows y se probó en dos computadores con Ubuntu Linux, en uno de estos fue necesario instalar

algunas librerías con los siguientes comandos:
 - sudo apt-get install xorg-dev libglu1-mesa-dev
 - sudo apt-get install libx11-dev

En uno de los computadores con linux en los que se probaron los ejemplos desarrollados fue necesario ejecutarlos con permisos de administrador
ya que OpenAL los necesita, de no hacerse se provocará un crash en medio de la ejecución de los ejemplos.

## Creando aplicaciones con el motor

Es posible utilizar este motor de distintas maneras, sin embargo, un enfoque sencillo es ubicar el código fuete del motor dentro del código fuente de la aplicación o videojuego, y enlazar la biblioteca estática `MonaEngine`.

La dependencia a OpenAL-soft (licencia LGPL) se mantiene como biblioteca dinámica para mantener la licencia de MonaEngine como MIT. Por lo que para ejecutar la aplicación el archivo de biblioteca dinámica de OpenAL debe estar disponible. La forma simple es copiar el archivo OpenAL32.dll u OpenAL.so al mismo directorio que el ejecutable.

Una estructura de archivos y carpetas de ejemplo:
 - Una carpeta de nombre MonaEngine que contiene los archivos del repositorio del motor (puede ser un submódulo git).
 - Un archivo llamado CMakeLists.txt con contenido similar al extracto de código mostrado más abajo.
 - El resto de los archivos fuentes y encabezados necesarios para la aplicación o videojuego. En el caso del ejemplo ilustrado por el extracto de código mostrado más abajo, estos corresponden a main.cpp, clase0.h, clase0.cpp, clase1.h y clase.cpp.
 - Generar (o escribir) un archivo `config.json` que especifíque el path para assets del motor y de la aplicación.

```
set(CMAKE_LEGACY_CYGWIN_WIN32 OFF)
cmake_minimum_required(VERSION 3.20)
project(SomeGame C CXX)
add_subdirectory(MonaEngine)
add_executable(SomeGame main.cpp clase0.h clase0.cpp clase1.h clase1.cpp) 
set_property(TARGET SomeGame  PROPERTY CXX_STANDARD 20)
target_link_libraries(SomeGame  PRIVATE MonaEngine)
target_include_directories(SomeGame  PRIVATE  ${MONA_INCLUDE_DIRECTORY} ${THIRD_PARTY_INCLUDE_DIRECTORIES})

# Configurando path para assets en archivo config.json
set(APPLICATION_ASSETS_DIR ${CMAKE_SOURCE_DIR}/MyApp/Assets)
set(ENGINE_ASSETS_DIR ${CMAKE_SOURCE_DIR}/EngineAssets)
configure_file(${CMAKE_SOURCE_DIR}/config.json.in config.json)
```

La última parte del cógido cmake anterior, se encarga de configurar un archivo json. Dado que cargaremos assets en tiempo de ejecución, es necesario ubicar dichos archivos de manera consistente. Si no se encuentra un archivo `config.json` junto al ejecutable, se deben encontrar las carpetas `Assets` y `EngineAssets`. En `Assets` se deben almacenar todos los assets propios de la aplicación, mientras que `EngineAssets`, se ubican aquellos que son propios del engine (Ejemplo: Shaders que son utilizados por el renderer).

Para acceder a estos directorios de assets desde código, se debe utilizar el singleton `Config` del siguiente modo:

```
Mona::Config& config = Mona::Config::GetInstance();
std::string assetPathStr = config.getPathOfApplicationAsset("AudioFiles/music.wav")
```
Usualmente una apliación no necesitará assets provistos por el Engine, pero sí se utilizan dentro del motor mismo. Ahí se accede a dichos assets con: `config.getPathOfEngineAsset(Shaders/UnlitFlat.vs)`.

La recomendación para comenzar, es revisar los ejemplos disponibles en la carpeta [examples](examples/README.md) y el repositorio https://github.com/dantros/BreakoutMona, donde se incluye MonaEngine como un submódulo git.

### Manejo de Assets en tiempo de desarrollo

Para facilitar el desarrollo, es posible utilizar un archivo de configuración `config.json` junto al ejecutable, dentro de este archivo se pueden especificar directorios absolutos donde se buscarán los assets. Esto facilita el desarrollo vía un IDE como Visual Studio, donde se crean carpetas intermedias dependiendo del tipo de compilación (Release, Debug, etc).

Convenientemente, los archivos CMake provistos utilizan `configure_file` para generar el archivo `config.json` (en base a `config.json.in`) y lo copia en las carpetas de desarrollo correspondientes, de esta forma, todo debiera funcionar inmediatamente, ya sea ejecutando una sesión de Debug en Visual Studio, o directamente vía doble clic en el ejecutable.

Ejemplo de archivo `config.json`:
```
{
    "windowTitle": "MonaEngine Application",
    "OpenGL_major_version": 4,
    "OpenGL_minor_version": 5,
    "N_OPENAL_SOURCES": 32,
    "expected_number_of_gameobjects": 1200,
    "application_assets_dir": "C:\mona_engine_app\install\Assets",
    "engine_assets_dir": "C:\mona_engine_app\install\EngineAssets"
}
```

Si no se especifican `application_assets_dir` y/o `engine_assets_dir`, o si no se encuentra un archivo `config.json`, se buscarán los directorios `Assets` y `EngineAssets` que deben estar ubicados junto al ejecutable.

Ejemplo de directorios para aplicación portable (*):
```
AnyFolder/
+---------SomeGame.exe
+---------OpenAL.dll
+---------Assets/
          +------model.fbx
          +------texture.png
          +------model.fbx
          +------...
+---------EngineAssets/
          +------Shaders/
                 +-------shader1.vs
                 +-------shader1.ps
          +------...
```

Ejemplo de directorios para aplicación en desarrollo:
```
C:\mona_engine_src\build\
+---------SomeGame.exe
+---------OpenAL.dll
+---------config.json
C:\mona_engine_app\install\
+---------Assets/
          +------model.fbx
          +------texture.png
          +------model.fbx
          +------...
C:\mona_engine_src\MonaEngine\
+--------EngineAssets/
          +------Shaders/
                 +-------shader1.vs
                 +-------shader1.ps
          +------...
```

En este último caso se deben especificar los directorios de assets en el archivo `config.json`:
```
{
    ...
    "application_assets_dir": "C:\mona_engine_app\install\Assets",
    "engine_assets_dir": "C:\mona_engine_src\MonaEngine\EngineAssets"
}
```

## Instalación

Una vez terminada la aplicación/videojuego, basta con construir el target INSTALL (de CMakePredefinedTargets) para generar una versión portable. CMake generará la estructura de archivos (*) dentro del directorio `${CMAKE_INSTALL_PREFIX}/bin` para los ejemplos, en caso de haberlos compilado. La biblioteca estática `MonaEngine.lib` (windows) o `MonaEngine.a` (linux) estará disponible en `${CMAKE_INSTALL_PREFIX}/lib`. El directorio `${CMAKE_INSTALL_PREFIX}/include` tendrá disponible los headers públicos de MonaEngine para la implementación de una nueva aplicación.

## Dependencias

MonaEngine depende de una lista de diferentes librerías, estas estan en el directorio `thirdParty/`
 - [ImGui](https://github.com/ocornut/imgui) Para gui de depuración.
 - [GLFW](https://www.glfw.org/) para la creación y manejo de ventanas e input.
 - [glad](https://glad.dav1d.de/) para cargar las funciones de OpenGL.
 - [glm](https://glm.g-truc.net/0.9.9/index.html) para manejo vectorial 3D.
 - [spdlog](https://github.com/gabime/spdlog) para Logging.
 - [stb\_image](https://github.com/nothings/stb) para cargar imagenes desde disco.
 - [dr_wav](https://mackron.github.io/dr_wav.html) para cargar archivos .wav desde disco.
 - [assimp](https://github.com/assimp/assimp) para cargar mallas, animaciones y esqueletos.
 - [OpenAL-Soft](https://github.com/kcat/openal-soft) para la implementacion del sistema de audio.
 - [BulletPhysics](https://github.com/bulletphysics/bullet3) para la implementacion de sistema de fisica.

## El origen de MonaEngine

Este motor fué inicialmente desarrollado por Byron Cornejo, en el contexto de su titulación en la carrera de Ingeniería en Computación de la Universidad de Chile. El documento escrito que describe en detalle el trabajo realizado esta en la carpeta Memoria.

Sobre el trabajo anterior, Agustín Matthey realizó su trabajo de titulación, para la misma carrera, el cual implementa una metodología para modificar animaciones de caminatas y adaptarlas a terrenos de altura variable vía utilizando cinemática inversa. El detalle de este trabajo de título se encuentra en la carpeta Memoria.

Posteriormente, Daniel Calderón ha mantenido el repositorio https://github.com/dantros/MonaEngine, agregando funcionalidades y mejorando su usabilidad. El objetivo es facilitar la enseñanza del curso "CC5512: Arquitectura de Motores de Juegos", dictado en el departamento de Ciencias de la Computación de la Universidad de Uchile. El programa académico de dicho curso se encuentra disponible en: https://www.u-cursos.cl/ingenieria/2023/1/CC5512/1/material_docente/detalle?id=8028657
