/*
	TODO:
		- Events class.
			- WindowResize/Close for the moment. -------------- DONE
			- Chain of responsability.
			- Reentrancy
				- Guard against it.
				- Allow reentraancy but maintain consistency.
			- unsubscribe.
			- duplicate registrations.
		- Link glfw event ------------------------- DONE
		- TIME using chrono? ------------------------------------------------ FIRST
		- Add first example.
			- Build Mona as static library ------ DONE
			- Add new target for each example ------ DONE
			- Application.h ----> interface for the moment.  --------------------- SECOND
			- write first example using different interfaces. -------------------- THIRD
		- Design OPENGL Abs ------------------------------> FOURTH
		- Device/Context OpenGL ->.
		- Buffer classes -> VBO/IBO/UBO.
		- Shader class -> .
		- VAO.
		- Basic primitives.
		- Textures.
		- Model loading.
		- Resource Manager
		- Mesh drawing.
		- Animation loading.
		- Animation rendering.
		- Collision.
*/
/*
	Reu:
		- Temas memoria
			- Cambio de fechas.
			- Contar lo que se espera de la segunda entrega.
			- Approarch de la memoria en general.
				- Preguntar cual debería ser mi acercamiento al Estado del arte.
				- Acercamiento a implementar cosas => probablemente nada sera del estado del arte.
					- Engine Approach examinar motores desde una perspectiva general, y detallar sistemas overviewmente.
			- Confirmar plazos.
		- Temas codigo.
			- Log/Assertion/Window/Config/Input.
			- Event System.
				- Tipos de eventos.
					- Basados en herencia.
					- Data-driven??.
					- Queued?.
					- Recursion problematica?.
			- Allocation ?? std::chrono etc..
			- Framework de testing.
			- Complejidad de ejemplos.
				- .
				- .
				- .

*/
#include "Engine.hpp"
int main()
{	
	Mona::Engine engine = Mona::Engine();
	engine.StartUp();
	engine.StartMainLoop();
	engine.ShutDown();
}