/*
	TODO:
		- Events class.
			- WindowResize/Close for the moment. -------------- DONE
			- Chain of responsability.
			- Reentrancy
				- Guard against it.
				- Allow reentraancy but maintain consistency.
			- unsubscribe. http://bitsquid.blogspot.com/2011/09/managing-decoupling-part-4-id-lookup.html
			- duplicate registrations.
			- Free functions
		- Add Virtual implmentation to make Application not an abstract class
		- ECS Base work
		- 
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

#include "MonaEngine.hpp"

class Sandbox : public Mona::Application
{
public:
	Sandbox() = default;
	~Sandbox() = default;
	virtual void StartUp() noexcept override{
		MONA_LOG_INFO("Starting User App: Sandbox");
	}

	virtual void ShutDown() noexcept override {
		MONA_LOG_INFO("StuttingDown User App: Sandbox");
	}

	virtual void Update(float timeStep) noexcept override {
		auto &input = Mona::Engine::GetInstance().GetInput();
		auto &window = Mona::Engine::GetInstance().GetWindow();
		if (input.IsKeyPressed(MONA_KEY_G))
		{
			window.SetFullScreen(true);
		}
		else if (input.IsKeyPressed(MONA_KEY_H))
		{
			window.SetFullScreen(false);
		}
		else if (input.IsKeyPressed(MONA_KEY_J))
		{
			window.SetWindowDimensions(glm::ivec2(1000, 1000));
		}

		else if (input.GetMouseWheelOffset().y > 0.0)
		{
			auto Offset = input.GetMouseWheelOffset();
			MONA_LOG_INFO("The mouse offset is ({0},{1})", Offset.x, Offset.y);

		}
	}
};
int main()
{	
	Mona::Engine& engine = Mona::Engine::GetInstance();
	engine.StartUp(std::unique_ptr<Mona::Application>(new Sandbox()));
	engine.StartMainLoop();
	engine.ShutDown();
}