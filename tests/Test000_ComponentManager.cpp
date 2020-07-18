#include "World/ComponentManager.hpp"
#include <iostream>
#include <cassert>
struct TestComponent {
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
};
void assert_message(bool value, const std::string& message)
{
	if (!value)
	{
		std::cout << message << "\n";
	}
	assert(value);
};
Mona::ComponentHandle handles[2000];
int main()
{
	Mona::Log::StartUp();
	Mona::ComponentManager<TestComponent> testComponentManager;
	testComponentManager.StartUp(1000);
	Mona::GameObjectHandle firstObjectHandle(1, 0);
	Mona::GameObjectHandle secondObjectHandle(2, 0);
	Mona::GameObjectHandle thirdObjectHandle(3, 0);
	Mona::GameObjectHandle fourthObjectHandle(4, 0);
	Mona::GameObjectHandle fifthObjectHandle(5, 0);

	testComponentManager.AddComponent(firstObjectHandle);
	auto initialComponentHandle = testComponentManager.AddComponent(secondObjectHandle);
	testComponentManager.AddComponent(thirdObjectHandle);
	assert_message(testComponentManager[0].x == 0.0f && testComponentManager[0].y == 0.0f && testComponentManager[0].z == 0.0f,
		"Incorrect Component Initialization");
	assert_message(testComponentManager.GetCount() == 3, "Incorrect manager count");
	
	testComponentManager[0].x += 10.0f;
	assert_message(testComponentManager[0].x == 10.0f, "Incorrect direct Update of component data");
	
	auto componentHandle = testComponentManager.AddComponent(fourthObjectHandle);
	auto componentPtr = testComponentManager.GetComponentPointer(componentHandle);
	componentPtr->x += 20.0f;
	assert_message(	testComponentManager.GetComponentPointer(componentHandle)->x == 20.0f,
					"Incorrect component data Update through pointer");

	testComponentManager.RemoveComponent(componentHandle);
	assert_message(testComponentManager.GetCount() == 3, "Incorrect manager count");
	auto componentHandle2 = testComponentManager.AddComponent(fifthObjectHandle);
	auto componentPtr2 = testComponentManager.GetComponentPointer(componentHandle2);
	componentPtr2->x += 77.0f;
	assert_message(	testComponentManager.GetComponentPointer(componentHandle2)->x == 77.0f,
					"Incorrect component data Update through handle after deleting another component");
	assert_message(testComponentManager.GetObjectHandle(componentHandle2).m_index == 5, "Incorrect Object Owner");
	assert_message(testComponentManager.GetObjectHandle(initialComponentHandle).m_index == 2, "Incorrect Object Owner");
	testComponentManager.RemoveComponent(initialComponentHandle);
	componentPtr2 = testComponentManager.GetComponentPointer(componentHandle2);
	componentPtr2->x += 33.0f;
	assert_message(testComponentManager.GetComponentPointer(componentHandle2)->x == 110.0f, 
		"Incorrect component data Update through pointer after deleting another component in the middle of its container");
	assert_message(testComponentManager.GetObjectHandle(componentHandle2).m_index == 5, "Incorrect Object Owner After removal");

	
	for (uint32_t i = 0; i < 2000; i++)
	{
		Mona::GameObjectHandle objHandle(6 + i, 0);
		handles[i] = testComponentManager.AddComponent(objHandle);
		assert_message(testComponentManager.GetObjectHandle(handles[i]).m_index == (6 +i) , "Incorrect Object Owner in Loop");
	}

	auto count = testComponentManager.GetCount() - 1;
	for (uint32_t i = 0; i < 2000; i++)
	{
		testComponentManager.RemoveComponent(handles[i]);
		assert_message(testComponentManager.GetCount() == (count - i) , "Incorrect Component count");
	}

	for (uint32_t i = 0; i < 2000; i++)
	{
		Mona::GameObjectHandle objHandle(count + 1 + i, 0);
		handles[i] = testComponentManager.AddComponent(objHandle);
		assert_message(testComponentManager.GetObjectHandle(handles[i]).m_index == (count + 1 + i), "Incorrect Object Owner in Loop");
	}

	testComponentManager.Clear();

	std::cout << "All test passed!!!\n";
	return 0;
}