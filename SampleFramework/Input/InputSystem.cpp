#include "InputSystem.h"

#include <GLFW/glfw3.h>
#include <algorithm>

void InputSystem::EventHandler::CreateCallbacks(GLFWwindow* window)
{
	glfwSetKeyCallback(window, KeyboardInputCallback);
	//glfwSetWindowSizeCallback(window, Window_Resize_Callback);
	glfwSetCursorPosCallback(window, MouseCursorInputCallback);
	glfwSetMouseButtonCallback(window, MouseButtonInputCallback);
	glfwSetScrollCallback(window, MouseScrollInputCallback);
}

void InputSystem::EventHandler::FlushFrameInputs()
{
	//memset(mMouseDt, 0, sizeof(mMouseDt));
	std::fill(std::begin(mMouseDt), std::end(mMouseDt), 0.0f);
	//bad as inputs that needs to repeat over frame is cleared
	std::fill(&mKeys[0][0], &mKeys[0][0] + NUM_KEYS * NUM_ACTIONS, false);

	std::fill(std::begin(mScroll), std::end(mScroll), 0.0f);
}


void InputSystem::EventHandler::KeyboardInputCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key >= 0 && key < NUM_KEYS && action >= 0 && action < NUM_ACTIONS)
	{
		Instance().mKeys[key][action] = true;

		//for held (GLFW internal repeat) immediate responsiveness too slow due this arcitecture for use case
		if(action == static_cast<int>(EInputMode::Press))
			Instance().mHeldKey[key] = true;
		else if(action == static_cast<int>(EInputMode::Release))
			Instance().mHeldKey[key] = false;
	}
}

void InputSystem::EventHandler::MouseCursorInputCallback(GLFWwindow* window, double x_pos, double y_pos)
{
	//first call update
	static float prev[2]{ float(x_pos), float(y_pos) };

	EventHandler& instance = Instance();
	instance.mMouseDt[0] += float(x_pos) - prev[0];			//<-- x 
	instance.mMouseDt[1] += prev[1] - float(y_pos);			//<-- y

	prev[0] = float(x_pos);
	prev[1] = float(y_pos);
}

void InputSystem::EventHandler::MouseButtonInputCallback(GLFWwindow* window, int button, int action, int mods)
{
	//if (button >= 0 && button < NUM_BUTTONS && action >= 0 && action < NUM_ACTIONS)
	if (button >= 0 && button < NUM_KEYS && action >= 0 && action < NUM_ACTIONS)
	{
		//Instance().mMouseButton[button][action] = true;
		Instance().mKeys[button][action] = true;
	}
}

void InputSystem::EventHandler::MouseScrollInputCallback(GLFWwindow* window, double x_offset, double y_offset)
{
	EventHandler& instance = Instance();
	instance.mScroll[0] += x_offset;
	instance.mScroll[1] += y_offset;
}



bool InputSystem::Input::GetKeyDown(IKeyCode key)
{
 	return EventHandler::Instance().mKeys[int(key)][static_cast<int>(EInputMode::Press)];
}

bool InputSystem::Input::GetKeyUp(IKeyCode key)
{
	return EventHandler::Instance().mKeys[int(key)][static_cast<int>(EInputMode::Release)];
}

bool InputSystem::Input::GetKey(IKeyCode key)
{
	return EventHandler::Instance().mHeldKey[int(key)];
}

bool InputSystem::Input::GetMouse(IKeyCode key)
{
	return EventHandler::Instance().mKeys[int(key)][static_cast<int>(EInputMode::Release)];
}

bool InputSystem::Input::GetMousePressed(IKeyCode key)
{
	//return EventHandler::Instance().mMouseButton[static_cast<int>(key)][GLFW_PRESS];
	return EventHandler::Instance().mKeys[int(key)][static_cast<int>(EInputMode::Press)];
}

float InputSystem::Input::GetMouseAxisFloat(IAxis axis)
{
	return EventHandler::Instance().mMouseDt[int(axis)];
}

float InputSystem::Input::GetScrollWheel()
{
	return  EventHandler::Instance().mScroll[1];
}
