/* Start Header -------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
File Name: InputManager.cpp
Purpose: Defines the FrameRateController class methods
Language: C++, MS C++ Compiler
Platform: MS Visual Studio 2017, Windows 10
Project: CS529_shashwatpandey_milestone1
Author: Shashwat Pandey, shashwat.pandey, ID: 60003718
Creation date: 18 October 2018
- End Header --------------------------------------------------------*/

#include "InputManager.h"
#include "CameraController.h"
#include "GLFW/glfw3.h"

extern GLFWwindow* window;
extern CameraController* camera;

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

InputManager::~InputManager()
{
}

float lastX = 540 / 2.0f;
float lastY = 960 / 2.0f;
bool firstMouse = true;
bool _leftClicked = false;
bool _rightClicked = false;
// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (_rightClicked) {
		if (firstMouse)
		{
			lastX = static_cast<float>(xpos);
			lastY = static_cast<float>(ypos);
			firstMouse = false;
		}

		float xoffset = static_cast<float>(xpos) - lastX;
		float yoffset = lastY - static_cast<float>(ypos); // reversed since y-coordinates go from bottom to top

		lastX = static_cast<float>(xpos);
		lastY = static_cast<float>(ypos);

		camera->ProcessMouseMovement(xoffset, yoffset, true);
	}
	else {
		firstMouse = true;
	}
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera->ProcessMouseScroll(static_cast<float>(yoffset));
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
		_leftClicked = true;
	else
		_leftClicked = false;

	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
		_rightClicked = true;
	else
		_rightClicked = false;

}

void InputManager::Update()
{
	leftClicked = _leftClicked;
	rightClicked = _rightClicked;

	UpdateStatesForKeyboardKeys();
}

void InputManager::UpdateStatesForKeyboardKeys() {
	int state = 0;
	for (int i = 0; i < NUM; ++i) {
		state = glfwGetKey(window, keyCodes[i]);

		if (state == GLFW_PRESS) {
			// check for triggered
			if (keyStateReleased[static_cast<KEYBOARD_KEYS>(i)]) {
				keyStateTriggered[static_cast<KEYBOARD_KEYS>(i)] = true;
			}
			else {
				keyStateTriggered[static_cast<KEYBOARD_KEYS>(i)] = false;
			}

			keyStatePressed[static_cast<KEYBOARD_KEYS>(i)] = true;
			keyStateReleased[static_cast<KEYBOARD_KEYS>(i)] = false;

		}
		else if (state == GLFW_RELEASE) {
			keyStatePressed[static_cast<KEYBOARD_KEYS>(i)] = false;
			keyStateTriggered[static_cast<KEYBOARD_KEYS>(i)] = false;
			keyStateReleased[static_cast<KEYBOARD_KEYS>(i)] = true;
		}
	}
}

bool InputManager::isPressed(unsigned int KeyScanCode) {
	return keyStatePressed[static_cast<KEYBOARD_KEYS>(KeyScanCode)];
}


bool InputManager::isTriggered(unsigned int KeyScanCode) {
	return keyStateTriggered[static_cast<KEYBOARD_KEYS>(KeyScanCode)];
}

bool InputManager::isReleased(unsigned int KeyScanCode) {
	return keyStateReleased[static_cast<KEYBOARD_KEYS>(KeyScanCode)];
}

InputManager::InputManager()
{
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);

	// set all keycodes for keys used in game
	keyCodes[0] = GLFW_KEY_W;
	keyCodes[1] = GLFW_KEY_S;
	keyCodes[2] = GLFW_KEY_A;
	keyCodes[3] = GLFW_KEY_D;
	keyCodes[4] = GLFW_KEY_SPACE;
	keyCodes[5] = GLFW_KEY_Z;
	keyCodes[6] = GLFW_KEY_P;
	keyCodes[7] = GLFW_KEY_F;
	keyCodes[8] = GLFW_KEY_X;
	keyCodes[9] = GLFW_KEY_N;
	keyCodes[10] = GLFW_KEY_D;
	keyCodes[11] = GLFW_KEY_ESCAPE;

	for (int i = 0; i < NUM; ++i) {
		keyStatePressed[static_cast<KEYBOARD_KEYS>(i)] = false;
		keyStateTriggered[static_cast<KEYBOARD_KEYS>(i)] = false;
		keyStateReleased[static_cast<KEYBOARD_KEYS>(i)] = true;
	}
}