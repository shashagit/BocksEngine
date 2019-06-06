/* Start Header -------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
File Name: InputManager.h
Purpose: Declare the InputManager class
Language: C++, MS C++ Compiler
Platform: MS Visual Studio 2017, Windows 10
Project: CS529_shashwatpandey_milestone1
Author: Shashwat Pandey, shashwat.pandey, ID: 60003718
Creation date: 18 October 2018
- End Header --------------------------------------------------------*/

#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

#include<unordered_map>

enum KEYBOARD_KEYS {
	UP,
	DOWN,
	LEFT,
	RIGHT,
	JUMP,
	Z,
	P,
	F,
	X,
	N,
	D,
	ESC,
	NUM
};

class InputManager
{
public:
	InputManager();
	~InputManager();

	void Update();

	bool isPressed(unsigned int KeyScanCode);

	bool isTriggered(unsigned int KeyScanCode);
	bool isReleased(unsigned int KeyScanCode);

	bool leftClicked;
	bool rightClicked;

private:
	void UpdateStatesForKeyboardKeys();

	int keyCodes[NUM];

	std::unordered_map<KEYBOARD_KEYS, bool> keyStatePressed;
	std::unordered_map<KEYBOARD_KEYS, bool> keyStateReleased;
	std::unordered_map<KEYBOARD_KEYS, bool> keyStateTriggered;
};

#endif // !INPUT_MANAGER_H
