/* Start Header -------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
File Name: FrameRateController.h
Purpose: Declares the FrameRateController class
Language: C++, MS C++ Compiler
Platform: MS Visual Studio 2017, Windows 10
Project: CS529_shashwatpandey_milestone1
Author: Shashwat Pandey, shashwat.pandey, ID: 60003718
Creation date: 18 October 2018
- End Header --------------------------------------------------------*/

#ifndef FRC_H
#define FRC_H
class FrameRateController
{
public:
	FrameRateController(unsigned int MaxFrameRate);
	~FrameRateController();

	void FrameStart();
	void FrameEnd();

	float GetFrameTime();

private:
	float mTickStart;
	float mTickEnd;
	float mFrameTime;
	float mNeededTicksPerFrame;
};

#endif // !FRC_H