#include "FrameRateController.h"
#include "GLFW/glfw3.h"

#include "stdio.h"

FrameRateController::FrameRateController(unsigned int MaxFrameRate)
{
	mTickEnd = mTickStart = mFrameTime = 0.0f;

	if (0 != MaxFrameRate) {
		mNeededTicksPerFrame = 1.0f / MaxFrameRate;
	}
	else
		mNeededTicksPerFrame = 0.0f;
}


FrameRateController::~FrameRateController()
{
}

void FrameRateController::FrameStart() {
	mTickStart = static_cast<float>(glfwGetTime());
	//printf("mTickStart : %lf\n", mTickStart);
}

void FrameRateController::FrameEnd() {
	mTickEnd = static_cast<float>(glfwGetTime());
	//printf("mTickEnd : %lf\n", mTickEnd);

	while (mTickEnd - mTickStart < mNeededTicksPerFrame) {
		mTickEnd = static_cast<float>(glfwGetTime());
	}

	mFrameTime = mTickEnd - mTickStart;
}

float FrameRateController::GetFrameTime() {
	return mFrameTime;
}