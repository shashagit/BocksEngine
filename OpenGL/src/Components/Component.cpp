#include "Component.h"

Component::Component(unsigned int Type) {
	mType = Type;
	mpOwner = nullptr;
}

Component::~Component()
{
}