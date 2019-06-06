#include "DebugVector.h"



DebugVector::DebugVector() : Component(DEBUGVEC)
{
}


DebugVector::~DebugVector()
{
}

DebugVector* DebugVector::Create()
{
	return new DebugVector();
}

void DebugVector::LoadDebugVector() {
	float positions[] = {
		0.5f, 0.0f, 0.0f,
		-0.5f, 0.0f, 0.0f
	};

	VertexBuffer vb(positions, 2 * 3 * sizeof(float));
	va.SetVertexCount(2);

	VertexBufferLayout layout;
	layout.Push<float>(3);

	va.AddBuffer(vb, layout);

	va.Unbind();
}

void DebugVector::Serialize(GenericObject<false, Value::ValueType> d) {

}