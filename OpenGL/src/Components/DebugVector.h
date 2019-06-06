#pragma once

#include "Component.h"
#include"../Rendering/VertexArray.h"
#include"../Rendering/IndexBuffer.h"
#include"../Rendering/VertexArray.h"
#include"../Rendering/VertexBufferLayout.h"
#include"../Rendering/Texture.h"

#include"../Rendering/Shader.h"

class DebugVector : public Component
{
public:
	DebugVector();
	~DebugVector();

	VertexArray va;
	glm::vec4 wireFrameColor;

	void LoadDebugVector();
	DebugVector* Create();
	void Serialize(GenericObject<false, Value::ValueType>);
};

