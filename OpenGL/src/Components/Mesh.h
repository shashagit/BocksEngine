#pragma once
#include "Component.h"
#include "DebugVector.h"

#include"../Rendering/VertexArray.h"
#include"../Rendering/IndexBuffer.h"
#include"../Rendering/VertexArray.h"
#include"../Rendering/VertexBufferLayout.h"
#include"../Rendering/Texture.h"

#include"../Rendering/Shader.h"

class Mesh : public Component
{
public:
	Mesh();
	~Mesh();

	void LoadMesh();

	Mesh* Create();
	void Serialize(GenericObject<false, Value::ValueType> doc);


	DebugVector* dv;
	Texture* tex;
	VertexArray va;
	
private:
	std::string mTexture;
};

