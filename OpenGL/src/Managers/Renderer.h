#pragma once
#include<GL/glew.h>
#include"../Rendering/VertexArray.h"
#include"../Rendering/Shader.h"
#include"../Rendering/IndexBuffer.h"
#define ASSERT(x) if(!(x)) __debugbreak();
#define GLCall(x) GLClearError();\
x;\
ASSERT(GLLogCall(#x,__FILE__,__LINE__))//ASSERT is compiler specific but __ FILE and __LINE is not

void GLClearError();
bool GLLogCall(const char* function, const char* file, int line);


class Renderer
{
public:
	void Draw(const VertexArray & va, const Shader & shader)const;
	void DebugDraw(const VertexArray & va, const Shader & shader)const;
	void Draw(const VertexArray& va,const IndexBuffer& ib,const Shader &shader)const;
	void Clear()const;
};
