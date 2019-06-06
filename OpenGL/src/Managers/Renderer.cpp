#include"Renderer.h"
#include<iostream>
void GLClearError()
{
	while (glGetError() != GL_NO_ERROR);
}

bool GLLogCall(const char* function, const char* file, int line)
{
	while (GLenum error = glGetError())
	{
		std::cout << "[OpenGL Error](" << error << ")" << function << " " << file << ":" << line << std::endl;
		return false;
	}
	return true;
}

void Renderer::Draw(const VertexArray & va, const IndexBuffer & ib, const Shader & shader)const
{
	shader.Bind();
	va.Bind();
	ib.Bind();
	GLCall(glDrawElements(GL_TRIANGLES, ib.GetCount(), GL_UNSIGNED_INT, NULL));

}

void Renderer::Draw(const VertexArray & va, const Shader & shader)const
{
	shader.Bind();
	va.Bind();
	unsigned int count = va.GetVertexCount();
	GLCall(glDrawArrays(GL_TRIANGLES, 0, count));

}

void Renderer::DebugDraw(const VertexArray & va, const Shader & shader)const
{
	shader.Bind();
	va.Bind();
	unsigned int count = va.GetVertexCount();
	GLCall(glDrawArrays(GL_LINE_LOOP, 0, count));

}

void Renderer::Clear()const
{
	glClearColor(0.0f, 0.5f, 0.5f, 1.0f);
	GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT))
}
