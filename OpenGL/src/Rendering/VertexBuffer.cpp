#include"VertexBuffer.h"
#include"../Managers/Renderer.h"
VertexBuffer::VertexBuffer(const void * data, unsigned int size)
{
	GLCall(glGenBuffers(1, &m_RendererID));//&buffer is the id of the buffer 1 created
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_RendererID));
	GLCall(glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW));

}

VertexBuffer::~VertexBuffer()
{
	GLCall(glDeleteBuffers(1, &m_RendererID));
}

void VertexBuffer::Bind()const
{
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_RendererID));
}

void VertexBuffer::Unbind()const
{
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, 0));
}
