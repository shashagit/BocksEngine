#include"IndexBuffer.h"
#include"../Managers/Renderer.h"
IndexBuffer::IndexBuffer(const unsigned int * data, unsigned int count):m_count(count)
{
	ASSERT(sizeof(unsigned int) == sizeof(GLuint));

	GLCall(glGenBuffers(1, &m_RendererID));//&buffer is the id of the buffer 1 created
	GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_RendererID));
	GLCall(glBufferData(GL_ELEMENT_ARRAY_BUFFER, count *sizeof(unsigned int), data, GL_STATIC_DRAW));

}

IndexBuffer::~IndexBuffer()
{
	GLCall(glDeleteBuffers(1, &m_RendererID));
}

void IndexBuffer::Bind()const
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_RendererID);
}

void IndexBuffer::Unbind()const
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
