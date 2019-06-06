#include "VertexArray.h"
#include"../Managers/Renderer.h"
#include"VertexBufferLayout.h"
VertexArray::VertexArray()
{
	GLCall(glGenVertexArrays(1, &m_RendererID));
	
}

VertexArray::~VertexArray()
{
	GLCall(glDeleteVertexArrays(1, &m_RendererID));
}

void VertexArray::AddBuffer(const VertexBuffer & vb, const VertexBufferLayout & layout)
{
	Bind();
	vb.Bind();
	const auto& elements = layout.GetElements();
	unsigned int offset = 0;
	for (unsigned int i = 0; i < elements.size(); i++)
	{
		const auto& element = elements[i];
		GLCall(glEnableVertexAttribArray(i));//VID 5//this will enable the verte so that opengl/gpu knows to draw this
		GLCall(glVertexAttribPointer(i, element.count, element.type, element.normalized,layout.GetStride(), (const void*)offset));//VID 5//telling gpu the details about the vertex and where it starts and what it contains
		offset += element.count * VertexBufferElement::GetSizeOfType(element.type);
	}
}

void VertexArray::Bind() const
{
	GLCall(glBindVertexArray(m_RendererID));
}

void VertexArray::Unbind() const
{
	GLCall(glBindVertexArray(0));
}
