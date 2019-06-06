#pragma once
#include"VertexBuffer.h"
//#include"VertexBufferLayout.h"
class VertexBufferLayout;
class VertexArray
{
private:
	unsigned int m_RendererID;
	unsigned int m_VertexCount;
public:
	VertexArray();
	~VertexArray();
	void SetVertexCount(const int verCount) { m_VertexCount = verCount; }
	unsigned int GetVertexCount() const { return m_VertexCount; }
	void AddBuffer(const VertexBuffer& vb, const VertexBufferLayout& layout);
	void Bind()const;
	void Unbind()const;
};