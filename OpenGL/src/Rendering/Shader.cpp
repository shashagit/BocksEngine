#include"Shader.h"

#include<iostream>
#include<fstream>
#include<string>
#include<sstream>

#include"../Managers/Renderer.h"


Shader::Shader(const std::string & filepath)
	: m_FilePath(filepath),m_RendererID(0)
{
	ShaderProgramSource Source = ParseShader(filepath);
	m_RendererID = CreateShader(Source.VectexSource, Source.FragmentSource);
	
}

Shader::~Shader()
{
	GLCall(glDeleteProgram(m_RendererID));
}

void Shader::Bind() const
{
	GLCall(glUseProgram(m_RendererID));
}

void Shader::Unbind() const
{
	GLCall(glUseProgram(0));
}

void Shader::SetUniform4f(const std::string & name, float v0, float v1, float v2, float v3)
{
	GLCall(glUniform4f(GetUniformLocation(name), v0, v1, v2, v3));
}

void Shader::SetUniform1i(const std::string & name, int value)
{
	GLCall(glUniform1i(GetUniformLocation(name), value));
}

void Shader::SetUniformMat4f(const std::string & name, const glm::fmat4 & matrix)
{
	GLCall(glUniformMatrix4fv(GetUniformLocation(name), 1, GL_FALSE,&matrix[0][0]));
}

unsigned int Shader::CompileShader(unsigned int type, const std::string& source)
{
	unsigned int id = glCreateShader(type);
	const char* src = source.c_str();
	glShaderSource(id, 1, &src, nullptr);//find what this function does online
	glCompileShader(id);

	//Error Handling
	int result;
	glGetShaderiv(id, GL_COMPILE_STATUS, &result);
	if (result == GL_FALSE)
	{
		int length;
		glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
		char* message = (char*)alloca(length * sizeof(char));//READ
		glGetShaderInfoLog(id, length, &length, message);
		std::cout << " Failed to compile the shader!" << (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << std::endl;
		std::cout << message << std::endl;
		glDeleteShader(id);
		return 0;


	}

	return id;
}

 ShaderProgramSource Shader::ParseShader(const std::string& filePath)
{
	std::ifstream stream(filePath);
	enum class ShaderType
	{
		NONE = -1, VERTEX = 0, FRAGMENT = 1
	};
	ShaderType type = ShaderType::NONE;
	std::string line;
	std::stringstream ss[2];
	while (std::getline(stream, line))
	{
		if (line.find("#shader") != std::string::npos)
		{
			if (line.find("vertex") != std::string::npos)
				type = ShaderType::VERTEX;
			else
				if (line.find("fragment") != std::string::npos)
					type = ShaderType::FRAGMENT;
		}
		else
		{
			ss[(int)type] << line << "\n";
		}
	}
	return { ss[0].str(),ss[1].str() };
}

unsigned int Shader::CreateShader(const std::string& vertexShader, const std::string& fragmentShader)
{
	unsigned int program = glCreateProgram();
	unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader);
	unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader);

	glAttachShader(program, vs);//READ ABOUT THESE LATER ON
	glAttachShader(program, fs);//READ ABOUT THESE LATER ON
	glLinkProgram(program);//READ ABOUT THESE LATER ON
	glValidateProgram(program);//READ ABOUT THESE LATER ON

	glDeleteShader(vs);//READ ABOUT THESE LATER ON
	glDeleteShader(fs);//READ ABOUT THESE LATER ON

	return program;

}

int Shader::GetUniformLocation(const std::string & name)
{
	if (m_UniformLocationCache.find(name) != m_UniformLocationCache.end())
		return m_UniformLocationCache[name];

	GLCall(int location=glGetUniformLocation(m_RendererID, name.c_str()));
	
	if (location == -1)
		std::cout << "Warning: uniform " << name << "doesnt exist" << std::endl;

		m_UniformLocationCache[name] = location;
	return location;
}
