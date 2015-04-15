/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/06
*/
#ifndef __SHADERCLASS_H_
#define __SHADERCLASS_H_

#include "ME\system\SystemSingleton.h"
#include "ME\system\ShaderCompiler.h"

namespace MyEngine
{

class MyEngineDLL ShaderClass
{
	String m_sProgramName;
	String m_sVertexShaderName;
	String m_sFragmentShaderName;
	GLuint m_nProgram;
public:
	ShaderClass(void);
	ShaderClass(const ShaderClass& other);
	ShaderClass& operator=(const ShaderClass& other);
	~ShaderClass(void);
	
	void Swap(ShaderClass& other);

	GLuint CompileShader(String a_sVertexShader, String a_sFragmentShader, String a_sName = "");

	//Attributes
	String GetProgramName();
	__declspec(property(get = GetProgramName)) String Name;

	String GetVertexShaderName();
	__declspec(property(get = GetVertexShaderName)) String VertexShader;

	String GetFragmentShaderName();
	__declspec(property(get = GetFragmentShaderName)) String FragmentShader;

	int GetProgramID();
	__declspec(property(get = GetProgramID)) int ProgramID;

	void Release(void);

protected:
	void Init(void);
	
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<ShaderClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<ShaderClass*>;

}

#endif //__SHADERCLASS_H__