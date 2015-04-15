/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/05
*/
#ifndef __SHADERMANAGERSINGLETON_H_
#define __SHADERMANAGERSINGLETON_H_

#include "ME\system\ShaderClass.h"
#include <vector>
#include <assert.h>

namespace MyEngine
{

//System Class
class MyEngineDLL ShaderManagerSingleton
{
	int m_nShaders; //Number of shaders

	static ShaderManagerSingleton* m_pInstance; // Singleton
	
	std::vector<ShaderClass> m_vShader; //vector of shaders

public:
	/* Singleton accessor */
	static ShaderManagerSingleton* GetInstance(); 

	/* Singleton Release */
	static void ReleaseInstance(void);
	
	/* Compiles a shader program and returns the OpenGL identifier */
	GLuint CompileShader(String a_sVertexShader, String a_sFragmentShader, String a_sName = "");

	/* Asks the manager for the OpenGL identifier for the specified name*/
	GLuint GetShaderID(String a_sName);

	/* Asks the manager for the OpenGL identifier for the specified index*/
	GLuint GetShaderID(int a_nIndex);

	/* Ask for the Index in the vector of the specified shader name*/
	int GetShaderIndex(String a_sName);

private:
	/* Constructor */
	ShaderManagerSingleton(void);

	/* Copy Constructor */
	ShaderManagerSingleton(ShaderManagerSingleton const& other);

	/* Copy Assignment Operator */
	ShaderManagerSingleton& operator=(ShaderManagerSingleton const& other);

	/* Destructor */
	~ShaderManagerSingleton(void);

	/* Release Memory */
	void Release(void);

	/* Initialize variables */
	void Init(void);
};

}
#endif //__SHADERMANAGERSINGLETON_H_