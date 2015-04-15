/*
From: http://www.opengl-tutorial.org/
Date: 2014/05
*/
#ifndef __SHADERCOMPILER_H_
#define __SHADERCOMPILER_H_

#include <GL\glew.h>
#include <GL\wglew.h>
#include "ME\system\MeDefinitions.h"



MyEngineDLL GLuint LoadShaders(const char * vertex_file_path,const char * fragment_file_path);


#endif //__SHADER_H__
