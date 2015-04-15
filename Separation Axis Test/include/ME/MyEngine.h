/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/07
*/
#ifndef __MYENGINE_H_
#define __MYENGINE_H_

#pragma warning( disable : 4251 )

#include "ME\system\MEDefinitions.h" //Definitions of MyEngine
#include "ME\system\SystemSingleton.h" //System related methods

#include "ME\system\WindowClass.h" //WinAPI encapsulated methods
#include "ME\system\FileReaderClass.h" //Reads an object and returns lines as strings
#include "ME\system\FolderSingleton.h" //Sets the working directory
#include "ME\system\GLSystemSingleton.h" //OpenGL 2.0 and 3.0 initialization

#include "ME\system\ShaderManagerSingleton.h" //Shader Manager, loads and manages different shader files
#include "ME\system\ShaderCompiler.h" //Compiles the shader objects

#include "ME\light\LightManagerSingleton.h" //Manages the light sources of the world

#include "ME\materials\MaterialManagerSingleton.h" //Manages the materials
#include "ME\materials\TextureManagerSingleton.h" //Manages the Textures, loads, display and release

#include "ME\Animation\AnimationClass.h" //Manages the animations in a model
#include "ME\Animation\FrameClass.h" //Manages the transformations of a model in a frame of time
#include "ME\Animation\SequenceClass.h" //Manages the starting and ending point of an animation
#include "ME\Animation\StateClass.h" //Manages the current state of the model

#include "ME\Mesh\AxisClass.h"  //Creates an Axis object
#include "ME\Mesh\GridClass.h"  //Creates a frame object
#include "ME\Mesh\LineManagerSingleton.h" //Manages lines
#include "ME\Mesh\MeshDrawerSingleton.h" //Singleton for drawing Meshes
#include "ME\Mesh\MeshManagerSingleton.h" //Singleton for drawing Meshes
#include "ME\Mesh\PrimitiveManagerSingleton.h" //Creates and manages the Primitive objects
#include "ME\Mesh\ModelManagerSingleton.h" //Manages models and instances

#include "ME\Camera\CameraSingleton.h" //Creates and manages the camera object for the world

static void ReleaseAllSingletons(void)
{
	LightManagerSingleton::ReleaseInstance();
	MaterialManagerSingleton::ReleaseInstance();
	TextureManagerSingleton::ReleaseInstance();
	LineManagerSingleton::ReleaseInstance();
	ModelManagerSingleton::ReleaseInstance();
	CameraSingleton::ReleaseInstance();
	OctreeSingleton::ReleaseInstance();
	FolderSingleton::ReleaseInstance();
	GLSystemSingleton::ReleaseInstance();
	ShaderManagerSingleton::ReleaseInstance();
	SystemSingleton::ReleaseInstance();
	PrimitiveManagerSingleton::ReleaseInstance();
	MeshManagerSingleton::ReleaseInstance();
	MeshDrawerSingleton::ReleaseInstance();
}

#endif //__MYENGINE_H__