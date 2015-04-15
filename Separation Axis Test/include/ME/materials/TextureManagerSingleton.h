/*--------------------------------------------------------------------------------------------------
Created by Alberto Bobadilla (labigm@rit.edu) in 2013
--------------------------------------------------------------------------------------------------*/
#ifndef __TEXTUREMANAGERSINGLETON_H_
#define __TEXTUREMANAGERSINGLETON_H_

#include "ME\materials\TextureClass.h"
#include <vector>

namespace MyEngine
{

class MyEngineDLL TextureManagerSingleton
{
public:
	std::vector<TextureClass*> m_vTextures; //Vector of Textures

	static TextureManagerSingleton* GetInstance(); //Singleton Accessor
	static void ReleaseInstance();	//Singleton Destructor
	
	int IdentifyTexure(String sName); //Returns the ID number of the texture, -1 if it does not exist.
	int LoadTexture(String sName);	//Loads a texture and returns the ID of where its stored in the vector

	int GetNumberOfTextures(void); //Returns the number of texures in the vector
	GLuint ReturnGLIndex(int a_nIndex); //Returns the OpenGL Buffer ID in the vector location of the provided Index
private:
	static TextureManagerSingleton* m_pInstance; //Singleton
	TextureManagerSingleton(void); //Constructor	
	TextureManagerSingleton(const TextureManagerSingleton& other); //Copy Constructor
	TextureManagerSingleton& operator=(const TextureManagerSingleton& other);//Copy Assignment Operator
	~TextureManagerSingleton(void); //Destructor

	void Release(void); //Release the fields in the class
	void Init(void); //Initialize variables
};

}
#endif __TEXTUREMANAGERSINGLETON_H_