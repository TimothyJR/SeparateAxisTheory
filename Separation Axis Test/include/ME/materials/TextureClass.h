/*--------------------------------------------------------------------------------------------------
Created by Alberto Bobadilla (labigm@rit.edu) in 2013
--------------------------------------------------------------------------------------------------*/
#ifndef __TEXTURECLASS_H_
#define __TEXTURECLASS_H_

#include "ME\system\SystemSingleton.h"
#include "SOIL\SOIL.h"

namespace MyEngine
{

class MyEngineDLL TextureClass
{
	GLuint m_nGLTextureID;
	SystemSingleton* m_pSystem;
	String m_sName;
	String m_sFileName;
public:
	TextureClass(void); //Constructor
	TextureClass(const TextureClass& other); //Copy Constructor
	TextureClass& operator=(const TextureClass& other); //Copy Assignment Operator
	void Release(void); //Release the fields in the class
	~TextureClass(void); //Destructor

	void LoadTexture(String a_sFileName);
//Accessors
	GLuint GetGLTextureID(void); //Returns the OpenGL Identifier
	__declspec(property(get = GetGLTextureID)) GLuint GLTextureID;
	String GetTextureName(void); //Returns the name of the texture
	__declspec(property(get = GetTextureName)) String TextureName;
	String GetTextureFileName(void); //Returns the filename of the texture
	__declspec(property(get = GetTextureFileName)) String TextureFileName;

private:
	void Init(void); //Initializes the variables
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<TextureClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<TextureClass*>;

}
#endif //_TEXTURECLASS_H
