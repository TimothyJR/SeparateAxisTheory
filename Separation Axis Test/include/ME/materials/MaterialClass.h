/*--------------------------------------------------------------------------------------------------
Created by Alberto Bobadilla (labigm@rit.edu) in 2013
--------------------------------------------------------------------------------------------------*/
#ifndef __MATERIALCLASS_H_
#define __MATERIALCLASS_H_

#include "ME\system\SystemSingleton.h"
#include "ME\materials\TextureManagerSingleton.h"

namespace MyEngine
{

class MyEngineDLL MaterialClass
{
	SystemSingleton* m_pSystem;
	TextureManagerSingleton* m_pTextureMngr;

	GLuint m_nMapDiffuse;
	GLuint m_nMapNormal;
	GLuint m_nMapSpecular;

	vector3 m_vKd;

	String m_sName;
	String m_sMapDiffuse;
	String m_sMapNormal;
	String m_sMapSpecular;
	
public:
	MaterialClass(String a_sName);
	MaterialClass(const MaterialClass& other);
	MaterialClass& operator=(const MaterialClass& other);
	~MaterialClass(void);
	
	void Release(void);

	void	SetName(String a_sName);
	String	GetName(void);
	__declspec(property(put = SetName, get = GetName)) String Name;

	void	SetDiffuseMapName(String a_sFileName);
	String	GetDiffuseMapName(void);
	__declspec(property(put = SetDiffuseMapName, get = GetDiffuseMapName)) String DiffuseMapName;

	void	SetNormalMapName(String a_sFileName);
	String	GetNormalMapName(void);
	__declspec(property(put = SetNormalMapName, get = GetNormalMapName)) String NormalMapName;

	void	SetSpecularMapName(String a_sFileName);
	String	GetSpecularMapName(void);
	__declspec(property(put = SetSpecularMapName, get = GetSpecularMapName)) String SpecularMapName;

	GLuint	GetDiffuseMap(void);
	__declspec(property(get = GetDiffuseMap)) GLuint DiffuseMap;

	GLuint	GetNormalMap(void);
	__declspec(property(get = GetNormalMap)) GLuint NormalMap;

	GLuint	GetSpecularMap(void);
	__declspec(property(get = GetSpecularMap)) GLuint SpecularMap;

	void	SetDiffuse(vector3 a_Kd);
	vector3	GetDiffuse(void);
	__declspec(property(put = SetDiffuse, get = GetDiffuse)) vector3 Kd;

	MEErrors LoadDiffuse(String a_sFileName);

	MEErrors LoadNormal(String a_sFileName);

	MEErrors LoadSpecular(String a_sFilename);

	MEErrors LoadMaps(void);
private:
	void Init(void);
};

}
#endif //_EMPTY_H
