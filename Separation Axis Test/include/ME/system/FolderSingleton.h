/*--------------------------------------------------------------------------------------------------
Created by Alberto Bobadilla (labigm@rit.edu) in 2013
--------------------------------------------------------------------------------------------------*/
#ifndef __FOLDERSINGLETON_H_
#define __FOLDERSINGLETON_H_

#include <string>
#include <Windows.h>
#include "ME\system\MeDefinitions.h"

namespace MyEngine
{

typedef std::string String;
class MyEngineDLL FolderSingleton
{
	String m_sRoot;
	String m_sData;
	String m_s3DS;
	String m_sBTO;
	String m_sPOM;
	String m_sFBX;
	String m_sOBJ;
	String m_sLVL;
	String m_sTextures;
	String m_sShaders;
	String m_sAddOn;

	static FolderSingleton* m_pInstance;
public:
	static FolderSingleton* GetInstance(void);
	static void ReleaseInstance();	//Singleton Destructor
	
	void SetFolderRoot(String input);
	String GetFolderRoot(void);
	__declspec(property(put = SetFolderRoot, get = GetFolderRoot)) String Root;

	void SetFolderData(String input);
	String GetFolderData(void);
	__declspec(property(put = SetFolderData, get = GetFolderData)) String Data;

	void SetFolderM3DS(String input);
	String GetFolderM3DS(void);
	__declspec(property(put = SetFolderM3DS, get = GetFolderM3DS)) String M3DS;

	void SetFolderMBTO(String input);
	String GetFolderMBTO(void);
	__declspec(property(put = SetFolderMBTO, get = GetFolderMBTO)) String MBTO;

	void SetFolderMPOM(String input);
	String GetFolderMPOM(void);
	__declspec(property(put = SetFolderMPOM, get = GetFolderMPOM)) String MPOM;

	void SetFolderMFBX(String input);
	String GetFolderMFBX(void);
	__declspec(property(put = SetFolderMFBX, get = GetFolderMFBX)) String MFBX;

	void SetFolderMOBJ(String input);
	String GetFolderMOBJ(void);
	__declspec(property(put = SetFolderMOBJ, get = GetFolderMOBJ)) String MOBJ;

	void SetFolderLVL(String input);
	String GetFolderLVL(void);
	__declspec(property(put = SetFolderLVL, get = GetFolderLVL)) String LVL;

	void SetFolderTextures(String input);
	String GetFolderTextures(void);
	__declspec(property(put = SetFolderTextures, get = GetFolderTextures)) String Textures;

	void SetFolderShaders(String input);
	String GetFolderShaders(void);
	__declspec(property(put = SetFolderShaders, get = GetFolderShaders)) String Shaders;

	void SetFolderAddOn(String input);
	String GetFolderAddOn(void);
	__declspec(property(put = SetFolderAddOn, get = GetFolderAddOn)) String AddOn;

private:
	FolderSingleton(void);
	FolderSingleton(const FolderSingleton& other);
	FolderSingleton& operator=(const FolderSingleton& other);
	~FolderSingleton(void);

	void Init(void);
	void Release(void);

	String GetRealProgramPath(void);
};

}

#endif //__FOLDERSINGLETON_H_
