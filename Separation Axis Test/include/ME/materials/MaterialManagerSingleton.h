/*--------------------------------------------------------------------------------------------------
	Created by Alberto Bobadilla (labigm@rit.edu) in 2013
--------------------------------------------------------------------------------------------------*/
#ifndef __MATERIALMANAGERSINGLETON_H_
#define __MATERIALMANAGERSINGLETON_H_

#include "ME\system\SystemSingleton.h"
#include "ME\materials\MaterialClass.h"


namespace MyEngine
{

class MyEngineDLL MaterialManagerSingleton
{
	/* 
		The material vector is public for accessing speed reasons its more secure to have it as private
		and provide the right accessors for it, but its call so often that it creates overhead the other way
	*/
public:
	std::vector<MaterialClass*> m_vMaterial; //Materials Vector
	

	static MaterialManagerSingleton* GetInstance(); // Singleton Accessor
	static void ReleaseInstance(void);	//Singleton Destructor

	int IdentifyMaterial(String a_sName); //Identifies the material, if found return its index -1 otherwise

	//returns the index number of the material if it created a new one, otherwise it returns the index
	int AddMaterial(String a_sName);
	int AddMaterial(MaterialClass input);

	int GetNumberOfMaterials(void); //Return the number of materials in the vector

	void ReloadMaps(void); //Tries to load all the textures for all of the materials

	MaterialClass GetMaterialCopy(String sName);

private:
	MaterialManagerSingleton(void); // Constructor
	MaterialManagerSingleton(MaterialManagerSingleton const& other); //Copy Constructor
	MaterialManagerSingleton& operator=(MaterialManagerSingleton const& other); // Copy Assignment Operator
	~MaterialManagerSingleton(void); // Destructor

	void Release(void); // Release Memory
	void Init(void); // Initialize variables
	
	SystemSingleton* m_pSystem; //System class Pointer
	static MaterialManagerSingleton* m_pInstance; // Singleton Pointer
};

}
#endif //_MATERIALMANAGERSINGLETON_H