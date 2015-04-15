/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/05
*/
#ifndef __LIGHTCLASS_H_
#define __LIGHTCLASS_H_

#include "ME\system\SystemSingleton.h"

namespace MyEngine
{

class MyEngineDLL LightClass
{
	//Private Fields
	float m_fIntensity;
	vector3 m_vPosition;
	vector3 m_vColor;
public:
	//The Big 3
	/* 	Constructor */
	LightClass(void);
	/* 	Constructor */
	LightClass( vector3 a_vPosition, vector3 a_vColor, float a_vIntensity);
	/* Copy Constructor */
	LightClass(const LightClass& other);
	/* Copy Assignment operator */
	LightClass& operator=(const LightClass& other);
	/* Destructor */
	~LightClass(void);
	
	//Helper Methods
	/* Swapts the information of two objects*/
	void Swap(LightClass& other);

	//Accessors
	/* Sets the position of the light */
	void SetPosition(vector3 a_vPosition);
	/* Gets the position of the light */
	vector3 GetPosition(void);
	/* Property SetPosition/GetPosition */
	__declspec(property(put = SetPosition, get = GetPosition)) vector3 Position;

	/* Sets the color of the light */
	void SetColor(vector3 a_vColor);
	/* Gets the color of the light*/
	vector3 GetColor(void);
	/* Property SetColor/GetColor */
	__declspec(property(put = SetColor, get = GetColor)) vector3 Color;

	/* Set Intensity of the light */
	void SetIntensity(float a_fIntensity);
	/* Gets the intensity of the light*/
	float GetIntensity(void);
	/* Property SetIntensity/GetIntensity */
	__declspec(property(put = SetIntensity, get = GetIntensity)) float Intensity;

protected:
	//Protected methods
	/* Initializates the object values*/
	void Init(void);
	/* Releases the object*/
	void Release(void);
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<LightClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<LightClass*>;

}
#endif //__LIGHT_H__