/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/07
*/
#ifndef __UnitaryPrimitiveSingleton_H_
#define __UnitaryPrimitiveSingleton_H_

#include "ME\Mesh\shape\PrimitiveWireClass.h"

namespace MyEngine
{

//System Class
class MyEngineDLL UnitaryPrimitiveSingleton
{
	static UnitaryPrimitiveSingleton* m_pInstance; // Singleton of the class
	PrimitiveWireClass* m_pWireAxis; //Wire Axis primitive
	PrimitiveWireClass* m_pWireCube; //Wire Cube primitive
	PrimitiveWireClass* m_pWireSphere;//Wire sphere primitive
public:
	/* Gets/Constructs the singleton pointer */
	static UnitaryPrimitiveSingleton* GetInstance();
	/* Destroys the singleton */
	static void ReleaseInstance(void); //Singleton Release

	/* Renders the axis on the specified position*/
	void RenderWireAxis(matrix4 a_mModelToWorld);
	/* Renders the cube on the specified position*/
	void RenderWireCube(matrix4 a_mModelToWorld, vector3 a_v3Color);
	/* Renders the sphere on the specified position*/
	void RenderWireSphere(matrix4 a_mModelToWorld, vector3 a_v3Color);
	
private:
	/* Constructor */
	UnitaryPrimitiveSingleton(void);
	/* Copy constructor */
	UnitaryPrimitiveSingleton(UnitaryPrimitiveSingleton const& other);
	/* Copy Assignment operator */
	UnitaryPrimitiveSingleton& operator=(UnitaryPrimitiveSingleton const& other);
	/* Destructor */
	~UnitaryPrimitiveSingleton(void);

	/* Releases the objects memory */
	void Release(void);
	/* Initializates the objects fields */
	void Init(void);
};

}

#endif //__EXAMPLECLASS_H__