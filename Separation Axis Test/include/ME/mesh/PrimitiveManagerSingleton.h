/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/12
*/
#ifndef __PRIMITIVEMANAGERSINGLETON_H_
#define __PRIMITIVEMANAGERSINGLETON_H_

#include "ME\Mesh\PrimitiveClass.h"
#include "ME\Mesh\PrimitiveWireClass.h"
#include "ME\Mesh\MeshDrawerSingleton.h"

namespace MyEngine
{

//System Class
class MyEngineDLL PrimitiveManagerSingleton
{
	static PrimitiveManagerSingleton* m_pInstance; // Singleton of the class
	
	MeshClass* m_pAxis; //Axis primitive
	MeshClass* m_pPlane; //Plane primitive
	MeshClass* m_pCube; //Cube primitive
	MeshClass* m_pCubeWire; //Cube Wire primitive
	MeshClass* m_pCone; //Cone primitive
	MeshClass* m_pConeWire; //Cone Wire primitive
	MeshClass* m_pCylinder; //Cylinder primitive
	MeshClass* m_pCylinderWire; //Cylinder Wire primitive
	MeshClass* m_pTube; //Tube primitive
	MeshClass* m_pTubeWire; //Tube Wire primitive
	MeshClass* m_pTorus; //Torus primitive
	MeshClass* m_pTorusWire; //Torus Wire primitive
	MeshClass* m_pSphere;//Sphere primitive
	MeshClass* m_pSphereWire;//Sphere Wire primitive

	MeshDrawerSingleton* m_pMeshDrawer;

public:
	/* Gets/Constructs the singleton pointer */
	static PrimitiveManagerSingleton* GetInstance();
	/* Destroys the singleton */
	static void ReleaseInstance(void); //Singleton Release

	/* Render the axis on the specified position */
	void AddAxisToQueue(matrix4 a_mModelToWorld);

	/* Renders the cube on the specified position*/
	void AddPlaneToQueue(matrix4 a_mModelToWorld, vector3 a_v3Color = MEDEFAULT);

	/* Renders the cube on the specified position*/
	void AddCubeToQueue(matrix4 a_mModelToWorld, vector3 a_v3Color = MEDEFAULT, int a_RenderOption = MERENDER::SOLID | MERENDER::WIRE);

	/* Renders the cone on the specified position*/
	void AddConeToQueue(matrix4 a_mModelToWorld, vector3 a_v3Color = MEDEFAULT, int a_RenderOption = MERENDER::SOLID | MERENDER::WIRE);

	/* Renders the cylinder on the specified position*/
	void AddCylinderToQueue(matrix4 a_mModelToWorld, vector3 a_v3Color = MEDEFAULT, int a_RenderOption = MERENDER::SOLID | MERENDER::WIRE);

	/* Renders the tube on the specified position*/
	void AddTubeToQueue(matrix4 a_mModelToWorld, vector3 a_v3Color = MEDEFAULT, int a_RenderOption = MERENDER::SOLID | MERENDER::WIRE);

	/* Renders the torus on the specified position*/
	void AddTorusToQueue(matrix4 a_mModelToWorld, vector3 a_v3Color = MEDEFAULT, int a_RenderOption = MERENDER::SOLID | MERENDER::WIRE);
	
	/* Renders the sphere on the specified position*/
	void AddSphereToQueue(matrix4 a_mModelToWorld, vector3 a_v3Color = MEDEFAULT, int a_RenderOption = MERENDER::SOLID | MERENDER::WIRE);
	
private:
	/* Constructor */
	PrimitiveManagerSingleton(void);
	/* Copy constructor */
	PrimitiveManagerSingleton(PrimitiveManagerSingleton const& other);
	/* Copy Assignment operator */
	PrimitiveManagerSingleton& operator=(PrimitiveManagerSingleton const& other);
	/* Destructor */
	~PrimitiveManagerSingleton(void);

	/* Releases the objects memory */
	void Release(void);
	/* Initializates the objects fields */
	void Init(void);
};

}

#endif //__PRIMITIVEMANAGERSINGLETON_H_