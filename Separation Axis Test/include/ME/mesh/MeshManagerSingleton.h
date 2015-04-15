/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2015/03
*/
#ifndef __MESHMANAGERSINGLETON_H_
#define __MESHMANAGERSINGLETON_H_

#include "ME\Mesh\MeshDrawerSingleton.h"
#include "ME\Mesh\PrimitiveManagerSingleton.h"
#include "ME\Mesh\ModelManagerSingleton.h"

namespace MyEngine
{
//MeshManagerSingleton
class MyEngineDLL MeshManagerSingleton
{
	static MeshManagerSingleton* m_pInstance; // Singleton pointer
public:
	MeshDrawerSingleton* m_pMeshDrawer;//pointer to the Mesh Drawer singleton
	PrimitiveManagerSingleton* m_pPrimitiveMngr; //pointer to the Primitive Manager singleton
	ModelManagerSingleton* m_pModelMngr; //pointer to the Model Manager singleton

	/* Gets/Constructs the singleton pointer */
	static MeshManagerSingleton* GetInstance();

	/* Destroys the singleton */
	static void ReleaseInstance(void);

	/* Loads the specified model file */
	MEErrors LoadModel (	String a_sFileName,
							String a_sInstanceName,
							matrix4 a_Matrix = matrix4(1.0),
							int a_nVisibility = 1,
							int a_nCollidable = 1,
							int a_nState = 0);

	/* Loads the specified model file without a thread*/
	MEErrors LoadModelUnthreaded(	String a_sFileName,
									String a_sInstanceName,
									matrix4 a_Matrix = matrix4(1.0),
									int a_nVisibility = 1,
									int a_nCollidable = 1,
									int a_nState = 0);

	/* Loads the specified level file */
	void LoadLevel(String a_sFileName);

	/* Loads an add-on file for the specified instance*/
	MEErrors LoadAddOn(String sInstanceName, String sFileName);

	/* Sets the model matrix of an specific instance finding it by name */
	void SetModelMatrix(matrix4 a_mMatrix, String a_sInstance = "All", bool a_bUpdateOctree = false);

	/* Asks the manager for an specific instance's model to world matrix */
	matrix4 GetModelMatrix(String a_sInstance);

	/* Updates the model manager */
	void Update(bool bCheckCollisions = true);

	/* Renders a mesh on the specified space */
	void AddMeshToRenderList(MeshClass* a_pMesh, matrix4& a_m4ToWorld);

	/* Renders a mesh on the specified space */
	void AddMeshToRenderList(int a_nIndex, matrix4& a_m4ToWorld);

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

	/* Renders the specified instance */
	void AddInstanceToRenderList(String a_sInstance = "ALL");

	/* Renders the list of meshes */
	void Render(void);

	/* Asks the Manager for the number of models */
	int GetNumberOfModels(void);

	/* Asks the Manager for the number of instances */
	int GetNumberOfInstances(void);

	/* Asks the manager for the list of vertices of an specified instance */
	std::vector<vector3> GetVertices(String a_sInstanceName);

	/* Asks the manager if an instance with this name has been created */
	bool IsInstanceCreated(String a_sInstanceName);

private:
	/* Constructor */
	MeshManagerSingleton(void);

	/* Copy Constructor */
	MeshManagerSingleton(MeshManagerSingleton const& other);

	/* Copy Assignment Operator */
	MeshManagerSingleton& operator=(MeshManagerSingleton const& other);

	/* Destructor */
	~MeshManagerSingleton(void);

	/* Initializates the objects fields */
	void Init(void);

	/* Releases the objects memory */
	void Release(void);
};

}

#endif //__MESHMANAGERSINGLETON_H_