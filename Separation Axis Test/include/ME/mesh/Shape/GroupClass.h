/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/05
*/
#ifndef __GROUP_H_
#define __GROUP_H_

#include "ME\mesh\shape\ShapeClass.h"
#include "ME\materials\MaterialManagerSingleton.h"
#include "ME\mesh\Animation\FrameClass.h"
#include "ME\mesh\Animation\AnimationClass.h"
#include "ME\physics\BoundingObjectClass.h"
#include "ME\Mesh\Shape\MeshManagerSingleton.h"
#include "ME\mesh\debug\AxisClass.h"

namespace MyEngine
{

class MyEngineDLL GroupClass
{	
	bool m_bVisible;	//Visibility flag
	bool m_bVisibleBO;	//Debug flag
	bool m_bCollidable;	//Collision flag
	bool m_bGlobalized;	//Global coordinates flag
	bool m_bModified;	//Modified flag

	int m_nShapes;		//number of shapes in this group
	int m_nHP;			//Hit Points of this group
	int m_nFrames;		//Frames in this group
	
	SystemSingleton* m_pSystem;	//System pointer
	MaterialManagerSingleton* m_pMatMngr;//Material Manager Pointer
	MeshManagerSingleton* m_pMeshMngr;//Mesh Manager Pointer
	GroupClass* m_pParent;	//Parent group pointer
	FrameClass* m_pFrame;	//List of frames of animation in the group
	BoundingObjectClass* m_pBO;	//Bounding Object of the group
	AxisClass* m_pAxis;		//Axis of the group
	
	String m_sName;			//Name of the group
	String m_sParent;		//Name of the parent of the group

	vector3 m_v3Pivot;		//Point in which the point is going to rotate around
	matrix4 m_m4ToWorld;	//Model to world matrix

	std::vector<ShapeClass> m_vShape;//List of shapes that compose the group
	std::vector<int> m_vMesh; //List of Meshes that compose a group
	std::vector<int> m_vMeshM; //List of Meshes that compose a group when modified
	matrix4 m_m4Mesh; //List of matrices for all Meshes that compose a group

public:
	GroupClass(void);
	GroupClass(const GroupClass& other);
	GroupClass& operator=(const GroupClass& other);
	~GroupClass(void);
	
	void Release(void);

	void Swap(GroupClass& other);

	int GetNumberOfShapes(void); //Returns the number of shapes in the vector of shapes
	String GetName(void); //Return the name of the group
	void SetName(String a_sName);
	__declspec(property(get = GetName, put = SetName)) String Name; //Name Property

	ShapeClass& ReturnShape(int a_nIndex);//Returns the shape in the given index

	void AddShape(String a_sName);

	void CompileOpenGL3X( void );

	int IdentifyShapeByMaterial( String a_sName);

	bool CloneFromGroup(const GroupClass& other);

	void ReleaseOpenGL(); //DeleteOpenGL

	void CompleteMissingShapeInfo(bool a_bAverageNormals = false);

	void SwapMaterial( String sOldMaterialName, String sNewMaterialName);

	void SetShaderProgram(String a_sVertexShaderName, String a_sFragmentShaderName, String a_sShaderName);
	void SetShaderProgram(String a_sShaderName = "Original");

	void SetMaterial(String a_sMaterialName);

	void SetPivot(vector3 a_v3Pivot);

	void SetParent(GroupClass* a_pParent);

	void SetHP(int a_nHP);
	int GetHP(void);

	void SetCollidable(bool a_bCollidable);
	bool GetCollidable(void);

	void SetVisible(bool a_bVisible);
	bool GetVisible(void);

	String GetParentName(void);
	void SetParentName(String a_sParent);

	void SetGlobalized(bool a_bGlobalized);

	int GetNumberOfFrames(void);

	void GenerateFrames(int a_nFrames);

	void ReleaseFrames(void);

	void SetFrame(int nFrame, vector3 a_v3Translation, vector3 a_v3Rotation, vector3 a_v3Scale, bool a_bVisible, bool a_bKeyframe);

	void SetPosX( float fValue, int nFrame);
	void SetPosY( float fValue, int nFrame);
	void SetPosZ( float fValue, int nFrame);

	void SetAngX( float fValue, int nFrame);
	void SetAngY( float fValue, int nFrame);
	void SetAngZ( float fValue, int nFrame);

	void SetSizeX( float fValue, int nFrame);
	void SetSizeY( float fValue, int nFrame);
	void SetSizeZ( float fValue, int nFrame);

	void SetVisible( bool bValue, int nFrame);
	void SetKeyframe( bool bValue, int nFrame);
	void SetVisibleBO(bool a_bVisibleBO);
	void SetVisibleAxis(bool a_bVisible);

	void ComputeMatrix(void);

	void Interpolate(int a_nFrames);

	glm::mat4& GetModelMatrix(void);

	matrix4 TraverseHierarchy(GroupClass* group, int nFrame = 0);

	std::vector<vector3> GetVertices(int a_nShape = -1);

	void Update(int a_nFrame, matrix4 a_mToWorld);

	BoundingObjectClass* GetBoundingObject(void);
	void CompileBoundingObject(void);

	bool IsColliding( vector3 a_v3RayOrigin, vector3 a_v3RayDirection, float& a_fDistance );

	void DisconectAnimationFrames(void);

	void SetModelMatrix(matrix4 a_nMatrix);

	void Render(int a_nFrame = 0);//Render all shapes
	void RenderBO(bool a_bForce = false);//Render all debug
	void RenderAxis(bool a_bForce = false);//Render all debug

private:
	void Init(void);
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<GroupClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<GroupClass*>;

}
#endif //__GROUP_H__