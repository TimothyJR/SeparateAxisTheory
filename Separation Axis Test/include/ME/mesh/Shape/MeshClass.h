/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/07
*/
#ifndef __MESHCLASS_H_
#define __MESHCLASS_H_

#include "ME\system\SystemSingleton.h"
#include "ME\mesh\CameraSingleton.h"
#include "ME\materials\MaterialManagerSingleton.h"
#include "ME\light\LightManagerSingleton.h"
#include "ME\System\ShaderManagerSingleton.h"
#include "ME\Mesh\Shape\ShapeClass.h"
#include <vector>

namespace MyEngine
{

//System Class
class MyEngineDLL MeshClass
{
	bool m_bBinded; //Binded flag
	bool m_bCopy;	//Checks to see if this Mesh is a copy or the original
	int m_nVertices; //Number of Vertices in this Mesh
	int m_nMaterial; //Material index of this mesh

	GLuint m_vao;			//OpenGL Vertex Array Object
	GLuint m_VertexBuffer;	//OpenGL Buffer (Will hold the vertex buffer pointer)
	GLuint m_ColorBuffer;	//OpenGL Buffer (Will hold the color buffer pointer)
	GLuint m_UVBuffer;		//OpenGL Buffer (Will hold the UV buffer pointer)
	GLuint m_NormalBuffer;	//OpenGL Buffer (will hold the Normal Buffer)
	GLuint m_TangentBuffer;	//OpenGL Buffer (will hold the Tangent Buffer)
	GLuint m_BinormalBuffer;//OpenGL Buffer (will hold the Binormal Buffer)
	GLuint m_nShader;	//Index of the shader
	GLuint m_nShaderO;	//Index of the original shader

	CameraSingleton* m_pCamera;				//Pointer to the singleton of CameraSingleton
	MaterialManagerSingleton* m_pMatMngr;	//Material Manager
	LightManagerSingleton* m_pLightMngr;	//Light Manager
	ShaderManagerSingleton* m_pShaderMngr;	//Shader Manager

	String m_sName;	//Identifier of the Mesh
	
	std::vector<vector3> m_vVertexPosition;	//List of Vertices
	std::vector<vector3> m_vVertexColor;	//List of Colors
	std::vector<vector3> m_vVertexNormal;	//List of Normals
	std::vector<vector3> m_vVertexTangent;	//List of Tangents
	std::vector<vector3> m_vVertexBinormal;	//List of Binormals
	std::vector<vector3> m_vVertexUV;		//List of UVS
	
public:
	/* Constructor */
	MeshClass(void);
	/* Copy Constructor */
	MeshClass(MeshClass const& other);
	/* Copy Assignment Operator*/
	MeshClass& operator=(MeshClass const& other);
	/* Destructor */
	~MeshClass(void);

	/* Swaps the contents of the object with another object's content */
	void Swap(MeshClass& other);

	/* Compiles the Mesh for OpenGL 3.X use*/
	void CompileOpenGL3X(void);

	/* Creates the Mesh from a ShapeClass object*/
	int CloneFromShape(ShapeClass& a_oShape);

	/* Asks the Mesh if it has been binded */
	bool GetBinded(void);
	/* Property GetBinded */
	__declspec(property(get = GetBinded)) bool Binded;

	/* Set the shader to the newly loaded shader */
	void SetShaderProgram(String a_sVertexShaderName, String a_sFragmentShaderName, String a_sShaderName);

	/* Sets the shader of the Shape to a loaded shader */
	void SetShaderProgram(String a_sShaderName = "Original");
	/* Property SetShaderProgram */
	__declspec(property(put = SetShaderProgram)) String ShaderProgram;

	/* Sets the material index by index in the material manager */
	void SetMaterialIndex(int a_nIndex);
	/* Gets the material used in the shape by index of the material manager */
	int GetMaterialIndex(void);
	/* Property SetMaterialIndex/GetMaterialIndex */
	__declspec(property(put = SetMaterialIndex, get = GetMaterialIndex)) int MaterialIndex;

	/* Asks the mesh for the Shader program used */
	GLuint GetShaderIndex (void);

	/* Adds a new point to the vector of vertices */
	void AddVertexPosition(vector3 input);

	/* Adds a new color to the vector of vertices */
	void AddVertexColor(vector3 input);

	/* Adds a new normal to the vector of vertices */
	void AddVertexNormal(vector3 input);

	/* Adds a new binormal to the vector of vertices */
	void AddVertexBinormal(vector3 input);

	/* Adds a new tangent to the vector of vertices */
	void AddVertexTangent(vector3 input);

	/* Adds a new uv to the vector of vertices */
	void AddVertexUV(vector3 input);

	/* Asks the shape for the list of vertices */
	std::vector<vector3> GetVertices(void);

	/* Asks the Mesh for its name */
	String GetName(void);
	/* Sets the Mesh's name */
	void SetName(String a_sName);
	/* Property SetName/GetName */
	__declspec(property(put = SetName, get = GetName)) String Name;

	/* Renders the shape asking for its rendering mode (default is GL_TRIANGLES) and the position in the world*/
	virtual void Render(std::vector<matrix4> a_vToWorld);

	/* Renders the shape asking for its rendering mode (default is GL_TRIANGLES) and the position in the world*/
	virtual void Render(float* a_fMatrixArray, int a_nInstances);

private:
	/* Initialize the object's fields */
	void Init(void);
	/* Releases the object from memory */
	void Release(void);
	/* Completes the information missing to create the mesh */
	void CompleteMesh(void);
	/* Disconects OpenGL3.x from the Mesh without releasing the buffers*/
	void DisconnectOpenGL3X(void);
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<MeshClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<MeshClass*>;

}

#endif //__MESHCLASS_H_