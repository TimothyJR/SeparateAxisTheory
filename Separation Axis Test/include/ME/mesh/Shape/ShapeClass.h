/*--------------------------------------------------------------------------------------------------
Created by Alberto Bobadilla (labigm@rit.edu) in 2013
--------------------------------------------------------------------------------------------------*/
#ifndef __SHAPECLASS_H_
#define __SHAPECLASS_H_
#include "ME\system\SystemSingleton.h"
#include "ME\mesh\CameraSingleton.h"
#include "ME\materials\MaterialManagerSingleton.h"
#include "ME\light\LightManagerSingleton.h"
#include "ME\System\ShaderManagerSingleton.h"
#include <vector>

namespace MyEngine
{

class MyEngineDLL ShapeClass
{
protected:
	bool m_bBinded;		//Binded flag
	bool m_bVisible;	//Visibility flag
	bool m_bCollidable;	//Collidable flag
	int m_nVertices;	//Number of vertices in the shape
	int m_nMaterialIndex;//Material of the Shape

	GLuint m_vao;			//OpenGL Vertex Array Object
	GLuint m_VertexBuffer;	//OpenGL Buffer (Will hold the vertex buffer pointer)
	GLuint m_ColorBuffer;	//OpenGL Buffer (Will hold the color buffer pointer)
	GLuint m_UVBuffer;		//OpenGL Buffer (Will hold the UV buffer pointer)
	GLuint m_NormalBuffer;	//OpenGL Buffer (will hold the Normal Buffer)
	GLuint m_TangentBuffer;	//OpenGL Buffer (will hold the Tangent Buffer)
	GLuint m_BinormalBuffer;//OpenGL Buffer (will hold the Binormal Buffer)

	CameraSingleton* m_pCamera;				//Pointer to the singleton of CameraSingleton
	MaterialManagerSingleton* m_pMatMngr;	//Material Manager
	LightManagerSingleton* m_pLightMngr;	//Light Manager
	ShaderManagerSingleton* m_pShaderMngr;	//Shader Manager
	
	matrix4 m_m4ToWorld; //GLM Model Matrix

	String m_sName;					//Name of the Shape
	String m_sShaderProgramName;	//Name of the shader program
	String m_sOriginalShaderProgram;//Name of the original shader program
	
	std::vector<vector3> m_vVertexPosition;	//List of Vertices
	std::vector<vector3> m_vVertexColor;	//List of Colors
	std::vector<vector3> m_vVertexNormal;	//List of Normals
	std::vector<vector3> m_vVertexTangent;	//List of Tangents
	std::vector<vector3> m_vVertexBinormal;	//List of Binormals
	std::vector<vector3> m_vVertexUV;		//List of UVS

public:
	/* Constructor */
	ShapeClass();
	/* Copy Constructor */
	ShapeClass(const ShapeClass& other);
	/* Copy Assignment Operator */
	virtual ShapeClass& operator=(const ShapeClass& other);

	/* Destructor */
	~ShapeClass(void);
	
	/* Release */
	void Release (bool a_bReleaseOpenGL = false);

	/* Clears OpenGL Buffers */
	void ReleaseOpenGL(void);

	/* Initializes the OpenGL vao, Buffer and Program */
	bool CompileOpenGL3X(void);
	
	/* Sets the Model Matrix in base of another Matrix (No Parameters sets the Identity) */
	void SetModelMatrix(const matrix4 a_mToWorld = matrix4(1.0f));
	
	/* Returns the Model Matrix */
	matrix4& GetModelMatrix(void);

	/* Property SetModelMatrix/GetModelMatrix */
	__declspec(property(put = SetModelMatrix, get = GetModelMatrix)) matrix4 ModelMatrix;

	/* Sets the name of the shape */
	void SetName(String input);
	/* Asks the shape for its name */
	String GetName(void);
	/* Property SetName/GetName */
	__declspec(property(put = SetName, get = GetName)) String Name;

	/* Swaps the content with other shape */
	void Swap(ShapeClass& other);
	
	/* Adds a new point to the vector of vertices */
	void AddVertexPosition(vector3 input);
	/* Adds a new point to the vector of vertices */
	void AddVertexPosition(const float x, const float y, const float z);

	/* Adds a new color to the vector of vertices */
	void AddVertexColor(vector3 input);
	/* Adds a new color to the vector of vertices */
	void AddVertexColor(const float r, const float g, const float b);

	/* Adds a new normal to the vector of vertices */
	void AddVertexNormal(vector3 input);
	/* Adds a new normal to the vector of vertices */
	void AddVertexNormal(const float x, const float y, const float z);

	/* Adds a new binormal to the vector of vertices */
	void AddVertexBinormal(vector3 input);
	/* Adds a new binormal to the vector of vertices */
	void AddVertexBinormal(const float x, const float y, const float z);

	/* Adds a new tangent to the vector of vertices */
	void AddVertexTangent(vector3 input);
	/* Adds a new tangent to the vector of vertices */
	void AddVertexTangent(const float x, const float y, const float z);

	/* Adds a new uv to the vector of vertices */
	void AddVertexUV(vector3 input);
	/* Adds a new uv to the vector of vertices */
	void AddVertexUV(const float u, const float v, const float w = 0);

	/* Releases the vectors (leaving the OpenGL buffer initializated) */
	void ReleaseVectors(void);

	/* Specifies if the model has been binded to a shader program */
	bool IsBinded(void);

	/* Clones another shape */
	bool InstanceFromShape(const ShapeClass& other);

	/* Completes the triangle information */
	void CompleteTriangleInfo(bool a_bAverageNormals = false);
	/* Calculates the missing tangets */
	void CalculateTangents(void);

	/* Set the shader to the newly load shader */
	void SetShaderProgram(String a_sVertexShaderName, String a_sFragmentShaderName, String a_sShaderName);

	/* Sets the shader of the Shape to a loaded shader */
	void SetShaderProgram(String a_sShaderName = "Original");

	/* Sets the collidable flag */
	void SetCollidable(bool a_bCollidable);
	/* Asks the shape for the collidable flag's value */
	bool GetCollidable(void);

	/* Sets the visibility flag */
	void SetVisible(bool a_bVisible);

	/* Asks the shape for the visibility flag's value */
	bool GetVisible(void);

	/* Sets the material of the shape by name */
	void SetMaterial(String a_sMaterialName);

	/* Sets the material index by index in the material manager */
	void SetMaterialIndex(int a_nIndex);
	/* Gets the material used in the shape by index of the material manager */
	int GetMaterialIndex(void);

	int GetVertexTotal(void);
	GLuint GetVAOIndex(void);
	GLuint GetVertexIndex(void);
	GLuint GetColorIndex(void);
	GLuint GetUVIndex(void);
	GLuint GetNormalIndex(void);
	GLuint GetTangentIndex(void);
	GLuint GetBinormalIndex(void);

	int GetShaderProgramIndex(void);

	/* Asks the shape for the list of vertices */
	std::vector<vector3> GetVertices(void);

	std::vector<vector3> GetColors(void);
	std::vector<vector3> GetUVs(void);
	std::vector<vector3> GetNormals(void);
	std::vector<vector3> GetTangents(void);
	std::vector<vector3> GetBinormals(void);

	/* Renders the shape asking for its rendering mode (default is GL_TRIANGLES) and the position in the world*/
	virtual void Render(GLenum mode = GL_TRIANGLES, matrix4 a_mToWorld = matrix4(1.0f));
	/* Renders the shape asking for its position in the world and a color */
	virtual void Render( matrix4 a_mToWorld, vector3 a_vColor);
	/* Renders the shape asking for its position in the world */
	virtual void Render( matrix4 a_mToWorld);

protected:
	/* Initialize the object's fields */
	void Init(void);
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<ShapeClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<ShapeClass*>;

}
#endif //_SHAPECLASS_H