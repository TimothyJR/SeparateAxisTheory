/*--------------------------------------------------------------------------------------------------
Created by Alberto Bobadilla (labigm@rit.edu) in 2013
--------------------------------------------------------------------------------------------------*/
#ifndef __PRIMITIVEWIRECLASS_H_
#define __PRIMITIVEWIRECLASS_H_

#include "ME\mesh\shape\ShapeClass.h"

namespace MyEngine
{

class MyEngineDLL PrimitiveWireClass : public ShapeClass
{
	typedef ShapeClass super;

public:
	/* Constructor */
	PrimitiveWireClass();
	/* Copy Constructor */
	PrimitiveWireClass(const PrimitiveWireClass& other);
	/* Copy Assignment Operator */
	virtual PrimitiveWireClass& operator=(const PrimitiveWireClass& other);

	/* Destructor */
	~PrimitiveWireClass(void);

	/* Generates an Axis shape */
	void GenerateAxis(void);
	/* Generates a cube shape */
	void GenerateCube(float a_fSize, vector3 a_vColor);
	/* Generates a cone shape */
	void GenerateCone(float a_fRadius, float a_fHeight, int a_nSubdivisions, vector3 a_vColor);
	/* Generates a cylinder shape */
	void GenerateCylinder(float a_fRadius, float a_fHeight, int a_nSubdivisions, vector3 a_vColor);
	/* Generates a Tube shape */
	void GenerateTube(float a_fOuterRadius, float a_fInnerRadius, float a_fHeight, int a_nSubdivisions, vector3 a_vColor);
	/* Generates a sphere shape */
	void GenerateSphere(float a_fRadius, int a_nSubdivisions, vector3 a_vColor);
	
	/* Renders the primitive asking for its rendering mode (default is GL_TRIANGLES)*/
	virtual void Render(GLenum mode = GL_TRIANGLES);
	/* Renders the primitive asking for its position in the world and a color */
	virtual void Render(matrix4 a_mModelMatrix, vector3 a_vColor);
	/* Renders the primitive asking for its position in the world */
	virtual void Render(matrix4 a_mModelMatrix);

private:
	/* Compiles the object with an specific color and name */
	void CompileObject(vector3 a_vColor, String a_sName);
	/* Adds a quad to the list points in the buffer to be compiled*/
	void AddQuad(vector3 a_vBottomLeft, vector3 a_vBottomRight, vector3 a_vTopLeft, vector3 a_vTopRight);
	/* Adds a tri to the list points in the buffer to be compiled*/
	void AddTri(vector3 a_vBottomLeft, vector3 a_vBottomRight, vector3 a_vTop);
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<PrimitiveWireClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<PrimitiveWireClass*>;

}

#endif //__PRIMITIVEWIRECLASS_H_