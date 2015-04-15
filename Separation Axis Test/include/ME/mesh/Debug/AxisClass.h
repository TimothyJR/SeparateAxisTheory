/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/10
*/
#ifndef __AxisClass_H_
#define __AxisClass_H_

#include "ME\Mesh\shape\UnitaryPrimitivesSingleton.h"
using namespace MyEngine;
//System Class
class MyEngineDLL AxisClass 
{
	bool m_bVisibleOrigin;	//Visibility of the Origin of the object
	bool m_bVisibleCenter;	//Visibility of the Center of the object
	bool m_bVisiblePivot;	//Visibility of the Pivot of the object

	float m_fSize;

	vector3 m_v3Pivot;		//Pivot of rotation in local space
	vector3 m_v3Centroid;	//Centroid of shape in local space

	matrix4 m_m4ToWorld;//Model to World maatrix
	matrix4 m_mPivot;	//Pivot origin in global coordinates
	matrix4 m_mCentroid;//Centroid in global coordinates

	UnitaryPrimitiveSingleton* m_pPrimitiveMngr;
public:
	/* Constructor 	*/
	AxisClass(void);
	/* Copy Constructor */
	AxisClass(AxisClass const& other);
	/* Copy Assignment Operator	*/
	AxisClass& operator=(AxisClass const& other);
	/*	Destructor	*/
	~AxisClass(void);


	/* Gets the "ModelToWorld" matrix associated with the bounding box */
	matrix4 GetModelMatrix(void);
	/* Sets the "ModelToWorld" matrix associated with the bounding box */
	void SetModelMatrix(matrix4 a_ModelMatrix);
	/* "ModelToWorld"
	GetModelMatrix() or SetModelMatrix() */
	__declspec(property(get = GetModelMatrix, put = SetModelMatrix)) matrix4 ModelMatrix;

	/*Sets the visibility of the OBB the AABB and the BS
	Args:
		bool a_bVisible -> true for draw false for not draw*/
	void SetVisible(bool a_bVisible);
	/* Property:
	SetVisibleOBB()*/
	__declspec(property(put = SetVisible)) bool Visible;

	/*Sets the visibility of the OBB
	Args:
		bool a_bVisible -> true for draw false for not draw*/
	void SetVisibleOrigin(bool a_bVisible);
	/* Property:
	SetVisibleOBB()*/
	__declspec(property(put = SetVisibleOrigin)) bool VisibleOrigin;
	
	/*Sets the visibility of the AABB
	Args:
		bool a_bVisible -> true for draw false for not draw*/
	void SetVisiblePivot(bool a_bVisible);
	/* Property:
	SetVisibleAABB()*/
	__declspec(property(put = SetVisiblePivot)) bool VisiblePivot;

	/*Sets the visibility of the BS
	Args:
		bool a_bVisible -> true for draw false for not draw*/
	void SetVisibleCentroid(bool a_bVisible);
	/* Property:
	SetVisibleBS()*/
	__declspec(property(put = SetVisibleCentroid)) bool VisibleCentroid;

	void SetCentroid(vector3 a_v3Centroid);
	void SetPivot(vector3 a_v3Pivot);
	void SetSize(float a_fSize);

	/* Renders the bounding box
		Args:
			a_vColor -> determinate the color of the box to be rendered, if MEDEFAULT
			it will render the shape in the constructed color (white) */
	void Render( bool bForceDraw = false );

private:
	/* Released Memory and objects allocated. */
	void Release(void);
};
//
//EXPIMP_TEMPLATE template class MyEngineDLL std::vector<AxisClass>;
//EXPIMP_TEMPLATE template class MyEngineDLL std::vector<AxisClass*>;

#endif //__AxisClass_H__