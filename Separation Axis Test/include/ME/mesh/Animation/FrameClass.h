/*--------------------------------------------------------------------------------------------------
Created by Alberto Bobadilla (labigm@rit.edu) in 2014
--------------------------------------------------------------------------------------------------*/
#ifndef __FRAMECLASS_H_
#define __FRAMECLASS_H_

#include "ME\system\SystemSingleton.h"

namespace MyEngine
{

class MyEngineDLL FrameClass
{
	bool m_bKeyframe;
	bool m_bVisible;
	vector3 m_v3Translation;
	vector3 m_v3Rotation;
	vector3 m_v3Scale;
	matrix4 m_m4Transformation;
public:
	
	//Constructor
	FrameClass();
	//Copy Constructor
	FrameClass(const FrameClass& other);
	//Copy Assignment Operator
	virtual FrameClass& operator=(const FrameClass& other);

	//Destructor
	~FrameClass(void);

	matrix4 GetTransformation(void);
	matrix4 ComputeTransformation(	vector3 a_v3Translation,
									vector3 a_v3Pivot,
									vector3 a_v3Rotation,
									vector3 a_v3Scale,
									bool a_bVisible,
									bool a_bKeyFrame,
									matrix4 a_mParent = matrix4(1.0f));
	matrix4 ComputeTransformation(matrix4 a_mParent);
	matrix4 ComputeTransformation(vector3 a_v3Pivot);
	
	bool IsKeyframe(void);
	void SetVisible(bool a_bVisible);
	bool IsVisible(void);
	void SetKeyframe(bool a_bKeyframe);

	void SetPosX( float fValue );
	void SetPosY( float fValue );
	void SetPosZ( float fValue );

	void SetAngX( float fValue );
	void SetAngY( float fValue );
	void SetAngZ( float fValue );

	void SetSizeX( float fValue );
	void SetSizeY( float fValue );
	void SetSizeZ( float fValue );

	float GetPosX( void );
	float GetPosY( void );
	float GetPosZ( void );

	float GetAngX( void );
	float GetAngY( void );
	float GetAngZ( void );

	float GetSizeX( void );
	float GetSizeY( void );
	float GetSizeZ( void );
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<FrameClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<FrameClass*>;

}
#endif //__ANIMATIONFRAMECLASS_H_