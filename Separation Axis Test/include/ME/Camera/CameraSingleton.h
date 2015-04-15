/*--------------------------------------------------------------------------------------------------
Created by Alberto Bobadilla (labigm@rit.edu) in 2013
--------------------------------------------------------------------------------------------------*/
#ifndef __CAMERASINGLETON_H_
#define __CAMERASINGLETON_H_

#include "ME\system\SystemSingleton.h"

namespace MyEngine
{

class MyEngineDLL CameraSingleton
{
	float m_fFieldOfView;	//Field of view of the camera
	float m_fAspectRatio;	//Aspect ratio of the view
	float m_fNear;			//Near clipping plane
	float m_fFar;			//Far clipping plane

	static CameraSingleton* m_pInstance;//Singleton
	SystemSingleton* m_pSystem; //System pointer

	matrix4 m_mProjection; //Projection
	matrix4 m_mView; //View
	matrix4 m_mMVP; //MVP

	vector3 m_vPosition; //CameraPosition
	vector3 m_vForward; //Camera view vector
	vector3 m_vUp; //Camera up vector

	
public:
	/* Gets/Constructs the singleton pointer */
	static CameraSingleton* GetInstance();
	/* Destroys the singleton */
	static void ReleaseInstance(void);

	/* Rotates the camera at an specified Euler angle */
	void Rotate(float a_fAngX, float a_fAngY);

	/* Translates the camera forward or backward */
	void MoveForward(float a_fDistance = 0.1f);
	/* Translates the camera horizontally */
	void MoveSideways(float a_fDistance = 0.1f);
	/* Translates the camera vertically */
	void MoveVertical(float a_fDistance = 0.1f);

	/* Sets the camera position by the specified vector3 */
	void SetPosition(vector3 a_Position);
	/* Gets the position of the camera in world space */
	vector3 GetPosition(void);

	/* Sets the direction of the view vector of the camera*/
	void SetForwardVector(vector3 a_vForward);
	/* Gets the view direction vector in world space */
	vector3 GetForwardVector(void);

	/* Sets the up vector of the camera */
	void SetUpVector(vector3 a_vDirection);
	/* Gets the up vector of the camera in world space */
	vector3 GetUpVector(void);

	/* Calculates the view of the camera */
	void CalculateView();
	/* Sets the view of the camera by the specified matrix */
	void SetView(matrix4 a_mView);
	/* Gets the view of the camera singleton*/
	matrix4 GetView(void);
	/* Property SetView/GetView*/
	__declspec(property(put = SetView, get = GetView)) matrix4 View;

	/* Calculates the projection of the camera */
	void CalculateProjection();
	/* Sets the projection of the camera by the specified matrix */
	void SetProjection(matrix4 a_mProjection);
	/* Asks the camera singleton for the projection */
	matrix4 GetProjection();
	/* Property SetProjection/GetProjection*/
	__declspec(property(put = SetProjection, get = GetProjection)) matrix4 Projection;
	
	/* Gets the Model-View-Projection matrix of the camera */
	matrix4 GetMVP();
	/* Gets the Model-View-Projection matrix of the camera specifying the model view matrix*/
	matrix4 GetMVP(matrix4 a_mModel);
	/* Property GetMVP*/
	__declspec(property(get = GetMVP)) matrix4 VMP;

	/* Sets the camera to be looking at the specified point in global space */
	void SetView(vector3 a_Target);

	/* Asks the camera for its field of view */
	float GetFieldOfView(void);
	/* Asks the camera for its near clipping plane */
	float GetNear(void);
	/* Asks the camera for its far clipping plane */
	float GetFar(void);

	void PrintInfo(void);

private:
	/* Constructor */
	CameraSingleton(void);
	/* Copy Constructor*/
	CameraSingleton(CameraSingleton const& other);
	/* Copy Assignment Operator*/
	CameraSingleton& operator=(CameraSingleton const& other);
	/* Destructor */
	~CameraSingleton(void);

	/* Releases the object from memory */
	void Release(void);
	/* Initialized the object's fields*/
	void Init(void);
};

}
#endif //_CAMERA_H