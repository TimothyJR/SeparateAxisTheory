/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/07
*/
#ifndef __LINEMANAGERSINGLETON_H_
#define __LINEMANAGERSINGLETON_H_

#include "ME\mesh\debug\LineClass.h"

namespace MyEngine
{

//System Class
class MyEngineDLL LineManagerSingleton
{
	int m_nLines;//Number of lines in the manager
	LineClass* m_pLine; //Line object
	std::vector<vector3> m_v3Start;	//Start of the line
	std::vector<vector3> m_v3End;	//Start of the line
	std::vector<vector3> m_v3Color;	//Color of the line

public:
	static LineManagerSingleton* GetInstance(); // Singleton accessor
	static void ReleaseInstance(void); //Singleton Release

	void ClearLines(void);
	void RenderLines(void);
	void AddLine(vector3 a_v3Start, vector3 a_v3End, vector3 a_v3Color = vector3( 0.0f, 1.0f, 0.0f));
	void CompileOpenGL3X(void);

private:
	LineManagerSingleton(void); // Constructor
	LineManagerSingleton(LineManagerSingleton const& other); //Copy Constructor
	LineManagerSingleton& operator=(LineManagerSingleton const& other); // Copy Assignment Operator
	~LineManagerSingleton(void); // Destructor

	void Release(void); // Release Memory
	void Init(void); // Initialize variables
	
	static LineManagerSingleton* m_pInstance; // Singleton
};

}
#endif //__LINEMANAGERSINGLETON_H_