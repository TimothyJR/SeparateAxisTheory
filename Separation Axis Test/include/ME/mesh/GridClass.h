/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/11
*/
#ifndef __GRIDCLASS_H_
#define __GRIDCLASS_H_

#include "ME\Mesh\MeshClass.h"

namespace MyEngine
{

class MyEngineDLL GridClass : public MeshClass
{
	typedef MeshClass super;

public:
	//Constructor
	GridClass(int a_Axis = MEAXIS::XY | MEAXIS::XZ | MEAXIS::YZ, float a_fSpacing = 100.0f);
	//Copy Constructor
	GridClass(const GridClass& other);
	//Copy Assignment Operator
	virtual GridClass& operator=(const GridClass& other);

	//Destructor
	~GridClass(void);
		
	//Renders the content of the shape
	virtual void Render(float a_fSize = 100.0f);

	void CompileGrid(void);
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<GridClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<GridClass*>;

}
#endif //GRIDCLASS