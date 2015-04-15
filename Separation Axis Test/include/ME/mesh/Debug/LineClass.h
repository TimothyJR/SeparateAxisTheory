/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/05
*/
#ifndef __LINECLASS_H_
#define __LINECLASS_H_

#include "ME\mesh\Shape\ShapeClass.h"

namespace MyEngine
{

class MyEngineDLL LineClass : public ShapeClass
{
	typedef ShapeClass super;
	bool m_bCompiled;
public:
	//Constructor
	LineClass();
	LineClass(vector3 a_vMin, vector3 a_vMax);

	//Copy Constructor
	LineClass(const LineClass& other);

	//Copy Assignment Operator
	virtual LineClass& operator=(const LineClass& other);

	//Destructor
	~LineClass(void);
		
	//Renders the content of the shape
	void Render(vector3 a_vStart, vector3 a_vEnd, vector3 a_vColor = MEGREEN);

	void CompileForColor(void);
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<LineClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<LineClass*>;

}
#endif //LINECLASS