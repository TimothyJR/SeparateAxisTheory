/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/11
*/
#ifndef __OCTANTCLASS_H_
#define __OCTANTCLASS_H_

#include "ME\System\SystemSingleton.h"
#include "ME\Mesh\PrimitiveManagerSingleton.h"
#include "ME\Physics\BoundingObjectClass.h"

namespace MyEngine
{

//System Class
class MyEngineDLL OctantClass
{
	/* The fields of this class will be declared as public to reduce the overhead of the accessors, thus sacrificing security over speed
	 this shouldnt represent an issue due the fact that octants are not meant to be used as individual objects but as part of an octree class*/
public:
	bool m_bEmpty;	//Is the octant Empty?
	bool m_bLeaf;	//Is the octant a leaf?
	
	int m_nLevel;	//Level of the octant in the tree
	int m_nID;		//ID of the octant
	
	float m_fSize;	//Size of the octant m_fSize^3 is its volume

	OctantClass* m_pParent;	//Parent of the octant in the tree
	OctantClass* m_pChild[8];	//Children of the octant
	BoundingObjectClass* m_pBO;	//Bounding object of the octant

	vector3 m_v3CenterG;//Center of the Octant in world coordinates

	std::vector<BoundingObjectClass*> m_vBoundingObjectList;//List of bounding objects inside the octant
	
// Methods

	/* Constructor */
	OctantClass(void);
	/* Copy constructor */
	OctantClass(OctantClass const& other);
	/* Copy Assignment Operator */
	OctantClass& operator=(OctantClass const& other);
	/* Destructor */
	~OctantClass(void);

	////Properties
	///* Asks the Octant if its a Leaf */
	//bool IsLeaf(void);
	///* Asks the octant if its colliding with any Bounding Objects */
	//bool IsEmpty(void);
	///* Asks the octant for its bounding object list */
	//std::vector<BoundingObjectClass*> GetBoundingObjectList(void);
	///* Sets the Bounding Object List for the octant */
	//void SetBoundingObjectList(std::vector<BoundingObjectClass*> a_vBoundingObjectList);
	///* Sets the size of the octant */
	//void SetSize(float a_fSize);
	///* Sets the center of the octant */
	//void SetCenter(vector3 a_v3Center);
	///* Gets the specified child of the octant */
	//OctantClass* GetChild(int a_nOctant);

	
	/* Renders the AABB of the octant if its not empty */
	void Render();
	
	/* Calculates the collider of the octant based on the center and its size */
	void CalculateCollider(void);
	
private:
	/* Releases the object from memory */
	void Release(void);

	/* Initialize the object's fields */
	void Init(void);

	/* Swaps the content of one octant with another */
	void Swap(OctantClass& other);
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<OctantClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<OctantClass*>;

}

#endif //__OctantClass_H__