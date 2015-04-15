#include "BoundingBoxManagerSingleton.h"

//  BoundingBoxManagerSingleton
BoundingBoxManagerSingleton* BoundingBoxManagerSingleton::m_pInstance = nullptr;
void BoundingBoxManagerSingleton::Init(void)
{
	m_nBoxs = 0;
}
void BoundingBoxManagerSingleton::Release(void)
{
	//Clean the list of Boxs
	for(int n = 0; n < m_nBoxs; n++)
	{
		//Make sure to release the memory of the pointers
		if(m_lBox[n] != nullptr)
		{
			delete m_lBox[n];
			m_lBox[n] = nullptr;
		}
	}
	m_lBox.clear();
	m_lMatrix.clear();
	m_lColor.clear();
	m_nBoxs = 0;
}
BoundingBoxManagerSingleton* BoundingBoxManagerSingleton::GetInstance()
{
	if(m_pInstance == nullptr)
	{
		m_pInstance = new BoundingBoxManagerSingleton();
	}
	return m_pInstance;
}
void BoundingBoxManagerSingleton::ReleaseInstance()
{
	if(m_pInstance != nullptr)
	{
		delete m_pInstance;
		m_pInstance = nullptr;
	}
}
//The big 3
BoundingBoxManagerSingleton::BoundingBoxManagerSingleton(){Init();}
BoundingBoxManagerSingleton::BoundingBoxManagerSingleton(BoundingBoxManagerSingleton const& other){ }
BoundingBoxManagerSingleton& BoundingBoxManagerSingleton::operator=(BoundingBoxManagerSingleton const& other) { return *this; }
BoundingBoxManagerSingleton::~BoundingBoxManagerSingleton(){Release();};
//Accessors
int BoundingBoxManagerSingleton::GetBoxTotal(void){ return m_nBoxs; }

//--- Non Standard Singleton Methods
void BoundingBoxManagerSingleton::GenerateBoundingBox(matrix4 a_mModelToWorld, String a_sInstanceName)
{
	MeshManagerSingleton* pMeshMngr = MeshManagerSingleton::GetInstance();
	//Verify the instance is loaded
	if(pMeshMngr->IsInstanceCreated(a_sInstanceName))
	{//if it is check if the Box has already been created
		int nBox = IdentifyBox(a_sInstanceName);
		if(nBox == -1)
		{
			//Create a new bounding Box
			BoundingBoxClass* pBB = new BoundingBoxClass();
			//construct its information out of the instance name
			pBB->GenerateOrientedBoundingBox(a_sInstanceName);
			//Push the Box back into the list
			m_lBox.push_back(pBB);
			//Push a new matrix into the list
			m_lMatrix.push_back(matrix4(IDENTITY));
			//Specify the color the Box is going to have
			m_lColor.push_back(vector3(1.0f));
			//Increase the number of Boxes
			m_nBoxs++;
		}
		else //If the box has already been created you will need to check its global orientation
		{
			m_lBox[nBox]->GenerateAxisAlignedBoundingBox(a_mModelToWorld);
		}
		nBox = IdentifyBox(a_sInstanceName);
		m_lMatrix[nBox] = a_mModelToWorld;
	}
}

void BoundingBoxManagerSingleton::SetBoundingBoxSpace(matrix4 a_mModelToWorld, String a_sInstanceName)
{
	int nBox = IdentifyBox(a_sInstanceName);
	//If the Box was found
	if(nBox != -1)
	{
		//Set up the new matrix in the appropriate index
		m_lMatrix[nBox] = a_mModelToWorld;
	}
}

int BoundingBoxManagerSingleton::IdentifyBox(String a_sInstanceName)
{
	//Go one by one for all the Boxs in the list
	for(int nBox = 0; nBox < m_nBoxs; nBox++)
	{
		//If the current Box is the one we are looking for we return the index
		if(a_sInstanceName == m_lBox[nBox]->GetName())
			return nBox;
	}
	return -1;//couldn't find it return with no index
}

void BoundingBoxManagerSingleton::AddBoxToRenderList(String a_sInstanceName)
{
	//If I need to render all
	if(a_sInstanceName == "ALL")
	{
		for(int nBox = 0; nBox < m_nBoxs; nBox++)
		{
			m_lBox[nBox]->AddAABBToRenderList(m_lMatrix[nBox], m_lColor[nBox], true);
		}
	}
	else
	{
		int nBox = IdentifyBox(a_sInstanceName);
		if(nBox != -1)
		{
			m_lBox[nBox]->AddAABBToRenderList(m_lMatrix[nBox], m_lColor[nBox], true);
		}
	}
}

void BoundingBoxManagerSingleton::CalculateCollision(void)
{
	//Create a placeholder for all center points
	std::vector<vector3> lCentroid;
	//for all Boxs...
	for(int nBox = 0; nBox < m_nBoxs; nBox++)
	{
		//Make all the Boxs white
		m_lColor[nBox] = vector3(1.0f);
		//Place all the centroids of Boxs in global space
		lCentroid.push_back(static_cast<vector3>(m_lMatrix[nBox] * vector4(m_lBox[nBox]->GetCentroid(), 1.0f)));
	}

	//Now the actual check
	for(int i = 0; i < m_nBoxs - 1; i++)
	{
		for(int j = i + 1; j < m_nBoxs; j++)
		{
			//If the distance between the center of both Boxs is less than the sum of their radius there is a collision
			//For this check we will assume they will be colliding unless they are not in the same space in X, Y or Z
			//so we place them in global positions
			vector3 v1Min = m_lBox[i]->GetMinimumAABB();
			vector3 v1Max = m_lBox[i]->GetMaximumAABB();

			vector3 v2Min = m_lBox[j]->GetMinimumAABB();
			vector3 v2Max = m_lBox[j]->GetMaximumAABB();

			bool bColliding = true;
			if(v1Max.x < v2Min.x || v1Min.x > v2Max.x)
				bColliding = false;
			else if(v1Max.y < v2Min.y || v1Min.y > v2Max.y)
				bColliding = false;
			else if(v1Max.z < v2Min.z || v1Min.z > v2Max.z)
				bColliding = false;

			// Does it pass a AABB test?
			if(bColliding)
			{
				// See if it passes separate axis theory
				bColliding = separateAxisCollision(*m_lBox[i], *m_lBox[j], m_lMatrix[i], m_lMatrix[j]);
				if(bColliding)
				{
					m_lColor[i] = m_lColor[j] = MERED; //We make the Boxes red
				}
			}
		}
	}
}

bool BoundingBoxManagerSingleton::separateAxisCollision(BoundingBoxClass& box1, BoundingBoxClass& box2, matrix4& box1Transform, matrix4& box2Transform)
{
	vector3 defaultAxis[] = {
		vector3(1, 0, 0),
		vector3(0, 1, 0),
		vector3(0, 0, 1)
	};

	// Get the rotation matrix
	glm::mat3 rotationBox1 = glm::mat3(box1Transform);
	glm::mat3 rotationBox2 = glm::mat3(box2Transform);

	vector3 axisBox1[3];
	vector3 axisBox2[3];

	//Get the axes to test on
#pragma region getAxis
	for(int i = 0; i < 3; i++)
	{
		axisBox1[i] = glm::normalize(rotationBox1 * defaultAxis[i]);
		axisBox2[i] = glm::normalize(rotationBox2 * defaultAxis[i]);
	}

	std::vector<vector3> testAxes;

	for(int i = 0; i < 3; i++)
	{
		testAxes.push_back(axisBox1[i]);
		testAxes.push_back(axisBox2[i]);
	}

	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			if(glm::abs(axisBox1[i]) != glm::abs(axisBox2[j]))
			{
				testAxes.push_back(glm::normalize(glm::cross(axisBox1[i], axisBox2[j])));
			}
		}
	}
#pragma endregion

	// Get the points of each box
#pragma region getPoints
	std::vector<vector3> box1Points;
	std::vector<vector3> box2Points;

	vector3 centerBox1 = rotationBox1 * box1.GetCentroid() + vector3(box1Transform[3]);
	vector3 centerBox2 = rotationBox2 * box2.GetCentroid() + vector3(box2Transform[3]);

	vector3 sizeBox1 = box1.GetSize() / 2.0f;
	vector3 sizeBox2 = box2.GetSize() / 2.0f;

	for(int x = -1; x <= 1; x+=2)
	{
		for(int y = -1; y <=1; y+=2)
		{
			for(int z = -1; z <=1; z+=2)
			{
				vector3 multiply(x,y,z);
				vector3 pointBox1(0.0f);
				vector3 pointBox2(0.0f);

				for(int i = 0; i < 3; i++)
				{
					pointBox1 += axisBox1[i] * (sizeBox1[i] * multiply[i]);
					pointBox2 += axisBox2[i] * (sizeBox2[i] * multiply[i]);
				}

				box1Points.push_back(pointBox1);
				box2Points.push_back(pointBox2);

			}
		}
	}
#pragma endregion

	vector3 centerDiff = centerBox2 - centerBox1;

	// Use separate axis theory to test for collisions
	// If there is an axis with no collisions, return false
	for(vector3 axis : testAxes)
	{
		float distance = abs(glm::dot(axis, centerDiff));
		float dimensionBox1 = 0.0f;
		float dimensionBox2 = 0.0f;

		for(vector3 point : box1Points)
		{
			dimensionBox1 = std::max(dimensionBox1, abs(glm::dot(point, axis)));
		}

		for(vector3 point : box2Points)
		{
			dimensionBox2 = std::max(dimensionBox2, abs(glm::dot(point,axis)));
		}

		if(distance > dimensionBox1 + dimensionBox2)
		{
			return false;
		}

	}

	return true;


}