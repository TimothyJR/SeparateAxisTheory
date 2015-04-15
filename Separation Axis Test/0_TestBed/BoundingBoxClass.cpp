#include "BoundingBoxClass.h"
//  BoundingBoxClass
void BoundingBoxClass::Init(void)
{
	m_bInitialized = false;
	m_v3Min = vector3(0.0f);
	m_v3Max = vector3(0.0f);
	m_v3Centroid = vector3(0.0f);
	m_sName = "NULL";
}
void BoundingBoxClass::Swap(BoundingBoxClass& other)
{
	std::swap(m_bInitialized, other.m_bInitialized);
	std::swap(m_v3Min, other.m_v3Min);
	std::swap(m_v3Max, other.m_v3Max);
	std::swap(m_v3Centroid, other.m_v3Centroid);
	std::swap(m_sName, other.m_sName);
}
void BoundingBoxClass::Release(void)
{
	//No pointers to release
}
//The big 3
BoundingBoxClass::BoundingBoxClass(){Init();}
BoundingBoxClass::BoundingBoxClass(BoundingBoxClass const& other)
{
	m_bInitialized = other.m_bInitialized;
	m_v3Min = other.m_v3Min;
	m_v3Max = other.m_v3Max;
	m_v3Centroid = other.m_v3Centroid;
	m_sName = other.m_sName;
}
BoundingBoxClass& BoundingBoxClass::operator=(BoundingBoxClass const& other)
{
	if(this != &other)
	{
		Release();
		Init();
		BoundingBoxClass temp(other);
		Swap(temp);
	}
	return *this;
}
BoundingBoxClass::~BoundingBoxClass(){Release();};
//Accessors
bool BoundingBoxClass::IsInitialized(void){ return m_bInitialized; }
vector3 BoundingBoxClass::GetMinimumOBB(void){ return m_v3Min; }
vector3 BoundingBoxClass::GetMaximumOBB(void){ return m_v3Max; }
vector3 BoundingBoxClass::GetMinimumAABB(void){ return m_v3MinG; }
vector3 BoundingBoxClass::GetMaximumAABB(void){ return m_v3MaxG; }
vector3 BoundingBoxClass::GetCentroid(void){ return m_v3Centroid; }
vector3 BoundingBoxClass::GetSize(){return m_v3Size;}
String BoundingBoxClass::GetName(void){return m_sName;}
//Methods
void BoundingBoxClass::GenerateOrientedBoundingBox(String a_sInstanceName)
{
	//If this has already been initialized there is nothing to do here
	if(m_bInitialized)
		return;
	MeshManagerSingleton* pMeshMngr = MeshManagerSingleton::GetInstance();
	if(pMeshMngr->IsInstanceCreated(a_sInstanceName))
	{
		m_sName = a_sInstanceName;
		
		std::vector<vector3> lVertices = pMeshMngr->GetVertices(m_sName);
		unsigned int nVertices = lVertices.size();
		m_v3Centroid = lVertices[0];
		m_v3Max = lVertices[0];
		m_v3Min = lVertices[0];
		for(unsigned int nVertex = 1; nVertex < nVertices; nVertex++)
		{
			//m_v3Centroid += lVertices[nVertex];
			if(m_v3Min.x > lVertices[nVertex].x)
				m_v3Min.x = lVertices[nVertex].x;
			else if(m_v3Max.x < lVertices[nVertex].x)
				m_v3Max.x = lVertices[nVertex].x;
			
			if(m_v3Min.y > lVertices[nVertex].y)
				m_v3Min.y = lVertices[nVertex].y;
			else if(m_v3Max.y < lVertices[nVertex].y)
				m_v3Max.y = lVertices[nVertex].y;

			if(m_v3Min.z > lVertices[nVertex].z)
				m_v3Min.z = lVertices[nVertex].z;
			else if(m_v3Max.z < lVertices[nVertex].z)
				m_v3Max.z = lVertices[nVertex].z;
		}
		m_v3Centroid = (m_v3Min + m_v3Max) / 2.0f;

		m_v3Size.x = glm::distance(vector3(m_v3Min.x, 0.0f, 0.0f), vector3(m_v3Max.x, 0.0f, 0.0f));
		m_v3Size.y = glm::distance(vector3(0.0f, m_v3Min.y, 0.0f), vector3(0.0f, m_v3Max.y, 0.0f));
		m_v3Size.z = glm::distance(vector3(0.0f, 0.0f, m_v3Min.z), vector3(0.0f, 0.0f, m_v3Max.z));

		m_bInitialized = true;
	}
}
void BoundingBoxClass::GenerateAxisAlignedBoundingBox(matrix4 a_m4ModeltoWorld)
{
	//Check if the Axis Aligned Bounding Box has already been created, if not return
	if(m_v3Max == m_v3Min)
		return;
	//Get the 8 vertices that compose the AABB
	vector3 v3Vertex[8];
	v3Vertex[0] = vector3(m_v3Min.x, m_v3Min.y, m_v3Min.z);
	v3Vertex[1] = vector3(m_v3Max.x, m_v3Min.y, m_v3Min.z);
	v3Vertex[2] = vector3(m_v3Max.x, m_v3Max.y, m_v3Min.z);
	v3Vertex[3] = vector3(m_v3Min.x, m_v3Max.y, m_v3Min.z);

	v3Vertex[4] = vector3(m_v3Min.x, m_v3Min.y, m_v3Max.z);
	v3Vertex[5] = vector3(m_v3Max.x, m_v3Min.y, m_v3Max.z);
	v3Vertex[6] = vector3(m_v3Max.x, m_v3Max.y, m_v3Max.z);
	v3Vertex[7] = vector3(m_v3Min.x, m_v3Max.y, m_v3Max.z);
	//Get those vertices in global space
	for(int nVertex = 0; nVertex < 8; nVertex++)
	{
		v3Vertex[nVertex] = static_cast<vector3>(a_m4ModeltoWorld * vector4(v3Vertex[nVertex],1));
	}
	//Get the Max and min of those global points
	m_v3MaxG = m_v3MinG = v3Vertex[0];
	for(int nVertex = 1; nVertex < 8; nVertex++)
	{
		if(v3Vertex[nVertex].x > m_v3MaxG.x)
			m_v3MaxG.x = v3Vertex[nVertex].x;
		else if(v3Vertex[nVertex].x < m_v3MinG.x)
			m_v3MinG.x = v3Vertex[nVertex].x;

		if(v3Vertex[nVertex].y > m_v3MaxG.y)
			m_v3MaxG.y = v3Vertex[nVertex].y;
		else if(v3Vertex[nVertex].y < m_v3MinG.y)
			m_v3MinG.y = v3Vertex[nVertex].y;
		
		if(v3Vertex[nVertex].z > m_v3MaxG.z)
			m_v3MaxG.z = v3Vertex[nVertex].z;
		else if(v3Vertex[nVertex].z < m_v3MinG.z)
			m_v3MinG.z = v3Vertex[nVertex].z;
	}
	m_v3SizeAABB.x = glm::distance(vector3(m_v3MinG.x, 0.0f, 0.0f), vector3(m_v3MaxG.x, 0.0f, 0.0f));
	m_v3SizeAABB.y = glm::distance(vector3(0.0f, m_v3MinG.y, 0.0f), vector3(0.0f, m_v3MaxG.y, 0.0f));
	m_v3SizeAABB.z = glm::distance(vector3(0.0f, 0.0f, m_v3MinG.z), vector3(0.0f, 0.0f, m_v3MaxG.z));
}
void BoundingBoxClass::AddAABBToRenderList(matrix4 a_m4ModelToWorld, vector3 a_vColor, bool a_bRenderCentroid)
{
	if(!m_bInitialized)
		return;
	MeshManagerSingleton* pMeshMngr = MeshManagerSingleton::GetInstance();
	if(a_bRenderCentroid)
		pMeshMngr->AddAxisToQueue(a_m4ModelToWorld * glm::translate(m_v3Centroid));
	pMeshMngr->AddCubeToQueue(a_m4ModelToWorld * glm::translate(m_v3Centroid) * glm::scale(m_v3Size), 
		vector3( 1.0f - a_vColor.x, 1.0f - a_vColor.y, 1.0f - a_vColor.z), MERENDER::WIRE);
	vector3 v3CentroidGlobal = static_cast<vector3>(a_m4ModelToWorld * vector4(m_v3Centroid,1));
	pMeshMngr->AddCubeToQueue(glm::translate(v3CentroidGlobal) * glm::scale(m_v3SizeAABB), a_vColor, MERENDER::WIRE);
}