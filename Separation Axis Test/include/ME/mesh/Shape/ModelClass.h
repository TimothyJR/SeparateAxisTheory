/*--------------------------------------------------------------------------------------------------
Created by Alberto Bobadilla (labigm@rit.edu) in 2013
--------------------------------------------------------------------------------------------------*/
#ifndef __MODELCLASS_H_
#define __MODELCLASS_H_

#include <thread>
#include "ME\mesh\shape\ShapeClass.h"
#include "ME\mesh\shape\GroupClass.h"
#include "ME\System\FileReaderClass.h"
#include "ME\mesh\Animation\SequenceClass.h"
#include "ME\mesh\Animation\StateClass.h"

#include "ME\physics\BoundingObjectClass.h"

namespace MyEngine
{

class MyEngineDLL ModelClass
{	
	bool m_bLoaded;		//Loaded flag
	bool m_bBinded;		//Binded flag
	bool m_bVisible;	//Visibility flag
	bool m_bCollidable;	//Collidable flag

	int m_nMaterials;	//number of materials
	int m_nGroups;		//Number of groups
	int m_nFrames;		//Number of frames
	int m_nStates;		//Number of states
	int m_nSequences;	//Number of sequences
	int m_nHP;			//Hit point of the model

	SystemSingleton* m_pSystem;	//Pointer to the system
	MaterialManagerSingleton* m_pMatMngr;	//Pointer to the Material Manager
	std::thread* m_Thread;	//Thread on which the model is loading
	BoundingObjectClass* m_pBO;	//Bounding Object

	String m_sName;	//Name of the Model
		
	std::vector<GroupClass*> m_vGroup;	//List of groups
	std::vector<SequenceClass*> m_vSequence;	//List of sequences
	std::vector<BoundingObjectClass*> m_vBO;	//List of Bounding Objects (one per frame)
	std::vector<StateClass*> m_vState;	//List of states

public:
	/* Constructor */
	ModelClass(void);
	/* Copy Constructor */
	ModelClass(const ModelClass& other);
	/* Copy Assignment operator*/
	ModelClass& operator=(const ModelClass& other);
	/* Destructor */
	~ModelClass(void);
	
	/* Releases the object from memory */
	void Release(void);

	/* Loads an obj file in memory */
	MEErrors LoadOBJ(String a_sFileName);

	/* Swaps the information of one model object into another */
	void Swap(ModelClass& other);

	/* Asks the model for its number of groups */
	int GetNumberOfGroups(void);
	/* Asks the model for its name */
	String GetName(void);
	/* Property GetName*/
	__declspec(property(get = GetName)) String Name;

	/* Asks the Model for a Group by index */
	GroupClass* GetGroup(int a_nIndex);
	/* Asks the model identify the index of a group by name, -1 if not found*/
	int IdentifyGroup( String a_sName);

	/* Asks the model if its done loading */
	bool IsLoaded(void);

	/* Create a copy of a group by index */
	bool InstanceGroup(GroupClass* a_Target, int a_nIndex);

	/* Binds the Model for opengl 3.x*/
	void CompileOpenGL3X(void);

	/* Sets the Hit Points of the model */
	void SetHP(int a_nHP);
	/* Asks the model for its Hit Points*/
	int GetHP(void);

	/* Sets the model to be collidable */
	void SetCollidable(bool a_bCollidable);
	/* Asks the model if its collidable */
	bool GetCollidable(void);

	/* Sets the model to be visible */
	void SetVisible(bool a_bVisible);
	/* Asks the model if its visible */
	bool GetVisible(void);

	/* 
	Asks the model for a list of vertices for a particular group's shape 
		Args:
			a_nGroup -> -1 for all groups
			a_nShape -> -1 for all shapes
	*/
	std::vector<vector3> GetVertices(int a_nGroup = -1, int a_nShape = -1);

	/* Get the number of frames of animation in this model */
	int GetNumberOfFrames(void);

	/* Asks the model for its sequence List */
	std::vector<SequenceClass*> GetSequenceList(void);

	/* Asks the model for a sequence from its List of sequences */
	SequenceClass* GetSequence(int nSequence);

	/* Asks the model for its number of states*/
	int GetNumberOfStates(void);
	/* Asks the model for its number of states*/
	int GetNumberOfSequences(void);
	/* Asks the model for a state by index */
	StateClass* GetState(int a_nState);

	/* Asks the model the index of the provided state, -1 if not found */
	int IdentifyState(StateClass* a_pState);
	
	/*
	Asks the model for the Bounding Object of a particular frame of animation
		Args:
			a_nFrame -> -1 for the BO that contain the model at all frames
	*/
	BoundingObjectClass* GetBoundingObject(int a_nFrame = -1);
private:
	/* Initializates the model object */
	void Init(void);
	/* Loads an obj on a thread */
	MEErrors LoadThreadedOBJ(String a_sFileName);
	/* Loads the material file for this model */
	MEErrors LoadMTL(String a_sFileName);
	/* Loads the hierarchy file for this model */
	MEErrors LoadHIE(String a_sFileName);
	/* Loads the animation file for this model */
	MEErrors LoadANIM(String a_sFileName);
	/* Loads the sequence file for this model */
	MEErrors LoadSEQ(String a_sFileName);
	/* Loads the state file for this model */
	MEErrors LoadSTA(String a_sFileName);

	/* Asks the model for a group by name*/
	GroupClass* GetGroup(String a_sName);

	/* Asks the model for a state by name */
	StateClass* GetState(String a_sName);

};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<ModelClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<ModelClass*>;

}

#endif //_MODEL_H