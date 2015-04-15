/*--------------------------------------------------------------------------------------------------
Created by Alberto Bobadilla (labigm@rit.edu) in 2013
--------------------------------------------------------------------------------------------------*/
#ifndef __FILEREADERCLASS_H_
#define __FILEREADERCLASS_H_

#include "ME\system\SystemSingleton.h"

namespace MyEngine
{

class MyEngineDLL FileReaderClass
{
	SystemSingleton* m_pSystem; //System pointer
	FILE* m_pFile; //File pointer

public:
	String m_sLine; //Line of file

	/* constructor */
	FileReaderClass(void);
	/* Copy constructor */
	FileReaderClass(const FileReaderClass& other);
	/* Copy assignment operator */
	FileReaderClass& operator=(const FileReaderClass& other);
	/* Destructor */
	~FileReaderClass(void);
	
	/* Dealocate object from memory */
	void Release(void);

	/* Rewinds the file */
	void Rewind(void) const;

	/* Reads Files and allocates it on the string */
	MEErrors ReadFile(String a_sFileName);
	/* closes the file */
	void CloseFile(void);
	/* reads a line of the file and saves it in the string */
	MEErrors ReadNextLine(bool bSkipComments = true);
	/* 
	Removes the blank spaces of the line
		Modified by AndrewWilkinson88@gmail.com
	*/
	MEErrors RemoveBlanks(char** szInput) const; 
	/* Gets the first word of the line */
	String GetFirstWord(bool a_bToUpper = false);

	/*Gets a file name from a explorer window*/
	String RetriveFileNameFromExplorer(LPCWSTR mFilter = L"Plain Text Files (*.txt)\0*.txt\0 All Files (*.*)\0*.*\0", LPCWSTR mDefaultExt = L"txt") const;
	/*Tells a file name from explorer window*/
	char* IndicateFileNameOnExplorer(LPWSTR mSuggested = NULL, LPCWSTR mFilter = L"Plain Text Files (*.txt)\0*.txt\0 All Files (*.*)\0*.*\0", LPCWSTR mDefaultExt = L"txt") const;

	/*Gets the filename with extention from the name */
	String GetFileNameAndExtension(String aFileName);

	/*Gets the filename from the name */
	String GetFileName(String aFileName);

	/* Gets the absolute route from a file name */
	String GetAbsoluteRoute(String aFileName);

private:
	/* Initializates the variables of the object*/
	void Init(void);
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<FileReaderClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<FileReaderClass*>;

}
#endif //_EMPTY_H
