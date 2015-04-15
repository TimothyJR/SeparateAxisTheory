/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
	Provides a fullscreen mode and icon to 
	http://www.winprog.org/tutorial/simple_window.html 
	Adapted into a class.
Date: 2014/05
*/
#ifndef __WINDOWCLASS_H_
#define __WINDOWCLASS_H_

#include <windows.h>
#include "Resource.h"
#include "ME\system\MeDefinitions.h"

namespace MyEngine
{

class MyEngineDLL WindowClass
{
	bool m_bFullscreen; //Run in fullscreen mode?
	bool m_bBorderless; //Run with borders?
	bool m_bWindowCrated; //Window has been created

	int m_nWindowPosX; //Window position on screen (X)
	int m_nWindowPosY; //Window position on screen (Y)
	
	int m_nWidth; //Window width
	int m_nHeight; //Window height

	int m_nCommandShow; //Number of arguments

	LPCWSTR m_sWindowName; //Name of the window
	HWND m_hWindowHandler; //Window handler

	HINSTANCE m_hInstance; //Window Instance
	WNDPROC m_Callback; //Associated Callback for this window
	
public:
	/*
	Constructor
	*/
	WindowClass(void);
	/*
	Constructor
	*/
	WindowClass(HINSTANCE hInstance, int nCmdShow, WNDPROC WndProc);

	/*
	void SetWindowInstance(HINSTANCE hInstance);
		Sets the window Instance
	*/
	void SetWindowInstance(HINSTANCE hInstance);

	/*
	void SetWindowCallback(WNDPROC WndProc);
		Sets the window callback
	*/
	void SetWindowCallback(WNDPROC WndProc);

	/*
	void SetIncomingArguments(int nCmdShow);
		Sets the number of incoming arguments
	*/
	void SetIncomingArguments(int nCmdShow);

	/*
	CreateMEWindow
		Creates window with the provided arguments
	Arguments:
		LPCWSTR windowName = L"Window" -> Name of the window
		int width = 1280 -> width of the window
		int height = 720 -> height of the window
	*/
	int CreateMEWindow(LPCWSTR windowName = L"Window", int width = 1280, int height = 720);

	/*
	GetHandler
		Returns the window handler
	*/
	HWND GetHandler(void);

	/*
	SetFullscreen(bool input = true)
		Sets whether or not the program will run in fullscreen
	*/
	void SetFullscreen(bool input = true);

	/*
	SetBorderless(bool input = true)
		Sets whether or not the window will be borderless
	*/
	void SetBorderless(bool input = true);

	/*
	void CreateConsoleWindow(void)
		Allocates a console window for output
	*/
	void CreateConsoleWindow(void);

private:
	/*
	Copy Constructor
	*/
	WindowClass(const WindowClass& other);

	/*
	Copy Assignment Operator
	*/
	WindowClass& operator=(const WindowClass& other);

	/*
	void Init(void);
		Initializes the member variables
	*/
	void Init(void);
};

}

#endif /*__WINDOWCLASS_H__*/