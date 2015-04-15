/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2014/05
*/
#ifndef __ANIMATIONCLASS_H_
#define __ANIMATIONCLASS_H_

#include "ME\system\SystemSingleton.h"
#include "ME\mesh\animation\FrameClass.h"

namespace MyEngine
{

class MyEngineDLL AnimationClass
{
public:
	AnimationClass(void);
	AnimationClass(const AnimationClass& other);
	AnimationClass& operator=(const AnimationClass& other);
	~AnimationClass(void);
	void Init(void);
	void Release(void);
	
	void Swap(AnimationClass& other);

	AnimationClass(FrameClass *pFrame, int nFrames);
	void CheckInterpolation(FrameClass *pFrame, int nFrames) const;
private:
	void Interpolation(FrameClass *pFrame, int nFirstKeyFrame, int nLastKeyFrame) const;
};

EXPIMP_TEMPLATE template class MyEngineDLL std::vector<AnimationClass>;
EXPIMP_TEMPLATE template class MyEngineDLL std::vector<AnimationClass*>;

}
#endif //__ANIMATIONCLASS_H__