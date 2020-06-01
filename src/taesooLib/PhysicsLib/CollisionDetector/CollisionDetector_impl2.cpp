
#include "StdAfxColDet.h"
#include "CdCache.h"
#include "CdScene.h"
#include "CollisionDetector_impl2.h"

CdScene* createCdScene()
{
	return new CdScene();
}

CdCharCache* createCdCharCache()
{
     return new CdCharCache();
}
  void deleteCdCharCache(void* data)
{
    delete (CdCharCache*)data;
}

CdModelCache* createCdModelCache()
{
    return new CdModelCache();
}
CdModelCache* getCachedModel(CdCharCache* cache_, const char* url)
{
    if(cache_->exist(url)){
        return cache_->getChar(url);
	}
	return NULL;
}
CdModelSet* createModelSet(int i)
{
	CdModelSet* modelSet;
	modelSet = new CdModelSet();
	modelSet->linkIndex = i;
	return modelSet;
}
void addTriangle(CdModelSet*  modelSet, double* x, double* y, double* z)
{
	modelSet->addTriangle(x,y,z);
}

void endModel(CdModelSet*  modelSet)
{
                modelSet->endModel();
}

void _addModel(CdModelCache* cachedModel, const char* jName, CdModelSet* modelSet)
{
			cachedModel->addModel(jName, modelSet);
}
void addChar(CdCharCache* cache_, const char* url, CdModelCache* cachedModel)
{
        cache_->addChar( url,cachedModel);
}
void addChar(CdScene* scene_, const char* charName, CdModelCache* cachedModel)
{
    scene_->addChar(charName, new CdChar(cachedModel,charName) );
}


void	CollisionDetector_impl_hidden::_addCollisionPair(const char* charName1, const char* charName2, const char* jointName1, const char* jointName2)
{

    //#ifdef COLLISIONDETECTOR_DEBUG
    cerr << "CollisionDetector_impl::addCollisionPair("
         << charName1 <<  "::" << jointName1 << ", "
         << charName2 <<  "::" << jointName2 << ")" << endl;
    //#endif

	CdJoint* joint1 = 0;
	CdJoint* joint2 = 0;

    CdChar* char1 = scene_->getChar(charName1);
    if (!char1){
        cerr << "CollisionDetector_impl::addCollisionPair : Character not found("
             << charName1 << ")" << endl;
		ASSERT(0);
    } else {
	    joint1 = char1->getJoint(jointName1);
    	if (!joint1){
        	cerr << "CollisionDetector_impl::addCollisionPair : Joint not found("
            	 << charName1 << ","
             	<< jointName1 << ")" << endl;
	    }
		ASSERT(0);

    }

    CdChar* char2 = scene_->getChar(charName2);
    if (!char2){
        cerr << "CollisionDetector_impl::addCollisionPair : Character not found("
             << charName2 << ")" << endl;
		ASSERT(0);

    } else {
    	joint2 = char2->getJoint(jointName2);
    	if (!joint2){
        	cerr << "CollisionDetector_impl::addCollisionPair : Joint not found("
            	 << charName2 << ","
             	<< jointName2 << ")" << endl;
    	}
		ASSERT(0);

    }

    static const bool NOT_SKIP_EMPTY_JOINT = true;

    if(NOT_SKIP_EMPTY_JOINT || (joint1 && joint2)){
    	scene_->addCheckPair(new CdCheckPair(joint2, joint1));
    }
}


	CdChar* CollisionDetector_impl_hidden::getChar(const char* name)
{
return scene_->getChar(name);
}

CdJoint* getJoint(CdChar* rChar, int index)
{
	return rChar->getJoint(index);

}

CollisionDetector_impl_hidden::CollisionDetector_impl_hidden()
{
	scene_=NULL;
}
CollisionDetector_impl_hidden::~CollisionDetector_impl_hidden()
{
    delete scene_;
}

class CdCheckPair;
int CollisionDetector_impl_hidden::_contactIntersection( CdCheckPair* rPair)
{
    collision_data* pair;
    int ret;
    int i;

    rPair->collide(&ret,&pair,CD_FIRST_CONTACT);

    int npoints = 0;
    for (i = 0; i < ret; ++i) {
        npoints += pair[i].num_of_i_points;
    }
    delete pair;
    return npoints;
}
void setConfig(CdJoint* cdJoint, double _14, double _24, double _34, 
		double _11, double _12, double _13,
		double _21, double _22, double _23,
		double _31, double _32, double _33)

{
				double* p    = cdJoint->translation_;
				double (*R)[3] = cdJoint->rotation_;
				p[0] = _14;
				p[1] = _24;
				p[2] = _34;
				
				R[0][0] = _11; R[0][1] = _12; R[0][2] = _13;
				R[1][0] = _21; R[1][1] = _22; R[1][2] = _23;
				R[2][0] = _31; R[2][1] = _32; R[2][2] = _33;
}


void collide(CdCheckPair* rPair, int* ret, collision_data** cdata)
{
	rPair->collide(ret, cdata);
}
void CollisionDetector_impl_hidden::clearCache(const char* url)
{
#ifdef COLLISIONDETECTOR_DEBUG
    cerr << "CollisionDetector_impl::clearCache()" << endl;
#endif
    cache_->removeChar(url);
}

void CollisionDetector_impl_hidden::clearAllCache()
{
#ifdef COLLISIONDETECTOR_DEBUG
    cerr << "CollisionDetector_impl::clearAllCache()" << endl;
    cache_->removeAllChar();
#endif
}

long unsigned int CollisionDetector_impl_hidden::getNumPair()
{
return scene_->getNumCheckPairs();
}


CdCheckPair* CollisionDetector_impl_hidden::getCheckPair(int pCount)
{
        return scene_->getCheckPair(pCount);
}
