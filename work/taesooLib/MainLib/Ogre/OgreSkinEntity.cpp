#include "stdafx.h"
#ifndef NO_OGRE
#if OGRE_VERSION_MINOR<9
/*
-----------------------------------------------------------------------------
This source file is part of OGRE
(Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org

Copyright (c) 2000-2006 Torus Knot Software Ltd
Also see acknowledgements in Readme.html

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/copyleft/lesser.txt.

You may alternatively use this source under the terms of a specific version of
the OGRE Unrestricted License provided you have obtained such a license from
Torus Knot Software Ltd.
-----------------------------------------------------------------------------
*/

#if OGRE_VERSION_MAJOR<13
#include "OgreStableHeaders.h"
#else
#include "OgreLodStrategy.h"
#endif
#include "OgreSkinEntity.h"

#include "OgreMeshManager.h"
#include "OgreSubMesh.h"
#include "OgreSkinSubEntity.h"
#include "OgreException.h"
#include "OgreSceneManager.h"
#include "OgreLogManager.h"
#include "OgreSkeleton.h"
#include "OgreBone.h"
#include "OgreCamera.h"
#include "OgreTagPoint.h"
#include "OgreAxisAlignedBox.h"
#include "OgreHardwareBufferManager.h"
#include "OgreVector4.h"
#include "OgreRoot.h"
#include "OgreTechnique.h"
#include "OgrePass.h"
#include "OgreSkeletonInstance.h"
#include "OgreEdgeListBuilder.h"
#include "OgreStringConverter.h"
#include "OgreAnimation.h"
#include "OgreAlignedAllocator.h"
#include "OgreOptimisedUtil.h"

namespace Ogre {
    
	inline bool isLodManual(Ogre::MeshPtr mMesh)
	{
#if OGRE_VERSION_MAJOR>=13
		return mMesh->hasManualLodLevel();
#else
		return mMesh->isLodManual();
#endif
	}
	void softwareVertexSkinning2(
        const float *pSrcPos, float *pDestPos,
        const float *pSrcNorm, float *pDestNorm,
        const float *pBlendWeight, const unsigned char* pBlendIndex,
		std::vector<dualQuaternion> const& _cachedDualQuaternions,
		Ogre::Mesh::IndexMap const& indexMap,
        size_t srcPosStride, size_t destPosStride,
        size_t srcNormStride, size_t destNormStride,
        size_t blendWeightStride, size_t blendIndexStride,
        size_t numWeightsPerVertex,
        size_t numVertices);
    
	//-----------------------------------------------------------------------
    SkinEntity::SkinEntity ()
		: mAnimationState(NULL),
          mSkelAnimVertexData(0),
		  mSoftwareVertexAnimVertexData(0),
		  mHardwareVertexAnimVertexData(0),
          mPreparedForShadowVolumes(false),
          mBoneWorldMatrices(NULL),
          mBoneMatrices(NULL),
          mNumBoneMatrices(0),
		  mFrameAnimationLastUpdated(std::numeric_limits<unsigned long>::max()),
          mFrameBonesLastUpdated(NULL),
		  mSharedSkeletonEntities(NULL),
		  mDisplaySkeleton(false),
	      mHardwareAnimation(false),
		  mHardwarePoseCount(0),
		  mVertexProgramInUse(false),
		  mSoftwareAnimationRequests(0),
		  mSoftwareAnimationNormalsRequests(0),
		  mMeshLodIndex(0),
		  mMeshLodFactorTransformed(1.0f),
		  mMinMeshLodIndex(99),
		  mMaxMeshLodIndex(0),		// Backwards, remember low value = high detail
		  mMaterialLodFactorTransformed(1.0f),
		  mMinMaterialLodIndex(99),
		  mMaxMaterialLodIndex(0), 		// Backwards, remember low value = high detail
          mSkeletonInstance(0),
		  mInitialised(false),
		  mLastParentXform(Matrix4::ZERO),
		  mMeshStateCount(0),
          mFullBoundingBox()
    {
    }
    //-----------------------------------------------------------------------
    SkinEntity::SkinEntity( const String& name, MeshPtr& mesh) :
		MovableObject(name),
        mMesh(mesh),
        mAnimationState(NULL),
		mSkelAnimVertexData(0),
		mSoftwareVertexAnimVertexData(0),
		mHardwareVertexAnimVertexData(0),
        mPreparedForShadowVolumes(false),
        mBoneWorldMatrices(NULL),
        mBoneMatrices(NULL),
        mNumBoneMatrices(0),
		mFrameAnimationLastUpdated(std::numeric_limits<unsigned long>::max()),
        mFrameBonesLastUpdated(NULL),
        mSharedSkeletonEntities(NULL),
		mDisplaySkeleton(false),
		mHardwareAnimation(false),
		mVertexProgramInUse(false),
		mSoftwareAnimationRequests(0),
		mSoftwareAnimationNormalsRequests(0),
		mMeshLodIndex(0),
		mMeshLodFactorTransformed(1.0f),
		mMinMeshLodIndex(99),
		mMaxMeshLodIndex(0),		// Backwards, remember low value = high detail
		mMaterialLodFactorTransformed(1.0f),
		mMinMaterialLodIndex(99),
		mMaxMaterialLodIndex(0), 		// Backwards, remember low value = high detail
		mSkeletonInstance(0),
		mInitialised(false),
		mLastParentXform(Matrix4::ZERO),
		mMeshStateCount(0),
        mFullBoundingBox()
	{
		_initialise();
    }
	//-----------------------------------------------------------------------
	void SkinEntity::backgroundLoadingComplete(Resource* res)
	{
		if (res == mMesh.get())
		{
			// mesh loading has finished, we can construct ourselves now
			_initialise();
		}
	}
	//-----------------------------------------------------------------------
	void SkinEntity::_initialise(bool forceReinitialise)
	{
		if (forceReinitialise)
			_deinitialise();

		if (mInitialised)
			return;

		if (mMesh->isBackgroundLoaded() && !mMesh->isLoaded())
		{
			// register for a callback when mesh is finished loading
			// do this before asking for load to happen to avoid race
			mMesh->addListener(this);
		}
		
		// On-demand load
		mMesh->load();
		// If loading failed, or deferred loading isn't done yet, defer
		// Will get a callback in the case of deferred loading
		// Skeletons are cascade-loaded so no issues there
		if (!mMesh->isLoaded())
			return;

		// Is mesh skeletally animated?
		if (mMesh->hasSkeleton() && !mMesh->getSkeleton().isNull())
		{
			mSkeletonInstance = new SkeletonInstance(mMesh->getSkeleton());
			mSkeletonInstance->load();
		}

		// Build main SkinSubEntity list
		buildSkinSubEntityList(mMesh, &mSkinSubEntityList);

		// Check if mesh is using manual LOD
		if (isLodManual(mMesh))
		{
			ushort i, numLod;
			numLod = mMesh->getNumLodLevels();
			// NB skip LOD 0 which is the original
			for (i = 1; i < numLod; ++i)
			{
				const MeshLodUsage& usage = mMesh->getLodLevel(i);
				// Manually create SkinEntity
				SkinEntity* lodEnt = new SkinEntity(mName + "Lod" + StringConverter::toString(i),
					usage.manualMesh);
				mLodSkinEntityList.push_back(lodEnt);
			}
		}


		// Initialise the AnimationState, if Mesh has animation
		if (hasSkeleton())
		{
			mFrameBonesLastUpdated = new unsigned long(std::numeric_limits<unsigned long>::max());
			mNumBoneMatrices = mSkeletonInstance->getNumBones();
#if OGRE_VERSION_MAJOR<13
			mBoneMatrices = static_cast<Matrix4*>(AlignedMemory::allocate(sizeof(Matrix4) * mNumBoneMatrices));
#else
            mBoneMatrices = static_cast<Affine3*>(OGRE_MALLOC_SIMD(sizeof(Affine3) * mNumBoneMatrices, MEMCATEGORY_ANIMATION));
#endif
		}
		if (hasSkeleton() || hasVertexAnimation())
		{
			mAnimationState = new AnimationStateSet();
			mMesh->_initAnimationState(mAnimationState);
			prepareTempBlendBuffers();
		}

		reevaluateVertexProcessing();
		
		// Update of bounds of the parent SceneNode, if Entity already attached
		// this can happen if Mesh is loaded in background or after reinitialisation
		if( mParentNode )
		{
			getParentSceneNode()->_updateBounds();
		}

		mInitialised = true;
		mMeshStateCount = mMesh->getStateCount();

	}
	//-----------------------------------------------------------------------
	void SkinEntity::_deinitialise(void)
	{
		if (!mInitialised)
			return;

		// Delete submeshes
		SkinSubEntityList::iterator i, iend;
		iend = mSkinSubEntityList.end();
		for (i = mSkinSubEntityList.begin(); i != iend; ++i)
		{
			// Delete SkinSubEntity
			delete *i;
		}
		mSkinSubEntityList.clear();
		
		// Delete LOD entities
		LODSkinEntityList::iterator li, liend;
		liend = mLodSkinEntityList.end();
		for (li = mLodSkinEntityList.begin(); li != liend; ++li)
		{
			// Delete
			delete (*li);
		}
        mLodSkinEntityList.clear();
        
		// Delete shadow renderables
		ShadowRenderableList::iterator si, siend;
		siend = mShadowRenderables.end();
		for (si = mShadowRenderables.begin(); si != siend; ++si)
		{
			delete *si;
		}
        mShadowRenderables.clear();
        
		// Detach all child objects, do this manually to avoid needUpdate() call
		// which can fail because of deleted items
		detachAllObjectsImpl();

		if (mSkeletonInstance) {
			AlignedMemory::deallocate(mBoneWorldMatrices);

            if (mSharedSkeletonEntities) {
                mSharedSkeletonEntities->erase(this);
                if (mSharedSkeletonEntities->size() == 1)
                {
                    (*mSharedSkeletonEntities->begin())->stopSharingSkeletonInstance();
                }
                // Should never occuring, just in case
                else if (mSharedSkeletonEntities->empty())
                {
                    delete mSharedSkeletonEntities;
                    delete mFrameBonesLastUpdated;
                    delete mSkeletonInstance;
                    AlignedMemory::deallocate(mBoneMatrices);
                    delete mAnimationState;
                }
            } else {
                delete mFrameBonesLastUpdated;
                delete mSkeletonInstance;
                AlignedMemory::deallocate(mBoneMatrices);
                delete mAnimationState;
            }
        }
		else if (hasVertexAnimation())
		{
			delete mAnimationState;
		}

		delete mSkelAnimVertexData;
		delete mSoftwareVertexAnimVertexData;
		delete mHardwareVertexAnimVertexData;

		mInitialised = false;
	}
    //-----------------------------------------------------------------------
    SkinEntity::~SkinEntity()
    {
		_deinitialise();
		// Unregister our listener
		mMesh->removeListener(this);
    }
	//-----------------------------------------------------------------------
	bool SkinEntity::hasVertexAnimation(void) const
	{
		return mMesh->hasVertexAnimation();
	}
    //-----------------------------------------------------------------------
    const MeshPtr& SkinEntity::getMesh(void) const
    {
        return mMesh;
    }
    //-----------------------------------------------------------------------
    SkinSubEntity* SkinEntity::getSkinSubEntity(unsigned int index) const
    {
        if (index >= mSkinSubEntityList.size())
            OGRE_EXCEPT(Exception::ERR_INVALIDPARAMS,
            "Index out of bounds.",
            "SkinEntity::getSkinSubEntity");
        return mSkinSubEntityList[index];
    }
    //-----------------------------------------------------------------------
    SkinSubEntity* SkinEntity::getSkinSubEntity(const String& name) const
    {
        ushort index = mMesh->_getSubMeshIndex(name);
        return getSkinSubEntity(index);
    }
    //-----------------------------------------------------------------------
    unsigned int SkinEntity::getNumSubEntities(void) const
    {
        return static_cast< unsigned int >( mSkinSubEntityList.size() );
    }

	SkinEntity* createSkinEntity(SceneManager& manager, 
                                   const String& entityName,
                                   const String& meshName )
	{
		// delegate to factory implementation
		NameValuePairList params;
		params["mesh"] = meshName;
		SkinEntity* pEntity=static_cast<SkinEntity*>(
			manager.createMovableObject(entityName, SkinEntityFactory::FACTORY_TYPE_NAME, 
				&params));

		pEntity->addSoftwareAnimationRequest(true);

		return pEntity;
	}

    //-----------------------------------------------------------------------
    SkinEntity* SkinEntity::clone( const String& newName) const
    {
   		if (!mManager)
		{
			OGRE_EXCEPT(Exception::ERR_ITEM_NOT_FOUND, 
				"Cannot clone an SkinEntity that wasn't created through a "
				"SceneManager", "SkinEntity::clone");
		}
		SkinEntity* newEnt = createSkinEntity(*mManager, newName, getMesh()->getName() );

		if (mInitialised)
		{
			// Copy material settings
			SkinSubEntityList::const_iterator i;
			unsigned int n = 0;
			for (i = mSkinSubEntityList.begin(); i != mSkinSubEntityList.end(); ++i, ++n)
			{
				newEnt->getSkinSubEntity(n)->setMaterialName((*i)->getMaterialName());
			}
			if (mAnimationState)
			{
				delete newEnt->mAnimationState;
				newEnt->mAnimationState = new AnimationStateSet(*mAnimationState);
			}
		}

        return newEnt;
    }
    //-----------------------------------------------------------------------
    void SkinEntity::setMaterialName(const String& name)
    {
        // Set for all subentities
        SkinSubEntityList::iterator i;
        for (i = mSkinSubEntityList.begin(); i != mSkinSubEntityList.end(); ++i)
        {
            (*i)->setMaterialName(name);
        }

    }
    //-----------------------------------------------------------------------
    void SkinEntity::_notifyCurrentCamera(Camera* cam)
    {
		MovableObject::_notifyCurrentCamera(cam);

        // Calculate the LOD
        if (mParentNode)
        {
            // Get mesh lod strategy
            const LodStrategy *meshStrategy = mMesh->getLodStrategy();
            // Get the appropriate lod value
            Real lodValue = meshStrategy->getValue(this, cam);
            // Bias the lod value
            Real biasedMeshLodValue = lodValue * mMeshLodFactorTransformed;

			
            // Get the index at this biased depth
            ushort newMeshLodIndex = mMesh->getLodIndex(biasedMeshLodValue);
            // Apply maximum detail restriction (remember lower = higher detail)
            newMeshLodIndex = std::max(mMaxMeshLodIndex, newMeshLodIndex);
            // Apply minimum detail restriction (remember higher = lower detail)
            newMeshLodIndex = std::min(mMinMeshLodIndex, newMeshLodIndex);

            // Construct event object
            EntityMeshLodChangedEvent evt;
            evt.entity = reinterpret_cast<Entity*>(this);
            evt.camera = cam;
            evt.lodValue = biasedMeshLodValue;
            evt.previousLodIndex = mMeshLodIndex;
            evt.newLodIndex = newMeshLodIndex;

            // Notify lod event listeners
            cam->getSceneManager()->_notifyEntityMeshLodChanged(evt);

            // Change lod index
            mMeshLodIndex = evt.newLodIndex;

            // Now do material LOD
            lodValue *= mMaterialLodFactorTransformed;
			
            SkinSubEntityList::iterator i, iend;
            iend = mSkinSubEntityList.end();
            for (i = mSkinSubEntityList.begin(); i != iend; ++i)
            {
                // Get sub-entity material
                const MaterialPtr& material = (*i)->mMaterial;
                
                // Get material lod strategy
                const LodStrategy *materialStrategy = material->getLodStrategy();
                
                // Recalculate lod value if strategies do not match
                Real biasedMaterialLodValue;
                if (meshStrategy == materialStrategy)
                    biasedMaterialLodValue = lodValue;
                else
                    biasedMaterialLodValue = materialStrategy->getValue(this, cam) * materialStrategy->transformBias(mMaterialLodFactor);

                // Get the index at this biased depth
                unsigned short idx = material->getLodIndex(biasedMaterialLodValue);
                // Apply maximum detail restriction (remember lower = higher detail)
                idx = std::max(mMaxMaterialLodIndex, idx);
                // Apply minimum detail restriction (remember higher = lower detail)
                idx = std::min(mMinMaterialLodIndex, idx);

                // Construct event object
                EntityMaterialLodChangedEvent subEntEvt;
                subEntEvt.subEntity = reinterpret_cast<Ogre::SubEntity*>(*i);
                subEntEvt.camera = cam;
                subEntEvt.lodValue = biasedMaterialLodValue;
                subEntEvt.previousLodIndex = (*i)->mMaterialLodIndex;
                subEntEvt.newLodIndex = idx;

                // Notify lod event listeners
                cam->getSceneManager()->_notifyEntityMaterialLodChanged(subEntEvt);

                // Change lod index
                (*i)->mMaterialLodIndex = subEntEvt.newLodIndex;

				// Also invalidate any camera distance cache
				(*i)->_invalidateCameraCache ();
            }


        }
        // Notify any child objects
        ChildObjectList::iterator child_itr = mChildObjectList.begin();
        ChildObjectList::iterator child_itr_end = mChildObjectList.end();
        for( ; child_itr != child_itr_end; child_itr++)
        {
            (*child_itr).second->_notifyCurrentCamera(cam);
        }
    }
    //-----------------------------------------------------------------------
    const AxisAlignedBox& SkinEntity::getBoundingBox(void) const
    {
		// Get from Mesh
        mFullBoundingBox = mMesh->getBounds();
        mFullBoundingBox.merge(getChildObjectsBoundingBox());

        // Don't scale here, this is taken into account when world BBox calculation is done

        return mFullBoundingBox;
    }
    //-----------------------------------------------------------------------
    AxisAlignedBox SkinEntity::getChildObjectsBoundingBox(void) const
    {
        AxisAlignedBox aa_box;
        AxisAlignedBox full_aa_box;
        full_aa_box.setNull();

        ChildObjectList::const_iterator child_itr = mChildObjectList.begin();
        ChildObjectList::const_iterator child_itr_end = mChildObjectList.end();
        for( ; child_itr != child_itr_end; child_itr++)
        {
            aa_box = child_itr->second->getBoundingBox();
            TagPoint* tp = (TagPoint*)child_itr->second->getParentNode();
            // Use transform local to skeleton since world xform comes later
#if OGRE_VERSION_MAJOR>=13
            aa_box.transform(tp->_getFullLocalTransform());
#else
            aa_box.transformAffine(tp->_getFullLocalTransform());
#endif

            full_aa_box.merge(aa_box);
        }

        return full_aa_box;
    }
	//-----------------------------------------------------------------------
	const AxisAlignedBox& SkinEntity::getWorldBoundingBox(bool derive) const
	{
		if (derive)
		{
			// derive child bounding boxes
			ChildObjectList::const_iterator child_itr = mChildObjectList.begin();
			ChildObjectList::const_iterator child_itr_end = mChildObjectList.end();
			for( ; child_itr != child_itr_end; child_itr++)
			{
				child_itr->second->getWorldBoundingBox(true);
			}
		}
		return MovableObject::getWorldBoundingBox(derive);
	}
	//-----------------------------------------------------------------------
	const Sphere& SkinEntity::getWorldBoundingSphere(bool derive) const
	{
		if (derive)
		{
			// derive child bounding boxes
			ChildObjectList::const_iterator child_itr = mChildObjectList.begin();
			ChildObjectList::const_iterator child_itr_end = mChildObjectList.end();
			for( ; child_itr != child_itr_end; child_itr++)
			{
				child_itr->second->getWorldBoundingSphere(true);
			}
		}
		return MovableObject::getWorldBoundingSphere(derive);

	}
    //-----------------------------------------------------------------------
    void SkinEntity::_updateRenderQueue(RenderQueue* queue)
    {
		// Do nothing if not initialised yet
		if (!mInitialised)
			return;

		// Check mesh state count, will be incremented if reloaded
		if (mMesh->getStateCount() != mMeshStateCount)
		{
			// force reinitialise
			_initialise(true);
		}

        SkinEntity* displayEntity = this;
		// Check we're not using a manual LOD
        if (mMeshLodIndex > 0 && isLodManual(mMesh))
        {
            // Use alternate SkinEntity
            assert( static_cast< size_t >( mMeshLodIndex - 1 ) < mLodSkinEntityList.size() &&
                "No LOD SkinEntityList - did you build the manual LODs after creating the SkinEntity?");
            // index - 1 as we skip index 0 (original lod)
            if (hasSkeleton() && mLodSkinEntityList[mMeshLodIndex - 1]->hasSkeleton())
            {
                // Copy the animation state set to lod SkinEntity, we assume the lod
                // SkinEntity only has a subset animation states
                mAnimationState->copyMatchingState(
					mLodSkinEntityList[mMeshLodIndex - 1]->mAnimationState);
            }
            mLodSkinEntityList[mMeshLodIndex - 1]->_updateRenderQueue(queue);
            return;
        }

        // Add each visible SkinSubEntity to the queue
        SkinSubEntityList::iterator i, iend;
        iend = mSkinSubEntityList.end();
        for (i = mSkinSubEntityList.begin(); i != iend; ++i)
        {
            if((*i)->isVisible())
            {
                if(mRenderQueueIDSet)
                {
                    queue->addRenderable(*i, mRenderQueueID);
                }
                else
                {
                    queue->addRenderable(*i);
                }
            }
        }

        // Since we know we're going to be rendered, take this opportunity to
        // update the animation
        if (hasSkeleton() || hasVertexAnimation())
        {
            updateAnimation();

            //--- pass this point,  we are sure that the transformation matrix of each bone and tagPoint have been updated
            ChildObjectList::iterator child_itr = mChildObjectList.begin();
            ChildObjectList::iterator child_itr_end = mChildObjectList.end();
            for( ; child_itr != child_itr_end; child_itr++)
            {
                if ((*child_itr).second->isVisible())
                    (*child_itr).second->_updateRenderQueue(queue);
            }
        }

        // HACK to display bones
        // This won't work if the SkinEntity is not centered at the origin
        // TODO work out a way to allow bones to be rendered when SkinEntity not centered
        if (mDisplaySkeleton && hasSkeleton())
        {
#if OGRE_VERSION_MAJOR<13
            int numBones = mSkeletonInstance->getNumBones();
            for (int b = 0; b < numBones; ++b)
            {
                Bone* bone = mSkeletonInstance->getBone(b);
                if(mRenderQueueIDSet)
                {
                     queue->addRenderable(bone->getDebugRenderable(1), mRenderQueueID);
                } else {
                     queue->addRenderable(bone->getDebugRenderable(1));
                }
            }
#endif
        }




    }
    //-----------------------------------------------------------------------
    AnimationState* SkinEntity::getAnimationState(const String& name) const
    {
        if (!mAnimationState)
        {
            OGRE_EXCEPT(Exception::ERR_ITEM_NOT_FOUND, "SkinEntity is not animated",
                "SkinEntity::getAnimationState");
        }

		return mAnimationState->getAnimationState(name);
    }
    //-----------------------------------------------------------------------
    AnimationStateSet* SkinEntity::getAllAnimationStates(void) const
    {
        return mAnimationState;
    }
    //-----------------------------------------------------------------------
    const String& SkinEntity::getMovableType(void) const
    {
		return SkinEntityFactory::FACTORY_TYPE_NAME;
    }
	//-----------------------------------------------------------------------
	bool SkinEntity::tempVertexAnimBuffersBound(void) const
	{
		// Do we still have temp buffers for software vertex animation bound?
		bool ret = true;
		if (mMesh->sharedVertexData && mMesh->getSharedVertexDataAnimationType() != VAT_NONE)
		{
			ret = ret && mTempVertexAnimInfo.buffersCheckedOut(true, false);
		}
		for (SkinSubEntityList::const_iterator i = mSkinSubEntityList.begin();
			i != mSkinSubEntityList.end(); ++i)
		{
			SkinSubEntity* sub = *i;
			if (!sub->getSubMesh()->useSharedVertices
				&& sub->getSubMesh()->getVertexAnimationType() != VAT_NONE)
			{
				ret = ret && sub->_getVertexAnimTempBufferInfo()->buffersCheckedOut(true, false);
			}
		}
		return ret;
	}
    //-----------------------------------------------------------------------
    bool SkinEntity::tempSkelAnimBuffersBound(bool requestNormals) const
    {
        // Do we still have temp buffers for software skeleton animation bound?
        if (mSkelAnimVertexData)
        {
            if (!mTempSkelAnimInfo.buffersCheckedOut(true, requestNormals))
                return false;
        }
        for (SkinSubEntityList::const_iterator i = mSkinSubEntityList.begin();
            i != mSkinSubEntityList.end(); ++i)
        {
            SkinSubEntity* sub = *i;
            if (sub->isVisible() && sub->mSkelAnimVertexData)
            {
                if (!sub->mTempSkelAnimInfo.buffersCheckedOut(true, requestNormals))
                    return false;
            }
        }
        return true;
    }
    //-----------------------------------------------------------------------
    void SkinEntity::updateAnimation(void)
    {
		// Do nothing if not initialised yet
		if (!mInitialised)
			return;

		Root& root = Root::getSingleton();
		bool hwAnimation = isHardwareAnimationEnabled();
		bool forcedSwAnimation = getSoftwareAnimationRequests()>0;
		bool forcedNormals = getSoftwareAnimationNormalsRequests()>0;
		bool stencilShadows = false;
		if (getCastShadows() && hasEdgeList() && root._getCurrentSceneManager())
			stencilShadows =  root._getCurrentSceneManager()->isShadowTechniqueStencilBased();
		bool softwareAnimation = !hwAnimation || stencilShadows || forcedSwAnimation;
		// Blend normals in s/w only if we're not using h/w animation,
		// since shadows only require positions
		bool blendNormals = !hwAnimation || forcedNormals;
        // Animation dirty if animation state modified or manual bones modified
        bool animationDirty =
            (mFrameAnimationLastUpdated != mAnimationState->getDirtyFrameNumber()) ||
            (hasSkeleton() && getSkeleton()->getManualBonesDirty());

		// We only do these tasks if animation is dirty
		// Or, if we're using a skeleton and manual bones have been moved
		// Or, if we're using software animation and temp buffers are unbound
        if (animationDirty ||
			(softwareAnimation && hasVertexAnimation() && !tempVertexAnimBuffersBound()) ||
			(softwareAnimation && hasSkeleton() && !tempSkelAnimBuffersBound(blendNormals)))
        {
			if (hasVertexAnimation())
			{
				if (softwareAnimation)
				{
					// grab & bind temporary buffer for positions
					if (mSoftwareVertexAnimVertexData
						&& mMesh->getSharedVertexDataAnimationType() != VAT_NONE)
					{
						mTempVertexAnimInfo.checkoutTempCopies(true, false);
						// NB we suppress hardware upload while doing blend if we're
						// hardware animation, because the only reason for doing this
						// is for shadow, which need only be uploaded then
						mTempVertexAnimInfo.bindTempCopies(mSoftwareVertexAnimVertexData,
							hwAnimation);
					}
					SkinSubEntityList::iterator i, iend;
					iend = mSkinSubEntityList.end();
					for (i = mSkinSubEntityList.begin(); i != iend; ++i)
					{
						// Blend dedicated geometry
						SkinSubEntity* se = *i;
						if (se->isVisible() && se->mSoftwareVertexAnimVertexData
							&& se->getSubMesh()->getVertexAnimationType() != VAT_NONE)
						{
							se->mTempVertexAnimInfo.checkoutTempCopies(true, false);
							se->mTempVertexAnimInfo.bindTempCopies(se->mSoftwareVertexAnimVertexData,
								hwAnimation);
						}

					}
				}
				applyVertexAnimation(hwAnimation, stencilShadows);
			}

			if (hasSkeleton())
			{
				cacheBoneMatrices();

				// Software blend?
				if (softwareAnimation)
				{

					// Ok, we need to do a software blend
					// Firstly, check out working vertex buffers
					if (mSkelAnimVertexData)
					{
						// Blend shared geometry
						// NB we suppress hardware upload while doing blend if we're
						// hardware animation, because the only reason for doing this
						// is for shadow, which need only be uploaded then
						mTempSkelAnimInfo.checkoutTempCopies(true, blendNormals);
						mTempSkelAnimInfo.bindTempCopies(mSkelAnimVertexData,
							hwAnimation);

						softwareVertexBlend(mBoneMatrices, mMesh->sharedBlendIndexToBoneIndexMap, 
							(mMesh->getSharedVertexDataAnimationType() != VAT_NONE) ?
								mSoftwareVertexAnimVertexData :	mMesh->sharedVertexData,
																mSkelAnimVertexData,
																blendNormals);
						/*
                        // Prepare blend matrices, TODO: Move out of here
                        Mesh::prepareMatricesForVertexBlend(blendMatrices,
                            mBoneMatrices, mMesh->sharedBlendIndexToBoneIndexMap);
						// Blend, taking source from either mesh data or morph data
						Mesh::softwareVertexBlend(
							(mMesh->getSharedVertexDataAnimationType() != VAT_NONE) ?
								mSoftwareVertexAnimVertexData :	mMesh->sharedVertexData,
							mSkelAnimVertexData,
							blendMatrices, mMesh->sharedBlendIndexToBoneIndexMap.size(),
							blendNormals);*/
					}
					SkinSubEntityList::iterator i, iend;
					iend = mSkinSubEntityList.end();
					for (i = mSkinSubEntityList.begin(); i != iend; ++i)
					{
						// Blend dedicated geometry
						SkinSubEntity* se = *i;
						if (se->isVisible() && se->mSkelAnimVertexData)
						{
							se->mTempSkelAnimInfo.checkoutTempCopies(true, blendNormals);
							se->mTempSkelAnimInfo.bindTempCopies(se->mSkelAnimVertexData,
								hwAnimation);
                            /*// Prepare blend matrices, TODO: Move out of here
                            Mesh::prepareMatricesForVertexBlend(blendMatrices,
                                mBoneMatrices, se->mSubMesh->blendIndexToBoneIndexMap);
							// Blend, taking source from either mesh data or morph data
							Mesh::softwareVertexBlend(
								(se->getSubMesh()->getVertexAnimationType() != VAT_NONE)?
									se->mSoftwareVertexAnimVertexData : se->mSubMesh->vertexData,
								se->mSkelAnimVertexData,
								blendMatrices, se->mSubMesh->blendIndexToBoneIndexMap.size(),
								blendNormals);*/

							softwareVertexBlend(mBoneMatrices, se->mSubMesh->blendIndexToBoneIndexMap, 
								(se->getSubMesh()->getVertexAnimationType() != VAT_NONE) ?
								se->mSoftwareVertexAnimVertexData :	se->mSubMesh->vertexData,
																se->mSkelAnimVertexData,
																blendNormals);
						}

					}

				}
			}

            // Trigger update of bounding box if necessary
            if (!mChildObjectList.empty())
                mParentNode->needUpdate();

			mFrameAnimationLastUpdated = mAnimationState->getDirtyFrameNumber();
        }

        // Need to update the child object's transforms when animation dirty
        // or parent node transform has altered.
        if (hasSkeleton() &&
            (animationDirty || mLastParentXform != _getParentNodeFullTransform()))
        {
            // Cache last parent transform for next frame use too.
            mLastParentXform = _getParentNodeFullTransform();

            //--- Update the child object's transforms
            ChildObjectList::iterator child_itr = mChildObjectList.begin();
            ChildObjectList::iterator child_itr_end = mChildObjectList.end();
            for( ; child_itr != child_itr_end; child_itr++)
            {
                (*child_itr).second->getParentNode()->_update(true, true);
            }

            // Also calculate bone world matrices, since are used as replacement world matrices,
            // but only if it's used (when using hardware animation and skeleton animated).
            if (hwAnimation && _isSkeletonAnimated())
            {
                // Allocate bone world matrices on demand, for better memory footprint
                // when using software animation.
                if (!mBoneWorldMatrices)
                {
#if OGRE_VERSION_MAJOR<13
                    mBoneWorldMatrices =
                        static_cast<Matrix4*>(AlignedMemory::allocate(sizeof(Matrix4) * mNumBoneMatrices));
#else
                    mBoneWorldMatrices =
                        static_cast<Affine3*>(OGRE_MALLOC_SIMD(sizeof(Affine3) * mNumBoneMatrices, MEMCATEGORY_ANIMATION));
                    std::fill(mBoneWorldMatrices, mBoneWorldMatrices + mNumBoneMatrices, Affine3::IDENTITY);
#endif
                }

                OptimisedUtil::getImplementation()->concatenateAffineMatrices(
                    mLastParentXform,
                    mBoneMatrices,
                    mBoneWorldMatrices,
                    mNumBoneMatrices);
            }
        }
    }
	//-----------------------------------------------------------------------
	void SkinEntity::initHardwareAnimationElements(VertexData* vdata,
		ushort numberOfElements)
	{
		if (vdata->hwAnimationDataList.size() < numberOfElements)
		{
#if OGRE_VERSION_MINOR>=9 || OGRE_VERSION_MAJOR>=13
			vdata->allocateHardwareAnimationElements(numberOfElements, true);
#else
			vdata->allocateHardwareAnimationElements(numberOfElements);
#endif
		}
		// Initialise parametrics incase we don't use all of them
		for (size_t i = 0; i < vdata->hwAnimationDataList.size(); ++i)
		{
			vdata->hwAnimationDataList[i].parametric = 0.0f;
		}
		// reset used count
		vdata->hwAnimDataItemsUsed = 0;

	}
	//-----------------------------------------------------------------------
	void SkinEntity::applyVertexAnimation(bool hardwareAnimation, bool stencilShadows)
	{
		const MeshPtr& msh = getMesh();
		bool swAnim = !hardwareAnimation || stencilShadows || (mSoftwareAnimationRequests>0);

		// make sure we have enough hardware animation elements to play with
		if (hardwareAnimation)
		{
			if (mHardwareVertexAnimVertexData
				&& msh->getSharedVertexDataAnimationType() != VAT_NONE)
			{
				initHardwareAnimationElements(mHardwareVertexAnimVertexData,
					(msh->getSharedVertexDataAnimationType() == VAT_POSE)
					? mHardwarePoseCount : 1);
			}
			for (SkinSubEntityList::iterator si = mSkinSubEntityList.begin();
				si != mSkinSubEntityList.end(); ++si)
			{
				SkinSubEntity* sub = *si;
				if (sub->getSubMesh()->getVertexAnimationType() != VAT_NONE &&
					!sub->getSubMesh()->useSharedVertices)
				{
					initHardwareAnimationElements(
						sub->_getHardwareVertexAnimVertexData(),
						(sub->getSubMesh()->getVertexAnimationType() == VAT_POSE)
						? sub->mHardwarePoseCount : 1);
				}
			}

		}
		else
		{
			// May be blending multiple poses in software
			// Suppress hardware upload of buffers
			if (mSoftwareVertexAnimVertexData &&
				mMesh->getSharedVertexDataAnimationType() == VAT_POSE)
			{
				const VertexElement* elem = mSoftwareVertexAnimVertexData
					->vertexDeclaration->findElementBySemantic(VES_POSITION);
				HardwareVertexBufferSharedPtr buf = mSoftwareVertexAnimVertexData
					->vertexBufferBinding->getBuffer(elem->getSource());
				buf->suppressHardwareUpdate(true);
			}
			for (SkinSubEntityList::iterator si = mSkinSubEntityList.begin();
				si != mSkinSubEntityList.end(); ++si)
			{
				SkinSubEntity* sub = *si;
				if (!sub->getSubMesh()->useSharedVertices &&
					sub->getSubMesh()->getVertexAnimationType() == VAT_POSE)
				{
					VertexData* data = sub->_getSoftwareVertexAnimVertexData();
					const VertexElement* elem = data->vertexDeclaration
						->findElementBySemantic(VES_POSITION);
					HardwareVertexBufferSharedPtr buf = data
						->vertexBufferBinding->getBuffer(elem->getSource());
					buf->suppressHardwareUpdate(true);
				}
			}
		}


		// Now apply the animation(s)
		// Note - you should only apply one morph animation to each set of vertex data
		// at once; if you do more, only the last one will actually apply
		markBuffersUnusedForAnimation();
		ConstEnabledAnimationStateIterator animIt = mAnimationState->getEnabledAnimationStateIterator();
		while(animIt.hasMoreElements())
		{
            const AnimationState* state = animIt.getNext();
            Animation* anim = msh->_getAnimationImpl(state->getAnimationName());
            if (anim)
            {
				ASSERT(0);
                //anim->apply(this, state->getTimePosition(), state->getWeight(),
                  //  swAnim, hardwareAnimation);
            }
		}
		// Deal with cases where no animation applied
		restoreBuffersForUnusedAnimation(hardwareAnimation);

		// Unsuppress hardware upload if we suppressed it
		if (!hardwareAnimation)
		{
			if (mSoftwareVertexAnimVertexData &&
				msh->getSharedVertexDataAnimationType() == VAT_POSE)
			{
				const VertexElement* elem = mSoftwareVertexAnimVertexData
					->vertexDeclaration->findElementBySemantic(VES_POSITION);
				HardwareVertexBufferSharedPtr buf = mSoftwareVertexAnimVertexData
					->vertexBufferBinding->getBuffer(elem->getSource());
				buf->suppressHardwareUpdate(false);
			}
			for (SkinSubEntityList::iterator si = mSkinSubEntityList.begin();
				si != mSkinSubEntityList.end(); ++si)
			{
				SkinSubEntity* sub = *si;
				if (!sub->getSubMesh()->useSharedVertices &&
					sub->getSubMesh()->getVertexAnimationType() == VAT_POSE)
				{
					VertexData* data = sub->_getSoftwareVertexAnimVertexData();
					const VertexElement* elem = data->vertexDeclaration
						->findElementBySemantic(VES_POSITION);
					HardwareVertexBufferSharedPtr buf = data
						->vertexBufferBinding->getBuffer(elem->getSource());
					buf->suppressHardwareUpdate(false);
				}
			}
		}

	}
	//-----------------------------------------------------------------------------
	void SkinEntity::markBuffersUnusedForAnimation(void)
	{
		mVertexAnimationAppliedThisFrame = false;
		for (SkinSubEntityList::iterator i = mSkinSubEntityList.begin();
			i != mSkinSubEntityList.end(); ++i)
		{
			(*i)->_markBuffersUnusedForAnimation();
		}
	}
	//-----------------------------------------------------------------------------
	void SkinEntity::_markBuffersUsedForAnimation(void)
	{
		mVertexAnimationAppliedThisFrame = true;
		// no cascade
	}
	//-----------------------------------------------------------------------------
	void SkinEntity::restoreBuffersForUnusedAnimation(bool hardwareAnimation)
	{
		// Rebind original positions if:
		//  We didn't apply any animation and
		//    We're morph animated (hardware binds keyframe, software is missing)
		//    or we're pose animated and software (hardware is fine, still bound)
		if (mMesh->sharedVertexData &&
			!mVertexAnimationAppliedThisFrame &&
			(!hardwareAnimation || mMesh->getSharedVertexDataAnimationType() == VAT_MORPH))
		{
			const VertexElement* srcPosElem =
				mMesh->sharedVertexData->vertexDeclaration->findElementBySemantic(VES_POSITION);
			HardwareVertexBufferSharedPtr srcBuf =
				mMesh->sharedVertexData->vertexBufferBinding->getBuffer(
					srcPosElem->getSource());

			// Bind to software
			const VertexElement* destPosElem =
				mSoftwareVertexAnimVertexData->vertexDeclaration->findElementBySemantic(VES_POSITION);
			mSoftwareVertexAnimVertexData->vertexBufferBinding->setBinding(
				destPosElem->getSource(), srcBuf);

		}

		// rebind any missing hardware pose buffers
		// Caused by not having any animations enabled, or keyframes which reference
		// no poses
		if (mMesh->sharedVertexData && hardwareAnimation 
			&& mMesh->getSharedVertexDataAnimationType() == VAT_POSE)
		{
			bindMissingHardwarePoseBuffers(mMesh->sharedVertexData, mHardwareVertexAnimVertexData);
		}


		for (SkinSubEntityList::iterator i = mSkinSubEntityList.begin();
			i != mSkinSubEntityList.end(); ++i)
		{
			(*i)->_restoreBuffersForUnusedAnimation(hardwareAnimation);
		}

	}
	//---------------------------------------------------------------------
	void SkinEntity::bindMissingHardwarePoseBuffers(const VertexData* srcData, 
		VertexData* destData)
	{
		// For hardware pose animation, also make sure we've bound buffers to all the elements
		// required - if there are missing bindings for elements in use,
		// some rendersystems can complain because elements refer
		// to an unbound source.
		// Get the original position source, we'll use this to fill gaps
		const VertexElement* srcPosElem =
			srcData->vertexDeclaration->findElementBySemantic(VES_POSITION);
		HardwareVertexBufferSharedPtr srcBuf =
			srcData->vertexBufferBinding->getBuffer(
				srcPosElem->getSource());

		for (VertexData::HardwareAnimationDataList::const_iterator i = destData->hwAnimationDataList.begin();
			i != destData->hwAnimationDataList.end(); ++i)
		{
#if OGRE_VERSION_MAJOR<13
			const VertexData::HardwareAnimationData& animData = *i;
			if (!destData->vertexBufferBinding->isBufferBound(
				animData.targetVertexElement->getSource()))
			{
				// Bind to a safe default
				destData->vertexBufferBinding->setBinding(
					animData.targetVertexElement->getSource(), srcBuf);
			}
#else
            const VertexData::HardwareAnimationData& animData = *i;
            if (!destData->vertexBufferBinding->isBufferBound(
                animData.targetBufferIndex))
            {
                // Bind to a safe default
                destData->vertexBufferBinding->setBinding(
                    animData.targetBufferIndex, srcBuf);
            }
#endif
		}

	}
	//-----------------------------------------------------------------------
	void SkinEntity::_updateAnimation(void)
	{
		// Externally visible method
		if (hasSkeleton() || hasVertexAnimation())
		{
			updateAnimation();
		}
	}
	//-----------------------------------------------------------------------
    bool SkinEntity::_isAnimated(void) const
    {
        return (mAnimationState && mAnimationState->hasEnabledAnimationState()) ||
               (getSkeleton() && getSkeleton()->hasManualBones());
    }
	//-----------------------------------------------------------------------
    bool SkinEntity::_isSkeletonAnimated(void) const
    {
        return getSkeleton() &&
            (mAnimationState->hasEnabledAnimationState() || getSkeleton()->hasManualBones());
    }
	//-----------------------------------------------------------------------
	VertexData* SkinEntity::_getSkelAnimVertexData(void) const
	{
		assert (mSkelAnimVertexData && "Not software skinned or has no shared vertex data!");
        return mSkelAnimVertexData;
	}
	//-----------------------------------------------------------------------
	VertexData* SkinEntity::_getSoftwareVertexAnimVertexData(void) const
	{
		assert (mSoftwareVertexAnimVertexData && "Not vertex animated or has no shared vertex data!");
		return mSoftwareVertexAnimVertexData;
	}
	//-----------------------------------------------------------------------
	VertexData* SkinEntity::_getHardwareVertexAnimVertexData(void) const
	{
		assert (mHardwareVertexAnimVertexData && "Not vertex animated or has no shared vertex data!");
		return mHardwareVertexAnimVertexData;
	}
	//-----------------------------------------------------------------------
	TempBlendedBufferInfo* SkinEntity::_getSkelAnimTempBufferInfo(void)
	{
		return &mTempSkelAnimInfo;
	}
	//-----------------------------------------------------------------------
	TempBlendedBufferInfo* SkinEntity::_getVertexAnimTempBufferInfo(void)
	{
		return &mTempVertexAnimInfo;
	}

	matrix4 toBase(Ogre::Matrix4 const& mat)
	{
		matrix4 temp;
		temp.setValue(mat[0][0], mat[0][1], mat[0][2], mat[0][3],
					mat[1][0], mat[1][1], mat[1][2], mat[1][3],
					mat[2][0], mat[2][1], mat[2][2], mat[2][3],
					mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
		return temp;
	}
	//-----------------------------------------------------------------------
    void SkinEntity::cacheBoneMatrices(void)
    {
        Root& root = Root::getSingleton();
//        unsigned long currentFrameNumber = root.getCurrentFrameNumber();
        // taesoo. if (*mFrameBonesLastUpdated  != currentFrameNumber) {
		{
            mSkeletonInstance->setAnimationState(*mAnimationState);
            mSkeletonInstance->_getBoneMatrices(mBoneMatrices);
            //*mFrameBonesLastUpdated  = currentFrameNumber;
        }

		_cachedDualQuaternions.resize(mNumBoneMatrices);

		for(int i=1; i<mNumBoneMatrices; i++)
		{
			_cachedDualQuaternions[i]=toBase(mBoneMatrices[i]);
		}
    }
    //-----------------------------------------------------------------------
    void SkinEntity::setDisplaySkeleton(bool display)
    {
        mDisplaySkeleton = display;
    }
    //-----------------------------------------------------------------------
    bool SkinEntity::getDisplaySkeleton(void) const
    {
        return mDisplaySkeleton;
    }
    //-----------------------------------------------------------------------
    SkinEntity* SkinEntity::getManualLodLevel(size_t index) const
    {
        assert(index < mLodSkinEntityList.size());

        return mLodSkinEntityList[index];
    }
    //-----------------------------------------------------------------------
    size_t SkinEntity::getNumManualLodLevels(void) const
    {
        return mLodSkinEntityList.size();
    }
    //-----------------------------------------------------------------------
    void SkinEntity::setMeshLodBias(Real factor, ushort maxDetailIndex, ushort minDetailIndex)
    {
        assert(factor > 0.0f && "Bias factor must be > 0!");
        mMeshLodFactorTransformed = 1.0f / factor;
        mMaxMeshLodIndex = maxDetailIndex;
        mMinMeshLodIndex = minDetailIndex;

    }
    //-----------------------------------------------------------------------
    void SkinEntity::setMaterialLodBias(Real factor, ushort maxDetailIndex, ushort minDetailIndex)
    {
        assert(factor > 0.0f && "Bias factor must be > 0!");
        mMaterialLodFactorTransformed = 1.0f / factor;
        mMaxMaterialLodIndex = maxDetailIndex;
        mMinMaterialLodIndex = minDetailIndex;

    }
    //-----------------------------------------------------------------------
    void SkinEntity::buildSkinSubEntityList(MeshPtr& mesh, SkinSubEntityList* sublist)
    {
        // Create SubEntities
        unsigned short i, numSubMeshes;
        SubMesh* subMesh;
        SkinSubEntity* subEnt;

        numSubMeshes = mesh->getNumSubMeshes();
        for (i = 0; i < numSubMeshes; ++i)
        {
            subMesh = mesh->getSubMesh(i);
            subEnt = new SkinSubEntity(this, subMesh);
#if OGRE_VERSION_MAJOR<13
            if (subMesh->isMatInitialised())
                subEnt->setMaterialName(subMesh->getMaterialName());
#else
            if (subMesh->getMaterial())
                subEnt->setMaterial(subMesh->getMaterial());
#endif
            sublist->push_back(subEnt);
        }
    }
    //-----------------------------------------------------------------------
    void SkinEntity::setPolygonModeOverrideable(bool overrideable)
    {
        SkinSubEntityList::iterator i, iend;
        iend = mSkinSubEntityList.end();

        for( i = mSkinSubEntityList.begin(); i != iend; ++i )
        {
            (*i)->setPolygonModeOverrideable(overrideable);
        }
    }

    
    //-----------------------------------------------------------------------
    void SkinEntity::attachObjectImpl(MovableObject *pObject, TagPoint *pAttachingPoint)
    {
        assert(mChildObjectList.find(pObject->getName()) == mChildObjectList.end());
        mChildObjectList[pObject->getName()] = pObject;
        pObject->_notifyAttached(pAttachingPoint, true);
    }

    //-----------------------------------------------------------------------
    MovableObject* SkinEntity::detachObjectFromBone(const String &name)
    {
        ChildObjectList::iterator i = mChildObjectList.find(name);

        if (i == mChildObjectList.end())
        {
            OGRE_EXCEPT(Exception::ERR_ITEM_NOT_FOUND, "No child object entry found named " + name,
                "SkinEntity::detachObjectFromBone");
        }
        MovableObject *obj = i->second;
        detachObjectImpl(obj);
        mChildObjectList.erase(i);

        // Trigger update of bounding box if necessary
        if (mParentNode)
            mParentNode->needUpdate();

        return obj;
    }
    //-----------------------------------------------------------------------
    void SkinEntity::detachObjectFromBone(MovableObject* obj)
    {
        ChildObjectList::iterator i, iend;
        iend = mChildObjectList.end();
        for (i = mChildObjectList.begin(); i != iend; ++i)
        {
            if (i->second == obj)
            {
                detachObjectImpl(obj);
                mChildObjectList.erase(i);

                // Trigger update of bounding box if necessary
                if (mParentNode)
                    mParentNode->needUpdate();
                break;
            }
        }
    }
    //-----------------------------------------------------------------------
    void SkinEntity::detachAllObjectsFromBone(void)
    {
        detachAllObjectsImpl();

        // Trigger update of bounding box if necessary
        if (mParentNode)
            mParentNode->needUpdate();
    }
    //-----------------------------------------------------------------------
    void SkinEntity::detachObjectImpl(MovableObject* pObject)
    {
        TagPoint* tp = static_cast<TagPoint*>(pObject->getParentNode());

        // free the TagPoint so we can reuse it later
        mSkeletonInstance->freeTagPoint(tp);

        pObject->_notifyAttached((TagPoint*)0);
    }
    //-----------------------------------------------------------------------
    void SkinEntity::detachAllObjectsImpl(void)
    {
        ChildObjectList::const_iterator i, iend;
        iend = mChildObjectList.end();
        for (i = mChildObjectList.begin(); i != iend; ++i)
        {
            detachObjectImpl(i->second);
        }
        mChildObjectList.clear();
    }

    //-----------------------------------------------------------------------
    SkinEntity::ChildObjectListIterator SkinEntity::getAttachedObjectIterator()
    {
        return ChildObjectListIterator(mChildObjectList.begin(), mChildObjectList.end());
    }
    //-----------------------------------------------------------------------
	Real SkinEntity::getBoundingRadius(void) const
    {
        Real rad = mMesh->getBoundingSphereRadius();
        // Scale by largest scale factor
        if (mParentNode)
        {
            const Vector3& s = mParentNode->_getDerivedScale();
            rad *= std::max(s.x, std::max(s.y, s.z));
        }
        return rad;
    }
    //-----------------------------------------------------------------------
    void SkinEntity::prepareTempBlendBuffers(void)
    {
        if (mSkelAnimVertexData)
        {
            delete mSkelAnimVertexData;
            mSkelAnimVertexData = 0;
        }
		if (mSoftwareVertexAnimVertexData)
		{
			delete mSoftwareVertexAnimVertexData;
			mSoftwareVertexAnimVertexData = 0;
		}
		if (mHardwareVertexAnimVertexData)
		{
			delete mHardwareVertexAnimVertexData;
			mHardwareVertexAnimVertexData = 0;
		}

		if (hasVertexAnimation())
		{
			// Shared data
			if (mMesh->sharedVertexData
				&& mMesh->getSharedVertexDataAnimationType() != VAT_NONE)
			{
				// Create temporary vertex blend info
				// Prepare temp vertex data if needed
				// Clone without copying data, don't remove any blending info
				// (since if we skeletally animate too, we need it)
				mSoftwareVertexAnimVertexData = mMesh->sharedVertexData->clone(false);
				extractTempBufferInfo(mSoftwareVertexAnimVertexData, &mTempVertexAnimInfo);

				// Also clone for hardware usage, don't remove blend info since we'll
				// need it if we also hardware skeletally animate
				mHardwareVertexAnimVertexData = mMesh->sharedVertexData->clone(false);
			}
		}

        if (hasSkeleton())
        {
            // Shared data
            if (mMesh->sharedVertexData)
            {
                // Create temporary vertex blend info
                // Prepare temp vertex data if needed
                // Clone without copying data, remove blending info
                // (since blend is performed in software)
                mSkelAnimVertexData =
                    cloneVertexDataRemoveBlendInfo(mMesh->sharedVertexData);
                extractTempBufferInfo(mSkelAnimVertexData, &mTempSkelAnimInfo);
            }

        }

		// Do SubEntities
		SkinSubEntityList::iterator i, iend;
		iend = mSkinSubEntityList.end();
		for (i = mSkinSubEntityList.begin(); i != iend; ++i)
		{
			SkinSubEntity* s = *i;
			s->prepareTempBlendBuffers();
		}

        // It's prepared for shadow volumes only if mesh has been prepared for shadow volumes.
        mPreparedForShadowVolumes = mMesh->isPreparedForShadowVolumes();
    }
    //-----------------------------------------------------------------------
    void SkinEntity::extractTempBufferInfo(VertexData* sourceData, TempBlendedBufferInfo* info)
    {
        info->extractFrom(sourceData);
    }
    //-----------------------------------------------------------------------
    VertexData* SkinEntity::cloneVertexDataRemoveBlendInfo(const VertexData* source)
    {
        // Clone without copying data
        VertexData* ret = source->clone(false);
        const VertexElement* blendIndexElem =
            source->vertexDeclaration->findElementBySemantic(VES_BLEND_INDICES);
        const VertexElement* blendWeightElem =
            source->vertexDeclaration->findElementBySemantic(VES_BLEND_WEIGHTS);
        // Remove blend index
        if (blendIndexElem)
        {
            // Remove buffer reference
            ret->vertexBufferBinding->unsetBinding(blendIndexElem->getSource());

        }
        if (blendWeightElem &&
            blendWeightElem->getSource() != blendIndexElem->getSource())
        {
            // Remove buffer reference
            ret->vertexBufferBinding->unsetBinding(blendWeightElem->getSource());
        }
        // remove elements from declaration
        ret->vertexDeclaration->removeElement(VES_BLEND_INDICES);
        ret->vertexDeclaration->removeElement(VES_BLEND_WEIGHTS);

        // Close gaps in bindings for effective and safely
        ret->closeGapsInBindings();

        return ret;
    }
    //-----------------------------------------------------------------------
    EdgeData* SkinEntity::getEdgeList(void)
    {
        // Get from Mesh
        return mMesh->getEdgeList(mMeshLodIndex);
    }
	//-----------------------------------------------------------------------
    bool SkinEntity::hasEdgeList(void)
    {
        // check if mesh has an edge list attached
        // give mesh a chance to built it if scheduled
        return (mMesh->getEdgeList(mMeshLodIndex) != NULL);
    }
    //-----------------------------------------------------------------------
    void SkinEntity::reevaluateVertexProcessing(void)
    {
        // init
        mHardwareAnimation = false;
        mVertexProgramInUse = false; // assume false because we just assign this
        bool firstPass = true;

        SkinSubEntityList::iterator i, iend;
        iend = mSkinSubEntityList.end();
        for (i = mSkinSubEntityList.begin(); i != iend; ++i)
        {
			SkinSubEntity* sub = *i;
            const MaterialPtr& m = sub->getMaterial();
            // Make sure it's loaded
            m->load();
            Technique* t = m->getBestTechnique();
            if (!t)
            {
                // No supported techniques
                continue;
            }
            Pass* p = t->getPass(0);
            if (!p)
            {
                // No passes, invalid
                continue;
            }
            if (p->hasVertexProgram())
            {
                // If one material uses a vertex program, set this flag
                // Causes some special processing like forcing a separate light cap
                mVertexProgramInUse = true;

                if (hasSkeleton())
				{
					// All materials must support skinning for us to consider using
					// hardware animation - if one fails we use software
					if (firstPass)
					{
						mHardwareAnimation = p->getVertexProgram()->isSkeletalAnimationIncluded();
						firstPass = false;
					}
					else
					{
						mHardwareAnimation = mHardwareAnimation &&
							p->getVertexProgram()->isSkeletalAnimationIncluded();
					}
				}

				VertexAnimationType animType = VAT_NONE;
				if (sub->getSubMesh()->useSharedVertices)
				{
					animType = mMesh->getSharedVertexDataAnimationType();
				}
				else
				{
					animType = sub->getSubMesh()->getVertexAnimationType();
				}
				if (animType == VAT_MORPH)
				{
					// All materials must support morph animation for us to consider using
					// hardware animation - if one fails we use software
					if (firstPass)
					{
						mHardwareAnimation = p->getVertexProgram()->isMorphAnimationIncluded();
						firstPass = false;
					}
					else
					{
						mHardwareAnimation = mHardwareAnimation &&
							p->getVertexProgram()->isMorphAnimationIncluded();
					}
				}
				else if (animType == VAT_POSE)
				{
					// All materials must support pose animation for us to consider using
					// hardware animation - if one fails we use software
					if (firstPass)
					{
						mHardwareAnimation = p->getVertexProgram()->isPoseAnimationIncluded();
						if (sub->getSubMesh()->useSharedVertices)
							mHardwarePoseCount = p->getVertexProgram()->getNumberOfPosesIncluded();
						else
							sub->mHardwarePoseCount = p->getVertexProgram()->getNumberOfPosesIncluded();
						firstPass = false;
					}
					else
					{
						mHardwareAnimation = mHardwareAnimation &&
							p->getVertexProgram()->isPoseAnimationIncluded();
						if (sub->getSubMesh()->useSharedVertices)
							mHardwarePoseCount = std::max(mHardwarePoseCount,
								p->getVertexProgram()->getNumberOfPosesIncluded());
						else
							sub->mHardwarePoseCount = std::max(sub->mHardwarePoseCount,
								p->getVertexProgram()->getNumberOfPosesIncluded());
					}
				}

            }
        }

        // Should be force update of animation if they exists, due reevaluate
        // vertex processing might switchs between hardware/software animation,
        // and then we'll end with NULL or incorrect mBoneWorldMatrices, or
        // incorrect blended software animation buffers.
        if (mAnimationState)
        {
            mFrameAnimationLastUpdated = mAnimationState->getDirtyFrameNumber() - 1;
        }
    }
    //-----------------------------------------------------------------------
    ShadowCaster::ShadowRenderableListIterator
        SkinEntity::getShadowVolumeRenderableIterator(
        ShadowTechnique shadowTechnique, const Light* light,
        HardwareIndexBufferSharedPtr* indexBuffer,
        bool extrude, Real extrusionDistance, unsigned long flags)
    {
        assert(indexBuffer && "Only external index buffers are supported right now");
        assert((*indexBuffer)->getType() == HardwareIndexBuffer::IT_16BIT &&
            "Only 16-bit indexes supported for now");

        // Potentially delegate to LOD SkinEntity
        if (isLodManual(mMesh) && mMeshLodIndex > 0)
        {
            // Use alternate SkinEntity
            assert( static_cast< size_t >( mMeshLodIndex - 1 ) < mLodSkinEntityList.size() &&
                "No LOD SkinEntityList - did you build the manual LODs after creating the SkinEntity?");
            // delegate, we're using manual LOD and not the top lod index
            if (hasSkeleton() && mLodSkinEntityList[mMeshLodIndex - 1]->hasSkeleton())
            {
                // Copy the animation state set to lod SkinEntity, we assume the lod
                // SkinEntity only has a subset animation states
                mAnimationState->copyMatchingState(
					mLodSkinEntityList[mMeshLodIndex - 1]->mAnimationState);
            }
            return mLodSkinEntityList[mMeshLodIndex-1]->getShadowVolumeRenderableIterator(
                shadowTechnique, light, indexBuffer, extrude,
                extrusionDistance, flags);
        }


        // Prepare temp buffers if required
        if (!mPreparedForShadowVolumes)
        {
            mMesh->prepareForShadowVolume();
            // reset frame last updated to force update of animations if they exist
            if (mAnimationState)
                mFrameAnimationLastUpdated = mAnimationState->getDirtyFrameNumber() - 1;
            // re-prepare buffers
            prepareTempBlendBuffers();
        }


        bool hasAnimation = (hasSkeleton() || hasVertexAnimation());

        // Update any animation
        if (hasAnimation)
        {
            updateAnimation();
        }

        // Calculate the object space light details
        Vector4 lightPos = light->getAs4DVector();
#if OGRE_VERSION_MAJOR<13
        Matrix4 world2Obj = mParentNode->_getFullTransform().inverseAffine();
        lightPos = world2Obj.transformAffine(lightPos);
#else
        Affine3 world2Obj = mParentNode->_getFullTransform().inverse();
        lightPos = world2Obj * lightPos;
#endif
        // We need to search the edge list for silhouette edges
        EdgeData* edgeList = getEdgeList();

		if (!edgeList)
		{
			// we can't get an edge list for some reason, return blank
			// really we shouldn't be able to get here, but this is a safeguard
			return ShadowRenderableListIterator(mShadowRenderables.begin(), mShadowRenderables.end());
		}

        // Init shadow renderable list if required
        bool init = mShadowRenderables.empty();

        EdgeData::EdgeGroupList::iterator egi;
        ShadowRenderableList::iterator si, siend;
        SkinEntityShadowRenderable* esr = 0;
        if (init)
            mShadowRenderables.resize(edgeList->edgeGroups.size());

        bool isAnimated = hasAnimation;
        bool updatedSharedGeomNormals = false;
        siend = mShadowRenderables.end();
        egi = edgeList->edgeGroups.begin();
        for (si = mShadowRenderables.begin(); si != siend; ++si, ++egi)
        {
            const VertexData *pVertData;
            if (isAnimated)
            {
                // Use temp buffers
                pVertData = findBlendedVertexData(egi->vertexData);
            }
            else
            {
                pVertData = egi->vertexData;
            }
            if (init)
            {
                // Try to find corresponding SkinSubEntity; this allows the
                // linkage of visibility between ShadowRenderable and SkinSubEntity
                SkinSubEntity* subent = findSkinSubEntityForVertexData(egi->vertexData);
                // Create a new renderable, create a separate light cap if
                // we're using a vertex program (either for this model, or
                // for extruding the shadow volume) since otherwise we can
                // get depth-fighting on the light cap

                *si = new SkinEntityShadowRenderable(this, indexBuffer, pVertData,
                    mVertexProgramInUse || !extrude, subent);
            }
            else
            {
                // If we have animation, we have no guarantee that the position
                // buffer we used last frame is the same one we used last frame
                // since a temporary buffer is requested each frame
                // therefore, we need to update the SkinEntityShadowRenderable
                // with the current position buffer
                static_cast<SkinEntityShadowRenderable*>(*si)->rebindPositionBuffer(pVertData, hasAnimation);

            }
            // Get shadow renderable
            esr = static_cast<SkinEntityShadowRenderable*>(*si);
            HardwareVertexBufferSharedPtr esrPositionBuffer = esr->getPositionBuffer();
            // For animated entities we need to recalculate the face normals
            if (hasAnimation)
            {
                if (egi->vertexData != mMesh->sharedVertexData || !updatedSharedGeomNormals)
                {
                    // recalculate face normals
                    edgeList->updateFaceNormals(egi->vertexSet, esrPositionBuffer);
                    // If we're not extruding in software we still need to update
                    // the latter part of the buffer (the hardware extruded part)
                    // with the latest animated positions
                    if (!extrude)
                    {
                        // Lock, we'll be locking the (suppressed hardware update) shadow buffer
                        float* pSrc = static_cast<float*>(
                            esrPositionBuffer->lock(HardwareBuffer::HBL_NORMAL));
                        float* pDest = pSrc + (egi->vertexData->vertexCount * 3);
                        memcpy(pDest, pSrc, sizeof(float) * 3 * egi->vertexData->vertexCount);
                        esrPositionBuffer->unlock();
                    }
                    if (egi->vertexData == mMesh->sharedVertexData)
                    {
                        updatedSharedGeomNormals = true;
                    }
                }
            }
            // Extrude vertices in software if required
            if (extrude)
            {
                extrudeVertices(esrPositionBuffer,
                    egi->vertexData->vertexCount,
                    lightPos, extrusionDistance);

            }
            // Stop suppressing hardware update now, if we were
            esrPositionBuffer->suppressHardwareUpdate(false);

        }
        // Calc triangle light facing
        updateEdgeListLightFacing(edgeList, lightPos);

        // Generate indexes and update renderables
#if OGRE_VERSION_MAJOR<13
        generateShadowVolume(edgeList, *indexBuffer, light,
            mShadowRenderables, flags);
#else
		size_t indexBufferUsedSize;
        generateShadowVolume(edgeList, *indexBuffer, indexBufferUsedSize, light,
            mShadowRenderables, flags);
#endif


        return ShadowRenderableListIterator(mShadowRenderables.begin(), mShadowRenderables.end());
    }
    //-----------------------------------------------------------------------
    const VertexData* SkinEntity::findBlendedVertexData(const VertexData* orig)
    {
		bool skel = hasSkeleton();

        if (orig == mMesh->sharedVertexData)
        {
			return skel? mSkelAnimVertexData : mSoftwareVertexAnimVertexData;
        }
        SkinSubEntityList::iterator i, iend;
        iend = mSkinSubEntityList.end();
        for (i = mSkinSubEntityList.begin(); i != iend; ++i)
        {
            SkinSubEntity* se = *i;
            if (orig == se->getSubMesh()->vertexData)
            {
				return skel? se->_getSkelAnimVertexData() : se->_getSoftwareVertexAnimVertexData();
            }
        }
        // None found
        OGRE_EXCEPT(Exception::ERR_ITEM_NOT_FOUND,
            "Cannot find blended version of the vertex data specified.",
            "SkinEntity::findBlendedVertexData");
    }
    //-----------------------------------------------------------------------
    SkinSubEntity* SkinEntity::findSkinSubEntityForVertexData(const VertexData* orig)
    {
        if (orig == mMesh->sharedVertexData)
        {
            return 0;
        }

        SkinSubEntityList::iterator i, iend;
        iend = mSkinSubEntityList.end();
        for (i = mSkinSubEntityList.begin(); i != iend; ++i)
        {
            SkinSubEntity* se = *i;
            if (orig == se->getSubMesh()->vertexData)
            {
                return se;
            }
        }

        // None found
        return 0;
    }
    //-----------------------------------------------------------------------
    void SkinEntity::addSoftwareAnimationRequest(bool normalsAlso)
    {
        mSoftwareAnimationRequests++;
        if (normalsAlso) {
            mSoftwareAnimationNormalsRequests++;
        }
    }
    //-----------------------------------------------------------------------
    void SkinEntity::removeSoftwareAnimationRequest(bool normalsAlso)
    {
        if (mSoftwareAnimationRequests == 0 ||
            (normalsAlso && mSoftwareAnimationNormalsRequests == 0))
        {
            OGRE_EXCEPT(Exception::ERR_INVALIDPARAMS,
                        "Attempt to remove nonexistant request.",
                        "SkinEntity::removeSoftwareAnimationRequest");
        }
        mSoftwareAnimationRequests--;
        if (normalsAlso) {
            mSoftwareAnimationNormalsRequests--;
        }
    }
    //-----------------------------------------------------------------------
    void SkinEntity::_notifyAttached(Node* parent, bool isTagPoint)
    {
        MovableObject::_notifyAttached(parent, isTagPoint);
        // Also notify LOD entities
        LODSkinEntityList::iterator i, iend;
        iend = mLodSkinEntityList.end();
        for (i = mLodSkinEntityList.begin(); i != iend; ++i)
        {
            (*i)->_notifyAttached(parent, isTagPoint);
        }

    }
    //-----------------------------------------------------------------------
    //-----------------------------------------------------------------------
    SkinEntity::SkinEntityShadowRenderable::SkinEntityShadowRenderable(SkinEntity* parent,
        HardwareIndexBufferSharedPtr* indexBuffer, const VertexData* vertexData,
        bool createSeparateLightCap, SkinSubEntity* subent, bool isLightCap)
        : mParent(parent), mSkinSubEntity(subent)
    {
        // Save link to vertex data
        mCurrentVertexData = vertexData;

        // Initialise render op
        mRenderOp.indexData = new IndexData();
        mRenderOp.indexData->indexBuffer = *indexBuffer;
        mRenderOp.indexData->indexStart = 0;
        // index start and count are sorted out later

        // Create vertex data which just references position component (and 2 component)
        mRenderOp.vertexData = new VertexData();
        // Map in position data
        mRenderOp.vertexData->vertexDeclaration->addElement(0,0,VET_FLOAT3, VES_POSITION);
        mOriginalPosBufferBinding =
            vertexData->vertexDeclaration->findElementBySemantic(VES_POSITION)->getSource();
        mPositionBuffer = vertexData->vertexBufferBinding->getBuffer(mOriginalPosBufferBinding);
        mRenderOp.vertexData->vertexBufferBinding->setBinding(0, mPositionBuffer);
        // Map in w-coord buffer (if present)
        if(!vertexData->hardwareShadowVolWBuffer.isNull())
        {
            mRenderOp.vertexData->vertexDeclaration->addElement(1,0,VET_FLOAT1, VES_TEXTURE_COORDINATES, 0);
            mWBuffer = vertexData->hardwareShadowVolWBuffer;
            mRenderOp.vertexData->vertexBufferBinding->setBinding(1, mWBuffer);
        }
        // Use same vertex start as input
        mRenderOp.vertexData->vertexStart = vertexData->vertexStart;

        if (isLightCap)
        {
            // Use original vertex count, no extrusion
            mRenderOp.vertexData->vertexCount = vertexData->vertexCount;
        }
        else
        {
            // Vertex count must take into account the doubling of the buffer,
            // because second half of the buffer is the extruded copy
            mRenderOp.vertexData->vertexCount =
                vertexData->vertexCount * 2;
            if (createSeparateLightCap)
            {
                // Create child light cap
                mLightCap = new SkinEntityShadowRenderable(parent,
                    indexBuffer, vertexData, false, subent, true);
            }
        }

    }
    //-----------------------------------------------------------------------
    SkinEntity::SkinEntityShadowRenderable::~SkinEntityShadowRenderable()
    {
        delete mRenderOp.indexData;
        delete mRenderOp.vertexData;
    }
    //-----------------------------------------------------------------------
    void SkinEntity::SkinEntityShadowRenderable::getWorldTransforms(Matrix4* xform) const
    {
        *xform = mParent->_getParentNodeFullTransform();
    }
    //-----------------------------------------------------------------------
    const Quaternion& SkinEntity::SkinEntityShadowRenderable::getWorldOrientation(void) const
    {
        return mParent->getParentNode()->_getDerivedOrientation();
    }
    //-----------------------------------------------------------------------
    const Vector3& SkinEntity::SkinEntityShadowRenderable::getWorldPosition(void) const
    {
        return mParent->getParentNode()->_getDerivedPosition();
    }
    //-----------------------------------------------------------------------
    void SkinEntity::SkinEntityShadowRenderable::rebindPositionBuffer(const VertexData* vertexData, bool force)
    {
        if (force || mCurrentVertexData != vertexData)
        {
            mCurrentVertexData = vertexData;
            mPositionBuffer = mCurrentVertexData->vertexBufferBinding->getBuffer(
                mOriginalPosBufferBinding);
            mRenderOp.vertexData->vertexBufferBinding->setBinding(0, mPositionBuffer);
            if (mLightCap)
            {
                static_cast<SkinEntityShadowRenderable*>(mLightCap)->rebindPositionBuffer(vertexData, force);
            }
        }
    }
    //-----------------------------------------------------------------------
    bool SkinEntity::SkinEntityShadowRenderable::isVisible(void) const
    {
        if (mSkinSubEntity)
        {
            return mSkinSubEntity->isVisible();
        }
        else
        {
            return ShadowRenderable::isVisible();
        }
    }
    //-----------------------------------------------------------------------
    void SkinEntity::setRenderQueueGroup(uint8 queueID)
    {
        MovableObject::setRenderQueueGroup(queueID);

        // Set render queue for all manual LOD entities
        if (isLodManual(mMesh))
        {
            LODSkinEntityList::iterator li, liend;
            liend = mLodSkinEntityList.end();
            for (li = mLodSkinEntityList.begin(); li != liend; ++li)
            {
                (*li)->setRenderQueueGroup(queueID);
            }
        }
    }
    //-----------------------------------------------------------------------
    void SkinEntity::shareSkeletonInstanceWith(SkinEntity* SkinEntity)
    {
        if (SkinEntity->getMesh()->getSkeleton() != getMesh()->getSkeleton())
        {
            OGRE_EXCEPT(Exception::ERR_RT_ASSERTION_FAILED,
                "The supplied SkinEntity has a different skeleton.",
                "SkinEntity::shareSkeletonWith");
        }
        if (!mSkeletonInstance)
        {
            OGRE_EXCEPT(Exception::ERR_RT_ASSERTION_FAILED,
                "This SkinEntity has no skeleton.",
                "SkinEntity::shareSkeletonWith");
        }
        if (mSharedSkeletonEntities != NULL && SkinEntity->mSharedSkeletonEntities != NULL)
        {
            OGRE_EXCEPT(Exception::ERR_RT_ASSERTION_FAILED,
                "Both entities already shares their SkeletonInstances! At least "
                "one of the instances must not share it's instance.",
                "SkinEntity::shareSkeletonWith");
        }

        //check if we already share our skeletoninstance, we don't want to delete it if so
        if (mSharedSkeletonEntities != NULL)
        {
            SkinEntity->shareSkeletonInstanceWith(this);
        }
        else
        {
            delete mSkeletonInstance;
            AlignedMemory::deallocate(mBoneMatrices);
            delete mAnimationState;
            delete mFrameBonesLastUpdated;
            mSkeletonInstance = SkinEntity->mSkeletonInstance;
            mNumBoneMatrices = SkinEntity->mNumBoneMatrices;
            mBoneMatrices = SkinEntity->mBoneMatrices;
            mAnimationState = SkinEntity->mAnimationState;
            mFrameBonesLastUpdated = SkinEntity->mFrameBonesLastUpdated;
            if (SkinEntity->mSharedSkeletonEntities == NULL)
            {
                SkinEntity->mSharedSkeletonEntities = new SkinEntitySet();
                SkinEntity->mSharedSkeletonEntities->insert(SkinEntity);
            }
            mSharedSkeletonEntities = SkinEntity->mSharedSkeletonEntities;
            mSharedSkeletonEntities->insert(this);
        }
    }
    //-----------------------------------------------------------------------
    void SkinEntity::stopSharingSkeletonInstance()
    {
        if (mSharedSkeletonEntities == NULL)
        {
            OGRE_EXCEPT(Exception::ERR_RT_ASSERTION_FAILED,
                "This SkinEntity is not sharing it's skeletoninstance.",
                "SkinEntity::shareSkeletonWith");
        }
        //check if there's no other than us sharing the skeleton instance
        if (mSharedSkeletonEntities->size() == 1)
        {
            //just reset
            delete mSharedSkeletonEntities;
            mSharedSkeletonEntities = 0;
        }
        else
        {
            mSkeletonInstance = new SkeletonInstance(mMesh->getSkeleton());
            mSkeletonInstance->load();
            mAnimationState = new AnimationStateSet();
            mMesh->_initAnimationState(mAnimationState);
            mFrameBonesLastUpdated = new unsigned long(std::numeric_limits<unsigned long>::max());
            mNumBoneMatrices = mSkeletonInstance->getNumBones();
#if OGRE_VERSION_MAJOR<13
            mBoneMatrices = static_cast<Matrix4*>(AlignedMemory::allocate(sizeof(Matrix4) * mNumBoneMatrices));
#else
            mBoneMatrices = static_cast<Affine3*>(OGRE_MALLOC_SIMD(sizeof(Affine3) * mNumBoneMatrices, MEMCATEGORY_ANIMATION));
#endif

            mSharedSkeletonEntities->erase(this);
            if (mSharedSkeletonEntities->size() == 1)
            {
                (*mSharedSkeletonEntities->begin())->stopSharingSkeletonInstance();
            }
            mSharedSkeletonEntities = 0;
        }
    }
    //-----------------------------------------------------------------------
	void SkinEntity::refreshAvailableAnimationState(void)
	{
		mMesh->_refreshAnimationState(mAnimationState);
	}
	//-----------------------------------------------------------------------
	uint32 SkinEntity::getTypeFlags(void) const
	{
		//ASSERT(0);
		return SceneManager::ENTITY_TYPE_MASK;
	}
	//-----------------------------------------------------------------------
	VertexData* SkinEntity::getVertexDataForBinding(void)
	{
		SkinEntity::VertexDataBindChoice c =
			chooseVertexDataForBinding(mMesh->getSharedVertexDataAnimationType() != VAT_NONE);
		switch(c)
		{
		case BIND_ORIGINAL:
			return mMesh->sharedVertexData;
		case BIND_HARDWARE_MORPH:
			return mHardwareVertexAnimVertexData;
		case BIND_SOFTWARE_MORPH:
			return mSoftwareVertexAnimVertexData;
		case BIND_SOFTWARE_SKELETAL:
			return mSkelAnimVertexData;
		};
		// keep compiler happy
		return mMesh->sharedVertexData;
	}
	//-----------------------------------------------------------------------
	SkinEntity::VertexDataBindChoice SkinEntity::chooseVertexDataForBinding(bool vertexAnim) const
	{
		if (hasSkeleton())
		{
			if (!mHardwareAnimation)
			{
				// all software skeletal binds same vertex data
				// may be a 2-stage s/w transform including morph earlier though
				return BIND_SOFTWARE_SKELETAL;
			}
			else if (vertexAnim)
			{
				// hardware morph animation
				return BIND_HARDWARE_MORPH;
			}
			else
			{
				// hardware skeletal, no morphing
				return BIND_ORIGINAL;
			}
		}
		else if (vertexAnim)
		{
			// morph only, no skeletal
			if (mHardwareAnimation)
			{
				return BIND_HARDWARE_MORPH;
			}
			else
			{
				return BIND_SOFTWARE_MORPH;
			}

		}
		else
		{
			return BIND_ORIGINAL;
		}

	}
	//---------------------------------------------------------------------
	void SkinEntity::visitRenderables(Renderable::Visitor* visitor, 
		bool debugRenderables)
	{
		// Visit each SubEntity
		for (SkinSubEntityList::iterator i = mSkinSubEntityList.begin(); i != mSkinSubEntityList.end(); ++i)
		{
			visitor->visit(*i, 0, false);
		}
		// if manual LOD is in use, visit those too
		ushort lodi = 1;
		for (LODSkinEntityList::iterator e = mLodSkinEntityList.begin(); 
			e != mLodSkinEntityList.end(); ++e, ++lodi)
		{
			
			uint nsub = (*e)->getNumSubEntities();
			for (uint s = 0; s < nsub; ++s)
			{
				visitor->visit((*e)->getSkinSubEntity(s), lodi, false);
			}
		}

	}
	//-----------------------------------------------------------------------
	//-----------------------------------------------------------------------
	String SkinEntityFactory::FACTORY_TYPE_NAME = "SkinEntity";
	//-----------------------------------------------------------------------
	const String& SkinEntityFactory::getType(void) const
	{
		return FACTORY_TYPE_NAME;
	}
	//-----------------------------------------------------------------------
	MovableObject* SkinEntityFactory::createInstanceImpl( const String& name,
		const NameValuePairList* params)
	{
		// must have mesh parameter
		MeshPtr pMesh;
		if (params != 0)
		{
			NameValuePairList::const_iterator ni = params->find("mesh");
			if (ni != params->end())
			{
				// Get mesh (load if required)
				pMesh = MeshManager::getSingleton().load(
					ni->second,
					// autodetect group location
					ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME );
			}

		}
		if (pMesh.isNull())
		{
			OGRE_EXCEPT(Exception::ERR_INVALIDPARAMS,
				"'mesh' parameter required when constructing an SkinEntity.",
				"SkinEntityFactory::createInstance");
		}

		return new SkinEntity(name, pMesh);

	}
	//-----------------------------------------------------------------------
	void SkinEntityFactory::destroyInstance( MovableObject* obj)
	{
		delete obj;
	}


#if OGRE_VERSION_MAJOR<13
	// taesoo
	void SkinEntity::softwareVertexBlend(Matrix4 *boneMatrices, Ogre::Mesh::IndexMap& indexMap, VertexData* sourceVertexData, VertexData* targetVertexData, bool blendNormals)
	{
#ifdef USE_ORIGINAL_IMPLEMENTATION
        const Matrix4* blendMatrices[256];
		Mesh::prepareMatricesForVertexBlend(blendMatrices, boneMatrices, indexMap);
		Mesh::softwareVertexBlend(sourceVertexData, targetVertexData, blendMatrices, indexMap.size(), blendNormals);
#else
		const Matrix4* blendMatrices[256];		
		const Matrix4** pBlendMatrices=blendMatrices;
        assert(indexMap.size() <= 256);
		Ogre::Mesh::IndexMap::iterator it, itend;
        itend = indexMap.end();
        for (it = indexMap.begin(); it != itend; ++it)
            *pBlendMatrices++ = boneMatrices + *it;

		float *pSrcPos = 0;
        float *pSrcNorm = 0;
        float *pDestPos = 0;
        float *pDestNorm = 0;
        float *pBlendWeight = 0;
        unsigned char* pBlendIdx = 0;
        size_t srcPosStride = 0;
        size_t srcNormStride = 0;
        size_t destPosStride = 0;
        size_t destNormStride = 0;
        size_t blendWeightStride = 0;
        size_t blendIdxStride = 0;


        // Get elements for source
        const VertexElement* srcElemPos = sourceVertexData->vertexDeclaration->findElementBySemantic(VES_POSITION);
        const VertexElement* srcElemNorm = sourceVertexData->vertexDeclaration->findElementBySemantic(VES_NORMAL);
        const VertexElement* srcElemBlendIndices = sourceVertexData->vertexDeclaration->findElementBySemantic(VES_BLEND_INDICES);
        const VertexElement* srcElemBlendWeights = sourceVertexData->vertexDeclaration->findElementBySemantic(VES_BLEND_WEIGHTS);
        assert (srcElemPos && srcElemBlendIndices && srcElemBlendWeights && "You must supply at least positions, blend indices and blend weights");
        
		// Get elements for target
        const VertexElement* destElemPos = targetVertexData->vertexDeclaration->findElementBySemantic(VES_POSITION);
        const VertexElement* destElemNorm = targetVertexData->vertexDeclaration->findElementBySemantic(VES_NORMAL);

        // Do we have normals and want to blend them?
        bool includeNormals = blendNormals && (srcElemNorm != NULL) && (destElemNorm != NULL);


        // Get buffers for source
        HardwareVertexBufferSharedPtr srcPosBuf, srcNormBuf, srcIdxBuf, srcWeightBuf;
        srcPosBuf = sourceVertexData->vertexBufferBinding->getBuffer(srcElemPos->getSource());
        srcPosStride = srcPosBuf->getVertexSize();
        srcIdxBuf = sourceVertexData->vertexBufferBinding->getBuffer(srcElemBlendIndices->getSource());
        blendIdxStride = srcIdxBuf->getVertexSize();
        srcWeightBuf = sourceVertexData->vertexBufferBinding->getBuffer(srcElemBlendWeights->getSource());
        blendWeightStride = srcWeightBuf->getVertexSize();
        if (includeNormals)
        {
            srcNormBuf = sourceVertexData->vertexBufferBinding->getBuffer(srcElemNorm->getSource());
            srcNormStride = srcNormBuf->getVertexSize();
        }
        
		// Get buffers for target
        HardwareVertexBufferSharedPtr destPosBuf, destNormBuf;
        destPosBuf = targetVertexData->vertexBufferBinding->getBuffer(destElemPos->getSource());
        destPosStride = destPosBuf->getVertexSize();
        if (includeNormals)
        {
            destNormBuf = targetVertexData->vertexBufferBinding->getBuffer(destElemNorm->getSource());
            destNormStride = destNormBuf->getVertexSize();
        }

        void* pBuffer;

        // Lock source buffers for reading
        pBuffer = srcPosBuf->lock(HardwareBuffer::HBL_READ_ONLY);
        srcElemPos->baseVertexPointerToElement(pBuffer, &pSrcPos);
        if (includeNormals)
        {
            if (srcNormBuf != srcPosBuf) pBuffer = srcNormBuf->lock(HardwareBuffer::HBL_READ_ONLY);
            srcElemNorm->baseVertexPointerToElement(pBuffer, &pSrcNorm);
        }

        // Indices must be 4 bytes
        assert(srcElemBlendIndices->getType() == VET_UBYTE4 &&
               "Blend indices must be VET_UBYTE4");
        pBuffer = srcIdxBuf->lock(HardwareBuffer::HBL_READ_ONLY);
        srcElemBlendIndices->baseVertexPointerToElement(pBuffer, &pBlendIdx);
        if (srcWeightBuf != srcIdxBuf) pBuffer = srcWeightBuf->lock(HardwareBuffer::HBL_READ_ONLY);
        srcElemBlendWeights->baseVertexPointerToElement(pBuffer, &pBlendWeight);
        unsigned short numWeightsPerVertex =
            VertexElement::getTypeCount(srcElemBlendWeights->getType());


        // Lock destination buffers for writing
        pBuffer = destPosBuf->lock(
            (destNormBuf != destPosBuf && destPosBuf->getVertexSize() == destElemPos->getSize()) ||
            (destNormBuf == destPosBuf && destPosBuf->getVertexSize() == destElemPos->getSize() + destElemNorm->getSize()) ?
            HardwareBuffer::HBL_DISCARD : HardwareBuffer::HBL_NORMAL);
        destElemPos->baseVertexPointerToElement(pBuffer, &pDestPos);
        if (includeNormals)
        {
            if (destNormBuf != destPosBuf) pBuffer = destNormBuf->lock(destNormBuf->getVertexSize() == destElemNorm->getSize() ? HardwareBuffer::HBL_DISCARD : HardwareBuffer::HBL_NORMAL);
            destElemNorm->baseVertexPointerToElement(pBuffer, &pDestNorm);
        }

//#define USE_OPTIMISEDUTIL
#ifdef USE_OPTIMISEDUTIL
        OptimisedUtil::getImplementation()->softwareVertexSkinning(
            pSrcPos, pDestPos,
            pSrcNorm, pDestNorm,
            pBlendWeight, pBlendIdx,
            blendMatrices,
            srcPosStride, destPosStride,
            srcNormStride, destNormStride,
            blendWeightStride, blendIdxStride,
            numWeightsPerVertex,
            targetVertexData->vertexCount);
#else
		// for debugging
		/*softwareVertexSkinning(
            pSrcPos, pDestPos,
            pSrcNorm, pDestNorm,
            pBlendWeight, pBlendIdx,
            blendMatrices,
            srcPosStride, destPosStride,
            srcNormStride, destNormStride,
            blendWeightStride, blendIdxStride,
            numWeightsPerVertex,
			targetVertexData->vertexCount);*/

		softwareVertexSkinning2(
            pSrcPos, pDestPos,
            pSrcNorm, pDestNorm,
            pBlendWeight, pBlendIdx,
            _cachedDualQuaternions,
			indexMap,
            srcPosStride, destPosStride,
            srcNormStride, destNormStride,
            blendWeightStride, blendIdxStride,
            numWeightsPerVertex,
			targetVertexData->vertexCount);
#endif

        // Unlock source buffers
        srcPosBuf->unlock();
        srcIdxBuf->unlock();
        if (srcWeightBuf != srcIdxBuf) srcWeightBuf->unlock();
        if (includeNormals && srcNormBuf != srcPosBuf) srcNormBuf->unlock();

        // Unlock destination buffers
        destPosBuf->unlock();
        if (includeNormals && destNormBuf != destPosBuf) destNormBuf->unlock();
#endif
	}
#else

	void SkinEntity::softwareVertexBlend(Affine3 *boneMatrices, Ogre::Mesh::IndexMap& indexMap, VertexData* sourceVertexData, VertexData* targetVertexData, bool blendNormals)
	{
		Msg::error("software vertex blend not implemented yet");
	}
#endif


	void softwareVertexSkinning(
        const float *pSrcPos, float *pDestPos,
        const float *pSrcNorm, float *pDestNorm,
        const float *pBlendWeight, const unsigned char* pBlendIndex,
        const Matrix4* const* blendMatrices,
        size_t srcPosStride, size_t destPosStride,
        size_t srcNormStride, size_t destNormStride,
        size_t blendWeightStride, size_t blendIndexStride,
        size_t numWeightsPerVertex,
        size_t numVertices)
    {
        // Source vectors
        Vector3 sourceVec, sourceNorm;
        // Accumulation vectors
        Vector3 accumVecPos, accumVecNorm;

        // Loop per vertex
        for (size_t vertIdx = 0; vertIdx < numVertices; ++vertIdx)
        {
            // Load source vertex elements
            sourceVec.x = pSrcPos[0];
            sourceVec.y = pSrcPos[1];
            sourceVec.z = pSrcPos[2];

            if (pSrcNorm)
            {
                sourceNorm.x = pSrcNorm[0];
                sourceNorm.y = pSrcNorm[1];
                sourceNorm.z = pSrcNorm[2];
            }

            // Load accumulators
            accumVecPos = Vector3::ZERO;
            accumVecNorm = Vector3::ZERO;

            // Loop per blend weight
            //
            // Note: Don't change "unsigned short" here!!! If use "size_t" instead,
            // VC7.1 unroll this loop to four blend weights pre-iteration, and then
            // loss performance 10% in this function. Ok, this give a hint that we
            // should unroll this loop manually for better performance, will do that
            // later.
            //
            for (unsigned short blendIdx = 0; blendIdx < numWeightsPerVertex; ++blendIdx)
            {
                // Blend by multiplying source by blend matrix and scaling by weight
                // Add to accumulator
                // NB weights must be normalised!!
                Real weight = pBlendWeight[blendIdx];
                if (weight)
                {
                    // Blend position, use 3x4 matrix
                    const Matrix4& mat = *blendMatrices[pBlendIndex[blendIdx]];

					// accumVecPos+= mat*sourceVec
                    accumVecPos.x +=
                        (mat[0][0] * sourceVec.x +
                         mat[0][1] * sourceVec.y +
                         mat[0][2] * sourceVec.z +
                         mat[0][3])
                         * weight;
                    accumVecPos.y +=
                        (mat[1][0] * sourceVec.x +
                         mat[1][1] * sourceVec.y +
                         mat[1][2] * sourceVec.z +
                         mat[1][3])
                         * weight;
                    accumVecPos.z +=
                        (mat[2][0] * sourceVec.x +
                         mat[2][1] * sourceVec.y +
                         mat[2][2] * sourceVec.z +
                         mat[2][3])
                         * weight;
                    if (pSrcNorm)
                    {
                        // Blend normal
                        // We should blend by inverse transpose here, but because we're assuming the 3x3
                        // aspect of the matrix is orthogonal (no non-uniform scaling), the inverse transpose
                        // is equal to the main 3x3 matrix
                        // Note because it's a normal we just extract the rotational part, saves us renormalising here

						// accumVecNorm+= mat.rotation*sourceNorm
                        accumVecNorm.x +=
                            (mat[0][0] * sourceNorm.x +
                             mat[0][1] * sourceNorm.y +
                             mat[0][2] * sourceNorm.z)
                             * weight;
                        accumVecNorm.y +=
                            (mat[1][0] * sourceNorm.x +
                             mat[1][1] * sourceNorm.y +
                             mat[1][2] * sourceNorm.z)
                            * weight;
                        accumVecNorm.z +=
                            (mat[2][0] * sourceNorm.x +
                             mat[2][1] * sourceNorm.y +
                             mat[2][2] * sourceNorm.z)
                            * weight;
                    }
                }
            }

            // Stored blended vertex in hardware buffer
            pDestPos[0] = accumVecPos.x;
            pDestPos[1] = accumVecPos.y;
            pDestPos[2] = accumVecPos.z;

            // Stored blended vertex in temp buffer
            if (pSrcNorm)
            {
                // Normalise
                accumVecNorm.normalise();
                pDestNorm[0] = accumVecNorm.x;
                pDestNorm[1] = accumVecNorm.y;
                pDestNorm[2] = accumVecNorm.z;
                // Advance pointers
                advanceRawPointer(pSrcNorm, srcNormStride);
                advanceRawPointer(pDestNorm, destNormStride);
            }

            // Advance pointers
            advanceRawPointer(pSrcPos, srcPosStride);
            advanceRawPointer(pDestPos, destPosStride);
            advanceRawPointer(pBlendWeight, blendWeightStride);
            advanceRawPointer(pBlendIndex, blendIndexStride);
        }
    }

	void softwareVertexSkinning2(
        const float *pSrcPos, float *pDestPos,
        const float *pSrcNorm, float *pDestNorm,
        const float *pBlendWeight, const unsigned char* pBlendIndex,
		std::vector<dualQuaternion> const& _cachedDualQuaternions,
		Ogre::Mesh::IndexMap const& indexMap,
        size_t srcPosStride, size_t destPosStride,
        size_t srcNormStride, size_t destNormStride,
        size_t blendWeightStride, size_t blendIndexStride,
        size_t numWeightsPerVertex,
        size_t numVertices)
    {
        // Source vectors
        vector3 sourceVec, sourceNorm;
        // Accumulation vectors
        vector3 accumVecPos, accumVecNorm;

		dualQuaternion dqTemp;
		dualQuaternion dqArray[4];
		m_real weights[4];

        // Loop per vertex
        for (size_t vertIdx = 0; vertIdx < numVertices; ++vertIdx)
        {
            // Load source vertex elements
            sourceVec.x = pSrcPos[0];
            sourceVec.y = pSrcPos[1];
            sourceVec.z = pSrcPos[2];

            if (pSrcNorm)
            {
                sourceNorm.x = pSrcNorm[0];
                sourceNorm.y = pSrcNorm[1];
                sourceNorm.z = pSrcNorm[2];
            }

            // Load accumulators
            ASSERT(numWeightsPerVertex ==4);

			for (unsigned short blendIdx = 0; blendIdx < numWeightsPerVertex; ++blendIdx)
			{
				dqArray[blendIdx]=_cachedDualQuaternions[indexMap[pBlendIndex[blendIdx]]];
				weights[blendIdx]=pBlendWeight[blendIdx];
			}

			//dqTemp=dualQuaternion::sDLB(numWeightsPerVertex, weights, dqArray);
			dqTemp=dualQuaternion::sDIB(numWeightsPerVertex, weights, dqArray,0.001);
			accumVecPos=dqTemp.transform(sourceVec);
			accumVecNorm.rotate(dqTemp.mReal, sourceNorm);

			// Stored blended vertex in hardware buffer
            pDestPos[0] = accumVecPos.x;
            pDestPos[1] = accumVecPos.y;
            pDestPos[2] = accumVecPos.z;

            // Stored blended vertex in temp buffer
            if (pSrcNorm)
            {
                // Normalise
                accumVecNorm.normalize();
                pDestNorm[0] = accumVecNorm.x;
                pDestNorm[1] = accumVecNorm.y;
                pDestNorm[2] = accumVecNorm.z;
                // Advance pointers
                advanceRawPointer(pSrcNorm, srcNormStride);
                advanceRawPointer(pDestNorm, destNormStride);
            }

            // Advance pointers
            advanceRawPointer(pSrcPos, srcPosStride);
            advanceRawPointer(pDestPos, destPosStride);
            advanceRawPointer(pBlendWeight, blendWeightStride);
            advanceRawPointer(pBlendIndex, blendIndexStride);
        }
    }

}
#endif
#endif
