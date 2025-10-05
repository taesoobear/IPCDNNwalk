#include "stdafx.h"
#ifndef NO_OGRE

#include "dynamicRenderable.h"
#include <OgreCamera.h>
#include <OgreHardwareBufferManager.h>
#include "OgreVertexIndexData.h"
#include <OgreRenderOperation.h>

#include <OgreMeshManager2.h>
#include "Vao/OgreVaoManager.h"
#include "Vao/OgreVertexArrayObject.h"

#include "OgreRoot.h"
#include "OgreHlms.h"
#include "OgreHlmsManager.h"
using namespace Ogre;

DynamicRenderable::DynamicRenderable()
	:MovableObject(RE::generateUniqueID(), RE::_objectMemoryManager(), RE::ogreSceneManager(), 224u),
	Renderable()
{
	vertices=NULL;
	Aabb aabb( Aabb::BOX_INFINITE );
	mObjectData.mLocalAabb->setFromAabb( aabb, mObjectData.mIndex );
	mObjectData.mWorldAabb->setFromAabb( aabb, mObjectData.mIndex );
	mObjectData.mLocalRadius[mObjectData.mIndex] = std::numeric_limits<Real>::max();
	mObjectData.mWorldRadius[mObjectData.mIndex] = std::numeric_limits<Real>::max();
}

DynamicRenderable::~DynamicRenderable()
{
	VaoManager *vaoManager = mManager->getDestinationRenderSystem()->getVaoManager();

	VertexArrayObjectArray::const_iterator itor = mVaoPerLod[0].begin();
	VertexArrayObjectArray::const_iterator end  = mVaoPerLod[0].end();
	while( itor != end )
	{
		VertexArrayObject *vao = *itor;

		const VertexBufferPackedVec &vertexBuffers = vao->getVertexBuffers();
		VertexBufferPackedVec::const_iterator itBuffers = vertexBuffers.begin();
		VertexBufferPackedVec::const_iterator enBuffers = vertexBuffers.end();

		while( itBuffers != enBuffers )
		{
			vaoManager->destroyVertexBuffer( *itBuffers );
			++itBuffers;
		}

		if( vao->getIndexBuffer() )
			vaoManager->destroyIndexBuffer( vao->getIndexBuffer() );
		vaoManager->destroyVertexArrayObject( vao );

		++itor;
	}
}

void DynamicRenderable::initialize(Ogre::OperationType operationType,
                                   bool useIndices)
{
  // Initialize render operation
  mRenderOp.operationType = operationType;
  mRenderOp.useIndexes = useIndices;

  // Reset buffer capacities
  mVertexBufferCapacity = 0;
  mIndexBufferCapacity = 0;

  // Create vertex declaration
  createVertexDeclaration();
}

void DynamicRenderable::prepareHardwareBuffers(size_t vertexCount, 
                                               size_t indexCount)
{
	if (vertexCount != mVertexBufferCapacity) 
	{
		vertices=reinterpret_cast<unsigned char*>( OGRE_MALLOC_SIMD(
					mRenderOp.offset * vertexCount,
					Ogre::MEMCATEGORY_GEOMETRY ) );
		mVertexBufferCapacity = vertexCount;
	}

	if (mRenderOp.useIndexes)
	{
		Msg::error("notimpl");
	}

}
void DynamicRenderable::finalizeHardwareBuffers()
{

	if(!mRenderOp.useIndexes)
	{
		int numIndices=mVertexBufferCapacity;
        Ogre::uint32 *cubeIndices = reinterpret_cast<Ogre::uint32*>( OGRE_MALLOC_SIMD(
                                                                         sizeof(Ogre::uint32) * numIndices,
                                                                         Ogre::MEMCATEGORY_GEOMETRY ) );
		for(int i=0; i<numIndices; i++)
			cubeIndices[i]=(Ogre::uint32)i;

        VaoManager *vaoManager = mManager->getDestinationRenderSystem()->getVaoManager();
        Ogre::IndexBufferPacked *indexBuffer = 0;

        try
        {
            indexBuffer = vaoManager->createIndexBuffer( Ogre::IndexBufferPacked::IT_32BIT,
                                                         numIndices,
                                                         Ogre::BT_IMMUTABLE,
                                                         cubeIndices, true );
        }
        catch( Ogre::Exception &e )
        {
            // When keepAsShadow = true, the memory will be freed when the index buffer is destroyed.
            // However if for some weird reason there is an exception raised, the memory will
            // not be freed, so it is up to us to do so.
            // The reasons for exceptions are very rare. But we're doing this for correctness.
            OGRE_FREE_SIMD( indexBuffer, Ogre::MEMCATEGORY_GEOMETRY );
            indexBuffer = 0;
            throw e;
        }
        Ogre::VertexBufferPacked *vertexBuffer = 0;
        try
        {
            //Create the actual vertex buffer.
            vertexBuffer = vaoManager->createVertexBuffer( mRenderOp.vertexElements, mVertexBufferCapacity,
                                                           BT_IMMUTABLE,
                                                           vertices, true );
        }
        catch( Ogre::Exception &e )
        {
            OGRE_FREE_SIMD( vertexBuffer, Ogre::MEMCATEGORY_GEOMETRY );
            vertexBuffer = 0;
            throw e;
        }

        //Now the Vao. We'll just use one vertex buffer source (multi-source not working yet)
        VertexBufferPackedVec vertexBuffers;
        vertexBuffers.push_back( vertexBuffer );
        Ogre::VertexArrayObject *vao = vaoManager->createVertexArrayObject(
                    vertexBuffers, indexBuffer, mRenderOp.operationType );

        mVaoPerLod[0].push_back( vao );
        //Use the same geometry for shadow casting. You can optimize performance by creating
        //a different Vao that only uses VES_POSITION, VES_BLEND_INDICES & VES_BLEND_WEIGHTS
        //(packed together to fit the caches) and avoids duplicated vertices (usually
        //needed by normals, UVs, etc)
        mVaoPerLod[1].push_back( vao );


        //This is very important!!! A MovableObject must tell what Renderables to render
        //through this array. Since we derive from both MovableObject & Renderable, add
        //ourselves to the array. Otherwise, nothing will be rendered.
        //Tip: You can use this array as a rough way to show or hide Renderables
        //that belong to this MovableObject.
        mRenderables.push_back( this );

        //If we don't set a datablock, we'll crash Ogre.
        this->setDatablock( Root::getSingleton().getHlmsManager()->
                            getHlms( HLMS_PBS )->getDefaultDatablock() );
	}
}


    void DynamicRenderable::getRenderOperation( Ogre::v1::RenderOperation& op , bool casterPass )
    {
        OGRE_EXCEPT( Exception::ERR_NOT_IMPLEMENTED,
                        "MyCustomRenderable do not implement getRenderOperation."
                        " You've put a v2 object in "
                        "the wrong RenderQueue ID (which is set to be compatible with "
                        "v1::Entity). Do not mix v2 and v1 objects",
                        "MyCustomRenderable::getRenderOperation" );
    }
    //-----------------------------------------------------------------------------------
    void DynamicRenderable::getWorldTransforms( Matrix4* xform ) const
    {
        OGRE_EXCEPT( Exception::ERR_NOT_IMPLEMENTED,
                        "MyCustomRenderable do not implement getWorldTransforms."
                        " You've put a v2 object in "
                        "the wrong RenderQueue ID (which is set to be compatible with "
                        "v1::Entity). Do not mix v2 and v1 objects",
                        "MyCustomRenderable::getRenderOperation" );
    }
    //-----------------------------------------------------------------------------------
    bool DynamicRenderable::getCastsShadows(void) const
    {
        OGRE_EXCEPT( Exception::ERR_NOT_IMPLEMENTED,
                        "MyCustomRenderable do not implement getCastsShadows."
                        " You've put a v2 object in "
                        "the wrong RenderQueue ID (which is set to be compatible with "
                        "v1::Entity). Do not mix v2 and v1 objects",
                        "MyCustomRenderable::getRenderOperation" );
    }
    const LightList& DynamicRenderable::getLights(void) const
    {
        return this->queryLights(); //Return the data from our MovableObject base class.
    }
    const String& DynamicRenderable::getMovableType(void) const
    {
        return Ogre::BLANKSTRING;
    }
#endif
