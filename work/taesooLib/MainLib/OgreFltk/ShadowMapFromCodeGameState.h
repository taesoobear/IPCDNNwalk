
#ifndef _Demo_ShadowMapFromCodeGameState_H_
#define _Demo_ShadowMapFromCodeGameState_H_
#ifndef NO_OGRE

#include "OgrePrerequisites.h"
#include "OgreOverlayPrerequisites.h"
#include "OgreOverlay.h"

#include "OgrePrerequisites.h"

#include "BaseSystem.h"
namespace Ogre
{
	namespace v1
	{
		class TextAreaOverlayElement;
	}
}
class  OgreRenderer;
class ShadowMapFromCodeGameState : public GameState
{
	public:
	OgreRenderer *mGraphicsSystem;

	Ogre::String        mHelpDescription;
	Ogre::uint16        mDisplayHelpMode;
	Ogre::uint16        mNumDisplayHelpModes;

	Ogre::v1::TextAreaOverlayElement *mDebugText;
	Ogre::v1::TextAreaOverlayElement *mDebugTextShadow;

	virtual void createDebugTextOverlay(void);
	virtual void generateDebugText( float timeSinceLast, Ogre::String &outText );
	virtual ~ShadowMapFromCodeGameState(){}
	Ogre::SceneNode     *mSceneNode[16];

	Ogre::SceneNode     *mLightNodes[3];

	bool                mAnimateObjects;


	/// Chooses between compute & pixel shader based ESM shadow node.
	/// Compute shader filter is faster for large kernels; but beware
	/// of mobile hardware where compute shaders are slow)
	/// Pixel shader filter is faster for small kernels, also to use as a fallback
	/// on GPUs that don't support compute shaders, or where compute shaders are slow).
	/// For reference large kernels means kernelRadius > 2 (approx)
	const char* chooseEsmShadowNode(void);
	void setupShadowNode( bool forEsm );


	void createShadowMapDebugOverlays(void);
	void destroyShadowMapDebugOverlays(void);

	public:
	ShadowMapFromCodeGameState( const Ogre::String &helpDescription );
	void _notifyGraphicsSystem( OgreRenderer* r){mGraphicsSystem=r;}

	virtual void createScene01(void);

	virtual void update( float timeSinceLast );

};

#endif
#endif
