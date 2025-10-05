#ifndef NO_OGRE
#include "stdafx.h"
#include "ShadowMapFromCodeGameState.h"
#include "renderer.h"

#include "OgreSceneManager.h"
#include "OgreItem.h"

#include "OgreMeshManager.h"
#include "OgreMeshManager2.h"
#include "OgreMesh2.h"
#include "OgreMath.h"
#include "OgreManualObject2.h"

#include "OgreCamera.h"

#include "OgreHlmsUnlitDatablock.h"
#include "OgreHlmsSamplerblock.h"

#include "OgreRoot.h"
#include "OgreHlmsManager.h"
#include "OgreHlms.h"
#include "OgreHlmsPbs.h"
#include "Compositor/OgreCompositorWorkspace.h"
#include "Compositor/OgreCompositorShadowNode.h"

#include "OgreOverlayManager.h"
#include "OgreOverlayContainer.h"
#include "OgreOverlay.h"
#include "OgreTextAreaOverlayElement.h"

#include "OgreFrameStats.h"
#include "Compositor/OgreCompositorManager2.h"
#include "Compositor/Pass/PassScene/OgreCompositorPassSceneDef.h"

#include "OgreHlmsCompute.h"
#include "OgreHlmsComputeJob.h"

#include "OgreHlmsCompute.h"
#include "OgreHlmsComputeJob.h"

#include "OgreGpuProgramManager.h"
#include "OgreLwString.h"

#include "OgreMaterialManager.h"
#include "OgreMaterial.h"
#include "OgreTechnique.h"

#include "TraceManager.h"
namespace Msg
{
	extern OgreTraceManager* otm1;
	extern OgreTraceManager* otm2;
}
//#define CREATE_DEFAULT_SCENE
namespace MiscUtils{
    void setGaussianLogFilterParams( Ogre::HlmsComputeJob *job, Ogre::uint8 kernelRadius,
                                                float gaussianDeviationFactor, Ogre::uint16 K )
    {
        using namespace Ogre;

        assert( !(kernelRadius & 0x01) && "kernelRadius must be even!" );

        if( job->getProperty( "kernel_radius" ) != kernelRadius )
            job->setProperty( "kernel_radius", kernelRadius );
        if( job->getProperty( "K" ) != K )
            job->setProperty( "K", K );
        ShaderParams &shaderParams = job->getShaderParams( "default" );

        std::vector<float> weights( kernelRadius + 1u );

        const float fKernelRadius = kernelRadius;
        const float gaussianDeviation = fKernelRadius * gaussianDeviationFactor;

        //It's 2.0f if using the approximate filter (sampling between two pixels to
        //get the bilinear interpolated result and cut the number of samples in half)
        const float stepSize = 1.0f;

        //Calculate the weights
        float fWeightSum = 0;
        for( uint32 i=0; i<kernelRadius + 1u; ++i )
        {
            const float _X = i - fKernelRadius + ( 1.0f - 1.0f / stepSize );
            float fWeight = 1.0f / std::sqrt ( 2.0f * Math::PI * gaussianDeviation * gaussianDeviation );
            fWeight *= exp( - ( _X * _X ) / ( 2.0f * gaussianDeviation * gaussianDeviation ) );

            fWeightSum += fWeight;
            weights[i] = fWeight;
        }

        fWeightSum = fWeightSum * 2.0f - weights[kernelRadius];

        //Normalize the weights
        for( uint32 i=0; i<kernelRadius + 1u; ++i )
            weights[i] /= fWeightSum;

        //Remove shader constants from previous calls (needed in case we've reduced the radius size)
        ShaderParams::ParamVec::iterator itor = shaderParams.mParams.begin();
        ShaderParams::ParamVec::iterator end  = shaderParams.mParams.end();

        while( itor != end )
        {
            String::size_type pos = itor->name.find( "c_weights[" );

            if( pos != String::npos )
            {
                itor = shaderParams.mParams.erase( itor );
                end  = shaderParams.mParams.end();
            }
            else
            {
                ++itor;
            }
        }

        const bool bIsMetal = job->getCreator()->getShaderProfile() == "metal";

        //Set the shader constants, 16 at a time (since that's the limit of what ManualParam can hold)
        char tmp[32];
        LwString weightsString( LwString::FromEmptyPointer( tmp, sizeof(tmp) ) );
        const uint32 floatsPerParam = sizeof( ShaderParams::ManualParam().dataBytes ) / sizeof(float);
        for( uint32 i=0; i<kernelRadius + 1u; i += floatsPerParam )
        {
            weightsString.clear();
            if( bIsMetal )
                weightsString.a( "c_weights[", i, "]" );
            else
                weightsString.a( "c_weights[", ( i >> 2u ), "]" );

            ShaderParams::Param p;
            p.isAutomatic   = false;
            p.isDirty       = true;
            p.name = weightsString.c_str();
            shaderParams.mParams.push_back( p );
            ShaderParams::Param *param = &shaderParams.mParams.back();

            param->setManualValue( &weights[i], std::min<uint32>( floatsPerParam, weights.size() - i ) );
        }

        shaderParams.setDirty();
    }

    int retrievePreprocessorParameter( const Ogre::String &preprocessDefines,
                                                  const Ogre::String &paramName )
    {
        size_t startPos = preprocessDefines.find( paramName + '=' );
        if( startPos == Ogre::String::npos )
        {
            OGRE_EXCEPT( Ogre::Exception::ERR_INVALID_STATE,
                         "Corrupted material? Expected: " + paramName +
                         " but preprocessor defines are: " + preprocessDefines,
                         "MiscUtils::retrievePreprocessorParameter" );
        }

        startPos += paramName.size() + 1u;

        size_t endPos = preprocessDefines.find_first_of( ";,", startPos );

        Ogre::String valuePart = preprocessDefines.substr( startPos, endPos - startPos );
        const int retVal = Ogre::StringConverter::parseInt( valuePart );
        return retVal;
    }
    void setGaussianLogFilterParams( const Ogre::String &materialName,
                                                Ogre::uint8 kernelRadius,
                                                float gaussianDeviationFactor,
                                                Ogre::uint16 K )
    {
        using namespace Ogre;

        assert( !(kernelRadius & 0x01) && "kernelRadius must be even!" );

        MaterialPtr material;
        GpuProgram *psShader = 0;
        GpuProgramParametersSharedPtr oldParams;
        Pass *pass = 0;

        material = MaterialManager::getSingleton().load(
                    materialName,
                    ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME ).
                staticCast<Material>();

        pass = material->getTechnique(0)->getPass(0);
        //Save old manual & auto params
        oldParams = pass->getFragmentProgramParameters();
        //Retrieve the HLSL/GLSL/Metal shader and rebuild it with new kernel radius
        psShader = pass->getFragmentProgram()->_getBindingDelegate();

        String preprocessDefines = psShader->getParameter( "preprocessor_defines" );
        int oldNumWeights = retrievePreprocessorParameter( preprocessDefines, "NUM_WEIGHTS" );
        int oldK = retrievePreprocessorParameter( preprocessDefines, "K" );
        if( oldNumWeights != (kernelRadius + 1) || oldK != K )
        {
            int horizontalStep  = retrievePreprocessorParameter( preprocessDefines, "HORIZONTAL_STEP" );
            int verticalStep    = retrievePreprocessorParameter( preprocessDefines, "VERTICAL_STEP" );

            char tmp[64];
            LwString preprocessString( LwString::FromEmptyPointer( tmp, sizeof(tmp) ) );

            preprocessString.a( "NUM_WEIGHTS=",     kernelRadius + 1u );
            preprocessString.a( ",K=",              K );
            preprocessString.a( ",HORIZONTAL_STEP=",horizontalStep );
            preprocessString.a( ",VERTICAL_STEP=",  verticalStep );

            psShader->setParameter( "preprocessor_defines", preprocessString.c_str() );
            pass->getFragmentProgram()->reload();
            //Restore manual & auto params to the newly compiled shader
            pass->getFragmentProgramParameters()->copyConstantsFrom( *oldParams );
        }

        std::vector<float> weights( kernelRadius + 1u );

        const float fKernelRadius = kernelRadius;
        const float gaussianDeviation = fKernelRadius * gaussianDeviationFactor;

        //It's 2.0f if using the approximate filter (sampling between two pixels to
        //get the bilinear interpolated result and cut the number of samples in half)
        const float stepSize = 1.0f;

        //Calculate the weights
        float fWeightSum = 0;
        for( uint32 i=0; i<kernelRadius + 1u; ++i )
        {
            const float _X = i - fKernelRadius + ( 1.0f - 1.0f / stepSize );
            float fWeight = 1.0f / std::sqrt ( 2.0f * Math::PI * gaussianDeviation * gaussianDeviation );
            fWeight *= exp( - ( _X * _X ) / ( 2.0f * gaussianDeviation * gaussianDeviation ) );

            fWeightSum += fWeight;
            weights[i] = fWeight;
        }

        fWeightSum = fWeightSum * 2.0f - weights[kernelRadius];

        //Normalize the weights
        for( uint32 i=0; i<kernelRadius + 1u; ++i )
            weights[i] /= fWeightSum;

        GpuProgramParametersSharedPtr psParams = pass->getFragmentProgramParameters();
        psParams->setNamedConstant( "weights", &weights[0], kernelRadius + 1u, 1 );
    }
}

    const Ogre::String c_shadowMapFilters[Ogre::HlmsPbs::NumShadowFilter] =
    {
        "PCF 2x2",
        "PCF 3x3",
        "PCF 4x4",
        "PCF 5x5",
        "PCF 6x6",
        "ESM"
    };

extern ConfigTable config;
    ShadowMapFromCodeGameState::ShadowMapFromCodeGameState( const Ogre::String &helpDescription ) :
        mGraphicsSystem( 0 ),
        mHelpDescription( helpDescription ),
        mDisplayHelpMode( 1 ),
        mNumDisplayHelpModes( 3 ),
        mDebugText( 0 ),
        mAnimateObjects( true )
    {
        memset( mSceneNode, 0, sizeof(mSceneNode) );
    }
    //-----------------------------------------------------------------------------------
    void ShadowMapFromCodeGameState::createScene01(void)
    {
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();

		/*
		 * test failed.
		auto* mSceneMgr=sceneManager;
		Ogre::ManualObject* man = mSceneMgr->createManualObject();
    man->setName("man");
    man->begin("spriteTest1", Ogre::OT_POINT_LIST);
    
    for (size_t i = 0; i < 1000; ++i)
    {
        man->position(Ogre::Math::SymmetricRandom() * 500, 
        Ogre::Math::SymmetricRandom() * 500, 
        Ogre::Math::SymmetricRandom() * 500);
        man->colour(Ogre::Math::RangeRandom(0.5f, 1.0f), 
        Ogre::Math::RangeRandom(0.5f, 1.0f), Ogre::Math::RangeRandom(0.5f, 1.0f));
		man->index(i);
    }
    man->end();
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(man);
	*/

#ifdef CREATE_DEFAULT_SCENE

        Ogre::v1::MeshPtr planeMeshV1 = Ogre::v1::MeshManager::getSingleton().createPlane( "Plane v1",
                                            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                            Ogre::Plane( Ogre::Vector3::UNIT_Y, 1.0f ), 500.0f, 500.0f,
                                            1, 1, true, 1, 4.0f, 4.0f, Ogre::Vector3::UNIT_Z,
                                            Ogre::v1::HardwareBuffer::HBU_STATIC,
                                            Ogre::v1::HardwareBuffer::HBU_STATIC );

        Ogre::MeshPtr planeMesh = Ogre::MeshManager::getSingleton().createByImportingV1(
                    "Plane", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                    planeMeshV1.get(), true, true, true );

        {
            Ogre::Item *item = sceneManager->createItem( planeMesh, Ogre::SCENE_DYNAMIC );
            Ogre::SceneNode *sceneNode = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                                                    createChildSceneNode( Ogre::SCENE_DYNAMIC );
            sceneNode->setPosition( 0, -30, 0 );
            //sceneNode->attachObject( item );
        }

        float armsLength = 25.f*3.f;

        for( int i=0; i<4; ++i )
        {
            for( int j=0; j<4; ++j )
            {
                Ogre::Item *item = sceneManager->createItem( "Cube_d.mesh",
                                                             Ogre::ResourceGroupManager::
                                                             AUTODETECT_RESOURCE_GROUP_NAME,
                                                             Ogre::SCENE_DYNAMIC );

                size_t idx = i * 4 + j;

                mSceneNode[idx] = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                        createChildSceneNode( Ogre::SCENE_DYNAMIC );

                mSceneNode[idx]->setPosition( (i - 1.5f) * armsLength,
                                              20.0f,
                                              (j - 1.5f) * armsLength );
                mSceneNode[idx]->setScale( 6.5f*3.f, 6.5f*3.f, 6.5f*3.f );

                mSceneNode[idx]->roll( Ogre::Radian( (Ogre::Real)idx ) );

                mSceneNode[idx]->attachObject( item );
            }
        }

#else
        mAnimateObjects= false;
#endif

#ifdef CREATE_DEFAULT_LIGHT
		if (config.GetInt("shadowTechnique")==0)
		{
			createDebugTextOverlay();
			return;
		}

        Ogre::SceneNode *rootNode = sceneManager->getRootSceneNode();

        Ogre::Light *light = sceneManager->createLight();
        Ogre::SceneNode *lightNode = rootNode->createChildSceneNode();
        lightNode->attachObject( light );
        light->setPowerScale( 1.0f );
        light->setType( Ogre::Light::LT_DIRECTIONAL );
        light->setDirection( Ogre::Vector3( -1, -1, -1 ).normalisedCopy() );

        mLightNodes[0] = lightNode;

        light = sceneManager->createLight();
        lightNode = rootNode->createChildSceneNode();
        lightNode->attachObject( light );
        light->setDiffuseColour( 0.8f, 0.4f, 0.2f ); //Warm
        light->setSpecularColour( 0.8f, 0.4f, 0.2f );
        light->setPowerScale( Ogre::Math::PI );
        light->setType( Ogre::Light::LT_SPOTLIGHT );
        lightNode->setPosition( -300.0f, 300.0f, 300.0f );
        light->setDirection( Ogre::Vector3( 1, -1, -1 ).normalisedCopy() );
        light->setAttenuationBasedOnRadius( 300.0f, 0.01f );

        mLightNodes[1] = lightNode;

        light = sceneManager->createLight();
        lightNode = rootNode->createChildSceneNode();
        lightNode->attachObject( light );
        light->setDiffuseColour( 0.2f, 0.4f, 0.8f ); //Cold
        light->setSpecularColour( 0.2f, 0.4f, 0.8f );
        light->setPowerScale( Ogre::Math::PI );
        light->setType( Ogre::Light::LT_SPOTLIGHT );
        lightNode->setPosition( 300.0f, 300.0f, -300.0f );
        light->setDirection( Ogre::Vector3( -1, -1, 1 ).normalisedCopy() );
        light->setAttenuationBasedOnRadius( 300.0f, 0.01f );

        mLightNodes[2] = lightNode;
#endif



#if !OGRE_NO_JSON
        //For ESM, setup the filter settings (radius and gaussian deviation).
        //It controls how blurry the shadows will look.
        Ogre::HlmsManager *hlmsManager = Ogre::Root::getSingleton().getHlmsManager();
        Ogre::HlmsCompute *hlmsCompute = hlmsManager->getComputeHlms();

        Ogre::uint8 kernelRadius = 8;
        float gaussianDeviationFactor = 15.f;
        Ogre::uint16 K = 80;
        Ogre::HlmsComputeJob *job = 0;

        //Setup compute shader filter (faster for large kernels; but
        //beware of mobile hardware where compute shaders are slow)
        //For reference large kernels means kernelRadius > 2 (approx)
        job = hlmsCompute->findComputeJob( "ESM/GaussianLogFilterH" );
        MiscUtils::setGaussianLogFilterParams( job, kernelRadius, gaussianDeviationFactor, K );
        job = hlmsCompute->findComputeJob( "ESM/GaussianLogFilterV" );
        MiscUtils::setGaussianLogFilterParams( job, kernelRadius, gaussianDeviationFactor, K );

        //Setup pixel shader filter (faster for small kernels, also to use as a fallback
        //on GPUs that don't support compute shaders, or where compute shaders are slow).
        MiscUtils::setGaussianLogFilterParams( "ESM/GaussianLogFilterH", kernelRadius,
                                               gaussianDeviationFactor, K );
        MiscUtils::setGaussianLogFilterParams( "ESM/GaussianLogFilterV", kernelRadius,
                                               gaussianDeviationFactor, K );
#endif

        createDebugTextOverlay();
    }
    void ShadowMapFromCodeGameState::createDebugTextOverlay(void)
    {
#ifndef _DISABLE_DEBUGTEXT
        Ogre::v1::OverlayManager &overlayManager = Ogre::v1::OverlayManager::getSingleton();
        Ogre::v1::Overlay *overlay = overlayManager.create( "DebugText" );

        Ogre::v1::OverlayContainer *panel = static_cast<Ogre::v1::OverlayContainer*>(
            overlayManager.createOverlayElement("Panel", "DebugPanel"));
        mDebugText = static_cast<Ogre::v1::TextAreaOverlayElement*>(
                    overlayManager.createOverlayElement( "TextArea", "DebugText" ) );
        mDebugText->setFontName( "DebugFont" );
        mDebugText->setCharHeight( 0.025f );

        mDebugTextShadow= static_cast<Ogre::v1::TextAreaOverlayElement*>(
                    overlayManager.createOverlayElement( "TextArea", "0DebugTextShadow" ) );
        mDebugTextShadow->setFontName( "DebugFont" );
        mDebugTextShadow->setCharHeight( 0.025f );
        mDebugTextShadow->setColour( Ogre::ColourValue::Black );
        mDebugTextShadow->setPosition( 0.002f, 0.002f );

        panel->addChild( mDebugTextShadow );
        panel->addChild( mDebugText );
        overlay->add2D( panel );
        overlay->show();
#endif
    }
    //-----------------------------------------------------------------------------------
    const char* ShadowMapFromCodeGameState::chooseEsmShadowNode(void)
    {
        Ogre::Root *root = mGraphicsSystem->getRoot();
        Ogre::RenderSystem *renderSystem = root->getRenderSystem();

        const Ogre::RenderSystemCapabilities *capabilities = renderSystem->getCapabilities();
        bool hasCompute = capabilities->hasCapability( Ogre::RSC_COMPUTE_PROGRAM );

        if( !hasCompute )
        {
            //There's no choice.
            return "ShadowMapFromCodeEsmShadowNodePixelShader";
        }
        else
        {
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS
            //On iOS, the A7 GPUs have slow compute shaders.
            Ogre::DriverVersion driverVersion = capabilities->getDriverVersion();
            if( driverVersion.major == 1 );
                return "ShadowMapFromCodeEsmShadowNodePixelShader";
#endif
            return "ShadowMapFromCodeEsmShadowNodeCompute";
        }
    }
    //-----------------------------------------------------------------------------------
    void ShadowMapFromCodeGameState::setupShadowNode( bool forEsm )
    {
        Ogre::Root *root = mGraphicsSystem->getRoot();
        Ogre::CompositorManager2 *compositorManager = root->getCompositorManager2();

        const Ogre::String nodeDefName = "AutoGen " +
                                         Ogre::IdString("ShadowMapFromCodeWorkspace/Node").
                                         getReleaseText();
        Ogre::CompositorNodeDef *nodeDef =
                compositorManager->getNodeDefinitionNonConst( nodeDefName );

        Ogre::CompositorTargetDef *targetDef = nodeDef->getTargetPass( 0 );
        const Ogre::CompositorPassDefVec &passes = targetDef->getCompositorPasses();

        assert( dynamic_cast<Ogre::CompositorPassSceneDef*>( passes[0] ) );
        Ogre::CompositorPassSceneDef *passSceneDef =
                static_cast<Ogre::CompositorPassSceneDef*>( passes[0] );

        if( forEsm && passSceneDef->mShadowNode == "ShadowMapFromCodeShadowNode" )
        {
            mGraphicsSystem->stopCompositor();
            passSceneDef->mShadowNode = chooseEsmShadowNode();
            mGraphicsSystem->restartCompositor();
        }
        else if( !forEsm && passSceneDef->mShadowNode != "ShadowMapFromCodeShadowNode" )
        {
            mGraphicsSystem->stopCompositor();
            passSceneDef->mShadowNode = "ShadowMapFromCodeShadowNode";
            mGraphicsSystem->restartCompositor();
        }
    }
    //-----------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------
    void ShadowMapFromCodeGameState::update( float timeSinceLast )
    {
        if( mAnimateObjects )
        {
            for( int i=0; i<16; ++i )
                mSceneNode[i]->yaw( Ogre::Radian(timeSinceLast * i * 0.125f) );
        }

        //TutorialGameState::update( timeSinceLast );
        //if( mDisplayHelpMode == 0 )
        {
            //Show FPS
            Ogre::String finalText;
            generateDebugText( timeSinceLast, finalText );
#ifndef _DISABLE_DEBUGTEXT
            mDebugText->setCaption( finalText );
            mDebugTextShadow->setCaption( finalText );
#endif
        }

    }
    //-----------------------------------------------------------------------------------
    void ShadowMapFromCodeGameState::generateDebugText( float timeSinceLast, Ogre::String &outText )
    {
        Ogre::Hlms *hlms = mGraphicsSystem->getRoot()->getHlmsManager()->getHlms( Ogre::HLMS_PBS );

        assert( dynamic_cast<Ogre::HlmsPbs*>( hlms ) );
        Ogre::HlmsPbs *pbs = static_cast<Ogre::HlmsPbs*>( hlms );


        if( mDisplayHelpMode == 1 )
        {
            outText = mHelpDescription;
            outText += "\nPress Ctrl + Alt + O to toggle debug text modes (1/3)";
			/*
            outText += "\n\nProtip: Ctrl+F1 will reload PBS shaders (for real time template editing).\n"
                       "Ctrl+F2 reloads Unlit shaders.\n"
                       "Ctrl+F3 reloads Compute shaders.\n"
                       "Note: If the modified templates produce invalid shader code, "
                       "crashes or exceptions can happen.\n";
					   */
        }


        Ogre::String finalText;
        finalText.reserve( 128 );

		if(mDisplayHelpMode ==0)
		{
			finalText = "";
		}
		else if(mDisplayHelpMode==2)
		{
			finalText += "\nPress Ctrl + Alt + O to toggle debug text modes (2/3). The 3rd mode is invisible. \n\n";
			const Ogre::FrameStats *frameStats = mGraphicsSystem->getRoot()->getFrameStats();
			finalText += "Frame time:\t";
			finalText += Ogre::StringConverter::toString( timeSinceLast * 1000.0f );
			finalText += " ms\n";
			finalText += "Frame FPS:\t";
			finalText += Ogre::StringConverter::toString( 1.0f / timeSinceLast );
			finalText += "\nAvg time:\t";
			finalText += Ogre::StringConverter::toString( frameStats->getAvgTime() );
			finalText += " ms\n";
			finalText += "Avg FPS:\t";
			finalText += Ogre::StringConverter::toString( 1000.0f / frameStats->getAvgTime() );

			finalText += "\n\n\n... RE.output() messages are shown below. (For example, see testTimer.lua) \n\n";

			if (Msg::otm2)
				finalText+=Msg::otm2->tid+"\n";

			if (Msg::otm1)
				finalText+=Msg::otm1->tid;
			outText.swap( finalText );
		}

#ifndef _DISABLE_DEBUGTEXT
        mDebugText->setCaption( finalText );
        mDebugTextShadow->setCaption( finalText );
#endif
    }
    //-----------------------------------------------------------------------------------
	/*
    void ShadowMapFromCodeGameState::keyReleased( const SDL_KeyboardEvent &arg )
    {
		printf("keyreleased\n");
        if( (arg.keysym.mod & ~(KMOD_NUM|KMOD_CAPS)) != 0 )
        {
        if( arg.keysym.scancode == SDL_SCANCODE_F1 && (arg.keysym.mod & ~(KMOD_NUM|KMOD_CAPS)) == 0 )
        {
            mDisplayHelpMode = (mDisplayHelpMode + 1) % mNumDisplayHelpModes;

            Ogre::String finalText;
            generateDebugText( 0, finalText );
            mDebugText->setCaption( finalText );
            mDebugTextShadow->setCaption( finalText );
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_F1 && (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            //Hot reload of PBS shaders. We need to clear the microcode cache
            //to prevent using old compiled versions.
            Ogre::Root *root = mGraphicsSystem->getRoot();
            Ogre::HlmsManager *hlmsManager = root->getHlmsManager();

            Ogre::Hlms *hlms = hlmsManager->getHlms( Ogre::HLMS_PBS );
            Ogre::GpuProgramManager::getSingleton().clearMicrocodeCache();
            hlms->reloadFrom( hlms->getDataFolder() );
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_F2  && (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            //Hot reload of Unlit shaders.
            Ogre::Root *root = mGraphicsSystem->getRoot();
            Ogre::HlmsManager *hlmsManager = root->getHlmsManager();

            Ogre::Hlms *hlms = hlmsManager->getHlms( Ogre::HLMS_UNLIT );
            Ogre::GpuProgramManager::getSingleton().clearMicrocodeCache();
            hlms->reloadFrom( hlms->getDataFolder() );
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_F3 && (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            //Hot reload of Compute shaders.
            Ogre::Root *root = mGraphicsSystem->getRoot();
            Ogre::HlmsManager *hlmsManager = root->getHlmsManager();

            Ogre::Hlms *hlms = hlmsManager->getComputeHlms();
            Ogre::GpuProgramManager::getSingleton().clearMicrocodeCache();
            hlms->reloadFrom( hlms->getDataFolder() );
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_F5 && (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            //Force device reelection
            Ogre::Root *root = mGraphicsSystem->getRoot();
            root->getRenderSystem()->validateDevice( true );
        }
        else
        {
            bool handledEvent = false;


            if( !handledEvent )
                GameState::keyReleased( arg );
        }
            return;
        }

        if( arg.keysym.sym == SDLK_F2 )
        {
            mAnimateObjects = !mAnimateObjects;
        }
        else if( arg.keysym.sym == SDLK_F5 )
        {
            Ogre::Hlms *hlms = mGraphicsSystem->getRoot()->getHlmsManager()->getHlms( Ogre::HLMS_PBS );

            assert( dynamic_cast<Ogre::HlmsPbs*>( hlms ) );
            Ogre::HlmsPbs *pbs = static_cast<Ogre::HlmsPbs*>( hlms );

            Ogre::HlmsPbs::ShadowFilter nextFilter = static_cast<Ogre::HlmsPbs::ShadowFilter>(
                        (pbs->getShadowFilter() + 1u) % Ogre::HlmsPbs::NumShadowFilter );

#if OGRE_NO_JSON
            if( nextFilter == Ogre::HlmsPbs::ExponentialShadowMaps )
            {
                nextFilter = static_cast<Ogre::HlmsPbs::ShadowFilter>(
                                 (nextFilter + 1u) % Ogre::HlmsPbs::NumShadowFilter );
            }
#endif

            pbs->setShadowSettings( nextFilter );

            if( nextFilter == Ogre::HlmsPbs::ExponentialShadowMaps )
                setupShadowNode( true );
            else
                setupShadowNode( false );
        }
        else
        {
        if( arg.keysym.scancode == SDL_SCANCODE_F1 && (arg.keysym.mod & ~(KMOD_NUM|KMOD_CAPS)) == 0 )
        {
            mDisplayHelpMode = (mDisplayHelpMode + 1) % mNumDisplayHelpModes;

            Ogre::String finalText;
            generateDebugText( 0, finalText );
            mDebugText->setCaption( finalText );
            mDebugTextShadow->setCaption( finalText );
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_F1 && (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            //Hot reload of PBS shaders. We need to clear the microcode cache
            //to prevent using old compiled versions.
            Ogre::Root *root = mGraphicsSystem->getRoot();
            Ogre::HlmsManager *hlmsManager = root->getHlmsManager();

            Ogre::Hlms *hlms = hlmsManager->getHlms( Ogre::HLMS_PBS );
            Ogre::GpuProgramManager::getSingleton().clearMicrocodeCache();
            hlms->reloadFrom( hlms->getDataFolder() );
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_F2  && (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            //Hot reload of Unlit shaders.
            Ogre::Root *root = mGraphicsSystem->getRoot();
            Ogre::HlmsManager *hlmsManager = root->getHlmsManager();

            Ogre::Hlms *hlms = hlmsManager->getHlms( Ogre::HLMS_UNLIT );
            Ogre::GpuProgramManager::getSingleton().clearMicrocodeCache();
            hlms->reloadFrom( hlms->getDataFolder() );
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_F3 && (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            //Hot reload of Compute shaders.
            Ogre::Root *root = mGraphicsSystem->getRoot();
            Ogre::HlmsManager *hlmsManager = root->getHlmsManager();

            Ogre::Hlms *hlms = hlmsManager->getComputeHlms();
            Ogre::GpuProgramManager::getSingleton().clearMicrocodeCache();
            hlms->reloadFrom( hlms->getDataFolder() );
        }
        else if( arg.keysym.scancode == SDL_SCANCODE_F5 && (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            //Force device reelection
            Ogre::Root *root = mGraphicsSystem->getRoot();
            root->getRenderSystem()->validateDevice( true );
        }
        else
        {
            bool handledEvent = false;


            if( !handledEvent )
                GameState::keyReleased( arg );
        }
        }
    }
	*/

#endif
