/**
 * File: MovableText.h
 *
 * Description: This creates a billboarding object that display a text.
 * Note: This object must have a dedicated scene node since it will rotate it to face the camera (OGRE 2.1)
 * 
 * @author	2003 by cTh see gavocanov@rambler.ru
 * @update	2006 by barraq see nospam@barraquand.com
 * @update	2012 to work with newer versions of OGRE by MindCalamity <mindcalamity@gmail.com>
 * @update	2015 to work on OGRE 2.1 (but not on older versions anymore) by Jayray <jeremy.richert1@gmail.com>
 *	- See "Notes" on: http://www.ogre3d.org/tikiwiki/tiki-editpage.php?page=MovableText
 */

#ifndef __include_MovableText_H__
#define __include_MovableText_H__
#ifndef NO_OGRE
#include <OgrePrerequisites.h>
#include <OgreCommon.h>
#include <OgreMovableObject.h>
#include <OgreFontManager.h>
#include <OgreHlmsDatablock.h>

namespace Ogre {

	class MovableText : public MovableObject, public Renderable, public FrameListener
	{
		// Allow MovableTextFactory full access
		friend class MovableTextFactory;

		/******************************** MovableText data ****************************/
		public:
		enum HorizontalAlignment    {H_LEFT, H_CENTER};
		enum VerticalAlignment      {V_BELOW, V_ABOVE, V_CENTER};

		public:
		String					mFontName;
		String					mName;
		String					mCaption;
		HorizontalAlignment		mHorizontalAlignment;
		VerticalAlignment		mVerticalAlignment;

		ColourValue				mColor;
		v1::RenderOperation		mRenderOp;
		LightList				mLList;

		Real					mCharHeight;
		Real					mSpaceWidth;

		bool					mNeedUpdate;
		bool					mUpdateColors;
		bool					mOnTop;

		Real					mTimeUntilNextToggle;

		Vector3					mGlobalTranslation;
		Vector3					mLocalTranslation;

		Camera					*mpCam;
		Window					*mpWin;
		Font					*mpFont;
		HlmsDatablock			*mpHlmsDatablock;

		/** Private constructor (instances cannot be created directly).
		*/
		MovableText(IdType id, ObjectMemoryManager *objectMemoryManager, SceneManager *sceneManager, const NameValuePairList* params);

		/******************************** public methods ******************************/
		public:
		virtual ~MovableText();

		// Set settings
		void						setFontName(const String &fontName);
		void						setCaption(const String &caption);
		void						setColor(const ColourValue &color);
		void						setCharacterHeight(Real height);
		void						setSpaceWidth(Real width);
		void						setTextAlignment(const HorizontalAlignment& horizontalAlignment, const VerticalAlignment& verticalAlignment);
		void						setGlobalTranslation( Vector3 trans );
		void						setLocalTranslation( Vector3 trans );
		void						showOnTop(bool show=true);

		// Get settings
		const   String				&getFontName()	const {return mFontName;}
		const   String				&getCaption()	const {return mCaption;}
		const   ColourValue			&getColor()		const {return mColor;}

		Real						getCharacterHeight() const {return mCharHeight;}
		Real						getSpaceWidth() const {return mSpaceWidth;}
		Vector3						getGlobalTranslation() const {return mGlobalTranslation;}
		Vector3						getLocalTranslation() const {return mLocalTranslation;}
		bool						getShowOnTop() const {return mOnTop;}

		/******************************** protected methods and overload **************/
		protected:

		// from MovableText, create the object
		void						_setupGeometry();
		void						_updateColors();
		void                        _updateHlmsMacroblock();

		// from MovableObject
		const   String				&getMovableType(void) const {static Ogre::String movType = "MovableText"; return movType;};

		void						_updateRenderQueue(RenderQueue* queue, Camera *camera, const Camera *lodCamera);

		// from Renderable
		void						getWorldTransforms(Matrix4 *xform) const;
		virtual void						getRenderOperation(v1::RenderOperation &op, bool casterPass);
		const   LightList			&getLights(void) const {return mLList;};

		// from FrameListener
		//bool 	                    frameRenderingQueued(const FrameEvent &evt);
		bool frameStarted( const FrameEvent &evt );
	};

	/** Factory object for creating MovableText instances */
	class MovableTextFactory : public MovableObjectFactory
	{
		protected:
			virtual MovableObject* createInstanceImpl( IdType id, ObjectMemoryManager *objectMemoryManager, SceneManager *sceneManager,
					const NameValuePairList* params = 0 );
		public:
			MovableTextFactory() {}
			~MovableTextFactory() {}

			static String FACTORY_TYPE_NAME;

			const String& getType() const;
			void destroyInstance(MovableObject* obj);

	};

} //end namespace Ogre

#endif
#endif
