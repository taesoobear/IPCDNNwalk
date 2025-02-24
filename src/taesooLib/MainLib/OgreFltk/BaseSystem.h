
#ifndef _BaseSystem_H_
#define _BaseSystem_H_

class GameState 
{
	public:
		virtual ~GameState() {}

		virtual void initialize(void) {}
		virtual void deinitialize(void) {}

		virtual void createScene01(void) {}
		virtual void createScene02(void) {}

		virtual void destroyScene(void) {}

		virtual void update( float timeSinceLast ) {}
		virtual void finishFrame() {}
};

class BaseSystem 
{
	protected:
		GameState   *mCurrentGameState;

	public:
		BaseSystem( GameState *gameState );
		virtual ~BaseSystem();

		virtual void initialize(void);
		virtual void deinitialize(void);

		virtual void createScene01(void);
		virtual void createScene02(void);

		virtual void destroyScene(void);

		void update( float timeSinceLast );
		void finishFrame(void);
};
#endif
