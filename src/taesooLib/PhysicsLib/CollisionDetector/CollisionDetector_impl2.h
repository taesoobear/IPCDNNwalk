
class CdCharCache;
class CdScene;
class CdChar;
class CdJoint;
class CdCheckPair;
class CollisionDetector_impl_hidden
{
	public:
		CollisionDetector_impl_hidden();
		~CollisionDetector_impl_hidden();

		long unsigned int getNumPair();
        CdCheckPair* getCheckPair(int pCount);
		// cache of models
		CdCharCache* cache_;

		// scene
		CdScene* scene_;
		void _addCollisionPair(const char* charName1, const char* charName2, const char* jointName1, const char* jointName2);
		CdChar* getChar(const char* name);
		int _contactIntersection ( CdCheckPair* rPair);
		virtual void clearCache(const char* url);

		virtual void clearAllCache();
};
CdJoint* getJoint(CdChar* rChar, int index);
void setConfig(CdJoint* cdJoint, double _14, double _24, double _34, 
		double _11, double _12, double _13,
		double _21, double _22, double _23,
		double _31, double _32, double _33);

class collision_data;
void collide(CdCheckPair* rPair, int* ret, collision_data** cdata);
CdCharCache* createCdCharCache();
void deleteCdCharCache(void* data);
