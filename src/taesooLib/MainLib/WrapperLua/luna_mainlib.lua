
--gen_lua.use_profiler=true -- uses performance profiler. See test_profiler sample.
array.pushBack(gen_lua.number_types, 'm_real') -- m_real is a type-def of double.
array.pushBack(gen_lua.enum_types, 'boolN::zeroCrossingMode')
array.pushBack(gen_lua.enum_types, 'uchar')
array.pushBack(gen_lua.enum_types, 'RE::PLDPrimSkinType') 
array.pushBack(gen_lua.enum_types, 'NonuniformSpline::boundaryCondition::bcT')
array.pushBack(gen_lua.enum_types, 'OBJloader::Buffer::Type')
array.pushBack(gen_lua.number_types, 'ushort') 
array.pushBack(gen_lua.number_types, 'uint') 
array.pushBack(gen_lua.number_types, 'Ogre::Real') 

-- snippets for luabind users:
-- luabind -> lunagen

--[[ luabind:
luabind::def("createVRMLskin", &RE::createVRMLskin,luabind::adopt(luabind::result))
-> luna-gen:
PLDPrimVRML* RE::createVRMLskin(VRMLloader*pTgtSkel, bool bDrawSkeleton) @ ;adopt=true; 


luabind:
		.enum_("constants")
		[
			luabind::value("EULER",OpenHRP::DynamicsSimulator::EULER),
			luabind::value("RUNGE_KUTTA",OpenHRP::DynamicsSimulator::RUNGE_KUTTA)
		]
-> luna-gen:

		.scope_[..] -> staticMemberFunctions={...}

		#ifndef AAA
			luabind::def("abc"...)
		#endif
		->
		void abc() @ ;ifndef=AAA;
]]--
bindTargetBaseLib={
	classes={
		{
			name='math.NonuniformSpline',
			className='NonuniformSpline',
			enums={
				{"ZERO_VEL", "(int)NonuniformSpline::boundaryCondition::ZERO_VEL"},
				{"ZERO_ACC", "(int)NonuniformSpline::boundaryCondition::ZERO_ACC"},
			},
			ctors={
				'(vectorn const& keytime, const matrixn& crtnts)',
				'(vectorn const& keytime, const matrixn& crtnts,NonuniformSpline::boundaryCondition::bcT bc)',
			},
			memberFunctions={[[
			void getCurve(vectorn const& time, matrixn& points);
			void getFirstDeriv(vectorn const& time, matrixn& points);
			void getSecondDeriv(vectorn const& time, matrixn& points);
			]]}
		},
		{
			name='math.interval',
			className='interval',
			ctors={'()',
					'(m_real a)',
					'(m_real a,m_real b)'
					
				  },
			memberFunctions={[[
				void setValue(m_real a, m_real b)
				m_real start_pt() const {return start;}
				m_real end_pt() const {return end;}
				m_real len() const {return end-start;}
			]]},
			staticMemberFunctions=
			{[[
				interval operator&(interval const& a,interval const& b) @ intersect
				]]
			}
		},
		{
			name='math.stitchOp',
			className='m::stitchOp',
			memberFunctions={[[
			void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
			]]}
		},
		{
			name='MotionUtil.RetargetOnline2D',
			cppname='MotionUtil::RetargetOnline2D',
			ctors=[[
				(Motion& target, int start)
				(MotionDOF& target, int start)
			]],
			memberFunctions=[[
				void adjust(int time, quater const& oriY, vector3 const& pos2D);
				void adjust(int time, quater const& oriY);
				void adjust(int time, vector3 const& pos2D);
				void adjust(int time, m_real deltarot);
				void adjustSafe(int time, m_real deltarot);
				void adjust(int time, int time2, intvectorn& times);	
			]]
		},
		{
			name='math.linstitch',
			className='m::linstitch',
			inheritsFrom='m::stitchOp',
			ctors={ [[
			()
			(m_real strength)
			]]},
		},
		{
			name='math.linstitchOnline',
			className='m::linstitchOnline',
			inheritsFrom='m::stitchOp',
			ctors={ [[
			()
			(m_real strength)
			]]},
		},
		{
			name='math.linstitchForward',
			className='m::linstitchForward',
			inheritsFrom='m::stitchOp',
			ctors={ [[
			()
			]]},
		},
		{
			name='math.linstitchMulti',
			className='m::linstitchMulti',
			inheritsFrom='m::stitchOp',
			ctors={ [[
			()
			(m_real strength)
			]]},
		},
		{
			name='math.c1stitchPreprocess',
			className='m::c1stitchPreprocess',
			inheritsFrom='m::stitchOp',
			ctors={ [[
			(int arow, int brow, m_real strength, bool bConMid)
			(int arow, int brow)
			]]},
		},
		{
			name='math.c1stitchPreprocessOnline',
			className='m::c1stitchPreprocessOnline',
			inheritsFrom='m::stitchOp',
			ctors={ [[
			(int arow, int brow, m_real strength)
			(int arow, int brow)
			]]},
		},
		{
			name='math.BSpline',
			className='BSpline',
			decl=[[class BSpline;]],
			ctors={[[
			(const matrixn& aCtrlPoints, int iDegree, bool bLoop, bool bOpen)
			(const matrixn& aCtrlPoints, int iDegree )
			]]},
			memberFunctions={[[
			int GetNumCtrlPoints () const
			int GetDegree () const		 
			bool IsOpen () const		 
			bool IsUniform () const		 
			bool IsLoop () const		 
			void SetControlPoint (int i, const vectorn& rkCtrl);
			vectornView GetControlPoint (int i) const	
			m_real Knot (int i);
			void GetPosition (m_real fTime, vectorn& kPos) const			
			void GetFirstDerivative (m_real fTime, vectorn& kDer1) const
			void GetSecondDerivative (m_real fTime, vectorn& kDer2) const
			void GetThirdDerivative (m_real fTime, vectorn& kDer3) const
			]]}
		},
		{
			name='util.PerfTimer2',
			className='QPerformanceTimerCount2',
			ctors={'()'},
			memberFunctions={[[
			void reset()
			void start()
			void pause()
			long stop()
			]]}
		},
		{
			name='util.FractionTimer',
			className='FractionTimer',
			ctors={'()'},
			staticMemberFunctions={[[
			static void FractionTimer::init()
			static double FractionTimer::stopOutside()
			static double FractionTimer::stopInside()
			]]}
		},
		{
			name='boolN',
			ctors={'()','(int n)'},
			wrapperCode=[[
			  static boolNView _range(boolN const& a, int start, int end)
			  {
				  return boolNView (a._vec, a._start+start, a._start+end);
			  }
			  ]],
			enums={
				{"ZC_MIN", "(int)boolN::ZC_MIN"},
				{"ZC_MAX", "(int)boolN::ZC_MAX"},
				{"ZC_ALL", "(int)boolN::ZC_ALL"},
			},
			staticMemberFunctions={[[
										   static boolNView _range(boolN const& a, int start, int end) @ range
			  ]]},
			  enums={
				  {"ZC_MIN", "(int)boolN::ZC_MIN"},
				  {"ZC_MAX", "(int)boolN::ZC_MAX"},
				  {"ZC_ALL", "(int)boolN::ZC_ALL"},
			  },
			memberFunctions={[[
				 void assign(const boolN& other)
				 void set(int i, bool b)
				 void setAllValue(bool b)
				 virtual void resize(int n)
				 int size();
				 TString output() @ __tostring
				 bool operator[](int i) @ __call
				 void findZeroCrossing(const vectorn& signal, boolN::zeroCrossingMode mode);
				 void findLocalOptimum(const vectorn& signal, boolN::zeroCrossingMode mode);
				 void save(const char* filename);	// text file에 쓰기.ex) a.txt = 1 3 6 12
				 void load(int size, const char* filename);	// text file에서 읽기.ex) a.txt = 1 3 6 12
			 	 int findNearest(float i) const;
			 	 int find(int i) const;
			 	 int findPrev(int i) const;
				 void _or(const boolN& a, const boolN& b);
				 void _and(const boolN& a, const boolN& b);
			 ]]}
		},
		{
			name='boolNView',
			inheritsFrom='boolN'
		},
		{
			name='CPixelRGB8',
			ctors={'(uchar, uchar, uchar)'},
			properties={'uchar R', 'uchar G', 'uchar B'},
		},
		{
			name='intvectorn',
			ctors=  -- constructors 
			{
				'()',
				'(int size)',
			},
			wrapperCode=
			[[
			inline static int get(intvectorn const& a, int i)
			{
				return a[i];
			}
			inline static void set(intvectorn & a, int i, int d)
			{
				a[i]=d;
			}
			inline static int count(intvectorn & a, int b)	
			{
				int count=0;
				for(int i=0; i<a.size(); i++)
					if(a[i]==b) count++;
				return count;
			}
			inline static void radd(intvectorn & a, int b)	{a+=b;}
			inline static void rsub(intvectorn & a, int b)	{a-=b;}
			inline static void rdiv(intvectorn & a, int b) {a/=b;}
			inline static void rmult(intvectorn & a, int b) {a*=b;}
			inline static void rmult(intvectorn & a, intvectorn const&b) {for(int i=0; i<a.size(); i++) a[i]*=b[i];}
			inline static void radd(intvectorn & a, intvectorn const& b)	{a+=b;}
			inline static void rsub(intvectorn & a, intvectorn const& b)	{a-=b;}
			inline static intvectorn add(intvectorn const& a, int b) { return a+b;}
			inline static intvectorn sub(intvectorn const& a, int b) { return a-b;}
			static int setValues(lua_State* L)
			{
				intvectorn& self=*luna_t::check(L,1);
				int n=lua_gettop(L)-1;
				self.setSize(n);
				for (int i=0;i<n; i++)
					self(i)=lua_tonumber(L,i+2);
				return 0;
			}
			]],
			staticMemberFunctions=
			{
				{'int get(intvectorn const& a, int i);'},
				{'void set(intvectorn & a, int i, m_real d);'},
				[[void radd(intvectorn & a, int b);
				void rsub(intvectorn & a, int b);
				void rdiv(intvectorn & a, int b);
				void rmult(intvectorn & a, int b);
				void rmult(intvectorn & a, intvectorn const&b);
				void radd(intvectorn & a, intvectorn const& b);
				void rsub(intvectorn & a, intvectorn const& b);
				int count(intvectorn & a, int b)	
				intvectorn add(intvectorn const& a, int b) @ __add
				intvectorn sub(intvectorn const& a, int b) @ __sub
				]]
			},
			memberFunctions=
			{
				[[
				void setAt( intvectorn const& columnIndex, intvectorn const& value);
				void findIndex(intvectorn const& source, int value);
				void findIndex(boolN const& source, bool value);
				void sortedOrder(vectorn const & input);
				void makeSamplingIndex(int nLen, int numSample);
				void makeSamplingIndex2(int nLen, int numSample);
				void bubbleOut(int start, int end)
				TString output() @ __tostring
				int maximum() const;
				int minimum() const;
				int sum() const;
				vectorn toVectorn();
				int findFirstIndex(int value) const;
				void pushBack(int x)
				void pushFront(int x)
				int size()
				void setSize(int)
				void resize(int)
				int at(int i); @ __call
				void setValue(int i, int d) @ set	
				intvectornView range(int start, int end, int step)
				intvectornView range(int start, int end)
				void  colon(int start, int endf, int stepSize);
				void setAllValue(int v);
				void assign(const intvectorn &other);
				]]
			},
			customFunctionsToRegister ={'setValues'},
		},
		{
			name='TRect',
			ctors={"()","(int left,int top,int right,int bottom)"},
			properties={"int left", "int top", "int right", "int bottom"},
		},
		{
			name='CImage',
			ctors={"()"},
			wrapperCode=[[
			inline static void setData(CImage& a,int width,int height,intvectorn dataVec,int stride)
			{
				uchar* data = (uchar*)&dataVec[0];
				a.SetData(width,height,data,stride);
			}
			]],
			memberFunctions={[[
			int GetWidth() const ;
			int GetHeight() const ;
			bool Load(const char* filename);
			bool Save(const char* filename);
			bool save(const char* filename, int BPP);
			bool Create(int width, int height); @ create
			void CopyFrom(CImage const& other);
			CPixelRGB8 * GetPixel(int i, int j) const
			]]},
			staticMemberFunctions={[[
			void Imp::drawBox(CImage& inout, TRect const& t, int R, int G, int B);
			void Imp::sharpen(CImage& inout, double factor, int iterations);
			void Imp::contrast(CImage& inout, double factor);
			void Imp::gammaCorrect(CImage& inout, double factor);
			void Imp::dither(CImage& inout, int levels);
			void Imp::resize(CImage& inout, int width, int height);
			void Imp::blit(CImage& out, CImage const& in, TRect const& rect_in, int x, int y);
			void Imp::concatVertical(CImage& out, CImage const& a, CImage const& b);
			void Imp::crop(CImage& out, CImage const& in, int left, int top, int right, int bottom);
			void Imp::rotateRight(CImage& other);
			void Imp::rotateLeft(CImage& other);
			void setData(CImage& a,int width,int height,intvectorn dataVec,int stride)
			]]}
		},
		{
			name='DrawChart',
			decl='class DrawChart;',
			ctors=[[(const char* xaxis, const char* yaxis, m_real minx, m_real maxx, m_real miny, m_real maxy)]],
			memberFunctions=[[
				void drawMatrix(matrixn const& matrix);
				void drawScatteredData(matrixn const& matrix, CPixelRGB8 color);
				void save(const char* filename);
			]],
		},
		{ 
			name='TStrings',
			ctors={'()'},
			wrapperCode=[[
			inline static void set(TStrings& a, int i, const char* b)
			{
				a[i]=b;
			}
			inline static void set(TStrings& a, int i)
			{
				a[i]="";
			}
			inline static void pushBack(TStrings& a, const char* i)
			{
				TString temp(i);
				a.pushBack(temp);
			}
			]],
			staticMemberFunctions={[[
			void set(TStrings& a, int i, const char* b)
			void set(TStrings& a, int i)
			void pushBack(TStrings& a, const char* i)
			]]},
			memberFunctions={[[
			TString operator[](int i)	@ __call
			TString data(int i)					
			TString back()						
			void init(int n);
			int size() const						
			void resize(int n)					
			TString prefix() const;	// 모든 문자열이 같은 prefix로 시작하는 경우 해당 prefix return.
			void trimSamePrefix(const TStrings& other);	// 모든 문자열이 같은 prefix로 시작하는 경우 잘라준다.
			int find(const char* other) const;
			]]}
		},
			{
				name='intvectornView',
				inheritsFrom='intvectorn'
			},
		{
			name='intmatrixn',
			ctors=  -- constructors 
			{
				'()',
				'(int,int)',
			},
			memberFunctions=[[
				int at(int,int) @ __call
				int getValue(int,int) @ get
				void setValue(int,int,int) @ set
				void resize(int,int)
				void assign(intmatrixn const& o)
				int rows()
				int cols()
				intvectornView		row(int i)const	
				intvectornView		column(int i)const
				void pushBack(const intvectorn& v)

			]]
		},
		{
			name='util.BinaryFile',
			className='BinaryFile',
			decl=[[#include "../../BaseLib/motion/version.h"]],
			ctors={[[
						   ()
						   (bool bReadToMemory)
						   (bool bWrite, const char* filename)
			   ]]},
			wrapperCode=[[
				inline static void _pack(BinaryFile& bf, MotionLoader* pLoader)
				{
					pLoader->pack(bf, MOT_RECENT_VERSION);
				}
				inline static void _unpack(BinaryFile& bf, MotionLoader* pLoader)
				{
					pLoader->unpack(bf);
				}
				inline static void _pack(BinaryFile& bf, Posture& pose)
				{
					bf.pack(MOT_VERSION_STRING[MOT_RECENT_VERSION]);
					bf.packInt(MOT_RECENT_VERSION);	
					pose.pack(bf, MOT_RECENT_VERSION);
				}
				inline static void _unpack(BinaryFile& bf, Posture& pose)
				{
					bf.unpackStr();
					int version=bf.unpackInt();	
					pose.unpack(bf,version);
				}
			   ]],
			staticMemberFunctions=[[
				void _pack(BinaryFile& bf, MotionLoader* pLoader);
				void _unpack(BinaryFile& bf, MotionLoader* pLoader);
				void _pack(BinaryFile& bf, Posture& pose)
				void _unpack(BinaryFile& bf, Posture& pose)
			]],
			memberFunctions={
				[[
					bool openWrite(const char *fileName);
					bool openWrite(const char *fileName, bool);
					bool openRead(const char *fileName);
					void close();
					void packInt(int num);
					void packFloat(double num);
					void pack(const char *str);
					void pack(const vectorn& vec);
					void pack(const vector3& vec);
					void pack(const quater& vec);
					void pack(const intvectorn& vec);
					void pack(const matrixn& mat);
					void pack(const intmatrixn& mat);
					void pack(const vector3N& mat);
					void pack(const quaterN& mat);
					void pack(const TStrings& aSz);
					void pack(const boolN& vec);
					void pack(const matrix4& mat);
					int	unpackInt()	
					double unpackFloat()	
					TString unpackStr();
					void unpack(vectorn& vec);
					void unpack(vector3& vec);
					void unpack(quater& vec);
					void unpack(intvectorn& vec);
					void unpack(matrixn& mat);
					void unpack(intmatrixn& mat);
					void unpack(TStrings& aSz);
					void unpack(boolN& vec);
					void unpack(quaterN& mat);
					void unpack(vector3N& mat);
					void unpack(matrix4& mat);
					int _unpackInt()	
					double _unpackFloat()
					TString _unpackStr();
					void _unpackVec(vectorn& vec);
					void _unpackVec(intvectorn& vec);
					void _unpackSPVec(vectorn& vec);
					void _unpackMat(matrixn& mat);
					void _unpackSPMat(matrixn& mat);
					void _unpackBit(boolN& vec);
					int getFrameNum(int numOfData);
				]]},
				enums={
					{"TYPE_INT", "(int)BinaryFile::TYPE_INT"},
					{"TYPE_FLOAT", "(int)BinaryFile::TYPE_FLOAT"},
					{"TYPE_FLOATN", "(int)BinaryFile::TYPE_FLOATN"},
					{"TYPE_INTN", "(int)BinaryFile::TYPE_INTN"},
					{"TYPE_BITN", "(int)BinaryFile::TYPE_BITN"},
					{"TYPE_FLOATMN", "(int)BinaryFile::TYPE_FLOATMN"},
					{"TYPE_INTMN", "(int)BinaryFile::TYPE_INTMN"},
					{"TYPE_BITMN", "(int)BinaryFile::TYPE_BITMN"},
					{"TYPE_STRING", "(int)BinaryFile::TYPE_STRING"},
					{"TYPE_STRINGN", "(int)BinaryFile::TYPE_STRINGN"},
					{"TYPE_ARRAY", "(int)BinaryFile::TYPE_ARRAY "},
					{"TYPE_EOF", "(int)BinaryFile::TYPE_EOF"},
				}
			
		},
		{
			name='util.MemoryFile',
			className='MemoryFile',
			inheritsFrom='BinaryFile',
			ctors={'()'},
			decl=[[class MemoryFile;]],
		},
		{
			name='transf',
			wrapperCode=[[
			inline 	static void assign(transf &b, transf const& a)
			{
				b.rotation=a.rotation;
				b.translation=a.translation;
			}
			]],
			ctors={
				[[
				()
				(quater const&, vector3 const&)
				(quater const&)
				(vector3 const&)
				(matrix4 const&)
				]]
			},
			properties={ 'quater rotation', 'vector3 translation'},
			memberFunctions={
				[[
				transf inverse() const;
				transf toLocal(transf const& global) const;
				transf toGlobal(transf const& local) const;

				quater toLocalRot(quater const& ori) const;
				quater toGlobalRot(quater const& ori) const;
				quater toLocalDRot(quater const& ori) const;
				quater toGlobalDRot(quater const& ori) const;

				vector3 toLocalPos(vector3 const& pos) const;
				vector3 toGlobalPos(vector3 const& pos) const;
				vector3 toLocalDir(vector3 const& dir) const;
				vector3 toGlobalDir(vector3 const& dir) const;
				void difference(transf const& f1, transf const& f2);			//!< f2*f1.inv
				void identity();
				void operator=(matrix4 const& a);
				vector3 encode2D() const;
				void decode2D(vector3 const& in);
		 		void align2D(transf const& other);
				void leftMult(const transf& a);	//!< this=a*this;
				void operator*=(const transf& a); @ rightMult	
				void operator*=(const transf& a);	//!< this=this*a;
				void slerp(double t, transf const& a, transf const& b);
				void mult(transf const& a, transf const&b);
				void integrateBodyVel(vector3 const& rel_ang_vel, vector3 const& rel_lin_vel, double dt);
				]]
			},
			staticMemberFunctions={
				[[
				static void assign(transf &b, transf const& a)
    transf       operator* ( transf const&, transf const& );
    vector3       operator* ( transf const& , vector3 const&);
				]]
			},
		},
		{
			name='vector3N',
			ctors=
			{
				'()',
				'(int)',
				'(const matrixn&)',
				'(const matrixnView&)',
			},
			memberFunctions=
			{
				[[
						void hermite(const vector3& a, const vector3& b, int duration, const vector3& c, const vector3& d);
						void transition(const vector3& a, const vector3& b, int duration);
						void bubbleOut(int start, int end)
						int rows() const
						int size() const
						void setSize(int)
						void resize(int)
						void reserve(int)
						vector3NView range(int,int)
						vector3NView range(int,int,int)
						void assign(vector3N const&)
						vector3& at(int) @ row
						vector3& at(int) 
						vector3& at(int) @ __call
						void transition(vector3 const&, vector3 const&, int duration) 
						void setAllValue(vector3)
						void pushBack(vector3 const& o)
						vectornView x()
						vectornView y()
						vectornView z()
				]]
				-- ,add,sub,mul
			},
			wrapperCode=[[
					static TString __tostring(vector3N const& a)
						{
							return matView(a).output();
						}
				  static vector3 sampleRow(vector3N const& in, m_real criticalTime)
				  {
				vector3 out;
				//!< 0 <=criticalTime<= numFrames()-1
				// float 0 이 정확하게 integer 0에 mapping된다.
				int a;
				float t;
				
				a=(int)floor(criticalTime);
				t=criticalTime-(float)a;
				
				if(t<0.005)
				  out=in.row(a);
				else if(t>0.995)
				  out=in.row(a+1);
				else
				  {
					if(a<0)
					  out.interpolate(t-1.0, in.row(a+1), in.row(a+2));
					else if(a+1>=in.rows())
					  out.interpolate(t+1.0, in.row(a-1), in.row(a));
					else
					  out.interpolate(t, in.row(a), in.row(a+1));
				  }
				return out;
				  }
				]],
			staticMemberFunctions=
			{
				[[
					  vector3 sampleRow(vector3N const& in, m_real criticalTime)
						matrixnView matView(vector3N const& a, int start, int end)
						matrixnView matView(vector3N const& a)
						vectornView vecView(vector3N const&)
						TString __tostring(vector3N const& a)
				]]
			},
		},
		{
			name='vector3NView', inheritsFrom='vector3N',
		},
		{
			name='quaterN',
			ctors=
			{
				'()',
				'(int)',
			},
			memberFunctions=
			{
				[[
						void align()
						void hermite(const quater& a, const quater& b, int duration, const quater& c, const quater& d);
						void hermite0(const quater& a, const quater& b, int duration, const quater& c, const quater& d);
						void hermite_mid(const quater& a, const quater& b, int duration, const quater& c, const quater& d,quater const& mid);
						void bubbleOut(int start, int end)
						int rows() const
						int size() const
						void setSize(int)
						void resize(int)
						void reserve(int)
						quaterNView range(int,int)
						quaterNView range(int,int,int)
						void assign(quaterN const&)
						quater& at(int) @ row
						quater& at(int) 
						quater& at(int) @ __call
						void transition(quater const&, quater const&, int duration) 
						void setAllValue(quater)
						void pushBack(quater const& o)
						void pushFront(quater const& o)
				]]
				-- sampleRow
			},
			wrapperCode=[[
					static TString __tostring(quaterN const& a)
						{
							return matView(a).output();
						}
      static quater sampleRow(quaterN const& in, m_real criticalTime)
      {
	quater out;
	//!< 0 <=criticalTime<= numFrames()-1
	// float 0 이 정확하게 integer 0에 mapping된다.
	int a;
	float t;
	
	a=(int)floor(criticalTime);
	t=criticalTime-(float)a;
	
	if(t<0.005)
	  out=in.row(a);
	else if(t>0.995)
	  out=in.row(a+1);
	else
	  {
	    if(a<0)
	      out.safeSlerp(in.row(a+1), in.row(a+2), t-1.0);
	    else if(a+1>=in.rows())
	      out.safeSlerp(in.row(a-1), in.row(a), t+1.0);
	    else
	      out.safeSlerp( in.row(a), in.row(a+1),t);
	  }
	return out;
      }
				]],
			staticMemberFunctions=
			{
				[[
				static quater sampleRow(quaterN const& in, m_real criticalTime)
						matrixnView matView(quaterN const& a, int start, int end)
						matrixnView matView(quaterN const& a)
					static TString __tostring(quaterN const& a)
				]]
			},
		},
		{
			name='quaterNView', inheritsFrom='quaterN',
		},
		{
			name='intIntervals',
			decl=[[#include "../../BaseLib/math/intervals.h"]],
			ctors={'()'},
			wrapperCode=[[
				inline static void set(intIntervals& v, int iint, int s, int e)
				{
					v.start(iint)=s;
					v.end(iint)=e;
				}
			]],
			staticMemberFunctions=[[
				void set(intIntervals& v, int iint, int s, int e)
			]],
			memberFunctions={
				[[
					int numInterval() const
					int size() const	
					void setSize(int n)	
					void resize(int n)
					void removeInterval(int i);
					int start(int iInterval) const	 @ startI
					int end(int iInterval) const @ endI
					void load(const char* filename);
					void pushBack(int start, int end)
					int findOverlap(int start, int end, int startInterval);
					int findOverlap(int start, int end);
					void runLengthEncode(const boolN& source)
					void runLengthEncode(const boolN& source , int start, int end);
					void runLengthEncode(const intvectorn& source);
					void findConsecutiveIntervals(const intvectorn& source);
					void runLengthEncodeCut(const boolN& cutState);
					void encodeIntoVector(intvectorn& out);
					void decodeFromVector(const intvectorn& in);
					void offset(int offset);
					void toBitvector(boolN& bitVector);
					void runLengthEncode( const boolN& source)
				]],
			},
		},
		{
			name='intervals',
			ctors={'()'},
			memberFunctions={
				[[
					int size() const	
					void setSize(int n)	
					void resize(int n)
					interval& operator[](int i) const	{ return row(i);}
				]],
			},
		},
		{
			name='vectorn', --necessary
			ctors=  -- constructors 
			{
				'()',
				'(int size)',
			},
			wrapperCode=
			[[
				inline static m_real get(vectorn const& a, int i)
				{
					return a[i];
				}
				inline static void set(vectorn & a, int i, m_real d)
				{
					a[i]=d;
				}
				inline static void assign(vectorn & a, intvectorn const& b)
				{
					a.setSize(b.size());
					for(int i=0; i<a.size(); i++) a[i]=(double)b[i];
				}
				inline static void radd(vectorn & a, m_real b)	{a+=b;}
				inline static void rsub(vectorn & a, m_real b)	{a-=b;}
				inline static void rdiv(vectorn & a, m_real b) {a/=b;}
				inline static void rmult(vectorn & a, m_real b) {a*=b;}
				inline static void rmult(vectorn & a, vectorn const&b) {for(int i=0; i<a.size(); i++) a[i]*=b[i];}
				inline static void setAllValue(vectorn & a, m_real b) {a.setAllValue(b);}
				inline static void radd(vectorn & a, vectorn const& b)	{a+=b;}
				inline static void rsub(vectorn & a, vectorn const& b)	{a-=b;}
				inline static void smoothTransition(vectorn &c, m_real s, m_real e, int size)
				{
					c.setSize(size);
					for(int i=0; i<size; i++)
						{
							m_real sv=sop::map(i, 0, size-1, 0,1);
							c[i]=sop::map(sop::smoothTransition(sv), 0,1, s, e);
						}
					}
				static int setValues(lua_State* L)
				{
					vectorn& self=*luna_t::check(L,1);
					int n=lua_gettop(L)-1;
					self.setSize(n);
					for (int i=0;i<n; i++)
						self(i)=lua_tonumber(L,i+2);
					return 0;
				}
				static int values(lua_State* L)
				{
					vectorn& self=*luna_t::check(L,1);
					int n=self.size();
					lua_createtable(L, n, 0);
					for(int i=0; i<n; i++)
					{
						lua_pushnumber(L, self(i));
						lua_rawseti(L, -2, i+1);
					}

					return 1;
				}
				inline static vectorn mult(m_real b, vectorn const& a)
					{
						return a*b;
					}
					inline static void clamp(vectorn &cc, double a, double b){
						for (int i=0; i<cc.size(); i++)
						{
							double c=cc[i];
							c=(c<a)?a:c;
							c=(c>b)?b:c;
							cc[i]=c;
						}
					}
					inline static void clamp(vectorn &cc, vectorn const& aa, vectorn const&bb){
						assert(aa.size()==bb.size());
						assert(aa.size()==cc.size());
						for (int i=0; i<cc.size(); i++)
						{
							double c=cc[i];
							double a=aa[i];
							double b=bb[i];
							c=(c<a)?a:c;
							c=(c>b)?b:c;
							cc[i]=c;
						}
					}
			]],
			staticMemberFunctions=
			{
				{'m_real get(vectorn const& a, int i);'},
				{'void set(vectorn & a, int i, m_real d);'},
				[[void radd(vectorn & a, m_real b);
				void assign(vectorn & a, intvectorn const& d);
				void rsub(vectorn & a, m_real b);
				void rdiv(vectorn & a, m_real b);
				void rmult(vectorn & a, m_real b);
				void rmult(vectorn & a, vectorn const&b);
				void clamp(vectorn &cc, double a, double b);
				void clamp(vectorn &cc, vectorn const& a, vectorn const& b);
				void setAllValue(vectorn & a, m_real b) ;
				void radd(vectorn & a, vectorn const& b);
				void rsub(vectorn & a, vectorn const& b);
				matrixnView matView(vectorn const& a, int start, int end)
				matrixnView matView(vectorn const& a, int col)
				vector3NView vec3View(vectorn const&)
				quaterNView quatView(vectorn const&)
				static void smoothTransition(vectorn &c, m_real s, m_real e, int size)
				  m_real v::sample(vectorn const& in, m_real criticalTime)
				  void v::interpolate(vectorn & out, m_real t, vectorn const& a, vectorn const& b) @ interpolate
				  void  v::hermite(vectorn& out, double a, double b, int duration, double c, double d) @ hermite
				  void  v::hermite(vectorn& out, double t, double T, const vectorn& a, const vectorn va, const vectorn& b,  const vectorn& vb) @ hermite
				vectorn operator+( vectorn const& a, vectorn const& b);
				vectorn operator-( vectorn const& a, vectorn const& b);
				vectorn operator*( vectorn const& a, vectorn const& b );
				vectorn operator+( vectorn const& a, m_real b);
				vectorn operator-( vectorn const& a, m_real b);
				vectorn operator*( vectorn const& a, m_real b);
				vectorn operator/( vectorn const& a, m_real b);
				vectorn operator*( matrixn const& a, vectorn const& b );
				double  operator%(vectorn const& a, vectorn const& b) @ dotProduct
				vectorn mult(m_real b, vectorn const& a) @ __mul
				]]
			},
			memberFunctions=
				[[
				m_real at(int i) @ __call
				void resample(vectorn const& vec, int numSample);
				void bubbleOut(int start, int end)
				vectorn operator-() const;
				void sub(vectorn const& a, vectorn const& b);
				void add(vectorn const& a, vectorn const& b);
				void extract(vectorn const& source, intvectorn const& index); @ _extract
				void assignSelective(intvectorn const& index, vectorn const& value);
				void assign(const vector3& other);
				void assign(const quater& other);
				void assign(const vectorn &other);
				m_real minimum();
				m_real maximum();
				TString output() @ __tostring
				vector3 toVector3()	const	
				vector3 toVector3(int startIndex)	const	
				quater toQuater() const	
				quater toQuater(int startIndex) const	
				void setVec3( int start, const vector3& src)
				void setQuater( int start, const quater& src)
				inline void pushBack(double x)
				int size()
				void setSize(int)
				void resize(int)
				void setValue(int i, double d) @ set	
				vectornView range(int start, int end, int step)
				vectornView range(int start, int end)
				m_real length() const ;
				m_real minimum() const;
				m_real maximum()	const;
				m_real sum()	const;
				m_real squareSum() const;
				m_real avg() const;
				void minimum(const matrixn& other) 
				void maximum(const matrixn& other) 
				void mean(const matrixn& other) 
				void lengths(matrixn const& in)    
				int argMin() const
				int argMax() const
				int argNearest(double) const;
				void colon(m_real start, m_real stepSize,int nSize);
				void colon(m_real start, m_real stepSize);
				void colon2(double start, double end, double stepSize);
				void linspace(m_real x1, m_real x2, int nSize);
				void linspace(m_real x1, m_real x2);
				void uniform(m_real x1, m_real x2, int nSize);
				void uniform(m_real x1, m_real x2);
				matrixnView column() const;	// return n by 1 matrix, which can be used as L-value (reference matrix)
				matrixnView row() const;	// return 1 by n matrix, which can be used as L-value (reference matrix)
				void minimum(const matrixn& other)
				void maximum(const matrixn& other)
				void lengths(matrixn const& in)   
				void fromMatrix(matrixn const& in)
				]],
			customFunctionsToRegister ={'setValues', 'values'},

		},
		{
			name='vectornView', inheritsFrom='vectorn',
		},
		{
			name='matrix3',
			wrapperCode=[[
			static TString out(matrix3& l)
			{
				TString v;
				v.format(" %f %f %f\n %f %f %f\n %f %f %f\n", l._11,l._12,l._13,l._21,l._22,l._23,l._31,l._32,l._33);
				return v;
			}
			]],
			ctors=
			{
				[[
				()
				(matrix3 const&)
				]]
			},	
			properties={'double _11','double _12','double _13',
			'double _21','double _22','double _23',
			'double _31','double _32','double _33',
					},
			staticMemberFunctions=
			{
				[[
				vector3 operator*(matrix3 const& a, vector3 const& b)
				vector3 operator*(vector3 const& b, matrix3 const& a)
				matrix3 operator*(matrix3 const& a, matrix3 const& b)
				TString out(matrix3 &l) @ __tostring
				]]
			},
			memberFunctions=
			{
				[[
					void setValue( m_real a00, m_real a01, m_real a02, m_real a10, m_real a11, m_real a12, m_real a20, m_real a21, m_real a22 );
					void setValue( vector3 const&row1, vector3 const&row2, vector3 const&row3 );
					void zero();
					void identity();
					void transpose( void );
					void negate( void );
					bool inverse(matrix3 const& a);
					void setTilde( vector3 const &v );
					void setFromQuaternion(quater const& q);
					void mult(matrix3 const& a,matrix3 const& b);
					void mult(matrix3 const& a, m_real b);
					matrix3 operator-(matrix3 const& b)	const 
					matrix3 operator+(matrix3 const& b)	const 
					]]
			},
		},
		{
			name='matrix4', --necessary
			ctors=  -- constructors 
			{
				[[
				()
				(const quater&, const vector3&)
				(const transf&)
				]]
			},
			properties={'double _11','double _12','double _13','double _14',
			'double _21','double _22','double _23','double _24',
			'double _31','double _32','double _33','double _34',
			'double _41','double _42','double _43','double _44',},
			wrapperCode=[[
			inline static void assign(matrix4& l, matrix4 const& m)
			{
				l=m;
			}
			static TString out(matrix4& l)
			{
				TString v;
				v.format(" %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n", l._11,l._12,l._13,l._14,l._21,l._22,l._23,l._24,l._31,l._32,l._33,l._34,l._41,l._42,l._43,l._44);
				return v;
			}
			static matrix4 inverse(matrix4 const& m)
			{
				matrix4 t;
				t.inverse(m);
				return t;
			}
			static vector3 translation(matrix4 const& other)
			{	
				vector3 o;
				o.x=other._14;
				o.y=other._24;
				o.z=other._34;
				return o;
			}
			]],
			staticMemberFunctions={
				[[
				inline static void assign(matrix4& l, matrix4 const& m)
				static TString out(matrix4& l) @__tostring
				static matrix4 inverse(matrix4 const& m)
				vector3 translation(const matrix4& other);

				vector4 operator*(matrix4 const&, vector4 const& a);
				]]
			},
			memberFunctions={
				[[
				m_real determinant() const;
				void identity()	
				void setValue( m_real x00, m_real x01, m_real x02, m_real x03, m_real x10, m_real x11, m_real x12, m_real x13, m_real x20, m_real x21, m_real x22, m_real x23, m_real x30, m_real x31, m_real x32, m_real x33)	;
				void setRotation(const quater& q);
				void setRotation(const matrix3& m);
				void setRotation(const vector3& axis, m_real angle);
				void setTranslation(const vector3& vec, bool bPreserveCurrentRotation);
				void setTransform(const vector3& position,const vector3& scale,const quater& orientation);
				void setScaling(m_real sx, m_real sy, m_real sz);
				void setProjection(m_real fovx, m_real fovy, m_real Z_near, m_real Z_far); 
				void leftMultRotation(const quater& b);
				void leftMultRotation(const vector3& axis, m_real angle);
				void leftMultTranslation(const vector3& vec);
				void leftMultScaling(m_real sx, m_real sy, m_real sz);
				void inverse(const matrix4& a);
				matrix4 operator*(matrix4 const& a) const
				vector3 operator*(vector3 const& a) const;
				vector3 rotate(vector3 const& a) const;
				matrix4 operator+(matrix4 const& a) const 
				matrix4 operator-(matrix4 const& a) const
				matrix4 operator=(matrix4 const& b) const
				void operator*=(double b) 
				]]
			},
		},
		{
			name='matrixn', --necessary
			ctors=  -- constructors 
			{
				'()',
				'(int rows, int cols)',
			},
			customFunctionsToRegister ={'setValues','values'},
			wrapperCode=
			[[
			static int setValues(lua_State* L)
			{
				matrixn& self=*luna_t::check(L,1);
				int n=lua_gettop(L)-1;
				if (n!=self.cols()*self.rows()) luaL_error(L, "setValues(nrows, ncols, values...)");
				int c=2;
				for (int i=0;i<self.rows(); i++)
					for (int j=0;j<self.cols(); j++)
						self(i,j)=lua_tonumber(L,c++);
				return 0;
			}
			static int values(lua_State* L)
			{
				matrixn& self=*luna_t::check(L,1);
				int n=self.rows();
				int m=self.cols();
				lua_createtable(L, n, 0);
				for(int i=0; i<n; i++)
				{
					lua_createtable(L, m, 0);
					for(int j=0; j<m; j++)
					{
						lua_pushnumber(L, self(i,j));
						lua_rawseti(L, -2, j+1);
					}
					lua_rawseti(L, -2, i+1);
				}
				return 1;
			}
			inline static matrixn __mul(matrixn const& a, m_real b)
			{matrixn c;c.mult(a,b);return c;}
			inline static matrixn __mul(m_real b, matrixn const& a)
			{matrixn c;c.mult(a,b);return c;}
			inline static matrixn __mul(matrixn const& a, matrixn const& b)
			{matrixn c;c.mult(a,b);return c;}

			inline static matrixn __add(matrixn const& a, m_real b)
			{matrixn c;c.add(a,b);return c;}
			inline static matrixn __add(m_real b, matrixn const& a)
			{matrixn c;c.add(a,b);return c;}

			inline static matrixn __div(matrixn const& a, m_real b)
			{
				matrixn c;
				c.mult(a,1.0/b);
				return c;
			}

			//static void radd(matrixn & a, m_real b)	{a+=b;}
			//static void rsub(matrixn & a, m_real b)	{a-=b;}
			inline static void rdiv(matrixn & a, m_real b) {a/=b;}
			inline static void rmult(matrixn & a, m_real b) {a*=b;}
			inline static void setAllValue(matrixn & a, m_real b) {a.setAllValue(b);}
			inline static void radd(matrixn & a, matrixn const& b)	{a+=b;}
			inline static void rsub(matrixn & a, matrixn const& b)	{a-=b;}

			inline static void pushBack(matrixn& mat, const vectorn& v) { mat.pushBack(v);}

			inline static void transpose(matrixn& a)
			{
				matrixn c;
				c.transpose(a);
				a=c;
			}
			inline static void sampleRow(matrixn const& in, m_real criticalTime, vectorn& out){
				out.setSize(in.cols());
				//!< 0 <=criticalTime<= numFrames()-1
				// float 0 이 정확하게 integer 0에 mapping된다.
				int a;
				float t;

				a=(int)floor(criticalTime);
				t=criticalTime-(float)a;

				if(t<0.005)
					out=in.row(a);
				else if(t>0.995)
					out=in.row(a+1);
				else {
					if(a<0)
						v::interpolate(out, t-1.0, in.row(a+1), in.row(a+2));
					else if(a+1>=in.rows())
						v::interpolate(out, t+1.0, in.row(a-1), in.row(a));
					else
						v::interpolate(out, t, in.row(a), in.row(a+1));
					}
				}


				]]
			,staticMemberFunctions=
			{
				[[
						matrixn operator-( matrixn const& a, matrixn const& b);
						matrixn operator+( matrixn const& a, matrixn const& b);
				static matrixn __mul(matrixn const& a, m_real b)
				static matrixn __mul(m_real b, matrixn const& a)
				static matrixn __mul(matrixn const& a, matrixn const& b)
				static matrixn __add(matrixn const& a, m_real b)
				static matrixn __add(m_real b, matrixn const& a)
				static matrixn __div(matrixn const& a, m_real b)
				static void rdiv(matrixn & a, m_real b) 
				static void rmult(matrixn & a, m_real b) 
				static void setAllValue(matrixn & a, m_real b) 
				static void radd(matrixn & a, matrixn const& b)	
				static void rsub(matrixn & a, matrixn const& b)	
				static void pushBack(matrixn& mat, const vectorn& v) 
				static void transpose(matrixn& a)
				static void sampleRow(matrixn const& in, m_real criticalTime, vectorn& out)
				vector3NView vec3ViewCol(matrixn const& a, int startCol)
				quaterNView quatViewCol(matrixn const& a, int startCol)
				void m::LUinvert(matrixn& out, const matrixn& in); @ inverse
				void m::pseudoInverse(matrixn& out, const matrixn& in); @ pseudoInverse
				void m::hermite(matrixn& out, const vectorn& a, const vectorn& b, int duration, const vectorn& c, const vectorn& d); @ hermite
				]]
			},
			memberFunctions=
			{
				[[
						void extractRows(matrixn const& mat, intvectorn const& rows);
						void extractColumns(matrixn const& mat, intvectorn const& columns);
						void assignRows(matrixn const& mat, intvectorn const& rows);
						void assignColumns(matrixn const& mat, intvectorn const& columns);
						double at(int,int) @ __call
						double getValue(int,int) @ get
						void setValue(int,int,double) @ set
						int rows()
						int cols()
						vectornView		row(int i)const	
						vectornView		column(int i)const
						vectornView		diag() const		
						void transpose(matrixn const& o)
						void assign(matrixn const& o)
						void setSize(int, int)
						void resize(int, int)
						TString output() @ __tostring
						void setAllValue(double)
						matrixnView range(int, int, int, int)
						double minimum()
						double maximum()
						double sum()
						void pushBack(vectorn const& o)
						void mult( matrixn const&, matrixn const& );
						void multABt(matrixn const& a, matrixn const& b); //!< a*b^T
						void multAtB(matrixn const& a, matrixn const& b); //!< a^T*b
						void multAtBt(matrixn const& a, matrixn const& b); //!< a^T*b^T
						void mult( matrixn const& a, double b );
						void resample(matrixn const& mat, int numSample);
						vectorn toVector() const
						void fromVector(const vectorn& vec, int column);
				]]
			},
		},
		{
			name='matrixnView', inheritsFrom='matrixn',
			ctors={
				[[
						(const matrixn& )
						(const matrixnView&)
				]]
			},
		},
		{
			name='hypermatrixn',
			ctors=[[
				(void)
				(int pages, int nrows, int columns)
				(const hypermatrixn& other)
			]],
			memberFunctions=[[
				int	page() const	
				int	page() const	@ pages
				int rows() const	
				int cols() const	
				void setSize( int, int, int);  //!< 원래 데이타 유지 보장 전혀 없음.
				void setSameSize(hypermatrixn const& other)	
				matrixnView& page(int index) const		
				void operator=(hypermatrixn const& other)
			]]
		},
		{
			decl='class vector2;',
			name='vector2', --necessary
			ctors=  -- constructors 
			{
				'()',
				'(double x, double y)',
				'(vector2 const& y)',
			},
			read_properties=
			{
				-- property name, lua function name
				{'x', 'getX'},
				{'y', 'getY'},
			},
			write_properties=
			{
				{'x', 'setX'},
				{'y', 'setY'},
			},
			wrapperCode=
			[[
			inline static double getX(vector2 const& a) { return a.x();}
			inline static double getY(vector2 const& a) { return a.y();}
			inline static void setX(vector2 & a, m_real b) { a.x()=b;}
			inline static void setY(vector2 & a, m_real b) { a.y()=b;}
			inline static void set(vector2& vv, int i, double v) { vv(i)=v; }
			inline static void assignXZ(vector2& vv, vector3 const& v) { vv(0)=v.x;vv(1)=v.z; }
			]],
			staticMemberFunctions=
			[[
			double getX(vector2 const& a);
			double getY(vector2 const& a);
			void setX(vector2 & a, m_real b);
			void setY(vector2 & a, m_real b);
			void set(vector2& vv, int i, double v)
			void assignXZ(vector2& vv, vector3 const& v) 
			]],
			memberFunctions =
			[[
			double value(int) @ __call
			vector3 toVector3() const
			void operator*=(double); @ scale
			vector2 operator+(vector2 const& b) const; @ __add
			vector2 operator-(vector2 const& b) const; @ __sub
			vector2 operator*(double b) const; @ __mul
			void operator=(vector2 const&) @ assign
			void rotate90(vector2 v)
			void difference(vector3 const& a, vector3 const& b)
			]]
		},
		{
			name='vector3', --necessary
			ctors=  -- constructors 
			{
				'()',
				'(double xyz)',
				'(double x, double y, double z)',
			},
			read_properties=
			{
				-- property name, lua function name
				{'x', 'getX'},
				{'y', 'getY'},
				{'z', 'getZ'},
			},
			write_properties=
			{
				{'x', 'setX'},
				{'y', 'setY'},
				{'z', 'setZ'},
			},
			wrapperCode=
			[[
			inline static double getX(vector3 const& a) { return a.x;}
			inline static double getY(vector3 const& a) { return a.y;}
			inline static double getZ(vector3 const& a) { return a.z;}
			inline static void setX(vector3 & a, m_real b) { a.x=b;}
			inline static void setY(vector3 & a, m_real b) { a.y=b;}
			inline static void setZ(vector3 & a, m_real b) { a.z=b;}
			inline static void set(vector3 & a, m_real x, m_real y, m_real z) {a.x=x; a.y=y; a.z=z;}
			inline static vector3 __mul(vector3 const& a, m_real b)
			{vector3 c;c.mult(a,b);return c;}
			inline static vector3 __mul(m_real b, vector3 const& a)
			{vector3 c;c.mult(a,b);return c;}
			inline static vector3 __div(vector3 const& a, m_real b)
			{
				vector3 c;
				c.mult(a,1.0/b);
				return c;
			}
			inline static vector3 __add(vector3 const& a, vector3 const& b)
			{vector3 c;c.add(a,b);return c;}
			inline static vector3 __sub(vector3 const& a, vector3 const& b)
			{vector3 c;c.sub(a,b);return c;}
			inline static double dotProduct(vector3 const& a, vector3 const& b)
			{
				return a%b;
			}
			// you can implement custom interface function too. (see customFunctionsToRegister below)
			static int __tostring(lua_State* L)
			{
				vector3& self=*luna_t::check(L,1);
				lua_pushstring(L, self.output().ptr());
				return 1;
			}
			static vector3 __unm(vector3 const& a,vector3 const& a2)
			{ return a*-1;}
			]],
			staticMemberFunctions=
			{
				[[
				vector3 __mul(vector3 const& a, double b);
				vector3 __mul(double b, vector3 const& a);
				vector3 __div(vector3 const& a, double b);
				vector3 __add(vector3 const& a, vector3 const& b);
				vector3 __sub(vector3 const& a, vector3 const& b);
				double dotProduct(vector3 const& a, vector3 const& b);
				double getX(vector3 const& a);
				double getY(vector3 const& a);
				double getZ(vector3 const& a);
				void setX(vector3 & a, m_real b);
				void setY(vector3 & a, m_real b);
				void setZ(vector3 & a, m_real b);
				void set(vector3 & a, m_real ,m_real, m_real);
				void set(vector3 & a, m_real ,m_real, m_real); @ setValue
				vector3 __unm(vector3 const& a,vector3 const & a2)
				]]
			},
			customFunctionsToRegister={'__tostring'},
			memberFunctions = -- list of strings of c++ function declarations.
			{
				-- you can enter multiline texts that looks like a cleaned header file
				[[
				void ln( const quater& q);
				void add(const vector3&, const vector3&);
				void sub(const vector3&, const vector3&);
				void zero();
				void operator*=(double);
				void operator*=(double); @ scale
				void leftMult(const matrix4& mat);
				void cross(const vector3&, const vector3&);
				vector3 cross(const vector3&) const;
				m_real distance(const vector3& other) const;
				void normalize();
				void multadd(const vector3&, m_real);	
				m_real length() const;
				void ln( const quater& q);
				quater exp() const;
				void interpolate( m_real, vector3 const&, vector3 const& );
				void difference(vector3 const& v1, vector3 const& v2);
				void rotate( const quater& q);
				void rotate( const quater& q, vector3 const& in);
				void rotationVector(const quater& in);
				void angularVelocity( quater const& q1, quater const& q2);
				void linearVelocity(vector3 const& v1, vector3 const& v2);
				void translation(const matrix4& other);		//!< extract translation vector from a matrix
				quater quaternion() const;	//!< rotation vector를 quaternion으로 바꾼다.	== quater q; q.setRotation(*this); }
				]],
				{'void add(const vector3&);', rename='radd'},
				{'void sub(const vector3&);', rename='rsub'},
				{'void operator=(const vector3&);', rename='assign'},
			},
		},
		{
			name='vector4', --necessary
			ctors=  -- constructors 
			{
				'()',
				'(double x, double y, double z, double w)',
			},
			read_properties=
			{
				-- property name, lua function name
				{'x', 'getX'},
				{'y', 'getY'},
				{'z', 'getZ'},
				{'w', 'getW'},
			},
			write_properties=
			{
				{'x', 'setX'},
				{'y', 'setY'},
				{'z', 'setZ'},
				{'w', 'setW'},
			},
			wrapperCode=
			[[
			inline static double getX(vector4 const& a) { return a.x();}
			inline static double getY(vector4 const& a) { return a.y();}
			inline static double getZ(vector4 const& a) { return a.z();}
			inline static double getW(vector4 const& a) { return a.w();}
			inline static void setX(vector4 & a, m_real b) { a.x()=b;}
			inline static void setY(vector4 & a, m_real b) { a.y()=b;}
			inline static void setZ(vector4 & a, m_real b) { a.z()=b;}
			inline static void setW(vector4 & a, m_real b) { a.w()=b;}
			inline static void set(vector4 & a, m_real x, m_real y, m_real z, m_real w) {a.x()=x; a.y()=y; a.z()=z; a.w()=w;}
			]],
			staticMemberFunctions=
			{
				[[
				vector4    operator*( matrix4 const& m, vector4 const& v) 
				double getX(vector4 const& a);
				double getY(vector4 const& a);
				double getZ(vector4 const& a);
				double getW(vector4 const& a);
				void setX(vector4 & a, m_real b);
				void setY(vector4 & a, m_real b);
				void setZ(vector4 & a, m_real b);
				void setW(vector4 & a, m_real b);
				]]
			},
			memberFunctions = -- list of strings of c++ function declarations.
			{
				-- you can enter multiline texts that looks like a cleaned header file
				[[
				vector3 toVector3() const
				]]
			},
		},
				{
					name='quater', --necessary
					properties={'double x', 'double y', 'double z', 'double w'},
					ctors=
					{
						[[
								()
								(double,double,double,double)
								(double, vector3 const&)
						]]
					},
					wrapperCode=
					[[
								static vector3 rotate(quater const& q, vector3 const& v)
								{
									vector3 out;
									out.rotate(q,v);
									return out;
								}
								static void setRotation(quater &q, const char* aChannel, vector3 & euler)
								{
									q.setRotation(aChannel, euler);
								}
								static void getRotation(quater const&q, const char* aChannel, vector3 & euler)
								{
									q.getRotation(aChannel, euler);
								}
								static vector3 getRotation(quater const&q, const char* aChannel)
								{
									vector3 euler;
									q.getRotation(aChannel, euler);
									return euler;
								}
								static m_real toAxisAngle(quater const& q, vector3& axis)
								{
									m_real angle;
									q.toAxisAngle(axis, angle);
									return angle;
								}
								inline static double dotProduct(quater const& a, quater const& b)
								{
									return a%b;
								}
								]],
					staticMemberFunctions={
						[[
								static vector3 rotate(quater const& q, vector3 const& v)
								static void setRotation(quater &q, const char* aChannel, vector3 & euler)
								static void getRotation(quater const&q, const char* aChannel, vector3 & euler)
								static vector3 getRotation(quater const&q, const char* aChannel)
								static m_real toAxisAngle(quater const& q, vector3& axis)
								static double dotProduct(quater const& a, quater const& b)
						]]
					},
					memberFunctions=
					[[
						void slerp( quater const&, quater const&, m_real );
						void safeSlerp( quater const& a, quater const& b, m_real weight);// no alignment necessary.
						void interpolate( m_real, quater const&, quater const& );
						void setAxisRotation(const vector3& vecAxis, const vector3& front, const vector3& vecTarget);
						void identity()
						quater   inverse() const
						void decompose(quater& rotAxis_y, quater& offset) const;
						void decomposeTwistTimesNoTwist (const vector3& rkAxis, quater& rkTwist, quater& rkNoTwist) const;
						void decomposeNoTwistTimesTwist (const vector3& rkAxis, quater& rkNoTwist, quater& rkTwist) const;
						void difference(quater const& q1, quater const& q2);			//!< quaternion representation of "angular velocity w" : q2*inv(q1);
						void toLocal(quater const& q1, quater const& q2);
						void scale(m_real s);
						void mult(quater const& a, quater const& b);// cross product
						quater    operator+( quater const& b) const		
						quater    operator-( quater const& b) const		
						quater    operator-() const						
						quater    operator*( quater const& b) const		
						quater    operator*( m_real b) const			
						vector3	  operator*(vector3 const& b) const		
						quater    operator/( m_real b) const			
						m_real  length() const;
						m_real rotationAngle() const	
						m_real rotationAngleAboutAxis(const vector3& axis) const;
						vector3 rotationVector() const
						void operator=(quater const& a)
						void operator=(quater const& a) @set
						void setValue( m_real ww,m_real xx, m_real yy, m_real zz )
						void axisToAxis( const vector3& vFrom, const vector3& vTo);
						void leftMult(quater const& a)			
						void rightMult(quater const& a)			
						void setRotation(const vector3&, m_real)
						void setRotation(const vector3& )
						void setRotation(const matrix4& a)
						void normalize();
						void align(const quater& other)
						TString output(); @ __tostring
						void blend(const vectorn& weight, matrixn& aInputQuater);
					]]
					,
				},
				{
					name='MotionDOFinfo',
					decl='class MotionDOFinfo;',
					ctors={"()"},
					enums={ 
						{'ROTATE','(int)MotionDOFinfo::ROTATE'}, 
						{'SLIDE', '(int)MotionDOFinfo::SLIDE'},
						{'QUATERNION_W','(int)MotionDOFinfo::QUATERNION_W'}, 
						{'QUATERNION_X','(int)MotionDOFinfo::QUATERNION_X'}, 
						{'QUATERNION_Y','(int)MotionDOFinfo::QUATERNION_Y'}, 
						{'QUATERNION_Z','(int)MotionDOFinfo::QUATERNION_Z'},
					}
					,memberFunctions={
						[[
						MotionLoader & skeleton() const	{return *_sharedinfo->mSkeleton;}
						int numDOF() const; // including quaternion's additional variables.
						int numActualDOF() const; // numDOF()- numSphericalJoint()
						int numBone() const;
						int numDOF(int ibone) const;
						int DOFtype(int ibone, int offset) const;
						int DOFindex(int ibone, int offset) const;
						int sphericalDOFindex(int isphericalJoint) const;
						int numSphericalJoint() const;
						double frameRate() const;
						void setFrameRate(double f);
						void getDOF(Posture const& p, vectorn& dof) const;
						void setDOF(vectorn const& dof, Posture& p) const;
						Posture const& setDOF(vectorn const& dof) const;	//!< non thread-safe
						bool hasTranslation(int iBone) const;
						bool hasQuaternion(int iBone) const;
						bool hasAngles(int iBone) const;
						int startT(int iBone) const;
						int startR(int iBone) const;
						int endR(int iBone) const;
						int startDQ(int iBone) const; 
						int endDQ(int iBone) const;
						int DQtoBone(int DQindex)
						int DOFtoBone(int DOFindex)
						int DOFtoDQ(int DOFindex) 
						int DQtoDOF(int DOFindex)
						void DOFtoDQ(vectorn const& dtheta, vectorn & dq);
						void DQtoDOF(vectorn const& dq, vectorn & dtheta);
						void blend(vectorn & out, vectorn const& p1, vectorn const& p2, m_real t) const;
						void blendDelta(vectorn & out, vectorn const& p1, vectorn const& p2, m_real t) const;
						void blendBone(int ibone, vectorn & c, vectorn const& a, vectorn const& b, m_real t) const;
						]]
					}
				},
				{
					name='CMotionDOFcontainer',
					className='MotionDOFcontainer',
					decl=[[class MotionDOFcontainer;]],
					ctors={[[
						(MotionDOFinfo const& info, const char* filename);
						(MotionDOFinfo const& info);
						(MotionDOF const& mot);
					]]},
					properties={
						'MotionDOF mot',
						'boolN discontinuity',
						'boolN conL',
						'boolN conR',
					},
					memberFunctions={[[
						void loadMotion(const char* fn);
						void resize(int nframes);
						void concat(MotionDOF const& mot);
						vectornView row(int i)
						int numFrames() const;
						bool isConstraint(int iframe, int con) const;
						void setConstraint(int iframe, int con);
						void setConstraint(int iframe, int con, bool bSet);
						bool isContinuous(int startTime) const;
						bool isValid(int startTime, int endTime) const;
						bool isValid(int startTime) const;
					]]},
				},
				{
					name='MotionUtil.Effector',
					ctors={'()'},
					properties={'Bone* bone', 'vector3 localpos'},
					wrapperCode=[[
					static void init(MotionUtil::Effector& e, Bone* bone, vector3 const& l)
					{
						e.bone=bone;
						e.localpos=l;
					}
					static void initCOM(MotionUtil::Effector& e, vector3 const& l)
					{
						e.bone=NULL;
						e.localpos=l;
					}
					]],
					staticMemberFunctions={[[
					static void init(MotionUtil::Effector& e, Bone* bone, vector3 const& l)
					static void initCOM(MotionUtil::Effector& e, vector3 const& l)
					]]},
				},
				{
					name='MotionUtil.RelativeConstraint',
					ctors={'()'},
					properties={'Bone* bone1','Bone* bone2', 'vector3 localpos1'},
					wrapperCode=[[
					static void init(MotionUtil::RelativeConstraint& e, Bone* bone1, Bone* bone2, vector3 const& l)
					{
						e.bone1=bone1;
						e.bone2=bone2;
						e.localpos1=l;
					}
					]],
					staticMemberFunctions={[[
					static void init(MotionUtil::RelativeConstraint& e, Bone* bone1, Bone* bone2, vector3 const& l)
					]]},
				},
				{
					name='MotionUtil.Effectors',
					className='std::vector<MotionUtil::Effector>',
					ctors={'()'},
					memberFunctions={[[

											 void resize(int)
											 MotionUtil::Effector& operator[](int i) @ at
											 MotionUtil::Effector& operator[](int i) @ __call
											 int size()
								 ]]}
				},
				{
					name='MotionUtil.Constraints',
					className='std::vector<MotionUtil::RelativeConstraint>',
					ctors={'()'},
					memberFunctions={[[

											 void resize(int)
											 MotionUtil::RelativeConstraint& operator[](int i) @ at
											 MotionUtil::RelativeConstraint& operator[](int i) @ __call
											 int size()
								 ]]}
				},
				{
					name='MotionUtil.Node',
					cppname='IK_sdls::Node',
					decl=[[
					namespace IK_sdls {
						class Node;
					}]],
					memberFunctions=[[
					const transf& globalFrame() const 
					void SetTheta(double newTheta);
					double GetTheta();
					bool IsEffector() const 
					bool IsJoint() const 
					bool IsHingeJoint() const 
					bool IsSlideJoint() const
					bool IsFreeJoint() const 
					int GetEffectorNum() const 
					int GetJointNum() const 
					vector3 const& bodyLinVel() 
					vector3 const& bodyAngVel() 
					void GetJointVel(vector3 & lin_vel, vector3 & ang_vel);
					]],
					properties={ 'transf _global' },
				},
				{
					decl=[[
					namespace Liegroup {
						class Inertia;
						class se3;
						class dse3;
					}]],
					name='Liegroup.Inertia',
					ctors={
						'()',
						'(double m)',
						'(double mass, double Ixx, double Iyy, double Izz)',
						'(const Liegroup::Inertia &J)',
					},
					memberFunctions={[[
					void zero(void)
					void setMass(double mass)
					double mass(void) 
					vector3 getOffDiag()
					vector3 getDiag()
					vector3 getSymm()
					void setInertia(double Ixx, double Iyy, double Izz, double Ixy, double Ixz, double Iyz) 
					void setOffDiag(const vector3& r)
					Liegroup::Inertia transform(const transf &T) const;
					]]},
				},
				{
					name='Liegroup.se3',
					ctors={
						'()',
						'(double)',
						'(double w0, double w1, double w2, double v0, double v1, double v2)',
						'(const Liegroup::se3 &)',
						'(const vector3& w, const vector3& v)',
					},
					read_properties=
					{
						{'w','W'},
						{'v', 'V'},
					},
					memberFunctions={[[
					vector3 & W() 
					vector3 & V()
					const Liegroup::se3 &operator + (void) const 
					Liegroup::se3 operator - (void) const 
					Liegroup::se3 &operator = (const Liegroup::se3 &t) 
					Liegroup::se3 &operator = (double d) 
					Liegroup::se3 &operator += (const Liegroup::se3 &t) 
					Liegroup::se3 &operator -= (const Liegroup::se3 &t) 
					Liegroup::se3 &operator *= (double d) 
					Liegroup::se3 &operator /= (double d) 
					Liegroup::se3 operator + (const Liegroup::se3& t) const 	
					Liegroup::se3 operator - (const Liegroup::se3& t) const 	
					Liegroup::se3 operator * (double d) const 
					double &operator [] (int i)  @ __call
					void zero(void) 
					double innerProduct(const Liegroup::se3& s) 
					double squaredLen() const 
					transf exp() const;
					void log(transf const& o);
					vectornView vec() const ;
					]]},
					staticMemberFunctions={[[
					Liegroup::se3 operator * (double d, const Liegroup::se3 &t) 
					]]}
				},
				{
					name='Liegroup.dse3',
					ctors={
						'()',
						'(double)',
						'(double m0, double m1, double m2, double f0, double f1, double f2)',
						'(const Liegroup::dse3 &)',
						'(const vector3& m, const vector3& f)',
					},
					memberFunctions={[[
					vector3 & M() 
					vector3 & F()
					const Liegroup::dse3 &operator + (void) const 
					Liegroup::dse3 operator - (void) const 
					Liegroup::dse3 &operator = (const Liegroup::dse3 &t) 
					Liegroup::dse3 &operator = (double d) 
					Liegroup::dse3 &operator += (const Liegroup::dse3 &t) 
					Liegroup::dse3 &operator -= (const Liegroup::dse3 &t) 
					Liegroup::dse3 &operator *= (double d) 
					Liegroup::dse3 &operator /= (double d) 
					Liegroup::dse3 operator + (const Liegroup::dse3& t) const 	
					Liegroup::dse3 operator - (const Liegroup::dse3& t) const 	
					Liegroup::dse3 operator * (double d) const 
					double &operator [] (int i)  @ __call
					void zero(void) 
					double innerProduct(const Liegroup::dse3& s) 
					double squaredLen() const 
					]]},
					staticMemberFunctions={[[
					Liegroup::dse3 operator * (double d, const Liegroup::dse3 &t) 
					vector3 const& getM(const Liegroup::dse3 &t) 
					vector3 const& getF(const Liegroup::dse3 &t) 
					]]}
				},
				{
					name='MotionUtil.LoaderToTree',
					cppname='IK_sdls::LoaderToTree',
					decl=[[
					namespace IK_sdls {
						class LoaderToTree;
					}]],
					ctors=[[
					(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors, std::vector<MotionUtil::RelativeConstraint>& con, bool useEulerRoot, bool useFixedRootPos)
					(MotionLoader& skeleton, bool useEulerRoot, bool useFixedRootPos)
					()
					]],
					wrapperCode=[[
					static void Print(IK_sdls::LoaderToTree& self)
					{
						self.mTree.Print();
					}
					]],
					memberFunctions=[[
					void computeTree()
					int nDOF() 
					int nDOFinTree() 
					double calcInertia(const VRMLloader& loader, vectorn& inertia) const; 
					intvectorn const& getDQindex();
					IK_sdls::Node* getJoint(int jointindex)
					IK_sdls::Node* getNode(int treeIndex, int dofIndex) 
					IK_sdls::Node* getLastNode(int treeIndex) 
					int getVarIndex(int treeIndex, int dofIndex) 
					int getVarIndexByAxis(int treeIndex, const char *axis);
					const transf & globalFrame(int ibone) const
					void setPoseDOF(MotionDOFinfo const& mDofInfo, vectorn const& pose)
					void getPoseDOF(MotionDOFinfo const& mDofInfo, vectorn& pose);
					void setVelocity(MotionDOFinfo const& mDofInfo, vectorn const& pose);
					void getVelocity(MotionDOFinfo const& mDofInfo, vectorn & pose);
					vector3 getWorldVelocity(int ibone);
					vector3 getWorldAngVel(int ibone);
					void integrate(MotionDOFinfo const& mDofInfo, vectorn const& dtheta, double timeStep);
					void calcEffectorJacobianTranspose(matrixn& J);
					void calcJacobianTransposeAt(matrixn& J, int ibone, vector3 const& localpos);
					void getJacobianSparsity(boolN& hasValue, int ibone);
					void findAxes(boolN& hasValue, vector3 const& axis)
					void calcJacobianTransposeAt(matrixn& J, int ibone, int ibone2, vector3 const& localpos);
					void calcGlobalJacobianTransposeAt(matrixn& J, int ibone, vector3 const& localpos);
					vector3 calcCOM(const VRMLloader& loader);
					void calcCOMjacobianTranspose(const VRMLloader& loader, matrixn& JT, int chainRootBone, int chainTailBone);
					void calcCOMjacobianTranspose(const VRMLloader& loader, matrixn& JT);
					void calcRotJacobianTranspose(matrixn& J, int ibone);
					void calcMomentumJacobianTranspose(const VRMLloader& loader, matrixn& JT);
					Liegroup::dse3 calcMomentumCOM(const VRMLloader& loader);
					Liegroup::dse3 calcMomentumCOMfromPose(const VRMLloader& loader,double delta_t, BoneForwardKinematics &chain1); 
					Liegroup::dse3 calcMomentumCOMtoPose(const VRMLloader& loader,double delta_t, BoneForwardKinematics &chain2); 
					void getDTheta( vectorn& dq) 
					void getTheta( vectorn& q) 
					void setTheta( vectorn const& q) 
					void getDQ( vectorn& dq) 
					void getQ( vectorn& q) 
					void setQ( vectorn const& q) 
					void setDQ( vectorn const& q) 
					]],
					staticMemberFunctions=[[
					void Print(IK_sdls::LoaderToTree& self)
					]]
				},
				{
					name='LimbIKsolverLua',
					className='MotionUtil::LimbIKsolverLua',
					ctors={
					'(MotionDOFinfo const& dofInfo, std::vector<MotionUtil::Effector>& effectors, intvectorn const& hip_bone_indexes, intvectorn const& knee_bone_indexes, vectorn const& axis_sign, lunaState l)',
					'(MotionDOFinfo const& dofInfo, std::vector<MotionUtil::Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign, lunaState l)',
					   },
					   properties={
							'MotionDOFinfo  mDofInfo',
							'MotionLoader mSkeleton',
							'Posture tempp',
							'vector3N con' ,
							'quaterN conori' ,
							'vectorn impor',
							'vector3N mRootPos',
							'quaterN mRootOri',
							'std::vector<MotionUtil::Effector> mEffectors',
					   },
					memberFunctions={[[
					void IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance);
					void IKsolve(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con);
					void IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con);
					void setSkeleton(vectorn & temp);
					void _limbIK(vector3N const& conpos, quaterN const& conori, vectorn const& importance);
					void _forwardKinematics(int c, quater const& theta);
					void optimize(vectorn & initial);
					vectorn& getResult();
					void init_cg(int ndim, double grad_step, int max_iter, double tol, double thr)
					Bone* getCenterBone(int i) const 
					Bone* getHipBone(int i) const 
					Bone* getKneeBone(int i) const 
					Bone* getAnkle(int i) const 
				]]}
				},
				{
					name='LimbIKsolverHybrid',
					className='MotionUtil::LimbIKsolverHybrid',
					ctors={
					'(MotionDOFinfo const& dofInfo, std::vector<MotionUtil::Effector>& effectors, intvectorn const& hip_bone_indexes, intvectorn const& knee_bone_indexes, vectorn const& axis_sign)',
					'(MotionDOFinfo const& dofInfo, std::vector<MotionUtil::Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign)',
					},
					properties={
							'MotionDOFinfo  mDofInfo',
							'MotionLoader mSkeleton',
							'Posture tempp',
							'vector3N con' ,
							'quaterN conori' ,
							'vectorn impor',
							'vector3N mRootPos',
							'quaterN mRootOri',
							'std::vector<MotionUtil::Effector> mEffectors',
					},
					memberFunctions={[[
					void IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance);
					void IKsolve(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con);
					void IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con);
					void setSkeleton(vectorn & temp);
					void _limbIK(vector3N const& conpos, quaterN const& conori, vectorn const& importance);
					void _forwardKinematics(int c, quater const& theta);
					void optimize(vectorn & initial);
					vectorn& getResult();
					void init_cg(int ndim, double grad_step, int max_iter, double tol, double thr)
					Bone* getCenterBone(int i) const 
					Bone* getHipBone(int i) const 
					Bone* getKneeBone(int i) const 
					Bone* getAnkle(int i) const 
					]]}
				},
			},
			modules={
				{
					namespace='_G' -- global
				},
				{
					namespace='Imp',
					enums={
						{"LINE_CHART", "(int)Imp::LINE_CHART"},
						{"BAR_CHART", "(int)Imp::BAR_CHART"},
					},
					functions={[[
					CImage* Imp::DrawChart(const vectorn& vector, int chart_type);
					CImage* Imp::DrawChart(const vectorn& vector, int chart_type, float min, float max); 	
					CImage* Imp::DrawChart(const matrixn& matrix)
					CImage* Imp::DrawChart(const matrixn& matrix, float min, float max);
					CImage* Imp::DrawChart(const matrixn& matrix, int chart_type);
					CImage* Imp::DrawChart(const matrixn& matrix, int chart_type, float min, float max);
					CImage* Imp::DrawChart(const matrixn& matrix, int chart_type, float min, float max, float horizLine);
					void Imp::ChangeChartPrecision(int precision);
					void Imp::DefaultPrecision();
					]]}
				},
				{
					namespace='util',
					decl=[[
					#include <stdio.h>
#ifndef _MSC_VER
					#include <termios.h>
					#include <unistd.h>
#endif
					]],
					wrapperCode=[[
					static int getch( ) {
					  int            ch;
#ifndef _MSC_VER
					  struct termios oldt,
									 newt;
					  tcgetattr( STDIN_FILENO, &oldt );
					  newt = oldt;
					  newt.c_lflag &= ~( ICANON | ECHO );
					  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
					  ch = getchar();
					  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
#endif
					  return ch;
					}
					static void putch(const char* str ) {
						printf("%s",str);
						fflush(stdout);
					}
					]],
					functions ={[[
									  void OutputToFile(const char* filename, const char* string); @ outputToFile
									  int getch( ) 
									  void putch(const char* str)
						  ]]}
				},
				{
					namespace='math',
					wrapperCode=
					[[
					static int frexp(lua_State* L)
					{
						double d=lua_tonumber(L,1);
						double f;
						int e;
						f=::frexp(d,&e);
						lua_pushnumber(L,f);
						lua_pushnumber(L,e);
						return 2;
					}
					static void filter(matrixn &c, int kernelSize)
					{
						vectorn kernel;
						Filter::GetGaussFilter(kernelSize, kernel);
						Filter::LTIFilter(1,kernel, c); 
					}
					static void filterQuat(matrixn &c, int kernelSize)
					{
						vectorn kernel;
						Filter::GetGaussFilter(kernelSize, kernel);
						Filter::LTIFilterQuat(1,kernel, c); 
					}

					static void drawSignals(const char* filename, matrixn & in)
					{
						intvectorn temp;
						m::drawSignals (filename,in,0,0,temp);
					}
					static void drawSignals(const char* filename, matrixn & in, double fmin, double fmax)
					{
						intvectorn temp;
						m::drawSignals (filename,in,fmin,fmax,temp);
					}
					static void changeChartPrecision(int precision)
					{
						Imp::ChangeChartPrecision(precision);
					}

					static void defaultPrecision()
					{
						Imp::DefaultPrecision();
					}

					static void projectAngle(m_real& delta)	// represent angle in [-pi, pi]
					{	
						while(delta>M_PI+FERR)
							delta-=2.0*M_PI;
						while(delta<-1.0*M_PI-FERR)
							delta+=2.0*M_PI;
					}

					static void alignAngle(m_real prev, m_real& next)
					{

						// next-prev는 [-PI,PI]에 있다고 보장을 못한다. 따라서 이범위에 들어오는 next-prev를 구한다.

						m_real delta=next-prev;

						projectAngle(delta);

						// 다시 원래의 prev범위로 되돌린다.
						next=prev+delta;
					}
					static void alignAngles(vectorn & value, m_real x=0)
					{
						alignAngle(x, value(0));
						for(int i=1; i<value.size(); i++)
							alignAngle(value(i-1), value(i));
					}

					static void _throwTest()
					{
						throw std::runtime_error("_throwTest");

					}
					]],
					customFunctionsToRegister ={'frexp'},
					functions=
					{
						{'int ::Hash(const char* string);',rename='Hash'},
						{'void Filter::gaussFilter(int kernelsize, matrixn& inout);',rename='gaussFilter'},
						{'void Filter::medianFilter(int kernelsize, matrixn& inout);',rename='medianFilter'},
						{'void Filter::gaussFilter(int kernelsize, vectorn& inout);',rename='gaussFilter'},
						{'void Filter::medianFilter(int kernelsize, vectorn& inout);',rename='medianFilter'},
						{'int Filter::CalcKernelSize(float time, float frameTime);',rename='calcKernelSize'},
						[[
						void Filter::GetGaussFilter(int kernelSize, vectorn & kernel); @ getGaussFilter
						static void filter(matrixn &c, int kernelSize)
						static void filterQuat(matrixn &c, int kernelSize)
						static void changeChartPrecision(int precision)
						static void drawSignals(const char* filename, matrixn & in)
						static void drawSignals(const char* filename, matrixn & in, double fmin, double fmax)
						static void defaultPrecision()
						static void alignAngles(vectorn & value, m_real x)
						static void alignAngles(vectorn & value)
						static void _throwTest()
	void m::LUsolve(matrixn const & A, vectorn const& b, vectorn& x) @ LUsolve
	void m::PIsolve(matrixn const & A, vectorn const& b, vectorn& x) @ PIsolve
	bool intersectionTest::testTriTriOverlap(vector3 const& p1,  vector3 const& q1, vector3 const& r1, vector3 const& p2,  vector3 const& q2, vector3 const& r2);
						]]
					}

					--[[module(L , "math")
					[
					def("Hash", &Hash),
					class_<FuncWrapper>("Function")
					.def(constructor<>())
					.def("getInout", &FuncWrapper::getInout)
					.def("func", &FuncWrapper::func)
					.def("func", &FuncWrapper::func_dfunc)
					.def("info", &FuncWrapper::info),
					class_<GSLsolver>("GSLsolver")
					.def(constructor<FuncWrapper*, const char*>())
					.def("solve", &GSLsolver::solve),
					class_<CMAwrap>("CMAwrap")
					.def(constructor<vectorn const&, vectorn const&,int,int>())
					.def("testForTermination",&CMAwrap::testForTermination)
					.def("samplePopulation",&CMAwrap::samplePopulation)
					.def("numPopulation",&CMAwrap::numPopulation)
					.def("dim",&CMAwrap::dim)
					.def("getPopulation",&CMAwrap::getPopulation)
					.def("setVal",&CMAwrap::setVal)
					.def("resampleSingle",&CMAwrap::resampleSingle)
					.def("update",&CMAwrap::update)
					.def("getMean",&CMAwrap::getMean)
					.def("getBest",&CMAwrap::getBest)
					];
					]]
				}
			},
		}
bindTargetMainLib={
	namespaces={
		MainLib={
			'VRMLloader','VRMLloaderView'
		}
	},
	classes={

		{
			name='SelectPanel',
			decl='class FltkScrollSelectPanel;',
			className='FltkScrollSelectPanel',
			ifndef='NO_GUI',
			ctors={'()'},
			memberFunctions={[[
			bool isCreated()	
			void init(MotionPanel* pPanel, const char* label, int height, int maxValue);
			void release(MotionPanel* pPanel);
			void drawBoxColormap(int start, int end, int colormapValue);
			void drawBox(int start, int end, CPixelRGB8  color);
			void drawFrameLines(intvectorn const& frames);
			void drawTextBox(int start, int end, int colormapValue, const char* text);
			void clear(int start, int end);
			]]}
		},
		{
			luaname='MotionUtil.PhysicalProperties',
			cppname='PhysicalProperties',
			ctors={'(VRMLloader& skel)'},
			decl='class PhysicalProperties;',
			memberFunctions=[[
			void segPositions(const MotionDOF& srcMotion, hypermatrixn& aaSegPos);
			void segVelocity(hypermatrixn& aaSegVel, const hypermatrixn& aaSegPos, float kernelSize);
			void segAcceleration(hypermatrixn& aaSegAcc, const hypermatrixn& aaSegVel, float kernelSize);
			void ZMP(matrixn& aZMP, const hypermatrixn& aaSegPos, const hypermatrixn& aaSegAcc);
			]]
		},
		{
			name='SelectUI',
			className='SelectUI_lunawrapper',
			decl='class SelectUI_lunawrapper;',
			isLuaInheritable=true,
			ifndef='NO_GUI',
			globalWrapperCode=[[
			class SelectUI_lunawrapper : public FltkScrollPanel::SelectUI, public luna_wrap_object
			{
				public:
				SelectUI_lunawrapper(){
					RE::motionPanel().scrollPanel()->connectSelectUI(*this);
				}
				virtual ~SelectUI_lunawrapper(){
					RE::motionPanel().scrollPanel()->disconnectSelectUI(*this);
				}
				virtual void click(int iframe){
					lunaStack l(_L);
					if(pushMemberFunc<SelectUI_lunawrapper>(l,"click")){
						l<<(double)iframe;
						l.call(2,0);
					} 
				}
				virtual void selected(int iframe, int endframe){
					lunaStack l(_L);
					if(pushMemberFunc<SelectUI_lunawrapper>(l,"selected")){
						l<<(double)iframe;
						l<<(double)endframe;
						l.call(3,0);
					} 
				}
				virtual void panelSelected(const char* label, int iframe){
					lunaStack l(_L);
					if(pushMemberFunc<SelectUI_lunawrapper>(l,"panelSelected")){
						l<<(TString)label;
						l<<(double)iframe;
						l.call(3,0);
					} 
				}
				virtual bool startDragging(const char* label, int iframe, int& adjusedStart, int& adjustedEnd) { 
					lunaStack l(_L);
					if(pushMemberFunc<SelectUI_lunawrapper>(l,"startDragging")){
						l<<(TString)label;
						l<<(double)iframe;
						l.call(3,1);
						bool temp;
						l>>temp;
						return temp;
					} 
					return false;
				}
				virtual void dragging(const char* label, int original_frame, int dragged_frame, int& adjusedStart, int& adjustedEnd){
					lunaStack l(_L);
					if(pushMemberFunc<SelectUI_lunawrapper>(l,"dragging")){
						l<<label;
						l<<(double)original_frame;
						l<<(double)dragged_frame;
						l.call(4,0);
					} 
				}
				virtual void finalize(const char* label, int original_iframe, int dragged_frame){
					lunaStack l(_L);
					if(pushMemberFunc<SelectUI_lunawrapper>(l,"finalize")){
						l<<label;
						l<<(double)original_iframe;
						l<<(double)dragged_frame;
						l.call(4,0);
					} 
				}
			};
			]],
			ctor={'()'}
		},
		{
			name='lunaState',
			memberFunctions=[[
			void print()
				]]
		},
		{
			name='EventReceiver',
			className='EventReceiver_lunawrapper',
			decl='class EventReceiver_lunawrapper;',
			isLuaInheritable=true,
			globalWrapperCode=[[
#ifdef NO_GUI
struct EventReceiver_lunawrapper: FrameMoveObject, public luna_wrap_object
{
	EventReceiver_lunawrapper()
	{
		RE::renderer().addFrameMoveObject(this);
	}

	virtual ~EventReceiver_lunawrapper()
	{
		RE::renderer().removeFrameMoveObject(this);
	}

	virtual int FrameMove(float fElapsedTime)  {
		lunaStack l(_L);
		if(pushMemberFunc<EventReceiver_lunawrapper>(l,"frameMove")){
			l<<(double)fElapsedTime;
			l.call(2,0);
		} 
		return 1;
	}
};
#else
struct EventReceiver_lunawrapper: FltkMotionWindow ::EventReceiver, FrameMoveObject , public luna_wrap_object
{
	EventReceiver_lunawrapper()
		: FltkMotionWindow ::EventReceiver()
	{
		RE::motionPanel().motionWin()->connect(*this);
		RE::renderer().addFrameMoveObject(this);
	}

	virtual ~EventReceiver_lunawrapper()
	{
		if(RE::motionPanelValid())
			RE::motionPanel().motionWin()->disconnect(*this);
		RE::renderer().removeFrameMoveObject(this);
	}

	virtual void OnNext(FltkMotionWindow* win)
	{
		lunaStack l(_L);
		if(pushMemberFunc<EventReceiver_lunawrapper>(l, "onNext")){
			l.push<FltkMotionWindow>(win);
			l.call(2,0);
		}
	}
	virtual void OnPrev(FltkMotionWindow* win)
	{
		lunaStack l(_L);
		if(pushMemberFunc<EventReceiver_lunawrapper>(l, "onPrev")){
			l.push<FltkMotionWindow>(win);
			l.call(2,0);
		}
	}
	virtual void OnFrameChanged(FltkMotionWindow* win, int i)
	{
		lunaStack l(_L);
		if(pushMemberFunc<EventReceiver_lunawrapper>(l,"onFrameChanged")){
			l.push<FltkMotionWindow>(win);
			l<<(double)i;
			l.call(3,0);
		} 
	}
	virtual int FrameMove(float fElapsedTime)
	{
		lunaStack l(_L);
		if(pushMemberFunc<EventReceiver_lunawrapper>(l, "frameMove")){
			l<<(double)fElapsedTime;
			l.call(2,0);
		}
		return 1;
	}
};
#endif
			]],
			ctors={'()'},
		},
		{
			name='Fltk.ChoiceWins',
			className='FlChoiceWins',
			wrapperCode=[[
			static FlChoiceWins* choiceWin(FlChoiceWins* win, int i)
			{
				return dynamic_cast<FlChoiceWins*>(win->window(i));
			}

			static FlLayout* layout(FlChoiceWins* win, int i)
			{
				return static_cast<FlLayout*>(win->window(i));
			}
			]],
			properties={'TStrings windowNames;'},
			memberFunctions={
				[[
				void show(int)
				]]
			},
			staticMemberFunctions={
				[[
				static FlChoiceWins* choiceWin(FlChoiceWins* win, int i)
				static FlLayout* layout(FlChoiceWins* win, int i)
				]]
			}
		},
		{
			name='math.Point2D',
			className='std::pair<double,double>',
			ctors={'()','(double,double)'},
			properties={'double first @ x', 'double second @ y'},
		},
		{
			name='math.vecPoint2D',
			className='std::vector<std::pair<double,double> >'
		},
		{
			name='MeshToEntity',
			className='OBJloader::MeshToEntity',
			ctors={
				'(const OBJloader::Mesh& mesh, const char* ogreMeshName)',
				'(const OBJloader::Mesh& mesh, const char* ogreMeshName, bool buildEdgeList, bool dynamicUpdate)'
			},
			memberFunctions={[[
			void updatePositions();
			void updatePositionsAndNormals();
			Ogre::Entity* createEntity(const char* entityName);
			Ogre::Entity* createEntity(const char* entityName, const char* materialName);
			Ogre::Entity* getLastCreatedEntity() const
			]] }
		},
		{
			name='Mesh',
			className='OBJloader::Mesh',
			ctors={'()'},
			properties={'boolN isBoundaryVertex'},
			enums={
				{"VERTEX", "(int)OBJloader::Buffer::VERTEX"},
				{"NORMAL", "(int)OBJloader::Buffer::NORMAL"},
				{"TEXCOORD", "(int)OBJloader::Buffer::TEXCOORD"},
				{"COLOR", "(int)OBJloader::Buffer::COLOR"},
				{"NUM_BUFFER", "(int)OBJloader::Buffer::NUM_BUFFER"},
			},
			memberFunctions={[[
				int numFace() const 
				OBJloader::Face & getFace(int i)
				int numVertex() const;
				int numNormal() const;
				int numTexCoord() const;
				vector3& getVertex(int i);
				vector3& getNormal(int i);
				vector2& getTexCoord(int i);
				bool saveMesh(const char* filename_);
				bool loadMesh(const char* filename_);
				bool loadMesh(const char* filename_, bool bInit);
				bool saveObj(const char* filename, bool vn, bool vt); @ saveOBJ
				bool loadObj(const char* filename); @ loadOBJ
				void copyFrom(OBJloader::Mesh const& otherMesh); @ assignMesh
				void merge(OBJloader::Mesh const& a, OBJloader::Mesh const& b); @ mergeMesh
				void transform(matrix4 const& b);
				void resize(int numVertex, int numFace);
				void resize(int numVertex, int numNormal, int numTexCoord, int numColor, int numFace);
				void pack(BinaryFile& bf);
				void unpack(BinaryFile& bf);
				void calculateVertexNormal();
				void removeFaces(intvectorn const& faceIndices);
				void addVertices(vector3N const& vertices);
				void addNormals(vector3N const& normals);
				void addFaces(intmatrixn const& faces);
				void resizeIndexBuffer(int numFace);
				void resizeVertexBuffer(int n) 
				void resizeBuffer(OBJloader::Buffer::Type t, int n)
				vector3 calcFaceCenter(int i) const;
				vector3 calcFaceNormal(int i) const;
				vector3 calcMeshCenter() const;
			]]},
			staticMemberFunctions={[[
				// For collision detection, use Geometry::initBox instead!!!
				void OBJloader::createBox(OBJloader::Mesh& mesh, m_real sizeX, m_real sizeY, m_real sizeZ);  @  _initBox
				void OBJloader::createCylinder(OBJloader::Mesh& mesh, m_real radius, m_real height, int ndiv); @ _initCylider
				void OBJloader::createCircle(OBJloader::Mesh& mesh, vector3 const& center, m_real radius, int ndiv);
				void OBJloader::createPlane(OBJloader::Mesh& mesh, int numSegX, int numSegZ, m_real sizeX, m_real sizeZ) @ _initPlane
				void OBJloader::convertTerrainToBMP(const char* filename, int sizeX,int sizeY, const char* outBMPfile);
				void OBJloader::convertTerrainFromBMP(const char* BMPfilename, const char* outfile)
			   ]]}
		},
		{
			name='OBJloader.Terrain',
			inheritsFrom='OBJloader::Mesh',
			headerFile='../../BaseLib/motion/Terrain.h',
			decl='namespace OBJloader { class Terrain;}',
			ctors=[[
			(const char* filename, int imageSizeX, int imageSizeY, m_real sizeX, m_real sizeZ, m_real heightMax, int ntexSegX, int ntexSegZ)
			(const char* filename, int imageSizeX, int imageSizeY, m_real sizeX, m_real sizeZ, m_real heightMax, int ntexSegX, int ntexSegZ, bool tileZ)
			(vectorn const& image1d, m_real sizeX, m_real sizeZ, m_real heightMax, int ntexSegX, int ntexSegZ, bool tileZ)
			]],
			memberFunctions=[[
			m_real height(vector2 x, vector3& normal) const;
			m_real height(vector2 x) const;
			vector3 pick(Ray const& ray, vector3& normal) const;
			bool isInsideTerrain(vector2 x) const
			]]
		},
		{
			decl='namespace OBJloader { struct FaceEdge;}',
			name= 'OBJloader::FaceEdge',
			ctor={'()', '(int fi, int vi)'},
			properties={'int faceIndex','int vertexIndex'},
			memberFunctions=[[
			int source(OBJloader::Mesh const& mesh) const;
			int target(OBJloader::Mesh const& mesh) const;
			int cross(OBJloader::Mesh const& mesh) const;	// 맞은편 vertex
			vector3& getSource(OBJloader::Mesh & mesh);
			vector3& getTarget(OBJloader::Mesh & mesh);
			bool isNull() const;
			bool operator==(const OBJloader::FaceEdge& other); @ __eq
			]],
		},
		{
			name='EdgeConnectivity',
			className='OBJloader::EdgeConnectivity',
			ctors={'(OBJloader::Mesh const&)'},
			memberFunctions=[[
			int numEdges() const;
			int numAdjEdges(int vertexIndex) const;
			bool isConnected(int vertex1, int vertex2);
			int getAdjVertex(int vertexIndex, int i) const;
			int source(int iEdge) const;
			int target(int iEdge) const;
			int getNumFaceEdge(int iEdge) const 
			void selectBoundaryEdgesFromVertices(boolN const& selectedVertices, boolN& selectedEdges);
			OBJloader::FaceEdge const& getFaceEdge(int iEdge, int iFaceEdge) const 
			]]
		},
		{
			decl='namespace OBJloader { struct MeshConnectivity;}',
			name='OBJloader::MeshConnectivity',
			ctors={'(OBJloader::Mesh const& mesh, OBJloader::EdgeConnectivity const & edges)'},
			memberFunctions=[[
			OBJloader::FaceEdge vertexToFace(int vertex);
			OBJloader::FaceEdge adjFace(OBJloader::FaceEdge const& fe) const;
			OBJloader::FaceEdge next(OBJloader::FaceEdge const& fe) const;
			OBJloader::FaceEdge prev(OBJloader::FaceEdge const& fe) const;
			bool isValid() const ;
			void selectVerticesFromFaces(boolN const& selectedFaces, boolN& selectedVertices);
			void selectFacesFromVertices(boolN const& selectedVertices, boolN& selectedFaces); // buggy for non-perfect meshes.
			]],
		},
		{
			name='OBJloader::Element',
			properties={
				'int elementType',
				'vector3 elementSize',
				'transf tf'
			},
			enums={
				{"BOX", "(int)OBJloader::Element::BOX"},
				{"CYLINDER", "(int)OBJloader::Element::BOX"},
				{"CAPSULE", "(int)OBJloader::Element::CAPSULE"},
				{"OBJ", "(int)OBJloader::Element::OBJ"},
				{"ELLIPSOID", "(int)OBJloader::Element::ELLIPSOID"},
				{"PLANE", "(int)OBJloader::Element::PLANE"},
				{"SPHERE", "(int)OBJloader::Element::SPHERE"},
			},
		},
		{
			name='Geometry',
			ctors={'()'},
			className='OBJloader::Geometry',
			inheritsFrom='OBJloader::Mesh',
			properties={'intIntervals faceGroups'},
			wrapperCode=[[
				
			]],
			memberFunctions=[[
			int numElements() 
			OBJloader::Element const& element(int i)
			void scale(vector3 const& scalef);
			void scale(vector3 const& scalef, int ifacegroup);
			void scaleElements(vector3 const& scalef);
			void scaleElements(vector3 const& scalef, int ifacegroup);
			void rigidTransform(transf const& b);
			void rigidTransform(transf const& b, int ifacegroup);
			void scaleAndRigidTransform(matrix4 const& m); // uses only scale, rotation, and translation components. Shearing components are discarded.
			void initBox(const vector3& size)
			void initCylinder(double radius, double height, int numDivision)
			void initCapsule(double radius, double height)
			void initEllipsoid(const vector3& size)
			void initPlane(double size_x, double size_z)  
			void extractSubMesh(OBJloader::Geometry const& otherMesh, int isubMesh);
			void merge(OBJloader::Geometry const& a, OBJloader::Geometry const& b); 
			void convertToOBJ()
			void copyFrom(OBJloader::Geometry const& otherMesh); @ assign
			]],
		},
		{
			name='MeshEntity',
			ifndef='NO_GUI',
			className='OBJloader::MeshEntity',
			parentClass='OBJloader.Mesh',
			memberFunctions=[[
				void setCastShadows(bool )
				void setMaterial(const char* )
				void setVisible(bool)
				void firstInit()
				void update()
			]]
		},
		{
			name='TriangleMesh',
			ifndef='NO_GUI',
			className='OBJloader::TriangleMesh',
			parentClass='OBJloader.MeshEntity',
			ctor='()',
		},
		{
			name='MeshLineStrip',
			ifndef='NO_GUI',
			className='OBJloader::MeshLineStrip',
			parentClass='OBJloader.MeshEntity',
			ctor='()',
		},
		{
			name='MeshLineStripReduced',
			ifndef='NO_GUI',
			className='OBJloader::MeshLineStripReduced',
			parentClass='OBJloader::MeshEntity',
			ctor='()',
		},
		{
			name='__luna.worker',
			className='LUAwrapper::Worker',
				wrapperCode=[[
					static int __call(lua_State *L)
				{
					lunaStack lua(L);
					LUAwrapper::Worker* self=lua.check<LUAwrapper::Worker>();
					std::string workName;
					lua>>workName;

					TString w(workName.c_str());
					return self->work(w,lua);
				}
			]],
			customFunctionsToRegister={'__call'},
		},
		{
			name='GlobalUI',
			inheritsFrom='LUAwrapper::Worker',
		},
		{
			name='FlLayout',
			inheritsFrom='LUAwrapper::Worker',
			ctors={'(int wx, int wy, const char* title )', },
			wrapperCode=[[
			inline static void wait()
			{
				#ifndef NO_GUI
				Fl::wait();
				#endif
			}
			]],
			memberFunctions=[[
				bool visible()
				void begin()
				FlLayout* layout(int n);
				FlLayout* findLayout(const char* id)
				void create(const char* type, const char* id, const char* title);
				void create(const char* type, const char* id);
				void create(const char* type, const char* id, const char* title, int startSlot, int endSlot, int height);
				void resetToDefault()
				void create(const char* type, const char* id, const char* title, int startSlot, int endSlot);
				void create(const char* type, const char* id, const char* title, int startSlot);
				void newLine(); // 다음줄로 옮긴다. (디폴트 widgetPosition에서는 자동으로 넘어가니 call안해주어도 됨.)
				void removeWidgets(int startWidgetIndex);
				void embedLayout(FlLayout* childLayout, const char* id, const char* title);
				void setLineSpace(int l);
				void setHorizSpace(int h);
				void setWidgetHeight(int h);
				void setWidgetPos(int startSlot, int endSlot); // guideline 따라 나누어져있는 영역에서 얼만큼 차지할지.
				void setUniformGuidelines(int totalSlot); // 가로로 totalSlot등분한다.
				void updateLayout();
				void resetToDefault()
				void redraw()
				void show()
				void activate() @ ;ifndef=NO_GUI;
				void deactivate() @ ;ifndef=NO_GUI;
				int minimumHeight();
				FlLayout::Widget& widgetRaw(int n); @ widget
				int widgetIndex(const char* id);	//!< 버튼 10개를 생성한후 첫번째 버튼의 index를 받아오면, 그 이후의 버튼은 index+1, index+2.등으로 접근 가능하다.
				FlLayout::Widget& findWidget(const char* id);
				void callCallbackFunction(FlLayout::Widget& w);
			 ]],
			 staticMemberFunctions=[[
			 void wait()
			 ]],
		},
		{
			name='FlLayout.Widget',
			wrapperCode=[[
			static void checkButtonValue(FlLayout::Widget& w, int value)
			{
				w.checkButton()->value(value);
			}
			static void checkButtonValue3(FlLayout::Widget& w, bool value)
			{
				w.checkButton()->value(value);
			}
			static bool checkButtonValue2(FlLayout::Widget& w)
			{
				return w.checkButton()->value();
			}
			static void menuSize(FlLayout::Widget& w,int nsize)
			{
				w.menu()->size(nsize);
			}
			static void menuItem(FlLayout::Widget& w,int i, const char* title)
			{
				w.menu()->item(i, title);
			}
			static void menuItem2(FlLayout::Widget& w,int i, const char* title, const char* shortc)
			{
				w.menu()->item(i, title, FlGenShortcut(shortc));
			}
			static void menuValue(FlLayout::Widget& w, int v)
			{
				w.menu()->value(v);
				w.menu()->redraw();
			}
			static std::string menuText(FlLayout::Widget& w, int v)
			{
#ifndef NO_GUI
				return w.menu()->text(v);
#else
				return "";
#endif
					
			}
			static std::string menuText2(FlLayout::Widget& w)
			{
#ifndef NO_GUI
				return w.menu()->text();
#else
				return "";
#endif
			}
			static int menuValue2(FlLayout::Widget& w)
			{
				return w.menu()->value();
			}
			static void setVisible(FlLayout::Widget& w)
			{
				w.widget<Fl_Widget>()->set_visible();
			}
			static void userData(FlLayout::Widget& w, int userData)
			{
				w.widget<Fl_Widget>()->user_data(reinterpret_cast<void*>((long long)(userData)));
			}
			static int userData(FlLayout::Widget& w)
			{
				return (int)reinterpret_cast<long long>(w.widget<Fl_Widget>()->user_data());
			}
			static void clearVisible(FlLayout::Widget& w)
			{
				w.widget<Fl_Widget>()->clear_visible();
			}
			static void sliderValue(FlLayout::Widget& w, double v)
			{
				w.slider()->value(v);
			}
			static void sliderStep(FlLayout::Widget& w, double v)
			{
#ifndef NO_GUI
				w.slider()->step(v);
#endif
			}
			static void sliderRange(FlLayout::Widget& w, double v1, double v2)
			{
#ifndef NO_GUI
				w.slider()->range(v1, v2);
#endif
			}
			static double sliderValue2(FlLayout::Widget& w)
			{
				return w.slider()->value();
			}
			static void buttonShortcut(FlLayout::Widget& w, const char* s)
			{
#ifndef NO_GUI				
				w.button()->shortcut(FlGenShortcut(s));
				w.button()->tooltip(s);
#endif
			}
			static void buttonTooltip(FlLayout::Widget& w, const char* s)
			{
#ifndef NO_GUI				
				w.button()->tooltip(s);
#endif
			}
			static void buttonSetLabel(FlLayout::Widget& w, const char* s)
			{
#ifndef NO_GUI				
				w.button()->copy_label(s);
#endif
			}
			static const char* buttonLabel(FlLayout::Widget& w)
			{
#ifndef NO_GUI				
				return w.button()->label();
#else
				return "button";
#endif
			}
			static void redraw(FlLayout::Widget& w)
			{
				w.widget<Fl_Widget>()->redraw();
			}
			static void deactivate(FlLayout::Widget& w)
			{
#ifndef NO_GUI				
				w.widget<Fl_Widget>()->deactivate();
#endif
			}
			static void activate(FlLayout::Widget& w)
			{
#ifndef NO_GUI				
				w.widget<Fl_Widget>()->activate();
#endif
			}
			static const char* id(FlLayout::Widget& w)
			{
				return w.mId;
			}
			static int browserSize(FlLayout::Widget& w)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				return browser->size();
#else
				return 0;
#endif
			}
			static bool browserSelected(FlLayout::Widget& w, int i)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				return browser->selected(i);
#else
				return false;
#endif
			}
			static int browserValue(FlLayout::Widget& w)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				return browser->value();
#else
				return 0;
#endif
			}
			static const char* browserText(FlLayout::Widget& w, int i)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				return browser->text(i);
#else
				return 0;
#endif
			}

			static void browserDeselect(FlLayout::Widget& w)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				browser->deselect();
#endif
			}
			static void browserSelect(FlLayout::Widget& w,int i)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				browser->select(i);
#endif
			}
			static void browserAdd(FlLayout::Widget& w, const char* name)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				browser->add(name,NULL);
#endif
			}
			static void browserRemove(FlLayout::Widget& w, int i)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				browser->remove(i);
#endif
			}
			static void browserClear(FlLayout::Widget& w)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				browser->clear();
#endif
			}
			static void inputValue1(FlLayout::Widget& w, const char* text)
			{
#ifndef NO_GUI
				Fl_Input* input=w.widget<Fl_Input>();
				input->value(text);
#endif
			}
			static std::string inputValue2(FlLayout::Widget& w)
			{
				std::string str;
#ifndef NO_GUI
				Fl_Input* input=w.widget<Fl_Input>();
				str=input->value();
#endif
				return str;
			}
static void inputType(FlLayout::Widget& w, const char* str)
			{
#ifndef NO_GUI
				if (TString(str)=="FL_MULTILINE_OUTPUT")
				{
					Fl_Input* ip=w.widget<Fl_Input>();
					ip->type(FL_MULTILINE_OUTPUT);
				}
#endif
			}
			]],
			staticMemberFunctions={[[
			static void checkButtonValue(FlLayout::Widget& w, int value)
			static bool checkButtonValue2(FlLayout::Widget& w) @ checkButtonValue
			static void checkButtonValue3(FlLayout::Widget& w, bool value) @ checkButtonValue
			static void menuSize(FlLayout::Widget& w,int nsize)
			static void menuItem(FlLayout::Widget& w,int i, const char* title)
			static void menuItem2(FlLayout::Widget& w,int i, const char* title, const char* shortc) @ menuItem
			static void menuValue(FlLayout::Widget& w, int v)
			static std::string menuText(FlLayout::Widget& w, int v)
			static std::string menuText2(FlLayout::Widget& w) @ menuText
			static int menuValue2(FlLayout::Widget& w) @ menuValue
			static void sliderValue(FlLayout::Widget& w, double v)
			static void sliderStep(FlLayout::Widget& w, double v)
			static void sliderRange(FlLayout::Widget& w, double v1, double v2)
			static double sliderValue2(FlLayout::Widget& w) @ sliderValue
			static void buttonShortcut(FlLayout::Widget& w, const char* s)
			static void buttonTooltip(FlLayout::Widget& w, const char* s)
			static void buttonSetLabel(FlLayout::Widget& w, const char* s)
			static const char* buttonLabel(FlLayout::Widget& w)
			static void redraw(FlLayout::Widget& w)
			static void setVisible(FlLayout::Widget& w)
			static int userData(FlLayout::Widget& w)
			static void userData(FlLayout::Widget& w, int u)
			static void clearVisible(FlLayout::Widget& w)
			static void deactivate(FlLayout::Widget& w)
			static void activate(FlLayout::Widget& w)
			static const char* id(FlLayout::Widget& w)
			static int browserSize(FlLayout::Widget& w)
			static bool browserSelected(FlLayout::Widget& w, int i)
			static void browserDeselect(FlLayout::Widget& w)
			static void browserSelect(FlLayout::Widget& w,int i)
			static void browserAdd(FlLayout::Widget& w, const char* name)
			static void browserRemove(FlLayout::Widget& w, int i)
			static void browserClear(FlLayout::Widget& w)
			static int browserValue(FlLayout::Widget& w)
			static const char* browserText(FlLayout::Widget& w, int i)
			static void inputValue1(FlLayout::Widget& w, const char* text) @ inputValue
			static std::string inputValue2(FlLayout::Widget& w) @ inputValue
			static void inputType(FlLayout::Widget& w, const char* str)
			]]}

		},
		{
			-- inherits LuaFlLayout and implements __init and onCallback
			name='LuaFlLayout',
			className='LuaFlLayout',
			decl='class LuaFlLayout;',
			isLuaInheritable=true,
			inheritsFrom='FlLayout',
			ifndef='NO_GUI',
			globalWrapperCode=[[
			class LuaFlLayout: public FlLayout, public luna_wrap_object
			{
				public:
				LuaFlLayout()
				:FlLayout(300,300){ // has to have a default constructor
					_L=NULL;
				}
				void init(lunaState* ll)
				{
					_L=ll->ptr();
				}

				virtual ~LuaFlLayout() { }
				void showAndWait()
				{
					show();
					while(visible())
						Fl::wait();
				}

				void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
				{
					lunaStack l(_L);
					if(pushMemberFunc<LuaFlLayout>(l,"onCallback")){
						l.push<FlLayout::Widget>(w);
						l<<(double)userData;
						l.call(3,0);
					} 
				}
			};
			]],
			ctors={'()', },
			memberFunctions=[[
				void showAndWait();
				void init(lunaState*ll);
				void resize(int x, int y, int w, int h);
			]]
		},
		{
			name='QuadraticFunction',
			ctors={'()'},
			memberFunctions={[[
					void addSquared(intvectorn const& index, vectorn const& value);
					void buildSystem(int nvar, matrixn & A, vectorn &b);
				]]},
		},
		{
			name='QuadraticFunctionHardCon',
			ctors={'(int,int)'},
			properties={
			'int mNumVar @ numVar',
			'int mNumCon @ numCon',
			},
			memberFunctions={[[
				void addSquared(intvectorn const& index, vectorn const& value);
				void addCon(intvectorn const& index, vectorn const& value);
				void buildSystem(matrixn & A, vectorn &b);
			]]},
		},
		{
			name='OBJloader.Face',
			memberFunctions=[[
			void setIndex(int i, int j, int k) // vertexIndex
			void setIndex(int i, int j, int k, OBJloader::Buffer::Type bufferType) // any index type
			void setIndex1(int i, int vi)  @ setVertexIndex
			void setIndex1(int i, int vi, OBJloader::Buffer::Type bufferType)  
			int vertexIndex(int i);
			int normalIndex(int i) ;
			int texCoordIndex(int i);
			int colorIndex(int i);
			void operator=(OBJloader::Face const&) @ assign
			]]
		},
{
	name='Ogre.MovableObject',
	memberFunctions={[[
        void setCastShadows(bool enabled) @ ;ifndef=NO_GUI;
]]}
},
{
name='Ogre.Light',
	ifndef='NO_GUI',
			inheritsFrom='Ogre::MovableObject',
	wrapperCode=[[
			static void setType(Ogre::Light* light, const char* type)
			{
				TString t=type;
				if(t=="LT_DIRECTIONAL")
					light->setType(Ogre::Light::LT_DIRECTIONAL);
				else if(t=="LT_POINT")
					light->setType(Ogre::Light::LT_POINT);
				else
					light->setType(Ogre::Light::LT_SPOTLIGHT);
			}

			static void setDirection(Ogre::Light* light, m_real x, m_real y, m_real z)
			{light->setDirection(Ogre::Vector3(x,y,z));}
			static void setPosition(Ogre::Light* light, m_real x, m_real y, m_real z)
			{light->setPosition(x,y,z);}
			static void setDiffuseColour(Ogre::Light* light, m_real x, m_real y, m_real z)
			{light->setDiffuseColour(Ogre::ColourValue(x,y,z));}
			static void setSpecularColour(Ogre::Light* light, m_real x, m_real y, m_real z)
			{light->setSpecularColour(Ogre::ColourValue(x,y,z));}
	]],
staticMemberFunctions={[[
			static void setType(Ogre::Light* light, const char* type)
			static void setSpecularColour(Ogre::Light* light, m_real x, m_real y, m_real z)
			static void setDiffuseColour(Ogre::Light* light, m_real x, m_real y, m_real z)
			static void setPosition(Ogre::Light* light, m_real x, m_real y, m_real z)
			static void setDirection(Ogre::Light* light, m_real x, m_real y, m_real z)
		   ]]}
		},
		{
			name='Ogre.SkeletonInstance',
			ifndef='NO_GUI',
			memberFunctions=[[
				Ogre::Bone* getBone(short boneIndex);
				short getNumBones(void) const;
				void _updateTransforms(void);
			]]
		},
		{ 
			name='Ogre.Entity',
			ifndef='NO_GUI',
			inheritsFrom='Ogre::MovableObject',
			wrapperCode=[[
				static void setNormaliseNormals(Ogre::Entity& e)
				{
					// do nothing.
				}
				static void setNormaliseNormals(Ogre::Entity& e, bool a)
				{
					// do nothing.
				}

				static void setMaterialName(Ogre::Entity& e, const char* matName)
				{
#ifndef NO_OGRE
					e.setMaterialName(matName);
#endif
				}
				static void getBoundingBox(Ogre::Entity& e, vector3& min, vector3& max)
				{
#ifndef NO_OGRE
					Ogre::AxisAlignedBox aabb;
					aabb=e.getBoundingBox();
					min=ToBase(aabb.getMinimum());

					max=ToBase(aabb.getMaximum());
#endif
				}
				static void setRenderingDistance (Ogre::Entity& e, double dist)
				{
#ifndef NO_OGRE
					e.setRenderingDistance(dist);
#endif
				}
			]],
			staticMemberFunctions={[[
				void setNormaliseNormals(Ogre::Entity& e)
				void setNormaliseNormals(Ogre::Entity& e, bool )
				void setMaterialName(Ogre::Entity& e, const char* matName)
				void getBoundingBox(Ogre::Entity& e, vector3& min, vector3& max)
				void setRenderingDistance (Ogre::Entity& e, double dist)
			]]},
			memberFunctions={[[
				Ogre::SkeletonInstance* getSkeleton()
			]]},
		},
		{
			name='Ogre.Overlay',
			ifndef='NO_GUI',
			wrapperCode=[[
			inline static void setChildPosition(Ogre::Overlay* overlay, const char* name, int x, int y)
			{
#if OGRE_VERSION_MINOR<9
				Ogre::OverlayElement* pElt=overlay->getChild(name);
#else
				Ogre::OverlayContainer* pElt=overlay->getChild(name);
#endif
				pElt->setPosition(x,y);
			}
			]],
			memberFunctions={[[
			void setZOrder(ushort zorder);
			ushort getZOrder(void) const;
			void add2D(Ogre::OverlayContainer* cont);
			void show(void);
			void hide(void);
			bool isVisible()
			]]},
			staticMemberFunctions=[[
			void setChildPosition(Ogre::Overlay* o, const char* name, int x, int y)
			]],
		},
		{
			name='Ogre.OverlayElement',
			ifndef='NO_GUI',
			wrapperCode=[[
			static void setCaption(Ogre::OverlayElement* p, const char* caption)
			{
				p->setCaption(caption);
			}

			static void setMaterialName(Ogre::OverlayElement* p, const char* c)
			{
				p->setMaterialName(c);
			}

			static void setParameter(Ogre::OverlayElement* p, const char* c, const char* d)
			{
				p->setParameter(c,d);
			}
			]],
			staticMemberFunctions={[[
			static void setCaption(Ogre::OverlayElement* p, const char* caption)
			static void setMaterialName(Ogre::OverlayElement* p, const char* c)
			static void setParameter(Ogre::OverlayElement* p, const char* c, const char* d)
			]]},
			memberFunctions=[[
			void setPosition(int, int);
			]]
		},
		{
			name='Ogre.OverlayContainer',
			ifndef='NO_GUI',
			inheritsFrom='Ogre::OverlayElement',
			memberFunctions={[[
			void addChild(Ogre::OverlayElement* elem);
			void setPosition(int, int);
			]]}
		},
		{
			name='math.Metric',
			className='Metric',
			memberFunctions={[[
				m_real CalcDistance(const vectorn& a, const vectorn& b); @ calcDistance
			]]}
		},
		{
			name='math.L2Metric',
			className='L2Metric',
			inheritsFrom='Metric',
			ctors={'()'},
			memberFunctions={[[
				m_real CalcDistance(const vectorn& a, const vectorn& b); @ calcDistance
			]]}
		},
		{
			name='math.PointCloudMetric',
			className = 'PointCloudMetric',
			ctors={'()'},
			properties={
			'matrix4 m_transfB @ transfB',
			'matrixn m_transformedB @ transformedB',
			},
			memberFunctions={[[
			// align b to a by rotating b.
			m_real CalcDistance(const vectorn& a, const vectorn& b); @ calcDistance
			]]}
		},
		{
			name='math.WeightedPointCloudMetric',
			className='WeightedPointCloudMetric',
			ctors={'(vectorn const&)'},
			properties={
			'matrix4 m_transfB @ transfB',
			'matrixn m_transformedB @ transformedB',	
			'bool errorOccurred',
			},
		memberFunctions={[[
			// align b to a by rotating b.
			m_real CalcDistance(const vectorn& a, const vectorn& b); @ calcDistance
		]]}
		},
		{
			name='math.KovarMetric',
			className='KovarMetric',
			inheritsFrom='Metric',
			ctors={'()', '(bool)'},
			properties={
			'matrix4 m_transfB @ transfB',
			'matrixn m_transformedB @ transformedB',	
			},
		memberFunctions={[[
			m_real CalcDistance(const vectorn& a, const vectorn& b); @ calcDistance
		]]}
		},
		{
			name='math.NoRotMetric',
			className='NoRotMetric',
			ctors={'()', '(bool)'},
			properties={
			'matrix4 m_transfB @ transfB',
			'matrixn m_transformedB @ transformedB',	
			},
		memberFunctions={[[
			m_real CalcDistance(const vectorn& a, const vectorn& b); @ calcDistance
		]]}
		},
		{
			name='Ogre.Node',
			wrapperCode=[[
			static void setOrientation(Ogre::Node* pNode, m_real w, m_real x, m_real y, m_real z)
			{OGRE_VOID(pNode->setOrientation(w, x,y,z));}
			static void setOrientation(Ogre::Node* pNode, quater const& q)
			{OGRE_VOID(pNode->setOrientation(q.w, q.x,q.y,q.z));}
			static quater getOrientation(Ogre::Node* pNode)
			{
#if !defined (NO_GUI)                                         
				return ToBase(pNode->getOrientation());
#else
				return quater(1,0,0,0);
#endif
			}
			static vector3 getPosition(Ogre::Node* pNode)
			{
#if !defined (NO_GUI)                                         
				return ToBase(pNode->getPosition());
#else
				return vector3(0,0,0);
#endif
			}
			static vector3 getScale(Ogre::Node* pNode)
			{
#if !defined (NO_GUI)                                         
				return ToBase(pNode->getScale());
#else
				return vector3(1,1,1);
#endif
			}
			static vector3 _getDerivedPosition(Ogre::Node* pNode)
			{
#if !defined (NO_GUI)                                         
				return ToBase(pNode->_getDerivedPosition());
#else
				return vector3(0,0,0);
#endif
			}
			static std::string getName(Ogre::Node* pNode)
			{
#if !defined (NO_GUI)                                         
				return std::string(pNode->getName());
#else
				return std::string("unknown");
#endif
			}
			static void setPosition(Ogre::Node* pNode, m_real x, m_real y, m_real z)
			{OGRE_VOID(pNode->setPosition(x,y,z));}
			static void setPosition(Ogre::Node* pNode, vector3 const& t)
			{OGRE_VOID(pNode->setPosition(t.x,t.y,t.z));}
			static void setScale(Ogre::Node* pNode, m_real x, m_real y, m_real z)
			{OGRE_VOID(pNode->setScale(x,y,z));}
			static void setTransformation(Ogre::Node* pNode, transf const& t)
			{
#ifndef NO_OGRE
				setOrientation(pNode, t.rotation);
				setPosition(pNode, t.translation);
#endif
			}
			static void translateGlobal(Ogre::Node* pNode,vector3 const& tt)
			{
				OGRE_VOID(pNode->translate(ToOgre(tt), Ogre::Node::TS_WORLD)); // both TS_LOCAL and TS_WORLD are buggy apparently, but for speed, I only reimplmented rotateGlobal and translateGlobal functions correctly.
			}
			static void rotateGlobal(Ogre::Node* pNode, quater const& tt)
			{
				//OGRE_VOID(pNode->rotate(ToOgre(tt), Ogre::Node::TS_WORLD)); // both TS_LOCAL and TS_WORLD are buggy apparently, but for speed, I only reimplmented rotateGlobal and translateGlobal functions correctly.
#ifndef NO_OGRE
				transf t(ToBase(pNode->getOrientation()), ToBase(pNode->getPosition()));
				t.leftMult(transf(tt, vector3(0,0,0)));
				setTransformation(pNode, t);
#endif
			}
			static void setScale(Ogre::Node* pNode, vector3 const& t)
			{OGRE_VOID(pNode->setScale(t.x,t.y,t.z));}
			static void setScaleAndPosition(Ogre::Node* pNode, vector3 const& s, vector3 const& t)
			{OGRE_VOID(pNode->setScale(s.x,s.y,s.z);pNode->setPosition(t.x,t.y,t.z));}
			static void _setDerivedPosition(Ogre::Node* pNode, vector3 const& t)
			{ OGRE_VOID(pNode->_setDerivedPosition(ToOgre(t)));}
			static void _setDerivedOrientation(Ogre::Node* pNode, quater const& t)
			{ OGRE_VOID(pNode->_setDerivedOrientation(ToOgre(t)));}
			]],
			staticMemberFunctions={[[
			static void setOrientation(Ogre::Node* pNode, m_real w, m_real x, m_real y, m_real z)
			static void setOrientation(Ogre::Node* pNode, quater const& q)
			static quater getOrientation(Ogre::Node* pNode)
			static std::string getName(Ogre::Node* pNode)
			static void setPosition(Ogre::Node* pNode, m_real x, m_real y, m_real z)
			static void setPosition(Ogre::Node* pNode, vector3 const& t)
			static void _setDerivedPosition(Ogre::Node* pNode, vector3 const& t)
			static void _setDerivedOrientation(Ogre::Node* pNode, quater const& t)
			static vector3 getPosition(Ogre::Node* pNode)
			static vector3 _getDerivedPosition(Ogre::Node* pNode)
			static vector3 getScale(Ogre::Node* pNode)
			static void setScale(Ogre::Node* pNode, m_real x, m_real y, m_real z)
			static void setScale(Ogre::Node* pNode, vector3 const& t)
			static void setScaleAndPosition(Ogre::Node* pNode, vector3 const& s, vector3 const& t)
			static void setTransformation(Ogre::Node* pNode, transf const& t)
			static void translateGlobal(Ogre::Node* pNode,vector3 const& tt)
			static void rotateGlobal(Ogre::Node* pNode, quater const& tt)
			]]},
			memberFunctions=[[
			Ogre::Node* getParent(void) const;
			]]
		},
		{ 
			name='Ogre.Bone',
			ifndef='NO_GUI',
			inheritsFrom='Ogre::Node',
			memberFunctions=[[
				void setManuallyControlled(bool manuallyControlled);
			]]
		},
		{ 
			name='Ogre.SceneNode',
			inheritsFrom='Ogre::Node',
			wrapperCode=[[ static void resetToInitialState(Ogre::SceneNode* pNode)
			{OGRE_VOID(pNode->resetToInitialState());}
			static void removeAndDestroyChild(Ogre::SceneNode* pNode, const char* name)
			{OGRE_VOID(pNode->removeAndDestroyChild(name));}
			static void showBoundingBox(Ogre::SceneNode* node, bool bValue)
			{
#ifndef NO_OGRE
			if(node->getName()!="BackgroundNode")
			{
				Ogre::Node::ChildNodeIterator it=node->getChildIterator();

				while(it.hasMoreElements())
				{
					Ogre::Node* childnode=it.getNext();
					Ogre::SceneNode* childsceneNode;
					childsceneNode=dynamic_cast<Ogre::SceneNode*>(childnode);
					if(childsceneNode)
						showBoundingBox(childsceneNode, bValue);
				};
				node->showBoundingBox(bValue);
			}
#else 
					return ;
#endif

			}
			static Ogre::SceneNode* createChildSceneNode(Ogre::SceneNode* pNode, const char* name)
			{OGRE_PTR(return pNode->createChildSceneNode(name));}
			static Ogre::SceneNode* createChildSceneNode2(Ogre::SceneNode* pNode)
			{OGRE_PTR(return pNode->createChildSceneNode());}
			static void translate(Ogre::SceneNode* pNode, vector3 const& t)
			{OGRE_VOID(pNode->translate(t.x, t.y, t.z));}
			static void rotate(Ogre::SceneNode* pNode, quater const& t)
			{OGRE_VOID(pNode->rotate(ToOgre(t)));}
			static void translate(Ogre::SceneNode* pNode, m_real x, m_real y, m_real z)
			{OGRE_VOID(pNode->translate(x, y, z));}
			static void scale(Ogre::SceneNode* pNode, vector3 const& t)
			{OGRE_VOID(pNode->scale(t.x, t.y, t.z));}
			static void scale(Ogre::SceneNode* pNode, m_real x, m_real y, m_real z)
			{OGRE_VOID(pNode->scale(x, y, z));}
			static void scale(Ogre::SceneNode* pNode, m_real x)
			{OGRE_VOID(pNode->scale(x, x, x));}

			static void setPosition(Ogre::SceneNode* pNode, m_real x, m_real y, m_real z)
			{OGRE_VOID(pNode->setPosition(x,y,z));}
			static void setPosition(Ogre::SceneNode* pNode, vector3 const& t)
			{OGRE_VOID(pNode->setPosition(t.x,t.y,t.z));}
			static void setScale(Ogre::SceneNode* pNode, m_real x, m_real y, m_real z)
			{OGRE_VOID(pNode->setScale(x,y,z));}
			static void setScale(Ogre::SceneNode* pNode, vector3 const& t)
			{OGRE_VOID(pNode->setScale(t.x,t.y,t.z));}
			static void setScaleAndPosition(Ogre::SceneNode* pNode, vector3 const& s, vector3 const& t)
			{OGRE_VOID(pNode->setScale(s.x,s.y,s.z);pNode->setPosition(t.x,t.y,t.z));}
			static void setOrientation(Ogre::SceneNode* pNode, m_real w, m_real x, m_real y, m_real z)
			{OGRE_VOID(pNode->setOrientation(w, x,y,z));}
			static void setOrientation(Ogre::SceneNode* pNode, quater const& q)
			{OGRE_VOID(pNode->setOrientation(q.w, q.x,q.y,q.z));}
			]],
			staticMemberFunctions={[[
			static void resetToInitialState(Ogre::SceneNode* pNode)
			static void removeAndDestroyChild(Ogre::SceneNode* pNode, const char* name)
			static Ogre::SceneNode* createChildSceneNode2(Ogre::SceneNode* pNode) @ createChildSceneNode
			static Ogre::SceneNode* createChildSceneNode(Ogre::SceneNode* pNode, const char* name) 
			static void translate(Ogre::SceneNode* pNode, vector3 const& t)
			static void rotate(Ogre::SceneNode* pNode, quater const& t)
			static void translate(Ogre::SceneNode* pNode, m_real x, m_real y, m_real z)
			static void scale(Ogre::SceneNode* pNode, vector3 const& t)
			static void scale(Ogre::SceneNode* pNode, m_real x, m_real y, m_real z)
			static void scale(Ogre::SceneNode* pNode, m_real x)
			static void showBoundingBox(Ogre::SceneNode* node, bool bValue)
static void setPosition(Ogre::SceneNode* pNode, m_real x, m_real y, m_real z)
			static void setPosition(Ogre::SceneNode* pNode, vector3 const& t)
			static void setScale(Ogre::SceneNode* pNode, m_real x, m_real y, m_real z)
			static void setScale(Ogre::SceneNode* pNode, vector3 const& t)
			static void setScaleAndPosition(Ogre::SceneNode* pNode, vector3 const& s, vector3 const& t)
			static void setOrientation(Ogre::SceneNode* pNode, m_real w, m_real x, m_real y, m_real z)
			static void setOrientation(Ogre::SceneNode* pNode, quater const& q)

			Ogre::Entity* RE::getEntity(Ogre::SceneNode* node) @ getEntity
			]]},
			memberFunctions={[[
			void attachObject(Ogre::MovableObject* ); @ ;ifndef=NO_OGRE;
			void attachObject(OBJloader::MeshEntity* ); @ ;ifndef=NO_OGRE;
			int numAttachedObjects(); @ ;ifndef=NO_OGRE;
			void setVisible(bool visible)@ ;ifndef=NO_OGRE;
			void flipVisibility()@ ;ifndef=NO_OGRE;
			]]}
		},
		{ 
			name='Ogre.SceneManager',
			decl='namespace Ogre { class Light; }',
			wrapperCode=[[
			static void setFog(Ogre::SceneManager* pmgr, double r, double g, double b, double a, double min, double max)
			{
#ifndef NO_OGRE
				pmgr->setFog(Ogre::FOG_LINEAR,Ogre::ColourValue(r,g,b), a, min, max);
#endif
			}
			static void setFogExponential(Ogre::SceneManager* pmgr, double r, double g, double b, double a, double min, double max)
			{
#ifndef NO_OGRE
				pmgr->setFog(Ogre::FOG_EXP2,Ogre::ColourValue(r,g,b), a, min, max);
#endif
			}
			static void setFogNone(Ogre::SceneManager* pmgr)
			{
#ifndef NO_OGRE
				pmgr->setFog(Ogre::FOG_NONE,Ogre::ColourValue(0,0,0), 0, 0, 0);
#endif
			}
			static Ogre::Entity* createEntity(Ogre::SceneManager* pmgr, const char* id, const char* mesh)
			{
#ifndef NO_OGRE
				BEGIN_OGRE_CHECK
					return pmgr->createEntity(id,mesh);
				END_OGRE_CHECK
#else 
					return NULL;
#endif
			}

			static Ogre::SceneNode* getSceneNode(Ogre::SceneManager* pmgr, const char* id)
			{
#ifndef NO_OGRE
				BEGIN_OGRE_CHECK
					return pmgr->getSceneNode(id);
				END_OGRE_CHECK
#else 
					return NULL;
#endif

			}

			static void showBoundingBox(Ogre::SceneNode* node, bool bValue)
			{
#ifndef NO_OGRE
			if(node->getName()!="BackgroundNode")
			{
				Ogre::Node::ChildNodeIterator it=node->getChildIterator();

				while(it.hasMoreElements())
				{
					Ogre::Node* childnode=it.getNext();
					Ogre::SceneNode* childsceneNode;
					childsceneNode=dynamic_cast<Ogre::SceneNode*>(childnode);
					if(childsceneNode)
						showBoundingBox(childsceneNode, bValue);
				};
				node->showBoundingBox(bValue);
			}
#else 
					return ;
#endif

			}
			static Ogre::Light* createLight(Ogre::SceneManager* pmgr, const char* id)
			{
#ifndef NO_OGRE
				BEGIN_OGRE_CHECK
					return pmgr->createLight(id);

				END_OGRE_CHECK
#else 
					return NULL;
#endif


			}
			static Ogre::Light* getLight(Ogre::SceneManager* pmgr, const char* id)
			{
#ifndef NO_OGRE
				BEGIN_OGRE_CHECK
					return pmgr->getLight(id);

				END_OGRE_CHECK
#else 
					return NULL;
#endif


			}
			static Ogre::Entity* getEntity(Ogre::SceneManager* pmgr, const char* id)
			{
#ifndef NO_OGRE
				BEGIN_OGRE_CHECK
					return pmgr->getEntity(id);
				END_OGRE_CHECK
#else 
					return NULL;
#endif
			}

			static void setAmbientLight(Ogre::SceneManager* pmgr, m_real x, m_real y, m_real z)
			{OGRE_VOID(pmgr->setAmbientLight(Ogre::ColourValue(x,y,z)));}
			static void setShadowColour(Ogre::SceneManager* pmgr, m_real x, m_real y, m_real z)
			{OGRE_VOID(pmgr->setShadowColour(Ogre::ColourValue(x,y,z)));}

			static void setSkyBox(Ogre::SceneManager* pmgr, bool enable, const char* materialName)
			{OGRE_VOID(pmgr->setSkyBox(enable, materialName));}
			static bool hasSceneNode(Ogre::SceneManager* pmgr, const char * name) 
			{
#if !defined (NO_GUI)                                         
				return pmgr->hasSceneNode(Ogre::String(name));
#else
				return true;
#endif
			}
			static void setRenderqueueOverlay(Ogre::SceneManager* pmgr,Ogre::Entity* roEntity, ushort groupID)
			{
				//Ogre::RenderQueue render_q;
				//Ogre::SceneManager::MovableObjectIterator iterator=
			//		pmgr->getMovableObjectIterator("Entity");
				//while(iterator.hasMoreElements())
				{
					//Ogre::Entity* e = static_cast<Ogre::Entity*>(iterator.getNext());
					//if(e->getName()!="testEntity")
					{
						//render_q.addRenderable(e,Ogre::RENDER_QUEUE_BACKGROUND);
						//e->getSubEntity(0)->getMaterial()->setDepthCheckEnabled(false);
						//e->getSubEntity(0)->getMaterial()->setDepthWriteEnabled(false);
						//e->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
					}
					//else
					{
						//render_q.addRenderable(e,Ogre::RENDER_QUEUE_MAX);
					}
					//printf("Entity Name: %s\n",e->getName().c_str());
					//printf("RenderQueueGroupID:%d \n",e->getRenderQueueGroup());
				}
				//roEntity->getSubEntity(0)->getMaterial()->setDepthCheckEnabled(false);
				//roEntity->getSubEntity(0)->getMaterial()->setDepthWriteEnabled(false);
				//roEntity->setRenderQueueGroup(groupID);
			}
			static int getShadowTechnique(Ogre::SceneManager* pmgr)
			{
#if !defined (NO_GUI)                                         
				return pmgr->getShadowTechnique();
#else
				return 18;
#endif
			}
			]],
			memberFunctions=[[
            void setShadowTextureSize(int size);
            void setShadowTextureCount(int count);
			]],
			staticMemberFunctions={[[
			static int getShadowTechnique(Ogre::SceneManager* pmgr)
			static void setFog(Ogre::SceneManager* pmgr, double r, double g, double b, double a, double min, double max)
			static void setFogExponential(Ogre::SceneManager* pmgr, double r, double g, double b, double a, double min, double max)
			static void setFogNone(Ogre::SceneManager* pmgr)
			static Ogre::Entity* createEntity(Ogre::SceneManager* pmgr, const char* id, const char* mesh)
			static void setAmbientLight(Ogre::SceneManager* pmgr, m_real x, m_real y, m_real z)
			static void setShadowColour(Ogre::SceneManager* pmgr, m_real x, m_real y, m_real z)
			static Ogre::SceneNode* getSceneNode(Ogre::SceneManager* pmgr, const char* id)
			static Ogre::Light* createLight(Ogre::SceneManager* pmgr, const char* id)
			static Ogre::Light* getLight(Ogre::SceneManager* pmgr, const char* id)
			static Ogre::Entity* getEntity(Ogre::SceneManager* pmgr, const char* id)
			static void setSkyBox(Ogre::SceneManager* pmgr, bool enable, const char* materialName)
			static bool hasSceneNode(Ogre::SceneManager* pmgr, const char * name) 
			static void showBoundingBox(Ogre::SceneNode* node, bool bValue)
			static void setRenderqueueOverlay(Ogre::SceneManager* pmgr,Ogre::Entity* roEntity, ushort groupID)
							  ]]},
			  memberFunctions={[[
			  ]]}
		},
		{
			name='Bone',
			wrapperCode=[[

			static bool isChildHeadValid(Bone& bone)   { return bone.m_pChildHead!=NULL;}
			static bool isChildTailValid(Bone& bone)   { return bone.m_pChildTail!=NULL;}
			static bool isSiblingValid(Bone& bone)   { return bone.m_pSibling!=NULL;}

			static Bone* childHead(Bone& bone)	{ return ((Bone*)bone.m_pChildHead);}
			static Bone* childTail(Bone& bone)	{ return ((Bone*)bone.m_pChildTail);}
			static Bone* sibling(Bone& bone)	{ return ((Bone*)bone.m_pSibling);}

			static void getTranslation(Bone& bone, vector3& trans)
			{
				bone.getTranslation(trans);
			}
			static vector3 getTranslation(Bone& bone)
			{
				return bone.getTranslation();
			}
			static vector3 getOffset(Bone& bone)
			{
				vector3 v;
				bone.getOffset(v);
				return v;
			}

			static void getRotation(Bone& bone, quater& q)
			{
				bone.getRotation(q);
			}
			static bool eq(Bone& bone1, Bone& bone2)
			{
				return &bone1==&bone2;
			}

			static std::string name(Bone& bone)
			{
				return std::string(bone.NameId);
			}
	]],
	read_properties={{'NameId', 'name'}},
			staticMemberFunctions={[[
			static bool isChildHeadValid(Bone& bone) 
			static bool isChildTailValid(Bone& bone)
			static bool isSiblingValid(Bone& bone) 
			static Bone* childHead(Bone& bone)	
			static Bone* childTail(Bone& bone)
			static Bone* sibling(Bone& bone)
			static void getTranslation(Bone& bone, vector3& trans)
			static vector3 getTranslation(Bone& bone)
			static vector3 getOffset(Bone& bone)
			static void getRotation(Bone& bone, quater& q)
			static bool eq(Bone& bone1, Bone& bone2) @ __eq
			static std::string name(Bone& bone)
			static std::string name(Bone& bone) @ __tostring
			]]},
			memberFunctions={[[
			int numChildren()
			void SetNameId(const char*) @ setName
			transf & _getOffsetTransform() ; @ getOffsetTransform
			transf & _getFrame() const; @ getFrame
			transf & _getLocalFrame() const; @ getLocalFrame
			m_real length() const;
			void getOffset(vector3& offset) const;
			vector3 axis(int ichannel);
			Bone* parent() const	
			Bone* sibling() const
			bool isDescendent(const Bone * parent) const;
			TString const& getRotationalChannels() const;
			TString const& getTranslationalChannels() const;
			void setChannels(const char* translation_axis, const char* rotation_axis);
			int numChannels() const;
			Bone* parent() const	
			int voca() const	
			MotionLoader const& getSkeleton() const
			int treeIndex() const		
			int rotJointIndex() const
			int transJointIndex() const
			]]}
		},
		{
			name='InterframeDifferenceC1',
	ctors={'(MotionDOF const& input)', '(m_real)'},
memberFunctions={[[
	void resize(int numFrames);
	int numFrames()	
	void initFromDeltaRep(vectorn const& start_transf, matrixn const& input);
	vectorn exportToDeltaRep(matrixn & output);
	void reconstruct(matrixn& output);	// output can be MotionDOF.
	static vectorn getTransformation(matrixn const& motionDOF, int iframe);
]]},
properties={
	'vector3 startPrevP',
	'vector3 startP',
	'quater startRotY',
	'm_real _frameRate',
	'vector3N dv',
	'vector3N dq',
	'vectorn offset_y',
	'quaterN offset_qy',
	'quaterN offset_q',
},
},
		{
			name='InterframeDifference',
	ctors={'(MotionDOF const& input)', '()'},
properties={
	'vector3 startP',
	'quater startRotY',
	'vector3N dv',
	'vector3N dq',
	'vectorn offset_y',
	'quaterN offset_q',
},
memberFunctions={[[
	void resize(int numFrames);
	int numFrames()
	void initFromDeltaRep(vector3 const& start_transf, matrixn const& input);
	vector3 exportToDeltaRep(matrixn & output);
	void reconstruct(matrixn& output, m_real frameRate);
]]}
		},
		{
			name='MotionDOF',
			inheritsFrom='matrixn',
			ctors={ "(const MotionDOFinfo&)", "(const MotionDOF&)"},
			properties={	"MotionDOFinfo mInfo @ dofInfo"},
			staticMemberFunctions={
				[[
				static transf MotionDOF::rootTransformation(vectorn const& pose);
				static void MotionDOF::setRootTransformation(vectorn & pose, transf const& t);	
				]]
			},

			memberFunctions={
				[[
				void operator=(const MotionDOF& other);
				void operator=(const MotionDOF& other); @ copyFrom
				void operator=(const MotionDOF& other); @ assign 
				matrixnView _matView(); @ matView
				int numFrames()	const			
				int numDOF()	const	
				void resize(int numFrames)	
				void changeLength(int length)
				transf rootTransformation(int i) const;
				void setDOFinfo(MotionDOFinfo const& info);
				void get(Motion& tgtMotion);
				void set(const Motion& srcMotion); 
				void set(const Motion& srcMotion, intvectorn const& shoulder_tree_indices, intvectorn const& elbow_tree_indices, intvectorn const& wrist_tree_indices); 
				MotionDOFview range(int start, int end);	// [start, end)
				MotionDOFview range_c(int first, int last);	// [first, last] (closed interval)
				void samplePose(m_real criticalTime, vectorn& out) const;
				void stitch(MotionDOF const& motA, MotionDOF const& motB);
				void align(MotionDOF const& motA, MotionDOF const& motB);
				void stitchDeltaRep(MotionDOF const& motA, MotionDOF const& motB);
				vectornView row(int i); @ __call
				vector3 convertToDeltaRep();	// see interframeDifference
				void generateID(vector3 const& start_transf, InterframeDifference& out) const;
				void reconstructData(vector3 const & startTransf);
				void reconstructData(vector3 const& startTransf, matrixn& out) const;
				void reconstructData(transf const& startTransf, matrixn& out) const;
				void reconstructOneFrame(vector3 const& startTransf, vectorn const& deltaPose, vectorn & outpose) const;
				vectornView dv_x() const	
				vectornView dv_z() const	
				vectornView dq_y() const	
				vectornView offset_y() const
				int length() const
				void transform(transf const& t)
				void scale(double t)
				void calcForwardDerivative(int i, vectorn & dpose, double frameRate) const;
				]]
			}
		},
		{ 
			name='MotionDOFview',
inheritsFrom='MotionDOF'
		},
		{
			name='BoneForwardKinematics',
ctors={'(MotionLoader*)'},
wrapperCode=[[
			static transf& localFrame(BoneForwardKinematics& fk, int i){ return fk._local(i);}
			static transf& localFrame(BoneForwardKinematics& fk, Bone& bone){ return fk._local(bone);}
			static transf& globalFrame(BoneForwardKinematics& fk, int i){ return fk._global(i);}
			static transf& globalFrame(BoneForwardKinematics& fk, Bone& bone){return fk._global(bone);}
]],
staticMemberFunctions={[[
			transf& localFrame(BoneForwardKinematics& fk, int i)
			transf& localFrame(BoneForwardKinematics& fk, Bone& bone)
			transf& globalFrame(BoneForwardKinematics& fk, int i)
			transf& globalFrame(BoneForwardKinematics& fk, Bone& bone)
]]},
memberFunctions={[[
	void init();
	void forwardKinematics();
	void inverseKinematics();
	void updateBoneLength(MotionLoader const& loader);
	void operator=(BoneForwardKinematics const& other);
	void setPose(const Posture& pose);
	void setPoseDOF(const vectorn& poseDOF);
	void setChain(const Posture& pose, const Bone& bone);
	void getPoseFromGlobal(Posture& pose) const;
	void getPoseDOFfromGlobal(vectorn& poseDOF) const;
	void getPoseFromLocal(Posture& pose) const;
	void getPoseDOFfromLocal(vectorn& poseDOF) const;
	MotionLoader const& getSkeleton() const		
]]}
		},
		{ 
			name='FrameSensor'
		},
		{
			name='FrameMoveObject'
		},
		{
			name='AnimationObject',
			inheritsFrom='FrameMoveObject',
			properties={'vector3 m_vTrans',},
		},
		{
			name='LuaAnimationObject',
			className='LuaAnimationObject',
			decl='class LuaAnimationObject;',
			isLuaInheritable=true,
			inheritsFrom='AnimationObject',
			globalWrapperCode=[[
			class LuaAnimationObject: public AnimationObject, public luna_wrap_object
			{
				public:
				LuaAnimationObject()
				:AnimationObject() { }

				virtual ~LuaAnimationObject() { }

				virtual int FrameMove(float fElapsedTime)  {
					lunaStack l(_L);
					if(pushMemberFunc<LuaAnimationObject>(l,"frameMove")){
						l<<(double)fElapsedTime;
						l.call(2,0);
					} 
					return AnimationObject::FrameMove(fElapsedTime);
				}
				void attachTimer(float frameTime, int numFrames)
				{
					if (m_pTimer) delete m_pTimer;
					InterpolatorLinear * pInterpolator=new InterpolatorLinear();
					pInterpolator->init(frameTime, numFrames);
					m_pTimer=new TimeSensor();
					m_pTimer->AttachInterpolator(pInterpolator);
				}
			};
			]],
			ctors={'()'},
			memberFunctions=[[
				void attachTimer(float frameTime, int numFrames)
			]]
		},
		{
			name='Ogre.ObjectList',
			className='ObjectList',
			ctors={'()'},
			memberFunctions={[[
			void clear();
			void setVisible(bool bVisible);
			void drawSphere(vector3 const& pos, const char* nameid, const char* materialName, double scale);
			void drawSphere(vector3 const& pos, const char* nameid);
			void drawAxes(transf const& tf, const char* nameid);
			void drawAxes(transf const& tf, const char* nameid, double scale);
			Ogre::SceneNode* registerEntity(const char* node_name, const char* filename);
			Ogre::SceneNode* registerEntity(const char* node_name, const char* filename, const char* materialName);
			Ogre::SceneNode* registerEntity(const char* node_name, Ogre::Entity* pObject);
			Ogre::SceneNode* registerObject(const char* node_name, Ogre::MovableObject* pObject);
			Ogre::SceneNode* registerObject(const char* node_name, const char* typeName, const char* materialName, matrixn const& data);
			Ogre::SceneNode* registerObject(const char* node_name, const char* typeName, const char* materialName, matrixn const& data, m_real thickness);
			Ogre::SceneNode* registerEntityScheduled(const char* filename, m_real destroyTime);
			Ogre::SceneNode* registerObjectScheduled(Ogre::MovableObject* pObject, m_real destroyTime);
			Ogre::SceneNode* registerObjectScheduled(m_real destroyTime, const char* typeName, const char* materialName, matrixn const& data, m_real thickness);
			Ogre::SceneNode* findNode(const char* node_name);
			void erase(const char* node_name);
			void eraseAllScheduled();
			Ogre::SceneNode* createSceneNode(const char* node_name);
			Ogre::SceneNode* getCurrRootSceneNode() const; // do not cache this value. This can change over time.
			]]},
		},
		{ 

			name='PLDPrimSkin',
			inheritsFrom='AnimationObject',
			read_properties={{"visible", "getVisible"}},
			write_properties={{"visible", "setVisible"}},
			wrapperCode=[[ 
			static void startAnim(PLDPrimSkin& s)
			{
				s.m_pTimer->StartAnim();
			}
			static void initAnim(PLDPrimSkin& s, float curframe, float endframe, float returnframe)
			{
				s.m_pTimer->InitAnim(curframe, endframe, returnframe);
			}
			static void stopAnim(PLDPrimSkin& s)
			{
				s.m_pTimer->StopAnim();
			}
			static void loop(PLDPrimSkin& s, bool b)
			{
				s.m_pTimer->loop(b);
			}
			static bool isPlaying(PLDPrimSkin& s)
			{
				return s.m_pTimer->IsPlaying();
			}
			static float calcCurFrameFromInterpolator(PLDPrimSkin& s, int iframe)
			{
				return s.m_pTimer->calcCurFrameFromInterpolator(iframe);
			}
			static int numFrames(PLDPrimSkin& s)
			{
				return s.m_pTimer->getNumFrameFromInterpolator();
			}
			static void setFrameTime(PLDPrimSkin& s, float fFrameTime)
			{
				InterpolatorLinear* apip=((InterpolatorLinear*)(s.m_pTimer->GetFirstInterpolator()));
				apip->init(fFrameTime);
			}
			static float totalTime(PLDPrimSkin& s)
			{
				return s.m_pTimer->getTotalTimeFromInterpolator();
			}
			static void setRotation(PLDPrimSkin& s,quater const& q)
			{
				#ifndef NO_OGRE
				s.m_pSceneNode->setOrientation(ToOgre(q));
				#endif
			}
			static quater getRotation(PLDPrimSkin& s)
			{
				#ifndef NO_OGRE
				return ToBase(s.m_pSceneNode->getOrientation());
				#else
				return quater(1,0,0,0);
				#endif
			}
			]],
			staticMemberFunctions={
				[[
				static void loop(PLDPrimSkin& s, bool b)
				static void startAnim(PLDPrimSkin& s)
				static void initAnim(PLDPrimSkin& s, float curframe, float endframe, float returnframe)
				static void stopAnim(PLDPrimSkin& s)
				static void setRotation(PLDPrimSkin& s,quater const& q)
				static quater getRotation(PLDPrimSkin& s)
				static bool isPlaying(PLDPrimSkin& s)
				static float calcCurFrameFromInterpolator(PLDPrimSkin& s, int iframe)
				static int numFrames(PLDPrimSkin& s)
				static void setFrameTime(PLDPrimSkin& s, float fFrameTime)
				static float totalTime(PLDPrimSkin& s)
				]]
			},
			enums={
				{"BLOB","(int)RE::PLDPRIM_BLOB"},
				{"LINE","(int)RE::PLDPRIM_LINE"},
				{"SKIN","(int)RE::PLDPRIM_SKIN"},
				{"POINT","(int)RE::PLDPRIM_POINT"},
				{"CYLINDER","(int)RE::PLDPRIM_CYLINDER"},
				{"CYLINDER_LOWPOLY","(int)RE::PLDPRIM_CYLINDER_LOWPOLY"},
				{"BOX","(int)RE::PLDPRIM_BOX"}
			},
			memberFunctions={
				[[
				void updateBoneLength(MotionLoader const& loader)
				void setPose(const Motion& mot, int iframe)
				void setPose(int iframe)
				void SetPose(const Posture & posture, const MotionLoader& skeleton) @ setPose
				// use _setPose and _setPoseDOF for new codes
				void SetPose(const Posture & posture, const MotionLoader& skeleton) @ _setPose
				void setPoseDOF(const vectorn& poseDOF, MotionDOFinfo const& info); @ _setPoseDOF

				bool GetVisible() const; @ getVisible
				void SetVisible(bool bVisible); @ setVisible
				void ApplyAnim(const Motion& mot); @ applyAnim
				void applyAnim(const MotionDOF& motion); @ applyMotionDOF
				void setThickness(float thick){}
				void scale(double x, double y, double z);
				void setScale(double x, double y, double z);
				  void SetTranslation(double x, double y, double z); @ setTranslation
				  vector3 const&  getTranslation() const; 
				  void setMaterial(const char* mat)
				void setDrawOrientation(int ijoint)
				]]
			}
		},
		{
			name='PLDPrimVRML',
			inheritsFrom='PLDPrimSkin',
			memberFunctions={[[
			void setPoseDOF(const vectorn& poseDOF);
			void setPose(BoneForwardKinematics const& in) @ setSamePose
			void setMaterial(int, const char*); @ setBoneMaterial
			void getPose(Posture & posture); @ getPose
			]]},
			wrapperCode=[[
						inline static PLDPrimSkin* downcast(PLDPrimVRML* skin)
						{
							return (PLDPrimSkin*) (skin);
						}
						]],
			staticMemberFunctions={[[
						static PLDPrimSkin* downcast(PLDPrimVRML* skin)
						]]},
		},
		{
			name='PLDPrimOgreSkin',
			inheritsFrom='PLDPrimSkin',
			properties={
				'quaterN m_aLocalRotOrig',
				'quaterN m_aRotOrigComb',
				'quaterN m_aInvRotOrigComb',
				'quaterN m_aBindPose',
			},
			memberFunctions={[[
				int getOgreBoneIndex(int ibone) const;
				int parentRotJointIndex(int iRotJoint) const;
			]]},
		},
		{
			name='Pose',
			className='Posture',
			ctors={'()', '(const Posture&)'},
			properties={
				'vector3N m_aTranslations; @ translations',
				'quaterN m_aRotations; @ rotations',
				'vector3 m_dv;',
				'quater m_dq;',
				'm_real m_offset_y;',
				'quater m_offset_q;',
				'quater m_rotAxis_y;',
			},
			memberFunctions={[[
				void Init(int numRotJoint, int numTransJoint); @ init
				void identity();
				int numRotJoint() const	
				int numTransJoint() const	
				Posture* clone() const; @ ;adopt=true;
				void Clone(const Posture* pPosture) @ assign
				void Blend(const Posture& a, const Posture& b, m_real t); @ blend	//!< a*(1-t)+b*(t)
				void Blend(const Posture& b, m_real t) @ blend
				void Align(const Posture& other); @ align
				vector3 front();
				void decomposeRot() const;
				transf rootTransformation() const;
				void setRootTransformation(transf const& rootTransf);
			]]}
		},
		{ 
			decl=[[
				bool OIS_event_ctrl();
				bool OIS_event_shift();
				bool OIS_event_alt();
			]],
			name='FltkRenderer',
			wrapperCode=
			[[
				static void onCallback(FltkRenderer* mp, const char* text)
				{
					mp->onCallback(NULL,Hash(text));
				}
			]],
			memberFunctions={
				[[
				vector3 screenToWorldXZPlane(float x, float y, float height);
				vector3 screenToWorldXZPlane(float x, float y );
				void screenToWorldRay(float x, float y, Ray& ray) const;
				void volumeQuery(TStrings& nodeNames, float left, float top, float right, float bottom);@;ifndef=NO_OGRE;
				void rayQuery(TStrings& nodeNames, float x, float y);@;ifndef=NO_OGRE;
				void saveView(int slot);
				void changeView(int slot);
				void changeViewNoAnim(int slot);

			]]},
			staticMemberFunctions={[[
				static void onCallback(FltkRenderer* mp, const char* text)
				bool OIS_event_ctrl();
				bool OIS_event_shift();
				bool OIS_event_alt();
			]]}
		},
		{
			name='Planes',
			className='std::vector<Plane>',
			ctors={'()'},
			memberFunctions={[[
			void resize(int)
			Plane& operator[](int i) @ at
			Plane& operator[](int i) @ __call
			int size()
			]]}
		},
		{
			decl='class Ray;',
			name='Ray',
			ctors={'()', '(const vector3& origin, const vector3& direction)'},
			memberFunctions=[[
				vector3 origin() const		
				vector3 direction() const	
				vector3 getPoint(m_real t) const
				void translate(vector3 const& t)
				]],
			wrapperCode=[[
			inline static vectorn intersects(Ray & r, const Plane& pl) 
			{
				vectorn v(2);
				std::pair<bool,m_real> p=r.intersects(pl);
				v(0)=(p.first)?1:0;
				v(1)=p.second;
				return v;
			}
			inline static vectorn intersects(Ray & r, const std::vector<Plane>& pl) 
			{
				vectorn v(2);
				std::pair<bool,m_real> p=r.intersects(pl);
				v(0)=(p.first)?1:0;
				v(1)=p.second;
				return v;
			}
			inline static vectorn intersects(Ray & r, const Sphere& pl) 
			{
				vectorn v(2);
				std::pair<bool,m_real> p=r.intersects(pl);
				v(0)=(p.first)?1:0;
				v(1)=p.second;
				return v;
			}
			]],
			staticMemberFunctions=[[
			static vectorn intersects(Ray & r, const Plane& p) 
			static vectorn intersects(Ray & r, const Sphere& p) 
			]],
		},
		{
			decl='class Plane;',
			name='Plane',
			ctors=[[
				()
				(const vector3& vNormal, m_real offset)
				(const vector3& vNormal, const vector3& vPoint)
				(const vector3& vPoint0, const vector3& vPoint1, const vector3& vPoint2)
				]],
			properties={'vector3 normal', 'double d'},
			memberFunctions=[[
				m_real distance (const vector3& point) const;
				void setPlane(const vector3& vPoint0, const vector3& vPoint1, const vector3& vPoint2);
				void setPlane(const vector3& vNormal, const vector3& vPoint);
				]]
		},
		{
			decl='class Sphere;',
			name='Sphere',
			ctors=[[(vector3 c, m_real r)]],
			properties={'vector3 center', 'double radius'},
		},
		{ 
			name='Motion',
			ctors={
				'()',
				'(MotionLoader*)',
				'(const Motion&, int,int)',
				'(const MotionDOF& srcMotion)',
				'(const MotionDOF& srcMotion, int startFrame, int endFrame)',
			},
			wrapperCode=[[
				static void initFromFile(Motion& motion, const char* fn)
				{
					motion.Init(RE::renderer().m_pMotionManager->GetMotionLoaderPtr(fn));
				}

				static void initSkeletonFromFile(Motion& motion, const char* fn)
				{
					motion.InitSkeleton(RE::renderer().m_pMotionManager->GetMotionLoaderPtr(fn));
				}

				static void concatFromFile(Motion& motion, const char* fn)
				{
					RE::motion::concatFromFile(motion, fn);
				}
				static void scale(Motion& motion, m_real fScale)
				{
					if (&motion.skeleton().m_cPostureIP==&motion)
						motion.skeleton().Scale(fScale);
					else
						motion.skeleton().scale(fScale, motion);
				}
				static void calcInterFrameDifference(Motion& motion)
				{
					motion.CalcInterFrameDifference(0);
				}

				static void translate(Motion& motion, vector3 const& t)
				{
					MotionUtil::translate(motion, t);
				}

				static void smooth(Motion& motion, float kernelRoot, float kernelJoint)
				{
					Motion copy=motion;
					MotionUtil::smooth(motion, copy, kernelRoot, kernelJoint);
				}
			]],
			enums={
				{ "IS_DISCONTINUOUS","(int)IS_DISCONTINUOUS" },
				{ "CONSTRAINT_LEFT_FOOT","(int)CONSTRAINT_LEFT_FOOT"},
				{ "CONSTRAINT_RIGHT_FOOT","(int)CONSTRAINT_RIGHT_FOOT"},
				{ "CONSTRAINT_LEFT_HAND","(int)CONSTRAINT_LEFT_HAND"},
				{ "CONSTRAINT_RIGHT_HAND","(int)CONSTRAINT_RIGHT_HAND"},
				{"LOCAL_COORD","(int)Motion::LOCAL_COORD"},
				{"GLOBAL_COORD","(int)Motion::GLOBAL_COORD"},
				{"FIXED_COORD","(int)Motion::FIXED_COORD"},
				{"FIRST_ARRANGED_COORD","(int)Motion::FIRST_ARRANGED_COORD"},
				{"PELVIS_LOCAL_COORD","(int)Motion::PELVIS_LOCAL_COORD"},
			},
			memberFunctions={[[
			int length() const
			void changeLength(int length)	
			void Resize(int size) @ resize
			void empty();
			void setPose(int iframe, const Posture& pose);
			void setSkeleton(int iframe) const;
			MotionLoader& skeleton() const
			int numRotJoints() const	
			int numTransJoints()	const	
			int numFrames() const
			int size() const		
			void ChangeCoord(int eCoord); @ changeCoord
			MotionView range(int start, int end);
			void Init(const Motion& srcMotion, int startFrame, int endFrame); @ init
			void Init(MotionLoader* pSource) @ init
			void InitEmpty(MotionLoader* pSource, int numFrames); @ initEmpty
			void InitEmpty(MotionLoader* pSource, int numFrames, float fFrameTime); @ initEmpty
			void InitEmpty(const Motion& source, int numFrames); @ initEmpty
			void InitSkeleton(MotionLoader* pSource); @ initSkeleton
			bool isConstraint(int fr, int eConstraint) const;
			void setConstraint(int fr, int con, bool bSet);
			void SetIdentifier(const char* id) @ setIdentifier
			const char* GetIdentifier() const @ getIdentifier
			void Concat(const Motion* pAdd, int startFrame, int endFrame) @ concat
			void Concat(const Motion* pAdd) @ concat
			float totalTime() const				{ return mInfo.m_fFrameTime*length();}
			int frameRate() const				{ return int(1.f/frameTime()+0.5f);}
			void frameTime(float ftime)		@ setFrameTime
			bool isDiscontinuous(int fr) const;//			{ return m_aDiscontinuity[fr%m_maxCapacity];}
			void setDiscontinuity(int fr, bool value);//	{ m_aDiscontinuity.setValue(fr%m_maxCapacity, value);}
			Posture& pose(int iframe) const;
			void CalcInterFrameDifference(int startFrame ); @ calcInterFrameDifference
			void ReconstructDataByDifference(int startFrame ); @ reconstructFromInterFrameDifference
			void exportMOT(const char* filename) const; @ exportMot
			void samplePose(Posture& pose, m_real criticalTime) const;
			]]},
			staticMemberFunctions={[[
			static void initSkeletonFromFile(Motion& motion, const char* fn)
			static void initFromFile(Motion& motion, const char* fn)
			static void concatFromFile(Motion& motion, const char* fn)
			static void scale(Motion& motion, m_real fScale)
			static void calcInterFrameDifference(Motion& motion)
			static void translate(Motion& motion, vector3 const& t)
			static void smooth(Motion& motion, float kernelRoot, float kernelJoint)
			static void MotionUtil::mirrorMotion(Motion& out, const Motion& in, intvectorn const& LrootIndices, intvectorn const& RrootIndices) @ mirrorMotion
			]]}
		},
		{
			name='MotionView',
			inheritsFrom='Motion'
		},
{
name='FltkMotionWindow',
ifndef='NO_GUI',
memberFunctions={[[
	void addSkin(AnimationObject* pSkin);
	void releaseAllSkin();
	void releaseSkin(AnimationObject* pSkin);
	void detachAllSkin();
	void detachSkin(AnimationObject* pSkin);
	void relaseLastAddedSkins(int nskins);

	int getCurrFrame()				
	int getNumFrame()			
	int getNumSkin()		
	AnimationObject* getSkin(int index)
	void changeCurrFrame(int iframe);
	int playUntil(int iframe);
	int playFrom(int iframe);
]]}
},
{
name='FltkScrollPanel',
ifndef='NO_GUI',
memberFunctions={[[
	void addPanel(const char* filename);
	void addPanel(CImage* pSource);	//!< image will be copyed. You should release pSource
	CImage* createPanel();		//! Create a dynamic panel(you can edit the content of this panel). return empty image. you can create the image by calling CImage::Create(...)
	void addPanel(const boolN& bits, CPixelRGB8 color);
	void addPanel(const boolN& bits, CPixelRGB8 color, int startFrame);
	void addPanel(const intvectorn& indexes);
	void addPanel(const vectorn& signal);
	void addPanel(const vectorn& input, double fmin, double fmax);
	void addPanel(const matrixn& signal);
	void setLabel(const char* label);
	void changeLabel(const char* prevLabel, const char* newLabel);
	const char* selectedPanel()	
	void removeAllPanel();
	void sortPanels();
	void setLastPanelXOffset(int xoffset);
	void removePanel(CImage* pImage);	//! Remove a dynamic panel.
	void changeXpos(CImage* pImage, int xpos)
	void setCutState(const boolN& abCutState) 
	const boolN& cutState()	
	void redraw()
	int currFrame() const
]]}
},
{
ifndef='NO_GUI',
name='Loader',
inheritsFrom='LUAwrapper::Worker',
},
       
		{ 
			name='MotionPanel',
			ifndef='NO_GUI',
			wrapperCode=
			[[
				static void onCallback(MotionPanel* mp, const char* text)
				{
					mp->onCallback(NULL,Hash(text));
				}
			]],
			memberFunctions={[[
				FltkMotionWindow* motionWin()	{ return m_motionWin;}
				FltkScrollPanel* scrollPanel()	{ return m_scrollPanel;}
				Loader* loader()				{ return m_loader;}
				Motion& currMotion();
				Motion& currPairMotion();
				MotionDOF& currMotionDOF();
				MotionDOFcontainer& currMotionDOFcontainer();
				bool hasMotionDOF();
				bool hasPairMotion();
				void registerMotion(Motion const& mot);
				void registerMotion(MotionDOF const& mot);
				void releaseMotions();
				void redraw();
				int numMotion();
				Motion& motion(int i) const
				MotionDOF& motiondof(int i) const	
			]]},
			staticMemberFunctions={[[
				static void onCallback(MotionPanel* mp, const char* text)
			]]}
		},
		{ 
			name='Viewpoint',
			wrapperCode=
			[[
			inline static void update(Viewpoint & view)
			{
				view.CalcHAngle();
				view.CalcVAngle();
				view.CalcDepth();
			}

			inline static void setClipDistances(Viewpoint& view, m_real fnear, m_real ffar)
			{
#ifndef NO_OGRE

				double nn=fnear;
				double ff=ffar;
//#ifdef OCULUS // do this outside of this function. This won't work anymore.
//				for (int i=0,n=RE::renderer().numViewport();i<n; i++)
//				{
//					RE::renderer().viewport(i).mCam->setNearClipDistance(Ogre::Real(fnear));
//					RE::renderer().viewport(i).mCam->setFarClipDistance(Ogre::Real(ffar));
//				}
//#else
				RE::renderer().viewport().mCam->setNearClipDistance(Ogre::Real(fnear));
				RE::renderer().viewport().mCam->setFarClipDistance(Ogre::Real(ffar));
//#endif
#endif
			}
			inline static void setOrthographicMode(Viewpoint& view, bool isOrtho)
			{
#ifndef NO_OGRE
				RE::renderer().viewport().setOrthographicMode(isOrtho);
#endif
			}
			inline static void setDimensions(Viewpoint& view, double left, double top, double width, double height)
			{
#ifndef NO_OGRE
				RE::renderer().viewport().mView->setDimensions(left,top, width, height);
#endif
			}
			inline static void setFOVy(Viewpoint& view, m_real degree)
			{
#ifndef NO_OGRE

//#ifdef OCULUS // do this outside of this function. This won't work anymore.
//				for (int i=0,n=RE::renderer().numViewport();i<n; i++)
//					{
//						printf("%f\n", degree);
//				RE::renderer().viewport(i).mCam->setFOVy(Ogre::Radian(Ogre::Degree(degree)));
//			}
//#else
				RE::renderer().viewport().mCam->setFOVy(Ogre::Radian(Ogre::Degree(degree)));
//#endif
#endif
			}
			inline static void setNearClipDistance(Viewpoint& view, m_real dist)
			{
#ifndef NO_OGRE
//#ifdef OCULUS // do this outside of this function. This won't work anymore.
//
//				for (int i=0,n=RE::renderer().numViewport();i<n; i++)
//				{
//					RE::renderer().viewport(i).mCam->setNearClipDistance(Ogre::Real(dist));
//				}
//#else
				RE::renderer().viewport().mCam->setNearClipDistance(dist);
//#endif
#endif
			}

			]],
			properties={'vector3 m_vecVPos @ vpos', 'vector3 m_vecVAt @ vat'},
			staticMemberFunctions={
				[[
			void update(Viewpoint & view)
			void setClipDistances(Viewpoint& view, m_real fnear, m_real ffar)
			void setFOVy(Viewpoint& view, m_real degree)
			void setNearClipDistance(Viewpoint& view, m_real dist)
			void setDimensions(Viewpoint& view,double left, double top, double width, double height)
			void setOrthographicMode(Viewpoint& view, bool isOrtho)
				]]
				},
			memberFunctionsFromFile={
				source_path..'/../../BaseLib/motion/viewpoint.h',
				"setScale",
				{"updateVPosFromVHD", "UpdateVPosFromVHD",},
				"TurnRight",
				"TurnLeft",
				"TurnUp",
				"TurnDown",
				"ZoomIn",
				"ZoomOut",
				"getZoom",
				"setZoom",
				{"assign", "operator="}, -- {luaname, cppname}
			},
			memberFunctions=[[
			int GetViewMatrix(matrix4& matView);
			int SetViewMatrix(matrix4 const& matView);
			void setYUp();
			void setZUp();
			]]
		},
		{
			name='OgreRenderer.Viewport',
			wrapperCode=[[
			static void setBackgroundColour(OgreRenderer::Viewport & v,CPixelRGB8 c)
			{
#ifndef NO_OGRE
				v.mView->setBackgroundColour(Ogre::ColourValue(c.R/255.f, c.G/255.f, c.B/255.f, 1.f));
#endif
			}
			]],
			staticMemberFunctions=[[
			static void setBackgroundColour(OgreRenderer::Viewport & v,CPixelRGB8 c)
			]],
			memberFunctions=[[
			virtual void changeView(Viewpoint const& view);	
			virtual void changeView(matrix4 const& matview);	
			void setOrthographicMode(bool isOrtho);
			void setCustomProjectionMatrix(matrix4 const& mat_proj);
			]],
		},
		{ 
			name='OgreRenderer',
			memberFunctions={
		[[
	void screenshot(bool b);
	void setScreenshotMotionBlur(int n);
	void setScreenshotPrefix(const char* prefix);
	void fixedTimeStep(bool b);
	void setCaptureFPS(float fps)
	void addNewViewport();
	int numViewport() 	
	OgreRenderer::Viewport const& viewport()	const;
	OgreRenderer::Viewport& viewport(int viewport);
	void addFrameMoveObject(FrameMoveObject* pFMO);
	void removeFrameMoveObject(FrameMoveObject* pFMO);
	void addAfterFrameMoveObject(FrameMoveObject* pFMO);
	void removeAfterFrameMoveObject(FrameMoveObject* pFMO);
	void createDynamicTexture(const char* name, CImage const& image);
	void createRenderTexture(const char* type, int width, int height, bool useCurrentViewport, const char* name);
	void updateRenderTexture(const char* param);
	void setMaterialParam(const char* mat, const char* paramName, double param_value)
	void cloneMaterial(const char* mat, const char* newMat)
	void ChageTextureImage(const char* name, const char* textureName);
	void setrotate(m_real degree, const char* name)'

	]]},
		},
		{
			name='MotionUtil.PoseTransfer',
			className='PoseTransfer',
			ctors={
				'(MotionLoader* pSrcSkel, MotionLoader* pTgtSkel)',
				'(MotionLoader* pSrcSkel, MotionLoader* pTgtSkel, bool bCurrPoseAsBindPose)',
				'(MotionLoader* pSrcSkel, MotionLoader* pTgtSkel, const char* convfilename, bool bCurrPoseAsBindPose)',
			},
			memberFunctions={[[
			void setTargetSkeleton(const Posture & srcposture);	
			void setTargetSkeletonBothRotAndTrans(const Posture& srcposture);
			MotionLoader* source() 
			MotionLoader* target()
			]]}
		},
		{ 
			name='MotionLoader',
			ctors={"()", "(const char*)"},
			properties={"Motion m_cPostureIP @ mMotion", 
						"MotionDOFinfo dofInfo"},
			memberFunctions= [[
				void insertSiteBones();
				BoneForwardKinematics & fkSolver() const	
				void readJointIndex(const char* filename);
				int numRotJoint() const;	//!< KeyFrame되어 있는 조인트의 개수를 구한다.
				int numTransJoint() const;
				int numBone() const; //!< including the DUMMY bone (= bone 0).
				void setCurPoseAsInitialPose();

				/// to retrieve the initial pose (identity pose)
				void UpdateInitialBone(); @ updateInitialBone
				void UpdateBone(); @ updateBone
				void getPose(Posture& pose) const;
				void Scale(float fScale); 
				void scale(float fScale, Motion& mot);
				void updateBoneLengthFromGlobal();
				void createDummyRootBone()
				void setChain(const Posture&, int)const
				void setChain(const Posture&, Bone&)const
				void setNewRoot(Bone& newRoot);
				void insertJoint(Bone&, const char* type)
				void insertChildBone(Bone& parent, const char* nameId, bool bMoveChildren)
				void insertChildBone(Bone& parent, const char* nameId)
				void exportSkeleton( const char* filename)
				void removeBone(Bone& target);
				void removeAllRedundantBones();
				void setPose(const Posture& pose);
				void getPose(Posture& pose) const;
				void setPoseDOF(const vectorn& poseDOF) const
				void getPoseDOF(vectorn& poseDOF) const
				Bone& bone(int index) const;
				Bone& getBoneByTreeIndex(int index)	const;
				Bone& getBoneByRotJointIndex(int iRotJoint)	const;
				Bone& getBoneByTransJointIndex(int iTransJoint)	const;
				Bone& getBoneByVoca(int jointVoca)	const;
				Bone& getBoneByName(const char*) const;
				int getTreeIndexByName(const char* name) const;
				int getTreeIndexByRotJointIndex(int rotjointIndex) const;
				int getTreeIndexByTransJointIndex(int transjointIndex) const;
				int getTreeIndexByVoca(int jointVoca) const;
				int getRotJointIndexByName(const char* nameID) const;
				int getRotJointIndexByTreeIndex(int treeIndex) const;
				int getRotJointIndexByVoca(int jointVoca) const;
				int getTransJointIndexByName(const char* nameID);
				int getTransJointIndexByTreeIndex(int treeIndex) const;
				int getVocaByTreeIndex(int treeIndex) const;
				int getVocaByRotJointIndex(int rotjointIndex) const;
				void _changeVoca(int jointVoca, Bone & bone);
				void _updateTreeIndex();
				void _initDOFinfo();
				]],
			wrapperCode=[[
			static void printHierarchy(MotionLoader& skel)
			{
				skel.GetNode(0)->printHierarchy();
			}

			static MotionLoader* getMotionLoader(const char* fn)
			{
				return RE::renderer().m_pMotionManager->GetMotionLoaderPtr(fn);
			}
			static MotionLoader* _create(const char* filename)
			{
				MotionLoader* l;

				TString fn(filename);
				TString ext=fn.right(3).toUpper();
				if(ext=="ASF")
				{
					l=new ASFLoader(filename);
				}
				else if (ext=="WRL")
				{
					l=new VRMLloader(filename);
				}
				else if(ext=="BVH")
				{
					l=new BVHLoader(filename);
				}
				else if(ext=="SKL"){
					l=new BVHLoader(fn.left(-3)+"BVH","loadSkeletonOnly");
				}
				else
				{
					l=new MotionLoader(filename);
				}
				return l;
			}
			static void _append(MotionLoader* l, const char* motFile)
			{
				Motion& mot=l->m_cPostureIP;

				if(mot.numFrames()==0)
					l->loadAnimation(mot, motFile);
				else
					RE::motion::concatFromFile(mot, motFile);
			}
			static void loadAnimation(MotionLoader& skel, Motion& mot, const char* fn)
			{
				RE::motion::loadAnimation(skel, mot, fn);
			}
			]],
			staticMemberFunctions={
				[[
				static void loadAnimation(MotionLoader& skel, Motion& mot, const char* fn)
				static MotionLoader* _create(const char* filename) @ ;adopt=true;
				static void _append(MotionLoader* l, const char* motFile)
				static void printHierarchy(MotionLoader& skel)
				static MotionLoader* getMotionLoader(const char* fn)
				]]},
			enums={
				{"HIPS", "(int)MotionLoader::HIPS"},
				{"LEFTHIP", "(int)MotionLoader::LEFTHIP"},
				{"LEFTKNEE", "(int)MotionLoader::LEFTKNEE"},
				{"LEFTANKLE", "(int)MotionLoader::LEFTANKLE"},
				{"LEFTTOES", "(int)MotionLoader::LEFTTOES"},
				{"RIGHTHIP", "(int)MotionLoader::RIGHTHIP"},
				{"RIGHTKNEE", "(int)MotionLoader::RIGHTKNEE"},
				{"RIGHTANKLE", "(int)MotionLoader::RIGHTANKLE"},
				{"RIGHTTOES", "(int)MotionLoader::RIGHTTOES"},
				{"CHEST", "(int)MotionLoader::CHEST"},
				{"CHEST2", "(int)MotionLoader::CHEST2"},
				{"LEFTCOLLAR", "(int)MotionLoader::LEFTCOLLAR"},
				{"LEFTSHOULDER", "(int)MotionLoader::LEFTSHOULDER"},
				{"LEFTELBOW", "(int)MotionLoader::LEFTELBOW"},
				{"LEFTWRIST", "(int)MotionLoader::LEFTWRIST"},
				{"RIGHTCOLLAR", "(int)MotionLoader::RIGHTCOLLAR"},
				{"RIGHTSHOULDER", "(int)MotionLoader::RIGHTSHOULDER"},
				{"RIGHTELBOW", "(int)MotionLoader::RIGHTELBOW"},
				{"RIGHTWRIST", "(int)MotionLoader::RIGHTWRIST"},
				{"NECK", "(int)MotionLoader::NECK"},
				{"HEAD", "(int)MotionLoader::HEAD"},
				{"LThWrist","(int)MotionLoader::LThWrist"},
				{"LThMetac","(int)MotionLoader::LThMetac"},
				{"LThIntra1","(int)MotionLoader::LThIntra1"},
				{"LF1Metac","(int)MotionLoader::LF1Metac"},
				{"LF1Intra1","(int)MotionLoader::LF1Intra1"},
				{"LF1Intra2","(int)MotionLoader::LF1Intra2"},
				{"LF2Metac","(int)MotionLoader::LF2Metac"},
				{"LF2Intra1","(int)MotionLoader::LF2Intra1"},
				{"LF2Intra2","(int)MotionLoader::LF2Intra2"},
				{"LF3Metac","(int)MotionLoader::LF3Metac"},
				{"LF3Intra1","(int)MotionLoader::LF3Intra1"},
				{"LF3Intra2","(int)MotionLoader::LF3Intra2"},
				{"LF4Metac","(int)MotionLoader::LF4Metac"},
				{"LF4Intra1","(int)MotionLoader::LF4Intra1"},
				{"LF4Intra2","(int)MotionLoader::LF4Intra2"},
				{"RThWrist","(int)MotionLoader::RThWrist"},
				{"RThMetac","(int)MotionLoader::RThMetac"},
				{"RThIntra1","(int)MotionLoader::RThIntra1"},
				{"RF1Metac","(int)MotionLoader::RF1Metac"},
				{"RF1Intra1","(int)MotionLoader::RF1Intra1"},
				{"RF1Intra2","(int)MotionLoader::RF1Intra2"},
				{"RF2Metac","(int)MotionLoader::RF2Metac"},
				{"RF2Intra1","(int)MotionLoader::RF2Intra1"},
				{"RF2Intra2","(int)MotionLoader::RF2Intra2"},
				{"RF3Metac","(int)MotionLoader::RF3Metac"},
				{"RF3Intra1","(int)MotionLoader::RF3Intra1"},
				{"RF3Intra2","(int)MotionLoader::RF3Intra2"},
				{"RF4Metac","(int)MotionLoader::RF4Metac"},
				{"RF4Intra1","(int)MotionLoader::RF4Intra1"},
				{"RF4Intra2","(int)MotionLoader::RF4Intra2"},
			}
		},
{
	name='LimbIKsolver',
	className='MotionUtil::LimbIKsolver',
	ctors={
	'(MotionDOFinfo const& dofInfo, std::vector<MotionUtil::Effector>& effectors, intvectorn const& hip_bone_indexes, intvectorn const& knee_bone_indexes, vectorn const& axis_sign)',
	'(MotionDOFinfo const& dofInfo, std::vector<MotionUtil::Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign)',
	'(MotionDOFinfo const& dofInfo, std::vector<MotionUtil::Effector>& effectors, Bone const& left_knee, Bone const& right_knee)',
	   },
memberFunctions={[[
	void IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance);
	void IKsolve(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con);
	void IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con);
		void setOption(const char* id, double val);
]]}
},
{
	name='LimbIKsolver2',
	className='MotionUtil::LimbIKsolver2',
	ctors={
	'(MotionDOFinfo const& dofInfo, std::vector<MotionUtil::Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign)',
	   },
memberFunctions={[[
	void IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance);
	void IKsolve(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con);
	void IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con);
	void setOption(int option);
	void setOption(const char* id, double val);
	void setValue(double ValL, double ValM, double ValN,int IterNum);
]]}
},
{
	name='HandIKsolver',
	className='MotionUtil::HandIKsolver',
	ctors={
	'(MotionDOFinfo const& dofInfo, std::vector<MotionUtil::Effector>& effectors, intvectorn const& hand_bone_indexes,intvectorn const& wrist_bone_indexes, vectorn const& axis_sign)',
	   },
	memberFunctions={
	[[	
		void IKsolve(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance)
		void IKsolve(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance, vectorn const& importance_wrist)
		void setOption(const char* id, double val);
	]]
	}	
},


{
	name='COM_IKsolver',
	className='MotionUtil::COM_IKsolver',
	ctors={
	'(VRMLloader const& skel, std::vector<MotionUtil::Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign)'
	   },
memberFunctions={[[
  void IKsolve(vectorn& origRootTF_, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& _con, vectorn const& _importance, vector3 const& _desiredCOM);
  void IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con);
]]}
},
			{
				name='MotionUtil.FullbodyIK',
				decl=[[namespace MotionUtil { class FullbodyIK; }]],
				memberFunctions={[[
					void IKsolve(Posture& , vector3N const& )
					void IKsolve(Posture const&, Posture&, vector3N const& )
					void IKsolve(Posture const& input_pose, vector3N const& cPositions, intvectorn & rot_joint_index, quaterN& delta_rot);
							 ]]},
			},
			{
				name='MotionUtil.FullbodyIK_MotionDOF',
				decl=[[namespace MotionUtil { class FullbodyIK_MotionDOF; }]],
				memberFunctions={[[
					void setParam(const char* type, double value)
					void setParam(const char* type, double value, double value2)
					void IKsolve(vectorn const& , vectorn& , vector3N const& )
					void IKsolve(vectorn& , vector3N const& )
					bool _updateBoneLength(MotionLoader const& loader);
					bool _changeNumEffectors(int n) 
					bool _changeNumConstraints(int n) 
					bool _setEffector(int i, Bone* bone, vector3 const& lpos) 
					bool _setRelativeConstraint(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2) 
					bool _setPlaneDistanceConstraint(int i, Bone* bone, vector3 const& lpos, vector3 const& global_normal, float idepth) 
					bool _setHalfSpaceConstraint(int i, Bone* bone, vector3 const& lpos, vector3 const& global_normal, float idepth) 
					bool _setCOMConstraint(int i, vector3 const& com)
					bool _setCOMConstraint(int i, vector3 const& com, double wx, double wy, double wz)
					bool _setPositionConstraint(int i, Bone* bone, vector3 const&lpos, vector3 const& desired_pos, double wx, double wy, double wz) 
					bool _setOrientationConstraint(int i, Bone* bone, quater const& desired_ori);
					bool _setOrientationConstraint(int i, Bone* bone, quater const& desired_ori, double weight);
					bool _setMomentumConstraint(int i, vector3 const& ang, vector3 const& lin);
					bool _setMomentumConstraint(int i, vector3 const& ang, vector3 const& lin, double w);
					bool _setPoseConstraint(int i, vectorn const& pose, double weight, int startBoneIndex, int endBoneIndex);
					bool _setPoseConstraint(int i, vectorn const& pose, double weight, int startBoneIndex);
					bool _setPoseConstraint(int i, vectorn const& pose, double weight);
					bool _setEffectorWeight(int i, double w) 
					bool _setEffectorYConstraint(int i, double weight, const vectorn& ew)
					bool _effectorUpdated()
				]]},
			},
			{
				name='MainLib.VRMLTransform',
				className='VRMLTransform',
				inheritsFrom='Bone',
				wrapperCode=[[
				static std::string HRPjointName(VRMLTransform& t, int i)
				{
					return std::string(t.HRPjointName(i).ptr());
				}
				]],
				enums={
					{"FREE","HRP_JOINT::FREE"},
					{"BALL","HRP_JOINT::BALL"},
					{"ROTATE","HRP_JOINT::ROTATE"},
					{"FIXED","HRP_JOINT::FIXED"},
					{"GENERAL","HRP_JOINT::GENERAL"},
					{"SLIDE","HRP_JOINT::SLIDE"}
				},
				staticMemberFunctions={[[
				static std::string HRPjointName(VRMLTransform& t, int i)
				]]},
				memberFunctions={[[
				void translateMesh( vector3 const& trans);
				void setJointPosition(vector3 const& trans);
				void setJointAxes(const char* axes)
				void translateBone(vector3 const& trans);
				void transformMesh(matrix4 const& m);
				void transformMeshLocal(matrix4 const& m);
				void scaleMesh( vector3 const& scale);
				void scaleMass( m_real scalef); 
				void initBones();
				void copyFrom(VRMLTransform const& bone);	
				bool hasShape() const 
				OBJloader::Geometry& getMesh() const 
				int numHRPjoints() const;
				int HRPjointIndex(int i) const;
				int DOFindex(int i) const;
				TString HRPjointName(int i) const;
				int HRPjointType(int i) const;
				TString HRPjointAxis(int i) const;
				int lastHRPjointIndex() const	
				vector3 localCOM() const;
				void setLocalCOM(vector3 const& com);
				double mass();
				vector3 inertia() const;
				void setInertia(double ix, double iy, double iz);
				void jointToBody(vector3& lposInOut) const;
				void bodyToJoint(vector3& lposInOut) const;
				]] },
			},
							{
								name='VRMLloader',
								inheritsFrom='MotionLoader',
								wrapperCode=[[
								inline static const char* name(VRMLloader& l)
								{
									return l.name;
								}
								inline static void setName(VRMLloader& l, const char* name)
								{
									l.name=name;
								}
								inline static VRMLTransform* VRMLloader_upcast(Bone& bone)
								{
									return dynamic_cast<VRMLTransform*>(&bone);
								}
								static VRMLTransform* upcast(Bone& bone)
								{
									return dynamic_cast<VRMLTransform*>(&bone);
								}
								static VRMLloader* upcast(MotionLoader* skel)
								{
									return dynamic_cast<VRMLloader*>(skel);
								}
								]]
								,
								ctors={
									"()", 
									"(const char*)",
									"(VRMLloader const&)",
									"(MotionLoader const&,double)"
								},
								staticMemberFunctions={[[
								VRMLTransform* upcast(Bone& bone)
								VRMLloader* upcast(MotionLoader* skel)
								void VRMLloader::projectAngles(vectorn & temp1); @ projectAngles
								const char* name(VRMLloader& l)
								static void setName(VRMLloader& l, const char* name)
								void VRMLloader_updateMeshEntity(VRMLloader& l); @ _updateMeshEntity ;ifndef=NO_OGRE;

								]]},
								memberFunctions={
								--void setTotalMass( m_real totalMass); 
								[[
								void operator=(VRMLloader const&);
								void _initDOFinfo();
								void printDebugInfo(); 
								void changeTotalMass( m_real totalMass); 
									VRMLTransform& VRMLbone(int treeIndex) const;
									void insertChildJoint(Bone& parent, const char* tchannels, const char* rchannels, const char* nameId, bool bMoveChildren);
									void insertChildJoint(Bone& parent, const char* tchannels, const char* rchannels, const char* nameId, bool bMoveChildren, vector3 const& offset);
									vector3 calcCOM() const;
									void calcZMP(const MotionDOF& motion, matrixn & aZMP, double kernelSize);
									void exportVRML(const char* filename); @ export
									void exportBinary(const char* filename); 
									int numHRPjoints()
									void scale(float fScale,Motion& mot);

									]]
								},
							},
									{
										name='VRMLloaderView',
										inheritsFrom='VRMLloader',
										ctors={'(VRMLloader const& source, Bone& newRootBone, vector3 const& localPos)'},
										properties={
											'Bone* _newRootBone',
											'Bone* _srcRootTransf',
											'vector3 _srcRootOffset',
										},
										memberFunctions={[[
										VRMLloader const & getSourceSkel() 
										void convertSourcePose(vectorn const& srcPose, vectorn& pose) const;
										void convertPose(vectorn const& pose, vectorn& srcPose) const;
										void convertSourceDOFexceptRoot(vectorn const& srcPose, vectorn& pose) const;
										void convertDOFexceptRoot(vectorn const& pose, vectorn& srcPose) const;
										void convertSourceDQexceptRoot(vectorn const& src_dq, vectorn& dq) const;
										void convertDQexceptRoot(vectorn const& dq, vectorn& src_dq) const;
										]]}
									},
							{
								decl='class SkinnedMeshLoader;',
								ifndef='NO_GUI',
								name='SkinnedMeshLoader',
								inheritsFrom='MotionLoader',
								ctors={"(const char*)","(const char*, bool)"},
								memberFunctions={ [[
								void reorderVertices()
								void loadMesh(const char* fn);
								OBJloader::Mesh& getCurrMesh();
								void gotoBindPose();
								virtual void sortBones(MotionLoader const& referenceSkeleton);
								void setCurPoseAsBindPose();
								void setPose(Posture& pose);
								void retrieveAnimatedMesh(OBJloader::Mesh& mesh);
								]] },
							},

	},
	modules={
		{
			namespace='MotionUtil',
			decl=[[
			namespace MotionUtil{
				void exportVRML(Motion const& mot, const char* filename, int start, int end);
				void exportBVH(Motion const& mot, const char* filename, int start, int end);
				void upsample(Motion& out, const Motion& in, int nSuperSample);
				void downsample(Motion& out, const Motion& in, int nDownSample);
				void setKneeDampingCoef_RO(double ro);
				intvectorn findChildren(MotionLoader& skeleton, int rootTreeIndex);
			}
			]],
			functions={[[
			void MotionUtil::exportVRML(Motion const& mot, const char* filename, int start, int end)
			void MotionUtil::exportBVH(Motion const& mot, const char* filename);
			void MotionUtil::exportBVH(Motion const& mot, const char* filename, int start);
			void MotionUtil::exportBVH(Motion const& mot, const char* filename, int start, int end);
			void MotionUtil::upsample(Motion& out, const Motion& in, int nSuperSample);
			void MotionUtil::downsample(Motion& out, const Motion& in, int nDownSample);
			void MotionUtil::setKneeDampingCoef_RO(double ro)
			void MotionUtil::timewarpingLinear(Motion& out, const Motion& in, const vectorn& timewarpFunction);
			Motion* MotionUtil::untimewarpingLinear(const Motion& timewarpedMotion, const vectorn& timewarpFunction);
			intvectorn MotionUtil::findChildren(MotionLoader& skeleton, int rootTreeIndex);
			]]}
		},
		{
			namespace='util',
			functions={[[
				void Msg::msgBox(const char*) @ msgBox
			]]}
		},
		{
			namespace='Ogre',
			ifndef='NO_GUI',
			functions={[[
			Ogre::OverlayContainer* Ogre::createContainer(int x, int y, int w, int h, const char* name) ;
			Ogre::OverlayElement* createTextArea_(const char* name, double width, double height, double top, double left, int fontSize, const char* caption, bool show); @ createTextArea
			Ogre::Overlay* createOverlay_(const char* name); @ createOverlay
			void destroyOverlay_(const char* name); @ destroyOverlay
			void destroyOverlayElement_(const char* name); @ destroyOverlayElement
			void destroyAllOverlayElements_(); @ destroyAllOverlayElements
			]]}
		},
		{
			namespace='Fltk',
			wrapperCode=[[
			static std::string chooseFile(const char* msg, const char* path, const char* mask, bool bCreate)
			{
#ifndef NO_GUI
				TString temp=FlChooseFile(msg,path,mask,bCreate).ptr();
				if(temp.length())
					return std::string(temp.ptr());
#endif
				return std::string("");
			}

			static bool ask(const char* msg)
			{ 
#ifndef NO_GUI

				return fl_ask("%s", msg);
#else
				return true;
#endif
			}

			]],
			functions={[[
			static std::string chooseFile(const char* msg, const char* path, const char* mask, bool bCreate)
			static bool ask(const char* msg)
			]]}
		},
		{
			namespace='sop',
			functions={[[
			m_real sop::map(m_real t, m_real min, m_real max, m_real v1, m_real v2)
			m_real sop::clampMap(m_real t, m_real min, m_real max, m_real v1, m_real v2)
			]]}
		},
		{
			namespace='MotionUtil',
			functions={[[
			MotionUtil::FullbodyIK* createFullbodyIk_LimbIK(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& effectors);
			MotionUtil::FullbodyIK_MotionDOF* createFullbodyIk_MotionDOF_MultiTarget(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors);
			MotionUtil::FullbodyIK_MotionDOF* createFullbodyIk_MotionDOF_MultiTarget_lbfgs(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors, std::vector<MotionUtil::RelativeConstraint> &con);
			MotionUtil::FullbodyIK_MotionDOF* MotionUtil::createFullbodyIk_MotionDOF_MultiTarget_lbfgs(MotionDOFinfo const& info);
			MotionUtil::FullbodyIK_MotionDOF* createFullbodyIkDOF_limbIK(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors, Bone const& left_knee, Bone const& right_knee);
			MotionUtil::FullbodyIK_MotionDOF* createFullbodyIkDOF_limbIK(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors, Bone const& left_knee, Bone const& right_knee, bool bReversed);
			MotionUtil::FullbodyIK_MotionDOF* createFullbodyIkDOF_limbIK_straight(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors, Bone const& left_knee, Bone const& right_knee);
			void setLimbIKParam_straight(MotionUtil::FullbodyIK_MotionDOF* ik, bool bStraight);
			void MotionUtil::exportVRMLforRobotSimulation(Motion const& mot, const char* filename, const char* robotname);
			void MotionUtil::exportVRMLforRobotSimulation(Motion const& mot, const char* filename, const char* robotname, double thickness);
			void MotionUtil::exportVRMLforRobotSimulation(MotionLoader const& skel, const char* filename, const char* robotname);
			void MotionUtil::exportVRMLforRobotSimulation(MotionLoader const& skel, const char* filename, const char* robotname, double thickness);
			]]}
		},
		{
			namespace='RE',
			wrapperCode=[[
				inline static Ogre::Entity* RE_createPlane2(const char* id, m_real width, m_real height, int xsegment, int ysegment)
				{
					return RE::createPlane(id,width,height,xsegment,ysegment,1,1);
				}
				inline static PLDPrimOgreSkin* RE_createOgreSkin(const Motion& m) 
				{
					return (PLDPrimOgreSkin*) RE::createOgreSkin(m) ;
				}
				inline static PLDPrimOgreSkin* RE_createOgreSkin2(const MotionLoader& skel, Ogre::Entity* entity, const char* mappingTable, bool bCurrPoseAsBindPose, double scale=1) 
				{
					return (PLDPrimOgreSkin*) RE::createOgreSkin(skel, entity, mappingTable, bCurrPoseAsBindPose, scale) ;
				}
				inline static void RE_buildEdgeList(const char* meshName)
				{
                    #ifndef NO_GUI
					Ogre::String groupName = Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME;
					// Get mesh (load if required)
					Ogre::MeshPtr pmesh = Ogre::MeshManager::getSingleton().load(meshName, groupName);
					pmesh->freeEdgeList();
					//pmesh->prepareForShadowVolume();
					pmesh->buildEdgeList();
					//pmesh-> setAutoBuildEdgeLists(true);
					//pmesh->load();
					#endif
				}
				inline static int RE_getOgreVersionMinor()
				{
					#ifdef NO_GUI
					return 7;
					#else
					return OGRE_VERSION_MINOR;
					#endif
				}
			]],
			functions=
			{
				[[
				void RE::usleep(int usec);
				Ogre::SceneManager* RE::ogreSceneManager();
				void RE_outputRaw(const char* key, const char* output, int i); @ _output
				void RE_dumpOutput(TStrings& output, int i); @ dumpOutput
				void RE_outputEraseAll(int i); @outputEraseAll
				void RE::outputState(bool bOutput);
				int RE::numOutputManager();
				MotionLoader* RE::motionLoader(const char* name);
				MotionLoader* RE::createMotionLoader(const char* name, const char* key)
				MotionLoader* MotionManager::createMotionLoader(const char* name)  @ createMotionLoaderExt
				Ogre::SceneNode* RE::ogreRootSceneNode();
				Ogre::SceneNode* RE::createChildSceneNode(Ogre::SceneNode* parent, const char* child_name);
				FrameSensor* RE::createFrameSensor();
				TString RE::generateUniqueName();
				static void RE_::remove(PLDPrimSkin* p)
				Ogre::SceneNode* RE::createEntity(const char* id, const char* filename)
				Ogre::SceneNode* RE::createEntity(const char* id, const char* filename, const char* materialName)
				Ogre::SceneNode* RE::createEntity(Ogre::SceneNode*, const char* id, const char* filename)
				void RE::removeEntity(Ogre::SceneNode*)
				void RE::setMaterialName(Ogre::SceneNode* pNode, const char* mat);
				void RE::moveEntity(Ogre::SceneNode*, quater const&, vector3 const&)
				Ogre::Entity* RE::createPlane(const char* id, m_real width, m_real height, int xsegment, int ysegment, int texSegx, int texSegy)
				Ogre::Entity* RE_createPlane2(const char* id, m_real width, m_real height, int xsegment, int ysegment) @ createPlane
				Ogre::Entity* RE::createTerrain(const char* id, const char* filename, int imageSizeX, int imageSizeY, m_real sizeX, m_real sizeZ, m_real heightMax, int ntexSegX, int ntexSegZ);
				void RE_::setBackgroundColour(m_real r, m_real g, m_real b)
				static Viewpoint* RE_::getViewpoint() @ viewpoint
				static Viewpoint* RE_::getViewpoint(int) @ viewpoint
				MotionPanel& RE::motionPanel(); @;ifndef=NO_GUI;
				bool RE::rendererValid();
				bool RE::motionPanelValid();
				OgreRenderer& RE::renderer();
				FltkRenderer& RE::FltkRenderer();
				void RE_::renderOneFrame(bool check)
				PLDPrimVRML* RE::createVRMLskin(VRMLloader*pTgtSkel, bool bDrawSkeleton) @ ;adopt=true; 
				PLDPrimOgreSkin* RE_createOgreSkin(const Motion&) @  createOgreSkin ;adopt=true;
				PLDPrimOgreSkin* RE_createOgreSkin2(const MotionLoader& skel, Ogre::Entity* entity, const char* mappingTable, bool bCurrPoseAsBindPose) @ createOgreSkin ;adopt=true;
 				PLDPrimOgreSkin* RE_createOgreSkin2(const MotionLoader& skel, Ogre::Entity* entity, const char* mappingTable, bool bCurrPoseAsBindPose,double scale) @ createOgreSkin ;adopt=true;
				int RE_getOgreVersionMinor() @ getOgreVersionMinor
				void RE_buildEdgeList(const char* meshName) @ buildEdgeList
				void ::loadPose(Posture& pose, const char* fn) 
				void ::savePose(Posture& pose, const char* fn)
				Ogre::SceneNode* RE::getSceneNode(PLDPrimSkin* skin)
				Ogre::SceneNode* RE::getSceneNode(const char* id)
				Ogre::SceneNode* RE::createSceneNode(const char* node_name);
				Ogre::SceneNode* RE::createChildSceneNode(Ogre::SceneNode* parent, const char* child_name);
				]],
				{"PLDPrimSkin* RE::createSkin(const Motion&)", adopt=true},
				{"PLDPrimSkin* RE::createOgreSkin(const Motion&)", adopt=true},
				{"PLDPrimSkin* RE::createSkin(const MotionLoader&)", adopt=true},
				{"PLDPrimSkin* RE::createSkin(const Motion&, RE::PLDPrimSkinType type)", adopt=true},
				{"PLDPrimSkin* RE::createSkin(const MotionLoader&, RE::PLDPrimSkinType type)", adopt=true},
			}
		}
	}
}

	-- definitions should be in a single lua file (Though, of couse, you can use "dofile" or "require" in this file. )
	-- output cpp files can be seperated by modules or classes so that C++ compilation time is reduced.

	function generateBaseLib()
		-- declaration
		write([[
					class vector4;
					class CPixelRGB8;
					class TRect;
					class CImage;
					  class boolNView;
					  class boolN;
					  class NonuniformSpline;
					  class LMat;
					  class LMatView;
					  class LVec;
					  class LVecView;
					  class QPerformanceTimerCount2;
					  class FractionTimer;
					  class BSpline;
					  class intIntervals;
					  namespace MotionUtil{
						  class RetargetOnline2D;
						class COM_IKsolver;
						class LimbIKsolver;
						class LimbIKsolver2;
						class LimbIKsolverLua;
						class LimbIKsolverHybrid;
						class HandIKsolver;
						class Effector;
						class RelativeConstraint;
						class FullbodyIK;
						class FullbodyIK_MotionDOF;
					  }
					  namespace m
					  {
						  struct stitchOp;
						  struct linstitch;
						  struct linstitchMulti;
						  struct c1stitchPreprocess ;
						  struct c1stitchPreprocessOnline;
						  struct linstitchPreprocessInc;
						  struct linstitchPreprocess;
						  struct linstitch2;
						  struct linstitchOnline;
						  struct linstitchForward;
						  struct stitchQuaterNN ;
					  }
		]])
		writeHeader(bindTargetBaseLib)
		if gen_lua.use_profiler then
			write('\n#define LUNA_GEN_PROFILER\n')
		end
		flushWritten(	source_path..'/luna_baselib.h')
		write(
		[[
		#include "stdafx.h"
		//#include "../BaseLib/utility/TWord.h"
		#include "../../BaseLib/motion/Motion.h"
		#include "../../BaseLib/motion/IK_sdls/Node.h"
		#include "../../BaseLib/motion/IK_sdls/NodeWrap.h"
		#include "../BaseLib/motion/IK_sdls/Tree.h"
		#include "../../BaseLib/motion/MotionLoader.h"
		#include "../MainLib/OgreFltk/MotionPanel.h"
		#include "../BaseLib/image/Image.h"
		#include "../BaseLib/image/DrawChart.h"
		#include "../BaseLib/image/ImageProcessor.h"
		#include "../BaseLib/math/OperatorStitch.h"
		#include "../BaseLib/motion/MotionRetarget.h"
		//#include "../BaseLib/motion/Concat.h"
		//#include "../BaseLib/motion/MotionUtilSkeleton.h"
		#include "../BaseLib/motion/MotionUtil.h"
		#include "../BaseLib/motion/ConstraintMarking.h"
		#include "../BaseLib/motion/postureip.h"
		#include "../BaseLib/math/Operator.h"
		#include "../BaseLib/math/tvector.h"
		#include "../BaseLib/math/Operator_NR.h"
		#include "../../BaseLib/math/bitVectorN.h"
		//#include "../BaseLib/math/GnuPlot.h"
		#include "../BaseLib/math/conversion.h"
		#include "../MainLib/Ogre/intersectionTest.h"
		//#include "../BaseLib/math/stransf.h"
		#include "../BaseLib/math/Filter.h"
		#include "../BaseLib/math/matrix3.h"
		#include "../BaseLib/math/hyperMatrixN.h"
		#include "../BaseLib/math/BSpline.h"
		//#include "../BaseLib/math/LMat.h"
		//#include "../BaseLib/math/LVec.h"
		//#include "../BaseLib/math/LMat_lapack.h"
		#include "../BaseLib/utility/operatorString.h"
		#include "../BaseLib/utility/QPerformanceTimer.h"
		#include "../BaseLib/utility/tfile.h"
		#include "../BaseLib/utility/BinaryFile.h"
#include "../BaseLib/motion/MotionUtil.h"
#include "../BaseLib/motion/VRMLexporter.h"
#include "../BaseLib/motion/LimbIKsolver.h"
#include "../BaseLib/motion/LimbIKsolver2.h"
#include "../MainLib/OgreFltk/LimbIKsolverLua.h"
#include "../BaseLib/motion/HandIKsolver.h"
#include "../BaseLib/motion/COM_IKsolver.h"
		//		#include "../MainLib/WrapperLua/BaselibLUA2.h"
				
		]]
		)
		writeIncludeBlock(true)
		write('#include "luna_baselib.h"')
		writeDefinitions(bindTargetBaseLib, 'Register_baselib') -- input bindTarget can be non-overlapping subset of entire bindTarget 
		flushWritten(source_path..'/luna_baselib.cpp') -- write to cpp file only when there exist modifications -> no-recompile.
	end
	function generate() 
		buildDefinitionDB(bindTargetBaseLib, bindTargetMainLib)--, bindTargetPhysics) -- input has to contain all classes to be exported.
		-- write function can be used liberally.
		generateBaseLib()
		generateMainLib()
		--generatePhysicsBind()
		writeDefinitionDBtoFile(source_path..'/luna_baselib_db.lua')
	end

	function generateMainLib() 
-- declaration
write([[
class CMAwrap;
namespace MotionUtil
{
class COM_IKsolver;
class LimbIKsolver;
class LimbIKsolver2;
class LimbIKsolverLua;
class HandIKsolver;
}
namespace Ogre
{
	class MovableObject;
	class Entity;
	class Overlay;
	class OverlayElement;
	class OverlayContainer;
	class SceneNode;
	class SceneManager;
}
namespace OBJloader
{
class Mesh;
class EdgeConnectivity;
class Element;
class Geometry;
class MeshToEntity;
class MeshEntity;
class TriangleMesh;
class MeshLineStrip;
class MeshLineStripReduced;
class Face;
}
class GlobalUI;
class Bone;
class PLDPrimSkin;
class MotionDOFinfo;
class InterframeDifference;
class InterframeDifferenceC1;
class Motion;
class MotionDOF;
class MotionDOFview;
class BoneForwardKinematics;
class FrameSensor;
class AnimationObject;
class Posture;
class FltkRenderer;
class MotionPanel;
class MotionView;
class OgreRenderer;
class PoseTransfer;
class MotionLoader;
class QuadraticFunction;
class QuadraticFunctionHardCon;
class FltkMotionWindow;
class FltkScrollPanel;
#ifndef NO_GUI
class Loader;
#endif
class ObjectList;
class WeightedPointCloudMetric;
class PointCloudMetric;
class L2Metric;
class Metric;
class KovarMetric;
class NoRotMetric;
class VRMLloader;
class VRMLloaderView;
class VRMLTransform;
class FlLayout;
class Viewpoint;
class FlChoiceWins;
class PLDPrimVRML;
#include "../OgreFltk/renderer.h"
]])
writeHeader(bindTargetMainLib)
flushWritten(	source_path..'/luna_mainlib.h')
-- write function can be used liberally.
write(
[[
#include "stdafx.h"
#include "../MainLib/OgreFltk/MotionPanel.h"
#include "../MainLib/OgreFltk/FltkScrollPanel.h"
#include "../BaseLib/motion/MotionRetarget.h"
#include "../BaseLib/motion/MotionUtil.h"
#include "../BaseLib/motion/VRMLexporter.h"
#include "../BaseLib/motion/LimbIKsolver.h"
#include "../BaseLib/motion/LimbIKsolver2.h"
#include "../MainLib/OgreFltk/LimbIKsolverLua.h"
#include "../BaseLib/motion/HandIKsolver.h"
#include "../BaseLib/motion/COM_IKsolver.h"
#include "../BaseLib/math/Operator.h"
#include "../BaseLib/math/Metric.h"
#include "../BaseLib/motion/viewpoint.h"
#include "../BaseLib/motion/version.h"
#include "../BaseLib/motion/Geometry.h"
#include "../MainLib/OgreFltk/FlLayout.h"
#include "../MainLib/OgreFltk/Mesh.h"
#include "../MainLib/OgreFltk/objectList.h"
#include "../MainLib/OgreFltk/MotionManager.h"
#include "../MainLib/OgreFltk/renderer.h"
#include "../MainLib/Ogre/intersectionTest.h"
#include "../BaseLib/motion/FullbodyIK_MotionDOF.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../MainLib/OgreFltk/OgreMotionLoader.h"
#include "../MainLib/OgreFltk/VRMLloaderView.h"
#include "../MainLib/OgreFltk/GlobalUI.h"
#include "../MainLib/OgreFltk/FltkRenderer.h"
#include "../MainLib/OgreFltk/FlChoice.h"
#include "../MainLib/OgreFltk/FlLayout.h"
#include "../MainLib/WrapperLua/LUAwrapper.h"
#include "../MainLib/WrapperLua/LUAwrapper.h"
#include "../MainLib/OgreFltk/framemoveobject.h"
#include "../MainLib/OgreFltk/timesensor.h"
#include "../MainLib/OgreFltk/AnimationObject.h"
#include "../MainLib/OgreFltk/FltkRenderer.h"
#include "../MainLib/OgreFltk/FltkAddon.h"
#include "../MainLib/OgreFltk/MotionPanel.h"
#include "../BaseLib/motion/MotionWrap.h"
#ifndef NO_GUI

#include <FL/fl_ask.H>
#include <FL/Fl_Browser.H>
#endif
#ifndef NO_OGRE
#include <Ogre.h>

#define OGRE_VOID(x) x
#define OGRE_PTR(x) x
#else
#define OGRE_VOID(x)
#define OGRE_PTR(x) return NULL
#endif
//#include <OgreViewport.h>
//#include <OgreSceneNode.h>
//#include <OgreSceneManager.h>
//#include <OgreEntity.h>
//#include <OgreOverlayManager.h>
//
void RE_outputRaw(const char* key, const char* output, int i);
void RE_dumpOutput(TStrings& output, int i);
void RE_outputEraseAll(int i);
void FastCapture_convert(const char* filename);
int FlGenShortcut(const char* s);

#ifndef NO_OGRE
#define BEGIN_OGRE_CHECK try {
#define END_OGRE_CHECK	} catch ( Ogre::Exception& e ) {Msg::msgBox(e.getFullDescription().c_str());}

#if OGRE_VERSION_MINOR>=9
#include "Overlay/OgreOverlay.h"
#include "Overlay/OgreOverlayManager.h"
#include "Overlay/OgreOverlayContainer.h"
#include "Overlay/OgreOverlayElement.h"
#else
#include "OgreOverlayContainer.h"
#include "OgreOverlayElement.h"
#endif
#include <OgreEntity.h>
#include <OgreAxisAlignedBox.h>

namespace Ogre
{

	Ogre::OverlayContainer* createContainer(int x, int y, int w, int h, const char* name) ;
	Ogre::OverlayElement* createTextArea(const String& name, Ogre::Real width, Ogre::Real height, Ogre::Real top, Ogre::Real left, uint fontSize, const String& caption, bool show) ;
}

Ogre::Overlay* createOverlay_(const char* name);
void destroyOverlay_(const char* name);
void destroyOverlayElement_(const char* name);
void destroyAllOverlayElements_();
Ogre::OverlayElement* createTextArea_(const char* name, double width, double height, double top, double left, int fontSize, const char* caption, bool show);
#endif
#include "../MainLib/WrapperLua/mainliblua_wrap.h"
#ifdef _MSC_VER
typedef unsigned short ushort;
#endif

		]]
		)
		writeIncludeBlock(true)
		write([[
					  class LMat;
					  class LMatView;
					  class LVec;
					  class LVecView;
		  ]])
		write('#include "luna_baselib.h"')
		write('#include "luna_mainlib.h"')
		write('#include "../BaseLib/motion/ASFLoader.h"')
		write('#include "../BaseLib/motion/BVHLoader.h"')
		write('#include "../MainLib/OgreFltk/VRMLloader.h"')
		writeDefinitions(bindTargetMainLib, 'Register_mainlib') -- input bindTarget can be non-overlapping subset of entire bindTarget 
		flushWritten(source_path..'/luna_mainlib.cpp') -- write to cpp file only when there exist modifications -> no-recompile.
	end

