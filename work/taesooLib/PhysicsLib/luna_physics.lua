
gen_lua.enum_types[#gen_lua.enum_types+1]='OpenHRP::DynamicsSimulator::IntegrateMethod'
gen_lua.enum_types[#gen_lua.enum_types+1]='OpenHRP::DynamicsSimulator::LinkDataType'

bindTargetPhysics={
	classes={
		{
			ifndef='EXCLUDE_GMBS_SIM',
			name='gmbs.RMatrix',
			className='RMatrix',
			ctors={ '()',
			'(int nr, int nc)'},
			memberFunctions={[[
			RMatrix operator + (const RMatrix &m) const
			RMatrix operator - (const RMatrix &m) const
			RMatrix operator * (const RMatrix &m) const
			]]},
			wrapperCode=[[
				static void assign(RMatrix& out, matrixn const& mat)
				{
					out.ReNew(mat.rows(), mat.cols());
					for(int i=0; i<mat.rows(); i++)
						for(int j=0; j<mat.cols(); j++)
							out(i,j)=mat(i,j);

				}
				static matrixn toMat(RMatrix const& in)
				{
					matrixn out;
					out.setSize(in.RowSize(), in.ColSize());

					for(int i=0; i<out.rows(); i++)
						for(int j=0; j<out.cols(); j++)
							out(i,j)=in(i,j);
					return out;
				}
			]],
			staticMemberFunctions={[[
				void assign(RMatrix& out, matrixn const& mat)
				matrixn toMat(RMatrix const& in)
				int Rank(const RMatrix &m);
				double Det(RMatrix A);
				RMatrix pInv(const RMatrix &A, double eps );								
				RMatrix srInv(const RMatrix &A, double alpha);									
				RMatrix Nullspace(const RMatrix &A, double eps );	
				RMatrix Zeros(int r, int c);
				RMatrix Ones(int r, int c);
				RMatrix Rand(int r, int c);
				RMatrix Eye(int r, int c );
			]]}
		},
		{
			name='Physics.InertiaCalculator',
			className='InertiaCalculator',
			decl='class InertiaCalculator;',
			ctors={'()','(int)'},
			memberFunctions={[[
			void calculateFromFile(const char* objfilename);
			void calculateFromMesh(OBJloader::Mesh const &mesh);
			void calculateFromBox(m_real sizeX, m_real sizeY, m_real sizeZ);	// sphere, cylinder, 
			void calculateFromCylinder(m_real radius, m_real height);
			void drawSamplingGrid(m_real radius, vector3 const& translate);
			const vector3& centerOfMass() const;
			const matrix3& inertia() const;	// at the center of mass.
			m_real volume() const;	// == mass assuming uniform and unit density.
			]]}
		},
		{
			name='Physics.MRDplot',
			className='MRDplot',
			properties={[[
			TStrings names;
			TStrings units;
			matrixn data;
			]]},
			ctors={'()'},
			memberFunctions={[[
			void initMRD(int n_channels, int numpoints, m_real frequency);
			int numPoints() const;
			int numChannels() const;
			void setAllUnits(const char* unitname);
			void load(const char* filename);
			void save(const char* filename) const;
			void addPoint(const vectorn& vv);
			void addChannel(const vectorn& vv, const char* name, const char* unit);
			]]}
			
		},
		{
			name='Physics.ContactBasis',
			className='OpenHRP::DynamicsSimulator_QP::ContactBasis',
			properties={'int ibody', 'int ibone', 'vector3 globalpos', 'vector3 relvel', 'vector3 normal', 
			'vector3N frictionNormal','double depth', 'int globalIndex', 'int globalFrictionIndex','int ilinkpair'},
		},
		{
			name='Physics.Vec_ContactBasis',
			className='std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis>',
			ctors={'()'},
			wrapperCode=[[
			static void assign(std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis> & out, std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis> const& in) 
			{
				out.resize(in.size());
				for(int i=0; i<in.size(); i++){ out[i]=in[i];}
			}
			]],
			staticMemberFunctions=[[
			void assign(std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis> & out, std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis> const& in) 
			]],
			memberFunctions={[[
									 int size();
									 OpenHRP::DynamicsSimulator_QP::ContactBasis& operator[](int i); @ __call
						 ]]}
			

		},
		{
			name='math.GrahamScan',
			className='GrahamScan',
			ctors={'()'},
			memberFunctions={
				[[
				void add_point(std::pair<double, double> const& point);
				void partition_points();
				void build_hull();
				void print_raw_points();
				void print_hull();
				void get_hull(matrixn& out);
				]]
			},
			wrapperCode=[[
			static void memTest()
			{
				auto* a=new OpenHRP::DynamicsSimulator_TRL_LCP("gjk");
				delete a;
			}
			]],
			staticMemberFunctions={
				[[
				static double GrahamScan::direction( std::pair<double,double> p0, std::pair<double,double> p1, std::pair<double,double> p2 ); 
				static void memTest()
				]]
			},
		},
		{
			name='Physics.ContactForce',
			className='OpenHRP::DynamicsSimulator::ContactForce',
			properties={
				'int chara',
				'VRMLTransform* bone',
				'vector3 f',
				'vector3 p',
				'vector3 tau',	
			}
		},
		{
			name='HessianQuadratic',
			ctors={'(int)'},
			properties={'matrixn H', 'vectorn R'},
			memberFunctions={[[
			void addSquared(intvectorn const& , vectorn const& );
			]]},
		},
		{
			name='CartPoleBallCpp',
			className='std::vector<float>', -- not yet wrapped.
		},
		{ 
			name='Physics.Vec_CFinfo',
			className='std::vector<OpenHRP::DynamicsSimulator::ContactForce>',
			ctors={'()'},
			memberFunctions={[[
			int size();
			]]},
			wrapperCode=[[
			static OpenHRP::DynamicsSimulator_QP::ContactBasis& __call2(std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis> const& in, int index)
			{
				return (OpenHRP::DynamicsSimulator_QP::ContactBasis&)(in[index]);
			}
			static OpenHRP::DynamicsSimulator::ContactForce & __call(std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in, int index)
			{
				return (OpenHRP::DynamicsSimulator::ContactForce &)(in[index]);
			}

			static void assign(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in) 
			{
				out.resize(in.size());
				for(int i=0; i<in.size(); i++){ out[i]=in[i];}
			}

			static void normalize(OpenHRP::DynamicsSimulator::ContactForce & cf2,
			OpenHRP::DynamicsSimulator::ContactForce const& cf,
			OpenHRP::DynamicsSimulator & sim)
			{
				Bone* b= cf.bone;
				vector3 gf=sim.getWorldState(cf.chara)._global(*b).toGlobalDir(cf.f);
				vector3 gp=sim.getWorldState(cf.chara)._global(*b).toGlobalPos(cf.p);
				vector3 gtau=gp.cross(gf)+sim.getWorldState(cf.chara)._global(*b).toGlobalDir(cf.tau);

				cf2.bone=cf.bone;
				cf2.chara=cf.chara;				
				cf2.f=sim.getWorldState(cf.chara)._global(*b).toLocalDir(gf);
				cf2.p=sim.getWorldState(cf.chara)._global(*b).toLocalPos(vector3(0,0,0));
				cf2.tau=sim.getWorldState(cf.chara)._global(*b).toLocalDir(gtau);
			}

			static void scale(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, 
			double s)
			{
				// assumes that cf has been normalized.
				for (int i=0; i<out.size(); i++){
					out[i].f*=s;
					out[i].tau*=s;
				}				
			}

			static void compaction(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in1, OpenHRP::DynamicsSimulator & sim)
			{
				out.reserve(in1.size());
				out.resize(0);

				for(int i=0; i<in1.size(); i++){

					if (in1[i].chara==0){// ignore chara2 assuming it's static object.
						if(out.size()==0 || out.back().bone!=in1[i].bone){
							OpenHRP::DynamicsSimulator::ContactForce cf;
							normalize(cf, in1[i], sim);
							out.push_back(cf);
						}
					else {
						OpenHRP::DynamicsSimulator::ContactForce cf;
						normalize(cf, in1[i], sim);

						OpenHRP::DynamicsSimulator::ContactForce& cf2=out.back();
						cf2.f+=cf.f;
						cf2.tau+=cf.tau;
					}
					}
				}
			}

			static void merge(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, 
			std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in1,
			std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in2)
			{
				// assumes that both in1 and in2 are normalized
				intvectorn indexes;

				for (int i=0; i<in1.size(); i++){
					Msg::verify(indexes.findFirstIndex(in1[i].bone->treeIndex())==-1, "index==-1");
					indexes.pushBack(in1[i].bone->treeIndex());
				}
				for (int i=0; i<in2.size(); i++){
					int idx=in2[i].bone->treeIndex();
					if(indexes.findFirstIndex(idx)==-1)
						indexes.pushBack(idx);
					}

					out.resize(indexes.size());
					for(int i=0; i<indexes.size(); i++){
						out[i].f.setValue(0,0,0);
						out[i].tau.setValue(0,0,0);
					}

					for (int i=0; i<in1.size(); i++){
						int idx=indexes.findFirstIndex(in1[i].bone->treeIndex());
						Msg::verify(idx!=-1, "idx ==-1");
						out[i].bone=in1[i].bone;
						out[i].chara=in1[i].chara;
						out[i].p=in1[i].p;
						out[i].f+=in1[i].f;
						out[i].tau+=in1[i].tau;
					}
					for (int i=0; i<in2.size(); i++){
						int idx=indexes.findFirstIndex(in2[i].bone->treeIndex());
						out[i].bone=in2[i].bone;
						out[i].chara=in2[i].chara;
						out[i].p=in2[i].p;
						out[i].f+=in2[i].f;
						out[i].tau+=in2[i].tau;
					}



				}

				static void interpolate(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, 					
				m_real t,
				std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in1,
				std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in2,
				OpenHRP::DynamicsSimulator & sim)
				{
					std::vector<OpenHRP::DynamicsSimulator::ContactForce> t1, t2;

					compaction(t1, in1, sim);
					scale(t1, 1-t);

					compaction(t2, in2, sim);
					scale(t2, t);

					printf("here\n");
					merge(out, t1, t2);
				}
				]]
				,staticMemberFunctions={[[

				static OpenHRP::DynamicsSimulator_QP::ContactBasis& __call2(std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis>const& in, int index)
				static OpenHRP::DynamicsSimulator::ContactForce & __call(std::vector<OpenHRP::DynamicsSimulator::ContactForce>const& in, int index)
				static void assign(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in)
				static void normalize(OpenHRP::DynamicsSimulator::ContactForce & cf2, OpenHRP::DynamicsSimulator::ContactForce const& cf, OpenHRP::DynamicsSimulator & sim)
				static void scale(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, double s)
				static void compaction(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in1, OpenHRP::DynamicsSimulator & sim)
				static void merge(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in1, std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in2)
				static void interpolate(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, m_real t, std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in1, std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in2, OpenHRP::DynamicsSimulator & sim)
				]]},
			},
			{
				name='Physics.RayTestResult',
				className='OpenHRP::CollisionDetector::RayTestResult',
				properties={
					'm_real m_closestHitFraction',
					'vector3 m_hitNormalWorld',
				},
				ctors={'()'},
				memberFunctions={[[
					bool hasHit() const;
					]]}
			},
			{
				name='Physics.CollisionPoint',
				className='OpenHRP::CollisionPoint',
				properties=[[
					vector3 position;
					vector3 normal;
					double	  idepth;
					]],
			},
			{
				name='Physics.CollisionPointSequence',
				className='OpenHRP::CollisionPointSequence',
				wrapperCode=[[
				static void erase(OpenHRP::CollisionPointSequence& self, int i)
				{ self.erase(self.begin()+i); }
				]],
				memberFunctions=[[
				OpenHRP::CollisionPoint& operator[](int i)
				int size()
				void resize(int n)
				]],
				staticMemberFunctions=[[
				void erase(OpenHRP::CollisionPointSequence& self, int i)
				]]
			},
			{
				name='Physics.CollisionSequence',
				className='OpenHRP::CollisionSequence',
				ctors={'()'},
				memberFunctions={[[
				OpenHRP::CollisionPointSequence& getCollisionPoints(int ilinkpair)
				int getNumLinkPairs() 
				intvectorn getCollisionLinkPairs() // array of ilinkpairs
				int getCharacterIndex1(int ilinkpair)
				VRMLTransform* getBone1(int ilinkpair)
				int getCharacterIndex2(int ilinkpair)
				VRMLTransform* getBone2(int ilinkpair)
				]]},
				staticMemberFunctions={[[
				OpenHRP::CollisionDetector* OpenHRP::createCollisionDetector_bullet(); 
				OpenHRP::CollisionDetector* OpenHRP::createCollisionDetector_gjk(); 
				OpenHRP::CollisionDetector* OpenHRP::createCollisionDetector_libccd();
				//OpenHRP::CollisionDetector* OpenHRP::createCollisionDetector_libccd_LBS();
				//OpenHRP::CollisionDetector* OpenHRP::createCollisionDetector_libccd_merged();
				OpenHRP::CollisionDetector* OpenHRP::createCollisionDetector_fcl();
				]]}
			},
			{
				name='Physics.CollisionDetector',
				className='OpenHRP::CollisionDetector',
				memberFunctions={[[
					int addModel(VRMLloader* loader); // returns characterIndex
					int addObstacle(OBJloader::Geometry const& mesh); // returns characterIndex
					VRMLloader* getModel(int ichar)  
					int numModels() const 
					void setMargin(int ilink, double margin);
					void setMarginAll(vectorn const & margin);
					void getMarginAll(vectorn & margin);
					virtual void addCollisionPair(VRMLloader* skel1, int ibone1, VRMLloader* skel2, int ibone2);
					virtual void setWorldTransformations(int charIndex, BoneForwardKinematics const& fk); 
					virtual void rayTest(int ichar, int ilink, vector3 const& from, vector3 const& to, OpenHRP::CollisionDetector::RayTestResult& result);
					virtual void rayTestBackside(int ichar, int ilink, vector3 const& from, vector3 const& to, OpenHRP::CollisionDetector::RayTestResult& result);
					virtual bool testIntersectionsForDefinedPairs(OpenHRP::CollisionSequence & collisions);
					bool getLocalBoundingBoxSize(int charIndex, int ibone, vector3& localSize) { return false;}
					virtual bool isSignedDistanceSupported() { return true;}
					virtual double calculateSignedDistance(int iloader, int ibody, vector3 const& position, vector3& normal);
					virtual bool isSphereTestSupported() 
					virtual double testSphereIntersection(int iloader, int ibody, vector3 const& position, double radius, vector3& contactpos, vector3& normal) 
					]]},
			},
			{
			   name='Physics.CollisionDetector_libccd',
			   className='OpenHRP::CollisionDetector_libccd',
			   inheritsFrom='OpenHRP::CollisionDetector',
			   decl=[[
			   namespace OpenHRP{
				   vector2 testSphereBox(vector3 const& center, double r, transf const& box, vector3 const& box_size, vector3& pos, vector3& normal);
			   }]],
			   staticMemberFunctions={[[
					bool OpenHRP::CollisionCheck(OpenHRP::CollisionDetector &s,OpenHRP::CollisionSequence & collisions, std::string chekmesh, std::string skipmesh);
					vector2 OpenHRP::testSphereBox(vector3 const& center, double r, transf const& box, vector3 const& box_size, vector3& pos, vector3& normal);
				]]},
			},
			{
				name='Physics.DynamicsSimulator',
				className='OpenHRP::DynamicsSimulator',
				wrapperCode=[[

				static void setParameter2(const char* _what, bool value)
				{
					TString what(_what);

					if(what=="usePenaltyMethod")
						OpenHRP::globals::usePenaltyMethod=value;
					else
						Msg::error("unknown parameter %s", _what);
					}

					static void setParameter(const char* _what, double value)
					{
						TString what(_what);

						Msg::error("unknown parameter %s", _what);
					}

					static int setSimulatorParam(lua_State *L)
					{/*OpenHRP::DynamicsSimulator&s, const char* t, luabind::object const& ll)
					int count=LUAwrapper::arraySize(ll);
					vectorn l;
					l.setSize(count);
					for(int i=0; i<count; i++)
						l[i]=luabind::object_cast<double>(ll[i+1]);	// lua indexing starts from 1.

						s.setSimulatorParam(t, l);
						*/
						return 0;
					}
					static vector3 calcCOM(OpenHRP::DynamicsSimulator&s, int ichara)
					{
						double totalMass;
						return s.calculateCOM(ichara,totalMass);
					}
					static double calcTotalMass(OpenHRP::DynamicsSimulator&s, int ichara)
					{
						double totalMass;
						s.calculateCOM(ichara,totalMass);
						return totalMass;
					}
					static vector3 calcCOMvel(OpenHRP::DynamicsSimulator&s, int ichara)
					{
						double totalMass;
						return s.calculateCOMvel(ichara,totalMass);
					}
					static void setPose(PLDPrimVRML& prim, OpenHRP::DynamicsSimulator& s, int ichara)
					{
						/*
						static Posture pose;
						OpenHRP::DynamicsSimulator::Character* c=s._characters[ichara];
						c->chain->getPoseFromLocal(pose);
						//			printf("y=%s \n", c->chain->local(5).translation.output().ptr());
						//			printf("y=%s \n", c->chain->global(5).translation.output().ptr());
						prim.SetPose(pose, *c->skeleton);
						*/
						prim.setPose(s.getWorldState(ichara));
					}
					static void setPose(PLDPrimVRML& prim, Posture const& pose)
					{
						prim.setPose(pose);
					}
					]],
					staticMemberFunctions={[[
					static void setParameter2(const char* _what, bool value) @ setParameter
					static void setParameter(const char* _what, double value)
					static vector3 calcCOM(OpenHRP::DynamicsSimulator&s, int ichara) @ calculateCOM
					static vector3 calcCOMvel(OpenHRP::DynamicsSimulator&s, int ichara) @ calculateCOMvel
					static double calcTotalMass(OpenHRP::DynamicsSimulator&s, int ichara)
					static void setPose(PLDPrimVRML& prim, OpenHRP::DynamicsSimulator& s, int ichara) 
					static void setPose(PLDPrimVRML& prim, Posture const& pose)
					]]},
					memberFunctions={[[	
					void drawLastContactForces();
					void drawLastContactForces(int ichara);
					void drawLastContactForces(int ichara, vector3 const& draw_offset);
					vector3 getContactForce(int ichar, int ibone) const;
					void calcInertia(int ichara,vectorn const& pose, vectorn& inertia) const
					Liegroup::dse3 calcMomentumCOMfromPose(int ichara, double t, vectorn const& poseFrom, vectorn const& poseTo)
					Liegroup::dse3 calcMomentumCOM(int ichara);
					void setPoseDOF(int ichara, vectorn const& v)
					void setDPoseDOF(int ichara, vectorn const& v)
					void getPoseDOF(int ichara, vectorn & v) 
					void getDPoseDOF(int ichara, vectorn & v) 
					vectorn getPoseDOF(int ichara) const 
					vectorn getDPoseDOF(int ichara) const 
					OpenHRP::CollisionDetector* getCollisionDetector();
					OpenHRP::CollisionSequence* getCollisionSequence();
					void drawDebugInformation() 
					void registerCharacter(VRMLloader*l);
					void createObstacle(OBJloader::Geometry const& mesh);
					void createFreeBody(OBJloader::Geometry const& mesh);
					void registerCollisionCheckPair( int ichara1, const char* name1, int ichara2, const char* name2, vectorn const& param); 
					void registerCollisionCheckPair( const char* char1, const char* name1, const char* char2, const char* name2, vectorn const& param); 
					void init( double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt)
					void setTimestep(double timeStep)
					double getTimestep()
					void setGVector(const vector3& wdata);
					void initSimulation();
					int rdof(int ichar) const 
					int dof(int ichar) const 
					int numSphericalJoints(int ichara) const;
					void getWorldPosition(int ichara, VRMLTransform* b, vector3 const& localpos, vector3& globalpos) const;
					vector3 getWorldPosition(int ichara, VRMLTransform* b, vector3 const& localpos) const;
					vector3 getWorldVelocity(int ichara,VRMLTransform* b, vector3 const& localpos) const;
					vector3 getWorldAcceleration(int ichara,VRMLTransform* b, vector3 const& localpos) const;
					vector3 getWorldAngVel(int ichara, VRMLTransform* b) const;
					vector3 getWorldAngAcc(int ichara, VRMLTransform* b) const;
					BoneForwardKinematics& getWorldState(int ichara) ;	
					VRMLloader & skeleton(int ichara)
					std::string name(int ichara)
					int findCharacter(const char* _name) const 
					int numSkeleton() const ;
					void setWorldState(int ichara);		
					void calcJacobianAt(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos);
					void calcDotJacobianAt(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos);
					void calcDotJacobian(int ichar, int ibone, matrixn& dotjacobian)
					void setLinkData(int i, OpenHRP::DynamicsSimulator::LinkDataType t, vectorn const& in);
					void getLinkData(int i, OpenHRP::DynamicsSimulator::LinkDataType t, vectorn& out);	
					bool stepSimulation();
					double currentTime();
					void setCurrentTime(double t);
					vector3 calculateZMP(int ichara);
					void registerContactQueryBone(int contactQueryIndex, VRMLTransform* bone);
					bool queryContact(int index);
					vectorn queryContacts();
					vectorn queryContactDepths();
					void _updateContactInfo();
					vectorn const& getLastSimulatedPose(int ichara);
					std::vector<OpenHRP::DynamicsSimulator::ContactForce> & queryContactAll();
					void addForceToBone(int ichara, VRMLTransform* b, vector3 const& localpos, vector3 const& localforce);
					void addGlobalForceToBone(int ichara, int ibone, vector3 const& globalpos, vector3 const& globalforce);
					virtual void calcMassMatrix(int ichara,matrixn& ,vectorn&);

					// for QPservo
					int getNumAllLinkPairs();
					void getContactLinkBoneIndex(int ipair, intvectorn & ibone);
					void addRelativeConstraint(int ichara, Bone& bone1,vector3 boneVector1,Bone& bone2, vector3 boneVector2);
					void removeRelativeConstraint(int ichara, Bone& bone1, Bone& bone2)	


					void setQ(int ichara, vectorn const& v);
					void getQ(int ichara, vectorn & v) const ;
					vectorn getQ(int ichara) const 
					void setDQ(int ichara, vectorn const& v);
					void getDQ(int ichara, vectorn& v) const;
					void getBodyDQ(int ichara, vectorn& v) const;
					vectorn getDQ(int ichara) const 
					void setU(int ichara, const vectorn& in);

					vectorn poseToSphericalQ(int ichara, vectorn const& posedof) const;
					vectorn dposeToSphericalDQ(int ichara, vectorn const& dposeDOF) const;
					]]},
					enums={
						{"EULER","(int)OpenHRP::DynamicsSimulator::EULER"},
						{"RUNGE_KUTTA","(int)OpenHRP::DynamicsSimulator::RUNGE_KUTTA"},
						{"JOINT_VALUE","(int)OpenHRP::DynamicsSimulator::JOINT_VALUE"},
						{"JOINT_VELOCITY","(int)OpenHRP::DynamicsSimulator::JOINT_VELOCITY"},
						{"JOINT_ACCELERATION","(int)OpenHRP::DynamicsSimulator::JOINT_ACCELERATION"},
						{"JOINT_TORQUE","(int)OpenHRP::DynamicsSimulator::JOINT_TORQUE"}
					},
					customFunctionsToRegister={'setSimulatorParam'},
					--properties={'vectorn _tempPose; @ _lastSimulatedPose'} -- use getLastSimulatedPose(0) instead.
			},
			{
				name='Physics.DynamicsSimulator_penaltyMethod',
				className='OpenHRP::DynamicsSimulator_penaltyMethod',
				inheritsFrom='OpenHRP::DynamicsSimulator',
			},
			{
				ifndef='EXCLUDE_GMBS_SIM',
				name='Physics.DynamicsSimulator_gmbs_penalty',
				className='OpenHRP::DynamicsSimulator_gmbs_penalty',
				inheritsFrom='OpenHRP::DynamicsSimulator_penaltyMethod',
				--isLuaInheritable=true,
				isExtendableFromLua=true,
				ctors={'()'},
				memberFunctions={[[
				void calcJacobian(int ichar, int ibone, matrixn& jacobian);
				Liegroup::dse3 calcMomentumCOMfromPoseUpper(int ichara, double t, vectorn const& poseFrom, vectorn const& poseTo)
				Liegroup::dse3 calcMomentumCOMfromPoseLower(int ichara, double t, vectorn const& poseFrom, vectorn const& poseTo)
				void addForceToBone(int ichara, VRMLTransform* b, vector3 const& localpos, vector3 const& force);
				void test(const char* test, matrixn& out);
				void calcBoneDotJacobian(int ichar, int ibone, vector3 const& localpos,matrixn& jacobian, matrixn& dotjacobian);
				void calcBoneDotJacobian2(int ichar, int ibone, vector3 const& localpos,matrixn& jacobian, matrixn& dotjacobian);
				void calcMassMatrix(int ichara, matrixn& M, vectorn& b) @ calcMassMatrix3
				void getBodyVelocity(int chara, VRMLTransform* b, Liegroup::se3& V) const
				]]}
			},
			{
				ifndef='EXCLUDE_GMBS_SIM',
				name='Physics.DynamicsSimulator_gmbs',
				className='OpenHRP::DynamicsSimulator_gmbs',
				inheritsFrom='OpenHRP::DynamicsSimulator',
				ctors={'()','(bool)', '(const char*)'},
				decl=[[
				namespace OpenHRP {
				class DynamicsSimulator_gmbs;
				}
				vector3 calcCOM2_gmbs(OpenHRP::DynamicsSimulator_gmbs&s, int ichara);
				]],
				staticMemberFunctions={[[
				vector3 calcCOM2_gmbs(OpenHRP::DynamicsSimulator_gmbs&s, int ichara)
				]]},
				memberFunctions={[[
				void setCollisionMargin(int ilinkpair, double a)
				void setAllCollisionMargin(vectorn const& a)
				void getAllCollisionMargin(vectorn & a)
				Liegroup::dse3 calcMomentumGlobal(int ichara);
				// ys start
				void stepKinematicMuscle_integrate(vectorn const& ddq, double dt);
				void stepKinematicMuscle_updateother();
				// ys end
				bool stepKinematic(vectorn const& ddq, vectorn const& tau, bool );
				//ys
				double calcKineticEnergy() const
				vector3 calculateCOMacc(int ichara);
				void getLCPmatrix(matrixn& A, vectorn& b);
				void getLCPsolution(vectorn& out);
				void calcContactJacobianAll(matrixn &J_all, matrixn & dotJ_all, matrixn& V_all, matrixn & dot_v_all, int link_pair_count, double invfrictionCoef);
				void calcJacobian(int ichar, int ibone, matrixn& jacobian);
				void calcDotJacobian(int ichar, int ibone, matrixn& dotjacobian);
				void _calcDotBodyJacobian(int ichar, int ibone, matrixn& jacobian, matrixn& dotjacobian, bool update);
				void calcCOMjacobian(int ichar, matrixn& jacobian);
				void calcMomentumDotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian)
				Liegroup::dse3 calcMomentumCOMfromPoseUpper(int ichara, double t, vectorn const& poseFrom, vectorn const& poseTo)
				Liegroup::dse3 calcMomentumCOMfromPoseLower(int ichara, double t, vectorn const& poseFrom, vectorn const& poseTo)
				void calcInertiaUpper(int ichara,vectorn const& pose, vectorn& inertia) const
				void calcInertiaLower(int ichara,vectorn const& pose, vectorn& inertia) const
				void setInertia(int ichara, int ibone, vector3 const& localCOM, vector3 const& inertia);
				void calcBoneDotJacobian(int ichar, int ibone, vector3 const& localpos,matrixn& jacobian, matrixn& dotjacobian);
				void calcBoneDotJacobian2(int ichar, int ibone, vector3 const& localpos,matrixn& jacobian, matrixn& dotjacobian);
				void calcCOMdotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian);
				void calcMassMatrix2(int ichara,matrixn&, vectorn& );
				void calcMassMatrix3(int ichara,matrixn&, vectorn& );
				void test(const char* test, matrixn& out);
				void collectExternalForce(int ichara, vectorn & extForce)
				// QP
				void getContactBases(std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis>& basis, double invfrictionCoef) const;
				void calcContactBasisAll(matrixn& v_all, matrixn & dot_v_all, int link_pair_count, double invfrictionCoef);
				void calcContactBoneIndex(int link_pair_count, intvectorn& boneIndex);
				int getNumContactLinkPairs();
				void getLinkPairBodiesBones(intvectorn& ibodies, intvectorn& ibones) const;
				void getBodyVelocity(int chara, VRMLTransform* b, Liegroup::se3& V) const
				]]}
			},
			{
				ifndef='EXCLUDE_AIST_SIM',
				name='Physics.DynamicsSimulator_AIST_penalty',
				className='OpenHRP::DynamicsSimulator_AIST_penalty',
				inheritsFrom='OpenHRP::DynamicsSimulator_penaltyMethod',
				ctors={'()'}
			},
			{
				ifndef='EXCLUDE_AIST_SIM',
				name='Physics.DynamicsSimulator_AIST',
				className='OpenHRP::DynamicsSimulator_impl',
				inheritsFrom='OpenHRP::DynamicsSimulator',
				ctors={'()','(const char*)'},
				memberFunctions={[[
				]]}
			},
			{
				ifndef='EXCLUDE_UT_SIM',
				name='Physics.DynamicsSimulator_UT',
				className='OpenHRP::DynamicsSimulator_UT',
				inheritsFrom='OpenHRP::DynamicsSimulator',
				ctors={'()', '(const char*)'},
			},
			{
				name='Physics.DynamicsSimulator_TRL_penalty',
				className='OpenHRP::DynamicsSimulator_TRL_penalty',
				inheritsFrom='OpenHRP::DynamicsSimulator_penaltyMethod',
				ctors={'()','(const char*)'},
				memberFunctions=[[
				void addWorldTorqueToBone(int ichara, VRMLTransform* b, vector3 const& world_torque);
				void calcBodyJacobianAt(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos);
				void calcDotBodyJacobianAt(int ichar, int ibone, matrixn& jacobian, matrixn& dotjacobian, vector3 const& localpos);
				void calcMomentumDotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian)
				void poseToQ(vectorn const& v, vectorn& out)
				void dposeToDQ(quater const& rootOri, vectorn const& v, vectorn& out)
				void torqueToU(const vectorn& cf, vectorn& U)
				void QToPose(vectorn const& v, vectorn& out)
				void setNonStatePoseDOF(int ichara, vectorn const& v);
				void setNonStateDQ(int ichara, vectorn const& dq);
				void setNonStateDDQ(int ichara, vectorn const& ddq);
				void getSphericalState(int ichara, vectorn & q, vectorn& dq); 
				void setSphericalState(int ichara, const vectorn& q, const vectorn& dq); 
				void setTau(int ichara, const vectorn& tau); 
				transf getNonStateRootQ(int ichara);
				void setState(int ichara, vectorn const& v)  
				void getState(int ichara, vectorn & v) const 
				void setDDQ(int ichara, vectorn const& v);
				void getU(int ichara, vectorn& out);
				vectorn getU(int ichara);
				int calcS(int ichara, int ibone, matrixn& S);
				void stateToEulerZYX(vectorn const& q, vectorn const& dq, vectorn& eulerState);
				void stateToEulerYXZ(vectorn const& q, vectorn const& dq, vectorn& eulerState);
				void eulerZYXtoState(vectorn const& eulerState, vectorn& state) const;
				void eulerYXZtoState(vectorn const& eulerState, vectorn& state) const;
				void setStablePDparam(int ichara, const vectorn& kp, const vectorn& kd);
				void calculateStablePDForces(int ichara, const vectorn& desired_q, vectorn& tau);
				]]
			},
			{
				ifndef='EXCLUDE_RBDL_SIMULATOR',
				name='Physics.DynamicsSimulator_Trbdl_penalty',
				className='Trbdl::DynamicsSimulator_Trbdl_penalty',
				inheritsFrom='OpenHRP::DynamicsSimulator_penaltyMethod',
				ctors={'(const char*)'},
				memberFunctions=[[
				void setNonStatePoseDOF(int ichara, vectorn const& v);
				void setNonStateDQ(int ichara, vectorn const& dq);
				void setNonStateDDQ(int ichara, vectorn const& ddq);
				transf getNonStateRootQ(int ichara);
				int getSphericalState(int ichara, vectorn& q, vectorn& dq);
				void setSphericalState(int ichara, const vectorn& q, const vectorn& dq);
				void setTau(int ichara, const vectorn& tau);
				void calcMomentumDotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian);

				void _calcMassMatrix(int ichara, matrixn& out, vectorn & b);
				void _calcJacobianAt(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos);
				void _enableDotJocobianComputation(int ichara)
				void _calcDotJacobianAt(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos);
				vectornView _Q(int ichara);
				vectornView _QDot(int ichara);
				vector3 _bodyW(int ichara, int treeIndex) const;
				vector3 _bodyV(int ichara, int treeIndex) const;
				void _stepKinematic(int ichara, vectorn const& QDDot);
				void setStablePDparam(int ichara, const vectorn& kp, const vectorn& kd);
				void calculateStablePDForces(int ichara, const vectorn& desired_q, vectorn& tau);
				void setStablePDparam_dof(int ichara, const vectorn& kp, const vectorn& kd);
				void calculateStablePDForces_dof(int ichara, const vectorn& desired_pose, const vectorn& desired_dpose, vectorn& tau)
				]],
			},
			{
				ifndef='EXCLUDE_RBDL_SIMULATOR',
				name='Physics.DynamicsSimulator_Trbdl_LCP',
				decl=[[
				namespace Trbdl {
				class DynamicsSimulator_Trbdl_LCP;
				}
				]],
				className='Trbdl::DynamicsSimulator_Trbdl_LCP',
				inheritsFrom='Trbdl::DynamicsSimulator_Trbdl_penalty',
				ctors={'()','(const char*)'},
				memberFunctions={[[
				const matrixn& getMLCP() const;
				const vectorn& getMLCP_B() const;
				const vectorn& getMLCP_X() const;
				void setParam_Epsilon_Kappa(double eps, double kap)
				void setParam_R_B_MA(double r, double b, double ma)
				void stepSimulation()
				void addRelativeConstraint(int ichara, Bone& bone1,vector3 boneVector1,Bone& bone2, vector3 boneVector2);
				void removeRelativeConstraint(int ichara, Bone& bone1, Bone& bone2)	
				Liegroup::dse3 getCOMbasedContactForce(int ichar, int ibone) const
				]]},
			},
			{
				ifndef='EXCLUDE_RBDL_SIMULATOR',
				name='Physics.DynamicsSimulator_Trbdl_impulse',
				decl=[[
				namespace Trbdl {
				class DynamicsSimulator_Trbdl_impulse;
				}
				]],
				className='Trbdl::DynamicsSimulator_Trbdl_impulse',
				inheritsFrom='Trbdl::DynamicsSimulator_Trbdl_penalty',
				ctors={'(const char*)'},
				memberFunctions={[[
				void setParam_restitution_MA(double r, double ma)
				void stepSimulation()
				]]},
			},
			{
				ifndef='EXCLUDE_RBDL_SIMULATOR',
				name='Physics.DynamicsSimulator_Trbdl_QP',
				className='Trbdl::DynamicsSimulator_Trbdl_QP',
				inheritsFrom='Trbdl::DynamicsSimulator_Trbdl_penalty',
				ctors={'(const char*)'},
				memberFunctions=[[
				// QP
				void getContactBases(std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis>& basis, double invfrictionCoef) const;
				void calcContactBasisAll(matrixn& v_all, matrixn & dot_v_all, int link_pair_count, double invfrictionCoef);
				void calcContactBoneIndex(int link_pair_count, intvectorn& boneIndex);
				int getNumContactLinkPairs();
				void getLinkPairBodiesBones(intvectorn& ibodies, intvectorn& ibones) const;
				]]
			},
			{
				name='Physics.DynamicsSimulator_TRL_LCP',
				className='OpenHRP::DynamicsSimulator_TRL_LCP',
				inheritsFrom='OpenHRP::DynamicsSimulator_TRL_penalty',
				ctors={'()','(bool)','(const char*)'},
				memberFunctions=[[
				void stepKinematic(int ichar, vectorn const& dq)
				void setParam_Epsilon_Kappa(double eps, double kap)
				void setParam_R_B_MA(double r, double b, double ma)
				void setDamping(double linearDamping, double angularDamping)
				void setDamping()
				Liegroup::dse3 getCOMbasedContactForce(int ichar, int ibone) const
				]]
			},
			{
				name='Physics.DynamicsSimulator_TRL_softbody',
				ifdef='USE_SOFTBODY',
				decl=[[
				namespace TRL {
				class DynamicsSimulator_TRL_softbody;
				}
				]],
				className='TRL::DynamicsSimulator_TRL_softbody',
				inheritsFrom='OpenHRP::DynamicsSimulator_TRL_penalty',
				ctors={'()'},
				memberFunctions={[[
				void registerLBScharacter(VRMLloader* l, SkinnedMeshFromVertexInfo * info);
				void registerCollisionCheckPair_Rigid_vs_LBS(int bodyIndex1, int treeIndex1, int bodyIndex2, vectorn const& param);
				void registerCollisionCheckPair_LBS_vs_LBS(int bodyIndex1, int bodyIndex2, vectorn const& param);
				void init(double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt);
				void setParam_Epsilon_Kappa(double eps, double kap)
				void setParam_R_B_MA(double r, double b, double ma)
				void stepSimulation()
				void addRelativeConstraint(int ichara, Bone& bone1,vector3 boneVector1,Bone& bone2, vector3 boneVector2);
				void removeRelativeConstraint(int ichara, Bone& bone1, Bone& bone2)	
				]]},
			},
			{
				name='Physics.DynamicsSimulator_TRL_QP',
				className='OpenHRP::DynamicsSimulator_TRL_QP',
				inheritsFrom='OpenHRP::DynamicsSimulator_TRL_penalty',
				ctors={'(const char*)'},
				memberFunctions=[[
				void skipIntegration();
				void stepKinematic(int ichar, vectorn const& dq)
				void stepKinematic2(int ichar, vectorn const& ddq)
				void setParam_Epsilon_Kappa(double eps, double kap)
				void setParam_R_B_MA(double r, double b, double ma)
				int getNumContactLinkPairs();
				int getNumAllLinkPairs();
				// QP
				void getContactBases(std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis>& basis, double invfrictionCoef) const;
				void calcContactBasisAll(matrixn& v_all, matrixn & dot_v_all, int link_pair_count, double invfrictionCoef);
				void calcContactBoneIndex(int link_pair_count, intvectorn& boneIndex);
				int getNumContactLinkPairs();
				void getLinkPairBodiesBones(intvectorn& ibodies, intvectorn& ibones) const;
				]]
			},

	},
	modules={
		{
			namespace='MotionUtil',
			ifndef='NO_GUI',
			functions={[[
			//void VRMLloader_setTotalMass(VRMLloader & l, m_real totalMass);
			void VRMLloader_checkMass(VRMLloader& l);
			]]}
		},
		{
			namespace='MotionUtil',
			ifndef='EXCLUDE_UT_SIM',
			functions={[[
			MotionUtil::FullbodyIK_MotionDOF* createFullbodyIk_UTPoser(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors); @ createFullbodyIkDOF_UTPoser ;adopt=true;
			void MotionUtil::FullbodyIK_UTPoser_setParam(MotionUtil::FullbodyIK_MotionDOF* solver, int numiter, double stepsize)
			]]}
		}, 
		{
			namespace='Physics',
			functions={
						[[
						void lqr (matrixn &k, matrixn const& a, matrixn const&b, matrixn &q, matrixn const&r) @ LQR
						]]
			}
		}
	}
}


function generatePhysicsBind() 
	write(
	[[
	#include "../BaseLib/baselib.h"
	#include "../MainLib/MainLib.h"
	#include "../MainLib/OgreFltk/FlLayout.h"
	//#include "RigidBodyWin.h"
	#include "../BaseLib/utility/scoped_ptr.h"
	#include "../BaseLib/utility/configtable.h"
	#include "../BaseLib/utility/operatorString.h"
	//#include "../MainLib/WrapperLua/BaselibLUA.h"
	//#include "../MainLib/WrapperLua/MainlibLUA.h"
	#include "../MainLib/OgreFltk/VRMLloader.h"
	#include "../MainLib/OgreFltk/VRMLloaderView.h"


	#include "../BaseLib/motion/FullbodyIK_MotionDOF.h"

	namespace MotionUtil
	{
		FullbodyIK_MotionDOF* createFullbodyIk_UTPoser(MotionDOFinfo const& info, std::vector<Effector>& effectors);
		void FullbodyIK_UTPoser_setParam(MotionUtil::FullbodyIK_MotionDOF* solver, int numiter, double stepsize);
	}

	#ifndef EXCLUDE_GMBS_SIM
	#include "GMBS_implementation/DynamicsSimulator_gmbs_penalty.h"
	#include "GMBS_implementation/DynamicsSimulator_gmbs.h"
	#endif
	//#include "Bullet_implementation/btDynamicsSimulator_impl.h"
	#include "CollisionDetector.h"
	//#include "RMatrixLUA.h"

	#include "DynamicsSimulator_penaltyMethod.h"
	#include "../BaseLib/motion/InertiaCalculator.h"
	#include "../BaseLib/math/OperatorStitch.h"
	#include "clapack_wrap.h"
	#include "SDRE.h"
	#ifndef EXCLUDE_GMBS_SIM
	#include "GMBS_implementation/rmatrix3j.h"
	#endif
	class GrahamScan;
        #include "CollisionDetector/CollisionDetector_libccd.h"

	namespace OpenHRP {
		class DynamicsSimulator_impl;
		class DynamicsSimulator_UT;
		class DynamicsSimulator_AIST_penalty;
		class DynamicsSimulator_TRL;
		class DynamicsSimulator_TRL_LCP;
		class DynamicsSimulator_TRL_QP;
		class DynamicsSimulator_TRL_penalty;
		class CollisionDetector_libccd;
		OpenHRP::CollisionDetector* createCollisionDetector_bullet();
		OpenHRP::CollisionDetector* createCollisionDetector_fcl();
		OpenHRP::CollisionDetector* createCollisionDetector_gjk();
		OpenHRP::CollisionDetector* createCollisionDetector_libccd();
		//OpenHRP::CollisionDetector* createCollisionDetector_libccd_LBS();
		//OpenHRP::CollisionDetector* createCollisionDetector_libccd_merged();
		bool CollisionCheck(OpenHRP::CollisionDetector &s, OpenHRP::CollisionSequence & collisions, std::string chekmesh, std::string skipmesh);
	}
	#ifndef EXCLUDE_RBDL_SIMULATOR
	namespace Trbdl {
		class DynamicsSimulator_Trbdl_penalty;
		class DynamicsSimulator_Trbdl_LCP;
		class DynamicsSimulator_Trbdl_QP;
		class DynamicsSimulator_Trbdl_impulse;
	}
	#endif
	class MRDplot;

	
	]])
	writeHeader(bindTargetPhysics)
	flushWritten(script_path..'/../../PhysicsLib/luna_physics.h') -- write to cpp file only when there exist modifications -> no-recompile.
	writeIncludeBlock()
	write([[
	#include "physicsLib.h"
	#ifndef EXCLUDE_UT_SIM
	#include "DynamicsSimulator_UT.h"
	#endif
	#ifndef EXCLUDE_AIST_SIM
	#include "AIST_implementation/DynamicsSimulator_impl.h"
	#include "AIST_implementation/DynamicsSimulator_AIST_penalty.h"
	#endif
	#include "TRL/DynamicsSimulator_TRL_penalty.h"
	#include "TRL/DynamicsSimulator_TRL_LCP.h"
	#ifdef USE_SOFTBODY
	#include "TRL/DynamicsSimulator_TRL_softbody.h"
	#endif
	#include "TRL/DynamicsSimulator_TRL_QP.h"
	#ifndef EXCLUDE_RBDL_SIMULATOR
	#include "rbdl/DynamicsSimulator_Trbdl_penalty.h"
	#include "rbdl/DynamicsSimulator_Trbdl_LCP.h"
	#include "rbdl/DynamicsSimulator_Trbdl_impulse.h"
	#include "rbdl/DynamicsSimulator_Trbdl_QP.h"
	#endif
	]])
	write('// 만약 luna_physics.h가 없다는 컴파일 에러가 나면, taesooLib/PhysicsLib/luna_physics.cpp 지우고 다시 컴파일 하세요. ')
	write('#include "luna_physics.h"')
	write('#include "../MainLib/WrapperLua/luna.h"')
	write('#include "../MainLib/WrapperLua/luna_baselib.h"')
	write('#include "../MainLib/WrapperLua/luna_mainlib.h"')
	write([[
	#include "../PhysicsLib/convexhull/graham.h"
	#include "../PhysicsLib/mrdplot.hpp"
	]])
	write([[
	//void VRMLloader_setTotalMass(VRMLloader & l, m_real totalMass);
	void VRMLloader_checkMass(VRMLloader& l);
	]])
	writeDefinitions(bindTargetPhysics, 'Register_physicsbind',
	[[
	MainLib.VRMLloader.checkMass=MotionUtil.checkMass
	]]
	) -- input bindTarget can be non-overlapping subset of entire bindTarget 
	flushWritten(script_path..'/../../PhysicsLib/luna_physics.cpp') -- write to cpp file only when there exist modifications -> no-recompile.
end

function generate()
	loadDefinitionDB(script_path..'/luna_baselib_db.lua')
	buildDefinitionDB(bindTargetPhysics)
	generatePhysicsBind()


	writeDefinitionDBtoFile(source_path..'/luna_physicslib_db.lua')
end
