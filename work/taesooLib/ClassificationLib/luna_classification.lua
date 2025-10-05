
array.pushBack(gen_lua.enum_types, 'GMMCluster::Option') 
bindTargetClassification={
	classes={
		{
			decl='class gnuPlotQueue;',
			name='gnuPlotQueue',
			ctors={
				'(const char* fn, int dim, const char* title, const char* xlabel, const char* ylabel, const char* zlabel)',
				'(const char* fn, int dim, const char* title)',
				'(const char* fn, int dim)'},
			memberFunctions=[[
			void setRange(const intervals& range);
			void plotSignal(const vectorn& data, const char* label);
			void plotScattered(const matrixn& data, const char* label);
			void plotParametric(const matrixn& data, const char* label);
			]]
		},
		{
			name='TArray_vectorn',
			cppname='TArray<vectorn>',
			memberFunctions={[[
			int size() const;
			vectorn& data(int i); @ __call
			]]}
		},
		{
			name='MotionClustering.CSVMWrap',
			cppname='CSVMWrap',
			ctors={'(matrixn& data, bool bNormalize)'},
			memberFunctions=[[
					void AddTrainingData(int hData, int hClass)		
					int GetPredictedClass(int hData)			
					int GetUserSpecifiedClass(int hdata)	
					void Train(const char* option);
					int Predict(const vectorn & datum);
					void TrainAndPredict(const char* option);
			]],
		},
		{
			ifdef='USE_CLUSTERING',
			name='MotionClustering.FeatureExtractor',
			decl='namespace MotionClustering { class FeatureExtractor;}',
			memberFunctions={[[
					int GetFeatureDimension() const
					void CalcFeature(int startFrame, int endFrame, vectorn& vecFeature) const
					]]},
		},
		{
			decl='class KNearestInterpolationFast;',
			name='KNearestInterpolationFast',
			ctors={
				'(int,float power)',
				'(int,float power, float noiseWeight)'
			},
			memberFunctions=[[
				virtual void learn(const matrixn& sources, const matrixn& targets)	
				virtual bool isLearned() const 
				int dimDomain() const		
				int dimRange() const		
				void mapping(const vectorn& source, vectorn& target) const ;
				void mapping(const vectorn& source, vectorn& target, intvectorn & index, vectorn& weights) const;
				m_real logProb(const vectorn& source) const							
			]]
		},
		{
			decl='class RBInterpolation;',
			name='RBInterpolation',
			ctors={ '()' },
			memberFunctions=[[
				void initialize(const matrixn& sourceSamples);
				void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
				void calcWeight(const vectorn& parameter, intvectorn& index, vectorn& weight);
				const vectorn& minimum() const
				const vectorn& maximum() const
				const vectorn& mean() const	  
				const vectorn& stddev()	const 
				int dimension()	const						
				int numSample()	const					
				vectornView sample(int i) const		
				const matrixn& allSample() const
			]]
		},
		{
			name='MotionClustering.Clustering',
			cppname='Clustering',
			ctor='()',
			wrapperCode=[[
			static int cluster(lua_State* L)
			{
				Clustering& self=*luna_t::check(L,1);
				int n=lua_gettop(L)-1;
				TArray<vectorn> inputvec;

				lunaStack l(L);
				inputvec.resize(n);
				for (int i=0;i<n; i++)
					inputvec[i]=* l.topointer<vectorn>(i+2);
				self.cluster(inputvec);
				return 0;
			}
			]],
			customFunctionsToRegister ={'cluster'},
			memberFunctions=[[
				int numGrp()	
				int groupIndex(int elt)
				intvectorn& groupIndex()
				void plot(matrixn const& samples,const char* fn);
			]],
		},
		{
			name='MotionClustering.KMeanCluster',
			cppname='KMeanCluster',
			ctor='(int numCluster)',
			inheritsFrom='Clustering',
		},
		{
			name='MotionClustering.GMMCluster',
			cppname='GMMCluster',
			ctor='(GMMCluster::Option option, int numCluster, int unused)',
			inheritsFrom='Clustering',
		},
		{
			name='MotionClustering.AggloCluster',
			cppname='AggloCluster',
			ctor='(const Metric& metric, int eLinkage, int numCluster)',
			inheritsFrom='Clustering',
			memberFunctions=[[
			void cluster_using_distances(const matrixn & distmat)
			]],
		},
		{
			ifdef='USE_CLUSTERING',
			name='MotionClustering.PostureFeatureExtractor',
			inheritsFrom='MotionClustering::FeatureExtractor',
			ctors=[[
			(Motion*, int, int)
			]],
		},
		{
			name='Function',
			memberFunctions=[[
				virtual void learn(const matrixn& sources, const matrixn& targets)	
				virtual void learn(const matrixn& sources)	
				virtual bool isLearned() const 
				int dimDomain() const		
				int dimRange() const		
				void mapping(const vectorn& source, vectorn& target) const ;
				void mapping(const matrixn& sources, matrixn& targets) const;
				m_real logProb(const vectorn& source) const							
			]]
		},
		{
			name='BayesianLinearRegression',
			inheritsFrom='Function',
			properties=[[
				matrixn X;
				matrixn Y;
			]],
			ctors={
				'()',
				'(int order, double noisePrior)'
			},
		},
		{
			name='NonlinearFunctionIDW',
			inheritsFrom='Function',
			ctors={
				'(int k, float power)',
				'()',
				'(Metric* pMetric, int k, float power)',
			},
			memberFunctions=[[
			void mapping(const vectorn& source, vectorn& target, vectorn& weights) const; @ mapping2
			void mapping(const vectorn& source, vectorn& target, intvectorn & index, vectorn& weights) const; @ mapping2
			]],
		},
		{
			name='LinearFunction',
			inheritsFrom='Function',
			ctors={
				'()'
			},
			properties={[[
				matrixn mLinearCoef;
				matrixn mInvLinearCoef;
			]]}
		},
		{
			name='MotionClustering.PCA',
			cppname='bijectiveFunctions::PCA',
			ctors=[[
			()
			(m_real errorGoal)
			]],
			memberFunctions=[[
			void setReducedDim(int dim);
			void learn(const matrixn& sources);
			void mapping(const vectorn& source, vectorn& target) const;
			void inverseMapping(const vectorn& target, vectorn& source) const;
			]],
		},
		{
			ifdef='USE_CLUSTERING',
			name='LuaFeatureExtractor',
			decl='class LuaFeatureExtractor;',
			isLuaInheritable=true,
			--inheritsFrom='MotionClustering::FeatureExtractor',

			memberFunctions={[[
					void Init(MotionDOFcontainer* motdofc, Motion* mot)
					int GetFeatureDimension() const
					TString RecommendedClusterMethod() const
					void CalcFeature(int startFrame, int endFrame, vectorn& vecFeature) const
					void BeforeFinalizeFeature(TArray<vectorn> const& features){}
					void FinalizeFeature(int startFrame, int endFrame, vectorn& vecFeature) const
					void NumGroup(int grp);
					]]},
			ctors={'()'},
		},
		{
			ifdef='USE_CLUSTERING',
			name='ParseGrcFile',
			decl='class ParseGrcFile;',
			wrapperCode=[[
			inline static void setSegment(ParseGrcFile& grc, int iseg, int s, int e, int classIndex)
			{
				grc.startTime(iseg)=s;
				grc.endTime(iseg)=e;
				grc.classIndex(iseg,0)=classIndex;
			}
			]],
			staticMemberFunctions=[[
			void setSegment(ParseGrcFile& grc, int iseg, int s, int e, int classIndex)
			]],
			memberFunctions={[[
			void init(int numSegment, const TStrings& classifierNames)
			void setClassifierName(int iclassifier, const char* name)
			void setClassName(int iclassifier, int iclass, const char* name)
			void setClassNames(int iclassifier, const TStrings& classNames)	
			void exportClassification(const char* grcFile);
			int	findSegment(int time) const
			int addClassifier(const char* classifierName, const TStrings& classNames);
			void addClass(int clsf, const char* className)	
			void deleteSegment(int iseg) ;
			void deleteEmptyClass(int clsf);
			void deleteClass(int clsf, int iclass);
			void deleteClassifier(int clsf);
			void split(int iSegment1, int time);
			void merge(int iSegment1, int iSegment2);
			int numClassifier() const									
			int numClass(int classifier) const						
			int numSegment() const								
			int findClassifier(const char* id) const;
			int findClass(int classifier, const char* classid) const;
			const TStrings& classifierNames() const
			const TString& classifierName(int classifier) const
			const TStrings& classNames(int classifier) const	
			const TString& className(int classifier, int iclass) const
			TString typeName(int iSegment);
			int startTime(int iSegment) const							
			int endTime(int iSegment) const						
			int classIndex(int iSegment, int classifier)	const	
			void setClassIndex(int iSegment, int classifier, int classIndex)   
			intvectornView classIndexes(int iseg);
			intvectorn search(const char* classifier, const char* classid);
			]]},
			ctors={'()', '(const char*)'}
		},
	},
	modules={
		{
			namespace='EXT',
			ifndef='NO_GUI',
			wrapperCode=[[
			static void register_classificationLib(Loader* Lll)
			{
				Register_classificationLib_bind(Lll->L->L);
			}
			]],
			functions=[[
				void register_classificationLib(Loader* Lll)
			]]
		},
		{
			ifdef='USE_CLUSTERING',
			namespace='CFuzzyCluster',
			headerFile='math/FuzzyCluster.h',
			wrapperCode=[[
			inline static void _FcmKMeanCluster(const matrixn &inputvectors, int cluster_n, matrixn& centers, intvectorn &GroupIndex)
			{
				GroupIndex.setSize(inputvectors.rows());
				CFuzzyCluster::FcmKMeanCluster(inputvectors, cluster_n, centers, &GroupIndex[0]);
			}
			inline static void _KMeanCluster(const matrixn &inputvectors, int cluster_n, matrixn& centers, intvectorn &GroupIndex)
			{
				GroupIndex.setSize(inputvectors.rows());
				CFuzzyCluster::KMeanCluster(inputvectors, cluster_n, centers, &GroupIndex[0]);
			}
			inline static void _AggloCluster(const matrixn& distMat, int cluster_n, intvectorn& GroupIndex, int eLinkage=1)
			{
				GroupIndex.setSize(distMat.rows());
				CFuzzyCluster::AggloCluster(distMat, cluster_n, &GroupIndex[0], eLinkage);
			}
			inline static int _AggloCluster(const matrixn& distMat, m_real inner_cluster_thr, intvectorn& GroupIndex)
			{
				GroupIndex.setSize(distMat.rows());
				int cluster_n=1;
				CFuzzyCluster::AggloCluster(distMat, inner_cluster_thr, cluster_n, &GroupIndex[0]);
				return cluster_n;
			}
			]],
			functions=[[
			void _KMeanCluster(const matrixn &inputvectors, int cluster_n, matrixn& centers, intvectorn& pGroupIndex) @ KMeanCluster
			void _FcmKMeanCluster(const matrixn &inputvectors, int cluster_n, matrixn& centers, intvectorn& pGroupIndex) @ FcmKMeanCluster
			void _AggloCluster(const matrixn& distMat, int cluster_n, intvectorn& group_index) @ AggloCluster
			void _AggloCluster(const matrixn& distMat, int cluster_n, intvectorn& group_index, int eLinkage) @ AggloCluster
			int _AggloCluster(const matrixn& distMat, m_real inner_cluster_thr, intvectorn& group_index) @ AggloCluster
			]],
		}
	}
}


function generateClassificationBind() 
	write(
	[[
	#include "../BaseLib/baselib.h"
	#include "../MainLib/MainLib.h"
	#include "../MainLib/OgreFltk/FlLayout.h"
#ifdef USE_CLUSTERING
	#include "../ClassificationLib/motion/FeatureExtractor.h"
	#include "../ClassificationLib/motion/ParseGrcFile.h"
#endif
	#include "../ClassificationLib/math/cluster.h"
	#include "../ClassificationLib/math/Function.h"
	#include "../ClassificationLib/math/BijectiveFunction.h"
	#include "../ClassificationLib/math/SVMWrap.h"
	#include "../ClassificationLib/math/GnuPlot.h"
	]])
	write(
	[[
#ifdef USE_CLUSTERING
			struct LuaFeatureExtractor : public MotionClustering::FeatureExtractor, public luna_wrap_object
			{
				LuaFeatureExtractor ()
					:MotionClustering::FeatureExtractor(NULL)
				{
				}

				virtual ~LuaFeatureExtractor ()
				{
				}

				void Init(MotionDOFcontainer* motdofc, Motion* mot)
				{
					lunaStack l(_L);
					if(((LuaFeatureExtractor*)this)->pushMemberFunc<LuaFeatureExtractor>(l,"init")){
						l.push<MotionDOFcontainer>(motdofc);
						l.push<Motion>(mot);
						l.call(3,0);
					}
				}
				virtual int GetFeatureDimension() const	{
					lunaStack l(_L);
					if(((LuaFeatureExtractor*)this)->pushMemberFunc<LuaFeatureExtractor>(l,"getFeatureDimension")){
						l.call(1,1);
						int dim;
						l>>dim;
						return dim;
					} 
					return 0; 
				}	
				virtual TString RecommendedClusterMethod() const
				{
					lunaStack l(_L);
					if(((LuaFeatureExtractor*)this)->pushMemberFunc<LuaFeatureExtractor>(l,"recommendedClusterMethod")){
						l.call(1,1);
						TString  dim;
						l>>dim;
						return dim;
					} 
					return 0; 
				}
				virtual void CalcFeature(int startFrame, int endFrame, vectorn& vecFeature) const	{
					lunaStack l(_L);
					if(((LuaFeatureExtractor*)this)->pushMemberFunc<LuaFeatureExtractor>(l,"calcFeature")){
						l<<(double)startFrame<<(double)endFrame;
						l.push<vectorn>(vecFeature);
						// input (self, startFrame, endFrame, vecFeature)
						l.call(4,0);
					}
				}
				virtual void BeforeFinalizeFeature(TArray<vectorn> const& features) const
				{
					lunaStack l(_L);
					if(((LuaFeatureExtractor*)this)->pushMemberFunc<LuaFeatureExtractor>(l,"beforeFinalizeFeature")){
						l.push<TArray<vectorn> >(features);
						// beforeFinalizeFeature(self, features)
						l.call(2,0);
					}
				}
				virtual void FinalizeFeature(int startFrame, int endFrame, vectorn& vecFeature) const	{
					lunaStack l(_L);
					if(((LuaFeatureExtractor*)this)->pushMemberFunc<LuaFeatureExtractor>(l,"finalizeFeature")){
						l<<(double)startFrame<<(double)endFrame;
						l.push<vectorn>(vecFeature);
						// input (self, startFrame, endFrame, vecFeature)
						l.call(4,0);
					}
				}
				virtual void NumGroup(int nnn) {
					lunaStack l(_L);
					if(((LuaFeatureExtractor*)this)->pushMemberFunc<LuaFeatureExtractor>(l,"numGroup")){
						l<<(double)nnn;
						// input (self, nnn)
						l.call(2,0);
					}
				}

				// utility functions 
				virtual TString featureString(const vectorn& feature) const { return  TString();}
			};
#endif
			]])

	writeIncludeBlock()
	write('#include "../MainLib/WrapperLua/luna.h"')
	write('#include "../MainLib/WrapperLua/luna_baselib.h"')
	write('#include "../MainLib/WrapperLua/LUAwrapper.h"')
	write('#include "../MainLib/WrapperLua/luna_mainlib.h"')
	writeHeader(bindTargetClassification)
	flushWritten(script_path..'/../../ClassificationLib/luna_classification.h') -- write to a header file only when there exist modifications -> no-recompile.
	write('#include "stdafx.h"')
	write('#include "luna_classification.h"')
	write('#include "../MainLib/OgreFltk/Loader.h"')
	write('#include "math/KNearestInterpolationFast.h"')
	write([[
	void Register_classificationLib_bind(lua_State*L);
	]])
	writeDefinitions(bindTargetClassification, 'Register_classificationLib_bind') -- input bindTarget can be non-overlapping subset of entire bindTarget 
	flushWritten(script_path..'/../../ClassificationLib/luna_classification.cpp') -- write to cpp file only when there exist modifications -> no-recompile.
end

function generate()
	loadDefinitionDB(script_path..'/luna_baselib_db.lua')
	buildDefinitionDB(bindTargetClassification)
	generateClassificationBind()
end
