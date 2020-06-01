
array.pushBack(gen_lua.enum_types, 'GMMCluster::Option') 
bindTargetClassification={
	classes={
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
		}
	}
}


function generateClassificationBind() 
	write(
	[[
	#include "../BaseLib/baselib.h"
	#include "../MainLib/MainLib.h"
	#include "../MainLib/OgreFltk/FlLayout.h"
	#include "../ClassificationLib/math/cluster.h"
	#include "../ClassificationLib/math/Function.h"
	#include "../ClassificationLib/math/BijectiveFunction.h"
	#include "../ClassificationLib/math/SVMWrap.h"
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
