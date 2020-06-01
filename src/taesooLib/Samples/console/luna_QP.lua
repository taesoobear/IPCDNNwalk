bindTargetClassification={
	classes={
		{
			name='math.CMAwrap',
			cppname='CMAwrap',
			decl=true,
			ctors=[[
				(vectorn const& start_p, vectorn const& stdev, int populationSize, int mu);
				(vectorn const& start_p, vectorn const& stdev, int populationSize);
			]],
			memberFunctions=[[
			std::string testForTermination();	
			void samplePopulation();
			int numPopulation();
			int dim();
			vectornView getPopulation(int i);
			void setVal(int i, double eval);
			void resampleSingle(int i);
			void update();
			void getMean(vectorn& out);
			void getBest(vectorn& out);
			]]
		},
	},
	modules={
		{
			namespace='Eigen',
			functions={[[
			double solve_quadprog( HessianQuadratic & problem, const matrixn & CE, const vectorn & ce0, const matrixn & CI, const vectorn & ci0, vectorn & x) @ solveQuadprog
			double solve_quadprog( HessianQuadratic & problem, const matrixn & CE, const vectorn & ce0, const matrixn & CI, const vectorn & ci0, vectorn & x, bool) @ solveQuadprog
			void solveLCP(const matrixn&  N, const vectorn& r, vectorn& g, vectorn & a);
			]]}
		}, 
									{
										namespace='MPI',
ifdef='USE_MPI',
										wrapperCode=
											[[
#ifdef USE_MPI


  static int size()
  {
    int size, rc;
    rc = MPI_Comm_size(MPI_COMM_WORLD, &size);
    return size;
  }

  static int rank()
  {
    int rank, rc;
    rc = MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    return rank;
  }

  static void send(const char* msg, int i)
  {
    int rc, tag=100;
    int len=strlen(msg);
    rc = MPI_Send(&len, 1, MPI_INT, i, tag, MPI_COMM_WORLD);
    rc = MPI_Send((void*)msg, len+1, MPI_CHAR, i, tag, MPI_COMM_WORLD);
  }

  static std::string receive(int i)
  {
    int rc, tag=100;
    int len;
    rc = MPI_Recv(&len, 1, MPI_INT, i, tag, MPI_COMM_WORLD, &status);
    TString temp;
    temp.reserve(len+1);
    rc = MPI_Recv((void*)&temp[0], len+1, MPI_CHAR, status.MPI_SOURCE, tag, MPI_COMM_WORLD, &status);
    temp.updateLength();
    return std::string(temp.ptr());
  }
 
  static int source()
  {
	  return status.MPI_SOURCE;
  }
  static void test()
  {
	vectorn a(1);
	a.set(0,1);

	for(int i=0;i<1000; i++)
	{
		for(int j=0; j<1000; j++)
		{
			a(0)=a(0)+1;
		}
	}
  }
  static void test2()
  {
	int a=1;
	for(int i=0;i<1000; i++)
	{
		for(int j=0; j<1000; j++)
		{
			a=a+1;
		}
	}
  }

  #endif
											]],
											functions={[[
   static int rank()
 static int size()
  static void send(const char* msg, int i)
  static std::string receive(int i)
  static int source()
  static void test()
  static void test2()
											]]},
											enums={
											{"ANY_SOURCE","MPI_ANY_SOURCE"},
											{"ANY_TAG","MPI_ANY_TAG"}
										}
									},
	},
}
function generate()
	loadDefinitionDB(script_path..'/luna_baselib_db.lua')
	loadDefinitionDB(script_path..'/../../PhysicsLib/luna_physicslib_db.lua')
	buildDefinitionDB(bindTargetClassification)
	write(
	[[
	#include "stdafx.h"
	]]);
	writeIncludeBlock()
	write('#include "../../MainLib/WrapperLua/luna.h"')
	write('#include "../../MainLib/WrapperLua/luna_baselib.h"')
	write('#include "../../PhysicsLib/luna_physics.h"')
	write('#include "quadprog.h"')
	write([[
#ifdef USE_MPI
	#include <mpi.h>
	static  MPI_Status status;
#endif
	#include "../classification/cma/CMAwrap.h"
]])
	writeHeader(bindTargetClassification)
	writeDefinitions(bindTargetClassification, 'Register_QP') -- input bindTarget can be non-overlapping subset of entire bindTarget 
	flushWritten(source_path..'/luna_QP.cpp') -- write to cpp file only when there exist modifications -> no-recompile.
end
