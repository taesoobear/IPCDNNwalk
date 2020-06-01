	#include "stdafx.h"
	#include <cstring>
	#include <string>
	#include <cstdio>
	#include <iostream>
	#include <cstdlib>
	#include <sstream>
	//#include "GPDservo.h"
	
		#include <stdio.h>
		#include <string.h>
		#include <string>
		extern "C"
		{
			#include "lua.h"
			#include "lualib.h"
			#include "lauxlib.h"
			//#include "luadebug.h"
		}
		
#include "MainLib/WrapperLua/luna.h"
#include "MainLib/WrapperLua/luna_baselib.h"
#include "PhysicsLib/luna_physics.h"
	#ifdef USE_MPI
	#include <mpi.h>
	static  MPI_Status status;
	#endif
	#include "cma/CMAwrap.h"
	#include "GA_simple.h"
	
#ifndef genlua_luna_QP_lua15473501_def12                      // 1300
#define genlua_luna_QP_lua15473501_def12                      // 1301
// Declare all classes before including this file or using the "decl" property in the luna_gen definition file. // 1302
// e.g. class LMat; class LMatView; ....                      // 1303
// The forward declaration is not automatically made because luna_gen cannot distinguish struct, class, or namespace. // 1304
// But you can declare a class explictly by using something like decl='class AAA;'  // 1305
// Also, you can add a "#include" sentence by using headerFile="AAA.h"  // 1306
// The number at the end of each line denotes the line number of luna_gen.lua which generated that line // 1309
class CMAwrap;                                                // 1313
class Individual;                                             // 1313
class FitnessCalcLua;                                         // 1313
template<>                                                    // 1324
 class LunaTraits<CMAwrap > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static CMAwrap* _bind_ctor(lua_State *L);                 // 1339
    static void _bind_dtor(CMAwrap* obj);                     // 1340
    typedef CMAwrap base_t;                                   // 1342
};                                                            // 1348
template<>                                                    // 1324
 class LunaTraits<FitnessCalc > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static FitnessCalc* _bind_ctor(lua_State *L);             // 1339
    static void _bind_dtor(FitnessCalc* obj);                 // 1340
    typedef FitnessCalc base_t;                               // 1342
};                                                            // 1348
template<>                                                    // 1324
 class LunaTraits<Individual > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static Individual* _bind_ctor(lua_State *L);              // 1339
    static void _bind_dtor(Individual* obj);                  // 1340
    typedef Individual base_t;                                // 1342
static luna__hashmap properties;                              // 1344
static luna__hashmap write_properties;                        // 1345
};                                                            // 1348
template<>                                                    // 1324
 class LunaTraits<Population > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static Population* _bind_ctor(lua_State *L);              // 1339
    static void _bind_dtor(Population* obj);                  // 1340
    typedef Population base_t;                                // 1342
};                                                            // 1348
template<>                                                    // 1324
 class LunaTraits<FitnessCalcLua > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static FitnessCalcLua* _bind_ctor(lua_State *L);          // 1339
    static void _bind_dtor(FitnessCalcLua* obj);              // 1340
    typedef FitnessCalc base_t;                               // 1342
};                                                            // 1348
#if defined (USE_MPI)                                         // 1320
 class luna__interface__MPI {
public:                                                       // 1326
    static const char moduleName[];                           // 1331
    typedef LunaModule<luna__interface__MPI> luna_t;          // 1332
    static luna_RegType methods[];                            // 1333
};                                                            // 1348
#endif //defined (USE_MPI)                                    // 1350
#endif                                                        // 1353
template<>                                                    // 1386
 class impl_LunaTraits<CMAwrap > {
public:                                                       // 1389
    typedef Luna<CMAwrap > luna_t;                            // 1393
// : number denotes the line number of luna_gen.lua that generated the sentence // 1396
  inline static bool _lg_typecheck_ctor_overload_1(lua_State *L)
  {                                                           // 1405
    if( lua_gettop(L)!=4) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=10150210) return false; // vectorn // 626
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 626
    if( lua_isnumber(L,3)==0) return false;                   // 628
    if( lua_isnumber(L,4)==0) return false;                   // 628
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_ctor_overload_2(lua_State *L)
  {                                                           // 1405
    if( lua_gettop(L)!=3) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=10150210) return false; // vectorn // 626
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 626
    if( lua_isnumber(L,3)==0) return false;                   // 628
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_testForTermination(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 626
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_samplePopulation(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 626
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_numPopulation(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 626
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_dim(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 626
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_getPopulation(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=2) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 626
    if( lua_isnumber(L,2)==0) return false;                   // 628
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_setVal(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=3) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 626
    if( lua_isnumber(L,2)==0) return false;                   // 628
    if( lua_isnumber(L,3)==0) return false;                   // 628
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_resampleSingle(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=2) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 626
    if( lua_isnumber(L,2)==0) return false;                   // 628
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_resampleSingleFrom(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=3) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 626
    if( lua_isnumber(L,2)==0) return false;                   // 628
    if( Luna<void>::get_uniqueid(L,3)!=10150210) return false; // vectorn // 626
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_update(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 626
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_getMean(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=2) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 626
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 626
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_getBest(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=2) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 626
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 626
    return true;
  }                                                           // 653
  inline static CMAwrap* _bind_ctor_overload_1(lua_State *L)
  {                                                           // 1427
    vectorn const & start_p=static_cast<vectorn &>(*Luna<vectorn >::check(L,1)); // 565
    vectorn const & stdev=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 565
    int populationSize=(int)lua_tonumber(L,3);                // 573
    int mu=(int)lua_tonumber(L,4);                            // 573
    return new CMAwrap( start_p, stdev, populationSize, mu);  // 1432
  }                                                           // 1433
  inline static CMAwrap* _bind_ctor_overload_2(lua_State *L)
  {                                                           // 1427
    vectorn const & start_p=static_cast<vectorn &>(*Luna<vectorn >::check(L,1)); // 565
    vectorn const & stdev=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 565
    int populationSize=(int)lua_tonumber(L,3);                // 573
    return new CMAwrap( start_p, stdev, populationSize);      // 1432
  }                                                           // 1433
  static CMAwrap* _bind_ctor(lua_State *L)
  {                                                           // 233
    if (_lg_typecheck_ctor_overload_1(L)) return _bind_ctor_overload_1(L); // 236
    if (_lg_typecheck_ctor_overload_2(L)) return _bind_ctor_overload_2(L); // 236
    luaL_error(L, "ctor ( cannot find overloads:)\n(vectorn const & start_p,vectorn const & stdev,int populationSize,int mu,)\n(vectorn const & start_p,vectorn const & stdev,int populationSize,)\n");
                                                              // 243
    return NULL;                                              // 244
  }                                                           // 245
  static int _bind_testForTermination(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_testForTermination(L)) { char msg[]="luna typecheck failed:\n  testForTermination(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 565
    try {                                                     // 343
    std ::string ret=self.testForTermination();               // 344
    lua_pushstring(L, ret.c_str());                           // 347
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 351
    return 1;                                                 // 352
  }                                                           // 371
  static int _bind_samplePopulation(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_samplePopulation(L)) { char msg[]="luna typecheck failed:\n  samplePopulation(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 565
    try {                                                     // 292
    self.samplePopulation();                                  // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
  static int _bind_numPopulation(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_numPopulation(L)) { char msg[]="luna typecheck failed:\n  numPopulation(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 565
    try {                                                     // 337
    int ret=self.numPopulation();                             // 338
    lua_pushnumber(L, ret);                                   // 339
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 340
    return 1;                                                 // 341
  }                                                           // 371
  static int _bind_dim(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_dim(L)) { char msg[]="luna typecheck failed:\n  dim(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 565
    try {                                                     // 337
    int ret=self.dim();                                       // 338
    lua_pushnumber(L, ret);                                   // 339
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 340
    return 1;                                                 // 341
  }                                                           // 371
  static int _bind_getPopulation(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_getPopulation(L)) { char msg[]="luna typecheck failed:\n  getPopulation(CMAwrap& self,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 565
    int i=(int)lua_tonumber(L,2);                             // 573
    try {                                                     // 325
    ::vectornView* ret=new ::vectornView(self.getPopulation( i)); // 330
    Luna< ::vectorn >::push(L,ret,true,"_vectornView");       // 331
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 332
    return 1;                                                 // 335
  }                                                           // 371
  static int _bind_setVal(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_setVal(L)) { char msg[]="luna typecheck failed:\n  setVal(CMAwrap& self,int i,double eval,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 565
    int i=(int)lua_tonumber(L,2);                             // 573
    double eval=(double)lua_tonumber(L,3);                    // 573
    try {                                                     // 292
    self.setVal( i, eval);                                    // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
  static int _bind_resampleSingle(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_resampleSingle(L)) { char msg[]="luna typecheck failed:\n  resampleSingle(CMAwrap& self,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 565
    int i=(int)lua_tonumber(L,2);                             // 573
    try {                                                     // 292
    self.resampleSingle( i);                                  // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
  static int _bind_resampleSingleFrom(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_resampleSingleFrom(L)) { char msg[]="luna typecheck failed:\n  resampleSingleFrom(CMAwrap& self,int i,vectorn const & _arg2,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 565
    int i=(int)lua_tonumber(L,2);                             // 573
    vectorn const & _arg2=static_cast<vectorn &>(*Luna<vectorn >::check(L,3)); // 565
    try {                                                     // 292
    self.resampleSingleFrom( i, _arg2);                       // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
  static int _bind_update(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_update(L)) { char msg[]="luna typecheck failed:\n  update(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 565
    try {                                                     // 292
    self.update();                                            // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
  static int _bind_getMean(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_getMean(L)) { char msg[]="luna typecheck failed:\n  getMean(CMAwrap& self,vectorn & out,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 565
    vectorn & out=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 565
    try {                                                     // 292
    self.getMean( out);                                       // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
  static int _bind_getBest(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_getBest(L)) { char msg[]="luna typecheck failed:\n  getBest(CMAwrap& self,vectorn & out,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 565
    vectorn & out=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 565
    try {                                                     // 292
    self.getBest( out);                                       // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
}; // end of class impl_LunaTraits<CMAwrap >                  // 1609
  CMAwrap* LunaTraits<CMAwrap >::_bind_ctor(lua_State *L)
  {                                                           // 1615
    return impl_LunaTraits<CMAwrap >::_bind_ctor(L);          // 1616
  }                                                           // 1617
  void LunaTraits<CMAwrap >::_bind_dtor(CMAwrap* obj){        // 1619
    delete obj;                                               // 1620
  }                                                           // 1621
const char LunaTraits<CMAwrap >::className[] = "math_CMAwrap"; // 1639
const int LunaTraits<CMAwrap >::uniqueID = 64707780;          // 1640
luna_RegType LunaTraits<CMAwrap >::methods[] = {              // 1646
    {"testForTermination", &impl_LunaTraits<CMAwrap >::_bind_testForTermination}, // 1651
    {"samplePopulation", &impl_LunaTraits<CMAwrap >::_bind_samplePopulation}, // 1651
    {"numPopulation", &impl_LunaTraits<CMAwrap >::_bind_numPopulation}, // 1651
    {"dim", &impl_LunaTraits<CMAwrap >::_bind_dim},           // 1651
    {"getPopulation", &impl_LunaTraits<CMAwrap >::_bind_getPopulation}, // 1651
    {"setVal", &impl_LunaTraits<CMAwrap >::_bind_setVal},     // 1651
    {"resampleSingle", &impl_LunaTraits<CMAwrap >::_bind_resampleSingle}, // 1651
    {"resampleSingleFrom", &impl_LunaTraits<CMAwrap >::_bind_resampleSingleFrom}, // 1651
    {"update", &impl_LunaTraits<CMAwrap >::_bind_update},     // 1651
    {"getMean", &impl_LunaTraits<CMAwrap >::_bind_getMean},   // 1651
    {"getBest", &impl_LunaTraits<CMAwrap >::_bind_getBest},   // 1651
    {0,0}                                                     // 1654
};                                                            // 1655
template<>                                                    // 1386
 class impl_LunaTraits<FitnessCalc > {
public:                                                       // 1389
    typedef Luna<FitnessCalc > luna_t;                        // 1393
// : number denotes the line number of luna_gen.lua that generated the sentence // 1396
}; // end of class impl_LunaTraits<FitnessCalc >              // 1609
  FitnessCalc* LunaTraits<FitnessCalc >::_bind_ctor(lua_State *L)
  {                                                           // 1626
   std::cerr<<"undefined contructor of FitnessCalc called\n"; // 1627
    return NULL;                                              // 1628
  }                                                           // 1629
  void LunaTraits<FitnessCalc >::_bind_dtor(FitnessCalc* obj){ // 1630
   delete obj;                                                // 1631
  }                                                           // 1632
const char LunaTraits<FitnessCalc >::className[] = "_FitnessCalc"; // 1639
const int LunaTraits<FitnessCalc >::uniqueID = 33618519;      // 1640
luna_RegType LunaTraits<FitnessCalc >::methods[] = {          // 1646
    {0,0}                                                     // 1654
};                                                            // 1655
template<>                                                    // 1386
 class impl_LunaTraits<Individual > {
public:                                                       // 1389
    typedef Luna<Individual > luna_t;                         // 1393
// : number denotes the line number of luna_gen.lua that generated the sentence // 1396
  inline static bool _lg_typecheck_toString(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=3133180) return false; // Individual // 626
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_setDefaultGeneLength(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( lua_isnumber(L,1)==0) return false;                   // 628
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_setDefaultTrueGeneRate(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( lua_isnumber(L,1)==0) return false;                   // 628
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck__property_get_genes(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=3133180) return false; // Individual // 626
    return true;
  }                                                           // 653
inline static boolN& _property_get_genes(Individual const& a) { return (boolN &) a.genes; }
                                                              // 1441
  static int _bind_toString(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_toString(L)) { char msg[]="luna typecheck failed:\n  toString(Individual& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    Individual& self=static_cast<Individual &>(*Luna<Individual >::check(L,1)); // 565
    try {                                                     // 343
    std ::string ret=self.toString();                         // 344
    lua_pushstring(L, ret.c_str());                           // 347
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 351
    return 1;                                                 // 352
  }                                                           // 371
  static int _bind_setDefaultGeneLength(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_setDefaultGeneLength(L)) { char msg[]="luna typecheck failed:\n  setDefaultGeneLength(int length,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    int length=(int)lua_tonumber(L,1);                        // 573
    try {                                                     // 292
    Individual::setDefaultGeneLength( length);                // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
  static int _bind_setDefaultTrueGeneRate(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_setDefaultTrueGeneRate(L)) { char msg[]="luna typecheck failed:\n  setDefaultTrueGeneRate(double rate,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    double rate=(double)lua_tonumber(L,1);                    // 573
    try {                                                     // 292
    Individual::setDefaultTrueGeneRate( rate);                // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
  static int _bind__property_get_genes(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck__property_get_genes(L)) { char msg[]="luna typecheck failed:\n  _property_get_genes(Individual const & a,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    Individual const & a=static_cast<Individual &>(*Luna<Individual >::check(L,1)); // 565
    try {                                                     // 318
    boolN & ret=_property_get_genes( a);                      // 319
    Luna<boolN >::push(L,&ret,false,"_boolN");                // 320
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 321
    return 1;                                                 // 335
  }                                                           // 371
  static void luna_init_hashmap()
  {                                                           // 1490
    LunaTraits<Individual >::properties["genes"]=&_bind__property_get_genes; // 1492
  }                                                           // 1494
  static void luna_init_write_hashmap()
  {                                                           // 1495
  }                                                           // 1501
            static int __index(lua_State* L)
            {                                                 // 1505
                        {
                        luna__hashmap::iterator i=LunaTraits<Individual >::properties.find((const char*)lua_tostring(L,2));

                        if (i!=LunaTraits<Individual >::properties.end())
                        {
                            luna_mfp fnc=i->second;
                            lua_pop(L,1); // remove self
                            return fnc(L); 
                        }
                            }
                                                              // 1514
                    int mt=lua_getmetatable(L, 1);
                    if(mt==0) luaL_error(L,"__index");//end
                    lua_pushstring(L, lua_tostring(L,2));
                    lua_rawget(L, -2);
                    return 1;
                                                              // 1547
            }                                                 // 1555
 
            static int __newindex(lua_State* L) {             // 1558
                    luaL_error(L,"__newindex doesn't allow defining non-property member");
                    return 0;
                                                              // 1600
            }                                                 // 1605
}; // end of class impl_LunaTraits<Individual >               // 1609
  Individual* LunaTraits<Individual >::_bind_ctor(lua_State *L)
  {                                                           // 1626
   std::cerr<<"undefined contructor of Individual called\n";  // 1627
    return NULL;                                              // 1628
  }                                                           // 1629
  void LunaTraits<Individual >::_bind_dtor(Individual* obj){  // 1630
   delete obj;                                                // 1631
  }                                                           // 1632
const char LunaTraits<Individual >::className[] = "_Individual"; // 1639
const int LunaTraits<Individual >::uniqueID = 3133180;        // 1640
luna__hashmap LunaTraits<Individual >::properties;            // 1643
luna__hashmap LunaTraits<Individual >::write_properties;      // 1644
luna_RegType LunaTraits<Individual >::methods[] = {           // 1646
    {"toString", &impl_LunaTraits<Individual >::_bind_toString}, // 1651
    {"setDefaultGeneLength", &impl_LunaTraits<Individual >::_bind_setDefaultGeneLength}, // 1651
    {"setDefaultTrueGeneRate", &impl_LunaTraits<Individual >::_bind_setDefaultTrueGeneRate}, // 1651
    {"_property_get_genes", &impl_LunaTraits<Individual >::_bind__property_get_genes}, // 1651
    {"__index", &impl_LunaTraits<Individual >::__index},      // 1651
    {"__newindex", &impl_LunaTraits<Individual >::__newindex}, // 1651
    {0,0}                                                     // 1654
};                                                            // 1655
template<>                                                    // 1386
 class impl_LunaTraits<Population > {
public:                                                       // 1389
    typedef Luna<Population > luna_t;                         // 1393
// : number denotes the line number of luna_gen.lua that generated the sentence // 1396
  inline static bool _lg_typecheck_getIndividual(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=2) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=44424126) return false; // Population // 626
    if( lua_isnumber(L,2)==0) return false;                   // 628
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_size(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=44424126) return false; // Population // 626
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_getFittest(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=44424126) return false; // Population // 626
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_GA_setParam(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=2) return false;                       // 618
    if( lua_isnumber(L,1)==0) return false;                   // 628
    if( lua_isnumber(L,2)==0) return false;                   // 628
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_GA_solve(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=2) return false;                       // 618
    if( lua_isnumber(L,1)==0) return false;                   // 628
    if( lua_isnumber(L,2)==0) return false;                   // 628
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_setFitness(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( Luna<void>::get_uniqueid(L,1)!=33618519) return false; // FitnessCalc // 626
    return true;
  }                                                           // 653
  static int _bind_getIndividual(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_getIndividual(L)) { char msg[]="luna typecheck failed:\n  getIndividual(Population& self,int index,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    Population& self=static_cast<Population &>(*Luna<Population >::check(L,1)); // 565
    int index=(int)lua_tonumber(L,2);                         // 573
    try {                                                     // 318
    Individual & ret=self.getIndividual( index);              // 319
    Luna<Individual >::push(L,&ret,false,"_Individual");      // 320
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 321
    return 1;                                                 // 335
  }                                                           // 371
  static int _bind_size(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_size(L)) { char msg[]="luna typecheck failed:\n  size(Population& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    Population& self=static_cast<Population &>(*Luna<Population >::check(L,1)); // 565
    try {                                                     // 337
    int ret=self.size();                                      // 338
    lua_pushnumber(L, ret);                                   // 339
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 340
    return 1;                                                 // 341
  }                                                           // 371
  static int _bind_getFittest(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_getFittest(L)) { char msg[]="luna typecheck failed:\n  getFittest(Population& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    Population& self=static_cast<Population &>(*Luna<Population >::check(L,1)); // 565
    try {                                                     // 318
    const Individual & ret=self.getFittest();                 // 319
    Luna<Individual >::push(L,&ret,false,"_Individual");      // 320
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 321
    return 1;                                                 // 335
  }                                                           // 371
  static int _bind_GA_setParam(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_GA_setParam(L)) { char msg[]="luna typecheck failed:\n  GA_setParam(double m,int t,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    double m=(double)lua_tonumber(L,1);                       // 573
    int t=(int)lua_tonumber(L,2);                             // 573
    try {                                                     // 292
    GA_setParam( m, t);                                       // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
  static int _bind_GA_solve(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_GA_solve(L)) { char msg[]="luna typecheck failed:\n  GA_solve(int numPop,int maxGenerations,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    int numPop=(int)lua_tonumber(L,1);                        // 573
    int maxGenerations=(int)lua_tonumber(L,2);                // 573
    try {                                                     // 308
    Population * ret=GA_solve( numPop, maxGenerations);       // 309
   if (ret==NULL) lua_pushnil(L); else                        // 310
    Luna<Population >::push(L,(Population *)ret,false,"_Population"); // 311
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 312
    return 1;                                                 // 335
  }                                                           // 371
  static int _bind_setFitness(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_setFitness(L)) { char msg[]="luna typecheck failed:\n  setFitness(FitnessCalc * pcalc,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    FitnessCalc * pcalc=static_cast<FitnessCalc *>(Luna<FitnessCalc >::check(L,1)); // 561
    try {                                                     // 292
    setFitness( pcalc);                                       // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
}; // end of class impl_LunaTraits<Population >               // 1609
  Population* LunaTraits<Population >::_bind_ctor(lua_State *L)
  {                                                           // 1626
   std::cerr<<"undefined contructor of Population called\n";  // 1627
    return NULL;                                              // 1628
  }                                                           // 1629
  void LunaTraits<Population >::_bind_dtor(Population* obj){  // 1630
   delete obj;                                                // 1631
  }                                                           // 1632
const char LunaTraits<Population >::className[] = "_Population"; // 1639
const int LunaTraits<Population >::uniqueID = 44424126;       // 1640
luna_RegType LunaTraits<Population >::methods[] = {           // 1646
    {"getIndividual", &impl_LunaTraits<Population >::_bind_getIndividual}, // 1651
    {"size", &impl_LunaTraits<Population >::_bind_size},      // 1651
    {"getFittest", &impl_LunaTraits<Population >::_bind_getFittest}, // 1651
    {"GA_setParam", &impl_LunaTraits<Population >::_bind_GA_setParam}, // 1651
    {"GA_solve", &impl_LunaTraits<Population >::_bind_GA_solve}, // 1651
    {"setFitness", &impl_LunaTraits<Population >::_bind_setFitness}, // 1651
    {0,0}                                                     // 1654
};                                                            // 1655
            class FitnessCalcLua: public FitnessCalc, public luna_wrap_object
            {
                public:
                FitnessCalcLua()
                {
                }

                virtual ~FitnessCalcLua()
                {
                }

                virtual double getFitness(Individual const& individual)
                {
                    lunaStack l(_L);
                    if(pushMemberFunc<FitnessCalcLua>(l,"getFitness")){
                        l.push<Individual>(individual);
                        l.call(2,1);
                        double ret;
                        l>>ret;
                        return ret;
                    }
                }
                virtual double getMaxFitness()
                {
                    lunaStack l(_L);
                    if(pushMemberFunc<FitnessCalcLua>(l,"getMaxFitness")){
                        l.call(1,1);
                        double ret;
                        l>>ret;
                        return ret;
                    } 
                    return 1;
                }
            };
                                                              // 1382
template<>                                                    // 1386
 class impl_LunaTraits<FitnessCalcLua > {
public:                                                       // 1389
    typedef Luna<FitnessCalcLua > luna_t;                     // 1393
// : number denotes the line number of luna_gen.lua that generated the sentence // 1396
  inline static bool _lg_typecheck_ctor(lua_State *L)
  {                                                           // 1405
    if( lua_gettop(L)!=0) return false;                       // 618
    return true;
  }                                                           // 653
  inline static FitnessCalcLua* _bind_ctor(lua_State *L)
  {                                                           // 1427
    if (!_lg_typecheck_ctor(L)) { char msg[]="luna typecheck failed:\n  ctor()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    return new FitnessCalcLua();                              // 1432
  }                                                           // 1433
            static int __index(lua_State* L)
            {                                                 // 1505
                    luna_t::userdataType* u=luna_t::checkRaw(L,1);
                    if (u->has_env){
                        lua_getfenv(L,1);
                        lua_pushvalue(L,2);
                        lua_gettable(L,-2);
                        if( !lua_isnil(L,-1))
                        return 1;
                    }
                                                              // 1536
                    int mt=lua_getmetatable(L, 1);
                    if(mt==0) luaL_error(L,"__index");//end
                    lua_pushstring(L, lua_tostring(L,2));
                    lua_rawget(L, -2);
                    return 1;
                                                              // 1547
            }                                                 // 1555
 
            static int __newindex(lua_State* L) {             // 1558
                    luna_t::userdataType* u=luna_t::checkRaw(L,1);
                    if(!u->has_env) {
                        lua_newtable(L);
                        lua_pushvalue(L,-1);
                        lua_setfenv(L,1);
                        u->has_env=1;
                    } else lua_getfenv(L,1);
                    lua_replace(L,1);
                    lua_settable(L,1);
                    return 0;
                                                              // 1586
            }                                                 // 1605
}; // end of class impl_LunaTraits<FitnessCalcLua >           // 1609
  FitnessCalcLua* LunaTraits<FitnessCalcLua >::_bind_ctor(lua_State *L)
  {                                                           // 1615
    return impl_LunaTraits<FitnessCalcLua >::_bind_ctor(L);   // 1616
  }                                                           // 1617
  void LunaTraits<FitnessCalcLua >::_bind_dtor(FitnessCalcLua* obj){ // 1619
    delete obj;                                               // 1620
  }                                                           // 1621
const char LunaTraits<FitnessCalcLua >::className[] = "_FitnessCalcLua"; // 1639
const int LunaTraits<FitnessCalcLua >::uniqueID = 33618519;   // 1640
luna_RegType LunaTraits<FitnessCalcLua >::methods[] = {       // 1646
    {"new_modified_T", &Luna<FitnessCalcLua >::new_modified_T}, // 1649
    {"__index", &impl_LunaTraits<FitnessCalcLua >::__index},  // 1651
    {"__newindex", &impl_LunaTraits<FitnessCalcLua >::__newindex}, // 1651
    {0,0}                                                     // 1654
};                                                            // 1655
#if defined (USE_MPI)                                         // 1369
 class impl_luna__interface__MPI {
public:                                                       // 1389
    typedef LunaModule<luna__interface__MPI> luna_t;          // 1391
// : number denotes the line number of luna_gen.lua that generated the sentence // 1396
  inline static bool _lg_typecheck_rank(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=0) return false;                       // 618
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_size(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=0) return false;                       // 618
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_send(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=2) return false;                       // 618
    if( lua_isstring(L,1)==0) return false;                   // 633
    if( lua_isnumber(L,2)==0) return false;                   // 628
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_receive(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=1) return false;                       // 618
    if( lua_isnumber(L,1)==0) return false;                   // 628
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_source(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=0) return false;                       // 618
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_test(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=0) return false;                       // 618
    return true;
  }                                                           // 653
  inline static bool _lg_typecheck_test2(lua_State *L)
  {                                                           // 1415
    if( lua_gettop(L)!=0) return false;                       // 618
    return true;
  }                                                           // 653
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
                                                              // 1441
  static int _bind_rank(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_rank(L)) { char msg[]="luna typecheck failed:\n  rank()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    try {                                                     // 337
    int ret=rank();                                           // 338
    lua_pushnumber(L, ret);                                   // 339
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 340
    return 1;                                                 // 341
  }                                                           // 371
  static int _bind_size(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_size(L)) { char msg[]="luna typecheck failed:\n  size()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    try {                                                     // 337
    int ret=size();                                           // 338
    lua_pushnumber(L, ret);                                   // 339
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 340
    return 1;                                                 // 341
  }                                                           // 371
  static int _bind_send(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_send(L)) { char msg[]="luna typecheck failed:\n  send(const char * msg,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    const char * msg=(const char *)lua_tostring(L,1);         // 568
    int i=(int)lua_tonumber(L,2);                             // 573
    try {                                                     // 292
    send( msg, i);                                            // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
  static int _bind_receive(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_receive(L)) { char msg[]="luna typecheck failed:\n  receive(int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    int i=(int)lua_tonumber(L,1);                             // 573
    try {                                                     // 343
    std ::string ret=receive( i);                             // 344
    lua_pushstring(L, ret.c_str());                           // 347
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 351
    return 1;                                                 // 352
  }                                                           // 371
  static int _bind_source(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_source(L)) { char msg[]="luna typecheck failed:\n  source()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    try {                                                     // 337
    int ret=source();                                         // 338
    lua_pushnumber(L, ret);                                   // 339
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 340
    return 1;                                                 // 341
  }                                                           // 371
  static int _bind_test(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_test(L)) { char msg[]="luna typecheck failed:\n  test()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    try {                                                     // 292
    test();                                                   // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
  static int _bind_test2(lua_State *L)
  {                                                           // 1448
    if (!_lg_typecheck_test2(L)) { char msg[]="luna typecheck failed:\n  test2()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 545
    try {                                                     // 292
    test2();                                                  // 293
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 294
    return 0;                                                 // 295
  }                                                           // 371
}; // end of class impl_luna__interface__MPI                  // 1609
const char luna__interface__MPI::moduleName[] = "_MPI";       // 1637
luna_RegType luna__interface__MPI::methods[] = {              // 1646
    {"rank", &impl_luna__interface__MPI::_bind_rank},         // 1651
    {"size", &impl_luna__interface__MPI::_bind_size},         // 1651
    {"send", &impl_luna__interface__MPI::_bind_send},         // 1651
    {"receive", &impl_luna__interface__MPI::_bind_receive},   // 1651
    {"source", &impl_luna__interface__MPI::_bind_source},     // 1651
    {"test", &impl_luna__interface__MPI::_bind_test},         // 1651
    {"test2", &impl_luna__interface__MPI::_bind_test2},       // 1651
    {0,0}                                                     // 1654
};                                                            // 1655
#endif //defined (USE_MPI)                                    // 1658
void Register_QP(lua_State* L) {                              // 1662
    luna_dostring(L,"if __luna==nil then __luna={} end");     // 1663
    luna_dostring(L,"    if __luna.copyMethodsFrom==nil then\n        function __luna.copyMethodsFrom(methodsChild, methodsParent)\n            for k,v in pairs(methodsParent) do\n                if k~='__index' and k~='__newindex' and methodsChild[k]==nil then\n                    methodsChild[k]=v\n                end\n            end\n        end\n        function __luna.overwriteMethodsFrom(methodsChild, methodsParent)\n            for k,v in pairs(methodsParent) do\n                if k~='__index' and k~='__newindex' then\n                    if verbose then print('registering', k, methodsChild[k]) end\n                    methodsChild[k]=v\n                end\n            end\n        end\n    end\n    "); // 1664
    Luna<CMAwrap >::Register(L);                              // 1705
    luna_dostring(L, "if not math then math={} end math.CMAwrap=__luna.math_CMAwrap"); // 1712
    luna_dostring(L,"                __luna.math_CMAwrap.luna_class='h.CMAwrap'"); // 1713
    Luna<FitnessCalc >::Register(L);                          // 1705
    luna_dostring(L, "FitnessCalc=__luna._FitnessCalc");      // 1728
    luna_dostring(L,"                __luna._FitnessCalc.luna_class='FitnessCalc'"); // 1729
   impl_LunaTraits<Individual >::luna_init_hashmap();         // 1699
   impl_LunaTraits<Individual >::luna_init_write_hashmap();   // 1700
    Luna<Individual >::Register(L);                           // 1705
    luna_dostring(L, "Individual=__luna._Individual");        // 1728
    luna_dostring(L,"                __luna._Individual.luna_class='Individual'"); // 1729
    Luna<Population >::Register(L);                           // 1705
    luna_dostring(L, "Population=__luna._Population");        // 1728
    luna_dostring(L,"                __luna._Population.luna_class='Population'"); // 1729
    Luna<FitnessCalcLua >::Register(L);                       // 1705
    luna_dostring(L, "FitnessCalcLua=__luna._FitnessCalcLua"); // 1728
    luna_dostring(L,"                __luna._FitnessCalcLua.luna_class='FitnessCalcLua'"); // 1729
    luna_dostring(L,"            __luna.copyMethodsFrom(__luna._FitnessCalcLua, __luna._FitnessCalc)"); // 1738
#if defined (USE_MPI)                                         // 1693
   LunaModule<luna__interface__MPI >::Register(L);            // 1703
    luna_dostring(L," \n                if MPI==nil then \n                    MPI={}\n                end \n                __luna.overwriteMethodsFrom(MPI, __luna._MPI)\n                "); // 1718
{
  std::stringstream stringStreams;// defining enums           // 1743
  stringStreams <<"MPI.ANY_SOURCE="<< MPI_ANY_SOURCE;         // 1747
  luna_dostring(L, stringStreams.str().c_str());
}                                                             // 1750
{
  std::stringstream stringStreams;// defining enums           // 1743
  stringStreams <<"MPI.ANY_TAG="<< MPI_ANY_TAG;               // 1747
  luna_dostring(L, stringStreams.str().c_str());
}                                                             // 1750
#endif //defined (USE_MPI)                                    // 1755
}                                                             // 1762