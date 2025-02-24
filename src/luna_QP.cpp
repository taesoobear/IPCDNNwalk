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
	#ifdef USE_MPI
	#include <mpi.h>
	static  MPI_Status status;
	#endif
	#include "cma/CMAwrap.h"
	#include "GA_simple.h"
	#include "taesooLib/BaseLib/math/OperatorStitch.h"
	#include "taesooLib/PhysicsLib/clapack_wrap.h"
	#include "QP_controller/quadprog.h"
	
#ifndef genlua_luna_QP_lua15473501_def12                      // 1303
#define genlua_luna_QP_lua15473501_def12                      // 1304
// Declare all classes before including this file or using the "decl" property in the luna_gen definition file. // 1305
// e.g. class LMat; class LMatView; ....                      // 1306
// The forward declaration is not automatically made because luna_gen cannot distinguish struct, class, or namespace. // 1307
// But you can declare a class explictly by using something like decl='class AAA;'  // 1308
// Also, you can add a "#include" sentence by using headerFile="AAA.h"  // 1309
// The number at the end of each line denotes the line number of luna_gen.lua which generated that line // 1312
class CMAwrap;                                                // 1319
class Individual;                                             // 1319
class FitnessCalcLua;                                         // 1319
#if defined (USE_MPI)                                         // 1316
#endif //defined (USE_MPI)                                    // 1322
template<>                                                    // 1333
 class LunaTraits<CMAwrap > {
public:                                                       // 1335
    static const char className[];                            // 1344
    static const int uniqueID;                                // 1345
    static luna_RegType methods[];                            // 1346
    static CMAwrap* _bind_ctor(lua_State *L);                 // 1348
    static void _bind_dtor(CMAwrap* obj);                     // 1349
    typedef CMAwrap base_t;                                   // 1351
};                                                            // 1357
template<>                                                    // 1333
 class LunaTraits<FitnessCalc > {
public:                                                       // 1335
    static const char className[];                            // 1344
    static const int uniqueID;                                // 1345
    static luna_RegType methods[];                            // 1346
    static FitnessCalc* _bind_ctor(lua_State *L);             // 1348
    static void _bind_dtor(FitnessCalc* obj);                 // 1349
    typedef FitnessCalc base_t;                               // 1351
};                                                            // 1357
template<>                                                    // 1333
 class LunaTraits<Individual > {
public:                                                       // 1335
    static const char className[];                            // 1344
    static const int uniqueID;                                // 1345
    static luna_RegType methods[];                            // 1346
    static Individual* _bind_ctor(lua_State *L);              // 1348
    static void _bind_dtor(Individual* obj);                  // 1349
    typedef Individual base_t;                                // 1351
static luna__hashmap properties;                              // 1353
static luna__hashmap write_properties;                        // 1354
};                                                            // 1357
template<>                                                    // 1333
 class LunaTraits<Population > {
public:                                                       // 1335
    static const char className[];                            // 1344
    static const int uniqueID;                                // 1345
    static luna_RegType methods[];                            // 1346
    static Population* _bind_ctor(lua_State *L);              // 1348
    static void _bind_dtor(Population* obj);                  // 1349
    typedef Population base_t;                                // 1351
};                                                            // 1357
template<>                                                    // 1333
 class LunaTraits<FitnessCalcLua > {
public:                                                       // 1335
    static const char className[];                            // 1344
    static const int uniqueID;                                // 1345
    static luna_RegType methods[];                            // 1346
    static FitnessCalcLua* _bind_ctor(lua_State *L);          // 1348
    static void _bind_dtor(FitnessCalcLua* obj);              // 1349
    typedef FitnessCalc base_t;                               // 1351
};                                                            // 1357
 class luna__interface_15473501_Eigen {
public:                                                       // 1335
    static const char moduleName[];                           // 1340
    typedef LunaModule<luna__interface_15473501_Eigen> luna_t; // 1341
    static luna_RegType methods[];                            // 1342
};                                                            // 1357
#if defined (USE_MPI)                                         // 1329
 class luna__interface_15473501_MPI {
public:                                                       // 1335
    static const char moduleName[];                           // 1340
    typedef LunaModule<luna__interface_15473501_MPI> luna_t;  // 1341
    static luna_RegType methods[];                            // 1342
};                                                            // 1357
#endif //defined (USE_MPI)                                    // 1359
#endif                                                        // 1362
template<>                                                    // 1395
 class impl_LunaTraits<CMAwrap > {
public:                                                       // 1398
    typedef Luna<CMAwrap > luna_t;                            // 1402
// : number denotes the line number of luna_gen.lua that generated the sentence // 1405
  inline static bool _lg_typecheck_ctor_overload_1(lua_State *L)
  {                                                           // 1414
    if( lua_gettop(L)!=4) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=10150210) return false; // vectorn // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 629
    if( lua_isnumber(L,3)==0) return false;                   // 631
    if( lua_isnumber(L,4)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_ctor_overload_2(lua_State *L)
  {                                                           // 1414
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=10150210) return false; // vectorn // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 629
    if( lua_isnumber(L,3)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_testForTermination(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_samplePopulation(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_numPopulation(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_dim(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getPopulation(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_setVal(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    if( lua_isnumber(L,3)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_resampleSingle(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_resampleSingleFrom(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    if( Luna<void>::get_uniqueid(L,3)!=10150210) return false; // vectorn // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_update(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getMean(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getBest(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 629
    return true;
  }                                                           // 656
  inline static CMAwrap* _bind_ctor_overload_1(lua_State *L)
  {                                                           // 1436
    vectorn const & start_p=static_cast<vectorn &>(*Luna<vectorn >::check(L,1)); // 568
    vectorn const & stdev=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 568
    int populationSize=(int)lua_tonumber(L,3);                // 576
    int mu=(int)lua_tonumber(L,4);                            // 576
    return new CMAwrap( start_p, stdev, populationSize, mu);  // 1441
  }                                                           // 1442
  inline static CMAwrap* _bind_ctor_overload_2(lua_State *L)
  {                                                           // 1436
    vectorn const & start_p=static_cast<vectorn &>(*Luna<vectorn >::check(L,1)); // 568
    vectorn const & stdev=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 568
    int populationSize=(int)lua_tonumber(L,3);                // 576
    return new CMAwrap( start_p, stdev, populationSize);      // 1441
  }                                                           // 1442
  static CMAwrap* _bind_ctor(lua_State *L)
  {                                                           // 233
    if (_lg_typecheck_ctor_overload_1(L)) return _bind_ctor_overload_1(L); // 236
    if (_lg_typecheck_ctor_overload_2(L)) return _bind_ctor_overload_2(L); // 236
    luaL_error(L, "ctor ( cannot find overloads:)\n(vectorn const & start_p,vectorn const & stdev,int populationSize,int mu,)\n(vectorn const & start_p,vectorn const & stdev,int populationSize,)\n");
                                                              // 243
    return NULL;                                              // 244
  }                                                           // 245
  static int _bind_testForTermination(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_testForTermination(L)) { char msg[]="luna typecheck failed:\n  testForTermination(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    try {                                                     // 346
    std ::string ret=self.testForTermination();               // 347
    lua_pushstring(L, ret.c_str());                           // 350
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 354
    return 1;                                                 // 355
  }                                                           // 374
  static int _bind_samplePopulation(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_samplePopulation(L)) { char msg[]="luna typecheck failed:\n  samplePopulation(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    try {                                                     // 295
    self.samplePopulation();                                  // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_numPopulation(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_numPopulation(L)) { char msg[]="luna typecheck failed:\n  numPopulation(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    try {                                                     // 340
    int ret=self.numPopulation();                             // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_dim(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_dim(L)) { char msg[]="luna typecheck failed:\n  dim(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    try {                                                     // 340
    int ret=self.dim();                                       // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_getPopulation(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_getPopulation(L)) { char msg[]="luna typecheck failed:\n  getPopulation(CMAwrap& self,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    int i=(int)lua_tonumber(L,2);                             // 576
    try {                                                     // 328
    ::vectornView* ret=new ::vectornView(self.getPopulation( i)); // 333
    Luna< ::vectorn >::push(L,ret,true,"_vectornView");       // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_setVal(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_setVal(L)) { char msg[]="luna typecheck failed:\n  setVal(CMAwrap& self,int i,double eval,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    int i=(int)lua_tonumber(L,2);                             // 576
    double eval=(double)lua_tonumber(L,3);                    // 576
    try {                                                     // 295
    self.setVal( i, eval);                                    // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_resampleSingle(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_resampleSingle(L)) { char msg[]="luna typecheck failed:\n  resampleSingle(CMAwrap& self,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    int i=(int)lua_tonumber(L,2);                             // 576
    try {                                                     // 295
    self.resampleSingle( i);                                  // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_resampleSingleFrom(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_resampleSingleFrom(L)) { char msg[]="luna typecheck failed:\n  resampleSingleFrom(CMAwrap& self,int i,vectorn const & _arg2,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    int i=(int)lua_tonumber(L,2);                             // 576
    vectorn const & _arg2=static_cast<vectorn &>(*Luna<vectorn >::check(L,3)); // 568
    try {                                                     // 295
    self.resampleSingleFrom( i, _arg2);                       // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_update(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_update(L)) { char msg[]="luna typecheck failed:\n  update(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    try {                                                     // 295
    self.update();                                            // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_getMean(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_getMean(L)) { char msg[]="luna typecheck failed:\n  getMean(CMAwrap& self,vectorn & out,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    vectorn & out=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 568
    try {                                                     // 295
    self.getMean( out);                                       // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_getBest(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_getBest(L)) { char msg[]="luna typecheck failed:\n  getBest(CMAwrap& self,vectorn & out,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    vectorn & out=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 568
    try {                                                     // 295
    self.getBest( out);                                       // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
}; // end of class impl_LunaTraits<CMAwrap >                  // 1618
  CMAwrap* LunaTraits<CMAwrap >::_bind_ctor(lua_State *L)
  {                                                           // 1624
    return impl_LunaTraits<CMAwrap >::_bind_ctor(L);          // 1625
  }                                                           // 1626
  void LunaTraits<CMAwrap >::_bind_dtor(CMAwrap* obj){        // 1628
    delete obj;                                               // 1629
  }                                                           // 1630
const char LunaTraits<CMAwrap >::className[] = "math_CMAwrap"; // 1648
const int LunaTraits<CMAwrap >::uniqueID = 64707780;          // 1649
luna_RegType LunaTraits<CMAwrap >::methods[] = {              // 1655
    {"testForTermination", &impl_LunaTraits<CMAwrap >::_bind_testForTermination}, // 1660
    {"samplePopulation", &impl_LunaTraits<CMAwrap >::_bind_samplePopulation}, // 1660
    {"numPopulation", &impl_LunaTraits<CMAwrap >::_bind_numPopulation}, // 1660
    {"dim", &impl_LunaTraits<CMAwrap >::_bind_dim},           // 1660
    {"getPopulation", &impl_LunaTraits<CMAwrap >::_bind_getPopulation}, // 1660
    {"setVal", &impl_LunaTraits<CMAwrap >::_bind_setVal},     // 1660
    {"resampleSingle", &impl_LunaTraits<CMAwrap >::_bind_resampleSingle}, // 1660
    {"resampleSingleFrom", &impl_LunaTraits<CMAwrap >::_bind_resampleSingleFrom}, // 1660
    {"update", &impl_LunaTraits<CMAwrap >::_bind_update},     // 1660
    {"getMean", &impl_LunaTraits<CMAwrap >::_bind_getMean},   // 1660
    {"getBest", &impl_LunaTraits<CMAwrap >::_bind_getBest},   // 1660
    {0,0}                                                     // 1663
};                                                            // 1664
template<>                                                    // 1395
 class impl_LunaTraits<FitnessCalc > {
public:                                                       // 1398
    typedef Luna<FitnessCalc > luna_t;                        // 1402
// : number denotes the line number of luna_gen.lua that generated the sentence // 1405
}; // end of class impl_LunaTraits<FitnessCalc >              // 1618
  FitnessCalc* LunaTraits<FitnessCalc >::_bind_ctor(lua_State *L)
  {                                                           // 1635
   std::cerr<<"undefined contructor of FitnessCalc called\n"; // 1636
    return NULL;                                              // 1637
  }                                                           // 1638
  void LunaTraits<FitnessCalc >::_bind_dtor(FitnessCalc* obj){ // 1639
   delete obj;                                                // 1640
  }                                                           // 1641
const char LunaTraits<FitnessCalc >::className[] = "_FitnessCalc"; // 1648
const int LunaTraits<FitnessCalc >::uniqueID = 33618519;      // 1649
luna_RegType LunaTraits<FitnessCalc >::methods[] = {          // 1655
    {0,0}                                                     // 1663
};                                                            // 1664
template<>                                                    // 1395
 class impl_LunaTraits<Individual > {
public:                                                       // 1398
    typedef Luna<Individual > luna_t;                         // 1402
// : number denotes the line number of luna_gen.lua that generated the sentence // 1405
  inline static bool _lg_typecheck_toString(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=3133180) return false; // Individual // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_setDefaultGeneLength(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( lua_isnumber(L,1)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_setDefaultTrueGeneRate(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( lua_isnumber(L,1)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck__property_get_genes(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=3133180) return false; // Individual // 629
    return true;
  }                                                           // 656
inline static boolN& _property_get_genes(Individual const& a) { return (boolN &) a.genes; }
                                                              // 1450
  static int _bind_toString(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_toString(L)) { char msg[]="luna typecheck failed:\n  toString(Individual& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Individual& self=static_cast<Individual &>(*Luna<Individual >::check(L,1)); // 568
    try {                                                     // 346
    std ::string ret=self.toString();                         // 347
    lua_pushstring(L, ret.c_str());                           // 350
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 354
    return 1;                                                 // 355
  }                                                           // 374
  static int _bind_setDefaultGeneLength(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_setDefaultGeneLength(L)) { char msg[]="luna typecheck failed:\n  setDefaultGeneLength(int length,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    int length=(int)lua_tonumber(L,1);                        // 576
    try {                                                     // 295
    Individual::setDefaultGeneLength( length);                // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_setDefaultTrueGeneRate(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_setDefaultTrueGeneRate(L)) { char msg[]="luna typecheck failed:\n  setDefaultTrueGeneRate(double rate,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    double rate=(double)lua_tonumber(L,1);                    // 576
    try {                                                     // 295
    Individual::setDefaultTrueGeneRate( rate);                // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind__property_get_genes(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck__property_get_genes(L)) { char msg[]="luna typecheck failed:\n  _property_get_genes(Individual const & a,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Individual const & a=static_cast<Individual &>(*Luna<Individual >::check(L,1)); // 568
    try {                                                     // 321
    boolN & ret=_property_get_genes( a);                      // 322
    Luna<boolN >::push(L,&ret,false,"_boolN");                // 323
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 324
    return 1;                                                 // 338
  }                                                           // 374
  static void luna_init_hashmap()
  {                                                           // 1499
    LunaTraits<Individual >::properties["genes"]=&_bind__property_get_genes; // 1501
  }                                                           // 1503
  static void luna_init_write_hashmap()
  {                                                           // 1504
  }                                                           // 1510
            static int __index(lua_State* L)
            {                                                 // 1514
                        {
                        luna__hashmap::iterator i=LunaTraits<Individual >::properties.find((const char*)lua_tostring(L,2));

                        if (i!=LunaTraits<Individual >::properties.end())
                        {
                            luna_mfp fnc=i->second;
                            lua_pop(L,1); // remove self
                            return fnc(L); 
                        }
                            }
                                                              // 1523
                    int mt=lua_getmetatable(L, 1);
                    if(mt==0) luaL_error(L,"__index");//end
                    lua_pushstring(L, lua_tostring(L,2));
                    lua_rawget(L, -2);
                    return 1;
                                                              // 1556
            }                                                 // 1564
 
            static int __newindex(lua_State* L) {             // 1567
                    luaL_error(L,"__newindex doesn't allow defining non-property member");
                    return 0;
                                                              // 1609
            }                                                 // 1614
}; // end of class impl_LunaTraits<Individual >               // 1618
  Individual* LunaTraits<Individual >::_bind_ctor(lua_State *L)
  {                                                           // 1635
   std::cerr<<"undefined contructor of Individual called\n";  // 1636
    return NULL;                                              // 1637
  }                                                           // 1638
  void LunaTraits<Individual >::_bind_dtor(Individual* obj){  // 1639
   delete obj;                                                // 1640
  }                                                           // 1641
const char LunaTraits<Individual >::className[] = "_Individual"; // 1648
const int LunaTraits<Individual >::uniqueID = 3133180;        // 1649
luna__hashmap LunaTraits<Individual >::properties;            // 1652
luna__hashmap LunaTraits<Individual >::write_properties;      // 1653
luna_RegType LunaTraits<Individual >::methods[] = {           // 1655
    {"toString", &impl_LunaTraits<Individual >::_bind_toString}, // 1660
    {"setDefaultGeneLength", &impl_LunaTraits<Individual >::_bind_setDefaultGeneLength}, // 1660
    {"setDefaultTrueGeneRate", &impl_LunaTraits<Individual >::_bind_setDefaultTrueGeneRate}, // 1660
    {"_property_get_genes", &impl_LunaTraits<Individual >::_bind__property_get_genes}, // 1660
    {"__index", &impl_LunaTraits<Individual >::__index},      // 1660
    {"__newindex", &impl_LunaTraits<Individual >::__newindex}, // 1660
    {0,0}                                                     // 1663
};                                                            // 1664
template<>                                                    // 1395
 class impl_LunaTraits<Population > {
public:                                                       // 1398
    typedef Luna<Population > luna_t;                         // 1402
// : number denotes the line number of luna_gen.lua that generated the sentence // 1405
  inline static bool _lg_typecheck_getIndividual(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=44424126) return false; // Population // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_size(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=44424126) return false; // Population // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getFittest(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=44424126) return false; // Population // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_GA_setParam(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=2) return false;                       // 621
    if( lua_isnumber(L,1)==0) return false;                   // 631
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_GA_solve(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=2) return false;                       // 621
    if( lua_isnumber(L,1)==0) return false;                   // 631
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_setFitness(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=33618519) return false; // FitnessCalc // 629
    return true;
  }                                                           // 656
  static int _bind_getIndividual(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_getIndividual(L)) { char msg[]="luna typecheck failed:\n  getIndividual(Population& self,int index,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Population& self=static_cast<Population &>(*Luna<Population >::check(L,1)); // 568
    int index=(int)lua_tonumber(L,2);                         // 576
    try {                                                     // 321
    Individual & ret=self.getIndividual( index);              // 322
    Luna<Individual >::push(L,&ret,false,"_Individual");      // 323
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 324
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_size(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_size(L)) { char msg[]="luna typecheck failed:\n  size(Population& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Population& self=static_cast<Population &>(*Luna<Population >::check(L,1)); // 568
    try {                                                     // 340
    int ret=self.size();                                      // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_getFittest(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_getFittest(L)) { char msg[]="luna typecheck failed:\n  getFittest(Population& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Population& self=static_cast<Population &>(*Luna<Population >::check(L,1)); // 568
    try {                                                     // 321
    const Individual & ret=self.getFittest();                 // 322
    Luna<Individual >::push(L,&ret,false,"_Individual");      // 323
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 324
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_GA_setParam(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_GA_setParam(L)) { char msg[]="luna typecheck failed:\n  GA_setParam(double m,int t,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    double m=(double)lua_tonumber(L,1);                       // 576
    int t=(int)lua_tonumber(L,2);                             // 576
    try {                                                     // 295
    GA_setParam( m, t);                                       // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_GA_solve(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_GA_solve(L)) { char msg[]="luna typecheck failed:\n  GA_solve(int numPop,int maxGenerations,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    int numPop=(int)lua_tonumber(L,1);                        // 576
    int maxGenerations=(int)lua_tonumber(L,2);                // 576
    try {                                                     // 311
    Population * ret=GA_solve( numPop, maxGenerations);       // 312
   if (ret==NULL) lua_pushnil(L); else                        // 313
    Luna<Population >::push(L,(Population *)ret,false,"_Population"); // 314
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 315
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_setFitness(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_setFitness(L)) { char msg[]="luna typecheck failed:\n  setFitness(FitnessCalc * pcalc,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    FitnessCalc * pcalc=static_cast<FitnessCalc *>(Luna<FitnessCalc >::check(L,1)); // 564
    try {                                                     // 295
    setFitness( pcalc);                                       // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
}; // end of class impl_LunaTraits<Population >               // 1618
  Population* LunaTraits<Population >::_bind_ctor(lua_State *L)
  {                                                           // 1635
   std::cerr<<"undefined contructor of Population called\n";  // 1636
    return NULL;                                              // 1637
  }                                                           // 1638
  void LunaTraits<Population >::_bind_dtor(Population* obj){  // 1639
   delete obj;                                                // 1640
  }                                                           // 1641
const char LunaTraits<Population >::className[] = "_Population"; // 1648
const int LunaTraits<Population >::uniqueID = 44424126;       // 1649
luna_RegType LunaTraits<Population >::methods[] = {           // 1655
    {"getIndividual", &impl_LunaTraits<Population >::_bind_getIndividual}, // 1660
    {"size", &impl_LunaTraits<Population >::_bind_size},      // 1660
    {"getFittest", &impl_LunaTraits<Population >::_bind_getFittest}, // 1660
    {"GA_setParam", &impl_LunaTraits<Population >::_bind_GA_setParam}, // 1660
    {"GA_solve", &impl_LunaTraits<Population >::_bind_GA_solve}, // 1660
    {"setFitness", &impl_LunaTraits<Population >::_bind_setFitness}, // 1660
    {0,0}                                                     // 1663
};                                                            // 1664
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
                                                              // 1391
template<>                                                    // 1395
 class impl_LunaTraits<FitnessCalcLua > {
public:                                                       // 1398
    typedef Luna<FitnessCalcLua > luna_t;                     // 1402
// : number denotes the line number of luna_gen.lua that generated the sentence // 1405
  inline static bool _lg_typecheck_ctor(lua_State *L)
  {                                                           // 1414
    if( lua_gettop(L)!=0) return false;                       // 621
    return true;
  }                                                           // 656
  inline static FitnessCalcLua* _bind_ctor(lua_State *L)
  {                                                           // 1436
    if (!_lg_typecheck_ctor(L)) { char msg[]="luna typecheck failed:\n  ctor()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    return new FitnessCalcLua();                              // 1441
  }                                                           // 1442
            static int __index(lua_State* L)
            {                                                 // 1514
                    luna_t::userdataType* u=luna_t::checkRaw(L,1);
                    if (u->has_env){
                        lua_getfenv(L,1);
                        lua_pushvalue(L,2);
                        lua_gettable(L,-2);
                        if( !lua_isnil(L,-1))
                        return 1;
                    }
                                                              // 1545
                    int mt=lua_getmetatable(L, 1);
                    if(mt==0) luaL_error(L,"__index");//end
                    lua_pushstring(L, lua_tostring(L,2));
                    lua_rawget(L, -2);
                    return 1;
                                                              // 1556
            }                                                 // 1564
 
            static int __newindex(lua_State* L) {             // 1567
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
                                                              // 1595
            }                                                 // 1614
}; // end of class impl_LunaTraits<FitnessCalcLua >           // 1618
  FitnessCalcLua* LunaTraits<FitnessCalcLua >::_bind_ctor(lua_State *L)
  {                                                           // 1624
    return impl_LunaTraits<FitnessCalcLua >::_bind_ctor(L);   // 1625
  }                                                           // 1626
  void LunaTraits<FitnessCalcLua >::_bind_dtor(FitnessCalcLua* obj){ // 1628
    delete obj;                                               // 1629
  }                                                           // 1630
const char LunaTraits<FitnessCalcLua >::className[] = "_FitnessCalcLua"; // 1648
const int LunaTraits<FitnessCalcLua >::uniqueID = 33618519;   // 1649
luna_RegType LunaTraits<FitnessCalcLua >::methods[] = {       // 1655
    {"new_modified_T", &Luna<FitnessCalcLua >::new_modified_T}, // 1658
    {"__index", &impl_LunaTraits<FitnessCalcLua >::__index},  // 1660
    {"__newindex", &impl_LunaTraits<FitnessCalcLua >::__newindex}, // 1660
    {0,0}                                                     // 1663
};                                                            // 1664
 class impl_luna__interface_15473501_Eigen {
public:                                                       // 1398
    typedef LunaModule<luna__interface_15473501_Eigen> luna_t; // 1400
// : number denotes the line number of luna_gen.lua that generated the sentence // 1405
  inline static bool _lg_typecheck_solveQuadprog_overload_1(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=6) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=55023510) return false; // HessianQuadratic // 629
    if( Luna<void>::get_uniqueid(L,2)!=23735758) return false; // matrixn // 629
    if( Luna<void>::get_uniqueid(L,3)!=10150210) return false; // vectorn // 629
    if( Luna<void>::get_uniqueid(L,4)!=23735758) return false; // matrixn // 629
    if( Luna<void>::get_uniqueid(L,5)!=10150210) return false; // vectorn // 629
    if( Luna<void>::get_uniqueid(L,6)!=10150210) return false; // vectorn // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_solveQuadprog_overload_2(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=7) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=55023510) return false; // HessianQuadratic // 629
    if( Luna<void>::get_uniqueid(L,2)!=23735758) return false; // matrixn // 629
    if( Luna<void>::get_uniqueid(L,3)!=10150210) return false; // vectorn // 629
    if( Luna<void>::get_uniqueid(L,4)!=23735758) return false; // matrixn // 629
    if( Luna<void>::get_uniqueid(L,5)!=10150210) return false; // vectorn // 629
    if( Luna<void>::get_uniqueid(L,6)!=10150210) return false; // vectorn // 629
    if( lua_isboolean(L,7)==0) return false;                  // 641
    return true;
  }                                                           // 656
  static int _bind_solveQuadprog_overload_1(lua_State *L)
  {                                                           // 1457
    HessianQuadratic & problem=static_cast<HessianQuadratic &>(*Luna<HessianQuadratic >::check(L,1)); // 568
    const matrixn & CE=static_cast<matrixn &>(*Luna<matrixn >::check(L,2)); // 568
    const vectorn & ce0=static_cast<vectorn &>(*Luna<vectorn >::check(L,3)); // 568
    const matrixn & CI=static_cast<matrixn &>(*Luna<matrixn >::check(L,4)); // 568
    const vectorn & ci0=static_cast<vectorn &>(*Luna<vectorn >::check(L,5)); // 568
    vectorn & x=static_cast<vectorn &>(*Luna<vectorn >::check(L,6)); // 568
    try {                                                     // 340
    double ret=solve_quadprog( problem, CE, ce0, CI, ci0, x); // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_solveQuadprog_overload_2(lua_State *L)
  {                                                           // 1457
    HessianQuadratic & problem=static_cast<HessianQuadratic &>(*Luna<HessianQuadratic >::check(L,1)); // 568
    const matrixn & CE=static_cast<matrixn &>(*Luna<matrixn >::check(L,2)); // 568
    const vectorn & ce0=static_cast<vectorn &>(*Luna<vectorn >::check(L,3)); // 568
    const matrixn & CI=static_cast<matrixn &>(*Luna<matrixn >::check(L,4)); // 568
    const vectorn & ci0=static_cast<vectorn &>(*Luna<vectorn >::check(L,5)); // 568
    vectorn & x=static_cast<vectorn &>(*Luna<vectorn >::check(L,6)); // 568
    bool _arg7=(bool)lua_toboolean(L,7);                      // 579
    try {                                                     // 340
    double ret=solve_quadprog( problem, CE, ce0, CI, ci0, x, _arg7); // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_solveQuadprog(lua_State *L)
  {                                                           // 233
    if (_lg_typecheck_solveQuadprog_overload_1(L)) return _bind_solveQuadprog_overload_1(L); // 236
    if (_lg_typecheck_solveQuadprog_overload_2(L)) return _bind_solveQuadprog_overload_2(L); // 236
    luaL_error(L, "solveQuadprog ( cannot find overloads:)\n(HessianQuadratic & problem,const matrixn & CE,const vectorn & ce0,const matrixn & CI,const vectorn & ci0,vectorn & x,)\n(HessianQuadratic & problem,const matrixn & CE,const vectorn & ce0,const matrixn & CI,const vectorn & ci0,vectorn & x,bool _arg7,)\n");
                                                              // 243
    return 0;                                                 // 244
  }                                                           // 245
}; // end of class impl_luna__interface_15473501_Eigen        // 1618
const char luna__interface_15473501_Eigen::moduleName[] = "_Eigen"; // 1646
luna_RegType luna__interface_15473501_Eigen::methods[] = {    // 1655
    {"solveQuadprog", &impl_luna__interface_15473501_Eigen::_bind_solveQuadprog}, // 1660
    {0,0}                                                     // 1663
};                                                            // 1664
#if defined (USE_MPI)                                         // 1378
 class impl_luna__interface_15473501_MPI {
public:                                                       // 1398
    typedef LunaModule<luna__interface_15473501_MPI> luna_t;  // 1400
// : number denotes the line number of luna_gen.lua that generated the sentence // 1405
  inline static bool _lg_typecheck_rank(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=0) return false;                       // 621
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_size(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=0) return false;                       // 621
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_send(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=2) return false;                       // 621
    if( lua_isstring(L,1)==0) return false;                   // 636
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_receive(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=1) return false;                       // 621
    if( lua_isnumber(L,1)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_source(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=0) return false;                       // 621
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_test(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=0) return false;                       // 621
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_test2(lua_State *L)
  {                                                           // 1424
    if( lua_gettop(L)!=0) return false;                       // 621
    return true;
  }                                                           // 656
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
                                                              // 1450
  static int _bind_rank(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_rank(L)) { char msg[]="luna typecheck failed:\n  rank()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    try {                                                     // 340
    int ret=rank();                                           // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_size(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_size(L)) { char msg[]="luna typecheck failed:\n  size()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    try {                                                     // 340
    int ret=size();                                           // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_send(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_send(L)) { char msg[]="luna typecheck failed:\n  send(const char * msg,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    const char * msg=(const char *)lua_tostring(L,1);         // 571
    int i=(int)lua_tonumber(L,2);                             // 576
    try {                                                     // 295
    send( msg, i);                                            // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_receive(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_receive(L)) { char msg[]="luna typecheck failed:\n  receive(int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    int i=(int)lua_tonumber(L,1);                             // 576
    try {                                                     // 346
    std ::string ret=receive( i);                             // 347
    lua_pushstring(L, ret.c_str());                           // 350
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 354
    return 1;                                                 // 355
  }                                                           // 374
  static int _bind_source(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_source(L)) { char msg[]="luna typecheck failed:\n  source()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    try {                                                     // 340
    int ret=source();                                         // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_test(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_test(L)) { char msg[]="luna typecheck failed:\n  test()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    try {                                                     // 295
    test();                                                   // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_test2(lua_State *L)
  {                                                           // 1457
    if (!_lg_typecheck_test2(L)) { char msg[]="luna typecheck failed:\n  test2()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    try {                                                     // 295
    test2();                                                  // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
}; // end of class impl_luna__interface_15473501_MPI          // 1618
const char luna__interface_15473501_MPI::moduleName[] = "_MPI"; // 1646
luna_RegType luna__interface_15473501_MPI::methods[] = {      // 1655
    {"rank", &impl_luna__interface_15473501_MPI::_bind_rank}, // 1660
    {"size", &impl_luna__interface_15473501_MPI::_bind_size}, // 1660
    {"send", &impl_luna__interface_15473501_MPI::_bind_send}, // 1660
    {"receive", &impl_luna__interface_15473501_MPI::_bind_receive}, // 1660
    {"source", &impl_luna__interface_15473501_MPI::_bind_source}, // 1660
    {"test", &impl_luna__interface_15473501_MPI::_bind_test}, // 1660
    {"test2", &impl_luna__interface_15473501_MPI::_bind_test2}, // 1660
    {0,0}                                                     // 1663
};                                                            // 1664
#endif //defined (USE_MPI)                                    // 1667
void Register_QP(lua_State* L) {                              // 1671
    luna_dostring(L,"if __luna==nil then __luna={} end");     // 1672
    luna_dostring(L,"    if __luna.copyMethodsFrom==nil then\n        function __luna.copyMethodsFrom(methodsChild, methodsParent)\n            for k,v in pairs(methodsParent) do\n                if k~='__index' and k~='__newindex' and methodsChild[k]==nil then\n                    methodsChild[k]=v\n                end\n            end\n        end\n        function __luna.overwriteMethodsFrom(methodsChild, methodsParent)\n            for k,v in pairs(methodsParent) do\n                if k~='__index' and k~='__newindex' then\n                    if verbose then print('registering', k, methodsChild[k]) end\n                    methodsChild[k]=v\n                end\n            end\n        end\n    end\n    "); // 1673
    Luna<CMAwrap >::Register(L);                              // 1714
    luna_dostring(L, "if not math then math={} end math.CMAwrap=__luna.math_CMAwrap"); // 1721
    luna_dostring(L,"                __luna.math_CMAwrap.luna_class='h.CMAwrap'"); // 1722
    Luna<FitnessCalc >::Register(L);                          // 1714
    luna_dostring(L, "FitnessCalc=__luna._FitnessCalc");      // 1737
    luna_dostring(L,"                __luna._FitnessCalc.luna_class='FitnessCalc'"); // 1738
   impl_LunaTraits<Individual >::luna_init_hashmap();         // 1708
   impl_LunaTraits<Individual >::luna_init_write_hashmap();   // 1709
    Luna<Individual >::Register(L);                           // 1714
    luna_dostring(L, "Individual=__luna._Individual");        // 1737
    luna_dostring(L,"                __luna._Individual.luna_class='Individual'"); // 1738
    Luna<Population >::Register(L);                           // 1714
    luna_dostring(L, "Population=__luna._Population");        // 1737
    luna_dostring(L,"                __luna._Population.luna_class='Population'"); // 1738
    Luna<FitnessCalcLua >::Register(L);                       // 1714
    luna_dostring(L, "FitnessCalcLua=__luna._FitnessCalcLua"); // 1737
    luna_dostring(L,"                __luna._FitnessCalcLua.luna_class='FitnessCalcLua'"); // 1738
    luna_dostring(L,"            __luna.copyMethodsFrom(__luna._FitnessCalcLua, __luna._FitnessCalc)"); // 1747
   LunaModule<luna__interface_15473501_Eigen >::Register(L);  // 1712
    luna_dostring(L," \n                if Eigen==nil then \n                    Eigen={}\n                end \n                __luna.overwriteMethodsFrom(Eigen, __luna._Eigen)\n                "); // 1727
#if defined (USE_MPI)                                         // 1702
   LunaModule<luna__interface_15473501_MPI >::Register(L);    // 1712
    luna_dostring(L," \n                if MPI==nil then \n                    MPI={}\n                end \n                __luna.overwriteMethodsFrom(MPI, __luna._MPI)\n                "); // 1727
{
  std::stringstream stringStreams;// defining enums           // 1752
  stringStreams <<"MPI.ANY_SOURCE="<< MPI_ANY_SOURCE;         // 1756
  luna_dostring(L, stringStreams.str().c_str());
}                                                             // 1759
{
  std::stringstream stringStreams;// defining enums           // 1752
  stringStreams <<"MPI.ANY_TAG="<< MPI_ANY_TAG;               // 1756
  luna_dostring(L, stringStreams.str().c_str());
}                                                             // 1759
#endif //defined (USE_MPI)                                    // 1764
}                                                             // 1771
