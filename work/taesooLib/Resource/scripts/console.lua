
function LUAclass(baseClass)

-- A function to define a class only using native lua calls.  
-- usage: MotionLoader=LUAclass()  -- define a class.
--        VRMLloader=LUAclass(MotionLoader) -- define a derived class

--  loader=VRMLloader:create({a,b,c}) -- Create an instance of the class.
-- Note that parameters are enclosed by {}

   local classobj={}
   if __classMTs==nil then
      __classMTs={}
      __classMTs.N=0
   end
  
   __classMTs.N=__classMTs.N+1
   local classId=__classMTs.N
   __classMTs[classId]={__index=classobj}
   classobj.__classId=classId

   classobj.create=function (classobj, tbl)
              local new_inst={}
              setmetatable(new_inst, __classMTs[classobj.__classId])
              new_inst:__init(unpack(tbl))
              return new_inst
           end

   if baseClass~=nil then
      setmetatable(classobj, {__index=baseClass})
   end

   return classobj     
end


class 'VVV' -- I used luabind class only here to check if dtor is called. You can also use a C++ class instead.
function VVV:__init(ss)
   print("ctor VVV",ss)
   self.name=ss
end

function VVV:__finalize()
   print("dtor VVV", self.name)
end

GCtest=LUAclass()

function GCtest:__init(ss)
  
   self.skel=VVV(ss)
  
end

 

GCtest2=LUAclass(GCtest)
function GCtest2:__init(ss)
   GCtest.__init(self,ss)
end

function gcTest()
   do
      local test=GCtest:create({"name1"})
   end
   collectgarbage("collect") -- dtor of VVV called

   do
      local test=GCtest2:create({"name2"})
   end
   collectgarbage("collect") -- dtor of VVV called

end

debug.debug()
