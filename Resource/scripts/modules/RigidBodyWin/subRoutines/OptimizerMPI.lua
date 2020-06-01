require("RigidBodyWin/subRoutines/Optimizer")


-- function fineLog(...)
--    local fn
--    if rank==nil then
--       fn="optimize_finelog.txt"
--    else
--       fn="optimize_finelog"..rank..".txt"
--    end

--    util.outputToFile(fn, os.date()..util.mergeString({...}))
-- end



-- function coarseLog(...)
--    util.outputToFile(Optimizer.outFile, os.date()..util.mergeString({...}))
-- end


function MPI.sendT(tbl, src)
   -- print("sendT")
   -- printTable(tbl)
   local tt=pickle(tbl)
   -- local ttt=unpickle(tt)
   -- print(tt)
   -- printTable(ttt)
   -- print("sendTend")
   MPI.send(tt, src)
end

function MPI.recvT(src)
   -- print("recvT")
   local tt=MPI.receive(src)
   -- print(tt, "recvTend")
   -- printTable(unpickle("{{[1]=1,},}"))
   -- print("recvTend2")

   return unpickle(tt)
end



--class 'OptimizerMPI'
OptimizerMPI=LUAclass()

function OptimizerMPI:__init(stepSize, opt_dimension, method)

   if rank==nil then
      numCores=MPI.size()
      rank=MPI.rank()
   end

   if rank==0 then
      self.server = Optimizer(stepSize, opt_dimension, method)
      self.server.gradientFunction=self.serverGradientFunction
      self.server.objectiveFunction=self.objectiveFunction
      self.server.batchObjectiveFunction=self.batchObjectiveFunction
      fineLog("server created")
   else
      self.client = Optimizer(stepSize, opt_dimension, method)
      self.client.objectiveFunction=self.objectiveFunction

      fineLog("client created")
   end
end

function OptimizerMPI:sync(tbl)

   if self.server then
      for r=1,numCores-1 do
	 MPI.sendT({"sync",tbl}, r)
      end

      print("hihi1")

      for r=1, numCores-1 do -- can receive msg in any order.
	 assert(MPI.recvT(r)[1]=="sync_received")
      end
   end
end

function OptimizerMPI:loop()

   if self.client then
      local work=MPI.recvT(0)

      fineLog(jobId, work[1])

      if work[1]=="doThis" then
	 
	 local jobId=work[2]

	 local pos=CT.vec(unpack(work[3]))


	 fineLog('work3', #work[3], work[3])
	 fineLog(jobId, work[2], pos)

	 local obj=self.client:objectiveFunction(pos)

	 fineLog("result", obj, jobId)
	 MPI.sendT({"result", jobId, obj},0)

      elseif work[1]=="sync" then
	 self.lastSync=work[2]
	 MPI.sendT({"sync_received"},0)
      else
	 MPI.sendT(work,0)
      end
   end
end

function OptimizerMPI.batchObjectiveFunction(opt, jobs)
   local jobId=1
   local function get_next_work_item()
      job=jobs[jobId]
      jobId=jobId+1

      if job==nil then
	 return {"noJob"}
      end
      local _pos=CT.tblVec(job)
      return {"doThis", jobId-1, _pos}
   end

   coarseLog("preparesend ")
   local msg=""
   for r=1,numCores-1 do
      local work=get_next_work_item()
      if work[1]=="doThis" then	 msg=msg.." "..tostring(r) end
      MPI.sendT(work, r)
   end

   coarseLog("send "..msg)

   local work=get_next_work_item()
   local results={}

   local function getResult(result)
      if result[1]~="noJob" then
	 results[result[2]]=result[3]
      end
   end


   while work[1]~="noJob" do
      local result=MPI.recvT(MPI.ANY_SOURCE)
      getResult(result)
      if work[1]=="doThis" then	 coarseLogShort(tostring(MPI.source()).." ") end
      MPI.sendT(work, MPI.source())
      work=get_next_work_item()
   end

   coarseLog("recv")

   for r=1, numCores-1 do
      local result=MPI.recvT(r)
      
      --      if result[1]~="noJob" then coarseLogShort(tostring(r).." ") end -- too verbose
      getResult(result)
   end

   assert(table.getn(results)==table.getn(jobs))

   return results
end

function OptimizerMPI.serverGradientFunction(opt, _pos)
   
   -- prepare jobs
   local jobs={}
   for i=0, opt.N_opt_dimension do
      if i==0 then
	 jobs[i]=_pos
      else
	 local optvar=opt.opt_dimension[i]
	 --      titlebar:setCaption("iter :"..opt.iteration.." dim :"..optvar.title.." +"..optvar.grad_step..posstr)

	 local pos=vectorn()
	 pos:assign(_pos)
	 set1(pos, i, get1(pos, i)+optvar.grad_step)
	 jobs[i]=pos
      end
   end
   
   local jobId=0
   local function get_next_work_item()
      job=jobs[jobId]
      jobId=jobId+1

      if job==nil then
	 return {"noJob"}
      end
      local _pos=CT.tblVec(job)
      return {"doThis", jobId-1, _pos}
   end

   for r=1,numCores-1 do
      local work=get_next_work_item()

      MPI.sendT(work, r)
   end

   local work=get_next_work_item()
   local results={}

   local function getResult(result)
      if result[1]~="noJob" then
	 results[result[2]]=result[3]
      end
   end

   while work[1]~="noJob" do
      local result=MPI.recvT(MPI.ANY_SOURCE)
      getResult(result)

      MPI.sendT(work, MPI.source())
      work=get_next_work_item()
   end

   for r=1, numCores-1 do
      local result=MPI.recvT(r)
      getResult(result)
   end

   local gradient=vectorn()
   gradient:setSize(opt.N_opt_dimension )
   
   local p0=results[0]
   for i=1,opt.N_opt_dimension do
      local optvar=opt.opt_dimension[i]
      local p1=results[i]
      assert(p1~=nil)

      if p1==100000 then
	 set1(gradient, i,0) -- turn off the dimension
      else
	 set1(gradient, i, (p1-p0)/(optvar.grad_step))
      end
  end

--  local outString="eval: ", p0, gradient
--  print(outString)
--  util.outputToFile(Optimizer.outFile,outString)			

   return gradient, p0
end


function OptimizerMPI:__finalize()
   if rank==0 then
      self.server = nil
   else
      self.client = nil
   end
end
function OptimizerMPI:optimize(clientLoop)

   if self.server~=nil then
      self.server:__init(stepSize, opt_dimension, method)
      self.server:optimize()
   end

   if clientLoop then

      if self.server~=nil then
	 self:sync({"finished", true})
      else
	 while self.lastSync==nil do
	    self:loop()
	 end
	 assert(self.lastSync[1]=="finished")
      end
   end
end

