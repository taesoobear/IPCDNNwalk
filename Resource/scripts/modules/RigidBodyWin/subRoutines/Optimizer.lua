
--class 'Optimizer'
Optimizer=LUAclass()

-- actual parameters used for optimization
--stepSize=0.0000000005	-- very important to be set properly. need some intuition based on optimizelog.txt

-- opt_dimension={ -- curvar: initialSolution
--	{title="rfoot", curval= -0.0042154777139752, max_step= 0.0005, grad_step=0.0005},
--	{title="body", curval=-0.033891291915885, max_step=0.001, grad_step=0.001}	,
--	{title="bodyx", curval=0.0013967637201659, max_step=0.0002, grad_step=0.0002}	
--	}

Optimizer.methods={
	GradientDecent={}, 
	GradientDecent2={tol=0.001, thr=0.01 },-- tol is tolerence for termination, thr is tolerence for linmin.
	ConjugateGradient={tol=0.001, thr=0.01 },-- tol is tolerence for termination, thr is tolerence for linmin.
	
	-- To use ConjugateGradient2, four functions are necessary
	-- p = func_dfunc:getInout()
	-- fp =func_dfunc:func(p)
	-- fp =func_dfunc:func_dfunc(p, out_dp)
	-- func_dfunc:info(iter, fp)
	-- opt_dimension is unnecessary for this method. An empty table: opt_dimension={} should be work.
	ConjugateGradient2={tol=0.001, thr=0.01, func_dfunc =nil},-- tol is tolerence for termination, thr is tolerence for linmin.
	RandomizedSearch={}, 
	FullSearch={
		numGrid=vectorn(),
		gridSize=vectorn(),
	},
	CMAes={nrestart=1, stddev=0.25, maxIteration=100, useMPIwhenPossible=true},
	CMAes_nolog={nrestart=1, stddev=0.25, maxIteration=100, useMPIwhenPossible=true},
	GSLGradientDescent={type="GradientDescent",tol=0.1, thr=0},
	NRgradientDescent={type="NRgradientDescent", tol=0.001, thr=0.01 },-- tol is tolerence for termination, thr is tolerence for linmin.
	NRconjugateGradient={type="NRconjugateGradient", tol=0.01, thr=0 },-- thr is unused.
	GSLBFGS={type="BFGS",tol=0.1, thr=0},
	GA={nPopulation=20, nElite=2, pMutate=0.3, nCrossOver=10, maxIteration=10000},
	Test={testCount=4, useRandom=true, useRandomStart=3},
	LBFGS={ 
		optimize=function(self, opt) -- self: LBFGS table.
			local epsilon=1e-10
			LBFGS_opt=LUAclass(Optimize)
			function LBFGS_opt:__init()
			end
			function LBFGS_opt:_objectiveFunction(pos)
				local out= self.opt:objectiveFunction(pos)
				return out
			end
			local method=Optimize.LBFGS_METHOD(epsilon)

			local lbfgs_opt=LBFGS_opt()
			local max_step=1 -- do not use normalization
			--local max_step=opt_dimension[1].max_step
			lbfgs_opt:init(0, opt.N_opt_dimension,max_step, opt_dimension[1].grad_step,method)
			lbfgs_opt.opt=opt
			local initialSol=opt:getVecOptResult()
			lbfgs_opt:optimize(initialSol)
			opt:setOptResult(lbfgs_opt:getResult())
		end
	},
	LBFGS_analytic={ 
		optimize=function(self, opt) -- self: LBFGS table.
			local epsilon=1e-10
			local LBFGS_opta=LUAclass(OptimizeAnalytic)
			function LBFGS_opta:__init()
			end
			function LBFGS_opta:_objectiveFunction(pos)
				assert(false)
				local out= self.opt:objectiveFunction(pos)
				return out
			end
			function LBFGS_opta:_gradientFunction(pos,grad)
				local out= self.opt:gradientFunction(pos, grad)
				return out
			end
			local method=Optimize.LBFGS_METHOD(epsilon)

			local lbfgs_opt=LBFGS_opta()
			local max_step=1 -- do not use normalization
			--local max_step=opt_dimension[1].max_step
			lbfgs_opt:init(0, opt.N_opt_dimension,max_step, opt_dimension[1].grad_step,method)
			lbfgs_opt.opt=opt
			local initialSol=opt:getVecOptResult()
			lbfgs_opt:optimize(initialSol)
			opt:setOptResult(lbfgs_opt:getResult())
		end
	}
}

Optimizer.outFile="optimizelog.txt"
discardErrorSamples=false
doFineLog=false
function fineLog(...)
	if not doFileLog then return end
	local fn
	if rank==nil then
		fn="optimize_finelog.txt"
	else
		fn="optimize_finelog"..rank..".txt"
	end

	--   util.outputToFile(fn, os.date()..util.mergeString({...}))
	dbg.outputToFile(fn, os.date()..util.mergeString({...}))
end

function fineLogShort(...)
	if not doFileLog then return end
	local fn
	if rank==nil then
		fn="optimize_finelog.txt"
	else
		fn="optimize_finelog"..rank..".txt"
	end

	--   util.outputToFileShort(fn, util.mergeString({...}))
	dbg.outputToFile(fn, util.mergeString({...}), "")
end


function coarseLog(...)
	if Optimizer.outFile then
		--   util.outputToFile(Optimizer.outFile, os.date()..util.mergeString({...}))
		dbg.outputToFile(Optimizer.outFile, os.date()..util.mergeString({...}))
	end
end

function coarseLogShort(...)
	if Optimizer.outFile then
		--   util.outputToFileShort(Optimizer.outFile, util.mergeString({...}))
		dbg.outputToFile(Optimizer.outFile, util.mergeString({...}))
	end
end

function Optimizer:__init(stepSize, opt_dimension, method)
	self.stepSize=stepSize
	self.opt_dimension=opt_dimension
	self.N_opt_dimension=table.getn(opt_dimension)
	self.iteration=0
	self.objectList=Ogre.ObjectList ()
	self.method=method

	if Optimizer.outFile then
		--print("create titlebar")

		util.printFile(Optimizer.outFile, "------------------New optimization--------")
		--titlebar:create()
		--subtitlebar:create()
		--subtitlebar:setCaption("asdf")
	end
end

Func=LUAclass (math.Function)
function Func:__init(optimizer)
	--math.Function.__init(self)
	self.x=vectorn()
	self.x:resize(optimizer.N_opt_dimension)
	self.opos=vectorn()
	self.opos:resize(optimizer.N_opt_dimension)
	self.opt=optimizer

	local pos=self.x


	-- for normalization
	local opos=self.opt:getCurPos()
	self.oeval=self.opt:objectiveFunction(opos)
	self.opt:notifyEval(1, self.oeval)

	local outputString="oeval:"..tostring(self.oeval).." dim:"..tostring(optimizer.N_opt_dimension)
	print(outputString)
	if Optimizer.outFile then
		self.opt:saveShort(Optimizer.outFile, outputString)
	end

	self:normalize(pos, opos)
end

function Func:normalize(pos, opos)
	for i=0, pos:size()-1 do
		pos:set(i, opos(i)/self.opt.opt_dimension[i+1].max_step)
	end
end

function Func:unnormalize(opos, pos)
	for i=0, pos:size()-1 do
		opos:set(i, pos(i)*self.opt.opt_dimension[i+1].max_step)
	end
end

function Func:Unnormalize(pos)

	local opos=vectorn(pos:size())
	self:unnormalize(opos,pos)
	return opos
end
function Func:Normalize(opos)

	local pos=vectorn(opos:size())
	self:normalize(pos,opos)
	return pos
end


function Func:getInout()
	return self.x
end

function Func:func(x)
	self:unnormalize(self.opos, x)

	--   print("func: ",self.opos)
	local verbose=false
	if verbose then
		self.opt:saveShort2("optimize_finelog.txt", self.opos, os.date())
	end

	local out=self.opt:objectiveFunction(self.opos)

	local outputstring=os.date().."eval:"..tostring(out).." "

	if verbose then
		print(outputstring)
		util.outputToFile("optimize_finelog.txt", outputstring)
	end

	if self.oeval~=self.oeval then
		debug.debug()
	end




	if Optimizer.outFile then
		util.outputToFile(Optimizer.outFile,outputstring)	
	end
	if discardErrorSamples then
		if out==100000 then
			return 100000
		end
	end
	return out/self.oeval, out
end


function Func:func_dfunc(x,dx)
	self:unnormalize(self.opos, x)
	local gradient, e=self.opt:gradientFunction(self.opos) 
	for i=0, dx:size()-1 do
		dx:set(i, gradient(i)/self.oeval)
	end

	if discardErrorSamples then
		if e==100000 then
			return 100000
		end
	end
	return e/self.oeval
end

function Func:info(iter, eval)
	local outputString="iter :"..iter.." "
	self.opt.iteration=iter

	if eval==nil then
		if math.fmod(iter,10)==0 then
			local ret, ret2=self:func(self.x)
			outputString=outputString.." eval:"..tostring(ret)
			self.opt:notifyEval(ret, ret2)
		end
	else
		outputString=outputString.." eval:"..tostring(eval)
	end

	self:unnormalize(self.opos, self.x)
	for i=1, self.opt.N_opt_dimension do
		local optvar=self.opt.opt_dimension[i]
		optvar.curval=get1(self.opos, i)
	end

	if Optimizer.outFile then
		print(outputString)
		self.opt:saveShort(Optimizer.outFile, outputString)
	end
end


function Optimizer:__finalize()
	titlebar:destroy()
	subtitlebar:destroy()
end

function Optimizer:notifyEval(eval, eval2)
	self.best_eval=math.min(self.best_eval, eval)
	self.best_eval_unscaled=math.min(self.best_eval_unscaled, eval2)
end
function Optimizer:optimize()
	self.opt_dimension=opt_dimension
	self.N_opt_dimension=#self.opt_dimension

	if false then 
		-- always save optimization results by default.
		self.best_eval=0.99 
		self.best_eval_unscaled=1e-10
	else
		-- always don't save optimization results by default
		self.best_eval=1.0
		self.best_eval_unscaled=1e10 
	end
	if self.method.optimize then
		self.method:optimize(self)
		return
	elseif self.method==Optimizer.methods.GradientDecent then
		self:updateGradientDecent()
	elseif self.method==Optimizer.methods.FullSearch then
		self:fullSearch()
	elseif self.method==Optimizer.methods.RandomizedSearch then
		self:randomizedSearch()
	elseif self.method==Optimizer.methods.GA then
		self:GAsearch()
	elseif self.method==Optimizer.methods.Test then
		local func=Func(self)

		local pos=vectorn()
		pos:setSize(self.N_opt_dimension)

		for i=1, self.N_opt_dimension do
			local optvar=self.opt_dimension[i]
			set1(pos, i, optvar.curval)
			optvar.prevval=optvar.curval
		end

		if Optimizer.outFile then
			self:saveShort(Optimizer.outFile, "")
		end

		if rank==nil then
			local nc=self.method.testCount

			local useRandom=self.method.useRandom

			if useRandom then
				useRandom={}
				for i=1, self.N_opt_dimension do
					local optvar=self.opt_dimension[i]
					useRandom[i]=math.random()*optvar.grad_step*0.5
				end
			end

			for i=1,nc do
				coarseLog("optimize "..i)
				for ii=1, self.N_opt_dimension do
					local optvar=self.opt_dimension[ii]
					if useRandom and i==self.method.useRandomStart  then
						set1(pos, ii, optvar.curval+useRandom[ii])
					else
						set1(pos, ii, optvar.curval)
					end
					optvar.prevval=optvar.curval
				end
				coarseLog(tostring(pos))
				local eval=self:objectiveFunction(pos)
				coarseLog(eval)
			end	    
		else
			assert(rank==0) -- server process dispatch jobs
			local jobs={}
			for i=1, numCores-1 do -- any number of jobs can be dispatched. 
				jobs[i]=pos
			end
			fineLog("optimize 1")
			local eval=self:batchObjectiveFunction(jobs)
			assert(table.getn(eval)==numCores-1)
			for i=1, numCores-1 do
				fineLog(i, eval[i])
			end

			fineLog("optimize 2")
			local eval=self:batchObjectiveFunction(jobs)
			assert(table.getn(eval)==numCores-1)
			for i=1, numCores-1 do
				fineLog(i, eval[i])
			end
		end
	elseif self.method==Optimizer.methods.GradientDecent2 then
		local func=Func(self)
		self:updateGradientDecent2(func)
		local outputString="finalpos:"..tostring(func.x)
		util.outputToFile(Optimizer.outFile,outputString)			
		func:info(-1)
	elseif self.method==Optimizer.methods.ConjugateGradient then
		local func=Func(self)
		self:updateConjugateGradient(func)
		if Optimizer.outFile then
			local outputString="finalpos:"..tostring(func.x)
			util.outputToFile(Optimizer.outFile,outputString)			
		end
		func:info(-1)
	elseif self.method==Optimizer.methods.ConjugateGradient2 then
		local func=self.method.func_dfunc
		self:updateConjugateGradient(func)
		if Optimizer.outFile then
			local outputString="finalpos:"..tostring(func.x)
			util.outputToFile(Optimizer.outFile,outputString)			
		end
		func:info(-1)
	elseif self.method==Optimizer.methods.CMAes then
		self.best_eval=1.0  -- override the default setting
		self.best_eval_unscaled=1e10
		local func=Func(self)

		local stdev=vectorn(self.N_opt_dimension)
		stdev:setAllValue(self.method.stddev)

		if self.N_opt_dimension==0 then
			func:info(-1)
			return 
		end
		for restart=1, self.method.nrestart do
			local lambda=func.x:size()

			if self.method.useMPIwhenPossible and numCores then
				lambda=math.max(lambda, numCores-2) -- utilize available processes
			end
			if self.method.lambda then
				lambda=self.method.lambda
			end

			local mu=math.ceil(lambda/2)

			coarseLog("restart", restart, func.x, stdev, lambda, mu)
			local opt=math.CMAwrap(func.x, stdev,lambda, mu)

			local iter=0

			while opt:testForTermination()=="" do
				if iter==self.method.maxIteration then
					break
				end
				opt:samplePopulation()

				local jobs=array:new()
				local popId=array:new()
				for i=1, opt:numPopulation() do
					jobs:pushBack(func:Unnormalize(opt:getPopulation(i-1)))
					popId:pushBack(i-1)
				end

				local eval
				repeat
					coarseLog("batchObjectiveFunction")
					eval=self:batchObjectiveFunction(jobs)

					local output=""
					for i=1, popId:size() do
						if popId[i]==nil then
							coarseLog("error 1")
						elseif eval[i]==nil then
							coarseLog("error 2:"..i)
						elseif func.oeval==nil then
							coarseLog("error 3")
						end
						opt:setVal(popId[i], eval[i]/func.oeval)
						output=output..popId[i]..": "..eval[i]..", "
					end
					if Optimizer.outFile then
						print(output)
						util.outputToFile(Optimizer.outFile, output)
					end

					local function check_feasible(jobs, popId, eval)
						local newJob=array:new()
						local newPopId=array:new()
						local n=table.getn(eval)

						if discardErrorSamples then
							local count=0 for i=1,n do if eval[i]>=100000-1 and eval[i]<=100000+1 then count=count+1 end end
							if count< mu/2 then -- small number of errors can be ignored
								if count~=0 then coarseLog(" "..tostring(count).." errors were safely ignored.") end
								return nil, nil 
							end

							for i=1, n do
								if eval[i]>=100000-1 and eval[i]<=100000+1 then
									opt:resampleSingle(popId[i])
									newJob:pushBack(func:Unnormalize(opt:getPopulation(popId[i])))
									newPopId:pushBack(popId[i])
								end
							end
						end
						if newJob:size()==0 then return nil, nil end
						return newJob, newPopId
					end

					jobs, popId=check_feasible(jobs, popId, eval)
				until jobs==nil

				opt:update()	 

				--opt:getMean(func.x)
				opt:getBest(func.x)
				func:info(iter)
				-- local outputString=iter.." mean:"..tostring(func:Unnormalize(func.x))
				-- util.outputToFile(Optimizer.outFile,outputString)			
				iter=iter+1	 
			end
			if Optimizer.outFile then
				if fineLog~=nil then
					fineLog(opt:testForTermination())
				else
					util.outputToFile(Optimizer.outFile, opt:testForTermination())
				end

				opt:getMean(func.x)
				local outputString="finalpos:"..tostring(func:Unnormalize(func.x))
				util.outputToFile(Optimizer.outFile,outputString)			

				opt:getBest(func.x)
				local outputString="best:"..tostring(func:Unnormalize(func.x))
				util.outputToFile(Optimizer.outFile,outputString)			
			else
				opt:getBest(func.x)
			end

			func:info(-1)
		end
	elseif self.method==Optimizer.methods.CMAes_nolog then
		Optimizer.outfile_backup=Optimizer.outfile
		Optimizer.outfile=nil
		self.best_eval=1.0  -- override the default setting
		self.best_eval_unscaled=1e10
		local func=Func(self)
		func.oeval = 1	-- oeval manual setting
		discardErrorSamples=true
		local stdev=vectorn(self.N_opt_dimension)
		stdev:setAllValue(self.method.stddev)

		if self.N_opt_dimension==0 then
			func:info(-1)
			return 
		end
		for restart=1, self.method.nrestart do
			local lambda=func.x:size()

			--lambda=math.max(lambda, 62) -- test if a large population helps
			if self.method.useMPIwhenPossible and numCores then
				--lambda=math.max(lambda, numCores-2) -- utilize available processes
			end
			if self.method.lambda then
				lambda=self.method.lambda
			end

			local mu=math.ceil(lambda/2)

			local opt=math.CMAwrap(func.x, stdev,lambda, mu)

			local iter=0

			while opt:testForTermination()=="" do
				if iter==self.method.maxIteration then
					break
				end
				opt:samplePopulation()

				local jobs=array:new()
				local popId=array:new()
				for i=1, opt:numPopulation() do

					local t=func:Unnormalize(opt:getPopulation(i-1))
					if discardErrorSamples then
						local c=0
						while not self:checkFeasible(t) do
							if c>4 then
								self:checkFeasible(t)
								print('?????')
								break
							end
							if c>3 then
								local t2=self:makeFeasible(t)
								local v=t2:copy()
								func:normalize(v, t2)
								opt:resampleSingleFrom(i-1, v);
							else
								opt:resampleSingle(i-1)
							end
							t=func:Unnormalize(opt:getPopulation(i-1))
							c=c+1
						end
					end
					jobs:pushBack(t)
					popId:pushBack(i-1)
				end

				local eval=self:batchObjectiveFunction(jobs)

				for i=1, popId:size() do
					if popId[i]==nil then
						coarseLog("error 1")
					elseif eval[i]==nil then
						coarseLog("error 2:"..i)
					elseif func.oeval==nil then
						coarseLog("error 3")
					end
					opt:setVal(popId[i], eval[i]/func.oeval)
				end

				opt:update()	 

				--opt:getMean(func.x)
				opt:getBest(func.x)
				func:info(iter)
				iter=iter+1	 
			end
			opt:getBest(func.x)
			func:info(-1)
		end
		Optimizer.outfile=Optimizer.outfile_backup



	else
		local func=Func(self)
		local solver=math.GSLsolver(func,self.method.type)
		solver:solve(self.stepSize, self.method.tol, self.method.thr)
		local outputString="finalpos:"..tostring(func.x)
		util.outputToFile(Optimizer.outFile,outputString)			
		func:info(-1)
	end

end

function Optimizer:getCurPos()
	local pos=vectorn()
	pos:setSize(self.N_opt_dimension)

	for i=1, self.N_opt_dimension do
		local optvar=self.opt_dimension[i]
		set1(pos, i, optvar.curval)
	end
	return pos
end
function Optimizer:fullSearch()

	local pos=self:getCurPos()
	local eval=self:objectiveFunction(pos)
	local preveval=eval

	local gran1=5
	local gran=11

	local center=vectorn()
	-- full search
	center:assign(pos)

	local index=vectorn(self.N_opt_dimension)
	index:setAllValue(1)

	local gran=Optimizer.methods.FullSearch.numGrid
	local gridSize=Optimizer.methods.FullSearch.gridSize
	local function updateIndex(index)
		for ii=0, self.N_opt_dimension-1 do
			if index(ii)<gran(ii) then
				index:set(ii, index(ii)+1)
				for jj=0, ii-1 do
					index:set(jj, 1)
				end
				return false
			end
		end
		return true
	end

	while true do
		--titlebar:setCaption(tostring(self.iteration))

		for ii=0, self.N_opt_dimension-1 do
			local gg=gridSize(ii)
			pos:set(ii, sop.map(index(ii),1,gran(ii), gg*-1, gg)+center(ii))
		end

		preveval=eval
		eval=self:objectiveFunction(pos)

		local outputString="iter="..self.iteration.." preveval="..preveval.." eval="..eval
		for i=1,self.N_opt_dimension do
			local optvar=self.opt_dimension[i]
			outputString=outputString.." "..optvar.title..": "..optvar.curval
		end
		outputString=outputString..pos:__tostring()

		util.outputToFile(Optimizer.outFile,outputString)			
		--subtitlebar:setCaption(tostring(pos))
		if eval<preveval then
			for i=1,self.N_opt_dimension do
				local optvar=self.opt_dimension[i]
				optvar.curval=get1(pos,i)
			end						
		else
			eval=preveval
		end

		self.iteration=self.iteration+1

		if updateIndex(index) then
			break
		end
	end
end

function Optimizer:save(fn)
	local fout,msg=io.open(fn, "w")
	if fout==nil then
		util.outputToFile(Optimizer.outFile, msg)
	end

	for i=1, self.N_opt_dimension do
		local optvar=self.opt_dimension[i]
		fout:write("assert(opt_dimension["..i.."].title=='"..optvar.title.."')\n")
		fout:write("opt_dimension["..i.."].curval="..tostring(optvar.curval).."\n")
	end
	fout:close()
end
function Optimizer:save_table(fn)
	local fout,msg=io.open(fn, "w")
	if fout==nil then
		util.outputToFile(Optimizer.outFile, msg)
	end

	for i=1, self.N_opt_dimension do
		local optvar=self.opt_dimension[i]
		fout:write("['"..optvar.title.."']=")
		fout:write(tostring(optvar.curval)..", ")
	end
	fout:close()
end

function Optimizer:getOptResult()
	local outputTable={}
	for i=1,self.N_opt_dimension do
		local optvar=self.opt_dimension[i]
		outputTable[optvar.title]= optvar.curval
	end
	return outputTable
end
function Optimizer:getVecOptResult()
	local output=vectorn(self.N_opt_dimension)
	for i=1,self.N_opt_dimension do
		local optvar=self.opt_dimension[i]
		output:set(i-1, optvar.curval)
	end
	return output
end
function Optimizer:setOptResult(pos)
	for i=1,self.N_opt_dimension do
		local optvar=self.opt_dimension[i]
		optvar.curval=pos(i-1)
	end
end

function Optimizer:save2(fn)
	-- local outputString="{ "
	-- for i=1,self.N_opt_dimension do
	--    local optvar=self.opt_dimension[i]
	--    outputString=outputString..optvar.title.."= "..optvar.curval..", "
	-- end

	if self.best_eval<1.0 then
		local outputTable=self:getOptResult()
		local outputString=table.tostring(outputTable)


		local fout,msg=io.open(fn, "w")
		if fout==nil then
			util.outputToFile(Optimizer.outFile, msg)
		end
		fout:write(' cp_mod='..outputString..'\n')
		fout:close()
	else
		util.writeFile(fn, 'cp_mod={}\n')
	end
end

function Optimizer:saveShort(fn, outputString)

	local outputStringOrig=outputString
	for i=1,self.N_opt_dimension do
		local optvar=self.opt_dimension[i]
		outputString=outputString.."['"..optvar.title.."']".."= "..optvar.curval..", "
	end
	if self.best_eval<1.0 then

		util.outputToFile(Optimizer.outFile,"accumulate({"..outputString.."})")	
	else
		util.outputToFile(Optimizer.outFile,"accumulate({"..outputStringOrig.."}) -- skipped "..outputString)	
	end
end

function Optimizer:saveShort2(fn, x, outputString)

	for i=1,self.N_opt_dimension do
		local optvar=self.opt_dimension[i]
		outputString=outputString.."['"..optvar.title.."']".."= "..x(i-1)..", "
	end

	util.outputToFile(fn,"accumulate({"..outputString.."})")	
end

function Optimizer:randomizedSearch()
	local pos=vectorn()
	pos:setSize(self.N_opt_dimension)

	for i=1, self.N_opt_dimension do
		local optvar=self.opt_dimension[i]
		set1(pos, i, optvar.curval)
	end

	local eval=self:objectiveFunction(pos)
	local preveval=eval


	-- randomized search
	while true do

		for i=1, self.N_opt_dimension do
			local optvar=self.opt_dimension[i]
			set1(pos, i, optvar.curval+(math.random()-0.5)*optvar.max_step)
		end

		preveval=eval
		eval=self:objectiveFunction(pos)


		local outputString="iter="..self.iteration.." preveval="..preveval.." eval="..eval
		for i=1,self.N_opt_dimension do
			local optvar=self.opt_dimension[i]
			outputString=outputString.." "..optvar.title..": "..optvar.curval
		end
		outputString=outputString..pos:__tostring()

		util.outputToFile(Optimizer.outFile,outputString)			

		if eval<preveval then
			for i=1,self.N_opt_dimension do
				local optvar=self.opt_dimension[i]
				optvar.curval=get1(pos,i)
			end						
		else
			eval=preveval
		end

		self.iteration=self.iteration+1
		if self.iteration==self.maxIteration then
			return
		end

	end

end

function Optimizer:GArandomMutate(pos)
	local pos2=vectorn()
	pos2:setSize(self.N_opt_dimension)

	for i=1, self.N_opt_dimension do

		if math.random()>0.5 then
			local optvar=self.opt_dimension[i]
			set1(pos2, i, pos(i-1)+(math.random()-0.5)*optvar.max_step)
		end
	end
	return pos2
end

function Optimizer:GAevaluate(population)
	local nPopulation=method.nPopulation

	for i=1, nPopulation do
		if population[i].eval==12345 then
			population[i].eval=self:objectiveFunction(population[i].pos)
		end
	end
end

function Optimizer:GAsort(population)

	local function comp(a,b)
		if a.eval<b.eval then return true end
		return false
	end

	table.sort(population, comp)
end

function Optimizer:GAprint(pos, eval, outputString)
	if eval~=nil then
		outputString= outputString.." eval:"..tostring(eval)
	end

	for i=1, self.N_opt_dimension do
		local optvar=self.opt_dimension[i]
		optvar.curval=get1(pos, i)
	end

	print(outputString)
	self:saveShort(Optimizer.outFile, outputString)
end

function Optimizer:GArandomChoose(population)
	local nPopulation=method.nPopulation

	if method.summed==nil then
		method.summed={}
		method.summed[1]=nPopulation
		for i=2, nPopulation do
			method.summed[i]=method.summed[i-1]+nPopulation-i+1
		end
	end

	local rand=math.random(method.summed[nPopulation])
	for i=1, nPopulation do
		if rand<= method.summed[i] then
			return i
		end
	end
	return 1
end

function Optimizer:GAcrossOver(i, j, population, k) -- interpolate
	population[k].pos=(population[i].pos+population[j].pos)*0.5
	population[k].eval=12345
end

function Optimizer:GAcrossOver2(i,j,population,k) -- extrapolate
	local p1=population[i].pos
	local p2=population[j].pos

	if population[i].eval < population[j].eval then
		population[k].pos=(p1-(p2-p1))
	else
		population[k].pos=(p2-(p1-p2))
	end
	population[k].eval=12345
end

function Optimizer:GAsearch()
	local pos=vectorn()
	pos:setSize(self.N_opt_dimension)

	--   math.randomseed()

	for i=1, self.N_opt_dimension do
		local optvar=self.opt_dimension[i]
		set1(pos, i, optvar.curval)
	end


	local nPopulation=method.nPopulation
	population={}
	for i=1, nPopulation do
		population[i]={pos=self:GArandomMutate(pos), eval=12345}
	end
	population[1].pos=pos:copy()

	for iter=1, method.maxIteration do 
		self:GAevaluate(population)

		self:GAsort(population)

		self:GAprint(population[1].pos, population[1].eval, "iter: "..tostring(iter))

		for k=1,method.nCrossOver do
			if math.random()>0.5 then
				self:GAcrossOver(self:GArandomChoose(population), self:GArandomChoose(population), population, nPopulation-k+1)
			else
				self:GAcrossOver2(self:GArandomChoose(population), self:GArandomChoose(population), population, nPopulation-k+1)
			end
		end

		for m=method.nElite+1, nPopulation do
			if math.random()<method.pMutate then
				population[m]={pos=self:GArandomMutate(population[m].pos), eval=12345}
			end
		end
	end
end


function Optimizer:updateGradientDecent()
	local pos=vectorn()
	pos:setSize(self.N_opt_dimension)


	for i=1, self.N_opt_dimension do
		local optvar=self.opt_dimension[i]
		set1(pos, i, optvar.curval)
		optvar.prevval=optvar.curval
	end

	local eval=self:objectiveFunction(pos)
	local preveval=eval


	while true do

		for i=1, self.N_opt_dimension do
			local optvar=self.opt_dimension[i]
			set1(pos, i, optvar.curval)
		end

		-- calc gradient and update curvals
		preveval=eval
		local gradient, e=self:gradientFunction(pos)
		eval=e
		local outputString="iter="..self.iteration.." eval="..preveval.."->"..eval
		for i=1,self.N_opt_dimension do
			local optvar=self.opt_dimension[i]
			outputString=outputString.."\n  "..optvar.title..": "..optvar.prevval.."->"..optvar.curval
		end
		outputString=outputString.."\n"..(gradient*stepSize):__tostring()

		if preveval<eval or isnan(eval) then	-- the second condition is for checking QNAN
			eval=preveval	
			for i=1,self.N_opt_dimension do
				local optvar=self.opt_dimension[i]
				optvar.curval=optvar.prevval
				optvar.grad_step=optvar.grad_step*0.5
			end

			stepSize=stepSize*0.1
			util.outputToFile(Optimizer.outFile,outputString)			
			util.outputToFile(Optimizer.outFile, "Error! reducing stepsize: "..stepSize)
		else

			-- check max_step
			local maxstep_clamped=false
			for i=1,self.N_opt_dimension do
				local optvar=self.opt_dimension[i]
				local g=get1(gradient, i)
				local gs=math.abs(g*stepSize)
				if gs>optvar.max_step then
					gradient:rmult(optvar.max_step/gs)
					outputString=outputString.."->"..(gradient*stepSize):__tostring()
					maxstep_clamped=true
				end
			end

			if maxstep_clamped==false then
				stepSize=stepSize*2
			end

			for i=1,self.N_opt_dimension do
				local optvar=self.opt_dimension[i]
				optvar.prevval=optvar.curval
				optvar.curval=optvar.curval-get1(gradient, i)*stepSize
			end
			util.outputToFile(Optimizer.outFile, outputString)
		end
		self.iteration=self.iteration+1
		if self.iteration==self.maxIteration then
			return
		end
	end

end

function linmin(p,xi,func, fp, ftol)
	local fret

	local ax=0
	local bx=1

	local fa= fp

	if fa==nil then
		fa=func:func(p)
	end

	--    local fb= func:func(p+bx*xi)

	--    local xmin=0
	--    p:assign(p+xmin*xi)

	--    util.printFile(Optimizer.outFile, "linmin xmin", xmin, fret)

	--    return fa
	-- end

	local fb= func:func(p+xi*bx)


	if Optimizer.outFile then
		util.printFile(Optimizer.outFile, "a,b", ax, bx, fa,fb)
	end

	-- mnbrak start
	if fb > fa then 
		fb,fa=fa,fb
		bx,ax=ax,bx
	end


	local GOLD=1.62
	local TINY=1.0e-20
	local GLIMIT=100

	cx=bx+GOLD*(bx-ax)

	local function func1(x)
		return func:func(p+x*xi)
	end

	fc=func1(cx)

	local function SIGN(a,b)
		if b<0 then
			return -1*math.abs(a)
		end
		return math.abs(a)
	end

	local r,q,u,ulim,fu


	while fb>fc do -- fc is the current minimum
		r = (bx - ax) * (fb - fc)
		q = (bx - cx) * (fb - fa)
		u = bx - ((bx - cx) * q - (bx - ax) * r) / (2.0 * SIGN(math.max(math.abs(q - r), TINY), q - r))
		ulim = bx + GLIMIT * (cx - bx)

		if ((bx - u) * (u - cx) > 0.0) then
			fu = func1(u)
			if (fu < fc) then
				ax = bx;
				bx = u;
				fa = fb;
				fb = fu;
				break;
			elseif (fu > fb) then
				cx = u;
				fc = fu;
				break;
			end
			u = cx + GOLD * (cx - bx);
			fu = func1(u)
		elseif ((cx - u) * (u - ulim) > 0.0) then
			fu = func1(u)
			if (fu < fc) then
				bx=cx
				cx=u 
				u=cx + GOLD * (cx - bx)
				fb=fc 
				fc=fu 
				fu=func1(u)
			end
		elseif ((u - ulim) * (ulim - cx) >= 0.0) then
			u = ulim;
			fu = func1(u)
		else
			u = cx + GOLD * (cx - bx);
			fu = func1(u)
		end

		ax=bx
		bx=cx
		cx=u

		fa=fb
		fb=fc
		fc=fu
	end
	if Optimizer.outFile then
		util.printFile(Optimizer.outFile, "a,b,c=", ax, bx, cx, fa,fb,fc)
	end

	assert(fa>=fb)
	assert(fb<=fc)

	-- brent start
	do
		local iter
		local a, b, d, etemp, fu, fv, fw, fx, p, q, r, tol1, tol2, u, v, w, x, xm;
		local e=0.0
		local ITMAX=200
		local ZEPS=1.0e-10
		local CGOLD=0.3819660
		a = math.min(ax, cx)
		b = math.max(ax, cx)
		v=bx
		w=bx
		x=bx
		--      fx = func1 (x);
		fx = fb
		fv = fx
		fw = fv

		for iter=1,ITMAX do
			xm = 0.5 * (a + b);
			tol1=ftol * math.abs(x) + ZEPS
			tol2 = 2.0 * tol1 ;

			if (math.abs(x - xm) <= (tol2 - 0.5 * (b - a))) then
				xmin = x;
				fret= fx;
				break
			end

			if (math.abs(e) > tol1) then

				r = (x - w) * (fx - fv);
				q = (x - v) * (fx - fw);
				p = (x - v) * q - (x - w) * r;
				q = 2.0 * (q - r);
				if (q > 0.0) then
					p = -p;
				end
				q = math.abs(q);
				etemp = e;
				e = d;
				if (math.abs(p) >= math.abs(0.5*q*etemp) or p <= q*(a-x) or p >= q*(b-x)) then
					if x>=xm then
						e=a-x
					else
						e=b-x
					end
					d = CGOLD * e
				else
					d = p / q;
					u = x + d;
					if (u - a < tol2 or b - u < tol2) then
						d = SIGN(tol1, xm - x);
					end
				end
			else
				if x>= xm then
					e=a-x
				else
					e=b-x
				end

				d = CGOLD * e
			end

			if math.abs(d)>=tol1 then
				u=x+d
			else
				u=x+SIGN(tol1, d);
			end

			fu = func1 (u);

			if (fu <= fx) then
				if (u >= x) then	a = x;
				else b = x; end
				v=w;w=x;x=u
				fv=fw;fw=fx;fx=fu
			else
				if (u < x) then a = u; 
				else b = u; end

				if (fu <= fw or w == x) then
					v = w;
					w = u;
					fv = fw;
					fw = fu;
				elseif (fu <= fv or v == x or v == w) then
					v = u;
					fv = fu;
				end
			end
		end 

		xmin = x;
		fret= fx;
	end

	p:assign(p+xmin*xi)

	if Optimizer.outFile then
		util.printFile(Optimizer.outFile, "linmin xmin", xmin, fret)
	end

	assert(fret<=fb)
	return fret
end

function Optimizer:updateGradientDecent2(func)
	local xi=vectorn()
	xi:resize(func:getInout():size())

	local p=func:getInout()
	local tol=self.method.tol
	local tol2=self.method.thr
	local DBL_EPSILON=2.22e-16
	local ITMAX=100
	for iter=0,ITMAX do
		local fp=func:func_dfunc(p, xi)

		util.printFile(Optimizer.outFile,"beforeLinMin", p, fp)
		local fret=linmin(p, xi, func, fp, tol2)

		util.printFile(Optimizer.outFile,"afterLinMin", p, fret)
		func:info(iter)

		if (2*math.abs(fret-fp)<=tol*(math.abs(fret)+math.abs(fp)+DBL_EPSILON)) then
			return
		end
	end

	print("too many iterations in gradientDescent")
end


function Optimizer:updateConjugateGradient(func)
	local xi=vectorn()
	xi:resize(func:getInout():size())

	local p=func:getInout()
	local tol=self.method.tol
	local tol2=self.method.thr
	local DBL_EPSILON=2.22e-16
	local ITMAX=20

	local n=p:size()
	local gg, gam, fp, dgg;

	local g=vectorn(n)
	local h=vectorn(n)
	local xi=vectorn(n)


	fp = func:func_dfunc (p, xi);

	--    util.outputToFile("optimizelog.txt", "test1"..fp)

	-- fp=func:func(p)

	--    util.outputToFile("optimizelog.txt", "test2"..fp)

	-- fp=func:func(p)

	--    util.outputToFile("optimizelog.txt", "test3"..fp)

	-- fp = func:func_dfunc (p, xi);

	--    util.outputToFile("optimizelog.txt", "test4"..fp)


	for j=0,n-1 do
		g:set(j, -xi(j))
		h:set(j,g(j))
		xi:set(j,h(j))
	end

	local prevFret=60000
	for iter=0,ITMAX-1 do

		local outputString="iter="..iter.." eval="..fp
		util.outputToFile("optimizelog.txt", outputString)

		fret=linmin(p, xi, func, nil, tol2);

		if iter>10 and prevFret==fret then
			break
		end
		prevFret=fret

		if (2.0 * math.abs(fret - fp) <= tol * (math.abs(fret) + math.abs(fp) + DBL_EPSILON)) then 
			return; 
		end

		fp = func:func_dfunc (p, xi);
		gg=0.0
		dgg = gg

		for j=0,n-1 do
			gg=gg+g(j)*g(j)
			dgg=dgg+(xi(j)+g(j))*xi(j)
		end

		func:info(iter,fp)

		if (gg == 0.0) then return; end

		gam = dgg / gg;

		for j=0,n-1 do
			g:set(j, -1*xi(j))
			h:set(j, g(j)+gam*h(j))
			xi:set(j, h(j))
		end
	end


	print("too many iterations in FRPRMN")
end

-- calc numerical gradient
function Optimizer:gradientFunction(_pos)
	local gradient=vectorn()
	local pos=vectorn()
	pos:assign(_pos)
	gradient:setSize(self.N_opt_dimension )

	local verbose=false
	if verbose then print(pos) end

	local posstr=" "..pos:__tostring()

	local p0
	local p1
	if verbose then titlebar:setCaption("iter :"..self.iteration.." pos:"..posstr) end
	p0=self:objectiveFunction(pos)
	if verbose then subtitlebar:setCaption("objective val: "..tostring(p0)) end

	for i=1,self.N_opt_dimension do

		local optvar=self.opt_dimension[i]
		if verbose then titlebar:setCaption("iter :"..self.iteration.." dim :"..optvar.title.." +"..optvar.grad_step..posstr) end
		set1(pos, i, get1(pos, i)+optvar.grad_step)
		p1=self:objectiveFunction(pos)

		--coarseLog('grad',pos,p1)
		set1(pos, i, get1(pos, i)-optvar.grad_step) -- recover pos

		if discardErrorSamples then
			if p1==100000 then
				set1(gradient, i,0) -- turn off the dimension
			else
				set1(gradient, i, (p1-p0)/(optvar.grad_step))
			end
		else
			set1(gradient, i, (p1-p0)/(optvar.grad_step))
		end
	end

	if verbose then coarseLog('gradent', gradient, p0) end
	return gradient, p0
end

-- override this
function Optimizer:objectiveFunction(pos)
	return 0
end

--function onCallback(w, userData)
--end

function Optimizer:batchObjectiveFunction(table_pos)
	local N=table.getn(table_pos)
	local eval={}
	for i=1,N do
		eval[i]=self:objectiveFunction(table_pos[i])
	end
	return eval
end
