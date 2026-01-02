--[[ config=
	{
		{
			fingers={'LThumb1', 'LIndex1', 'LMiddle1', 'LRing1', 'LLittle1'},
			{'LThumb3', vector3(3.0,0.0,0.0), reversed=true},
			{'LIndex3', vector3(3.0,0.0,0.0), reversed=true},
			{'LIndex2', vector3(1.0,0.0,0.0), reversed=true},
			{'LIndex1', vector3(1.0,0.0,0.0), reversed=true},
			{'LMiddle3', vector3(3.0,0.0,0.0), reversed=true},
			{'LRing3', vector3(3.0,0.0,0.0), reversed=true},
			{'LLittle3', vector3(3.0,0.0,0.0), reversed=true},
	
		},
		{
			{'LeftElbow', 'LeftWrist', vector3(0.000000,0.0,0), reversed=true},
		},			
	}
	]]--

function createHandIKsolver(loader, config)
	local mEffectors=MotionUtil.Effectors()
	local wristEffectors = MotionUtil.Effectors()
	local numCon=#config[1]
	local numHand=#config[2]
	
	local fingers=config[1].fingers
	local handIndex=intvectorn(#fingers)
	for i,bone in ipairs(fingers) do
		handIndex:set(i-1,loader:getBoneByName(bone):treeIndex())
	end
	mEffectors:resize(numCon);
	wristEffectors:resize(numHand);

	local wristIndex = intvectorn(numHand)
	local axis=vectorn(numCon)
	local wristAxis=vectorn(numHand)

	for i=0, numCon-1 do
		local conInfo=config[1][i+1]
		mEffectors(i):init(loader:getBoneByName(conInfo[1]), conInfo[2])
		if conInfo.reversed then
			axis:set(i,-1)
		else
			axis:set(i,1)
		end
	end

	for i=0,numHand-1 do
		local conInfo=config[2][i+1]
		local bone=loader:getBoneByName(conInfo[1])
		wristEffectors(i):init(loader:getBoneByName(conInfo[2]),conInfo[3])
		wristIndex:set(i,bone:treeIndex())
		if conInfo.reversed then
			wristAxis:set(i,-1)
		else
			wristAxis:set(i,1)
		end
	end
		

	--wristIK = LimbIKsolver(loader.dofInfo,wristEffectors,wristIndex,wristAxis)
	local mIK=HandIKsolver(loader.dofInfo,mEffectors, handIndex,wristIndex, axis)
	return mIK, numCon, mEffectors, wristEffectors
end
--[[
--config_gymnist_hand={
		{'lradius', 'lhand', vector3(0,0,0), reversed=true},
		{'rradius', 'rhand', vector3(0,0,0), reversed=true},
}
]]--
solvers={ LimbIKsolver=1, LimbIKsolver2=2, MultiTarget=3 , MultiTarget_lbfgs=4, MultiTarget_selected=5, MultiTarget_selected2=6, COM_IK=7, JT=8, LUA=9 , UTPoser=9, LimbIKsolverHybrid=10, MultiTarget_relative=11, MultiTarget_relative_lbfgs=12, LimbIKsolverT=13}
function createIKsolver(solverType, loader, config)
	if loader.loader then
		-- fbxloader
		loader=loader.loader
	end
	local out={}
	local mEffectors=MotionUtil.Effectors()
	local numCon=#config
	mEffectors:resize(numCon);
	local _hipIndex=intvectorn() -- used only when the config contains shoulder joints, e.g. {'lshoulder', 'lradius', 'lhand', vector3(0,0,0), reversed=true},
	local kneeIndex=intvectorn(numCon)
	local axis=vectorn(numCon)
	out.effectors=mEffectors
	out.numCon=numCon
	out.kneeIndex=kneeIndex
	out._hipIndex=_hipIndex
	out.axis=axis

	for i=0, numCon-1 do
		local conInfo=config[i+1]
		local kneeInfo=1
		if #conInfo==4 then
			_hipIndex:resize(numCon)
			local lhip=loader:getBoneByName(conInfo[1])
			_hipIndex:set(i, lhip:treeIndex())
			kneeInfo=2
		elseif #conInfo==2 then
			kneeInfo=0
		end
		if dbg.lunaType(conInfo[kneeInfo+1])=='Bone' then
			mEffectors(i):init(conInfo[kneeInfo+1], conInfo[kneeInfo+2])
			conInfo[kneeInfo+1]=conInfo[kneeInfo+1]:name()
		else
			mEffectors(i):init(loader:getBoneByName(conInfo[kneeInfo+1]), conInfo[kneeInfo+2])
		end
		if kneeInfo==0 then
			kneeIndex:set(i, mEffectors(i).bone:parent():treeIndex())
		else
			local lknee=loader:getBoneByName(conInfo[kneeInfo])
			kneeIndex:set(i, lknee:treeIndex())
		end
		if conInfo.reversed then
			axis:set(i,-1)
		else
			axis:set(i,1)
		end
	end
	out.solver=_createIKsolver(solverType,loader,mEffectors, kneeIndex, axis, _hipIndex, config)
	return out
end

-- simplified_config for both python and lua
function createLimbIksolverToeAndHeel(loader, simplified_config)
	local config={}
	for i, eff in ipairs(simplified_config) do
		local new_eff={ 
			eff.bone, eff.lpos, reversed=eff.reversed
		}
		table.insert(config, new_eff)
		if math.fmod(i-1,2)==0 then
			new_eff.childCon=i+1
		end
	end
	local out=createIKsolver(solvers.LimbIKsolverT, loader, config)
	out.solver:setOption('lengthAdjust', true)
	MotionUtil.setMaxLengthAdjustmentRatio(0.1)
	out.IKsolve=function (self, pose, conPos, _importance)
		return self.solver:IKsolve(pose, conPos, _importance)
	end
	return out
end
--simplified_config={
--	{ bone=mLoader.loader:getBoneByVoca(MotionLoader.LEFTANKLE), lpos=vector3(0,-3,17), reversed=false, }
--	{ bone=mLoader.loader:getBoneByVoca(MotionLoader.RIGHTANKLE), lpos=vector3(0,-3,17), reversed=false, }
--}
function createLimbIksolverToeOnly(loader, simplified_config)
	local config={}
	for i, eff in ipairs(simplified_config) do
		local new_eff={ 
			eff.bone, eff.lpos, reversed=eff.reversed
		}
		table.insert(config, new_eff)
	end
	local out=createIKsolver(solvers.LimbIKsolverT, loader, config)
	out.solver:setOption('lengthAdjust', true)
	MotionUtil.setMaxLengthAdjustmentRatio(0.1)
	out.IKsolve=function (self, pose, conPos, _importance)
		return self.solver:IKsolve(pose, conPos, _importance)
	end
	return out
end

function MotionUtil.Effectors:getCurrentPosition(i)
	if not i then
		local out=vector3N(self:size())
		for i=0, self:size()-1 do
			local eff=self(i)
			out(i):assign(eff.bone:getFrame()*eff.localpos)
		end
		return out
	else
		local eff=self(i)
		return eff.bone:getFrame()*eff.localpos
	end
end

function createIKsolver_sub(solverType, loader, config)
	local out={}
	local mEffectors=MotionUtil.Effectors()
	local numCon=1
	mEffectors:resize(numCon);
	local _hipIndex=intvectorn() -- used only when the config contains shoulder joints, e.g. {'lshoulder', 'lradius', 'lhand', vector3(0,0,0), reversed=true},
	local kneeIndex=intvectorn(numCon)
	local axis=vectorn(numCon)
	out.effectors=mEffectors
	out.numCon=numCon
	out.kneeIndex=kneeIndex
	out._hipIndex=_hipIndex
	out.axis=axis

	for i=0, numCon-1 do
		local conInfo=config
		local kneeInfo=1
		if #conInfo==4 then
			_hipIndex:resize(numCon)
			local lhip=loader:getBoneByName(conInfo[1])
			_hipIndex:set(i, lhip:treeIndex())
			kneeInfo=2
		end
		--dbg.console()
		--print('coninfo :',conInfo[kneeInfo])
		local lknee=loader:getBoneByName(conInfo[kneeInfo])
		mEffectors(i):init(loader:getBoneByName(conInfo[kneeInfo+1]), conInfo[kneeInfo+2])
		kneeIndex:set(i, lknee:treeIndex())
		if conInfo.reversed then
			axis:set(i,-1)
		else
			axis:set(i,1)
		end
	end
	out.solver=_createIKsolver(solverType,loader,mEffectors, kneeIndex, axis, _hipIndex)
	return out
end
function Bone:getAxis(i)
	if i<0 then
		i=self:getRotationalChannels():len()+i
	end
	assert(i<self:getRotationalChannels():len())
	local c=self:getRotationalChannels():sub(i+1,i+1)
	if c=='X' then
		return vector3(1,0,0)
	elseif c=='Y' then
		return vector3(0,1,0)
	elseif c=='Z' then
		return vector3(0,0,1)
	elseif c=='A' then
		return self:getArbitraryAxis(i)
	end
end

LimbIKsolverT=LUAclass()
function LimbIKsolverT:__init(dofInfo, eff, kneeIndex, axis, config)
	self.dofInfo=dofInfo
	self.skel=dofInfo:skeleton()
	self.effectors=eff
	self.kneeIndex=kneeIndex
	self.axis=axis
	self.config=deepCopyTable(config)
	for i, limbConfig in ipairs(self.config) do
		if limbConfig[#limbConfig]:length()==0 then
			local ankle=self.effectors(i-1).bone
			if ankle:parent():treeIndex()~=kneeIndex(i-1) then
				local newoffset=ankle:getOffset()
				self.effectors(i-1):init(ankle:parent(), newoffset)
				local con_i=self.config[i]
				con_i[#con_i-1]=ankle:parent():name()
				con_i[#con_i]=newoffset:copy()
			end

		end
		if limbConfig.childCon then
			local c=limbConfig.childCon
			self.config[c].unused=true
			self.config[c].localpos=self.effectors(c-1).localpos:copy()
			if self.effectors(c-1).bone:parent()==self.effectors(i-1).bone then
				self.config[c].localpos:radd(self.effectors(c-1).bone:getOffset())
			end
		end
	end
	self.options={}
end
function LimbIKsolverT:setValue()
end
function LimbIKsolverT:setOption(type, value)
	self.options[type]=value
end
function LimbIKsolverT:IKsolve(pose, conPos, _importance)

	local origRootTF=pose:toTransf(0)

	local skel=self.skel
	skel:setPoseDOF(pose)

	local goal=vector3()
	local sh=vector3() 
	local elb=vector3() 
	local v1=vector3() 
	local v2=vector3() 
	local v3=vector3() 
	local v4=vector3() 
	local wrist=vector3()
	local hand=vector3()

	local q0=quater()
	local q1=quater()
	local q2=quater()
	local q3=quater()
	local qo1=quater()
	local qo2=quater()
	local qt=quater()

	local len={}
	for c=0, self.effectors:size()-1 do
		local limbconfig=self.config[c+1]
		if not limbconfig.unused then
			local kneeBone=skel:bone(self.kneeIndex(c))
			local hipBone=kneeBone:parent()
			local ankleBone=self.effectors(c).bone
			local ankleGlobal=ankleBone:getFrame().rotation:copy()
			hipBone:parent():getRotation(q0)
			hipBone:getTranslation(sh);
			hipBone:getRotation(q1);
			kneeBone:getTranslation(elb);
			kneeBone:getRotation(q2);
			ankleBone:getTranslation(wrist);
			v1=q1:inverse()*(elb-sh)
			v2=q2:inverse()*(wrist-elb)

			local ccon=limbconfig.childCon 
			local importance=1
			if _importance then
				importance=_importance(c)
			end
			if ccon then
				local cimportance=1
				local pimportance=importance
				local lpos= self.config[ccon].localpos
				if _importance then
					cimportance=_importance(ccon-1)
					importance=math.max(cimportance, importance)
				end
				do
					local i0=cimportance /(cimportance+pimportance+1e-3)
					local i1=pimportance/(cimportance+pimportance+1e-3)
					local weight=0.5*(i0-i1)+0.5
					if true then
						-- adjust global ankle orientation part1.
						local chand=ankleBone:getFrame()*lpos;
						local cgoal=conPos(ccon-1)-chand+wrist;

						local hand=ankleBone:getFrame()*self.effectors(c).localpos;
						local pgoal=conPos(c)-hand+wrist;

						local gpos=vector3()
						gpos:interpolate(weight,  pgoal, cgoal)
						local curr_dir=ankleBone:getFrame().translation - hipBone:getFrame().translation
						local desired_dir=gpos-hipBone:getFrame().translation
						local delta=quater()
						delta:axisToAxis(curr_dir, desired_dir)
						delta:scale(importance)
						ankleGlobal:leftMult(delta)
						ankleBone:getFrame().rotation:assign(ankleGlobal)
					end

					local gpos=ankleBone:getFrame()*lpos
					local curr_dir=gpos-ankleBone:getFrame()*self.effectors(c).localpos;
					local desired_dir=conPos(ccon-1)-conPos(c)
					local delta=quater()
					delta:axisToAxis(curr_dir, desired_dir)
					-- adjust  global ankle orientation.
					delta:scale(math.min(pimportance, cimportance))
					ankleGlobal:leftMult(delta)
					ankleBone:getFrame().rotation:assign(ankleGlobal)
					local chand=ankleBone:getFrame()*lpos;
					local cgoal=conPos(ccon-1)-chand+wrist;

					local hand=ankleBone:getFrame()*self.effectors(c).localpos;
					local pgoal=conPos(c)-hand+wrist;

					goal:interpolate(weight,  pgoal, cgoal)
					--goal:interpolate(0,  pgoal, cgoal)
				end
			end
			if not ccon then
				-- preserve original global ankle orientation.
				ankleBone:getFrame().rotation:assign(ankleGlobal)
				assert(self.effectors(c).bone==ankleBone)
				local hand=ankleBone:getFrame()*self.effectors(c).localpos;
				goal=conPos(c)-hand+wrist;
			end

			v3:difference(sh, elb);
			v4:difference(elb, ankleBone:getTranslation());

			local useKneeDamping=true

			qo1:assign(q1);
			qo2:assign(q2);

			local r=0
			if self.options.lengthAdjust then
				r=MotionUtil.limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, kneeBone:getAxis(-1)*self.axis(c), useKneeDamping, 1.0, true);

				local d1=v3:length()
				local d2=v4:length()

				local s1=(d1+r)/d1
				local s2=(d2+r)/d2

				if importance~=1.0 then
					s1=sop.map(importance, 0, 1, 1, s1)
					s2=sop.map(importance, 0, 1, 1, s2)
				end
				table.insert(len,{hipBone:treeIndex(), s1, kneeBone:treeIndex(), s2})
			else
				MotionUtil.limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, kneeBone:getAxis(-1)*self.axis(c), useKneeDamping);
			end

			if importance ~=1.0 then
				qt:difference(qo1, q1);
				qt:scale(importance);
				q1=qt*qo1;

				qt:difference(qo2, q2);
				qt:scale(importance);
				q2=qt*qo2;
			end

			hipBone:getLocalFrame().rotation:assign(q0:inverse()*q1);
			kneeBone:getLocalFrame().rotation:assign(q1:inverse()*q2);
			ankleBone:getLocalFrame().rotation:assign(q2:inverse()*ankleGlobal);
			skel:fkSolver():setChain(ankleBone)
		end
	end
	skel:getPoseDOF(pose)
	if #len>0 then
		local lenScale=vectorn(skel:numBone())
		lenScale:setAllValue(1.0)
		for i, v in ipairs(len) do
			lenScale:set(v[1], v[2])
			lenScale:set(v[3], v[4])
		end
		return lenScale
	end
end

function _createIKsolver(solverType, loader, eff, kneeIndex, axis , _hipIndex, config)

	if solverType==solvers.LimbIKsolver2 then
		local solver= LimbIKsolver2(loader.dofInfo,eff, kneeIndex, axis)
		solver:setOption('optiR', 1)
		solver:setOption('optiT',1)	
		solver:setOption('iterik',1)	
		ValL = ValL or 100
		ValM = ValM or 30
		ValN = ValN or 10
		iterNum = iterNum or 2
		solver:setValue(ValL,ValM,ValN,iterNum)
		return solver
	elseif solverType==solvers.LimbIKsolverT then 
		solver=LimbIKsolverT(loader.dofInfo,eff, kneeIndex, axis, config)
		return solver
	elseif solverType==solvers.LimbIKsolverHybrid then
		local solver
		if _hipIndex and _hipIndex:size()>0 then
			solver= LimbIKsolverHybrid(loader.dofInfo,eff, _hipIndex, kneeIndex, axis)
		else
			solver= LimbIKsolverHybrid(loader.dofInfo,eff, kneeIndex, axis)
		end
		if solver.setValue==nil then
			LimbIKsolverHybrid.setValue=function() end
		end
		return solver
	elseif solverType==solvers.LUA then
		local solver
		if _hipIndex and _hipIndex:size()>0 then
			solver= LimbIKsolverLua(loader.dofInfo,eff, _hipIndex, kneeIndex, axis, this("getState"))
		else
			solver= LimbIKsolverLua(loader.dofInfo,eff, kneeIndex, axis, this("getState"))
		end

		if solver.setValue==nil then
			LimbIKsolverLua.setValue=function() end
		end
		if _IKsolve==nil then
			assert(_objectiveFunction==nil)
			function _IKsolve(solver, pose, newRootTF, conpos, conori, importance)
				-- results go to solver.tempp
				local dim=6
				local max_iter=10
				local tol=0.001 -- used in frprmn termination condition
				local thr=1 -- used in NR_brek
				solver:init_cg(dim, 0.005, max_iter, tol, thr)
				local v=vectorn(dim)
				v:setAllValue(0)
				solver:optimize(v)
				local out=solver:getResult()
				_objectiveFunction(solver, out)
				solver.mSkeleton:getPoseDOF(pose)
			end
			function _objectiveFunction(solver, x)
				local eff=mEffectors
				--local hips=vector3N(eff:size())
				local pelvis=solver:getCenterBone(0)
				pelvis:getLocalFrame().translation:add(solver.mRootPos(0),x:toVector3(0))

				local theta=quater()
				theta:setRotation(x:toVector3(3))
				pelvis:getLocalFrame().rotation:mult(solver.mRootOri(0),theta)
				solver.mSkeleton:fkSolver():forwardKinematics()
				solver:_limbIK(solver.con, solver.conori, solver.impor)
				
				local d=0
				for c=0, solver.mEffectors:size()-1 do
					local eff=solver.mEffectors(c)
					local cpos=eff.bone:getFrame():toGlobalPos(eff.localpos)
					d=d+cpos:squaredDistance(solver.con(c))
				end
				-- skin scale 이 100인 경우에 적합하게 튜닝되어 있음.
				-- 
				local skinScale=100
				local w=skinScale/100
				local l=x:length()
				d=d*w*w+0.1*l*l
				return d
			end
		end

		return solver
	else
		local solver

		if solverType==solvers.LimbIKsolver then
			if _hipIndex and _hipIndex:size()>0 then
				print(axis)
				solver=LimbIKsolver(loader.dofInfo,eff, _hipIndex, kneeIndex, axis)
			else
				print(axis)
				solver=LimbIKsolver(loader.dofInfo,eff, kneeIndex, axis)
			end
		elseif solverType==solvers.MultiTarget or solverType==solvers.MultiTarget_lbfgs then
			--solver=MotionUtil.createFullbodyIkDOF_limbIK_straight(loader.dofInfo,eff,lknee,rknee);
			if solverType==solvers.MultiTarget_lbfgs then
				g_con=MotionUtil.Constraints() -- std::vector<MotionUtil::RelativeConstraint>
				solver=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(loader.dofInfo, eff,g_con);
			else
				solver=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget(loader.dofInfo, eff);
			end
		elseif solverType==solvers.MultiTarget_relative then
			g_con=MotionUtil.Constraints() -- std::vector<MotionUtil::RelativeConstraint>
			g_con:resize(1)
			local bone1=loader:getBoneByName("lfoot");
			local bone2=loader:getBoneByName("lhand");
			g_con(0):init(bone1, bone2, vector3(0,0,0))

			solver=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget(loader.dofInfo, eff, g_con);
		elseif solverType==solvers.MultiTarget_relative_lbfgs then
			g_con=MotionUtil.Constraints() -- std::vector<MotionUtil::RelativeConstraint>
			g_con:resize(1)
			local bone1=loader:getBoneByName("lfoot");
			local bone2=loader:getBoneByName("lhand");
			g_con(0):init(bone1, bone2, vector3(0,0,0))

			solver=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(loader.dofInfo, eff, g_con);
		elseif solverType==solvers.MultiTarget_selected or solverType==solvers.MultiTarget_selected2 then
			local IKtarget=intvectorn()
			IKtarget:pushBack(1) -- root
			local IKconfig=config[3]
			IKtarget:pushBack(loader:getTreeIndexByName(IKconfig[1][1]))
			IKtarget:pushBack(loader:getTreeIndexByName(IKconfig[1][2]))
			IKtarget:pushBack(loader:getTreeIndexByName(IKconfig[2][1]))
			IKtarget:pushBack(loader:getTreeIndexByName(IKconfig[2][2]))
			IKtarget:pushBack(loader:getTreeIndexByName(IKconfig[3][1]))
			IKtarget:pushBack(loader:getTreeIndexByName(IKconfig[3][2]))
			IKtarget:pushBack(loader:getTreeIndexByName(IKconfig[4][1]))
			IKtarget:pushBack(loader:getTreeIndexByName(IKconfig[4][2]))
			if solverType==solvers.MultiTarget_selected then
				solver=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_Selected(loader.dofInfo, eff, IKtarget);
			else
				solver=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_Selected(loader.dofInfo, MotionUtil.Effectors(), IKtarget);
			end
		elseif solverType==solvers.UTPoser then
			--solver=MotionUtil.createFullbodyIkDOF_limbIK_straight(loader.dofInfo,eff,lknee,rknee);
			solver=MotionUtil.createFullbodyIkDOF_UTPoser(loader.dofInfo, eff);
		elseif solverType==solvers.COM_IK then
			solver=COM_IKsolver(loader, eff, kneeIndex, axis)

			loader:setPoseDOF(mMotionDOF:row(0));
			mDesiredCOM=loader:calcCOM()

			local fcnTable=getmetatable(solver)
			fcnTable.IKsolve3=function (self, pose, rootTF,footPos, footOri, importance)

				local conDelta=quaterN(numCon)
				for i=0, numCon -1 do
					conDelta(i):assign(quater(1,0,0,0))
				end
				return self:IKsolve(pose, rootTF.rotation:rotationY(), rootTF, conDelta, footPos, importance, mDesiredCOM)
			end
		elseif solverType==solvers.JT then
			require("subRoutines/ikSolver")
			local effectors={}
			local knees={}
			for i=0,  numCon -1 do
				effectors[i+1]={mEffectors(i).bone, mEffectors(i).localpos}
				knees[i+1]={loader:getBoneByTreeIndex(kneeIndex(i)), 1, 170}
			end
			useCase={
				IKnumIter=500,
				IKspeedBoost=1,
				IKtimestep=0.01
			}
			solver=IKsolver(loader, effectors, knees)
			local fcnTable=solver
			fcnTable.IKsolve3=function (self, pose, rootTF,footPos, footOri, importance)
				local positions={}
				for i=1, numCon do
					local pos=footPos(i-1)
					positions[i]={pos, importance(i-1)}
				end
				self:solve(rootTF.rotation:rotationY(), rootTF, pose, unpack(positions))
			end
		end
		--[[
		MotionUtil.createFullbodyIkDOF_limbIK(loader.dofInfo, mEffectors, lknee, rknee, config.reversed);
		]]
		local fcnTable=getmetatable(solver) 

		-- setOption 이나 setValue함수가 없으면 아무일 하지 않는 함수를 만들어 에러가 나지 않도록 한다.
		if not fcnTable.setOption then
			fcnTable.setOption=function() end
		end
		if not fcnTable.setValue then
			fcnTable.setValue=function() end
		end
		return solver
	end
end

Displacement={}

function Displacement.calcDisplacement(poses1, poses2)
	local out=matrixn(poses1[0]:rows(), 3+#poses1*4)
	out:sub(0,0, 0,3):assign(poses1[0]:matView()-poses2[0]:matView())

	local tempq=quaterN(poses1[1]:rows())
	for j=1, #poses1 do
		local q1=poses1[j]
		local q2=poses2[j]
		for i=0,q1:rows()-1 do
			tempq(i):difference(q1(i), q2(i))
		end
		tempq(0):align(quater(1,0,0,0))
		tempq:align()
		out:sub(0,0, 3+(j-1)*4, 3+j*4):assign(tempq:matView())
	end
	return out
end
function Displacement.applyDisplacement(output_poses, input_poses, disp)
	local out=output_poses:matView():sub(0, disp:rows(),0,0)
	local input=input_poses:matView():sub(0, disp:rows(),0,0)

	out:sub(0,0,0,3):assign(input:sub(0,0,0,3)- disp:sub(0,0,0,3))

	local fk=mLoader:fkSolver()
	local conpos=vector3N(2)
	local conori=quaterN(2)

	for i=0, input:rows()-1 do
		fk:setPoseDOF(input:row(i))
		for c=0, 1 do
			local tf=fk:globalFrame(mEffectors_foot(c).bone:treeIndex())
			conpos(c):assign(tf.translation)
			conori(c):assign(tf.rotation)
		end

		fk:localFrame(1).translation:assign(out:row(i):toVector3(0))
		for j=1,mLoader:numBone()-1 do
			local q=disp:row(i):toQuater(3+(j-1)*4)
			q:normalize()
			fk:localFrame(j).rotation:leftMult(q)
		end
		fk:forwardKinematics()
		fk:getPoseDOFfromGlobal(out:row(i))
		local pose=out:row(i)
		mIK_foot:IKsolve3(pose, MotionDOF.rootTransformation(pose), conpos, conori, CT.vec(1,1))
	end
	--[[
	local out=matrixn(poses1[0]:rows(), 3+#poses1*4)
	out:sub(0,0, 0,3):assign(poses1[0]:matView()-poses2[0]:matView())

	local tempq=quaterN(poses1[1]:rows())
	for j=1, #poses1 do
		local q1=poses1[j]
		local q2=poses2[j]
		for i=0,q1:rows()-1 do
			tempq(i):difference(q1(i), q2(i))
		end
		tempq(0):align(quater(1,0,0,0))
		tempq:align()
		out:sub(0,0, 3+(j-1)*4, 3+j*4):assign(tempq:matView())
	end
	return out
	]]--
end

-- support all combinations of skeleton and motion formats
-- the output is always in the MotionDOF format
-- loadmotion('a.wrl', 'a.dof', 100) or loadmotion('a.wrl', 'a.bvh', 100) or loadmotion('a.bvh', nil, 100)
function loadMotion(skel, motion, skinScale)
	local mot={}
	local skel_ext=''
	if type(skel)=='string' then
		skel_ext=string.upper(string.sub(skel,-3))
	end
	if skel_ext=='WRL' or skel_ext=='LUA' or skel_ext=='' then
		if skel_ext=='LUA' then
			require('subRoutines/WRLloader')
			mot.loader=MainLib.WRLloader (skel)
		elseif skel_ext=='' then
			if skel then
				mot.loader=skel
			else
				local l=RE.createMotionLoaderExt_cpp(motion)
				-- bvh files are usually  in cm unit.
				mot.loader= l:toVRMLloader(2.5)
			end
		else
			mot.loader=MainLib.VRMLloader (skel)
		end

		if motion==nil or motion=='' then
			mot.motionDOFcontainer=MotionDOFcontainer(mot.loader.dofInfo)
			mot.motionDOFcontainer:resize(100)
			mot.motionDOFcontainer.mot:matView():setAllValue(0)
			mot.motionDOFcontainer.mot:matView():column(1):setAllValue(1)
			mot.motionDOFcontainer.mot:matView():column(3):setAllValue(1)
		elseif string.upper(string.sub(motion, -3))=='DOF' then
			mot.motionDOFcontainer=MotionDOFcontainer(mot.loader.dofInfo, motion)
		else
			local tempMot=loadMotion(motion, nil, nil)

			local dof2=tempMot.motionDOFcontainer.mot.dofInfo:numDOF()
			local dof1=mot.loader.dofInfo:numDOF()

			if dof1~=dof2 then			
				print('numdof: '..tostring(dof1).. " ~="..tostring(dof2))
				print('retrying without sliding joints')
				
				MotionUtil.exportBVHwithoutSlidingJoints(tempMot.loader.mMotion, '_temp.bvh',0, tempMot.loader.mMotion:numFrames())

				tempMot=loadMotion('_temp.bvh', nil, nil)
				dof2=tempMot.motionDOFcontainer.mot.dofInfo:numDOF()
			end
			if dof1==dof2 then
				mot._loader=tempMot.loader -- ASF or BVH skeleton compatible to VRMLloader
				mot.motionDOFcontainer=tempMot.motionDOFcontainer
				mot.motionDOFcontainer.mot:setDOFinfo(mot.loader.dofInfo)
			else
				print("Error! numDOF still doesn't match")
				loadMotionFromAnotherMotion(mot, tempMot)
			end
		end
	else

		local loader
		if select(1,string.find(skel,'%*')) then
			local files=os.glob(skel)
			if #files >0 then
				table.sort(files)
				local loaders={}
				for i,skel in ipairs(files) do
					loaders[i]=RE.createMotionLoaderExt(skel)
				end
				loader=loaders[1]
				for i=2, #loaders do
					loader.mMotion:concat(loaders[i].mMotion)
				end
			else
				assert(false)
			end
		else

			if string.upper(string.sub(skel,-3))=='FBX' or
				string.upper(string.sub(skel,-8))=='.FBX.DAT' then
				FBXloader=require("FBXloader")
				fbx=FBXloader(skel)
				loader=fbx.loader
				mot.fbx=fbx
			elseif string.upper(string.sub(skel,-3))=='NPZ' then
				assert(type(motion)=='table')
				assert(motion.fbxLoader)
				assert(motion.motion)
				mot.fbx=motion.fbxLoader
				loader=mot.fbx.loader
				motion=motion.motion
			else
				loader=RE.createMotionLoaderExt(skel)
			end
		end
		mot.loader=loader
		if motion and motion~='' then
			mot.motionDOFcontainer=MotionDOFcontainer(mot.loader.dofInfo, motion)
		else
			if mot.loader.mMotion and mot.loader.mMotion:numFrames()>0 then
				mot.motionDOFcontainer=MotionDOFcontainer(mot.loader.dofInfo)
				mot.motionDOFcontainer.mot:set(mot.loader.mMotion)
				mot.motionDOFcontainer:resize(mot.loader.mMotion:numFrames())

				mot.motion=mot.loader.mMotion:copy()
			else
				mot.motion=Motion(mot.loader)
				mot.motion:resize(10)
				local pose=mot.loader:pose()
				for i=0,9 do
					mot.motion:pose(i):assign(pose)
				end

				mot.motionDOFcontainer=MotionDOFcontainer(mot.loader.dofInfo)
				mot.motionDOFcontainer.mot:set(mot.motion)
				mot.motionDOFcontainer:resize(mot.motion:numFrames())
			end
		end
	end
	if skinScale then
		mot.skin=createSkin(skel, mot, skinScale)
		if mot.motion then
			mot.skin:applyAnim(mot.motion)
		else
			mot.skin:applyMotionDOF(mot.motionDOFcontainer.mot)
		end
		mot.skin:setMaterial('lightgrey_transparent')
	end
	return mot
end
function _loadMotionFromAnotherMotion(mot, motion)
	local rotJointToTreeIndex=mot.rotJointToTreeIndex
	mot.motionDOFcontainer=MotionDOFcontainer(mot.loader.dofInfo) 
	mot.motionDOFcontainer:resize(motion:numFrames())
	for i=0, motion:numFrames()-1 do
		mot.loader:updateInitialBone()
		local pose=motion:pose(i)
		local fkSolver=mot.loader:fkSolver()
		for j=0, pose:numRotJoint()-1 do
			local ti=rotJointToTreeIndex[j]
			if ti~=-1 then
				fkSolver:localFrame(ti).rotation:assign(pose.rotations(j))
			end
		end
		fkSolver:localFrame(1).translation:assign(pose.translations(0))
		fkSolver:forwardKinematics()
		mot.loader:getPoseDOF(mot.motionDOFcontainer.mot:row(i))
	end
end
function loadMotionFromAnotherMotion(mot, tempMot)
	mot._loader=tempMot.loader
	local motion=Motion(tempMot.motionDOFcontainer.mot)
	local numRotJoint=mot._loader:numRotJoint()
	local numTransJoint=mot._loader:numTransJoint()
	assert(numTransJoint==1)
	local rotJointToTreeIndex
	if not mot.rotJointToTreeIndex then
		rotJointToTreeIndex={} -- Note that _loader and loader is different.
		for j=0,numRotJoint-1 do
			local ti=mot._loader:getTreeIndexByRotJointIndex(j)
			local name=mot._loader:bone(ti):name()
			rotJointToTreeIndex[j]=mot.loader:getTreeIndexByName(name)
		end
		mot.rotJointToTreeIndex=rotJointToTreeIndex
	else
		rotJointToTreeIndex=mot.rotJointToTreeIndex
	end
	_loadMotionFromAnotherMotion(mot, motion)
end
function annotateEndEffectors(skel, config)
	local bones={}
	bones.right_hip=config.rightLeg[1]
	bones.right_knee=config.rightLeg[2]
	bones.right_heel=config.rightLeg[3]
	bones.left_hip=config.leftLeg[1]
	bones.left_knee=config.leftLeg[2]
	bones.left_heel=config.leftLeg[3]
	bones.right_shoulder=config.rightArm[1]
	bones.right_elbow=config.rightArm[2]
	bones.right_wrist=config.rightArm[3]
	bones.left_shoulder=config.leftArm[1]
	bones.left_elbow=config.leftArm[2]
	bones.left_wrist=config.leftArm[3]

end

function createSkin(skel, loader, skinScale)
	if type(loader)=='table' then
		loader=loader.fbx or loader.loader
	end

	local skin
	if skel and string.upper(string.sub(skel,-3))=='WRL' then
		skin= RE.createVRMLskin(loader, false);	-- to create character
	elseif skel and string.upper(string.sub(skel,-7))=='FBX.DAT' then
		skin= RE.createFBXskin(loader, false);	-- to create character
	elseif skel and string.upper(string.sub(skel,-3))=='FBX' then
		skin= RE.createFBXskin(loader, false);	-- to create character
	elseif skel and string.upper(string.sub(skel,-3))=='NPZ' then
		skin= RE.createFBXskin(loader, false);	-- to create character
	else
		skin= RE.createSkin(loader);	-- to create character
	end
	local s=skinScale 
	skin:setScale(s,s,s);					-- motion data is in meter unit while visualization uses cm unit.
	skin:setMaterial('lightgrey_transparent')
	return skin
end

function createIKsolverForRetargetting(loader, marker_bone_indices,  markerDistance, markerDistanceOverride)
	local mIKsolver=
	{
		effectors=MotionUtil.Effectors(),
		effectorPos=vector3N(),
		src_bones={},
	}
	-- excluding the root
	local effectors=mIKsolver.effectors
	local effectorPos=mIKsolver.effectorPos
	local src_bones=mIKsolver.src_bones
	local markerBoneToBone={}
	local boneToMarkerBone={}
	for ii=0, marker_bone_indices:size()-1 do
		local i=marker_bone_indices(ii)
		markerBoneToBone[ii]=i
		boneToMarkerBone[i]=ii
	end
	local function hasMarkerBoneChild(bone)
		if bone==nil then
			return false
		end
		if boneToMarkerBone[bone:treeIndex()] then
			return true
		end
		return hasMarkerBoneChild(bone:sibling()) or hasMarkerBoneChild(bone:childHead())
	end
	local c=0
	do
		-- count markers 
		for ii=0, marker_bone_indices:size()-1 do
			local i=marker_bone_indices(ii)
			local srcbone=loader:bone(i)
			c=c+3
			--if srcbone:childHead()==nil then
			if not hasMarkerBoneChild(srcbone:childHead()) then
				c=c+3
			end
		end
	end
	effectors:resize(c)
	effectorPos:resize(c)
	c=0

	for ii=0, marker_bone_indices:size()-1 do
		local i=marker_bone_indices(ii)
		local bone=loader:bone(i)
		local md=markerDistance
		if markerDistanceOverride and markerDistanceOverride[bone:name()] then
			md=markerDistanceOverride[bone:name()]
		end
		-- add joint marker
		effectors:at(c):init(bone, vector3(md,0,0))
		src_bones[c]=bone
		c=c+1
		effectors:at(c):init(bone, vector3(0,md,0))
		src_bones[c]=bone
		c=c+1
		effectors:at(c):init(bone, vector3(0,0,md))
		src_bones[c]=bone
		c=c+1

		if not hasMarkerBoneChild(bone:childHead()) then
			print(bone:name() ..' is a SITE bone for IK')
			-- add SITE markers
			local offset=vector3(0,0,0)
			if bone.localCOM then
				offset=bone:localCOM()
			end
			print("SITE", bone, srcbone, offset)
			local mlen=markerDistance*2.5 -- ankle orientation is very important
			effectors:at(c):init(bone, vector3(mlen,0,0)+offset*2)
			src_bones[c]=bone
			c=c+1
			effectors:at(c):init(bone, vector3(0,mlen,0)+offset*2)
			src_bones[c]=bone
			c=c+1
			effectors:at(c):init(bone, vector3(0,0,mlen)+offset*2)
			src_bones[c]=bone
			c=c+1
		end
	end
	mIKsolver.loader=loader
	return mIKsolver
end

function fixJoints(jointNames, loader, motdof)
	local fixedJoints=jointNames
	local mot=Motion(motdof)
	for j, v in ipairs(fixedJoints) do
		local bone=loader:VRMLbone(loader:getTreeIndexByName(v))
		local ri=bone:rotJointIndex()

		for i=0, mot:numFrames()-1 do
			local rr=mot:pose(i).rotations
			for j=ri, rr:size()-2 do
				rr(j):set(rr(j+1))
			end
			rr:resize(rr:size()-1)
			bone:setJointAxes("")
		end
		loader:_initDOFinfo()
	end
	motdof:set(mot)
end

SkeletonEditorModule=LUAclass()

function SkeletonEditorModule.changeLength(loader, b, len_scale)
	local ri=loader:getRotJointIndexByTreeIndex(b:treeIndex())
	if ri==-1 then return 1 end
	local bone=loader:VRMLbone(b:treeIndex())
	local parentBone=MainLib.VRMLloader.upcast(b:parent())
	local qoffset=b:getOffsetTransform():copy()
	local to=qoffset.translation	
	local to_len=to:length()
	
	if to_len<0.0001 then return 1 end

	-- project to bone axes
	local dir=qoffset.translation:copy()
	dir:normalize()
	local delta=dir*(to_len*len_scale)

	bone:setJointPosition(delta)
	loader:updateInitialBone()
	local tc=bone:getOffsetTransform().translation
	if to:length()>0.01 then

		local function scaleMesh(bone, to, tc)
			local q=quater()
			local q2=quater()
			q:axisToAxis(to, vector3(0,0,1))
			q2:axisToAxis(vector3(0,0,1), tc)
			local m=matrix4()
			m:identity()
			m:leftMultRotation(q)
			m:leftMultScaling(1,1, tc:length()/to:length())
			m:leftMultRotation(q2)
			local parentBone=MainLib.VRMLloader.upcast(bone:parent())
			parentBone:transformMeshLocal(m)
		end
		scaleMesh(bone,  to, tc)
	end
	loader:_updateMeshEntity()
end
function SkeletonEditorModule.exportBVHusingASFskeleton(asfFile, amcFile, loader, motdof, bvhFileName)
	local out={}
	out.loader=RE.createMotionLoaderExt(asfFile)
	out.loader:loadAnimation(out.loader.mMotion, amcFile) 
	local l=out.loader
	MotionUtil.removeSlidingJoints(l)
	local bChanged;
	-- remove redundant bones
	repeat
		bChanged=false
		for i=2, l:numBone()-1 do
			local target=l:bone(i);
			if(target:getRotationalChannels()==nil
				and target:getTranslationalChannels()==nil
				and target:numChildren()~=0) then
				print("Removing redundant bone ".. target:name().." (parent of "..target:childHead():name())

				if target:childHead():name()=='SITE' then
					local newname=string.gsub(target:name(), "Dummy", "")
					target:childHead():setName(newname)
					print("Creating dummy end :", newname)
					--l:insertChildBone(target:childHead(), 'SITE')
					--l:insertJoint(target:childHead(), "R")
				end

				l:removeBone(target);
				bChanged=true;
				break
			end
		end
	until bChanged==false 

	-- 
	for i=2, l:numBone()-1 do
		local target=l:bone(i);
		if target:numChildren()==0 and target:name()~='SITE' then
			l:insertChildBone(target, 'SITE')
		end
	end

	local motion=Motion(l)
	motion:resize(motdof:rows())

	local index2ToIndex={}
	for ii=1, loader:numBone()-1 do
		local b=loader:bone(ii)
		local index=l:getBoneByName(b:name())
		index2ToIndex[ii]=index
	end
	for i=0, motdof:rows() -1 do
		loader:setPoseDOF(motdof:row(i))
		l:updateInitialBone()

		for ii=1, loader:numBone()-1 do
			local ibone=index2ToIndex[ii]
			if ibone~=-1 then
				l:fkSolver():localFrame(ibone):assign(loader:fkSolver():localFrame(ii))
			end
		end
		l:fkSolver():forwardKinematics()
		l:getPose(motion:pose(i))
	end
	MotionUtil.exportBVHwithoutSlidingJoints(motion, bvhFileName, 0, motion:numFrames())
end

function changeBoneLength(loader, config, v)
	for k,v in pairs(v) do
		local bottom=config[k][#config[k]]
		local top=config[k][1]
		local b=loader:getBoneByName(bottom)
		local bt=loader:getBoneByName(top)
		repeat
			SkeletonEditorModule.changeLength(loader, b, v)
			b=b:parent()
		until b:parent()==bt 
	end
end

function MotionLoader:rotateBoneGlobal(bone, q_delta)
	-- current global
	-- q0*q1*lq= qo
	--> parent_qo==q0*q1==qo*lq:inverse()
	local qo=bone:getFrame().rotation
	local parent_qo=qo*bone:getLocalFrame().rotation:inverse()

	--qo_new= q_delta*qo = parent_qo*lq_new
	--lq_new= parent_qo:inverse()*q_delta*qo
	local x=parent_qo:inverse()*q_delta*qo
	x:normalize()
	bone:getLocalFrame().rotation:assign(x)
	self:fkSolver():forwardKinematics()
end
MainLib.VRMLloader.rotateBoneGlobal=MotionLoader.rotateBoneGlobal

function MotionLoader:setAxesForIK()
	local loaderClean=self
	loaderClean:bone(1):setChannels('XYZ', 'YZX') -- trans, rot
	for i=2, loaderClean:numBone()-1 do
		loaderClean:bone(i):setChannels('', 'YZX') -- trans, rot
	end
	loaderClean:getBoneByVoca(MotionLoader.LEFTSHOULDER):setChannels('', 'ZXY') -- trans, rot
	loaderClean:getBoneByVoca(MotionLoader.RIGHTSHOULDER):setChannels('', 'ZXY') -- trans, rot
end
