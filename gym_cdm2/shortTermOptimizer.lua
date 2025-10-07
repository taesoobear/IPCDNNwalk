local LBFGS_opta=require("subRoutines/OptimizerLBFGS")
local ShortTrajOpt=LUAclass()

function ShortTrajOpt:calculateOriginalEulerPoses(mot, options)
	local invDT=self.invDT
	local loader=self.loader
	local loaderToTree=self.loaderToTree

	local ndof=self.loaderToTree:nDOF()
	local nframes=mot:rows()
	local x_original=vectorn(ndof*nframes)
	local com=nil
	local momentum=nil
	if options and options.calculateMomentumTraj then
		com=vector3N(nframes)
		momentum=matrixn(nframes-1,6) -- com, momentum
	end
	self.dof_weights=CT.ones(ndof)
	for i=0, nframes-1 do
		loaderToTree:setPoseDOF(loader.dofInfo,mot:row(i))
		local xf=x_original:range(i*ndof, (i+1)*ndof)
		loaderToTree:getQ(xf)
		-- 아래 정도 범위에 들어오면 align필요 없음. 축 두개 이상이 2pi 범위를 넘나들면 align거의 불가능함(euler singularity). 
		if(xf:slice(4,0):maximum()>5.5) or xf:slice(4,0):minimum()<-5.5 then
			varIndex=xf:slice(4,0):argMax()+4;
			print('Error! frame:', i, 'varIndex:', varIndex)

			for b=1, loader:numBone()-1 do
				if loaderToTree:getVarIndex(b,0)>=varIndex then
					print(loader:bone(b), loaderToTree:getVarIndex(b,0))
					break
				end
			end
			xf:slice(4,0):clamp(-5.5, 5.5)

			-- 이 assert에 걸리면 wrl수정해서 해당 조인트의 euler 조인트를 바꿔볼것.
			-- 예를 들어 ZXY -> XZY
			--assert(false)
		end
		if com then
			com(i):assign(loaderToTree:calcCOM(loader))
		end
	end
	x_original:matView(ndof):column(3):alignEulerAngles()
	if options and options.poseOnly then
		return x_original
	end
	local dx=self:deriv(x_original)*invDT
	-- 사람이 낼 수 없는 속도. 
	if (dx:maximum()>100) then
		local index=dx:argMax()
		local errorRow=math.floor(index/ndof)
		local errorCol=index-errorRow*ndof
		print(errorRow, errorCol, dx:matView(ndof):row(errorRow)(errorCol))
		print('Error! frame:', errorRow, 'varIndex:', errorCol)

		for b=1, loader:numBone()-1 do
			if loaderToTree:getVarIndex(b,0)>=errorCol then
				print(loader:bone(b), loaderToTree:getVarIndex(b,0))
				break
			end
		end
		--assert(false)
	end
	--assert(dx:minimum()>-100)
	local ddx=self:deriv(dx)*invDT

	if momentum then
		for i=0, nframes-2 do
			loaderToTree:setQ(x_original:range(i*ndof, (i+1)*ndof))
			loaderToTree:setDQ(dx:range(i*ndof, (i+1)*ndof))
			local m=loaderToTree:calcMomentumCOM(loader)
			local com=loaderToTree:calcCOM(loader)
			local r=momentum:row(i)
			r:setVec3(0, m:M())
			r:setVec3(3, m:F())
		end
	end
	return { x_original= x_original, dx=dx, ddx=ddx, com=com, momentum=momentum}
end


-- mot: reference motion (from which desired velocities and accelerations will be calculated)
function ShortTrajOpt:__init(loader, mot,_options) 
	local nframes=mot:rows()
	self.loader=loader
	self._effectors_unused=MotionUtil.Effectors()
	self._con_unused=MotionUtil.Constraints()
	-- option: euler_root=true
	self.loaderToTree=MotionUtil.LoaderToTree(loader, self._effectors_unused, self._con_unused, true, false)

	local invDT=30
	self.invDT=invDT
	local traj=self:calculateOriginalEulerPoses(mot)


	-- 아래 세개가 desired euler motion에 해당함. 
	-- x_original대신 solveEndEffector 입력으로 들어온 mot을 desired motion으로 사용할 수 있다. 
	self.x_original=traj.x_original
	self.dx=traj.dx
	self.ddx=traj.ddx

	self.weightDDX=10
	self.weightDX=0.1
	self.weightAngleInput=0  -- 입력으로 들어온 mot을 desired motion으로 사용하려면 이 weight를 set할 것. 
	self.weightAngleOriginal=200 -- x_original을 사용하지 않으려면 이 weight를 0으로 놓을 것.
	self.weightEE=100000
	self.weightEEcon=100000
	self.weightEEvel=1
	self.weightMomentum=100
	self.weightCOM=100000

	-- half-space terms are for collision avoidance
	self.weightHalfSpace=10000
	self.weightRelativeHalfSpace=100000
	self.weightVelHalfSpace=1000
	self.onlineOpt=true

	self.clampThr=25 -- degrees
	if _options then
		for k, v in pairs(_options) do
			if k:sub(1,6)=='weight' then
				self[k]=v
			end
		end
	end

	-- a tree caches the forward-kinematics solutions for each frame
	self.trees={}
	for i=1,nframes do 
		self.trees[i]=MotionUtil.LoaderToTree(loader, self._effectors_unused, self._con_unused, true, false)
	end
end

-- fullTraj={ x_orignal=vectorn(), dx=vectorn(), ddx=vectorn() }
function ShortTrajOpt:initPartial(fullTraj, startFrame, endFrame, con_info)
	local ndof=self.loaderToTree:nDOF()
	self.x_original:assign(fullTraj.x_original:range(startFrame*ndof, endFrame*ndof))
	self.dx:assign(fullTraj.dx:range(startFrame*ndof, (endFrame-1)*ndof))
	self.ddx:assign(fullTraj.ddx:range(startFrame*ndof, (endFrame-2)*ndof))
	if fullTraj.momentum then
		self.com=fullTraj.com:range(startFrame, endFrame):copy()
		self.momentum=fullTraj.momentum:sub(startFrame, endFrame-1,0,0):copy()
	end

	if con_info then
		-- collisions
		local con_info_i={}
		for f=startFrame, endFrame-1 do
			con_info_i[f-startFrame+1]=con_info[f+1]
		end
		return con_info_i
	end
end

function ShortTrajOpt:deriv(src)
	local ndof=self.loaderToTree:nDOF()
	local nf=src:size()/ndof

	local dx=vectorn(ndof*(nf-1))
	-- forward diff
	for i=0, nf-2 do
		dx:range(i*ndof, (i+1)*ndof):sub(
		src:range((i+1)*ndof, (i+2)*ndof),
		src:range(i*ndof, (i+1)*ndof))
	end
	return dx
end

function ShortTrajOpt:addDefaultTerms_euler(euler_mot, h)
	local ndof=self.loaderToTree:nDOF()
	local nf=self.x_original:size()/ndof
	local invDT=self.invDT
	local invDTS=invDT*invDT
	local dof_weights=self.dof_weights
	if self.weightDDX~=0 then
		local w=self.weightDDX
		-- add(3,0,4,1,5,2,-1) : add objective to minimize (3x+4y+5z-1)^2
		for i=1, nf-2 do
			for j=0, ndof-1 do
				-- minimize accelerations : ( 1, -2, 1 ) kernel
				-- (1* y_{i-1} - 2* y_i + 1*y_{i+1})^2
				h:addWeighted(w*dof_weights(j), 1*invDTS, (i-1)*ndof+j, -2*invDTS, i*ndof+j, 1*invDTS, (i+1)*ndof+j, -self.ddx((i-1)*ndof+j) )
			end
		end
	end

	if self.weightDX~=0 then
		local w=self.weightDX
		for i=1, nf-2 do
			for j=0, ndof-1 do
				-- minimize velocities : ( -1, 1 ) kernel
				-- (-1* y_{i-1} + 1* y_i)^2
				h:addWeighted(w*dof_weights(j), -1*invDT, (i-1)*ndof+j, 1*invDT, i*ndof+j, -self.dx((i-1)*ndof+j) )
			end
		end
	end

	local loaderToTree=self.loaderToTree
	local x=vectorn(ndof)
	local sum=vectorn(ndof)
	local sum_orig=vectorn(ndof)
	local clamp_thr=self.clampThr

	if self.weightAngleInput~=0 then
		local w=self.weightAngleInput
		local iframe=0
		do
			sum:setAllValue(0)
			for i=iframe, iframe+nf-1 do
				sum:radd(self.euler_mot:row(i))
			end

			for j=0, ndof-1 do
				local param={}
				for k=0, nf-1 do
					table.insert(param, 1)
					table.insert(param, (iframe+k)*ndof+j)
				end
				table.insert(param, -sum(j))
				h:addWeighted(w*dof_weights(j), unpack(param))

				--h:addWeighted(1, 
				--1, iframe*ndof+j, 
				--1, (iframe+1)*ndof+j, 
				--1, (iframe+2)*ndof+j, 
				--1, (iframe+3)*ndof+j, 
				--1, (iframe+4)*ndof+j, 
				--1, (iframe+5)*ndof+j, 
				--1, (iframe+6)*ndof+j, 
				--1, (iframe+7)*ndof+j, 
				--1, (iframe+8)*ndof+j, 
				--1, (iframe+9)*ndof+j, 
				---sum(j))
			end
		end
	end
	if self.weightAngleOriginal~=0 then
		local w=self.weightAngleOriginal
		local dof_weights=self.dof_weights
		local iframe=0
		do
			sum:setAllValue(0)
			for i=iframe, iframe+nf-1 do
				sum:radd(self.x_original:range(i*ndof, (i+1)*ndof))
			end

			for j=0, ndof-1 do
				local param={}
				for k=0, nf-1 do
					table.insert(param, 1)
					table.insert(param, (iframe+k)*ndof+j)
				end
				table.insert(param, -sum(j))
				h:addWeighted(w*dof_weights(j), unpack(param))

				--h:addWeighted(1, 
				--1, iframe*ndof+j, 
				--1, (iframe+1)*ndof+j, 
				--1, (iframe+2)*ndof+j, 
				--1, (iframe+3)*ndof+j, 
				--1, (iframe+4)*ndof+j, 
				--1, (iframe+5)*ndof+j, 
				--1, (iframe+6)*ndof+j, 
				--1, (iframe+7)*ndof+j, 
				--1, (iframe+8)*ndof+j, 
				--1, (iframe+9)*ndof+j, 
				---sum(j))
			end
		end
	end
end
function ShortTrajOpt.annotateCollisions(mChecker, config, mMotionDOF)
	local g_con={}
	local col=boolN(mMotionDOF:numFrames())
	local isHand=config.isHand
	for iframe=0, mMotionDOF:numFrames()-1 do
		local pose=mMotionDOF:row(iframe)
		local lines=vector3N()
		local info=intvectorn()
		local lpos=vector3N()
		mChecker:setPoseDOF(0,pose)
		local bases, maxDepth=mChecker:checkCollision(float_options)
		for i=0, bases:getNumLinkPairs()-1 do
			local collisionPoints=bases:getCollisionPoints(i)
			if collisionPoints:size()>0 and isHand(bases:getBone1(i):treeIndex()) then
				for icp=0, collisionPoints:size()-1 do
					local cp=collisionPoints(icp)

					if cp.idepth>0.01 then
						lines:pushBack(cp.position+cp.normal*cp.idepth) --on body 1
						lines:pushBack(cp.position) -- on body 2
						info:pushBack(bases:getBone1(i):treeIndex())
						info:pushBack(bases:getBone2(i):treeIndex())
						lpos:pushBack(bases:getBone1(i):getFrame():toLocalPos(lines(lines:size()-2)))
						lpos:pushBack(bases:getBone2(i):getFrame():toLocalPos(cp.position))
						col:set(iframe, true)
					end
				end
			end
		end
		if lines:size()>0 then
			g_con[iframe+1]={lines, info, lpos}
		end
	end
	RE.motionPanel():scrollPanel():addPanel(col,CPixelRGB8(255,255,255))

	return g_con, col
end
-- input 
-- mot : the result of per-frame IK
-- hsRelConInfo from self:annotateCollisions
function ShortTrajOpt:solveEE(mot, hsConInfo, _options)
	local markers=config.spaceTimeMarkers
	local gpos_target=self:calcMarkerPosVel(markers, mot)
	return self:solveEndEffectors(mot, markers, gpos_target, hsConInfo, _options)
end
function ShortTrajOpt:calcMarkerPosVel(mot, markers)
	local ndof=self.loaderToTree:nDOF()
	local nf=mot:rows()
	local gpos_target=matrixn(nf, 3*#markers)
	local loaderToTree=self.loaderToTree
	for i=0, nf-1 do
		loaderToTree:setPoseDOF(self.loader.dofInfo,mot:row(i))
		for m, marker in ipairs(markers) do
			local lpos=marker.lpos
			gpos_target:row(i):setVec3((m-1)*3, loaderToTree:globalFrame(marker.bone:treeIndex())*lpos)
		end
	end
	local gvel_target=gpos_target:copy()
	for i=1, nf-1 do
		gvel_target:row(i):assign((gpos_target:row(i)-gpos_target:row(i-1))*self.invDT)
	end
	gvel_target:row(0):assign(gvel_target:row(1))
	return gpos_target..gvel_target
end
function ShortTrajOpt:calcMarkerPos(pose, markers)
	local ndof=self.loaderToTree:nDOF()
	local gpos_target=vectorn(3*#markers)
	local loaderToTree=self.loaderToTree
	loaderToTree:setPoseDOF(self.loader.dofInfo, pose)
	for m, marker in ipairs(markers) do
		local lpos=marker.lpos
		gpos_target:setVec3((m-1)*3, loaderToTree:globalFrame(marker.bone:treeIndex())*lpos)
	end
	return gpos_target
end

function ShortTrajOpt:solveEndEffectors(mot, markers, gpos_target, hsConInfo, hsRelConInfo, hsVelConInfo, _options )

	local euler_mot=self:calculateOriginalEulerPoses(mot, { poseOnly=true})
	local ndof=self.loaderToTree:nDOF()
	local nf=self.x_original:size()/ndof
	euler_mot=euler_mot:matView(ndof):copy()

	-- align euler ry with x_original
	local qy=euler_mot:column(3)
	qy=CT.vec(self.x_original:matView(ndof)(0,3))..qy
	qy:alignEulerAngles()
	euler_mot:column(3):assign(qy:slice(1,0))
	
	local res=self:solveEndEffectors_euler(euler_mot, markers, gpos_target, hsConInfo, hsRelConInfo, hsVelConInfo, _options)

	local loaderToTree=self.loaderToTree
	local out=MotionDOF(self.loader.dofInfo)
	out:resize(nf)
	for i=0, nf-1 do
		loaderToTree:setQ(res:row(i))
		loaderToTree:getPoseDOF(self.loader.dofInfo,out:row(i))
	end
	return out
end
function ShortTrajOpt:solveEndEffectors_euler(euler_mot, markers, gpos_target, hsConInfo, hsRelConInfo, hsVelConInfo,  _options )

	local ndof=self.loaderToTree:nDOF()
	local nf=self.x_original:size()/ndof
	local h
	if self.onlineOpt then
		h=LBFGS_opta(nf*ndof, 1e-9, ndof*2,0 ) --online 
	else
		h=LBFGS_opta(nf*ndof, 1e-9, ndof*2, ndof*2) --offline
	end

	self:addDefaultTerms_euler(euler_mot, h)

	local invDT=self.invDT
	local invDTS=invDT*invDT

	h.opt=self
	h.markers=markers
	h.hsConInfo=hsConInfo
	h.hsRelConInfo=hsRelConInfo
	h.hsVelConInfo=hsVelConInfo
	h.gpos_target=gpos_target

	local useMomentumCache=true -- slightly inaccurate but much faster.
	if useMomentumCache then
		self.mcache=matrixn()
	else
		self.mcache=nil
	end


	h.customGradientFunction=function (self, pos, grad)
		local o=0
		local loaderToTree=self.opt.loaderToTree
		local loader=self.opt.loader
		local ndof=loaderToTree:nDOF()

		local JT=matrixn(ndof, 3) -- jacobian transpose
		if false then
		else
			local trees=self.opt.trees

			local lastAdded=1
			local weightEEvel =self.opt.weightEEvel
			local invDT=self.opt.invDT
			local weightHalfSpace=self.opt.weightHalfSpace
			local weightRelativeHalfSpace=self.opt.weightRelativeHalfSpace
			local weightVelHalfSpace=self.opt.weightVelHalfSpace
			if self.opt.weightEEvel==0 and weightHalfSpace==0 and weightRelativeHalfSpace==0 then
				lastAdded=nf -- disable adding vel terms
			else
			end
			local hsConInfo=self.hsConInfo 
			local hsRelConInfo=self.hsRelConInfo 
			local hsVelConInfo=self.hsVelConInfo 
			
			local disableEE=boolN(loader:numBone())
			-- constrain average velocity per interval
			do
				local iframe=0
				for i=0, nf-1 do
					local p=pos:range(i*ndof, (i+1)*ndof)
					trees[i-iframe+1]:setQ(p)

					if i>lastAdded then
						local loaderToTree=trees[i-iframe+1]
						if hsConInfo then
							-- frame 2 부터 순차적으로 호출됨. frame 0, 1은 상수. 
							--print(i)
							local frameConInfo=hsConInfo[i+1]
							if frameConInfo then

								if weightHalfSpace~=0 then
									local conpos=frameConInfo[1]
									local ncon=conpos:rows()

									local g=grad:range(i*ndof, (i+1)*ndof)
									for icon=0, ncon-1 do
										local normal=frameConInfo[2](icon)
										local pos2=conpos(icon)
										local treeIndex1=frameConInfo[3](icon)
										local tf=loaderToTree:getLastNode(treeIndex1):globalFrame()
										local localpos=frameConInfo[4](icon)
										local plane=Plane(normal, pos2)
										local d=plane:distance(tf*localpos) 

										if d>0 then
											disableEE:set(treeIndex1, true)
											o=o+d*d*weightHalfSpace
											loaderToTree:updateGrad_S_JT(g, weightHalfSpace*(2.0*d)*normal,treeIndex1, localpos)
										end
									end
								end
							end
						end
						if hsVelConInfo then
							local frameConInfo=hsVelConInfo[i+1]
							if frameConInfo then

								local prevLoaderToTree=trees[i-iframe]
								if weightVelHalfSpace~=0 then
									-- target velocity 도 specify 된 경우. 
									local conspeed=frameConInfo[1]
									local ncon=conspeed:size()

									local g=grad:range(i*ndof, (i+1)*ndof)
									for icon=0, ncon-1 do
										local normal=frameConInfo[2](icon)
										local min_speed=conspeed(icon)
										local treeIndex1=frameConInfo[3](icon)
										local tf=loaderToTree:getLastNode(treeIndex1):globalFrame()
										local lpos=frameConInfo[4](icon)
										local w=weightVelHalfSpace
										local gvel_target=normal*min_speed
										local gvel=(loaderToTree:globalFrame(treeIndex1)*lpos -prevLoaderToTree:globalFrame(treeIndex1)*lpos)*invDT
										local deltaS=gvel_target-gvel 

										local d=deltaS:dotProduct(normal)
										if d>0 then
											o=o+d*d*w
											loaderToTree:updateGrad_S_JT(g, -(2.0*w*invDT*d)*normal, treeIndex1, lpos)
											local prevg=grad:range((i-1)*ndof, i*ndof)
											prevLoaderToTree:updateGrad_S_JT(prevg, (2.0*w*invDT*d)*normal, treeIndex1, lpos)
										--o=o+deltaS:dotProduct(deltaS)*w
										--loaderToTree:updateGrad_S_JT(g, -(2.0*w*invDT)*deltaS, treeIndex1, lpos)
										--local prevg=grad:range((i-1)*ndof, i*ndof)
										--prevLoaderToTree:updateGrad_S_JT(prevg, (2.0*w*invDT)*deltaS, treeIndex1, lpos)
										end
									end
								end
							end
						end
						if hsRelConInfo then
							-- frame 2 부터 순차적으로 호출됨.
							--print(i)
							local frameConInfo=hsRelConInfo[i+1]
							if frameConInfo then
								if weightRelativeHalfSpace~=0 then
									local conpos=frameConInfo[1]
									local ncon=conpos:rows()/2

									local g=grad:range(i*ndof, (i+1)*ndof)
									for icon=0, ncon-1 do
										local pos1=conpos(icon*2)
										local pos2=conpos(icon*2+1)
										local treeIndex1=frameConInfo[2](icon*2)
										local treeIndex2=frameConInfo[2](icon*2+1)
										local tf1=loaderToTree:getLastNode(treeIndex1):globalFrame()
										local tf2=loaderToTree:getLastNode(treeIndex2):globalFrame()
										local localpos1=frameConInfo[3](icon*2)
										local localpos2=frameConInfo[3](icon*2+1)
										local normal=pos1-pos2
										local idepth=normal:length()
										normal:rdiv(idepth)
										local plane=Plane(normal, tf2*localpos2)
										local d=plane:distance(tf1*localpos1) 

										if d>0 then
											o=o+d*d*weightRelativeHalfSpace
											loaderToTree:updateGrad_S_JT(g, weightRelativeHalfSpace*(2.0*d)*normal,treeIndex1, localpos1)
											loaderToTree:updateGrad_S_JT(g, -weightRelativeHalfSpace*(2.0*d)*normal,treeIndex2, localpos2)
										end
									end
								end

							
							end


						end
						assert(i>iframe)
						local prevLoaderToTree=trees[i-iframe]
						if weightEEvel~=0 and #markers>0 and self.gpos_target:cols()==#markers*6 then
							-- target velocity 도 specify 된 경우. 
							local gvel_target=self.gpos_target:sub(0,0, #markers*3,0)
							for m, marker in ipairs(self.markers) do
								local w=weightEEvel*marker.weight
								local lpos=marker.lpos
								local bone=marker.bone
								local gvel_target=(self.gpos_target:row(i):toVector3((m-1)*3) -self.gpos_target:row(i-1):toVector3((m-1)*3))*invDT
								local gvel=(loaderToTree:globalFrame(bone:treeIndex())*lpos -prevLoaderToTree:globalFrame(bone:treeIndex())*lpos)*invDT
								local deltaS=gvel_target-gvel
								o=o+deltaS:dotProduct(deltaS)*w
								local g=grad:range(i*ndof, (i+1)*ndof)
								loaderToTree:updateGrad_S_JT(g, -(2.0*w*invDT)*deltaS, bone:treeIndex(), lpos)
								local prevg=grad:range((i-1)*ndof, i*ndof)
								prevLoaderToTree:updateGrad_S_JT(prevg, (2.0*w*invDT)*deltaS, bone:treeIndex(), lpos)
							end
						end
						if weightEEvel~=0 and #markers>0 then

							if self.gpos_target:cols()==#markers*6 then
								-- target velocity 도 specify 된 경우. 
								local gvel_target=self.gpos_target:sub(0,0, #markers*3,0)
								for m, marker in ipairs(self.markers) do
									local w=weightEEvel*marker.weight
									local lpos=marker.lpos
									local bone=marker.bone
									local gvel_target=(self.gpos_target:row(i):toVector3((m-1)*3) -self.gpos_target:row(i-1):toVector3((m-1)*3))*invDT
									local gvel=(loaderToTree:globalFrame(bone:treeIndex())*lpos -prevLoaderToTree:globalFrame(bone:treeIndex())*lpos)*invDT
									local deltaS=gvel_target-gvel
									o=o+deltaS:dotProduct(deltaS)*w
									local g=grad:range(i*ndof, (i+1)*ndof)
									loaderToTree:updateGrad_S_JT(g, -(2.0*w*invDT)*deltaS, bone:treeIndex(), lpos)
									local prevg=grad:range((i-1)*ndof, i*ndof)
									prevLoaderToTree:updateGrad_S_JT(g, (2.0*w*invDT)*deltaS, bone:treeIndex(), lpos)
								end
							elseif self.opt.con then -- 0 or 1
								local con=self.opt.con
								assert(#con==#markers)
								for m, marker in ipairs(self.markers) do
									local w=weightEEvel*marker.weight
									local lpos=marker.lpos
									local bone=marker.bone
									if con[m](i-1) then
										--local gvel_target=vector3(0,0,0)
										local gvel=(loaderToTree:globalFrame(bone:treeIndex())*lpos -prevLoaderToTree:globalFrame(bone:treeIndex())*lpos)*invDT
										local deltaS=-gvel
										o=o+deltaS:dotProduct(deltaS)*w
										local g=grad:range(i*ndof, (i+1)*ndof)
										loaderToTree:updateGrad_S_JT(g, -(2.0*w*invDT)*deltaS, bone:treeIndex(), lpos)
										local prevg=grad:range((i-1)*ndof, i*ndof)
										prevLoaderToTree:updateGrad_S_JT(g, (2.0*w*invDT)*deltaS, bone:treeIndex(), lpos)
									end
								end

							end
						end

						--print(i)
						lastAdded=i
					end
				end
				local momentum=self.opt.momentum
				if momentum and self.opt.weightMomentum>0 then

					if false then
						-- box filtered 
						local weightMomentum=self.opt.weightMomentum
						local r=vectorn(6)
						local deltaS=CT.zeros(6)
						local grad_frame=vectorn(ndof)
						for i=0, nf-2 do
							local p=pos:range((i+1)*ndof, (i+2)*ndof)
							local prev_p=pos:range(i*ndof, (i+1)*ndof)
							local tree=trees[i-iframe+1]
							tree:setDQ((p-prev_p)*invDT)

							local desiredM=momentum:row(i)
							local actualM=tree:calcMomentumCOM(loader)
							r:setVec3(0, actualM:M())
							r:setVec3(3, actualM:F())
							deltaS:radd((desiredM-r))
						end
						o=o+deltaS:dotProduct(deltaS)*weightMomentum


						-- deltaS=sum(desiredM-actualM(dq))
						-- d_deltaS^2/d_dq = -2*deltaS*sum(J_momentum) -> defined as grad_frame
						-- but dq=(q_(i+1)-	q_i)*invDT
						-- so
						-- d_deltaS^2/d_q = d_deltaS^2/d_dq * d_dq/d_q
						--                = grad_frame * d_dq/d_q
						grad_frame:zero()
						local temp=vectorn(ndof)
						for i=0, nf-2 do
							local tree=trees[i-iframe+1]
							tree:calcMomentumJacobianTranspose(loader, JT)
							temp:row():multABt((-2.0*weightMomentum*deltaS):row(), JT)
							grad_frame:radd(temp)
						end

						grad_frame:rmult(invDT)
						for i=0, nf-2 do
							grad:range((i+1)*ndof,(i+2)*ndof):radd(grad_frame)
							grad:range(i*ndof, (i+1)*ndof):rsub(grad_frame)
						end
					else
						local weightMomentum=self.opt.weightMomentum
						local r=vectorn(6)
						local deltaS=vectorn(6)
						local grad_frame=vectorn(ndof)
						local filter=vectorn()
						--filter:linspace(1,0, nf-1)
						math.getGaussFilter(nf, filter)

						--local T=util.Timer()
						--T:start()

						local mcache=self.opt.mcache
						if mcache then
							if mcache:rows()==0 then
								--T:start()
								mcache:setSize(ndof, 6) -- average JT
								mcache:setAllValue(0.0)
								for i=0, nf-2 do
									local tree=trees[i-iframe+1]
									local mJT=matrixn()
									tree:calcMomentumJacobianTranspose(loader, mJT)
									mcache:radd(mJT)
								end
								mcache:rmult(1.0/(nf-1))
								--print('p3', T:stop2()/1e3)
							end
						end



						for i=0, nf-2 do
							local p=pos:range((i+1)*ndof, (i+2)*ndof)
							local prev_p=pos:range(i*ndof, (i+1)*ndof)
							local tree=trees[i-iframe+1]
							tree:setDQ((p-prev_p)*invDT)

							local desiredM=momentum:row(i)
							local actualM=tree:calcMomentumCOM(loader)
							r:setVec3(0, actualM:M())
							r:setVec3(3, actualM:F())
							deltaS:sub(desiredM,r)
							o=o+deltaS:dotProduct(deltaS)*weightMomentum*filter(i)
							-- deltaS=(desiredM-actualM(dq))
							-- d_deltaS^2/d_dq = -2*deltaS*(J_momentum) -> defined as grad_frame
							-- but dq=(q_(i+1)-	q_i)*invDT
							-- so
							-- d_deltaS^2/d_q = d_deltaS^2/d_dq * d_dq/d_q
							--                = grad_frame * d_dq/d_q
							if mcache then
								local mJT=mcache
								--local mJT=mcache:sub(0, ndof, i*6,(i+1)*6)
								grad_frame:row():multABt(((-2.0*weightMomentum*filter(i)*invDT)*deltaS):row(), mJT)
							else
								tree:calcMomentumJacobianTranspose(loader, JT)
								grad_frame:row():multABt(((-2.0*weightMomentum*filter(i)*invDT)*deltaS):row(), JT)
							end
							grad:range((i+1)*ndof,(i+2)*ndof):radd(grad_frame)
							grad:range(i*ndof, (i+1)*ndof):rsub(grad_frame)
						end
					end
				end

				local weightEE=self.opt.weightEE
				local filterKernel=vectorn()
				math.getGaussFilter(nf, filterKernel)
				if weightEE>0 then

					-- constrain Gaussian-filtered position per interval 
					for m, marker in ipairs(self.markers) do
						local lpos=marker.lpos
						local bone=marker.bone
						local w=weightEE*marker.weight

						if not disableEE(bone:treeIndex()) then

							local deltaS=vector3(0,0,0)
							for i=0, nf-1 do
								local loaderToTree=trees[i-iframe+1]
								local gpos=loaderToTree:globalFrame(bone:treeIndex())*lpos
								local gpos_target=self.gpos_target:row(i):toVector3((m-1)*3)
								deltaS:radd((gpos_target-gpos)*filterKernel(i))
							end
							o=o+deltaS:dotProduct(deltaS)*w

							for i=0, nf-1 do
								local loaderToTree=trees[i-iframe+1]
								local g=grad:range(i*ndof, (i+1)*ndof)
								loaderToTree:updateGrad_S_JT(g, -(2.0*w*filterKernel(i))*deltaS, bone:treeIndex(), lpos)
							end
						end
					end
				end

				if self.opt.con and self.opt.weightEEcon>0 then -- 0 or 1
					local weightEEcon=self.opt.weightEEcon
					local con=self.opt.con

					-- constrain Gaussian-filtered position per interval 
					for m, marker in ipairs(self.markers) do
						local lpos=marker.lpos
						local bone=marker.bone
						local w=weightEEcon*marker.weight
						local conm=con[m]

						if not disableEE(bone:treeIndex()) then

							local deltaS=vector3(0,0,0)
							local c=0
							for i=0, nf-1 do

								if conm(i) then
									c=c+1
									local loaderToTree=trees[i-iframe+1]
									local gpos=loaderToTree:globalFrame(bone:treeIndex())*lpos
									local gpos_target=self.gpos_target:row(i):toVector3((m-1)*3)
									deltaS:radd(gpos_target-gpos)
								end
							end
							if c>0 then
								deltaS:rmult(1/c)
								o=o+deltaS:dotProduct(deltaS)*w

								for i=0, nf-1 do
									if conm(i) then
										local loaderToTree=trees[i-iframe+1]
										local g=grad:range(i*ndof, (i+1)*ndof)
										loaderToTree:updateGrad_S_JT(g, -(2.0*w/c)*deltaS, bone:treeIndex(), lpos)
									end
								end
							end
						end
					end
				end

				local com=self.opt.com
				if com and self.opt.weightCOM>0 then
					local weightCOM=self.opt.weightCOM
					local weightedDS=vectorn(3)
					local grad_frame=vectorn(ndof)
					if weightCOM~=0 then
						local deltaS=vector3(0,0,0)
						local filterSize=math.max(math.floor(nf/2),2)
						for i=0, filterSize-1 do
							local desiredCOM=com(i)
							local actualCOM=trees[i+1]:calcCOM(loader)
							deltaS:radd(desiredCOM-actualCOM)
						end
						deltaS:rmult(1.0/filterSize)
						o=o+deltaS:dotProduct(deltaS)*weightCOM
						for i=0, filterSize-1 do
							trees[i+1]:calcCOMjacobianTranspose(loader, JT)
							weightedDS:setVec3(0, (-2.0*weightCOM/filterSize)*deltaS)
							grad_frame:row():multABt(weightedDS:row(), JT)
							grad:range(i*ndof, (i+1)*ndof):radd(grad_frame)
						end
					end
				end



			end
		end

		return o
	end


	--print('started solving')
	local timer=util.Timer()
	local initial=self.x_original:copy()

	do 
		local frames
		if self.onlineOpt then
			frames={0,1,} 
		else
			frames={0,1,nf-2, nf-1} -- offline
		end
		local loaderToTree=self.loaderToTree
		local clamp_thr=self.clampThr
		for i, frame in ipairs(frames) do
			local x=initial:range(frame*ndof, (frame+1)*ndof)
			x:assign(euler_mot:row(frame))
			x_orig=self.x_original:range(frame*ndof, (frame+1)*ndof)
			x:clamp(x_orig-math.rad(clamp_thr), x_orig+math.rad(clamp_thr))
		end
		if _options and  _options.useCurrentAsInitial then
			local frames
			if self.onlineOpt then
				frames={0}
			else
				frames={0, nf-1}
			end
			for j, i in ipairs(frames) do
				local xf=initial:range(i*ndof, (i+1)*ndof)
				xf:assign(euler_mot:row(i))
			end
			local i=1
			initial:range(i*ndof, (i+1)*ndof):add( initial:range((i-1)*ndof, i*ndof), self.dx:range((i-1)*ndof, i*ndof)/self.invDT)
			i=nf-1
			initial:range((i-1)*ndof, i*ndof):sub(initial:range(i*ndof, (i+1)*ndof), self.dx:range((i-1)*ndof, i*ndof)/self.invDT)
		end
	end
	--local res= h:solve(self.x_original)
	local res= h:solve(initial)

	RE.output('solved (ms):', timer:stop2()/1e3)
	return res:matView(ndof):copy()
end

function ShortTrajOpt.prepareSpaceTimeMarkers(config, mMarker)
	config.spaceTimeMarkers={}
	for k, v in ipairs( mMarker.config.con) do
		config.spaceTimeMarkers[k]=v
	end
	assert(#config.relativeMarkers==#mMarker.config.relativeMarkers)
	for i, v in ipairs( config.relativeMarkers) do
		assert(v[1]==mMarker.config.relativeMarkers[i][1])
		table.insert(config.spaceTimeMarkers,v)
	end
end
return ShortTrajOpt
