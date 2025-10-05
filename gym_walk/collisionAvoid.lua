

--obstaclePos={vector3(40,-1000,600), vector3(0,-1150,345), vector3(-125,120,990)} --, vector3(0,10,0)}	-- locomot, use_spline, obstacle
--obstacleSize={vector3(20,20,40),vector3(20,20,40), vector3(20,20,540)} --, vector3(1000,0,2000)}
--obstaclePos={vector3(40,100,600), vector3(0,150,350), vector3(-182,120,745)} --, vector3(0,10,0)}
--obstacleSize={vector3(20,20,40),vector3(20,20,40), vector3(20,20,40)} --, vector3(1000,0,2000)}
--obstaclePos={vector3(45,150,280), vector3(-90,60,340), vector3(-202,90,500)} --, vector3(0,10,0)}
--obstacleSize={vector3(40,20,40),vector3(15,20,40), vector3(10,20,20)} --, vector3(1000,0,2000)}
obstaclePos={vector3(-77,135,560), vector3(29,150,320), vector3(22,120,830)} --, vector3(0,10,0)}
obstacleSize={vector3(20,20,40),vector3(20,20,20), vector3(20,20,40)} --, vector3(1000,0,2000)}
obstacleType={"BOX","SPHERE","CAPSULE"}

Displacement=LUAclass()

function Displacement:__init(dofInfo, frameRate)
	assert(dofInfo)
	self.dofInfo=dofInfo
	self.frameRate=frameRate
	self.velThr=1000
end

function vectorn.projectAngles(dmotionDOF_i)
   MainLib.VRMLloader.projectAngles(dmotionDOF_i) -- align angles
end
-- CollisionAvoid:calcVelocity와 동일.
function Displacement:calcVelocity(prevpose, nextpose)
	local frameRate=self.frameRate
	local dmotionDOF_i=vectorn(nextpose:size())
	dmotionDOF_i:sub(nextpose, prevpose)
	dmotionDOF_i:projectAngles()
	dmotionDOF_i:rmult(frameRate) -- 1/T
	assert(self.dofInfo:numSphericalJoint()==1) 
	-- otherwise following code is incorrect
	local T=MotionDOF.rootTransformation(nextpose)
	local V=T:twist( MotionDOF.rootTransformation(prevpose), 1/frameRate)
	local velThr=self.velThr
	local v=math.clampVec3( V.v,velThr)
	local w=math.clampVec3( V.w,velThr)
	dmotionDOF_i:setVec3(0, V.v)
	dmotionDOF_i:setVec3(4, V.w)
	return dmotionDOF_i
end

function Displacement:integrate(pose, dotpose)
	local frameRate=self.frameRate
	local timestep=1/frameRate
	local out=vectorn(pose:size())
	
	if pose:size()>=7 then
		if pose:size()>7 then
			out:range(7, pose:size()):add(pose:range(7, pose:size()), dotpose:range(7, dotpose:size())*timestep)
		end
		local T=MotionDOF.rootTransformation(pose)
		local v=liegroup.se3()
		v.v:assign(dotpose:toVector3(0))
		v.w:assign(dotpose:toVector3(4))
		T:integrate(v, timestep)
		MotionDOF.setRootTransformation(out, T)
	else
		assert(false)
	end
	return out
end

DisplacementFilter=LUAclass()
function DisplacementFilter:__init(dofinfo)
	-- OnlineFilter:
	-- setCurrPose(pose)
	-- newpose =getFiltered()
	--
	--
	-- newpose=syn:synthesizeOneFrame(refPose, dAngle, controlParam, i, state, currFrame, isWalk, exForce)
	--
	-- newpose2=integrate(newpose, filter(prev_v1))
	--
	-- newpose=colAvoid:actuallySolveIK(newpose2)
	--
	-- v1=calcVelocity(pose, newpose)
	-- prev_v1=add(prev_v1+ v1)*0.9
end

CollisionAvoid=LUAclass()

function CollisionAvoid:createIKsolver(ikc, kneeIndex, axis)
--[[
	if true then
		-- nlopt or lbfgs
		self.mIK=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(mLoader.dofInfo,self.mEffectors,self.g_con)
		--self.mIK=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_nlopt(mLoader.dofInfo,self.mEffectors,self.g_con)
		--mIK=MotionUtil.createFullbodyIk_MotionDOF_hybrid_lbfgs(mLoader.dofInfo,mEffectors,g_con, CT.ivec(lknee:treeIndex(), rknee:treeIndex()), CT.vec(1,1))
		self.mEffectors:resize(2);
		self.mEffectors(0):init(mLoader:getBoneByName(ikc[1][2]), ikc[1][3])
		self.mEffectors(1):init(mLoader:getBoneByName(ikc[2][2]), ikc[2][3])

		local mEffectors=self.mEffectors
		self.mIK:_changeNumEffectors(2)
		self.mIK:_setEffector(0, mEffectors(0).bone, mEffectors(0).localpos)
		self.mIK:_setEffector(1, mEffectors(1).bone, mEffectors(1).localpos)
	else
		-- semi-hybrid lbfgs
		self.mEffectors:resize(2);

		self.mEffectors(0):init(mLoader:getBoneByName(ikc[1][2]), ikc[1][3])
		self.mEffectors(1):init(mLoader:getBoneByName(ikc[2][2]), ikc[2][3])

		--self.mIK=MotionUtil.createFullbodyIk_MotionDOF_semihybrid_lbfgs(mLoader.dofInfo,self.mEffectors,self.g_con,kneeIndex, axis )
		self.mIK=MotionUtil.createFullbodyIk_MotionDOF_semihybrid_nlopt(mLoader.dofInfo,self.mEffectors,self.g_con,kneeIndex, axis )
	end
]]--
	self.mEffectors:resize(2);
	self.mEffectors(0):init(self.loader:getBoneByName(ikc[1][2]), ikc[1][3])
	self.mEffectors(1):init(self.loader:getBoneByName(ikc[2][2]), ikc[2][3])

	self.mIK=IKChain(self.loader, ikc, kneeIndex, axis, self.mEffectors, self.g_con, {true, false,false})
end

function CollisionAvoid:setIter(n)
	self.mIK.iter=n
end
function CollisionAvoid:__init(loader, config)
	self.config=config;
	self.loader=loader
	self.g_con=MotionUtil.Constraints() -- std::vector<MotionUtil::RelativeConstraint>
	self.g_con:resize(0)
	self.mEffectors=MotionUtil.Effectors()

	local ikc=self.config[3]
	self.lknee=loader:getBoneByName(ikc[1][1])
	self.rknee=loader:getBoneByName(ikc[2][1])
	local kneeIndex=intvectorn(2)
	local axis=vectorn(2)
	axis:setAllValue(1)

	kneeIndex:set(0, self.lknee:treeIndex())
	kneeIndex:set(1, self.rknee:treeIndex())
	if ikc[1].reversed then axis:set(0,-1) end
	if ikc[2].reversed then axis:set(1,-1) end
	if ikc[3] then
		kneeIndex:resize(4)
		axis:resize(4)
		kneeIndex:set(2, loader:getTreeIndexByName(ikc[3][1]))
		kneeIndex:set(3, loader:getTreeIndexByName(ikc[4][1]))
		if ikc[3].reversed then axis:set(2,-1) end
		if ikc[4].reversed then axis:set(3,-1) end
	end

	self:createIKsolver(ikc, kneeIndex, axis)

	self.mIntegrator=VelocityFields(loader, mMotionDOFcontainer, mMotion1:row(startFrame))
	self.mFilter=OnlineFilter(loader, mMotion1:row(startFrame), 3)
	self.mIntegrator.pose:assign(mMotion1:row(startFrame));
	self.mDPose=self.mIntegrator.dmot:row(0):copy()

	self.mCON=Constraints(unpack(obstaclePos))
	self.mCON:connect(solveIK)

	mObstacles={}
	for i=1,#obstaclePos do
		mObstacles[i]=Geometry()

		do 
			local mesh=mObstacles[i]
			if obstacleType[i]=="BOX" then
				mesh:initBox(vector3(20, 20, 40))
			elseif obstacleType[i]=="SPHERE" then
				mesh:initEllipsoid(obstacleSize[i])
			elseif obstacleType[i]=="CAPSULE" then
				mesh:initCapsule(obstacleSize[i].x, obstacleSize[i].z)
			else
				mesh:initPlane(obstacleSize[i].x, obstacleSize[i].z)
			end
			local mat=matrix4()
			mat:identity()
			mat:leftMultScale(1/input.skinScale)
			mesh:transform(mat)
		end
	end

	self.mChecker=CollisionChecker(unpack{loader, unpack(mObstacles)})

	local obj={loader, unpack(mObstacles)}
	for i,v in ipairs(obj) do
		local ltype=getmetatable(v).luna_class
		if ltype=='MainLib.VRMLloader' then
			self.mChecker.collisionDetector:addModel(v)
		else
			self.mChecker.collisionDetector:addObstacle(v)
		end
	end

	for i=1,#obstaclePos do
		local initialState=vectorn(7)
		initialState:setVec3(0, (obstaclePos[i])/input.skinScale)
		initialState:setQuater(3,quater(1,0,0,0))
		self.mChecker:setPoseDOF(i, initialState)
	end

	self.mObstacleSkins={}
	for i=1, #obstaclePos do
		local v=self.mChecker.collisionDetector:getModel(i)
		self.mObstacleSkins[i]= RE.createVRMLskin(v, false);
		local s=input.skinScale
		self.mObstacleSkins[i]:scale(s,s,s)
		self.mChecker:registerPair(0,i) -- 0 means mLoader, 1 means mObstacles[1]
		local state=self.mChecker.pose[i]
		self.mObstacleSkins[i]:setPoseDOF(state)
	end

	self.mOriginalPos={}
	for i=0,1 do
		local mEffectors=self.mEffectors
		local originalPos=mEffectors(i).bone:getFrame():toGlobalPos(mEffectors(i).localpos)
		self.mOriginalPos[i+1]=originalPos
	end
end

function CollisionAvoid:actuallySolveIK(npose)
	
	self.loader:setPoseDOF(npose);
	local mEffectors=self.mEffectors;

	local footPos=vector3N(2)
	for i=0,1 do
		local originalPos=mEffectors(i).bone:getFrame():toGlobalPos(mEffectors(i).localpos)
		footPos(i):assign(originalPos)
		--dbg.namedDraw("Sphere", (footPos(i)+character2offset)*100, 'foot'..i,'red',10);
	end

	self.mChecker:setPoseDOF(0,npose)

	self.mIntegrator.pose:assign(npose);

	if self.mObstacleSkins then
		for i=1, #self.mObstacleSkins do
			self.mObstacleSkins[i]:setMaterial('lightgrey_transparent')
			local initialState=vectorn(7)
			initialState:setVec3(0, (self.mCON.conPos(i-1))/input.skinScale)
			initialState:setQuater(3,quater(1,0,0,0))
			self.mChecker:setPoseDOF(i, initialState)
			self.mObstacleSkins[i]:setPoseDOF(initialState)
		end
	end

	local restartOver=false
	local maxAvoid=0.005
	if restartOver then maxAvoid=nil -- collisionAvoid at once (for debugging)
	end
	local bases=self.mChecker:checkCollision()
	local collisionLinkPairs=bases:getCollisionLinkPairs()
	local maxDepth, lines=self:prepareEffectors(bases, collisionLinkPairs,0,maxAvoid )
	local npose1=npose:copy();
	self:setIter(1)
	self.mIK:IKsolve(npose1, footPos);

	npose2=npose1:copy();

	local numIter
	if restartOver then
		numIter=1 -- restart over
	else
		numIter=maxDepth/maxAvoid
	end
	if numIter>=2 then
		numIter=math.floor(numIter)

		for i=1, numIter-1 do
			self.mChecker:setPoseDOF(0,npose2)
			local bases=self.mChecker:checkCollision()
			local collisionLinkPairs=bases:getCollisionLinkPairs()
			local maxDepth=self:prepareEffectors(bases, collisionLinkPairs,0,maxAvoid )
			self:setIter(i+1)
			self.mIK:IKsolve(npose2, footPos);
		end
	end
	local v=self:calcVelocity(npose, npose2, 120)
	--mSkin:setPoseDOF(npose2);
	v:clamp(10)
	self.mIntegrator:singleStep(v)

	self.mDPose:setAllValue(0)
	self.mIntegrator:stepKinematic(self.mDPose, mMotion1:row(0), 0.05)
	self.mFilter:setCurrPose(npose2)
	--mSkin:setPoseDOF(self.mFilter:getFiltered());
	--mSkin:setPoseDOF(newpose);
	--return lines
	return self.mFilter:getFiltered(), lines, maxDepth
	--return npose2, lines
end


function projectAngles(dmotionDOF_i)
   MainLib.VRMLloader.projectAngles(dmotionDOF_i) -- align angles
end

function CollisionAvoid:calcVelocity(p1, p2, frame_rate)
   local q=quater()
   local v=vector3()
   local dmotionDOF_i=p1:copy()
   dmotionDOF_i:sub(p2, p1)
   local pcall_ok, errMsg=pcall(projectAngles, dmotionDOF_i)
   if not pcall_ok then
	   print(errMsg)
	   dmotionDOF_i:setAllValue(0);
	   dbg.console()
   end
   dmotionDOF_i:rmult(frame_rate)

   local T=MotionDOF.rootTransformation(p1)
   local twist=T:twist(MotionDOF.rootTransformation(p2),1/frame_rate)
   -- incorrect, but previous results were optimized using this.
   local v=math.clampVec3( twist.v,1000)
   local w=math.clampVec3( twist.w,1000)
   dmotionDOF_i:setVec3(0, v)
   dmotionDOF_i:setVec3(4, w)
   return dmotionDOF_i
end

-- module.lua
function vectorn:clamp(mag1, mag2,w)
	if mag2==nil then
		mag2=mag1
		mag1=mag2*-1
	end
	assert(mag2>0)
	local smc=math.clamp
	for i=0,self:size()-1 do
		self:set(i, smc(self(i), mag1, mag2))
	end
 end

-- module.lua
function math.clamp(i,a,b)
   return math.min(math.max(i,a),b)
end

Queue=LUAclass()
function Queue:__init(n)
	self.n=n
	self.data={}
	self.front=1
end
function Queue:pushBack(data)
	if #self.data==self.n then
		self.data[self.front]=data
		self.front=self.front+1
		if self.front>self.n then
			self.front=1
		end
	else
		table.insert(self.data, data)
	end
end
function Queue:back()
	local f=self.front
	if f==1 then
		return self.data[#self.data]
	else
		return self.data[f-1]
	end
end
function Queue:front()
	return self.data[self.front]
end
function Queue:get(i)-- i==1 : front, i>= #self.data : back
	assert(i<=self.n)
	local f=self.front
	if f==1 then
		return self.data[math.min(i+f-1, #self.data)]
	else
		assert(#self.data==self.n)
		if f+i-1>self.n then
			return self.data[i-self.n+f-1]
		else
			return self.data[f+i-1]
		end
	end
end


function CollisionAvoid:prepareEffectors(bases, collisionLinkPairs, loaderIndex, maxAvoid)
	local maxDepth, lines=self:_prepareEffectors(0, bases, collisionLinkPairs, loaderIndex, maxAvoid)
	local mIK=self.mIK
	mIK:_effectorUpdated()
	return maxDepth, lines
end


function CollisionAvoid:_prepareEffectors(start_con, bases, collisionLinkPairs, loaderIndex, maxAvoid)
	local mIK=self.mIK
	local mEffectors=self.mEffectors
	loaderIndex=loaderIndex or 0

	local method=3 -- 1: use end-effectors. 2: use plane-distance constraints, 3: use half-space constraints without smoothing, 4: with smoothing

	if method==4 then
		if not self.queue then -- indexed by ibone
			self.queue={}
		end

		local numHistory=10
		for i=1, self.loader:numBone()-1 do
			if not self.queue [i] then
				self.queue[i]=Queue(numHistory)
			end
			self.queue[i]:pushBack({linkpairs={}}) -- means  no-collision
		end
	end

	local count=0
	if method~=4 then
		if collisionLinkPairs:size()>=1 then
			for i=0, collisionLinkPairs:size()-1 do
				local ilinkpair=collisionLinkPairs(i)
				local collisionPoints=bases:getCollisionPoints(ilinkpair)

				local iloader1=bases:getCharacterIndex1(ilinkpair)
				local iloader2=bases:getCharacterIndex2(ilinkpair)
				if iloader1==loaderIndex or iloader2==loaderIndex then
					count=count+collisionPoints:size()
				end
			end
		end
	end

	if method==1 then
		mIK:_changeNumEffectors(2+count)
		mIK:_setEffector(0, mEffectors(0).bone, mEffectors(0).localpos)
		mIK:_setEffector(1, mEffectors(1).bone, mEffectors(1).localpos)
		footPos:resize(2+count)
	else
		mIK:_changeNumEffectors(2)
		mIK:_setEffector(0, mEffectors(0).bone, mEffectors(0).localpos)
		mIK:_setEffector(1, mEffectors(1).bone, mEffectors(1).localpos)

		if method~=4 then
			mIK:_changeNumConstraints(start_con+count) 
		end
	end

	count=0
	local lines=vector3N()

	local maxDepth=0
	if collisionLinkPairs:size()>=1 then
		for i=0, collisionLinkPairs:size()-1 do
			local ilinkpair=collisionLinkPairs(i)
			local iloader1=bases:getCharacterIndex1(ilinkpair)
			local bone1=bases:getBone1(ilinkpair)
			local iloader2=bases:getCharacterIndex2(ilinkpair)
			local collisionPoints=bases:getCollisionPoints(ilinkpair)

			if iloader1==loaderIndex or iloader2==loaderIndex then
				for j=0, collisionPoints:size()-1 do
					local b=collisionPoints(j)
					--print(loader1:name(), loader1:VRMLbone(linkpair[2]).NameId, loader2:name(), loader2:VRMLbone(linkpair[4]).NameId)
					if self.mObstacleSkins then
						--self.mObstacleSkins[iloader2]:setMaterial('red_transparent')
					end
					if method~=4 then
						lines:pushBack(b.position*self.config.skinScale) -- inside bone1, on the bone2's surface
						--lines:pushBack((b.position+b.normal)*config.skinScale)
						lines:pushBack((b.position+b.normal*b.idepth)*self.config.skinScale) -- inside bone2, on the bone1's surface
						--lines:pushBack((b.position+b.normal*b.idepth)*config.skinScale)
						lines:pushBack((b.position+b.normal*b.idepth*2)*self.config.skinScale)
						lines:pushBack((b.position+b.normal)*self.config.skinScale)
					end
					maxDepth=math.max(maxDepth, b.idepth)

					if method==1 then
						local bone=bone1
						local localpos=bone:getFrame():toLocalPos(b.position+b.normal*b.idepth)
						mIK:_setEffector(2+count, bone, localpos)
						footPos(2+count):assign(b.position)
					elseif method==2 then
						local bone=bone1
						local localpos=bone:getFrame():toLocalPos(b.position+b.normal*b.idepth)
						local plane=Plane(b.normal, b.position)
						mIK:_setPlaneDistanceConstraint(start_con+count, bone, localpos, plane.normal, plane.d)
					elseif method==3 then
						local margin=0.02 -- max penetration depth

						if maxAvoid then
							-- maximum-avoidance during a single-step
							if b.idepth>margin+maxAvoid then -- max penetration vel
								margin=b.idepth-maxAvoid
							end
						end

						if iloader1==loaderIndex then
							local bone=bone1
							local localpos=bone:getFrame():toLocalPos(b.position+b.normal*b.idepth)
							--local localpos=bone:getFrame():toLocalPos(b.position+b.normal*b.idepth*0.5)
							local plane=Plane(b.normal, b.position)
							mIK:_setHalfSpaceConstraint(start_con+count, bone, localpos, plane.normal, plane.d-margin)
						else
							local bone=bases:getBone2(ilinkpair)
							local localpos=bone:getFrame():toLocalPos(b.position)
							--local localpos=bone:getFrame():toLocalPos(b.position+b.normal*b.idepth*0.5)
							local plane=Plane(-b.normal, b.position+b.normal*b.idepth)
							mIK:_setHalfSpaceConstraint(start_con+count, bone, localpos, plane.normal, plane.d-margin)
						end
					else
						-- method == 4 
						if iloader1==loaderIndex then
							local bone=bone1
							local back=self.queue[bone:treeIndex()]:back()
							local localpos=bone:getFrame():toLocalPos(b.position+b.normal*b.idepth)
							assert(back[ilinkpair]==nil)
							table.insert(back.linkpairs, ilinkpair)
							back[ilinkpair]={bone:treeIndex(), localpos, b.normal:copy(), b.position}
						else
							local bone=bases:getBone2(ilinkpair)
							local back=self.queue[bone:treeIndex()]:back()
							local localpos=bone:getFrame():toLocalPos(b.position)
							assert(back[ilinkpair]==nil)
							table.insert(back.linkpairs, ilinkpair)
							back[ilinkpair]={bone:treeIndex(), localpos, -b.normal, b.position+b.normal*b.idepth}
						end
					end
					count=count+1
				end
			end
		end
		if method~=4 then
			--dbg.draw('Traj', lines:matView(), 'normals'..loaderIndex,'solidred')
		end
	else
		if method~=4 then
			dbg.erase('Traj', 'normals'..loaderIndex)
		end
	end

	if method==4 then
		local count=0
		local smoothed={}
		for i=1, self.loader:numBone()-1 do

			local linkpairs={}
			local q=self.queue[i].data
			for j=1,#q do
				local qlp=q[j].linkpairs
				for k, v in ipairs(qlp) do
					array.pushBackIfNotExist(linkpairs, v)
				end
			end
			if #linkpairs>0 then
				local out={}
				smoothed[i]=out
				count=count+#linkpairs
				out.linkpairs=linkpairs

				for k, v in ipairs(linkpairs) do
					local out_k={-1, vector3(0,0,0), vector3(0,0,0), vector3(0,0,0)}
					out[k]=out_k
					local c=0
					for j=1, #q do
						local d=q[j][v]
						if d then
							out_k[1]=d[1]
							for jj=2,4 do
								out_k[jj]:radd(d[jj])
							end
							c=c+1
						end
					end
					for jj=2,4 do
						out_k[jj]:scale(1/c)
					end
					out_k[3]:normalize() -- normal
				end
			end
		end


		mIK:_changeNumConstraints(start_con+count) 
		count=0
		for i=1, self.loader:numBone()-1 do
			if smoothed[i] then
				local linkpairs=smoothed[i].linkpairs
				for k, v in ipairs(linkpairs) do
					local out_k=smoothed[i][k]
					local plane=Plane(out_k[3], out_k[4])
					local margin=0.02 -- max penetration depth

					lines:pushBack(out_k[4]*self.config.skinScale) 
					lines:pushBack(out_k[4]+out_k[3]*self.config.skinScale) 
					mIK:_setHalfSpaceConstraint(start_con+count, self.loader:VRMLbone(out_k[1]), out_k[2],plane.normal, plane.d-margin )
					count=count+1
				end
			end
		end
		if count>0 then
			dbg.draw('Traj', lines:matView(), 'normals'..loaderIndex,'solidred')
		else
			dbg.erase('Traj', 'normals'..loaderIndex)
		end
	end

	return maxDepth, lines
end

OnlineFilter=LUAclass()

function OnlineFilter:__init(loader, pose, filterSize)
	self.filterSize=filterSize
	self.loader=loader
	self.queue=Queue(filterSize)
	self:setCurrPose(pose)

	self.kernel=vectorn()
	math.getGaussFilter(filterSize, self.kernel)
end

function OnlineFilter:setCurrPose(pose)
	self.queue:pushBack(pose:copy())
end

function OnlineFilter:getFiltered()
	if #self.queue.data==self.queue.n then
		local sum=vectorn(self.queue:back():size())
		sum:setAllValue(0)

		local w=0
		--for i,v in ipairs(self.queue.data) do local v=self.queue.data[i]
		for i=1,self.queue.n do local v=self.queue:get(i)
			sum:radd(v*self.kernel(i-1))
			w=w+self.kernel(i-1)
		end
		if true then
			sum:range(3,7):setAllValue(0)
			for i=1,self.queue.n do local v=self.queue:get(i)
				--local k=self.kernel(i-1)
				local k
				if i==math.floor(self.queue.n/2) then
					k=1
				else
					k=0
				end
				sum:range(3,7):radd(v:range(3,7)*(k*k))
			end
		end
		sum:rmult(1/w)
		sum:setQuater(3, sum:toQuater(3):Normalize())


		if false then
			local out={}
			for i=1, self.queue.n-1 do
				local qmid=quater()
				qmid:safeSlerp(self.queue:get(i):toQuater(3), self.queue:get(i+1):toQuater(3), 0.5)
				table.insert(out, qmid)
			end
			while #out>1 do
				local out2={}
				for i=1, #out-1 do
					local qmid=quater()
					qmid:safeSlerp(out[i], out[i+1], 0.5)
					table.insert(out2, qmid)
				end
				out=out2
			end

			assert(#out==1)
			sum:setQuater(3, out[1])
		end
		return sum
	else
		return self.queue:back()
	end
end
OnlineLTIFilter=LUAclass()

function OnlineLTIFilter:__init(pose, filterSize)
	self.filterSize=filterSize
	self.queue=Queue(filterSize)
end

function OnlineLTIFilter:setCurrPose(pose)
	self.queue:pushBack(pose:copy())
end

function OnlineLTIFilter:getFiltered()
	if #self.queue.data==self.queue.n then
		local sum=vectorn(self.queue:back():size())
		sum:setAllValue(0)

		for i,v in ipairs(self.queue.data) do
			sum:radd(self.queue.data[i])
		end
		sum:rmult(1/self.queue.n)
		return sum
	else
		return self.queue:back()
	end
end


IKChain=LUAclass()

function IKChain:__init(loader, ikc, kneeIndex, axis, mEffectors, g_con, option)
	self.solvers={}
	if not option then option={true, true, true} end
	if option[1] then
		-- semi-hybrid lbfgs
		local ik=MotionUtil.createFullbodyIk_MotionDOF_semihybrid_nlopt(loader.dofInfo,mEffectors,g_con,kneeIndex, axis )
		table.insert(self.solvers, ik)
	end

	if option[2] then
		local emptyEffectors=MotionUtil.Effectors()
		local ik=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(loader.dofInfo,emptyEffectors,g_con)
		ik:_changeNumEffectors(2)
		ik:_setEffector(0, mEffectors(0).bone, mEffectors(0).localpos)
		ik:_setEffector(1, mEffectors(1).bone, mEffectors(1).localpos)
		table.insert(self.solvers, ik)
	end

	if option[3] then
		-- finger ik
		local emptyEffectors=MotionUtil.Effectors()
		local ik=MotionUtil.createFullbodyIk_subtree_lbfgs(loader.dofInfo, emptyEffectors, kneeIndex)
		ik:_changeNumEffectors(2)
		ik:_setEffector(0, mEffectors(0).bone, mEffectors(0).localpos)
		ik:_setEffector(1, mEffectors(1).bone, mEffectors(1).localpos)

		table.insert(self.solvers, ik)
	end

end
function IKChain:_changeNumEffectors(n)
	for i,v in ipairs(self.solvers) do
		v:_changeNumEffectors(n)
	end
end
function IKChain:_setEffector(a,b,c)
	for i,v in ipairs(self.solvers) do
		v:_setEffector(a,b,c)
	end
end
function IKChain:_setHalfSpaceConstraint(a,bone,lpos, normal,d)
	for i,v in ipairs(self.solvers) do
		v:_setHalfSpaceConstraint(a,bone,lpos,normal,d)
	end
end
function IKChain:_setPlaneDistanceConstraint(a,bone,lpos, normal,d)
	for i,v in ipairs(self.solvers) do
		v:_setPlaneDistanceConstraint(a,bone,lpos,normal,d)
	end
end
function IKChain:_effectorUpdated()
	for i,v in ipairs(self.solvers) do
		v:_effectorUpdated()
	end
end
function IKChain:_numConstraints()
	return self.solvers[1]:_numConstraints()
end
function IKChain:_changeNumConstraints(n)
	for i,v in ipairs(self.solvers) do
		v:_changeNumConstraints(n)
	end
end
function IKChain:IKsolve(pose,footPos)

	--[[
	local footOri=quaterN(2)
	local footPos=vector3N(2)
	local mEffectors=self.mEffectors
	for i=0,1 do
		--local originalPos=mEffectors(i).bone:getFrame():toGlobalPos(mEffectors(i).localpos)
		local originalPos=mCollAvoid.mOriginalPos[i+1]
		--footPos(i):assign(mCON.conPos(i)*0.01);
		footPos(i):assign(originalPos)
		footOri(i):assign(mCollAvoid.mEffectors(i).bone:getFrame().rotation)
	end
	]]--

	if not self.iter then dbg.console() end
	local csolver=math.mod(self.iter-1, #self.solvers)+1
	self.solvers[csolver]:IKsolve(pose, footPos)
end


