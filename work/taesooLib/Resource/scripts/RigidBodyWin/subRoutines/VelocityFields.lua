
require("common")
VelocityFields=LUAclass()

-- default_options={  frameRate=120, alignRoot=false }
function VelocityFields:__init(loader, motdofc, initialPose, options )
	self.skel=loader

	if not options then
		options={}
	end
	local frameRate=options.frameRate
	if not frameRate then frameRate=120 end
	if not options.alignRoot then options.alignRoot=false end

	if not model then
		model={
			frame_rate=frameRate
		}
	else
		model.frame_rate=frameRate
	end
	self.dmot=calcDerivative(motdofc.mot, motdofc.discontinuity)

	self.frameRate=frameRate

	if initialPose then
		self.pose=initialPose:copy()
	end
	self.alignRoot=options.alignRoot
	self.integrator=MotionUtil.LoaderToTree(loader, false, false)
end

function VelocityFields:setInitPose(initialPose)
	self.pose=initialPose:copy();
end

function projectAngles(dmotionDOF_i)
   MainLib.VRMLloader.projectAngles(dmotionDOF_i) -- align angles
end

function calcVelocity(p1, p2, frame_rate)
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
function VelocityFields:singleStep(dpose)
	local dofInfo=self.skel.dofInfo

	self.integrator:setPoseDOF(dofInfo, self.pose)
	self.integrator:integrate(dofInfo, dpose, 1/self.frameRate)
	self.integrator:getPoseDOF(dofInfo, self.pose)
end

function VelocityFields:stepKinematic(dpose, nextpose, weight, clampVel)
	local dofInfo=self.skel.dofInfo

	self.integrator:setPoseDOF(dofInfo, self.pose)
	self.integrator:integrate(dofInfo, dpose, 1/self.frameRate)
	self.integrator:getPoseDOF(dofInfo, self.pose)

	if nextpose then
		local npose
		if self.alignRoot then
			local root=MotionDOF.rootTransformation(self.pose) 
			local root2=MotionDOF.rootTransformation(nextpose) 
			root2:align2D(root)
			npose=nextpose:copy()
			MotionDOF.setRootTransformation(npose, root2)
		else
			npose=nextpose
		end

		if clampVel then
			local out=zero
			local v=calcVelocity(self.pose, npose, self.frameRate)
			v:rmult(weight)
			v:clamp(clampVel or 10)
			self:singleStep(v)
		else
			local out=self.pose:copy()
			dofInfo:blend(out, self.pose, npose, weight)
			self.pose=out
		end
	end
end



CollisionAvoid=LUAclass()

function CollisionAvoid:createIKsolver(ikc, kneeIndex, axis)
	if true then
		-- nlopt or lbfgs
		self.mIK=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(self.loader.dofInfo,self.mEffectors,self.g_con)
		--self.mIK=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_nlopt(mLoader.dofInfo,self.mEffectors,self.g_con)
		--mIK=MotionUtil.createFullbodyIk_MotionDOF_hybrid_lbfgs(mLoader.dofInfo,mEffectors,g_con, CT.ivec(lknee:treeIndex(), rknee:treeIndex()), CT.vec(1,1))
		self.mEffectors:resize(2);
		self.mEffectors(0):init(self.loader:getBoneByName(ikc[1][2]), ikc[1][3])
		self.mEffectors(1):init(self.loader:getBoneByName(ikc[2][2]), ikc[2][3])

		local mEffectors=self.mEffectors
		self.mIK:_changeNumEffectors(2)
		self.mIK:_setEffector(0, mEffectors(0).bone, mEffectors(0).localpos)
		self.mIK:_setEffector(1, mEffectors(1).bone, mEffectors(1).localpos)
	else
		-- semi-hybrid lbfgs
		self.mEffectors:resize(2);

		self.mEffectors(0):init(self.loader:getBoneByName(ikc[1][2]), ikc[1][3])
		self.mEffectors(1):init(self.loader:getBoneByName(ikc[2][2]), ikc[2][3])

		--self.mIK=MotionUtil.createFullbodyIk_MotionDOF_semihybrid_lbfgs(mLoader.dofInfo,self.mEffectors,self.g_con,kneeIndex, axis )
		self.mIK=MotionUtil.createFullbodyIk_MotionDOF_semihybrid_nlopt(self.loader.dofInfo,self.mEffectors,self.g_con,kneeIndex, axis )
	end
end

function CollisionAvoid:setIter(n)
end
function CollisionAvoid:__init(mLoader, config)
	assert(config.skinScale)
	self.config=config
	self.loader=mLoader
	self.g_con=MotionUtil.Constraints() -- std::vector<MotionUtil::RelativeConstraint>
	self.g_con:resize(0)
	self.mEffectors=MotionUtil.Effectors()

	local ikc=config[3]
	self.lknee=mLoader:getBoneByName(ikc[1][1])
	self.rknee=mLoader:getBoneByName(ikc[2][1])
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
		kneeIndex:set(2, mLoader:getTreeIndexByName(ikc[3][1]))
		kneeIndex:set(3, mLoader:getTreeIndexByName(ikc[4][1]))
		if ikc[3].reversed then axis:set(2,-1) end
		if ikc[4].reversed then axis:set(3,-1) end
	end

	self:createIKsolver(ikc, kneeIndex, axis)
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
-- i in [0, n-1]
function Queue:getElt(i)
	return self.data[math.fmod(i+self.front-1, self.n)+1]
end

function Queue:front()
	return self.data[self.front]
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
					if iloader1==iloader2 then
						--count=count+collisionPoints:size()*2 --양쪽 밀기.
						count=count+collisionPoints:size() -- 한쪽 밀기(링크 페어중 첫번째 놈을 민다. 즉 누구를 밀지 지정 가능).
					else
						count=count+collisionPoints:size()
					end
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

			local config=self.config
			if iloader1==loaderIndex or iloader2==loaderIndex then
				for j=0, collisionPoints:size()-1 do
					local b=collisionPoints(j)
					--print(loader1:name(), loader1:VRMLbone(linkpair[2]).NameId, loader2:name(), loader2:VRMLbone(linkpair[4]).NameId)
					if mObstacleSkins and iloader2~=0 then
						mObstacleSkins[iloader2]:setMaterial('red_transparent')
					end
					if method~=4 then
						lines:pushBack(b.position*config.skinScale) -- inside bone1, on the bone2's surface
						--lines:pushBack((b.position+b.normal)*config.skinScale)
						lines:pushBack((b.position+b.normal*b.idepth)*config.skinScale) -- inside bone2, on the bone1's surface
						--lines:pushBack((b.position+b.normal*b.idepth)*config.skinScale)
						lines:pushBack((b.position+b.normal*b.idepth*2)*config.skinScale)
						lines:pushBack((b.position+b.normal)*config.skinScale)
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

						local depth=b.idepth
						if iloader1==iloader2 then
							--depth=depth*0.5
						end

						if iloader1==loaderIndex then
							local bone=bone1
							local localpos=bone:getFrame():toLocalPos(b.position+b.normal*depth)
							--local localpos=bone:getFrame():toLocalPos(b.position+b.normal*depth*0.5)
							local plane=Plane(b.normal, b.position)
							mIK:_setHalfSpaceConstraint(start_con+count, bone, localpos, plane.normal, plane.d-margin)
						end
						if iloader1==iloader2 then
							--count=count+1 -- 양쪽 밀기시 uncomment.
						else
							if iloader2==loaderIndex then
								local bone=bases:getBone2(ilinkpair)
								local localpos=bone:getFrame():toLocalPos(b.position)
								--local localpos=bone:getFrame():toLocalPos(b.position+b.normal*depth*0.5)
								local plane=Plane(-b.normal, b.position+b.normal*depth)
								mIK:_setHalfSpaceConstraint(start_con+count, bone, localpos, plane.normal, plane.d-margin)
							end
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
			dbg.draw('Traj', lines:matView(), 'normals'..loaderIndex,'solidred')
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

					lines:pushBack(out_k[4]*config.skinScale) 
					lines:pushBack(out_k[4]+out_k[3]*config.skinScale) 
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
OnlineFilter6D=LUAclass()
function OnlineFilter6D:__init(loader, pose, filterSize, useMitchellFilter)
	self.filterSize=filterSize
	self.loader=loader
	local pose6D
	if pose then
		loader:setPoseDOF(pose)
		pose6D=loader:getPose6D()
	end

	self.filter=OnlineFilter(nil,pose6D, filterSize, useMitchellFilter) 
end
function OnlineFilter6D:to6D(pose)
	local loader=self.loader
	loader:setPoseDOF(pose)
	return loader:getPose6D()
end
function OnlineFilter6D:toEuler(pose6D)
	local loader=self.loader
	loader:setPose6D(pose6D)
	return loader:getPoseDOF()
end
function OnlineFilter6D:setCurrPose(pose)
	self.filter:setCurrPose(self:to6D(pose))
end
function OnlineFilter6D:getCenterAndFiltered()
	local center, filtered=self.filter:getCenterAndFiltered()
	return center, self:toEuler(filtered)
end

function OnlineFilter6D:getFiltered()
	local filtered=self.filter:getFiltered()
	return self:toEuler(filtered)
end

OnlineFilter=LUAclass()

function OnlineFilter:__init(loader, pose, filterSize, useMitchellFilter)
	self.filterSize=filterSize
	self.loader=loader
	self.queue=Queue(filterSize)
	if pose then
		self:setCurrPose(pose)
	end
	self.useMitchellFilter=useMitchellFilter or false
end

function OnlineFilter:setCurrPose(pose)
	self.queue:pushBack(pose:copy())
end
-- center: unfiltered frame, temporally corresponding to the filtered frame
function OnlineFilter:getCenterAndFiltered()
	if #self.queue.data==self.queue.n then
		local centerFrame=math.floor(self.queue.n/2)
		local center=self.queue:getElt(centerFrame):copy()
		local filtered=self:getFiltered()
		return center, filtered
	else
		local center=self:getFiltered()
		return center, center:copy()
	end
end


function OnlineFilter:getFiltered()
	if #self.queue.data==self.queue.n then
		function filterSingle(v, n, useMitchellFilter)
			if n==2 then
				return v:row(0)*0.5+v:row(1)*0.5
			else
				return math.filterSingle(v, n, useMitchellFilter)
			end
		end
		if not self.loader then
			local sum=vectorn(self.queue:back():size())
			-- use gaussian filter for all DOFs
			local arrayV=matrixn(self.queue.n, self.queue:back():size())
			for i=0, arrayV:size()-1 do
				arrayV:row(i):assign(self.queue:getElt(i))
			end
			local sum=filterSingle(arrayV, self.queue.n, self.useMitchellFilter)
			return sum
		end

		local sum=vectorn(self.queue:back():size())
		sum:setAllValue(0)

		function filterQuatSingle(v, n, useMitchellFilter)
			if n==2 then
				assert(v:cols()==4)
				local out=vectorn(4)
				local q=quater()
				q:safeSlerp(v:row(0):toQuater(0), v:row(1):toQuater(0), 0.5)
				out:setQuater(0, q)
				return out
			else
				return math.filterSingle(v, n, useMitchellFilter)
			end
		end
		if true then
			-- use gaussian filter for joint angles
			local arrayV=matrixn(self.queue.n, self.queue:back():size()-7)
			for i=0, arrayV:size()-1 do
				arrayV:row(i):assign(self.queue:getElt(i):slice(7,0))
			end
			local v=filterSingle(arrayV, self.queue.n, self.useMitchellFilter)
			sum:slice(7,0):assign(v)
		else

			for i,v in ipairs(self.queue.data) do
				sum:radd(self.queue.data[i])
			end
			sum:rmult(1/self.queue.n)
		end
		if false then
			-- simple renormalization works only when filter size is small
			sum:setQuater(3, sum:toQuater(3):Normalize())
		else
			-- use gaussian filter for root pos and ori.
			local arrayQ=quaterN(self.queue.n)
			local arrayV=vector3N(self.queue.n)
			for i=0, arrayQ:size()-1 do
				arrayQ(i):assign(self.queue:getElt(i):toQuater(3))
				arrayV(i):assign(self.queue:getElt(i):toVector3(0))
			end

			arrayQ:align()
			--math.filterQuat(arrayQ:matView(), self.queue.n)
			--sum:setQuater(3, arrayQ(math.floor(self.queue.n/2)))
			local q=filterQuatSingle(arrayQ:matView(), self.queue.n, self.useMitchellFilter)
			local v=filterSingle(arrayV:matView(), self.queue.n, self.useMitchellFilter)
			sum:setVec3(0, v:toVector3())
			sum:setQuater(3, q:toQuater())
		end
		return sum
	else
		return self.queue:back()
	end
end

MaskedOnlineFilter=LUAclass()

function MaskedOnlineFilter:__init(filterSize, mask_for_filtered, loader)
	self.mask=mask_for_filtered
	self.filter=OnlineFilter(filterSize, loader)
end
function MaskedOnlineFilter:setCurrPose( posedof)
	self.filter:setCurrPose(posedof)
end
function MaskedOnlineFilter:getFiltered()
	local unfiltered, filtered=self.filter:getCenterAndFiltered()
	return filtered*self.mask - unfiltered*(self.mask-1.0)
end

IKChain=LUAclass()

function IKChain:__init(mLoader, ikc, kneeIndex, axis, mEffectors, g_con, option)
	self.solvers={}
	if not option then option={true, true, true} end
	if option[1] then
		-- semi-hybrid lbfgs
		local emptyEffectors=MotionUtil.Effectors()
		local ik=MotionUtil.createFullbodyIk_MotionDOF_semihybrid_nlopt(mLoader.dofInfo,emptyEffectors,g_con,kneeIndex, axis )
		ik:_changeNumEffectors(2)
		ik:_setEffector(0, mEffectors(0).bone, mEffectors(0).localpos)
		ik:_setEffector(1, mEffectors(1).bone, mEffectors(1).localpos)
		table.insert(self.solvers, ik)
	end

	if option[2] then
		local emptyEffectors=MotionUtil.Effectors()
		local ik=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(mLoader.dofInfo,emptyEffectors,g_con)
		ik:_changeNumEffectors(2)
		ik:_setEffector(0, mEffectors(0).bone, mEffectors(0).localpos)
		ik:_setEffector(1, mEffectors(1).bone, mEffectors(1).localpos)
		table.insert(self.solvers, ik)
	end

	if option[3] then
		-- finger ik
		local emptyEffectors=MotionUtil.Effectors()
		local ik=MotionUtil.createFullbodyIk_subtree_lbfgs(mLoader.dofInfo, emptyEffectors, kneeIndex)
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

	--for i,v in ipairs(self.solvers) do
	--	v:IKsolve(pose,footPos)
	--end
	if not self.iter then dbg.console() end
	local csolver=math.mod(self.iter-1, #self.solvers)+1
	self.solvers[csolver]:IKsolve(pose, footPos)
end
