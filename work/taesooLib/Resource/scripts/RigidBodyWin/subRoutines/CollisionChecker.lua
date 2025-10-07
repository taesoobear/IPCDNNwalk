
CollisionChecker=LUAclass()
function CollisionChecker:__init(...)
	local obj={...}
	local colType='libccd'
	--local colType='fcl'
	--local colType='gjk'
	if type(obj[1])=='string' then
		colType=obj[1]
		obj=obj[2] or {}
	end
	self.nbObj=0;
	-- in cpp, use OpenHRP::createCollisionDetector_libccd() or ..._bullet()
	if colType=='libccd' then
		self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_libccd()
	elseif colType=='fcl' then
		self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_fcl()
	elseif colType=='gjk' then
		self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_gjk()
	elseif colType=='libccd_merged' then
		self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_libccd_merged()
	elseif colType=='sdf' then
		require("Kinematics/meshTools")
		self.collisionDetector=SDFcollider({debugDraw=false})
		--self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_gjk()
		--function Physics.CollisionDetector:calculateNearestSurfacePoint(iloader, ibody, in_global_pos)
		--	local out_normal=vector3()
		--	local dist=self:calculateSignedDistance(iloader, ibody, in_global_pos, out_normal)
		--	return dist, in_global_pos-out_normal*dist
		--end
	else
		print('colType' , colType)
		self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_gjk()
	end
	--self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_libccd_LBS()
	--self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_bullet()
	--self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_gjk() -- experimental
	self.collisionSequence=Physics.CollisionSequence()

	self.pose={}
	for i,v in ipairs(obj) do
		self:addObject(v)
	end
end

function CollisionChecker._buildObstacleMeshes(obstacles, meshScale)
	local mObstacles={}
	for i=1,#obstacles.obstaclePos do
		mObstacles[i]=Geometry()

		do 
			local mesh=mObstacles[i]
			if obstacles.obstacleType[i]=="BOX" then
				mesh:initBox(obstacles.obstacleSize[i])
			elseif obstacles.obstacleType[i]=="SPHERE" then
				mesh:initEllipsoid(obstacles.obstacleSize[i])
			elseif obstacles.obstacleType[i]=="CAPSULE" then
				mesh:initCapsule(obstacles.obstacleSize[i].x, obstacles.obstacleSize[i].z)
			elseif obstacles.obstacleType[i]=="CHAR" then
				mObstacles[i]='CHAR'
			else
				mesh:initPlane(obstacles.obstacleSize[i].x, obstacles.obstacleSize[i].z)
			end

			if obstacles.obstacleType[i]~="CHAR" then
				local mat=matrix4()
				mat:identity()
				mat:leftMultScale(meshScale)
				mesh:transform(mat)
			end
		end
	end
	return mObstacles
end
function CollisionChecker:addObstacles(obstacles, meshScale)
	self.obstacles=obstacles
	self.obstacleMeshes=self._buildObstacleMeshes(obstacles, meshScale)
	local coldet=self.collisionDetector
	assert(coldet:numModels()==1)
	for i,v in ipairs(self.obstacleMeshes) do
		local ltype=getmetatable(v).luna_class
		if ltype=='MainLib.VRMLloader' then
			coldet:addModel(v)
		else
			coldet:addObstacle(v)
		end
		if obstacles.obstacleName then
			coldet:getModel(coldet:numModels()-1):setName(obstacles.obstacleName[i])
		end
	end
	assert(coldet:numModels()==1+#self.obstacleMeshes)
end
function CollisionChecker:detectCollisionFrames(motdof, float_options, thr)
	thr=thr or 0
	local mChecker=self
	local col=boolN(motdof:numFrames())

	for i=0, motdof:rows()-1 do
		mChecker:setPoseDOF(0,motdof:row(i))
		local bases, maxDepth=mChecker:checkCollision(float_options)
		if float_options.countSelfCollisionOnly then
			local newmaxDepth=0
			local collisionLinkPairs=bases:getCollisionLinkPairs()
			if collisionLinkPairs:size()>=1 then
				for i=0, collisionLinkPairs:size()-1 do
					local ilinkpair=collisionLinkPairs(i)
					local iloader1=bases:getCharacterIndex1(ilinkpair)
					local iloader2=bases:getCharacterIndex2(ilinkpair)
					if iloader1==0 and iloader2==0 then
						local collisionPoints=bases:getCollisionPoints(ilinkpair)
						if collisionPoints:size()>0 then
							for icp=0, collisionPoints:size()-1 do
								local cp=collisionPoints(icp)
								newmaxDepth=math.max(cp.idepth, newmaxDepth)
							end
						end
					end
				end
			end
			maxDepth=newmaxDepth
		end
		if maxDepth>thr then
			col:set(i, true)
		end
	end

	return col
end

function CollisionChecker:addObject(v)
	local ltype=getmetatable(v).luna_class
	if ltype=='MainLib.VRMLloader' then
		self.collisionDetector:addModel(v)
	else
		self.collisionDetector:addObstacle(v)
	end
	self.nbObj=self.nbObj+1
end

-- s: from , t: to in cm scale.
function CollisionChecker:checkRayIntersection(s, t)
	local cd=self.collisionDetector
	local r=Physics.RayTestResult ()
	s=s/config.skinScale
	t=t/config.skinScale

	local clostest_T=1.0
	local hitPos
	local hitNormal

	for j=1,self.nbObj do
		local nb=cd:getModel(j-1):numBone()
		for i=1, nb -1 do
			cd:rayTest(j-1, i,s ,t , r)
			--cd:rayTestBackside(j-1, i,s ,t , r)

			if r:hasHit() and r.m_closestHitFraction<clostest_T then
				clostest_T=r.m_closestHitFraction
				local o=vector3()
				o:interpolate(r.m_closestHitFraction, s,t)
				hitPos=o
				hitNormal=r.m_hitNormalWorld:copy()
			end
		end
	end
	return clostest_T~=1.0, hitPos, hitNormal
end
function CollisionChecker:setPoseDOF(iloader, dof)
	local loader=self.collisionDetector:getModel(iloader)
	assert(dof:size()==loader.dofInfo:numDOF())
	loader:setPoseDOF(dof)
	self.collisionDetector:setWorldTransformations(iloader, loader:fkSolver())
	self.pose[iloader]=dof:copy()
end


function CollisionChecker:registerSelfPairs(iloader1, collpairs)
	local loader1=self.collisionDetector:getModel(iloader1)
	for ii, v in ipairs(collpairs) do
		local i, j
		if type(v[1])=='string' then
			i=loader1:getTreeIndexByName(v[1])
			j=loader1:getTreeIndexByName(v[2])
		else
			i=v[1]
			j=v[2]
		end
		local bone_i=loader1:VRMLbone(i)
		local bone_j=loader1:VRMLbone(j)
		self.collisionDetector:addCollisionPair(loader1, bone_i:treeIndex(), loader1, bone_j:treeIndex())
	end
	assert(iloader1==0)
	self.collpairs=collpairs
end

function CollisionChecker:registerPair(iloader1, iloader2)
	local loader1=self.collisionDetector:getModel(iloader1)
	local loader2=self.collisionDetector:getModel(iloader2)
	for i=1,loader1:numBone()-1 do
		for j=1,loader2:numBone()-1 do
			local bone_i=loader1:VRMLbone(i)
			local bone_j=loader2:VRMLbone(j)
			self.collisionDetector:addCollisionPair(loader1, bone_i:treeIndex(), loader2, bone_j:treeIndex())
		end
	end
end
function CollisionChecker:checkCollision(float_options)
	if float_options then
		local maxDepth=self.collisionDetector:testIntersectionsForDefinedPairs(self.collisionSequence, float_options)
		return self.collisionSequence, maxDepth
	else
		self.collisionDetector:testIntersectionsForDefinedPairs(self.collisionSequence)
		return self.collisionSequence
	end
end

do 

	CollisionCheckerUsingSimulator=LUAclass()
	function CollisionCheckerUsingSimulator:__init(...)
		local obj={...}
		self.nbObj=#obj;
		--self.simulator=Physics.DynamicsSimulator_gmbs("bullet") -- works well
		--self.simulator=Physics.DynamicsSimulator_gmbs("opcode") -- works best, but extremely slow.
		self.simulator=Physics.DynamicsSimulator_gmbs("libccd") -- currently developing. works only for simple geometries now.


		for i,v in ipairs(obj) do
			local ltype=getmetatable(v).luna_class
			if ltype=='MainLib.VRMLloader' then
				self.simulator:registerCharacter(v);
			else
				--self.simulator:createObstacle(v)
				self.simulator:createFreeBody(v);
			end

			local v=self.simulator:skeleton(i-1)

			print(v:name())
			local zero=CT.zeros(v.dofInfo:numDOF())
			if zero:size()>0 then
				zero:set(3, 1) -- quater(1,0,0,0)
				self.simulator:setLinkData(i-1, Physics.DynamicsSimulator.JOINT_VALUE, zero)
			end
		end

		local timestep=1
		--local timestep=1/8000
		self.timestep=timestep
		self.simulator:init(timestep, Physics.DynamicsSimulator.EULER)
	end

	function registerContactPairAll(model, loader, floor, simulator)
		local param=vectorn()
		param:setValues(0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp)
		for i=1,loader:numBone()-1 do
			local bone_i=loader:VRMLbone(i)
			simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param)
		end
	end
	function CollisionCheckerUsingSimulator:setPoseDOF(iloader, dof)
		self.simulator:setLinkData(iloader, Physics.DynamicsSimulator.JOINT_VALUE, dof)
	end



	function CollisionCheckerUsingSimulator:registerPair(iloader1, iloader2)
		local param=vectorn ()
		--param:setValues(0.5,0.5, 10000,1000)
		param:setValues(0.6,0.6, 10000,1000)
		local loader1=self.simulator:skeleton(iloader1)
		local loader2=self.simulator:skeleton(iloader2)
		--print(loader1:name(),loader2:name())
		self.linkpairs=self.linkpairs or {}
		for i=1,loader1:numBone()-1 do
			for j=1,loader2:numBone()-1 do
				local bone_i=loader1:VRMLbone(i)
				local bone_j=loader2:VRMLbone(j)
				self.simulator:registerCollisionCheckPair(loader1:name(),bone_i.NameId, loader2:name(), bone_j.NameId, param)
				--print('registerPair', bone_i.NameId, bone_j.NameId);
				table.insert(self.linkpairs, {iloader1, i,iloader2, j})
			end
		end
	end
	function CollisionCheckerUsingSimulator:checkCollision()
		self.simulator:initSimulation()
		local bases=Physics.Vec_ContactBasis()
		self.simulator:getContactBases(bases,0.5)
		--print(bases:size(), #self.linkpairs)
		--
		return bases
	end
	function CollisionCheckerUsingSimulator:checkRayIntersection(iloaders, startPos, direction)
		local sim=self.simulator 
		local cd=sim:getCollisionDetector()
		for i=0, startPos:size()-1 do
			local r=Physics.RayTestResult ()
			local s=startPos(i)/config.skinScale
			local t=s+direction*100

			for j=1,#iloaders do
				local ibone=1
				cd:rayTest(iloaders[j], ibone,s ,t , r)

				if r:hasHit() then
					local o=vector3()
					o:interpolate(r.m_closestHitFraction, s,t)
					--dbg.draw("Sphere", o*config.skinScale, "x0")
					dbg.draw('Arrow', startPos(i), o*config.skinScale, 'arraow'..i..'_'..j, 5, 'red_transparent')
				else
					dbg.erase('Arrow', 'arraow'..i..'_'..j)
				end
			end
		end
	end
end

do 
	EVR=LUAclass(EventReceiver)
	function EVR:__init()
	end
	function EVR:onFrameChanged(win, iframe)
		self:_onFrameChanged(win, iframe)
	end

	EVRecoder=LUAclass(EventReceiver)
	-- mode: "save", "load", "none",
	function EVRecoder:__init(mode, filename)
		self.drawCursor=true
		self.currFrame=0
		self.crow=0
		self.filename=filename
		self:changeMode(mode)

		self.invEvMap={"DRAG", "RELEASE", "KEYUP","KEYDOWN", "MOVE", "PUSH", "onFrameChanged","FrameMove"}
		self.evMap={}
		for i,v in ipairs(self.invEvMap) do
			self.evMap[v]=i-1
		end
	end

	function EVRecoder:changeMode(mode)
		self.data=matrixn()
		self.mode=mode
		if mode=="load" then
			self.tbl=util.loadTable(self.filename)
			self.data=self.tbl[1]
		end
	end
	-- reimplment these functions (whose name start with _)
	function EVRecoder:_onFrameChanged(win, iframe)
	end
	function EVRecoder:_handleRendererEvent(ev, button, x,y) 
	end
	function EVRecoder:_frameMove(fElapsedTime)
	end

	function handleRendererEvent(ev, button, x, y)
		return mEventReceiver:handleRendererEvent(ev, button, x,y)
	end
	function frameMove(fElapsedTime)
		mEventReceiver:frameMove(fElapsedTime)
	end
	function EVRecoder:frameMove(fElapsedTime)
		if self.mode=="load" then
			local crow=self.data:row(self.crow)
			self.crow=math.min(self.crow+1, self.data:rows()-1)
			
			ev=self.invEvMap[crow(0)+1]

			if ev=="onFrameChanged" then
				local iframe=crow(1)
				self:_onFrameChanged(win, iframe)
			elseif ev=="FrameMove" then
				local fElapsedTime=crow(1)
				self:_frameMove(fElapsedTime, fElapsedTime)
			else
				local button=crow(1)
				if ev=='KEYDOWN' or ev=='KEYUP' then
					print("button", button)
					button=tostring(button)
				end
				local x=crow(2)
				local y=crow(3)

				if self.drawCursor then
					local overlay=RE.getOverlayByName("CursorOverlay")
					if not overlay:isVisible() then
						overlay:show()
					end
					overlay:setChildPosition( "TCursor", x, y)
				end
				self:_handleRendererEvent(ev, button, x,y)
			end
		elseif self.mode=="save" then
			self.data:pushBack(CT.vec(self.evMap.FrameMove, fElapsedTime, 0,0))
			self:save()
			self:_frameMove(fElapsedTime)
		else
			self:_frameMove(fElapsedTime)
		end
	end
	function EVRecoder:save()
		if math.mod(self.data:rows(), 50)==0 then
			util.saveTable( {self.data}, self.filename)
		end
	end
	function EVRecoder:saveData()
	end
	function EVRecoder:handleRendererEvent(ev, button, x,y)
		print(ev, button, x,y)
		if self.mode=="save" then
			self.data:pushBack(CT.vec(self.evMap[ev], tonumber(button), x or 0, y or 0))
			self:save()
		elseif self.mode=="load" then
			return 0 -- ignore events
		end
		return self:_handleRendererEvent(ev, button, x,y)
	end

	function EVRecoder:onFrameChanged(win, iframe)
		if self.mode=="save" then
			self.data:pushBack(CT.vec(self.evMap.onFrameChanged, iframe, 0,0))
			self:save()
		elseif self.mode=="load" then
			return -- all events are played back in framemove
		end
		self:_onFrameChanged(win, iframe)
	end

end


Collider2D=LUAclass()
function Collider2D:__init(info)
	for k, v in pairs(info) do
		self[k]=v
	end
	assert(self.pos)
	self.pos.y=0
	self.colType=info[1]
	self.color='blue_transparent'
	if Physics.ConvexShape2D then
		function vector2:toZX()
			return vector3(self.y, 0, self.x)
		end
		function vector3:projectZX()
			return vector2(self.z, self.x)
		end
		function Physics.ConvexShape2D :setPosition3D(v3)
			self:setPosition(v3:projectZX())
		end
		function Physics.ConvexShape2D :setTransform3D(q3, v3)
			if q3 then
				self:setTransform(q3:rotationAngleAboutAxis(vector3(0,1,0)), v3:projectZX())
			else
				self:setPosition(v3:projectZX())
			end
		end
		function Physics.ConvexShape2D:getPosition3D()
			local v=self:getPosition()
			return v:toZX()
		end
		local v
		if self.colType=='Circle' then
			v=vector2N(10)
			for i=0, 9 do
				local r=vector3(0,0,info.r)
				local R=quater(sop.map(i, 0,10, -math.pi, math.pi), vector3(0,1,0))*r
				v(i).x=R.z
				v(i).y=R.x
			end
		else
			assert(self.colType=='Box')
			v=vector2N(4)
			local l=info.localSize*0.5
			v(0).x=l.z
			v(0).y=l.x
			v(1).x=l.z
			v(1).y=-l.x
			v(2).x=-l.z
			v(2).y=-l.x
			v(3).x=-l.z
			v(3).y=l.x
		end
		self.shape=Physics.ConvexShape2D(v)
		self.shape:setTransform3D(info.q, info.pos)
	end
end
function Collider2D:draw(i)
	i = i or 1
	if self.colType=='Circle' then
		dbg.draw("Circle", self.pos*100, "_ball"..i, self.color, self.r*100)
	elseif self.colType=="Box" then
		dbg.draw("Box", transf(self.q, self.pos), '_box'..i, self.localSize, 100, self.color)
	end
end
function Collider2D:check(other)
	-- box-box checks are not implemented yet
	if self.colType=='Circle' and other.colType=='Circle' then
		local p1=self.pos:copy()
		local p2=other.pos:copy()
		p1.y=0 p2.y=0
		local depth=p1:distance(p2)-(self.r+other.r)
		if depth<0 then
			self.idepth=depth*-0.5
			other.idepth=depth*-0.5
			self.normal=p1-p2
			self.normal:normalize()
			other.normal=self.normal*-1
			if self.r>other.r then
				self.contactCenter=other.pos+(other.r-other.idepth)*self.normal
				self.contactCenter.y=0
				other.contactCenter=self.contactCenter
			else
				other.contactCenter=self.pos+(self.r-self.idepth)*other.normal
				other.contactCenter.y=0
				self.contactCenter=other.contactCenter
			end
			return true
		end
		return false
	elseif self.shape then
		self.shape:setTransform3D(self.q, self.pos)
		other.shape:setTransform3D(other.q, other.pos)
		local contactInfo=self.shape:testIntersection(other.shape)
		if contactInfo.depth<=0 then
			return false
		else
			self.contactCenter=contactInfo.posB:toZX()*0.5+contactInfo.posA:toZX()*0.5
			self.idepth=contactInfo.depth
			self.normal=contactInfo.normal:toZX()*-0.5
				other.contactCenter=self.contactCenter
				other.normal=self.normal*-1
				other.idepth=self.idepth
			return true
		end

	elseif self.colType=='Circle' then
		return other:check(self)
	elseif other.colType=='Circle' then
		local circle=other.pos:copy()
		circle.y=0
		local rect=transf(self.q, self.pos)
		rect.translation.y=0

		local pos=vector3()
		local normal=vector3()
		local out=Physics.CollisionDetector_libccd.testSphereBox(circle, other.r, rect,self.localSize, pos, normal) 
		if out.x==-1 then
			return false
		else
			pos.y=0
			normal.y=0
			normal:normalize()
			self.normal=normal
			other.normal=normal*-1
			self.contactCenter=pos
			other.contactCenter=pos
			self.idepth=out.y*0.5
			other.idepth=out.y*0.5
			return true
		end
	end
	return false
end

