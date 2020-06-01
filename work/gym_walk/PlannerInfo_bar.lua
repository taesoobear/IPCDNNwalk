
function matrixn:sampleVec(t, c)
	local v=vectorn()
	if t>self:rows()-1 then
		t=self:rows()-1
	end
	self:sampleRow(t, v)
	return v:toVector3(c)
end
function matrixn:sampleQuat(t, c)
	local v=vectorn()
	if t>self:rows()-1 then
		t=self:rows()-1
	end
	self:sampleRow(t, v)
	local q=v:toQuater(c)
	q:normalize()
	return q
end
matrixnView.sampleQuat=matrixn.sampleQuat
matrixnView.sampleVec=matrixn.sampleVec

function toYUP0(v)
	return vector3(v.y, v.z, v.x)
end
function toZUP0(v)
	return vector3(v.z, v.x, v.y)
end
function toZUP_ori0(q)
	return quater(q.w, q.z, q.x, q.y)
end
PlannerInfo=LUAclass ()
function PlannerInfo:__init(planner, constrainFootPosition, config)
	self._terrainOffset=vector3(0,0,0)
	self.planner=planner
	if config.useHeelAndToe then
		self.planner:setUseHeelAndToe()
		self.nLimb=config.nLimb
		self.nEE=config.nLimb*2
		self.nEE_perLimb=2
		self.useHeelAndToe=true
	else
		self.nEE=config.nLimb
		self.nEE_perLimb=1
	end
	if config.useFD then
		self.useFD=true
	else 	
		self.useFD=false
	end
	self.nLimb=config.nLimb
	self.q_toYUP=quater(1,0,0,0) -- can be changed. (rotating local coordinate)
	self.p_toYUP=vector3(0,0,0)
	self.constrainFootPosition=constrainFootPosition
	self.vinitialFootPos=vector3N(self.nEE)
	self.vinitialFootRotZ={0,0}


	local height=config.nominal_height

	local nominalFootPos={}
	for i, offset in ipairs(config.nominal_offset) do
		nominalFootPos[i]=vector3(offset, -height, 0)
	end
	if config.nominal_offset_z then
		for i, offset in ipairs(config.nominal_offset_z) do
			nominalFootPos[i].z=offset -- ZUP
		end
	end
	for i, offset in ipairs(config.nominal_offset) do
		planner:setNominalStance(i-1, toZUP0(nominalFootPos[i]))
	end

	self.nominalFootPos=nominalFootPos
	local nboxSize=vector3(0.75,0.35,0.15)*2*math.abs(height)/0.5
	planner:setMaxDevFromNominal(nboxSize) -- unused now.
end

function PlannerInfo:getState(switchTime)

	local state=self.planner:_getState(switchTime, self)
	for i=3,5 do
		local th=state[1](i)
		while th>math.pi+1e-3 do
			state[1]:set(i, th-2.0*math.pi)
			th=state[1](i)
		end
		while th<-math.pi-1e-3 do
			state[1]:set(i, th+2.0*math.pi)
			th=state[1](i)
		end
	end
	local stateYup=self:stateToYup(state)

	local customState
	if self.getCustomState then
		customState=self:getCustomState(switchTime)
	end
	local footstate={}
	if self.spprtCoord then
		footstate[1]=copyTable(self.spprtCoord)
		footstate[2]=self.footsteps
	end

	return { stateYup, footstate, customState}
end

function PlannerInfo:setInitialState(statet)
	local state2
	local stateYup=statet[1]
	-- update world coordinates so that it aligns with the body
	local rotY=stateYup.quat[1]:rotationY()

	self.q_toYUP= rotY
	self.p_toYUP=stateYup.lin[1]:copy()
	self.p_toYUP.y=0

	state2=self:stateToZup(stateYup)
	self.planner:_setInitialState(state2, self)
	if statet[2] and statet[2][1] then
		self.spprtCoord=statet[2][1]
		self.footsteps=statet[2][2]
	end

	if self.setCustomState then
		self:setCustomState(statet[3])
	end
end

function PlannerInfo:initializeGlobal()
	mMot=fc.loadMotion(input,true)
	local limbs=input.limbs
	if input.motionType and input[motionType].limbs then
		limbs=input[motionType].limbs 
	end
	mSolverInfo=fc.createIKsolver(mMot.loader, limbs)
	g_iframe=0
	g_global_time=0

	mCameraInfo={
		vpos= RE.viewpoint().vpos:copy(),
		vat=RE.viewpoint().vat:copy(),
	}
	mEventReceiver=EVR()
end

function PlannerInfo:toYUP(v)
	return self.q_toYUP*vector3(v.y, v.z, v.x)
end
function PlannerInfo:toZUP(v)
	local vv=self.q_toYUP:inverse()*v
	return vector3(vv.z, vv.x, vv.y)
end
function PlannerInfo:toYUP_pos(v)
	return self:toYUP(v)+self.p_toYUP
end
function PlannerInfo:toZUP_pos(v)
	return self:toZUP(v-self.p_toYUP)
end
-- for orientation
function PlannerInfo:toYUP_ori(q)
	return self.q_toYUP*quater(q.w, q.y,  q.z, q.x)
end
function PlannerInfo:toZUP_ori(q)
	-- qy2= q_toYUP*   qy
	-- qy=q_toYUP:inverse()*qy2
	local qq=self.q_toYUP:inverse()*q
	return quater(qq.w, qq.z,  qq.x, qq.y)
end

if not BipedPlanner then
	-- QP_controller does not have c++ implementation of BipedPlanner, so ...
	BipedPlanner=LUAclass()
	function BipedPlanner:__init() dbg.console() end
	function BipedPlanner._getSegmentID(t_global, durations)
		local eps = 1e-10 -- double precision
		assert(t_global >= 0.0);

		local t = 0;
		local i=0;
		local ni=durations:size();
		while(i<ni) do
			d=durations(i);
			t = t+d;

			if (t >= t_global-eps) then -- at junctions, returns previous spline (=)
				return vector2(i, t_global-t+d);
			end

			i=i+1
		end
		assert(false); -- this should never be reached
		return vector2(-1,0);
	end
end
function BipedPlanner:_getState(t, pi)
	local theta=vectorn()
	local dtheta=vectorn()
	if pi.useFD then
		self:getThetaFD(t, theta)
		self:getDThetaFD(t, dtheta)
	else
		self:getTheta(t, theta)
		self:getDTheta(t, dtheta)
	end

	local footpos={} 
	local footrotZ={}
	local cf=vector3N(pi.nEE)
	local dotcf=vector3N(pi.nEE)

	for i=0, pi.nEE-1, pi.nEE_perLimb do
		table.insert(footpos, self:getFootCenterPos(i, t))
		table.insert(footrotZ, vector3(0,0,self:getFootRotZ(i, t)))
	end
	for i=0, pi.nEE-1 do
		cf(i):assign(self:getContactForce(i, t))
		dotcf(i):assign(self:getContactForceDeriv(i, t))
	end

	local state=
	{
		theta, 
		dtheta,
		footpos,
		footrotZ,
		cf, 
		dotcf
	}
	return state
end
function PlannerInfo:stateToYup(state)
	local theta=state[1]
	local dtheta=state[2]
	local cf=state[5]
	local dotcf=state[6]
	local pl=self.planner

	local lin={
		theta:toVector3(0),
	}
	array.concat(lin, state[3]) -- rootpos, footpos

	local euler={
		theta:toVector3(3),
	}
	array.concat(euler, state[4]) -- rootori, footrotZ


	for i=1,#lin do
		lin[i]=self:toYUP_pos(lin[i])
	end
	local quat={}
	for i=1,#euler do
		quat[i]=self:toYUP_ori(pl:toQuater(euler[i]))
	end
	local out={}
	out.lin=lin
	out.quat=quat
	out[1]=self:toYUP(dtheta:toVector3(0))
	local euler_rate= dtheta:toVector3(3)
	out[2]=self:toYUP(pl:getAngularVelocityInWorld(euler[1], euler_rate))
	if cf then
		for i=0,cf :size()-1 do
			cf(i):assign(self:toYUP(cf(i)))
		end
		for i=0,dotcf:size()-1  do
			dotcf(i):assign(self:toYUP(dotcf(i)))
		end
		out.cf=cf
		out.dotcf=dotcf
	end
	return out
end
function PlannerInfo:stateToZup(out)
	local pl=self.planner
	local theta=vectorn(6)
	local dtheta=vectorn(6)
	theta:setVec3(0, self:toZUP_pos(out.lin[1]))
	theta:setVec3(3, pl:toEuler(self:toZUP_ori(out.quat[1])))

	local footpos={}
	for i=2,#out.lin do
		footpos[i-1]=self:toZUP_pos(out.lin[i])
	end
	local footrotZ={}
	for i=2,#out.quat do
		footrotZ[i-1]=pl:toEuler(self:toZUP_ori(out.quat[i]))
	end
	dtheta:setVec3(0, self:toZUP(out[1]))
	local euler_rate=pl:getEulerRate(theta:toVector3(3), self:toZUP(out[2]))
	dtheta:setVec3(3, euler_rate)

	local cf
	local dotcf
	if(out.cf) then
		cf=out.cf
		dotcf=out.dotcf
		for i=0,cf:size()-1 do
			cf(i):assign(self:toZUP(cf(i)))
		end
		for i=0,dotcf:size()-1 do
			dotcf(i):assign(self:toZUP(dotcf(i)))
		end
	end
	local state={
		theta, 
		dtheta,
		footpos,
		footrotZ,
		cf,
		dotcf
	}

	return state
end

function BipedPlanner:_setInitialState(state, pi)
	local theta,dtheta, footpos, footrotZ, cf, dotcf=unpack(state)
	self:setInitialPos(theta:toVector3(0))
	self:setInitialOri(theta:toVector3(3))
	self:setInitialVel(dtheta:toVector3(0))
	self:setInitialAngVel(dtheta:toVector3(3))
	if pi.useHeelAndToe then
		for i=1, pi.nLimb do
			pi.vinitialFootPos(2*(i-1)):assign(footpos[i])
			pi.vinitialFootPos(2*(i-1)+1):assign(footpos[i])
		end
	else
		for i=1, pi.nEE do
			pi.vinitialFootPos(i-1):assign(footpos[i])
		end
	end
	for i=1, pi.nLimb do
		pi.vinitialFootRotZ[i]=footrotZ[i].z
	end
	if cf then
		pi.vinitialFootCF={cf, dotcf}
	else
		pi.vinitialFootCF=nil
	end
end

function PlannerInfo:COMtrajToEulerZYX(rootTraj, t, totalTime, debug_draw)
	matrixn.numFrames=matrixn.rows -- necessary in calcDerivative
	matrixn.numDOF=matrixn.cols
	local pi=self
	local planner=self.planner
	local pendFrameRate=30
	local dotRoot=calcDerivative(rootTraj)
	dotRoot:rmult(1.0/4.0) -- 120hz vs 30hz

	local COM=rootTraj:sub(0,0,0,3):copy().. dotRoot:sub(0,0,0,3):copy()
	local ZMP=rootTraj:sub(0,0,0,3):copy()
	ZMP:column(1):setAllValue(0)
	local ANG=rootTraj:sub(0,0,3,7):copy()
	local ANGVEL=dotRoot:sub(0,0,4,7):copy()
	if debug_draw then
		local thickness=10 -- 10 cm
		dbg.namedDraw('Traj', COM*100, 'goal3', 'blueCircle', thickness, 'QuadListY' )
		dbg.namedDraw('Traj', ZMP*100, 'goal2', 'greenCircle', thickness, 'QuadListV' )
		for i=0, COM:rows()-1, 3 do
			local goalPos=COM:row(i):toVector3(0)
			local goalOri=ANG:row(i):toQuater(0)
			dbg.draw('Axes', transf(goalOri, goalPos), 'final ori'..i, 100, 0.5)
		end
	end

	local desiredCOM=vector3N(t:size())
	local desiredDotCOM=vector3N(t:size())
	local desiredEuler=vector3N(t:size())
	local desiredDotEuler=vector3N(t:size())

	self.t=t
	--self.COMheight_offset=vectorn(t:size()) -- only for drawing
	local COMheight_offset=0

	for i=0, t:size()-1 do
		local ii=t(i)*pendFrameRate

		local pendOri=ANG:sampleQuat(ii, 0)
		desiredDotCOM(i):assign(pi:toZUP(pendOri*COM:sampleVec(ii, 3)))

		desiredCOM(i):assign(pi:toZUP_pos(COM:sampleVec(ii, 0)+vector3(0,COMheight_offset,0)))

		--RE.output2('penori',pendOri.y)
		local ang=pi:toZUP_ori(pendOri)
		local angvel=pi:toZUP(pendOri*ANGVEL:sampleVec(ii, 0))
		local euler=planner:toEuler(ang)
		local eulerRate=planner:getEulerRate(euler, angvel)
		desiredEuler(i):assign(euler)
		desiredDotEuler(i):assign(eulerRate)
	end
	print(desiredCOM)

	return {COM, ZMP, ANG, ANGVEL}, {desiredCOM, desiredDotCOM, desiredEuler, desiredDotEuler}, t
end
function PlannerInfo:dotCOMtrajToEulerZYX(initialTF, dotRoot, t, totalTime, debug_draw)
	local pi=self
	local planner=self.planner
	local pendFrameRate=30


	local tf=initialTF:copy()
	local totalFrame0=dotRoot:rows()-1

	local COM=matrixn(totalFrame0+1,3)
	local ZMP=matrixn(totalFrame0+1,3)
	local ANG=matrixn(totalFrame0+1,4)
	local ANGVEL=matrixn(totalFrame0+1,3)

	local _desiredCOM=vector3N(totalFrame0+1)
	local _desiredDotCOM=vector3N(totalFrame0+1)
	local _desiredEuler=vector3N(totalFrame0+1)
	local _desiredDotEuler=vector3N(totalFrame0+1)

	local ang=pi:toZUP_ori(tf.rotation)
	local euler=planner:toEuler(ang)
	_desiredEuler(0):assign(euler)
	_desiredCOM(0):assign(pi:toZUP_pos(tf.translation))

	COM:row(0):setVec3(0, tf.translation)
	ANG:row(0):setQuater(0, tf.rotation)

	local dt=1/30
	for i=1, totalFrame0 do
		local dpose=dotRoot:row(i-1)
		tf:integrateBodyVel(dpose:toVector3(4), dpose:toVector3(0), 1/30)
		_desiredCOM(i):assign(pi:toZUP_pos(tf.translation))
		ANGVEL:row(i-1):setVec3(0, tf.rotation*dpose:toVector3(4))
		local angvel=pi:toZUP(ANGVEL:row(i-1):toVector3(0))
		local eulerRate=planner:getEulerRate(euler, angvel)

		_desiredEuler(i):assign(_desiredEuler(i-1)+eulerRate*dt)
		_desiredDotEuler(i-1):assign(eulerRate)
		_desiredDotCOM(i-1):assign(pi:toZUP(dpose:toVector3(0)))

		local temp =planner:toQuater(_desiredEuler(i))
		tf.rotation:assign(pi:toYUP_ori(temp))
		dbg.draw('Axes', tf, 'axes_tf'..i,100, 1)
		COM:row(i):setVec3(0, tf.translation)
		ANG:row(i):setQuater(0, tf.rotation)
	end
	ANGVEL:row(totalFrame0):assign(ANGVEL:row(totalFrame0-1))
	_desiredDotEuler:row(totalFrame0):assign(_desiredDotEuler:row(totalFrame0-1))
	_desiredDotCOM:row(totalFrame0):assign(_desiredDotCOM:row(totalFrame0-1))

	local ZMP=COM:copy()
	ZMP:column(1):setAllValue(0)

	local desiredCOM=vector3N(t:size())
	local desiredDotCOM=vector3N(t:size())
	local desiredEuler=vector3N(t:size())
	local desiredDotEuler=vector3N(t:size())

	self.t=t
	--self.COMheight_offset=vectorn(t:size()) -- only for drawing
	local COMheight_offset=0

	local tf=initialTF:copy()

	for i=0, t:size()-1 do
		local ii=t(i)*pendFrameRate
		desiredCOM(i):assign(_desiredCOM:matView():sampleVec(ii,0))
		desiredDotCOM(i):assign(_desiredDotCOM:matView():sampleVec(ii,0))
		desiredEuler(i):assign(_desiredEuler:matView():sampleVec(ii,0))
		desiredDotEuler(i):assign(_desiredDotEuler:matView():sampleVec(ii,0))
	end

	return {COM, ZMP, ANG, ANGVEL}, {desiredCOM, desiredDotCOM, desiredEuler, desiredDotEuler, _desiredEuler}, t
end

function PlannerInfo:drawFrame(iframe, boxpos_offset)
	local t=iframe/framerate

	local planner=self.planner
	local rootpos=planner:getBasePos(t)
	local rootori=planner:getBaseOri(t)
	local theta=vectorn()
	local dtheta=vectorn()
	planner:getTheta(t, theta)
	planner:getDTheta(t, dtheta)
	--print(dtheta)

	--dbg.eraseAllDrawn()
	for ifoot=0,self.nEE-1 do
		dbg.erase('Sphere', "ballf"..ifoot)
		dbg.erase('Arrow',  'arrow'..ifoot)
	end
	boxpos_offset=boxpos_offset or vector3(0,0,0)

	if true then
		-- draw inertia box
		--dbg.draw('Sphere', self:toYUP_pos(rootpos)*100, "ballr", "green", 15)
		dbg.draw('Arrow', (self:toYUP_pos(rootpos)+boxpos_offset)*100, (self:toYUP_pos(rootpos)+boxpos_offset)*100+self:toYUP(dtheta:toVector3(0))*25, 'arrow_com')
		--dbg.draw('Axes', transf(self:toYUP_ori(rootori), (self:toYUP_pos(rootpos)+vector3(1,0,0))), 'axes',100)


		local inertia=g_inertia
		local size=VRMLexporter.boxSizeFromInertia(60, inertia.x, inertia.y, inertia.z)
		dbg.draw('Box', transf(self:toYUP_ori(rootori), self:toYUP_pos(rootpos)+boxpos_offset), 'bodyInertia',toYUP0(size), 100, 'lightgrey_verytransparent')
	end

	local nEE=self.nEE

	for ifoot=0,nEE-1 do
		local foot=planner:getFootPos(ifoot, t)
		local cf=planner:getContactForce(ifoot, t)
		local contact=planner:getContact(ifoot, t)

		local color='red'
		if ifoot>=self.nEE_perLimb then color='green'
		end
		if contact then 
			--dbg.draw('Sphere', self:toYUP_pos(foot)*100, "ballf"..ifoot, color, 5)
			dbg.draw('Arrow', self:toYUP_pos(foot)*100, self:toYUP_pos(foot)*100+self:toYUP(cf)*0.15, 'arrow'..ifoot, 5, color)
			dbg.draw('Sphere', self:toYUP_pos(foot)*100, "foot"..ifoot, color, 5)
		end
	end
	if boolean_options.attachCamera then
		local curPos= self:toYUP_pos(rootpos)*100
		curPos.y=0
		if g_prevRootPos then
			RE.viewpoint().vpos:assign(RE.viewpoint().vpos+curPos-g_prevRootPos)
			RE.viewpoint().vat:assign(RE.viewpoint().vat+curPos-g_prevRootPos)
			RE.viewpoint():update()     
		end
		g_prevRootPos=curPos:copy()
	end
end

function mergeDur(totalTime, dur1, dur2)
	local cur_t=0
	local cur1=dur1[1]
	local cur2=dur2[1]
	local key1=dur1[2]
	local key2=dur2[2]
	local curi1=0
	local curi2=0
	local start=cur1 or cur2
	local cur=start
	local key=vectorn()

	while true do
		local t1=key1:range(0,curi1+1):sum()
		local t2=key2:range(0,curi2+1):sum()
		if t1< t2 then
			cur_t=t1
			curi1=curi1+1
			cur1=not cur1
		else
			cur_t=t2 
			curi2=curi2+1
			cur2=not cur2
		end
		if curi1==key1:size() or curi2==key2:size() then
			key:pushBack(totalTime)
			break
		end
		if (cur1 or cur2) ~=cur then
			key:pushBack(cur_t)
			cur=not cur
		end
	end
	for i=key:size()-1,1, -1 do
		key:set(i, key(i)-key(i-1))
	end

	return {start, key}
end
function showPoses(planner, skip, startFrame)
	if not startFrame then
		startFrame=0
	end
	skip=skip or 10
	skins={}
	for i=startFrame, g_history.dtheta:rows()-1 do
		planner:drawFrame(i)
		if math.fmod(i,skip)==0 then
			local pose=Pose()
			mMot.skin:getPose(pose)

			local skin=fc.createSkin(mMot.loader, config.skinScale)
			PLDPrimSkin.setPose(skin,pose,mMot.loader)
			table.insert(skins, skin)
		end
	end
end
function showPoses2(planner, frames)
	skins={}
	for i=0, frames[#frames] do
		print(i)
		planner:drawFrame(i)
		for j, jj in ipairs(frames) do
			if jj==i then
				local pose=Pose()
				mMot.skin:getPose(pose)

				local skin=fc.createSkin(mMot.loader, config.skinScale)
				PLDPrimSkin.setPose(skin,pose,mMot.loader)
				--skin:setMaterial('lightgrey_verytransparent')
				table.insert(skins, skin)
				break
			end
		end
	end
end
