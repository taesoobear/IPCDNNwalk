
CDMphaseFilter=LUAclass()
function CDMphaseFilter:__init(delay, refFrameToPhase)
	self.CDMstates=matrixn()
	self.refFrameToPhase=refFrameToPhase
	self.phases=vectorn()
	self.delay=delay
	self.windowSize=delay*2+1
end
CDMphaseFilter.default_delay=4

function CDMphaseFilter:setState(CDM, CDMdq, LrefCoord, RrefCoord, Lcon, Rcon, ref_phase)
	local ii=self.refFrameToPhase:sample(ref_phase)

	self.phases:pushBack(ii)
	local cdmstate=vectorn(21+6+2)
	cdmstate:setTransf(0, CDM)
	cdmstate:setTransf(7, LrefCoord)
	cdmstate:setTransf(14, RrefCoord)
	local c=14+7
	cdmstate:slice(c, c+6):assign(CDMdq) c=c+6
	local function bton(con)
		if con then 
			return 1 
		end
		return 0
	end
	cdmstate:set(c, bton(Lcon))
	cdmstate:set(c+1, bton(Rcon))
	self.CDMstates:pushBack(cdmstate)

	assert(self.CDMstates:rows()==self.phases:size())
	local windowSize=self.windowSize
	if self.CDMstates:rows()>windowSize then
		self.CDMstates=self.CDMstates:sub(-windowSize,0,0,0):copy()
		self.phases=self.phases:slice(-windowSize,0):copy()
	end
	self:_updatePhase()
end

function CDMphaseFilter:_updatePhase()
	if self.phases:size()>=self.windowSize then
		assert(self.phases:size()==self.windowSize)
		local center=math.floor(self.windowSize/2)

		local maxPhase=self.refFrameToPhase.timing:size()-1
		local aligned=self.phases:copy()
		local delta=0
		for i=1, aligned:size()-1 do
			if aligned(i)<aligned(i-1) then
				delta=maxPhase
			end
			aligned:set(i, aligned(i)+delta)
		end
		local centerPhase=math.filterSingle(aligned:column(), aligned:size())(0)
		centerPhase=centerPhase-1.0

		if centerPhase>maxPhase then
			centerPhase=centerPhase-maxPhase
		end
		self.filteredPhase=centerPhase
		self.centerFrame=center
	else
		local center=math.floor(self.windowSize/2)
		self.centerFrame=math.max(0, self.phases:size()-1-center)
		self.filteredPhase=self.phases(self.centerFrame)
	end
end

function CDMphaseFilter:_unpackState(phase, cdmstate)
	return cdmstate:toTransf(0), cdmstate:toTransf(7), cdmstate:toTransf(14), phase
end

function CDMphaseFilter:getState()
	local phase, cdmstate
	return self:_unpackState(self.filteredPhase, self.CDMstates:row(self.centerFrame))
end

function CDMphaseFilter:getPhase()
	return self.filteredPhase
end
function CDMphaseFilter:getCDMbodyState()
	local cdmstate=self.CDMstates:row(self.centerFrame)
	local c=14+7
	return cdmstate:toTransf(0), cdmstate:slice(c, c+6)
end
function CDMphaseFilter:getFootPos(ilimb)
	if ilimb==0 then
		ilimb=2
	end
	local function ntob(con)
		if con ==1 then 
			return true 
		end
		return false
	end
	local c=14+7+6
	local cstate=self.CDMstates:row(self.centerFrame)
	if ilimb==1 then
		return cstate:toTransf(7), ntob(cstate(c))
	else
		return cstate:toTransf(14), ntob(cstate(c+1))
	end
end
function CDMphaseFilter:getCon(ilimb)
	if ilimb==0 then
		ilimb=2
	end
	local function ntob(con)
		if con ==1 then 
			return true 
		end
		return false
	end
	local c=14+7+6
	local cstate=self.CDMstates:row(self.centerFrame)
	if ilimb==1 then
		return ntob(cstate(c))
	else
		return  ntob(cstate(c+1))
	end
end
