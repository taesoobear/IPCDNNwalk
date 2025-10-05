
-- removes discontinuity (vectorn)
DiscontinuityRemoverLinear=LUAclass()

function DiscontinuityRemoverLinear:__init(windowSize)
	self.windowSize=windowSize
	self.currentDelta=nil
end

function DiscontinuityRemoverLinear:startBlending(poseA, poseB)
	self.currentDelta={0, poseA-poseB}

	if self.zeroDelta then
		local a,b=unpack(self.zeroDelta)
		self.currentDelta[2]:slice(a,b):zero()
	end
end

function DiscontinuityRemoverLinear:update(pose)
	local delta=self.currentDelta
	if delta then
		local t=delta[1]
		local D=delta[2]* sop.smoothMap(t, 0, self.windowSize, 1, 0)
		pose:assign(pose+D)

		if t>self.windowSize then
			self.currentDelta=nil
		else
			delta[1]=t+1
		end
		return D
	end
end


DiscontinuityRemoverQuintic=LUAclass()

function DiscontinuityRemoverQuintic:__init(windowSize)
	self.windowSize=windowSize
	self.currentDelta=nil
end
function DiscontinuityRemoverQuintic:startBlending(poseA, poseB, velA, velB)
	local delta=matrixn(self.windowSize, poseA:size())
	self.currentDelta={0, delta}
	local x0=poseA-poseB
	local v0=velA-velB
	for i=0, poseA:size()-1 do
		delta:column(i):quintic(x0(i),v0(i),0,      
		                        0,    0,    0,  
								delta:rows()+1)
	end
end

function DiscontinuityRemoverQuintic:update(pose)
	local delta=self.currentDelta
	if delta then
		local t=delta[1]
		if t<self.windowSize then
			local D=delta[2]:row(t)
			pose:assign(pose+D)
			delta[1]=t+1
		else
			self.currentDelta=nil
		end
		return D
	end
end

DiscontinuityRemoverQuinticHybrid=LUAclass()

function DiscontinuityRemoverQuinticHybrid:__init(windowSizeQuintic, windowSizeLinear)
	self.delta1=DiscontinuityRemoverQuintic(windowSizeQuintic)
	self.delta2=DiscontinuityRemoverLinear(windowSizeLinear)
end
function DiscontinuityRemoverQuinticHybrid:startBlending(poseA, poseB, velA, velB)
	self.delta1:startBlending(poseA, poseA, velA, velB) 
	self.delta2:startBlending(poseA, poseB)
end

function DiscontinuityRemoverQuinticHybrid:update(pose)
	self.delta2:update(pose) -- remove positional discontinuity
	self.delta1:update(pose) -- remove velocity discontinuity
end

