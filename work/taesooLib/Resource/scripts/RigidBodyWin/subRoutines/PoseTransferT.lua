local PoseTransferT=LUAclass()



-- unlike PoseTransfer2 PoseTransferT can adjust bone lengths.
--param
-- loaderA : source pose
-- loaderB : target pose
-- convInfoA (optional) : bone names of A (can be a subset)
-- convInfoB (optional) : the corresponding bone names of B
-- posScaleFactor : skinScaleB/skinScaleA
--
-- usage:
--        1. setPose loaderA and loaderB so that both loaders are at the same pose. 
--        (The same pose can be expressed using different quaternions though)
--
-- 	      PT=PoseTransferT(loaderA, loaderB)
--        PT:setTargetSkeleton(poseA)
--        local poseB=Pose()
--        loaderB:getPose(poseB) -- now poseB (quaternions and positions) obtained
--
--        or 
--
--        local posedofB=vectorn()
--        loaderB:getPoseDOF(posedofB) -- now poseB (DOFs) obtained
--
function PoseTransferT:__init(loaderA, loaderB, convInfoA, convInfoB, posScaleFactor)
	self.loaderA=loaderA;
	self.loaderB=loaderB;
	self.targetIndexAtoB=intvectorn(self.loaderA:numBone());
	self.targetIndexAtoB:setAllValue(-1);
	local BtoA=intvectorn(self.loaderB:numBone());
	BtoA:setAllValue(-1);

	if not convInfoA then
		convInfoA=TStrings()
		convInfoA:resize(loaderA:numBone()-1)
		convInfoB=TStrings()
		convInfoB:resize(loaderA:numBone()-1)
		for i=1, loaderA:numBone()-1 do
			convInfoA:set(i-1, loaderA:bone(i):name())
			convInfoB:set(i-1, loaderA:bone(i):name())
		end
	end

	self.rAtoB_additionalAindices=intvectorn()
	self.rAtoB_additionalBindices=intvectorn()
	self.rAtoB_additional=vector(0, transf)

	for i=0, convInfoA:size()-1 do
		local iboneA=self.loaderA:getTreeIndexByName(convInfoA(i));
		local iboneB=self.loaderB:getTreeIndexByName(convInfoB(i));
		assert(iboneA~=-1);
		assert(iboneB~=-1);
		if self.targetIndexAtoB(iboneA)==-1 then
			self.targetIndexAtoB:set(iboneA, iboneB);
			BtoA:set(iboneB, iboneA);
		else
			-- one to many mapping
			self.rAtoB_additionalAindices:pushBack(iboneA)
			self.rAtoB_additionalBindices:pushBack(iboneB)
			self.rAtoB_additional:pushBack(transf(quater(1,0,0,0), vector3(0,0,0)))
		end
	end
	self.BtoA=BtoA
	self.parentIdx=intvectorn(self.loaderB:numBone())
	self.parentIdx:setAllValue(-1)
	for i=2, self.parentIdx:size()-1 do
		if BtoA(i)==-1 then
			local j=self.loaderB:bone(i):parent():treeIndex()
			--print('parentIdx:', self.loaderB:bone(i):name(), i)
			local list={i}
			while true do
				if BtoA(j)~=-1 then
					for ii,vv in ipairs(list) do
						self.parentIdx:set(vv,j)
						assert(BtoA(vv)==-1)
					end
					--print('done:', self.loaderA:bone(BtoA(j)):name())
					break
				end
				if j==0 then break end
				table.insert(list, j)
				--print(j)
				j=self.loaderB:bone(j):parent():treeIndex()
			end
		end
	end
	--local pIdx=self.parentIdx
	--for i=1, loaderB:numBone()-1 do
	--	print(i, loaderB:bone(i):name(), loaderB:bone(i):parent():treeIndex(), BtoA(i), pIdx(i))
	--end

	local AtoB=self.targetIndexAtoB
	self.bindPoseA=Pose();
	self.bindPoseB=Pose();
	self.loaderA:getPose(self.bindPoseA);
	self.loaderB:getPose(self.bindPoseB);
	self.rAtoB=vector(self.loaderA:numBone(), transf)
	self.rAtoB_missing=vector(self.loaderB:numBone(), transf) -- B-bones missing in A



	--
	--/////////////////////////////////////////////////////////////////////////////////////////
	--// Bind pose는 임의의 포즈가 될수 있다. 

	--// 현재 오우거 노드가 부모로부터 3번째 노드라고 하면.
	--//  노드의 global orientation CN2=LN0*LN1*LN2 (B의 현재 자세)
	--//  rB가 모델의 binding pose에서의 global orientation이라 하자.
	--
	-- 목표:
	-- binding pose를 SetPose함수에 입력한 경우, CN2가 rB가 되도록 해야한다.
	-- rA 가 A의 동작 데이타 바인딩 포즈의 combined mat이라 하자.
	-- CN2 = C2A * rA.inv * rB 가 되면 된다.
	-- (즉 rA와 C2A(SetPose함수의 입력자세)가 동일한 경우, CN2==rB)
	for i=1, self.loaderA:numBone()-1 do
		if AtoB(i)~=-1 then
			local rA=self.loaderA:bone(i):getFrame()
			local rB=self.loaderB:bone(AtoB(i)):getFrame()
			--           rB*rA:inverse() 
			self.rAtoB(i):assign(rA:inverse()*rB)
		end
	end


	local pIdx=self.parentIdx
	for i=1, loaderB:numBone()-1 do
		if pIdx(i)~=-1 then
			assert(BtoA(pIdx(i))~=-1)
			local iA= BtoA(pIdx(i))
			local rA=self.loaderA:bone(iA):getFrame()
			local rB=self.loaderB:bone(i):getFrame()
			self.rAtoB_missing(i):assign(rA:inverse()*rB)
		end
	end

	for i=0, self.rAtoB_additionalAindices:size()-1 do
		local iA=self.rAtoB_additionalAindices(i)
		local iB=self.rAtoB_additionalBindices(i)
		local rA=self.loaderA:bone(iA):getFrame()
		local rB=self.loaderB:bone(iB):getFrame()
		self.rAtoB_additional(i):assign(rA:inverse()*rB)
	end

	self.posScaleFactor=posScaleFactor or 1
	local Aroot=self.loaderA:bone(1):getFrame():copy()
	Aroot.translation:scale(1/self.posScaleFactor)
	self.rootAtoB=Aroot:inverse()*self.loaderB:bone(1):getFrame()
end

function PoseTransferT:convertPose(poseA)
	self:setTargetSkeleton(poseA)
	return self.loaderB:pose()
end
function PoseTransferT:setTargetSkeleton(poseA)
	if dbg.lunaType(poseA)=='Pose' then
		self.loaderA:setPose(poseA)
	else
		self.loaderA:setPoseDOF(poseA)
	end
	local loaderA=self.loaderA
	local loaderB=self.loaderB
	local AtoB=self.targetIndexAtoB
	local rAtoB=self.rAtoB
	for i=1, loaderA:numBone()-1 do
		if AtoB(i)~=-1 then
			loaderB:bone(AtoB(i)):getFrame():assign(loaderA:bone(i):getFrame()*rAtoB(i))
		end
	end
	local rootB=AtoB(1)
	assert(rootB~=-1)
	local BtoA=self.BtoA
	--[[
	for i=rootB+1, loaderB:numBone()-1 do
		-- fill-in missing internal joints
		if BtoA(i)~=-1 then
			local q=loaderB:bone(i):getFrame().rotation
			local j=self.loaderB:bone(i):parent():treeIndex()
			while true do
				if BtoA(j)~=-1 then
					break
				end
				self.loaderB:bone(j):getFrame().rotation:assign(q)
				j=self.loaderB:bone(j):parent():treeIndex()
				assert(j>0) --  루트 조인트의 correspondence가 설정이 안되어 있으면 여기서 오류남.
			end
		end
	end
	]]
	--print('before:')
	--print(self.loaderB:bone(24):getFrame().rotation)
	--print(self.loaderB:bone(54):getFrame().rotation)
	local pIdx=self.parentIdx
	for i=rootB+1, loaderB:numBone()-1 do
		-- fill-in missing leaf joints
		if pIdx(i)~=-1 then
			assert(BtoA(pIdx(i))~=-1)
			--self.loaderB:bone(i):getFrame().rotation:assign(self.loaderB:bone(pIdx(i)):getFrame().rotation)
			local iA= BtoA(pIdx(i))
			loaderB:bone(i):getFrame():assign(loaderA:bone(iA):getFrame()*self.rAtoB_missing(i))
		end
	end


	for i=0, self.rAtoB_additionalAindices:size()-1 do
		local iA=self.rAtoB_additionalAindices(i)
		local iB=self.rAtoB_additionalBindices(i)
		loaderB:bone(iB):getFrame():assign(loaderA:bone(iA):getFrame()*self.rAtoB_additional(i))
	end

	-- set root
	loaderB:bone(1):getFrame():assign(loaderA:bone(1):getFrame()*self.rootAtoB)

	-- an inverse kinematics .
	-- Gi=Gi-1*Li (forward kinematics)
	
end
return PoseTransferT
