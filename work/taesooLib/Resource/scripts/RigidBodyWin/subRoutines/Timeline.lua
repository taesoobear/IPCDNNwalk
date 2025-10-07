--	example usage:
--	require('subRoutines/Timeline')
--	function ctor()
--	mEventReceiver=EVR()
--	mTimeline=Timeline("Timeline", 10000, 1/30)
--	end
--  function EVR:onFrameChanged(win, iframe)
--  end
--
--
-- an example is in testScrollPanel.lua
Timeline=LUAclass(LuaAnimationObject)
function Timeline:__init(label, numframes, frametime)
	self.totalTime=numframes
	self:attachTimer(frametime or 1/60, numframes)		
	RE.renderer():addFrameMoveObject(self)
	RE.motionPanel():motionWin():addSkin(self)
end
-- has to be manually called
-- mTimeline:dtor() mTimeline=nil
function Timeline:dtor()
	RE.motionPanel():motionWin():changeCurrFrame(0)
	RE.renderer():removeFrameMoveObject(self)
	RE.motionPanel():motionWin():detachSkin(self)
end
function Timeline:numFrames()
	return self.totalTime
end

function Timeline:reset(numframes, frameTime)
	RE.motionPanel():motionWin():changeCurrFrame(0)
	RE.renderer():removeFrameMoveObject(self)
	RE.motionPanel():motionWin():detachSkin(self)
	self.totalTime=numframes
	self:attachTimer(frameTime or 1/60, numframes)		
	RE.renderer():addFrameMoveObject(self)
	RE.motionPanel():motionWin():addSkin(self)
end

if EventReceiver then
	--class 'EVR'(EventReceiver)
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		--EventReceiver.__init(self)
		self.currFrame=0
		self.cameraInfo={}
	end

	function EVR:attachCameraToMotion(mLoader, mMotionDOF, discontinuity)
		if mLoader~=nill then

			self.trajectory=matrixn(mMotionDOF:numFrames(),3)

			self.trajectoryOri=matrixn(mMotionDOF:numFrames(),4)
			if discontinuity then
				local discont=discontinuity
				local segFinder=SegmentFinder(discont)

				for i=0, segFinder:numSegment()-1 do
					local s=segFinder:startFrame(i)
					local e=segFinder:endFrame(i)

					for f=s,e-1 do
						self.trajectory:row(f):setVec3(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).translation)
						self.trajectory:row(f):set(1,0)
						self.trajectoryOri:row(f):setQuater(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).rotation:rotationY())
					end
					print("filtering",s,e)
					math.filter(self.trajectory:range(s,e,0, 3), 63)
					math.filter(self.trajectoryOri:range(s,e,0, 4), 63)
				end
			else
				if dbg.lunaType(mMotionDOF)=='Motion' or mMotionDOF.mot then
					return self:_attachCameraToMotion(mLoader, mMotionDOF)
				end

				local s=0
				local e=mMotionDOF:numFrames()

				for f=s,e-1 do
					self.trajectory:row(f):setVec3(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).translation)
					self.trajectory:row(f):set(1,0)
					self.trajectoryOri:row(f):setQuater(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).rotation:rotationY())
				end
				print("filtering",s,e)
				math.filter(self.trajectory:range(s,e,0, 3), 63)
				math.filter(self.trajectoryOri:range(s,e,0, 4), 63)
			end

			local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
			self.cameraInfo.vpos=RE.viewpoint().vpos-curPos
			self.cameraInfo.vat=RE.viewpoint().vat-curPos
			self.cameraInfo.dist=RE.viewpoint().vpos:distance(curPos)
			self.cameraInfo.refRot=self.trajectoryOri:row(self.currFrame):toQuater(0):rotationY()
		end
	end
	function EVR:_attachCameraToMotion(mLoader, mMotionDOFcontainer)
		-- actually work for both motion and motiondofcontainer
		if mLoader~=nill then

			local discont
			local numFrames
			local getRoot=function (mMotionDOFcontainer, f)
				return mMotionDOFcontainer.mot:row(f):toTransf(0)
			end

			if mMotionDOFcontainer.mot then
				discont=mMotionDOFcontainer.discontinuity
				numFrames=mMotionDOFcontainer:numFrames()
			else
				assert(lunaType(mMotionDOFcontainer)=='Motion')
				local mot=mMotionDOFcontainer
				discont=mot:getConstraints(Motion.IS_DISCONTINUOUS)
				numFrames=mot:numFrames()
				getRoot=function (mot, f)
					local out=transf()
					out.translation:assign(mot:pose(f).translations(0))
					out.rotation:assign(mot:pose(f).rotations(0))
					return out
				end
			end

			self.trajectory=matrixn(numFrames,3)
			self.trajectoryOri=matrixn(numFrames,4)

			local segFinder=SegmentFinder(discont)

			for i=0, segFinder:numSegment()-1 do
				local s=segFinder:startFrame(i)
				local e=segFinder:endFrame(i)

				for f=s,e-1 do
					local tf=getRoot(mMotionDOFcontainer, f)
					self.trajectory:row(f):setVec3(0, tf.translation)
					self.trajectory:row(f):set(1,0)
					self.trajectoryOri:row(f):setQuater(0, tf.rotation)
				end
				print("filtering",s,e)
				math.filter(self.trajectory:range(s,e,0, 3), 63)
				math.filter(self.trajectoryOri:range(s,e,0, 4), 63)
			end

			local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
			self.cameraInfo.vpos=RE.viewpoint().vpos-curPos
			self.cameraInfo.vat=RE.viewpoint().vat-curPos
			self.cameraInfo.dist=RE.viewpoint().vpos:distance(curPos)
			self.cameraInfo.refRot=self.trajectoryOri:row(self.currFrame):toQuater(0):rotationY()
		end

	end
	function EVR:detachCamera()
		self.trajectory=nil
	end
	function EVR:moveCamera(iframe)

		if mEventReceiver.trajectory and iframe< mEventReceiver.trajectory:rows() then
			local curPos=mEventReceiver.trajectory:row(iframe):toVector3(0)*100
			local curDir=RE.viewpoint().vat-RE.viewpoint().vpos
			RE.viewpoint().vpos:assign(mEventReceiver.cameraInfo.vat-curDir+curPos)
			RE.viewpoint().vat:assign(mEventReceiver.cameraInfo.vat+curPos)
			RE.viewpoint():update()     
		end
	end

end
