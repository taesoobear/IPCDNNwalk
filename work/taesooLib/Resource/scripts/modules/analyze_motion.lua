
require("config")
require("module")
require("common")

debugVis=true 

function ctor()

	mEventReceiver=EVR()

	this:create("Button", "draw COM traj", "draw COM traj")
	this:create("Button", "attach camera", "attach camera")
	this:widget(0):buttonShortcut("FL_ALT+c")

	this:create("Check_Button", "draw skeleton", "draw skeleton")
	this:widget(0):checkButtonValue(0)

	this:updateLayout()
	this:redraw()

	mObjectList=Ogre.ObjectList()

	do -- set viewpoint
		RE.viewpoint():setFOVy(44.999999)
		local vpos=vector3(94.777964, 126.724047, 352.393547)
		local vat=vector3(-34.317428, 67.508947, -4.622992)
		RE.viewpoint().vpos:assign(vat+(vpos-vat)*1.5)
		RE.viewpoint().vat:assign(vat)
		RE.viewpoint():update()
		RE.renderer():fixedTimeStep(false)   
	end
	_start("../Resource/motion/justin_straight_run/justin_straight_run.wrl")
	_applyMotion("../Resource/motion/justin_straight_run/justin_straight_run.dof")
end


function onCallback(w, userData)
	if w:id()=="draw COM traj" then
		do -- test
			local comtraj=calcCOMtrajectory(mLoader, mMotionDOFcontainer.mot, mMotionDOFcontainer.discontinuity)
			mObjectList:registerObject("comtraj", "LineList", "solidred", comtraj:matView()*100,0)
		end
	elseif w:id()=="attach camera" then
		mEventReceiver:attachCamera()
	end
end


function dtor()
	dbg.finalize()
	detachSkins()
end
if EventReceiver then
	--class 'EVR'(EventReceiver)
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		--EventReceiver.__init(self)
		self.currFrame=0
		self.cameraInfo={}
	end
end

function EVR:onFrameChanged(win, iframe)
	self.currFrame=iframe
	if self.trajectory then
		if self.currFrame<self.trajectory:rows() then
			local mMotionDOF=mMotionDOFcontainer.mot
			local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
			local pPos=
			MotionDOF.rootTransformation(mMotionDOF:row(self.currFrame)).translation
			local currRot=
			self.trajectoryOri:row(self.currFrame):toQuater(0):rotationY()


			dbg.draw('Line', pPos*100, pPos*100+rotate(vector3(0,0,100), currRot), 'prot')

			do
				RE.viewpoint().vpos:assign(self.cameraInfo.vpos+curPos)
				RE.viewpoint().vat:assign(self.cameraInfo.vat+curPos)
				RE.viewpoint():update()     
			end
		end
	end
end

function EVR:attachCamera()

	if mLoader~=nill then

		local discont=mMotionDOFcontainer.discontinuity
		local mMotionDOF=mMotionDOFcontainer.mot

		self.trajectory=matrixn(mMotionDOFcontainer:numFrames(),3)

		self.trajectoryOri=matrixn(mMotionDOFcontainer:numFrames(),4)
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

		local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
		self.cameraInfo.vpos=RE.viewpoint().vpos-curPos
		self.cameraInfo.vat=RE.viewpoint().vat-curPos
		self.cameraInfo.dist=RE.viewpoint().vpos:distance(curPos)
		self.cameraInfo.refRot=self.trajectoryOri:row(self.currFrame):toQuater(0):rotationY()
	end
end

function frameMove(fElapsedTime)
end

function detachSkins()
	if RE.motionPanelValid() then
		if mSkin then
			RE.motionPanel():motionWin():detachSkin(mSkin)
			mSkin=nil
		end
	end
	-- remove objects that are owned by LUA
	collectgarbage()
end
function _start(chosenFile)
	detachSkins()
	print(chosenFile.."\n")
	mLoader=MainLib.VRMLloader(chosenFile)
	drawSkeleton=this:findWidget("draw skeleton"):checkButtonValue()
	mSkin=RE.createVRMLskin(mLoader, drawSkeleton)
	mSkin:setThickness(0.03)
	mSkin:scale(100,100,100)
end

function _applyMotion(chosenFile)
	if mLoader~=nill then

		_G.chosenMotFile=chosenFile
		mMotionDOFcontainer=MotionDOFcontainer(mLoader.dofInfo,chosenFile)
		mSkin:applyMotionDOF(mMotionDOFcontainer.mot)
		RE.motionPanel():motionWin():detachSkin(mSkin)
		RE.motionPanel():motionWin():addSkin(mSkin)
	end
end
