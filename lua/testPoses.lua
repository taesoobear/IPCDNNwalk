require("config")

package.projectPath='../Samples/classification/'
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
require("common")
require("module")
require("subRoutines/Timeline")

numCharacter=5
BOX_SIZE=vector3(10,0,10)
function ctor()

	mEventReceiver=EVR()
	mTimeline=Timeline("Timeline", 150)
	this:updateLayout()

	mLoader=MainLib.VRMLloader (
	"work/taesooLib/Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_boxfoot.wrl") 

	mMotionDOFcontainer=MotionDOFcontainer(mLoader.dofInfo, "work/taesooLib/Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof")
	mMotionDOF=mMotionDOFcontainer.mot

	-- translate the motion 7cm up
	for i=0, mMotionDOF:rows()-1 do
		mMotionDOF:matView():set(i, 1, mMotionDOF:matView()(i,1)+0.07)
	end


	mSkins={}
	mSkinInfo={}
	for i=0, numCharacter-1 do
		mSkins[i]= RE.createVRMLskin(mLoader, false);	-- to create character
		mSkins[i]:scale(100,100,100);					-- motion data is in meter unit while visualization uses cm unit.
		local startFrame=math.round(math.random()*200)+100

		local pose=mMotionDOF:row(startFrame):copy()
		local rootpos=pose:toVector3(0)
		local rootoffset=vector3(math.random()*BOX_SIZE.x, 0, math.random()*BOX_SIZE.z)-BOX_SIZE*0.5
		mSkinInfo[i]={startFrame, rootoffset}
	end

	collisionAvoid(0, skinInfo)
	drawPoses(0)

	this:updateLayout()
end
function collisionAvoid(iframe)

	local THR=0.8
	local MIN_MOVE=0.1

	local function rootPos(iframe, i)
		local startFrame, rootoffset=unpack(mSkinInfo[i])
		return mMotionDOF:row(startFrame+iframe):toVector3(0)+rootoffset
	end

	for iter=1,1000 do
		local maxDist=0
		for i=0, numCharacter-1 do
			for j=i+1, numCharacter-1 do
				local ri=rootPos(iframe, i)
				local rj=rootPos(iframe, j)
				if ri:distance(rj)<THR then
					local dir=rj-ri
					local dist=dir:length()
					if dist>MIN_MOVE then
						dir:normalize()
						dir:scale(MIN_MOVE)
					end
					if dist>maxDist then
						maxDist=dist
					end
					dir.y=0
					mSkinInfo[i][2]:radd(-dir)
					mSkinInfo[j][2]:radd(dir)
				end
			end
		end
		if maxDist==0 then break end
	end
end
function drawPoses(iframe)
	for i=0, numCharacter-1 do
		local startFrame, rootoffset=unpack(mSkinInfo[i])
		local pose=mMotionDOF:row(startFrame+iframe):copy()
		pose:setVec3(0, pose:toVector3(0)+rootoffset)
		mSkins[i]:setPoseDOF(pose);

		mLoader:setPoseDOF(pose)
		pose=mLoader:pose()
		mSkins[i]:setPose(pose);
	end
end

function dtor()
	RE.motionPanel():motionWin():detachSkin(mSkin)
end

function handleRendererEvent(ev, button, x, y)
	return 0
end

function onCallback(w, userData)
	if w:id()=='attach motion to UI' then
	elseif w:id()=='start game loop' then
	end
end

			
function frameMove(fElapsedTime)
end

function EVR:onFrameChanged(win, iframe)
	collisionAvoid(iframe, skinInfo)
	drawPoses(iframe)
end
