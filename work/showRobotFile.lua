-- run using tp or tl
package.projectPath='../Samples/classification/'
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/sample_luatorch/lua/?.lua" --;"..package.path

--local uiscale=1.5
--createMainWin(((1024+180)*uiscale),((600+100)*uiscale), (1024*uiscale), (600*uiscale),uiscale)

require('RigidBodyWin/subRoutines/MujocoLoader')
require('RigidBodyWin/subRoutines/URDFloader')

--scriptPath=util.getScriptPath()
scriptPath='.'

robot_files={
	{ title='go2_urdf', scriptPath.."/go2/urdf/go2.urdf" },
	{ title='go2_mjcf', scriptPath.."/go2/go2.mjcf" },
	{ title='deepmimic', '../Resource/mujoco/humanoid_deepmimic_Tpose.xml', posedof=CT.vec(  1.90658036, 1.8824253 , 0.14074333, -0.18771314, 0.64872936, 0.00982938, -0.73743975, -1.16603354, -0.01891335, 0.68193487, 0.04290145, 0.05784741, -0.04927042, -1.57000184, -1.50010476, 0.02102934, 0.36392316, -1.40035254, -0.29961699, -1.65902603, -2.11852643, 1.20051112, -2.57052383, 1.00013403, -2.70118472, -0.800849 , -0.76477599, -0.97184688, -0.48210675, -0.16290015, 1.0000922 , -2.70021054, 1.00000127, -1.00002148, 0.13190098)},
	{ title='panda', scriptPath.."/franka_emika_panda/panda.xml",  posedof=CT.vec(1, 1, 0, 0, 0, 0, 0, 0.0, 0.0)},
	{ title='hyunwoo', '../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_boxfoot.mujoco.xml' },
}

function ctor() 
	local items={}
	titleMap={}
	for i, v in ipairs(robot_files) do
		items[i]=v.title
		titleMap[v.title]=v
	end
	this:addMenu('file', items)
	this:updateLayout()


	setViewYUp(false) -- set Zup. (Usually taesooLib uses Yup convention.)
	local robotfile= robot_files[1][1]
	taesooLib_pose=robot_files[1].posedof
	start(robotfile)

end
function start(robotfile)

	--mLoader=MujocoLoader(robotfile, { useVisualMesh=true, convertToYUP=false, exportToMemory=true}) -- todo: .obj rotation when convertToYUP==true. set exportToMemory false to generate .xml.wrl (which is a little slow)
	
	if robotfile:sub(-3)=='xml' then
		mLoader=MujocoLoader(robotfile)
	else
		mLoader=URDFloader(robotfile)
	end
	mLoader:updateInitialBone()


	this:create("Box", "bones", "bones")
	for i=2, mLoader:numBone()-1 do
		this:addFloatSlider(mLoader:bone(i):name(),0,-2,2)
	end
	this:updateLayout()

	mSkin=RE.createVRMLskin(mLoader, true)
	mSkin:scale(100,100,100)
	--mSkin:setMaterial('lightgrey_transparent')

	dbg.draw("Axes", transf(quater(1,0,0,0), vector3(0,0,0)), 'axes', 100)

	if not taesooLib_pose then
		taesooLib_pose=mLoader:getPoseDOF()
	end


	mSkin:setPoseDOF(taesooLib_pose)

end

function dtor()

	this:removeWidgets(this:widgetIndex('bones'));
	mLoader=nil
	mSkin=nil
	taesooLib_pose=nil
	collectgarbage()
end

function handleRendererEvent()
	return 0
end
function onCallback(w, userData)
	if w:id()=='file' then
		dtor()
		local robotfile= titleMap[w:menuText()]
		taesooLib_pose=robotfile.posedof
		start(robotfile[1])
	end
	local ti=mLoader:getTreeIndexByName(w:id())
	if ti~=-1 then
		taesooLib_pose:set(mLoader.dofInfo:startT(ti), w:sliderValue())
		mSkin:setPoseDOF(taesooLib_pose)
		print(taesooLib_pose)
	end
	return 0
end
function frameMove(fElapsedTime)
end
