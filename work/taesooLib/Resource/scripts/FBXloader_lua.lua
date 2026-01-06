require('subRoutines/WRLloader')
-- Use FBXloader.lua instead
-- : This class uses an external tool fbxdumpLua  and is very slow
-- Now that fbx is supported natively by taesooLib,
-- this class is deprecated. Use only for debugging purposes.

local FBXloader=LUAclass()
-- An example:
-- ./fbxdumpLua ../../UE4/Source/FBX/Root_Motion/MOB1_Run_L_180.fbx > out
-- LimbNode 
-- 		- Properties
--	 		- PreRotation
--	 		- Lcl Translation
--	 		- Lcl Rotation
-- Other properties are usually in the template
--	 		- Lcl Scaling
--	 		- RotationOrder
--
-- AnimationCurveNode - Rot(yaw) : 867129088 -> does not seem to be used.
-- 		- : 948366096 (AnimationLayer "MOB1_Run_L_180:BaseAnimation" ) 
-- 		- : 951939936 (root, Model, LimbNode)
-- 		- 970816944 :  (AnimCurve) - [0,0,0,0]

function FBXloader.getCardinalSplineABS(t_global, spline,tension )
	if not tension then
		tension=0.33 -- tension 0 : catmal-rom
	end
	local time=spline:column(0)
	local control1=spline:column(1):range(0, spline:rows()-1)
	local control2=spline:column(1):range(1, spline:rows())
	local eps=1e-10
	assert(t_global>=0.0)
	local t=0
	local w
	local iseg=-1
	for i=0, time:size()-2 do
		t=time(i+1)
		local d=t-time(i)
		if (t >= t_global-eps) then -- at junctions, returns previous spline (=)
			iseg=i
			w=sop.map(t_global, t-d, t, 0, 1)
			break
		end
	end
	--assert(iseg~=-1)
	if iseg==-1 then
		return control2(control2:size()-1)
	end
	local pc
	local nc
	if iseg==0 then
		pc=control1(0)
		if w<0 then w=0 end
	else
		pc=control1(iseg-1)
	end
	if iseg==control2:size()-1 then
		if w>1 then w=1 end
		nc=control2(iseg)
	else
		nc=control2(iseg+1)
	end
	local function ccCardinalSplineAt( p0, p1, p2, p3, tension, t )
		local t2 = t * t;
		local t3 = t2 * t;

		local s = (1 - tension) / 2;

		local b1 = s * ((-t3 + (2.0 * t2)) - t);					-- s(-t3 + 2 t2 – t)P1
		local b2 = s * (-t3 + t2) + (2.0 * t3 - 3.0 * t2 + 1.0);		-- s(-t3 + t2)P2 + (2 t3 – 3 t2 + 1)P2
		local b3 = s * (t3 - 2.0 * t2 + t) + (-2.0 * t3 + 3.0 * t2);	-- s(t3 – 2 t2 + t)P3 + (-2 t3 + 3 t2)P3
		local b4 = s * (t3 - t2);									-- s(t3 – t2)P4


		return (p0*b1 + p1*b2 + p2*b3 + p3*b4); 
	end
	return ccCardinalSplineAt( pc, control1(iseg), control2(iseg), nc, tension, w )
end

function FBXloader.motionLoader(filename)
	-- move to member
	g_bones={}
	g_nodes={}
	g_rotYaw=nil
	g_nFrames=0
	g_frameTime=0
	g_T=0

	assert(os.isFileExist(filename))
	if(not os.isFileExist('fbxdumpLua')) then
		util.msgBox('Error! copy taesooLib/work/fbxdumpLua  to the working folder of this program')
		assert(false)
	end

	local script=os.capture('./fbxdumpLua "'..filename..'"' )
	return FBXloader.motionLoaderFromScript(script)
end

function FBXloader.motionLoaderFromScript(script)
	local fcn,msg=loadstring('return '..script)
	if not fcn then
		print(msg)
		return
	end
	local tbl=fcn()
	tbl=simplifyTable(tbl)
	tbl.bones=g_bones
	for i, bone in ipairs(tbl.bones) do
		bone.curves={}
		assert(bone.nodes)
		for j, node in ipairs(bone.nodes) do
			bone.curves[j]=g_nodes[tostring(node)]
			simplifyCurve2(bone.curves[j])
			g_nodes[tostring(node)]=nil
		end
		bone.nodes=nil
		g_nodes[tostring(bone[2])]=nil
	end
	tbl.nodes=g_nodes
	tbl.nframe=g_nFrames
	tbl.frameTime=g_frameTime -- check if this is consistent with nframe and totalTime. Otherwise, keytimes are not uniformly sampled.
	print(g_frameTime*(g_nFrames-1), g_T) -- should be similar
	tbl.totalTime=g_T 
	-- not an actual script, but useful for debugging.
	--util.writeFile('gym_walkCDM/run_f_jump_simplified.lua', table.toHumanReadableString(tbl))

	local loader=MotionLoader()
	loader:createDummyRootBone()

	local mapBones={}
	for i, bone in ipairs(g_bones) do
		mapBones[bone.name]=bone
		if bone.parent then
			loader:insertChildBone(loader:getBoneByName(bone.parent), bone.name, false)
		else
			loader:insertChildBone(loader:bone(0), bone.name, false)
		end
		local b=loader:getBoneByName(bone.name)

		if bone.LclTranslation then
			if i~=1 then
				b:getOffsetTransform().translation:assign(bone.LclTranslation[2])
			end
		end
		local function findRT(curves, startIndex)
			if not curves[2] then return false end
			for i=startIndex, #curves do 
				local tid=curves[i][2]
				if tid=='R' or tid=='T' then
					return true
				end
			end
			return false
		end

		if bone.curves[1] and bone.curves[2] and bone.curves[1][2]=="T" and bone.curves[2][2]=="R" then
			loader:insertJoint(b, "RT")
		elseif bone.curves[1] and not findRT(bone.curves,2) and bone.curves[1][2]=="R" then
			loader:insertJoint(b, "R")
		elseif bone.curves[1] and not findRT(bone.curves,2) and bone.curves[1][2]=="T" then
			loader:insertJoint(b, "T")
		elseif #bone.curves>0 then
			for ic, curve in ipairs(bone.curves) do
				assert(curve[2]~="R")
				assert(curve[2]~="T")
				print('ignoring '..b:name()..':'..curve[2])
			end
		else
			assert(#bone.curves==0)
			loader:insertJoint(b, "R")
		end
	end
	loader:insertSiteBones();
	loader:_initDOFinfo()
	loader:printHierarchy()

	loader.mMotion:init(loader)
	loader.mMotion:resize(g_nFrames)
	loader.mMotion:setFrameTime(g_frameTime)
	if not (math.abs(loader.mMotion:totalTime()- g_T)<1e-3) then
		print("warning! timing seems to be incorrect", g_nFrames, g_frameTime, g_T)
	end
	loader:updateInitialBone();
	local mot=loader.mMotion


	if false then
		g_correctMot=MotionLoader._create("gym_walkCDM/out2.bvh")
		-- debugMode
	end
	if false then
		-- for debugging
		local ti=2
		local bone=loader:bone(ti)
		local j=loader:getRotJointIndexByTreeIndex(ti)
		local tj=loader:getTransJointIndexByTreeIndex(ti)
		local boneInfo=mapBones[bone:name()]
		if boneInfo then
			local preRot=quater(1,0,0,0)
			if boneInfo.preRotation then
				preRot=boneInfo.preRotation
			end
			local rinfo=1
			if j~=-1 then

				local rotCurves=boneInfo.curves[rinfo]
				if rotCurves and #rotCurves.curves>0 then
					print('rootAX', rotCurves.curves[1][2].KeyValueFloat)
					print('rootAXt', rotCurves.curves[1][2].KeyTime:size(), rotCurves.curves[1][2].KeyTime)
					print('rootAY', rotCurves.curves[2][2].KeyValueFloat)
					print('rootAYt', rotCurves.curves[2][2].KeyTime:size(), rotCurves.curves[2][2].KeyTime)
					print('rootAZ', rotCurves.curves[3][2].KeyValueFloat)
					print('rootAZt', rotCurves.curves[3][2].KeyTime:size(), rotCurves.curves[3][2].KeyTime)
				end
			end
		end
	end
	for iframe=0, mot:numFrames()-1 do
		local pose=loader.mMotion:pose(iframe)
		loader:getPose( pose)

		for ti=1, loader:numBone()-1 do
			local bone=loader:bone(ti)
			local t=sop.map(iframe, 0, mot:numFrames()-1, 0, g_T)
			local j=loader:getRotJointIndexByTreeIndex(ti)
			local tj=loader:getTransJointIndexByTreeIndex(ti)
			local boneInfo=mapBones[bone:name()]
			if boneInfo then
				local preRot=quater(1,0,0,0)
				if boneInfo.preRotation then
					preRot=boneInfo.preRotation
				end
				local rinfo=1
				if tj~=-1 then
					local transCurves=boneInfo.curves[1]
					assert(transCurves[2]=='T')
					if #transCurves.curves>0 then
						local v=sampleCurve3D(transCurves, t)
						pose.translations(tj):assign(v)
					end
					rinfo=2
				end
				if j~=-1 then

					local rotCurves=boneInfo.curves[rinfo]
					if rotCurves and #rotCurves.curves>0 then
						assert(rotCurves[2]=='R')
						local q=sampleRotCurve(rotCurves, t)
						pose.rotations(j):assign(preRot*q)

						if false then
							-- debugMode
							local pose2=g_correctMot.mMotion:pose(iframe)
							local rj2=g_correctMot:getRotJointIndexByName(bone:name())
							local q2=pose2.rotations(rj2) -- correct
							if j==2 and q.w~=1 then
								dbg.console()
							end
						end
					end
				end
			end
		end
	end

	--local curve_yaw=g_rotYaw.curves
	--assert(curve_yaw)
	--assert(#curve_yaw==1)
	--for iframe=0, mot:numFrames()-1 do
		--local pose=loader.mMotion:pose(iframe)
		--local qy=quater(math.rad(-90), vector3(1,0,0))

		--local t=sop.map(iframe, 0, mot:numFrames()-1, 0, g_T)
		--local v=sampleCurve1D(curve_yaw[1], t)
		--print(v)
		--pose.rotations(0):assign(qy)
	--end
	--dbg.console()
	return loader
end

-- currently assumes the followings!!! 
-- Lcl Scaling==1.0
-- RotationOrder=0 (eOrderXYZ)
-- KeyAttrFlags== 264 or 65800  
-- (both finishes with 0x108 (eInterpolationCubic|eTangentAuto), and optionally can have 0x10000 (undocumented in FBXSDK)
--
-- check the input file throughly if it satisfies these assumptions!!!
-- FBX flags
EOrder={
	eOrderXYZ=0,
	eOrderXZY=1,
	eOrderYZX=2,
	eOrderYXZ=3,
	eOrderZXY=4,
	eOrderZYX=5,
	eOrderSphericXYZ=6
}

EInterpolationTime={
	eInterpolationConstant = 0x00000002,	-- Constant value until next key.
	eInterpolationLinear = 0x00000004,		-- Linear progression to next key.
	eInterpolationCubic = 0x00000008		-- Cubic progression to next key.
}
ETangentMode=
{
	eTangentAuto = 0x00000100,										-- Auto key (spline cardinal).
	eTangentTCB = 0x00000200,										-- Spline TCB (Tension, Continuity, Bias)
	eTangentUser = 0x00000400,										-- Next slope at the left equal to slope at the right.
	eTangentGenericBreak = 0x00000800,								-- Independent left and right slopes.
	eTangentBreak = 0x00000800+0x00000400,							-- Independent left and right slopes, with next slope at the left equal to slope at the right.
	eTangentAutoBreak = 0x00000800+0x00000100,						-- Independent left and right slopes, with auto key.
	eTangentGenericClamp = 0x00001000,								-- Clamp: key should be flat if next or previous key has the same value (overrides tangent mode).
	eTangentGenericTimeIndependent = 0x00002000,					-- Time independent tangent (overrides tangent mode).
	eTangentGenericClampProgressive = 0x00004000+0x00002000			-- Clamp progressive: key should be flat if tangent control point is outside [next-previous key] range (overrides tangent mode).
}

function simplifyCurve(t)
	local out={}
	for i,v in ipairs(t) do
		local key=v[1]
		if string.len(key)>0 then
			out[key]=v[2]
		end
	end
	return out
end
function simplifyCurve2(t)
	for i, v in ipairs(t.curves) do
		t.curves[i]=g_nodes[tostring(v)]
		local keyTime=t.curves[i][2].KeyTime
		local nframe=#keyTime
		assert(not t.curves[i].keytimeConverted)
		for j, key in ipairs(keyTime) do
			keyTime[j]=key/46186158000 --; I hate autodesk.
		end
		t.curves[i][2].KeyTime=CT.vec(keyTime)
		t.curves[i][2].KeyValueFloat=CT.vec(t.curves[i][2].KeyValueFloat)
		t.curves[i].keytimeConverted=true

		local kt=t.curves[i].keyTime

		g_nFrames=math.max(g_nFrames, nframe)
		if nframe>1 then
			if nframe==g_nFrames then
				g_frameTime=keyTime[2]-keyTime[1]
				print('ft:', g_nFrames, g_frameTime)
			end
			g_T=math.max(g_T, keyTime[#keyTime])
		end
		g_nodes[tostring(v)]=nil
	end
end
function findNode(tbl, name)
	for i, v in ipairs(tbl) do
		if v[1]==name then
			return v.children
		end
	end
	return nil
end
function findProperty(tbl, name)
	for i, v in ipairs(tbl) do
		if v[2] then
			if v[2][2][1]==name then
				return v[2], i
			end
		end
	end
	return nil
end
function simplifyTable(t)
	if type(t)=='string' then
		assert(string.sub(t, 0,2)==' [')
		local tbl=loadstring('return {'..string.sub(t, 3, -3)..'}')()
		return tbl
	end
	assert(type(t)=="table", "You must specify a table to copy")

	local result={}
	for k, v in ipairs(t) do
		if type(v)=="table" then
			local kk=v[1]
			if kk=='properties'  and #v==2 then
				if type(v[2])=='table' and #v[2]==1 then
					table.insert(result,v[2][1])
				else
					table.insert(result,simplifyTable(v[2]))
				end
			elseif kk=='Model' then
				local vv=v[2]
				if vv[4][1]=='LimbNode' then
					local boneName=string.sub(vv[3][1], 1, -16)
					local bone={'LimbNode', vv[2][1], name=boneName, nodes={}}

					local prop=findNode(v.children, "Properties70")
					assert(prop)
					local preR=findProperty(prop, 'PreRotation')
					if preR then
						assert(preR[3][1]=="Vector3D")
						bone.preRotation=getRotation("ZYX", vector3(preR[6][1], preR[7][1], preR[8][1]))
					end
					-- Get the node’s default TRS properties
					local pi, pi2
					local LclTranslation,pi=findProperty(prop, 'Lcl Translation')
					if LclTranslation then
						bone.LclTranslation={LclTranslation[5][1] , vector3( LclTranslation[6][1], LclTranslation[7][1], LclTranslation[8][1])}
					end
					local LclRotation,pi2=findProperty(prop, 'Lcl Rotation')
					if LclRotation then
						bone.LclRotation={LclRotation[5][1], vector3( LclRotation[6][1], LclRotation[7][1], LclRotation[8][1])}
						local eulerXYZ=bone.LclRotation[2]
						bone.LclRotation[2]=getRotation("ZYX", eulerXYZ)
					end
					local LclScale=findProperty(prop, 'Lcl Scaling')
					if LclScale then
						-- PropertyTemplate 에만 들어있어서 안잡힘
						--dbg.console()
					end
					
					if pi and pi2 then
						assert(pi2>pi)
					end
					table.insert(g_bones, bone)
					g_nodes[tostring(vv[2][1])]=bone
				else
					g_nodes[tostring(vv[2][1])]={vv[4][1], 'ignored'}
				end
			elseif kk=='AnimationCurveNode' then
				local vv=v[2]
				local channel=string.sub(vv[3][1], 1, -16-8)
				g_nodes[tostring(vv[2][1])]={kk, channel, curves={}}

				if channel:lower()=='rot(yaw)' then
					assert( g_rotYaw==nil)
					g_rotYaw=g_nodes[tostring(vv[2][1])]
				end
			elseif kk=="C" then
				local vv=v[2]
				local node1=g_nodes[tostring(vv[3][1])]
				local node2=g_nodes[tostring(vv[4][1])]
				if not node1 or not node2 then
					table.insert(result, simplifyTable(v))
				elseif node1[1]=='LimbNode' and node2[1]=='LimbNode' then
					node1.parent=node2.name
				elseif node2[1]=='LimbNode' then
					assert(vv[2][1]=='OP')
					table.insert(node2.nodes, vv[3][1])
				elseif node2[1]=='AnimationCurveNode' then
					assert(vv[2][1]=='OP')
					table.insert(node2.curves,vv[3][1])
					assert(node1.channel==nil)
					node1.channel=vv[5][1]
				elseif node1[1]=='LimbNode' then
					table.insert(result, {'C', vv[2][1], node1.name, vv[4][1]})
				else
					table.insert(result, simplifyTable(v))
				end
			elseif kk=="AnimationCurve" then
				local vv=v[2]
				g_nodes[tostring(vv[2][1])]={kk, simplifyCurve(simplifyTable(v.children))}
			elseif kk=='FBXHeaderExtension' then
				table.insert(result, {kk, 'ignored'}) --change 'ignored' to simplifyTable(v) to use the data.
			else
				table.insert(result, simplifyTable(v))
			end
		elseif type(v)=="userdata" then
			if v.copy then
				table.insert(result,v:copy())
			else
				print('Cannot copy '..k)
			end
		else
			table.insert(result,v)
		end
	end
	if t.children then
		result.children=simplifyTable(t.children)
	end
	if t.version then
		result.version=t.version
	end

	return result
end
function sampleCurve1D(curves, t)
	local kt=curves[2].KeyTime
	local kv=curves[2].KeyValueFloat
	if kt:size()==0 then
		dbg.console()
	elseif kt:size()==1 then
		return kv(0)
	end
	--local v=fc.getLinearSplineABS(t, kt:column().. kv:column())
	local v=FBXloader.getCardinalSplineABS(t, kt:column().. kv:column())
	return v
end
function sampleCurve3D(curves, t)
	local o=vector3()
	o.x=sampleCurve1D(curves.curves[1], t)
	o.y=sampleCurve1D(curves.curves[2], t)
	o.z=sampleCurve1D(curves.curves[3], t)
	return o
end

function getRotation(reversed_channels, eulerDeg)
	local function rad_reverse(v)
		return vector3(math.rad(v.z), math.rad(v.y), math.rad(v.x))
	end
	local q=quater()
	q:setRotation(reversed_channels, rad_reverse(eulerDeg))
	return q
end

function sampleRotCurve(rotCurves, t)
	assert(rotCurves.curves[1].channel=='d|X')
	assert(rotCurves.curves[2].channel=='d|Y')
	assert(rotCurves.curves[3].channel=='d|Z')

	return getRotation("ZYX", sampleCurve3D(rotCurves, t))
end
return FBXloader
