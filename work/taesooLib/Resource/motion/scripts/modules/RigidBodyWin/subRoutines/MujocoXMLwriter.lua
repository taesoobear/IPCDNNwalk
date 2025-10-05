require('tl') -- for BaseLib <-> torch interfacing


-- input: vrmlloaders, filename, option={ groundPlane=true/false , recalculateParam=true/false, convertToZUP=true/false, useEulerIntegrator=true/false, useMotorActuator=true/false }
-- all boolean options are false by default
function writeMujocoXML(loaders, fn, option)

	option=option or {}
	local out={}
	local default_armature='armature="'..(option.default_armature or 0.1)..'" '
	if option.default_armature==0 then
		default_armature=''
	end

	if option.convertToZUP then
		option.vec3str=function (v3)
			-- changes from YUP to ZUP
			return ''..v3.z..' '..v3.x..' '..v3.y
		end

		option.quat2str=function (v3)
			-- changes from YUP to ZUP
			return ''..v3.w..' '..v3.z..' '..v3.x..' '..v3.y
		end
	else
		option.vec3str=function (v3)
			return ''..v3.x..' '..v3.y..' '..v3.z
		end

		option.quat2str=function (v3)
			return ''..v3.w..' '..v3.x..' '..v3.y..' '..v3.z
		end
	end
	if option.recalculateParam then
		table.insert(out, [[
<mujoco model="humanoid">
    <compiler angle="degree" inertiafromgeom="true"/>
    <default>
	]])
	else
		table.insert(out, [[
<mujoco model="humanoid">
    <compiler angle="degree" />
    <default>
	]])
	end
	if option.default_armature or option.default_damping then
	table.insert(out, [[
        <joint ]]..default_armature..[[damping="]]..(option.default_damping or 1)..[[" limited="true"/>
	]])
	end
	table.insert(out, [[
        <geom conaffinity="1" condim="1" contype="1" margin="0.001" rgba="0.8 0.6 .4 1"/>
	]])
	if option.useMotorActuator then
		table.insert(out, [[
        <motor ctrllimited="true" ctrlrange="-.4 .4"/>
		]])
	else
		table.insert(out, [[
		<general dyntype="none" biastype="affine" ctrlrange="-2.8973 2.8973" forcerange="-87 87" gainprm="4500" biasprm="0 -4500 -450"/>
		]])
	end
	table.insert(out, [[
    </default>
	]])
	if option.useEulerIntegrator then
		table.insert(out, [[
    <option integrator="Euler" iterations="50" solver="PGS" timestep="0.003">
	]])
	else
		table.insert(out, [[
    <option integrator="implicitfast">
	]])
	end
	table.insert(out, [[
        <!-- <flags solverstat="enable" energy="enable"/>-->
    </option>
    <size nkey="5" nuser_geom="1"/>
    <visual>
        <map fogend="5" fogstart="3"/>
    </visual>
    <asset>
        <!-- <texture builtin="gradient" height="100" rgb1="1 1 1" rgb2="0 0 0" type="skybox" width="100"/>-->
	]])
	if option.groundPlane then
		table.insert(out,[[

        <texture builtin="flat" height="1278" mark="cross" markrgb="1 1 1" name="texgeom" random="0.01" rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" type="cube" width="127"/>
        <texture builtin="checker" height="100" name="texplane" rgb1="0 0 0" rgb2="0.8 0.8 0.8" type="2d" width="100"/>
        <material name="geom" texture="texgeom" texuniform="true"/>
        <material name="MatPlane" reflectance="0.5" shininess="1" specular="1" texrepeat="60 60" texture="texplane"/>
		]])
	end
	table.insert(out, [[
    </asset>
    <worldbody>
        <light cutoff="100" diffuse="1 1 1" dir="-0 0 -1.3" directional="true" exponent="1" pos="0 0 1.3" specular=".1 .1 .1"/>
	]])

	if option.groundPlane then
		table.insert(out, [[
        <geom condim="3" friction="1 .1 .1" material="MatPlane" name="floor" pos="0 0 -1.9" rgba="0.8 0.9 0.8 1" size="20 20 0.125" type="plane"/>
        <!-- <geom condim="3" material="MatPlane" name="floor" pos="0 0 0" size="10 10 0.125" type="plane"/>-->]])
	end

	if type(loaders)~='table' then
		loaders={loaders}
	end
	option.usedNames={}
	for iloader, loader in ipairs(loaders) do
		writeMujocoBody(out, loader, loader:VRMLbone(1), 0, option)
	end
	table.insert(out, [[
    </worldbody>
</mujoco>
]])

util.writeFile(fn, table.concat(out, '\n'))

end
function makeTabs(level)
	local out=''
	for i=1, level do
		out=out..'\t'
	end
	return out
end
function mujoco_packShape(out, loader, bone, level, setConstraints, init_pos, option)
	local tabs=makeTabs(level)
	if bone:hasShape() then
		local mesh=bone:getMesh()
		local ne= mesh:numElements()

		local condim=''
		if setConstraints then
			condim='condim="3" friction="0.5 0.5 0.5" '
		end
		local densityString=''
		local totalVolume=mesh:totalVolume()
		if totalVolume>0 and option.recalculateParam then
			densityString='density="'..(bone:mass()/totalVolume)..'" '
		end
		for i=0, ne-1 do
			local e=mesh:element(i)
			if e.elementType==OBJloader.Element.BOX then
				table.insert(out,tabs.. [[			<geom type="box" ]]..condim.. densityString..[[size="]]..option.vec3str(e.elementSize*0.5)..[[" pos="]]..option.vec3str(e.tf.translation+init_pos)..[[" quat="]]..option.quat2str(e.tf.rotation)..[["/>]])
			elseif e.elementType==OBJloader.Element.SPHERE or 
				e.elementType==OBJloader.Element.ELLIPSOID then
				local es=e.elementSize
				if math.abs(es.x-es.y)<1e-3 and math.abs(es.y-es.z)<1e-3 then
					table.insert(out,tabs.. [[			<geom type="sphere" ]]..condim.. densityString..[[size="]]..(e.elementSize.x)..[[" pos="]]..option.vec3str(e.tf.translation+init_pos)..[[" quat="]]..option.quat2str(e.tf.rotation)..[["/>]])
				else
					table.insert(out,tabs.. [[			<geom type="ellipsoid" ]]..condim.. densityString..[[size="]]..option.vec3str(e.elementSize*0.5)..[[" pos="]]..option.vec3str(e.tf.translation+init_pos)..[[" quat="]]..option.quat2str(e.tf.rotation)..[["/>]])
				end
			end
		end
	else
		local COM=bone:localCOM()
		if COM:length()>0.02 then
			table.insert(out,tabs.. [[		<geom fromto="0 0 0]]..option.vec3str(COM*2)..[[" size="0.049" type="capsule"/>]])
		else
			table.insert(out, tabs..[[		<geom fromto="0 0 0 0 0 0.02" size="0.049" type="capsule"/>]])
		end
	end
end
function writeMujocoBody(out, loader, bone, level, option)
	local offset=bone:getOffsetTransform().translation
	local tabs=makeTabs(level)
	local writeBody=true
	local init_pos=vector3(0,0,0)

	if loader.dofInfo:numDOF()==0 then
		writeBody=false
		-- in this case, only the geometries are written in the xml file.
		init_pos=loader:bone(1):getOffset()
	end
	if writeBody then
		-- give unique name
		local orig_name=bone:name()
		local c=0
		while option.usedNames[bone:name()] do
			bone:setName(orig_name..c)
			c=c+1
		end
		table.insert(out, tabs..[[
		<body name="]]..bone:name()..[[" pos="]]..option.vec3str(offset)..'">')
		if not option.recalculateParam then
			table.insert(out, tabs..[[
		  <inertial mass="]]..bone:mass()..'" diaginertia="'..option.vec3str(bone:inertia())..'" pos="'..option.vec3str(bone:localCOM())..'"/>')
		end

		option.usedNames[bone:name()]=true
	end

	local shapePacked=false
	if bone:treeIndex()==1 then
		if bone:HRPjointType(0)==MainLib.VRMLTransform.FREE then
			table.insert(out, tabs..[[
			<joint limited="false" pos="0 0 0" stiffness="0" type="free"/>]])
		elseif loader:numBone()==2 then
			mujoco_packShape(out, loader, bone, level, not option.no_condim, init_pos, option)
			shapePacked=true
		end
	elseif bone:numChannels()>1 and option.useSphericalJoints then
		table.insert(out, tabs..[[
		<joint limited="false" pos="0 0 0" stiffness="0" type="ball"/>]])
	else
		for i=0, bone:numChannels()-1 do
			table.insert(out, tabs..[[
			<joint axis="]]..option.vec3str(bone:axis(i))..[[" pos="0 0 0" range="-170 170" type="hinge"/>]])
		end
	end

	if not shapePacked then
		mujoco_packShape(out, loader, bone, level, false, init_pos, option)
	end

	bone=bone:childHead()
	while bone do
		bone=MainLib.VRMLloader.upcast(bone)
		writeMujocoBody(out, loader, bone, level+1, option)
		bone=bone:sibling()
	end

	if writeBody then
		table.insert(out, makeTabs(level)..[[
		</body>]])
	end
end
