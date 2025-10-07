require('tl') -- for BaseLib <-> torch interfacing


-- input: vrmlloaders, filename, option={ groundPlane=true/false , recalculateParam=true/false, useEulerIntegrator=true/false, useMotorActuator=true/false, exportAsVisual=false }
-- all boolean options are false by default
function writeMujocoXML(loaders, fn, option)

	option=option or {}
	if not option.loaderOptions then
		option.loaderOptions={}
	end

	option.out_filename=fn
	
	local out={}
	local default_armature='armature="'..(option.default_armature or 1)..'" '
	if option.default_armature==0 then
		default_armature=''
	end

	if option.convertToZUP then
		-- do not use this option. use newLoader=loader:YtoZ() instead
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
    <compiler angle="radian" inertiafromgeom="true" />
    <default>
	]])
	else
		table.insert(out, [[
<mujoco model="humanoid">
    <compiler angle="radian" balanceinertia="true" />
    <default>
	]])
	end
	-- define default classes
	-- I am using group 1 for static geometries
	table.insert(out, [[
		<default class="visual">
			<geom type="mesh" contype="0" conaffinity="0" group="2"/>
		</default>
        <joint ]]..default_armature..[[damping="]]..(option.default_damping or 1)..[[" limited="true"/>
	]])
	table.insert(out, [[
        <geom conaffinity="1" condim="1" contype="1" margin="0.001" rgba="0.8 0.6 .4 1"/>
	]])
	if option.useMotorActuator then
		table.insert(out, [[
        <motor ctrllimited="true" ctrlrange="-5 5"/>
		]])
	else
		table.insert(out, [[
		<general dyntype="none" biastype="affine" ctrlrange="-2.8973 2.8973" forcerange="-87 87" gainprm="4500" biasprm="0 -4500 -450"/>
		]])
	end
	table.insert(out, [[
    <muscle ctrllimited="true" ctrlrange="0 1"/>
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

	local header=table.concat(out, '\n')
	out={}
	local out_asset={}

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
	local all_muscles={}
	for iloader, loader in ipairs(loaders) do
		local loaderoption=option
		if option.loaderOptions[iloader] then
			loaderoption=table.merge(option, option.loaderOptions[iloader])
		end
		loaderoption.iloader=iloader

		if not loaderoption.sites then
			loaderoption.sites={}
			for i=1, loader:numBone()-1 do
				loaderoption.sites[i]={}
			end
		end
		if loaderoption.muscles then
			for i, muscle in ipairs(loaderoption.muscles) do
				assert(muscle[1]==i)
				local muscleName=muscle[2]
				local npathpoint=muscle[3]
				for j=1, npathpoint do
					local treeIndex=muscle[3+j*2-1]
					local lpos=muscle[3+j*2]
					table.insert(loaderoption.sites[treeIndex], {muscleName..tostring(j), lpos})
				end
			end
			table.insert(all_muscles, loaderoption.muscles)
		end
		writeMujocoBody(out, out_asset, loader, loader:VRMLbone(1), 0, loaderoption)
	end
	table.insert(out, [[
    </worldbody>]])

	if #all_muscles>0 then
		table.insert(out, "<tendon>")

		local names={}
		for iset, muscles in ipairs(all_muscles) do
			for i, muscle in ipairs(muscles) do
				assert(muscle[1]==i)
				local muscleName=muscle[2]
				local npathpoint=muscle[3]
				table.insert(out, [[    <spatial name="]]..muscleName..[[" width="0.009" rgba=".9 0 0 1">]])
				for j=1, npathpoint do
					local treeIndex=muscle[3+j*2-1]
					local lpos=muscle[3+j*2]
					table.insert(out, [[      <site site="]]..muscleName..tostring(j)..[["/>]])
				end
				table.insert(out, [[    </spatial>]])
				table.insert(names, muscleName)
			end
		end

		table.insert(out, "</tendon>")
		all_muscles.names=names
	end


	-- contact
	table.insert(out, "\t<contact>")
	-- exclude self collisions (only within the same chain)
	for  iloader, loader in ipairs(loaders) do
		if loader:numBone()>2 then
			for i=2,loader:numBone()-1 do
				local name1=loader:bone(i):name()
				local name2=loader:bone(i):parent():name()
				table.insert(out, "\t\t<exclude body1="..'"'.. name1 ..'" body2="'..name2..'"/>')
				if loader:bone(i):parent():treeIndex()~=1 then
					local pbone=loader:bone(i):parent():parent()
					local name3=pbone:name()
					table.insert(out, "\t\t<exclude body1="..'"'.. name1 ..'" body2="'..name3..'"/>')

					if pbone:treeIndex()~=1 then
						pbone=pbone:parent()
						local name3=pbone:name()
						table.insert(out, "\t\t<exclude body1="..'"'.. name1 ..'" body2="'..name3..'"/>')
						if pbone:treeIndex()~=1 then
							pbone=pbone:parent()
							local name3=pbone:name()
							table.insert(out, "\t\t<exclude body1="..'"'.. name1 ..'" body2="'..name3..'"/>')
						end
					end
				end
			end
		end
	end
	table.insert(out, "\t</contact>")

	-- actuators
	table.insert(out, "\t<actuator>")
	for  iloader, loader in ipairs(loaders) do
		if loader:numBone()>2 and not option.useSphericalJoints then
			local level=2
			local tabs=makeTabs(level)


			for i=1,loader:numBone()-1 do
				local bone=loader:VRMLbone(i)
				if bone:treeIndex()==1 and bone:HRPjointType(0)==MainLib.VRMLTransform.FREE then
				else
					for i=0, bone:numChannels()-1 do
						local name=bone:name().."_"..tostring(i)
						table.insert(out, "\t\t"..[[<motor ctrllimited="true" ctrlrange="-5 5" gear="50" joint="]]..name..[[" name="]]..name..[["/>]])
					end
				end
			end
		end
	end
	if #all_muscles>0 then
		local names=all_muscles.names
		for i, name in ipairs(names) do
			table.insert(out, [[      <muscle name="]]..name..[[" tendon="]]..name..[[" range="0 1"/>]])
		end
	end
	table.insert(out, "\t</actuator>")
	table.insert(out, [[
</mujoco>
]])

util.writeFile(fn, header..'\n'..table.concat(out_asset,'\n')..'\n'..table.concat(out, '\n'))

end
function makeTabs(level)
	local out=''
	for i=1, level do
		out=out..'\t'
	end
	return out
end
function mujoco_packShape(out, out_asset, loader, bone, level, setConstraints, init_pos, option, group)
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
		local groupString=''
		if group then
			groupString=' group="'..tostring(group)..'" '
		end
		for i=0, ne-1 do
			local e=mesh:element(i)
			if e.elementType==OBJloader.Element.BOX then
				table.insert(out,tabs.. [[			<geom type="box" ]]..condim.. densityString..groupString..[[size="]]..option.vec3str(e.elementSize*0.5)..[[" pos="]]..option.vec3str(e.tf.translation+init_pos)..[[" quat="]]..option.quat2str(e.tf.rotation)..[["/>]])
			elseif e.elementType==OBJloader.Element.SPHERE or 
				e.elementType==OBJloader.Element.ELLIPSOID then
				local es=e.elementSize
				if math.abs(es.x-es.y)<1e-3 and math.abs(es.y-es.z)<1e-3 then
					table.insert(out,tabs.. [[			<geom type="sphere" ]]..condim.. densityString..groupString..[[size="]]..(e.elementSize.x)..[[" pos="]]..option.vec3str(e.tf.translation+init_pos)..[[" quat="]]..option.quat2str(e.tf.rotation)..[["/>]])
				else
					table.insert(out,tabs.. [[			<geom type="ellipsoid" ]]..condim.. densityString..groupString..[[size="]]..option.vec3str(e.elementSize*0.5)..[[" pos="]]..option.vec3str(e.tf.translation+init_pos)..[[" quat="]]..option.quat2str(e.tf.rotation)..[["/>]])
				end
			elseif e.elementType==OBJloader.Element.PLANE then
				local tf=bone:getLocalFrame() -- todo: this is strange. offset frame?
				--local quat=quater(math.rad(180), vector3(1,0,0))
				local quat=quater(math.rad(-90),vector3(1,0,0))
				table.insert(out,tabs.. [[			<geom type="plane" ]]..condim.. groupString..[[size="]]..option.vec3str(vector3(e.elementSize.x, e.elementSize.x, e.elementSize.z))..[[" pos="]]..option.vec3str(tf.translation)..[[" quat="]]..option.quat2str(quat*tf.rotation)..[["/>]])
			elseif e.elementType==OBJloader.Element.OBJ then
				os.createDir(option.out_filename..'.assets')
				local tempGeom=Geometry()
				tempGeom:extractSubMesh(mesh, i)
				local meshName='geom'..tostring(option.iloader)..'_'..tostring(bone:treeIndex())..'_'..tostring(i)
				local stlFileName=option.out_filename..'.assets/'..meshName..'.stl'
				assert(tempGeom:saveMesh(stlFileName))

				local localStlFileName=os.filename(option.out_filename)..'.assets/'..meshName..'.stl'

				table.insert(out_asset, [[		<mesh name="]]..meshName..[[" file="]]..localStlFileName..[["/>]])

				--table.insert(out,tabs.. [[			<geom mesh="]]..meshName..[[" class="collision" ]]..condim.. groupString..[[" pos="]]..option.vec3str(tf.translation)..[[" quat="]]..option.quat2str(tf.rotation)..[["/>]])
				if option.exportAsVisual then
					if option.rgba then
						table.insert(out,tabs.. [[			<geom class="visual" mesh="]]..meshName..[[" rgba="]]..option.rgba..[["/>]])
					else
						table.insert(out,tabs.. [[			<geom class="visual" mesh="]]..meshName..[["/>]])
					end
				else
					table.insert(out,tabs.. [[			<geom mesh="]]..meshName..[[" ]]..condim.. groupString..[[/>]])
				end
			elseif e.elementType==OBJloader.Element.CAPSULE then

				if e.elementSize.y>1e-3 then
					local q=e.tf.rotation
					local vfrom=q*vector3(0,  -e.elementSize.y*0.5,0)+e.tf.translation+init_pos
					local vto=q*vector3(0, e.elementSize.y*0.5,0)+e.tf.translation+init_pos
					table.insert(out,tabs.. [[			<geom type="capsule" fromto="]]..option.vec3str(vfrom)..' '..option.vec3str(vto)..[[" ]]..condim.. densityString..groupString..[[size="]]..tostring(e.elementSize.x*0.5)..[[" />]])
				end
			end
		end
	else
		local COM=bone:localCOM()
		if COM:length()>0.02 then
			table.insert(out,tabs.. [[		<geom fromto="0 0 0 ]]..option.vec3str(COM*2)..[[" name="]]..bone:name()..[[" mass="0.01"  size="0.049" type="capsule"/>]])
		else
			table.insert(out, tabs..[[		<geom fromto="0 0 0 0 0 0.02" name="]]..bone:name()..[[" mass="0.01" size="0.049" type="capsule"/>]])
		end
	end
end
function writeMujocoBody(out, out_asset, loader, bone, level, option)
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
	elseif loader:_get_terrain() then
		local info=string.tokenize(loader:name(), ',')	
		if #info==5 then
			local sizeX=tonumber(info[2])
			local sizeZ=tonumber(info[3])
			local h=tonumber(info[4])
			local fn=info[5]
			local size=vector3(sizeX, 0, sizeZ)
			local pos=loader:bone(1):getOffset()+size
			local quat=quater(math.rad(-90), vector3(0,1,0))*quater(0.7071067812,-0.7071067812,0,0)
			if option.convertToZUP then
				quat=quater(1,0,0,0)
			end

			table.insert(out_asset, [[
			<hfield name="]]..info[1]..
[[" size="]]..tostring(sizeX)..' '..tostring(sizeZ)..' '..tostring(h)..[[ 0.1" file="]]..fn..[[" nrow="0" ncol="0"/>]])

			table.insert(out, makeTabs(level)..[[ <geom type="hfield" group="1" hfield="]]..info[1]..[[" pos="]]..option.vec3str(pos)..[[" quat="]]..option.quat2str(quat)..[[" condim="3" friction="1 .1 .1" contype="1" conaffinity="1"/>]]			)
			return
		end
	end

	local shapePacked=false
	if bone:treeIndex()==1 then
		if bone:HRPjointType(0)==MainLib.VRMLTransform.FREE then
			table.insert(out, tabs..[[
			<joint armature="0" damping="0" limited="false" pos="0 0 0" stiffness="0" type="free"/>]])
		elseif loader:numBone()==2 then
			mujoco_packShape(out, out_asset, loader, bone, level, not option.no_condim, init_pos, option, 1)

			shapePacked=true
		end
	elseif bone:numChannels()>1 and option.useSphericalJoints then
		table.insert(out, tabs..[[
		<joint limited="false" pos="0 0 0" stiffness="0" type="ball"/>]])
	else
		for i=0, bone:numChannels()-1 do
			table.insert(out, tabs..[[
			<joint name="]]..bone:name().."_"..tostring(i)..[[" axis="]]..option.vec3str(bone:axis(i))..[[" pos="0 0 0" range="-2.9 2.9" type="hinge"/>]])
		end
	end

		if option and option.sites[bone:treeIndex()] and #option.sites[bone:treeIndex()]>1 then
			local sites=option.sites[bone:treeIndex()]
			for i, site in ipairs(sites) do
				table.insert(out, tabs..[[
		<site name="]]..site[1]..[[" pos="]]..option.vec3str(site[2])..[[" size="0.01"/>]])
			end
		end
	if not shapePacked then
		mujoco_packShape(out, out_asset, loader, bone, level, false, init_pos, option)
	end

	bone=bone:childHead()
	while bone do
		bone=MainLib.VRMLloader.upcast(bone)
		writeMujocoBody(out, out_asset, loader, bone, level+1, option)
		bone=bone:sibling()
	end

	if writeBody then
		table.insert(out, makeTabs(level)..[[
		</body>]])
	end
end
