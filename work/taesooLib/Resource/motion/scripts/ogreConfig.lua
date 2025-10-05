depthShadow=false -- these setting will be auto-detected below
stencilShadow=true

local osm=RE.ogreSceneManager()
if osm then
	local st=osm:getShadowTechnique()
	if st==34 then
		stencilShadow=false
	elseif st==37 then
		stencilShadow=false
		depthShadow=true
	end
end


function shadowMaterial(input)
   if depthShadow then
      return input.."_depthshadow"
   end
   return input
end

if RE.getOgreVersionMinor()>=12 then 
	numMainLights=2 -- 5이상이어야 품질이 좋지만, m1 macbook에서 너무 느림
	if RE.useSeperateOgreWindow and RE.useSeperateOgreWindow() then
		numMainLights=5
	end
	if depthShadow then
		numMainLights=1 -- no difference
	end
elseif RE.getOgreVersionMinor()<=3 then
	numMainLights=5
end
