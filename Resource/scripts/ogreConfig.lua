depthShadow=false -- these setting will be auto-detected below
stencilShadow=true

local st=RE.ogreSceneManager():getShadowTechnique()
if st==34 then
	stencilShadow=false
elseif st==37 then
	stencilShadow=false
	depthShadow=true
end


function shadowMaterial(input)
   if depthShadow then
      return input.."_depthshadow"
   end
   return input
end

if RE.getOgreVersionMinor()>=12 then
	numMainLights=5
	if depthShadow then
		numMainLights=1 -- no difference
	end
end
