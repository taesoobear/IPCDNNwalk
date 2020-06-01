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

