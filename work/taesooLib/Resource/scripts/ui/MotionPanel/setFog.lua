
dofile('../Resource/scripts/ogreConfig.lua')
if RE.ogreSceneManager().setFog then
	RE.ogreSceneManager():setFog( 0.77,0.92,1, 0.0,1200, 1800 )
end
