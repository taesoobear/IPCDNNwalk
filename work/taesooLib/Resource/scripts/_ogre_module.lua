-- ogre-next specific functions 

-- mainlight colors, filllight colors. and shadow color.  see default values
function RE.createLight(ambient, light1D, light1S, lightFD, lightFS, sc, _optionalNumMainLights, _optionalLightVariance) 
	if not ambient then ambient=0.4 end
	if not light1D then light1D=0.9 end
	if not light1S then light1S=0.8 end
	if not lightFD then lightFD=0.3 end
	if not lightFS then lightFS=0.3 end
	if not sc then sc=0.95 end
	local f=io.open(RE.taesooLibPath()..'Resource/scripts/ogreConfig_personal.lua','r')
	if f then
		f:close()
		dofile(RE.taesooLibPath()..'Resource/scripts/ogreConfig_personal.lua')
	else
		dofile(RE.taesooLibPath()..'Resource/scripts/ogreConfig.lua')
	end
	if not _optionalNumMainLights then _optionalNumMainLights =numMainLights end
	if not _optionalLightVariance then _optionalLightVariance=0.02 end

	numMainLights=_optionalNumMainLights
	local lightVar=_optionalLightVariance


	local rootnode =RE.ogreRootSceneNode()
	if rootnode then
		local lightnode=RE.createChildSceneNode(rootnode, "LightNode")
		RE.ogreSceneManager():setAmbientLight(ambient, ambient, ambient)

		if RE.getOgreVersionMinor()<=3 then

			local function randomNormal()
				return (math.random()-0.5)*2
			end

			assert(stencilShadow)

			local lightOD=0.0
			local lightOS=0.0


			for i=1,numMainLights do
				local light
				if i==1 then
					light=RE.ogreSceneManager():createLight("Mainlight")
				else
					light=RE.ogreSceneManager():createLight("Mainlight"..i)
				end
				light:setType("LT_DIRECTIONAL")

				local node=lightnode:createChildSceneNode("mainlightnode"..i)
				local dir=vector3(-0.5+lightVar*(randomNormal()),-0.7,0.5+lightVar*(randomNormal()))
				dir:normalize()
				node:setDirection(dir)
				node:attachObject(light)
				light:setDiffuseColour(light1D,light1D,light1D)
				light:setSpecularColour(light1S,light1S,light1S)
				light:setCastShadows(true)
			end
			light=RE.ogreSceneManager():createLight("FillLight")
			local node=lightnode:createChildSceneNode("filllightnode")
			local dir=vector3(0.5,0.7,-0.5)
			dir:normalize()
			node:setDirection(dir)
			node:attachObject(light)
			light:setType("LT_DIRECTIONAL")
			light:setDiffuseColour(lightFD,lightFD,lightFD)
			light:setSpecularColour(lightFS,lightFS,lightFS)
			light:setCastShadows(false)
		else
			print('not supported')
		end
	end

	local useESM=false
	RE.renderer():setupShadowNode(useESM)
end

-- for faster rendering
function RE.turnOffSoftShadows()

	if RE.getOgreVersionMinor()<=3 then
		-- not ported yet
		return
	end

	-- replace the default light setting in (createLight_default.lua)
	--
	rootnode =RE.ogreRootSceneNode()
	lightnode=RE.createChildSceneNode(rootnode, "LightNode")
	light=RE.ogreSceneManager():createLight("MainlightNew")
	light:setType("LT_DIRECTIONAL")
	light:setDiffuseColour(0.8,0.8,0.8)
	light:setSpecularColour(0.2,0.2,0.2)
	light:setCastShadows(true)
	local sc=0.8
	RE.ogreSceneManager():setShadowColour(sc,sc,sc)

	if RE.getOgreVersionMinor()<=12 then
		light:setDirection(-0.5,-0.7,0.5)
		lightnode:attachObject(light)
	else
		local node=lightnode:createChildSceneNode("mainlightnodenew")
		node:setDirection(vector3(-0.5,-0.7, 0.5))
		node:attachObject(light)
	end
end

function RE.turnOffShadows()
	RE.ogreSceneManager():setShadowTechnique(0)
end
function RE.turnOnShadows()
	RE.ogreSceneManager():setShadowTechnique(18)
end
TextArea=LUAclass()
function TextArea:__init(overlayname, containername, textareaname,x,y,sx,sy ,fontsize)
   if RE.motionPanelValid()==false then return end
   self.overlay=Ogre.createOverlay(overlayname)
   self.overlay:setZOrder(500)
   self.container=Ogre.createContainer(x,y,sx,sy, containername)
   --self.container:setMaterialName("redCircle")
   self.container:setParameter("border_size", "0 0 0 0")
   
   self.element=Ogre.createTextArea(textareaname, 720-6,480-6, 2, 2, fontsize, textareaname,true);
   self.element:setParameter("colour_top", "0 0 0")
   self.element:setParameter("colour_bottom", "0 0 0")	
   --	self.element:setParameter("font_name", "BlueHighway_mod");	
   --	self.element:setParameter("font_name", "IronMaiden")
   self.element:setParameter("font_name", "StarWars")
   --	self.element:setParameter("font_name", "Ogre")
   self.container:addChild(self.element)
   
   self.overlay:add2D(self.container)
   self.overlay:show()

   self.overlayname=overlayname
   self.containername=containername
   self.textareaname=textareaname
end

function TextArea:setCaption(caption)
   if self.overlay ~= nil then
      self.element:setCaption(caption)
   end
end

function TextArea:kill()
   if self.overlay~=nil then
      Ogre.destroyOverlayElement(self.textareaname)-- textArea
      Ogre.destroyOverlayElement(self.containername) -- container
      --      Ogre.destroyAllOverlayElements()
      Ogre.destroyOverlay(self.overlayname)
      self.overlay=nil
   end
end
function TextArea:__finalize()
   self:kill()
end
function dbg.drawTraj(objectlist, matrix, nameid, color, thickness, linetype)
	if nameid==nil then
		nameid=RE.generateUniqueName()
	end
	linetype=linetype or "LineList"
	if linetype=='ArrowsM' then
		local c=color or dbg.linecolor
		objectlist:registerObject(nameid, 'ColorWidthBillboardLineList', c, dbg.linesToArrows(matrix, c, thickness or 0, 100), 0)
	elseif linetype=='Arrows' then
		local c=color or dbg.linecolor
		objectlist:registerObject(nameid, 'ColorWidthBillboardLineList', c, dbg.linesToArrows(matrix, c, thickness or 0, 1), 0)
	else
		if linetype=='PointList' and RE.getOgreVersionMinor()<=3 then
			-- pointlist doesnt' work on ogre2.
			if not RE.taesooLibVersion then
				if color:sub(1,5)=='Point' then
					thickness=tonumber(color:sub(6))*0.2
					color='white'
				elseif color=='' then
					thickness=1.5
				end
				objectlist:registerObject(nameid, 'QuadListV', color or dbg.linecolor, matrix:sub(0,0,-3,0), thickness or 0)
			else
				objectlist:registerObject(nameid, linetype, color or dbg.linecolor, matrix, thickness or 0)
			end
		else
			objectlist:registerObject(nameid, linetype, color or dbg.linecolor, matrix, thickness or 0)
		end
	end
end
