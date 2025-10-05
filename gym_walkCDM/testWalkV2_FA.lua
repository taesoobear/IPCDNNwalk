require("config")

package.projectPath='../Samples/classification/'
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

require("module")
arc=require('gym_walk/arc')
require("gym_walk/IPC3d_approx")
require("RigidBodyWin/subRoutines/Constraints")

package.path=package.path..";../Samples/sample_luatorch/lua/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/RigidBodyWin/subRoutines/?.lua" --;"..package.path

useMonopod=false
useQuadruped=false
useDotMapping=false
debugDraw=false

require("gym_walk/pendulumOnlineControl")
fc=require("gym_walk/footCoord")
require("gym_walk/PlannerInfo")
require("gym_walk/ConvexHull2D")
TE=require("gym_walk/COMtrajEdit")
require('gym_walk/pendulumOnlineWalk')
FE=require('gym_walkCDM/featureExtractor')
function handleRendererEvent()
	return 0
end
function frameMove() 
	return 0
end
package.path=package.path..";../Samples/QP_controller/ILIPM/?.lua" --;"..package.path
require("gym_walk/collisionAvoid")

function ctor(no_replan)

	--dbg.startTrace2() -- uncomment this to see where the program crashes. (see work/trace.txt)
	local osm=RE.ogreSceneManager()
	if osm:hasSceneNode("LightNode") then
		local lightnode=osm:getSceneNode("LightNode")
		lightnode:rotate(quater(math.rad(180), vector3(0,1,0)))
	end

	RE.viewpoint():setFOVy(45.000000)
	RE.viewpoint():setZoom(1.000000)
	RE.viewpoint().vpos:set(-1.480754, 117.474929, 324.994381)
	RE.viewpoint().vat:set(2.554417, 84.619466, -7.925027)
	RE.viewpoint():update()

	--RE.FltkRenderer():onCallback('OgreTraceManager')

	planner=PendPlanner(totalTime) 
	planner:createUI()

	--this:create("Button", 'save state', 'save state')
	--this:create("Button", 'load state', 'load state')
	this:create("Check_Button", 'debug draw', 'debug draw')
	this:widget(0):checkButtonValue(false)

	this:updateLayout()

	m_pendulum=IPC3d("gym_walk/cart_pole_ball.wrl",0, 9.8,1.0/30, 30000)
	m_pendulum.skin:setVisible(false)

	planner:initializeGlobal()

	planner.refPendState={
		vector3(0,0.948, 0), -- com
		vector3(0,0,0), -- dotcom
		quater(1,0,0,0), -- ang
		vector3(0,0,0), -- angvel
		vector3(0,0,0), -- zmp
		vector3(0,0,0), -- zmpvel
	}

	if useNN then
		if not os.isFileExist('work/feature_data_info.dat') then
			g_dataset=util.loadTable('work/feature_data.dat')
			local info={ dataInfo=g_dataset.dataInfo}
			util.saveTable(info,'work/feature_data_info.dat')
		end
		g_dataset=util.loadTable('work/feature_data_info.dat')
		python.F('__main__', 'loadNN', 'work/walkrun.net', useDotMapping) -- walk_FlexLoco2020.py. loads walkrun.net
	else
		g_dataset=util.loadTable('work/feature_data.dat')
		local numIDW_samples=30
		g_FA=KNearestInterpolationFast(numIDW_samples, 2.0, 2.0)
		g_FA:learn(g_dataset[1], g_dataset[2])
	end
	


	if false then
		-- debug plot phases
		-- panel at row 3 : a dynamic panel
		mImage=RE.motionPanel():scrollPanel():createPanel()
		RE.motionPanel():scrollPanel():setLabel("dynamic")
		mImage:create(3600, 30)
	end

	if not no_replan then
		planner:replan()
	end

end

onCallback_FA=onCallback

function onCallback(w,uid)
	if w:id()=='debug draw' then
		debugDraw=w:checkButtonValue()
	else
		if w:id()=='use terrain' then
			if w:checkButtonValue() and not g_terrainPos then
				_createTerrain()
			end
		end
		onCallback_FA(w,uid)
	end
end

PendPlanner._replan_pendulum=PendPlanner.replan
function PendPlanner:replan()
	g_timer=util.PerfTimer2()
	g_timer:start()
	self:_replan_pendulum()
	print(g_timer:stop()/1000.0, "ms replan_pend.")
	g_timer:start()
	local COM, ZMP, ZMPVEL=unpack(self.traj)
	local ANG=ZMP:sub(0,0,6,0)

	local sphase={}
	local sf=0
	local sm, ef, sn
	local sw, sp
	if self.timing[1].timing(0)<self.timing[2].timing(0) then
		sw=1 sp=2
		ef=math.round(self.timing[2].timing(0)*30)
		sm=math.round(self.timing[1].timing(0)*30)
		sn=math.round(self.timing[1].timing(1)*30)
	else                                         
		sw=2 sp=1                                
		ef=math.round(self.timing[1].timing(0)*30)
		sm=math.round(self.timing[2].timing(0)*30)
		sn=math.round(self.timing[2].timing(1)*30)
	end



	sphase[sp]=vectorn(ef+1)
	sphase[sw]=vectorn(sn+1)
	sphase[sp]:linspace(g_sp[sp], 1)
	sphase[sw]:range(0, sm+1):linspace(g_sp[sw], 1)
	sphase[sw]:range(sm, sn+1):linspace(0, 1)

	local gnf=math.round(g_global_time*30)
	if mImage and gnf+ef<3600 then
		for i=gnf, gnf+ef-1 do
			local s1=sphase[1](i-gnf)
			local s2=sphase[2](i-gnf)
			s1=math.clamp(s1,0,1)
			s2=math.clamp(s2,0,1)
			mImage:drawBox(TRect(i, 0, i+1, 15), 255,255,255)
			mImage:drawBox(TRect(i, 0, i+1, 15*s1), 0,0,0)
			mImage:drawBox(TRect(i, 15, i+1, 30), 255,255,0)
			mImage:drawBox(TRect(i, 15, i+1, 15+15*s2), 0,0,0)
		end
	end
	local motionType=vectorn(ef+1)
	motionType:range(0, sm+1):linspace(g_sp[3], g_currState)
	motionType:range(sm, ef+1):setAllValue(g_currState)

	g_sp[sw]=sphase[sw](sm)
	g_sp[sp]=sphase[sp](sm)
	g_sp[3]=g_currState

	--local offsetQm=ANG:row(math.round(sf*0.5+ef*0.5)):toQuater(0):offsetQ()
	--g_sp.offsetQm=offsetQm

	g_sphase=sphase
	local vfeature, initialTerrainHeight, regressor=FE.extractFeature(sphase, COM, ZMP, ANG, motionType, sf, ef, g_terrain~=nil)
	local vdata=vectorn()
	local vdotdata=vectorn()
	local localfoot, cdmT
	if self.synthesized then
		local t=self.synthesizedInfo[1]
		local pendZMP=ZMP:row(0):toVector3(0)
		local refTerrainHeight=regressor:getHeight(pendZMP)
		localfoot, cdmT=self:getCDMobservation(t, ANG:row(0):toQuater(0), COM:row(0):toVector3(0), pendZMP, refTerrainHeight, regressor)
		cdmT=CT.vec(cdmT.x, cdmT.y, cdmT.z)
	else
		localfoot=vectorn(12)
		localfoot:setAllValue(0.0)
		cdmT=vectorn(3)
		cdmT:setAllValue(0)
	end
	local feature=cdmT..localfoot..vfeature
	g_timer2=util.PerfTimer2()
	g_timer2:start()
	if g_FA then
		g_FA:mapping(vfeature, vdata)
	else
		python.F('__main__', 'mapping', feature, vdata)	
		if useDotMapping then
			python.F('__main__', 'dotmapping', feature, vdotdata)	
		end
	end
	print(g_timer2:stop()/1000.0, "nn.")
	local info=g_dataset.dataInfo
	local _matdata=matrixn()
	_matdata:fromVector(vdata, vdata:size()/info[1])

	local prev
	if self.synthesized then
		prev={self.synthesized, self.dotsynthesized, self.globalFootDOF, self.globalCOMDOF, switchTime=self.synthesizedInfo[1]}
	end


	local showRawData=false
	--if showRawData==true then
	--	this:findWidget('solve ik'):checkButtonValue(false)
	--end
	if not this:findWidget('solve ik'):checkButtonValue() then
		showRawData=true
	end

	print(g_timer:stop()/1000.0, "ms nn.")

	if not showRawData then
		-- filter CDM delta
		-- 1: local feet, 2: pose, 3: CDM
		local outputCols=g_dataset.dataInfo[2]
		local ccols=outputCols[1]+outputCols[2] 
		assert(outputCols[3]==3)
		assert(outputCols[4]==3)
		--math.medianFilter(3, _matdata:sub(0,0, ccols, ccols+6))
		--math.gaussFilter(5, _matdata:sub(0,0, ccols, ccols+6))
	end


	self.synthesized=matrixn()
	self.synthesized:resample(_matdata, ef-sf+1)
	self.synthesizedInfo={self.switchTime}

	if useDotMapping then
		-- unpack initial height here.
		local _matdotdata=matrixn()

		_matdotdata:fromVector(vdotdata, vdotdata:size()/info[1])
		self.dotsynthesized=matrixn()
		self.dotsynthesized:resample(_matdotdata, ef-sf+1)
	end

	if prev and not showRawData then
		-- stitch synthesized
		removeDiscontinuity(prev[1], prev.switchTime, self.synthesized)
		-- 동작전이 순간에 살짝 불연속성이 있어서 블렌드 스티치를 사용하는 경우
		--removeDiscontinuityBlend(prev[1], prev.switchTime, self.switchTime, self.synthesized)
	end

	local nf=self.synthesized:rows()
	local cons={boolN(nf), boolN(nf), boolN(nf), boolN(nf)}
	do
		-- mark contact status
		local dur=self.plannerInfo.dur
		cons[1]:setAllValue(false)
		cons[2]:setAllValue(false)
		cons[3]:setAllValue(false)
		cons[4]:setAllValue(false)
		
		for icon=0, 3 do
			local ccon=dur[icon][1]
			local ph=dur[icon][2]
			local t=0
			for i=0, ph:size()-1 do
				local et=t+ph(i)
				local sf=math.min(math.ceil(t*30))
				local ef=math.min(math.floor(et*30-0.01), nf-1)
				if sf>=nf then break end

				for iframe=sf, ef do
					cons[icon+1]:set(iframe, ccon)
				end

				t=et
				ccon=not ccon
			end
		end
	end
	self.cons={cons[2], cons[1], cons[4], cons[3] }
	cons=self.cons


	
	local numCon=mSolverInfo.numCon
	self.globalFootDOF=matrixn(nf, 3*numCon)
	self.globalCOMDOF=matrixn(nf, 7)
	
	local outputCols=g_dataset.dataInfo[2]
	local mot
	local dmot
	if not useDotMapping then
		local outputCols=g_dataset.dataInfo[2]
		mot=self.synthesized:sub(0,0,outputCols[1]-1, outputCols[1]+outputCols[2]):copy()
		dmot=matrixn(mot:rows(), mot:cols()+1)
	end

	if debugDraw then
		local traj=matrixn(nf,3)
		for ii=0, nf-1 do
			local pendCOM=self.traj[1]:row(ii):toVector3(0)
			local pendZMP=self.traj[2]:row(ii):toVector3(0)
			refTerrainHeight=regressor:getHeight(pendZMP)
			pendCOM.y=pendCOM.y+refTerrainHeight
			traj:row(ii):setVec3(0, pendCOM)
		end
		dbg.namedDraw('Traj', traj*100, 'terr_adaptive_com', 'blueCircle', thickness, 'QuadListV' )
	end

	-- convert localfoot to globalfoot
	for ii=0, nf-1 do
		local localfoot=self.synthesized:row(ii)
		local footDOF=self.globalFootDOF:row(ii)
		local pendROOTQ=self.traj[2]:row(ii):toQuater(6)
		local pendCOM=self.traj[1]:row(ii):toVector3(0)
		local pendZMP=self.traj[2]:row(ii):toVector3(0)

		refTerrainHeight=regressor:getHeight(pendZMP)

		pendCOM.y=pendCOM.y+refTerrainHeight
		pendZMP.y=pendZMP.y+refTerrainHeight
		local pend2Dtf=transf(pendROOTQ, pendCOM)
		local pend2Dtf_foot=transf(pendROOTQ:rotationY(), pendZMP)
		pend2Dtf_foot.translation.y=0

		for i=0, numCon-1 do
			local gfoot=pend2Dtf_foot:toGlobalPos(localfoot:toVector3(i*3))
			gfoot.y=gfoot.y+regressor:getHeight(gfoot)
			footDOF:setVec3(i*3, gfoot)
		end

		-- 1: local feet, 2: pose, 3: CDM
		local ccols=outputCols[1]+outputCols[2] 
		local ncols=ccols+outputCols[3] 
		local data=localfoot
		assert(ncols+3==data:size())
		local cdmR=data:toVector3(ccols)
		local cdmT=data:toVector3(ncols)
		local lcdmtf=transf()
		lcdmtf.rotation:setRotation(cdmR)
		lcdmtf.translation:assign(cdmT)
		local cdmtf=pend2Dtf*lcdmtf
		MotionDOF.setRootTransformation(self.globalCOMDOF:row(ii), cdmtf)
	end
	if g_FDtracker and g_FDtracker.force then
		local out=g_FDtracker:processInput(self.globalCOMDOF, self.switchTime)



		for i=0, out:rows()-1 do
			local q =out:row(i):toQuater(3)
			q=q*quater(math.rad(5), vector3(0,1,0))
			out:row(i):setQuater(3, q)
		end

		self.globalCOMDOF:assign(out)
	end

	if not useDotMapping then
		for ii=0, nf-1 do
			local cdmtf= MotionDOF.rootTransformation(self.globalCOMDOF:row(ii))
			local pose=mot:row(ii):slice(1,0)
			local lposetf=transf()
			lposetf.rotation:setRotation(pose:toVector3(0))
			lposetf.translation:assign(pose:toVector3(3))
			pose=CT.vec(0)..pose
			MotionDOF.setRootTransformation(pose, cdmtf*lposetf)
			mot:row(ii):assign(pose)
		end
	end
	if false then
		Imp.ChangeChartPrecision(50)
		RE.motionPanel():scrollPanel():addPanel(self.globalCOMDOF:column(1), 0.5, 2)
		local gnf=math.round(g_global_time*30)
		RE.motionPanel():scrollPanel():setLastPanelXOffset(gnf)
	end


	if not useDotMapping then
		self.dotsynthesized={
			MotionDOF.calcDerivative(self.globalCOMDOF, 30), -- dotcdm
			MotionDOF.calcDerivative(mot, 30) -- dotmot
		}
	end

	if showRawData then
		return 
	end
	if prev then
		-- stitch globalFootDOF
		removeDiscontinuity(prev[3], prev.switchTime, self.globalFootDOF)
		removeDiscontinuityTF(prev[4], prev.switchTime, self.switchTime, self.globalCOMDOF)
	end
	
	-- remove leg-intersection by adjusting swing foot trajectory
	dbg.erase('Arrow', 'arrow1')
	dbg.erase('Arrow', 'arrow2')
	local mcon={ boolN(), boolN()}
	mcon[1]:_or(cons[1],cons[2])
	mcon[2]:_or(cons[3],cons[4])
	for ileg=1, 2 do
	--if false then
		local oleg=math.fmod(ileg, 2)+1
		local icon=(ileg-1)*2
		--local con=cons[icon+1]
		local con=mcon[ileg]
		local ii=intIntervals()
		ii:runLengthEncode(con)
		do
			local icontact=0
			local si=ii:startI(icontact)
			local ei=ii:endI(icontact)
			if si==0 then

			else
				local mf=math.round(si*0.5+ei*0.5)
				local spprtPos=self.globalFootDOF:row(mf):toVector3(icon*3)
				local spprtPos2=self.globalFootDOF:row(mf):toVector3((icon+1)*3)
				local pendRotY=self.traj[2]:row(mf):toQuater(6):rotationY()

				local ocon=(oleg-1)*2
				if debugDraw then
					local thickness=10
					dbg.namedDraw('Traj', self.globalFootDOF:sub(0,0,ocon*3, (ocon+1)*3)*100, 'before', 'blueCircle', thickness, 'QuadListY' )
				end
				--local origtraj=avoidObstacle(self.globalFootDOF:sub(0,0,ocon*3, (ocon+1)*3), ileg==1, spprtPos, pendRotY)
				--applyDiffTo(origtraj, self.globalFootDOF:sub(0,0,ocon*3, (ocon+1)*3), self.globalFootDOF:sub(0,0,(ocon+1)*3, (ocon+2)*3))
				--origtraj=avoidObstacle(self.globalFootDOF:sub(0,0,(ocon+1)*3, (ocon+2)*3), ileg==1, spprtPos, pendRotY)
				--applyDiffTo(origtraj, self.globalFootDOF:sub(0,0,(ocon+1)*3, (ocon+2)*3), self.globalFootDOF:sub(0,0,ocon*3, (ocon+1)*3))

				--origtraj=avoidObstacle(self.globalFootDOF:sub(0,0,ocon*3, (ocon+1)*3), ileg==1, spprtPos2, pendRotY)
				--applyDiffTo(origtraj, self.globalFootDOF:sub(0,0,ocon*3, (ocon+1)*3), self.globalFootDOF:sub(0,0,(ocon+1)*3, (ocon+2)*3))
				local origtraj=avoidObstacle(self.globalFootDOF:sub(0,0,(ocon+1)*3, (ocon+2)*3), ileg==1, spprtPos2, pendRotY)
				applyDiffTo(origtraj, self.globalFootDOF:sub(0,0,(ocon+1)*3, (ocon+2)*3), self.globalFootDOF:sub(0,0,ocon*3, (ocon+1)*3))

				if debugDraw then
					--dbg.namedDraw('Traj', self.globalFootDOF:sub(0,0,ocon*3, (ocon+1)*3)*100, 'after', 'redCircle', 9, 'QuadListY' )
					dbg.namedDraw('Traj', self.globalFootDOF:sub(0,0,(ocon+1)*3, (ocon+2)*3)*100, 'after', 'redCircle', 9, 'QuadListY' )
				end
			end
		end
	end

	local function clearGroundTraj_old(gfd, icon, si, ei)
		for i=si, ei-1 do
			local data=gfd:row(i)
			clearGround(data, icon)
		end
	end

	-- remove foot-sliding
	for icon=0,3 do
		local con=cons[icon+1]
		local ii=intIntervals()
		ii:runLengthEncode(con)
		local fi=math.round(self.switchTime*30)
		for icontact=0, math.min(ii:size()-1, 1) do
			local si=ii:startI(icontact)
			local ei=ii:endI(icontact)
			if si==0 then
				-- continued con
				local pdata=self.globalFootDOF:row(si)
				local err=pdata:toVector3(icon*3)-self.globalFootDOF:row(ei-1):toVector3(icon*3)
				for i=si, ei-1 do
					local data=self.globalFootDOF:row(i)
					data:setVec3(icon*3, pdata:toVector3(icon*3))
				end

				if (ii:size()==icontact+2) then
					--local nsi=ii:startI(icontact+1)
					local nsi=fi
					for i=ei, nsi-1 do
						local data=self.globalFootDOF:row(i)
						local s=sop.smoothMapB(i, ei-1, nsi-1, 1,0)
						data:setVec3(icon*3, data:toVector3(icon*3)+err*s)
					end
					clearGroundTraj(self.globalFootDOF, icon, ei-1, self.globalFootDOF:rows())
				end
			else
				local avgPos=vector3(0,0,0)
				for i=si, ei-1 do
					local data=self.globalFootDOF:row(i)
					avgPos:radd(data:toVector3(icon*3))
				end
				avgPos:scale(1.0/(ei-si))
				avgPos.y=0.05
				if g_terrain then
					avgPos.y=g_terrain:_getHeight(avgPos)+0.05
				end

				if true then
					local ileg=math.floor(icon/2)+1
					local ii=math.round(si*0.5+ei*0.5)
					local pendROOTQ=self.traj[2]:row(ii):toQuater(6)
					local pendZMP=self.traj[2]:sampleVec(ii,0)
					local invrot=pendROOTQ:rotationY():inverse()
					local p1=(invrot*avgPos).x
					local p2=(invrot*pendZMP).x
					local thr=0.04
					if ileg==1 then
						--avgPos:radd(pendROOTQ:rotationY()*vector3(0.01,0,0))
						if p1<p2+thr then
							avgPos:radd(pendROOTQ:rotationY()*vector3(p2+thr-p1,0,0))
						end
					else
						--avgPos:radd(pendROOTQ:rotationY()*vector3(-0.01,0,0))
						if p1>p2-thr then
							avgPos:radd(pendROOTQ:rotationY()*vector3(-(p1-p2+thr),0,0))
						end
					end
				end

				local err1=avgPos-self.globalFootDOF:row(si):toVector3(icon*3)
				local err2=avgPos-self.globalFootDOF:row(ei-1):toVector3(icon*3)
				for i=0, si-1 do
					local data=self.globalFootDOF:row(i)
					local s=sop.smoothMapA(i, 0, si, 0,1)
					data:setVec3(icon*3, data:toVector3(icon*3)+err1*s)
				end
				clearGroundTraj(self.globalFootDOF, icon, 0, si)
				local nf=self.globalFootDOF:rows()
				for i=ei, nf-1 do
					local data=self.globalFootDOF:row(i)
					local s=sop.smoothMapB(i, ei-1, nf-1, 1,0)
					data:setVec3(icon*3, data:toVector3(icon*3)+err2*s)
				end
				clearGroundTraj(self.globalFootDOF, icon, ei, nf)
				local nf=self.globalFootDOF:rows()
				for i=si, ei-1 do
					local data=self.globalFootDOF:row(i)
					data:setVec3(icon*3, avgPos)
				end


			end
		end
	end

	-- remove COM constraint error 
	for ileg=1, 2 do
		local oleg=math.fmod(ileg, 2)+1
		local icon=(ileg-1)*2
		local con=mcon[ileg]
		local ii=intIntervals()
		ii:runLengthEncode(con)
		do
			local icontact=0
			local si=ii:startI(icontact)
			local ei=ii:endI(icontact)
			if si==0 then
			else
				assert(si<=self.switchTime*30 and ei>=self.switchTime*30)
				local spprtPos=self.globalFootDOF:row(si+1):toVector3(icon*3)
				local spprtPos2=self.globalFootDOF:row(si+1):toVector3((icon+1)*3)

				local ocon=(oleg-1)*2

				local midf=math.round(self.switchTime*30)
				local COM=self.globalCOMDOF:row(midf):toVector3(0)
				local footMidPos=spprtPos*0.5+spprtPos2*0.5
				local maxDist=0.95
				local minDist=0.8
				local COMerr
				if footMidPos:distance(COM)>maxDist then
					local err=COM-footMidPos
					err:normalize()
					err:scale(maxDist)
					COMerr=COM-footMidPos-err
				elseif footMidPos:distance(COM)<minDist then
					local err=COM-footMidPos
					err:normalize()
					err:scale(minDist)
					COMerr=COM-footMidPos-err
				end

				if COMerr then
					for i=0, midf-1 do
						local s=sop.smoothMap(i, 0, midf, 0,1)
						local COM=self.globalCOMDOF:row(i):toVector3(0)
						self.globalCOMDOF:row(i):setVec3(0, COM+COMerr*s)
					end
					local nf=self.globalFootDOF:rows()
					for i=midf, nf-1 do
						local s=sop.smoothMap(i, midf, nf-1, 1,0)
						local COM=self.globalCOMDOF:row(i):toVector3(0)
						self.globalCOMDOF:row(i):setVec3(0, COM+COMerr*s)
					end
				end
			end
		end
	end
	
end

PendPlanner._initializeGlobal_pendulumOnlineWalk=PendPlanner.initializeGlobal
function PendPlanner:initializeGlobal()
	self:_initializeGlobal_pendulumOnlineWalk()
	g_prevDesiredPose=g_prevPose:copy()
	g_sp={0,0, g_currState, offsetQm=quater(1,0,0,0)}
end

function PendPlanner:getCDMobservation(t, pendROOTQ, pendCOM, pendZMP, refTerrainHeight, regressor)
	pendCOM.y=pendCOM.y+refTerrainHeight
	pendZMP.y=pendZMP.y+refTerrainHeight
	--dbg.draw('Sphere', pendCOM*100, 'pendCOM', 'red', 15)
	--dbg.draw('Sphere', pendZMP*100, 'pendZMP', 'red', 15)
	local pend2Dtf=transf(pendROOTQ, pendCOM)
	local pend2Dtf_foot=transf(pendROOTQ:rotationY(), pendZMP)
	pend2Dtf_foot.translation.y=0

	local pendFrameRate=30
	local ii=t*pendFrameRate

	local numCon=mSolverInfo.numCon
	local localfoot=vectorn(numCon*3)
	for i=0, numCon-1 do
		local gfoot=self.globalFootDOF:sampleVec(ii, i*3)
		gfoot.y=gfoot.y-regressor:getHeight(gfoot)
		localfoot:setVec3(i*3, pend2Dtf_foot:toLocalPos(gfoot))
		--dbg.draw('Sphere', gfoot*100, 'gfoot'..i, 'red', 5)
	end
	local vcdmtf=vectorn()
	self.globalCOMDOF:sampleRow(ii, vcdmtf)
	local cdmtf=MotionDOF.rootTransformation(vcdmtf)

	local lcdmtf=pend2Dtf:inverse()*cdmtf
	lcdmtf.rotation:align(quater(1,0,0,0))
	return localfoot, lcdmtf.translation 
end
function PendPlanner:drawFrame(t)
	if false then
		--local spprtCoord=self.spprtCoord[#self.spprtCoord]
		--local spprtCoord=self.spprtCoord[#self.spprtCoord-1]
		local spprtCoord=self.plannerInfo.spprtCoord
		dbg.namedDraw('Axes', spprtCoord[1], 'p1', 100)
		dbg.namedDraw('Axes', spprtCoord[2], 'p2', 100)
	end
	--drawFrame_old(self,t)

	local pendFrameRate=30
	local ii=t*pendFrameRate


	local pendCOM=self.traj[1]:sampleVec(ii,0)
	local pendROOTQ=self.traj[2]:sampleQuat(ii,6)
	local pend2Dtf=transf(pendROOTQ, pendCOM)
	--local pendZMP=self.traj[2]:sampleVec(ii,0)
	--local pendCOMVEL=self.traj[1]:sampleVec(ii,3)
	--local pendANGVEL=self.traj[2]:sampleVec(ii,3)
	--local pend2Dtf_foot=transf(pendROOTQ:rotationY(), pendZMP)


	RE.output2('syn sample', ii,self.synthesizedInfo[1]*30)

	local data=vectorn()
	local outputCols=g_dataset.dataInfo[2]
	-- localfoot is not necessary here.
	--self.synthesized:sampleRow(ii, data)
	--local ccols=0
	--local ncols=outputCols[1]
	--local localfoot=data:slice(ccols, ncols) ccols=ncols ncols=ncols+outputCols[2] 
	self.synthesized:sub(0,0,outputCols[1],0):sampleRow(ii, data)
	local ccols=0
	local ncols=outputCols[2]
	local pose=data:slice(ccols, ncols) ccols=ncols ncols=ncols+outputCols[3] 
	assert(ncols+3==data:size())	
	local vcdmtf=vectorn()
	self.globalCOMDOF:sampleRow(ii, vcdmtf)
	local cdmtf=MotionDOF.rootTransformation(vcdmtf)

	local lposetf=transf()
	lposetf.rotation:setRotation(pose:toVector3(0))
	lposetf.translation:assign(pose:toVector3(3))

	pose=CT.vec(0)..pose
	MotionDOF.setRootTransformation(pose, cdmtf*lposetf)

	RE.output2('sphase', g_sphase[1]:sample(ii), g_sphase[2]:sample(ii))

	local numCon=mSolverInfo.numCon
	--local footDOF=vectorn(localfoot:size())
	local footDOF=vectorn()
	self.globalFootDOF:sampleRow(ii, footDOF)

	if debugDraw then
		for i=0, numCon-1 do
			--footDOF:setVec3(i*3, pend2Dtf_foot:toGlobalPos(localfoot:toVector3(i*3)))
			if self.cons[i+1](math.round(ii)) then
				dbg.draw('Sphere', footDOF:toVector3(i*3)*100, 'ball'..i,'blue',5)
			else
				dbg.draw('Sphere', footDOF:toVector3(i*3)*100, 'ball'..i,'red',5)
			end
		end
	end

	local COM=cdmtf.translation
	local ROOTQ=cdmtf.rotation
	local gridpos=math.floor(COM.z/4)
	local gridposx=math.floor(COM.x/4)
	local bgnode=RE.ogreSceneManager():getSceneNode("BackgroundNode")
	bgnode:setPosition(vector3(gridposx*4*100,0,gridpos*4*100))


	if debugDraw then
		dbg.draw('Axes', transf(ROOTQ, COM), 'COM', 100, 5)
	end
	local renderPose
	if this:findWidget('solve ik'):checkButtonValue() then
		assert(COM.y==COM.y)

		local comdof=vectorn(7+6)
		--comdof:range(0,7):assign(pose:range(0,7))
		comdof:setVec3(0, COM) comdof:setQuater(3, ROOTQ)
		local dpose

		if useDotMapping then
			local dotdata=vectorn()
			self.dotsynthesized:sampleRow(ii, dotdata)
			local ANGVEL=pend2Dtf.rotation*dotdata:toVector3(0)
			local COMVEL=pend2Dtf.rotation*dotdata:toVector3(3) 
			comdof:setVec3(10, COMVEL)
			comdof:setVec3(7, ANGVEL)
			dpose=dotdata:slice(6,0):copy()	
		else
			local dotdata=vectorn()
			self.dotsynthesized[1]:sampleRow(ii, dotdata)
			comdof:setVec3(10, ROOTQ*dotdata:toVector3(0))
			comdof:setVec3(7, ROOTQ*dotdata:toVector3(3))
			dpose=vectorn()
			self.dotsynthesized[2]:sampleRow(ii, dpose)
		end
		--local footDOF=vectorn(3*numCon)
		--for i=0,numCon-1,2 do
		--	--footDOF:setVec3(i*3, footPos(i+1))
		--	--footDOF:setVec3((i+1)*3, footPos(i))
		--	footDOF:setVec3(i*3, footPos(i))
		--	footDOF:setVec3((i+1)*3, footPos(i+1))
		--end
		local velScale=1
		local IKconfig={
			wVelRoot=0.95,
			wCOMy=0.3,
			wFullVel=0.1*velScale,
			wFullVelPose=0.02,
			wMM=1,
			wHead_y=0.0001,
			wHead_z=0.1,
			--effWeightsY=2,
			effWeightsY=0.5,
			v2_max=50,
		}

		local effWeights=CT.vec(1,1,1,1)
		local effY=2
		--if g_currState==2 then
		--	IKconfig.wMM=0.5
		--	IKconfig.effWeightsY=IKconfig.effWeightsY*0.5
		--	effWeights=effWeights*0.1
		--	effY=4
		--end


		g_timer2=util.PerfTimer2()
		g_timer2:start()

		if true then
			fc.solveIK(mSolverInfo, g_prevPose, dpose, pose, comdof, footDOF, effWeights ,IKconfig)
		else
			g_prevpose:assign(pose)
		end
		print(g_timer2:stop()/1000.0, "ik.")
		renderPose=g_prevPose:copy()
		fc.solveIK_postprocess(mSolverInfo, renderPose, comdof, footDOF,effWeights,
		{
			wCOMy=0.3,
			wMM=0.1,
			wHead_y=0.0001,
			effWeightsY=effY,
			--v2_max=5,
		}
		)
	else
		g_prevPose=pose
		renderPose=pose
	end


	mMot.skin:setPoseDOF(renderPose);
	if debugDraw then
		mMot.skin:setMaterial('lightgrey_transparent')
	else
		mMot.skin:setMaterial('lightgrey')
	end
	local attachCamera=true
	if attachCamera then
		mCOMfilter:setCurrPose(CT.vec(COM))
		local curPos= mCOMfilter:getFiltered():toVector3(0)*100
		curPos.y=0
		if g_prevRootPos then
			RE.viewpoint().vpos:assign(RE.viewpoint().vpos+curPos-g_prevRootPos)
			RE.viewpoint().vat:assign(RE.viewpoint().vat+curPos-g_prevRootPos)
			RE.viewpoint():update()     
		end
		g_prevRootPos=curPos:copy()
	end
end
function isIn(p, a, b)
	return p>=a-0.001 and p<=b+0.001
end
function removeDiscontinuity(prevData, switchTime, newData)
	-- remove discontinuity
	-- modifies newData
	local prevsf=switchTime*30
	local prevef=prevData:rows()-2-(math.floor(prevsf)-prevsf)
	if prevef-prevsf> newData:rows()/2 -1 then
		prevef=prevsf+math.floor(newData:rows()/4 ) -1 
	end
	assert(prevef>prevsf)
	if false then
		-- method 1: blending
		for i=prevsf, prevef+0.001, 1 do
			local w=math.pow(sop.smoothMap(i, prevsf, prevef, 1, 0),2)
			local data=vectorn()
			prevData:sampleRow(i, data)
			newData:row(i-prevsf):interpolate(w, newData:row(i-prevsf), data)
		end
	else
		-- method 2: c0 stitching
		local data=vectorn()
		prevData:sampleRow(prevsf, data)
		local err=data-newData:row(0)
		for i=prevsf, prevef+0.001, 1 do
			local w=math.pow(sop.smoothMap(i, prevsf, prevef, 1, 0),2)
			newData:row(i-prevsf):radd(err*w)
		end
	end
end
function removeDiscontinuityBlend(prevData, prevswitchTime, switchTime, newData)
	-- remove discontinuity
	-- modifies newData
	local prevsf=prevswitchTime*30
	local ef=switchTime*30
	if prevsf+ef >prevData:rows()-1 then
		ef=math.round(prevData:rows()-1-prevsf)-1
	end
	for i=0, ef+0.001, 1 do
		local w=math.pow(sop.smoothMap(i, 0, ef, 1, 0),2)
		local data=vectorn()
		prevData:sampleRow(i+prevsf, data)
		newData:row(i):interpolate(w, newData:row(i), data)
	end
end
function removeDiscontinuityTF(prevData, prevswitchTime, switchTime, newData)
	-- remove discontinuity
	-- modifies newData
	local prevsf=prevswitchTime*30
	local ef=switchTime*30
	if false then
		if prevsf+ef >prevData:rows()-1 then
			ef=math.round(prevData:rows()-1-prevsf)-1
		end
		for i=0, ef+0.001, 1 do
			local w=math.pow(sop.smoothMap(i, 0, ef, 1, 0),2)
			local data=vectorn()
			prevData:sampleRow(i+prevsf, data)
			newData:row(i):interpolate(w, newData:row(i), data)
			local q=newData:row(i):toQuater(3)
			q:normalize()
			newData:row(i):setQuater(3, q)
		end
	else
		local data=vectorn()
		prevData:sampleRow(prevsf, data)
		local err=data:toVector3(0)-newData:row(0):toVector3(0)
		local errQ=quater()
		errQ:difference(newData:row(0):toQuater(3), data:toQuater(3))
		-- method 2: c0 stitching
		for i=0, ef+0.001, 1 do
			local w=math.pow(sop.smoothMap(i, 0, ef, 1, 0),2)
			--local w=sop.smoothMap(i, 0, ef, 1, 0)

			local T=newData:row(i):toVector3(0)
			local Q=newData:row(i):toQuater(3)
			newData:row(i):setVec3(0, T+err*w)
			local errw=errQ:copy()
			errw:scale(w)
			newData:row(i):setQuater(3, errw*Q)
		end
	end
end

