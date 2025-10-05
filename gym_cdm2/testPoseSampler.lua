--require('gym_deepmimic/module/DeepMimicLoader')
require('module')
require('common')
require('subRoutines/Timeline')
fc=require('gym_cdm2/module/CDMTraj')
require("control/SDRE")
--motion='run2'
--motion='run3'
--motion='jog'
--motion='jog2'
--motion='runjump'
--motion='runjump2'
--motion='walk'
--motion='walk2' if not param then param={envName='walk2-v6'} end
motion='walk3' if not param then param={envName='walk2-v1'} end
--motion='run180'if not param then param={envName='run180l2-v5'} end

--motion='run90'
--motion='run90R'
--motion='fastwalk'
--motion='backflip'
if param and param.motionId then
	motion=param.motionId
end

RL_step=1/60
model=require('gym_cdm2/defaultRLsetting')(motion)
motionInfo=model._motionInfo
assert(model.frame_rate==30)

assert(useHanyangInsteadOfHyunwoo)

feet_config={
	-- this local positions have to be same as those defined in showHuman.lua (input.limbs[1][3] and input.limbs[2][3]))
	toe=input.limbs[1][3], 
	heel=input.limbs[2][3],
}
testReconstruction=false


if EventReceiver then
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		self.cameraInfo={}
	end
end
function ctor()
	mEventReceiver=EVR()

	RE.turnOffSoftShadows()
	mLoader=MainLib.VRMLloader ( motionInfo.skel)
	mLoader:setVoca(input.bones)
	local initial_height=0
	if not motionInfo.isDeepmimicMot then
		--mMotionDOFcontainer=MotionDOFcontainer(mLoader.dofInfo, "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof")
		mMotionDOFcontainer=MotionDOFcontainer(mLoader.dofInfo, motionInfo.fn)
		mMotionDOF=mMotionDOFcontainer.mot

		mMotionDOFcontainer=nil
		motionInfo.fixDOF(mLoader, mMotionDOF)
	else
		--[[
		dog3d_canter.txt
		dog3d_pace.txt
		dog3d_trot.txt
		humanoid3d_backflip.txt
		humanoid3d_cartwheel.txt
		humanoid3d_crawl.txt
		humanoid3d_dance_a.txt
		humanoid3d_dance_b.txt
		humanoid3d_getup_facedown.txt
		humanoid3d_getup_faceup.txt
		humanoid3d_jump.txt
		humanoid3d_kick.txt
		humanoid3d_punch.txt
		humanoid3d_roll.txt
		humanoid3d_run.txt
		humanoid3d_spinkick.txt
		humanoid3d_spin.txt
		humanoid3d_walk.txt
		]]
		--local fn='gym_cdm/motions/humanoid3d_spinkick.txt'
		--local fn='gym_cdm/motions/humanoid3d_backflip.txt'
		--local fn='gym_cdm/motions/humanoid3d_jump.txt'
		--local fn='gym_cdm/motions/humanoid3d_run.txt' 
		--local fn='gym_cdm/motions/humanoid3d_roll.txt'
		--local fn='gym_cdm/motions/humanoid3d_dance_a.txt'
		--local fn='gym_cdm/motions/humanoid3d_dance_b.txt'
		--local fn='gym_cdm/motions/humanoid3d_crawl.txt'
		--local fn='gym_cdm/motions/humanoid3d_cartwheel.txt'
		info1=MainLib.DeepMimicLoader('gym_deepmimic/module/humanoid3d.txt', false)
		local loop, frameTime
		info1.mot,loop, frameTime=MainLib.DeepMimicMotion(info1, motionInfo.fn)

		mMotion, mMotionDOF= MainLib.retargetDeepMimicMotionToHanyang(info1, mLoader)
		mLoader:setVoca(input.bones)
		motionInfo.fixDOF(mLoader, mMotionDOF)
	end

	for i=0, mMotionDOF:rows()-1 do
		mMotionDOF:matView():set(i,1, mMotionDOF:matView()(i,1)+initial_height)
	end

	local option={
		bones=input.bones,
	}
	if model.option_refCDMtraj then
		table.mergeInPlace(option, model.option_refCDMtraj, true)
	end
	local touchDown=motionInfo.contact.touchDown
	local touchOff=motionInfo.contact.touchOff

	option.con=fc.buildContactConstraints(mMotionDOF:numFrames(), touchDown, touchOff)


	mLoader:setVoca(option.bones) -- setting bone dictionary.

	fc.removeFeetSliding(option.con, feet_config, mLoader, mMotionDOF, {debugDraw=true})

	CDMtraj=fc.CDMTraj(mLoader, mMotionDOF, option)

	do
		local tree=MotionUtil.LoaderToTree(mLoader, false, false)

		g_realGlobalVel=matrixn(mMotionDOF:rows(), 6)
		g_realBodyVel=matrixn(mMotionDOF:rows(), 6)

		for i=0, mMotionDOF:rows() -1 do
			tree:setLinkData(mMotionDOF:row(i),CDMtraj.dmot:row(i))
			local gv=tree:calcInverseInertiaTimesMomentumCOM(mLoader)
			g_realGlobalVel:row(i):setVec3(0, gv.w)
			g_realGlobalVel:row(i):setVec3(3, gv.v)

			local invR=mMotionDOF:row(i):toQuater(3):inverse()
			g_realBodyVel:row(i):setVec3(0, invR*gv.w)
			g_realBodyVel:row(i):setVec3(3, invR*gv.v)
		end
	end

	local fn2=motionInfo.fn2
	if param and param.envName then
		fn2='gym_cdm2/spec/refTraj_'..param.envName..'.dat'
	end
	if fn2 and os.isFileExist(fn2) then
		refTraj=util.loadTable(fn2)

		if refTraj.rawFootInfo then
			require("subRoutines/MatplotLib")

			do 
				-- graph 1
				local downsample=motionInfo.downsample or 1
				--local nf=math.min(mMotionDOF:numFrames()*downsample, refTraj.rawFootInfo:rows())
				local nf=math.round(refTraj.rawFootInfo:rows()*0.5)
				local xfn=CT.colon(0, nf)

				plotter=MatplotLib()
				plotter:figure{1, nsubfig={1,1}, subfig_size={5,3}} -- one by one plotting area.
				plotter:add('grid(True)')
				plotter:subplot(0,0)
				local Z=2
				plotter:plot(xfn, refTraj.rawFootInfo:column(2+Z):range(0, nf))
				plotter:plot(xfn, refTraj.globalTraj:column(10+Z):range(0, nf))
				plotter:plot(xfn, refTraj.rawFootInfo:column(5+Z):range(0, nf))
				plotter:plot(xfn, refTraj.globalTraj:column(7+Z):range(0, nf))
				plotter:xlabel('x')
				plotter:ylabel('z')
				plotter:legends('lfoot(t)', 'lfoot_filtered(t)', 'rfoot(t)', 'rfoot_filtered(t)')
				plotter:savefig('plot.png')
				plotter:savefig('plot.eps')
				plotter:close()
			end

			do
				-- graph 2
				local downsample=motionInfo.downsample or 1
				local nf_mocap=math.min(mMotionDOF:numFrames()*downsample)
				local nf
				for i=4, refTraj.refFrames:size()-1 do
					if refTraj.refFrames(i)==1.0 then
						nf=i
						break
					end
				end
				local xfn=CT.colon(0, nf)

				local actualFootPos=matrixn(nf, 6)

				local loader=mLoader
				local footbone={MotionLoader.LEFTANKLE, MotionLoader.LEFTANKLE, MotionLoader.RIGHTANKLE, MotionLoader.RIGHTANKLE}
				local lpos={feet_config.toe, feet_config.heel, feet_config.toe, feet_config.heel, }

				local feetTraj=fc.getFeetTraj(feet_config, mLoader, mMotionDOF)
				local rfoot_midPos=fc.getMidSolePositions(option.con[1], feetTraj[1], feetTraj[2])
				local lfoot_midPos=fc.getMidSolePositions(option.con[2], feetTraj[3], feetTraj[4])
				actualFootPos:sub(0,0,0,3):resample(lfoot_midPos:matView(), nf)
				actualFootPos:sub(0,0,3,6):resample(rfoot_midPos:matView(), nf)
					
				plotter=MatplotLib()
				plotter:figure{1, nsubfig={1,1}, subfig_size={5,3}} -- one by one plotting area.
				plotter:add('grid(True)')
				plotter:subplot(0,0)
				local Z=2
				plotter:plot(xfn, refTraj.rawFootInfo:column(2+Z):range(0, nf))

				local lfoot_aligned=actualFootPos:column(3+Z):range(0, nf):copy()
				local lfoot_z={lfoot_aligned(0), lfoot_aligned(lfoot_aligned:size()-1)}
				for i=0, lfoot_aligned:size()-1 do
					lfoot_aligned:set(i, sop.map(lfoot_aligned(i), lfoot_z[1], lfoot_z[2], refTraj.rawFootInfo(0, 2+Z), refTraj.rawFootInfo(nf-1, 2+Z)))
				end
				plotter:plot(xfn, lfoot_aligned)



				--plotter:plot(xfn, filtered+0.01)


				--plotter:plot(xfn, refTraj.rawFootInfo:column(5+Z):range(0, nf))
				--plotter:plot(xfn, actualFootPos:column(0+Z):range(0, nf))
				plotter:xlabel('x')
				plotter:ylabel('z')
				if true then
					plotter:plot(xfn, refTraj.globalTraj:column(10+Z):range(0, nf))
					plotter:legends('lfoot(t)', 'lfoot_mocap(t)', 'lfoot_filtered(t)', 'filtered2')
				else
					plotter:legends('lfoot(t)', 'lfoot_mocap(t)', 'lfoot_filtered(t)')
				end
				plotter:savefig('plot2.png')
				plotter:savefig('plot2.eps')
				plotter:close()
			end
		end


		if motionInfo.downsample then
			local n=motionInfo.downsample 
			local nf=math.floor(refTraj.refFrames:size()/n)

			local refFrames=vectorn(nf)
			local globalTraj=matrixn(nf, refTraj.globalTraj:cols())

			for i=0, nf-1 do
				refFrames:set(i, refTraj.refFrames(i*n))
				globalTraj:row(i):assign( refTraj.globalTraj:row(i*n))
			end
			refTraj.refFrames=refFrames
			refTraj.globalTraj=globalTraj

		end

		local refFrames=refTraj.refFrames
		local prevRef=-1

		-- indexed by phase
		local count=intvectorn(mMotionDOF:rows())
		count:setAllValue(0)
		local CDMtoRoot=matrixn(mMotionDOF:rows(), 6)
		CDMtoRoot:setAllValue(0)
		local CDMfeetToHumanFeetPositions=matrixn(mMotionDOF:rows(), 13) -- lfoot, rfoot, refFrames
		CDMfeetToHumanFeetPositions:setAllValue(0)
		local comdof_error=matrixn(mMotionDOF:rows(), 9) -- com position error, com vel/avel error.
		comdof_error:setAllValue(0)

		local startF=nil
		local lastF
		local function saveDelta(startF, lastF)

			if lastF>startF then
				local firstRef=refFrames(startF)
				local lastRef=refFrames(lastF)

				if firstRef==1 and startF>0 then
					startF=startF-1
					firstRef=firstRef-1
				end
				print(firstRef, lastRef, startF, lastF)

				local refFrames_aligned=refFrames:range(startF, lastF+1):copy()
				if firstRef==0 then
					assert(refFrames_aligned(0)>refFrames_aligned(1))
					refFrames_aligned:set(0, 
					refFrames_aligned(0)-(mMotionDOF:rows()-1))
					assert(refFrames_aligned(0)<refFrames_aligned(1))
					--assert(refFrames_aligned(0)>=0)
				end

				local simGlobal=refTraj.globalTraj

				local COMtraj=CDMtraj.COMtraj
				local COMtf1=transf(CDMtraj.rotY(firstRef), COMtraj:row(firstRef))
				local COMtf2=transf(CDMtraj.rotY(lastRef), COMtraj:row(lastRef))

				local delta=transf()
				local delta2=vector3()
				local scaleFactor

				if COMtf1.translation:distance(COMtf2.translation)<0.5 or motionInfo.noLooping then
					-- different align scheme may be necessary.
					dbg.console()
				else
					-- method 2: use rigid rotation and uniform scale.
					local v1=COMtf2.translation-COMtf1.translation
					local p1=simGlobal:row(startF):toTransf(0).translation
					local p2=simGlobal:row(lastF):toTransf(0).translation
					local v2=p2-p1
					v1.y=0
					v2.y=0
					scaleFactor=v2:length()/v1:length()
					local q=quater()
					q:setAxisRotation(vector3(0,1,0), v1, v2)
					delta:identity()	
					delta:leftMultRotation(q)
					delta:leftMultTranslation( -(COMtf1.translation*0.5+COMtf2.translation*0.5))
					delta2=(p1*0.5+p2*0.5)
					delta.translation.y=0
					delta2.y=0
				end

				function applyDeltaTransf(tf)
					local out=delta*tf
					out.translation.x=out.translation.x*scaleFactor
					out.translation.z=out.translation.z*scaleFactor
					out.translation:radd(delta2)
					return out
				end
				function applyDelta(v)
					local out=delta*v
					out.x=out.x*scaleFactor
					out.z=out.z*scaleFactor
					out:radd(delta2)
					return out
				end

				local CDMfeetTraj=matrixn(lastRef-firstRef+1, 14)

				local footOrientationOrig=matrixn(lastF-startF+1, 8)
				for i=startF, lastF do
					local gi=refTraj.globalTraj:row(i)
					local q1=quater(gi(13+6), vector3(0,1,0))
					local q2=quater(gi(13+7), vector3(0,1,0))
					footOrientationOrig:row(i-startF):setQuater(0, q1)
					footOrientationOrig:row(i-startF):setQuater(4, q2)
				end
				footOrientationOrig:quatViewCol(0):align()
				footOrientationOrig:quatViewCol(4):align()


				for phase=firstRef, lastRef do
					local i=sop.map(phase, firstRef, lastRef, startF, lastF) -- frame
					local globalFeet=refTraj.globalTraj:sample(i)
					--local w=sop.map(i, startF, lastF, 0, 1)
					--local errorToRemove=accumulatedError1:Interpolate(w, accumulatedError2)

					local lfootsim=globalFeet:toVector3(7)
					local rfootsim=globalFeet:toVector3(10)
					local footori=footOrientationOrig:sample(i-startF)
					local lfootori=footori:toQuater(0)
					local rfootori=footori:toQuater(4)
					lfootori:normalize()
					rfootori:normalize()

					local lfootref=transf(lfootori, lfootsim)
					local rfootref=transf(rfootori, rfootsim)

					CDMfeetTraj:row(phase-firstRef):setTransf(0, lfootref)
					CDMfeetTraj:row(phase-firstRef):setTransf(7, rfootref)

					--local feetorig=CDMfeetTrajOrig:row(phase-firstRef)
					--feetorig:setVec3(0, midori*lfootsim)
					--feetorig:setVec3(3, midori*rfootsim)
				end

				local conL=option.con[2]:range(firstRef, lastRef+1)
				local conR=option.con[1]:range(firstRef, lastRef+1)

				local CDMrootTraj=matrixn(lastRef-firstRef+1, 7)
				local CDMbodyVel=matrixn(lastRef-firstRef+1, 6)

				for phase=firstRef, lastRef do
					--dbg.draw('Axes', MotionDOF.rootTransformation(refTraj.globalTraj:row(i)),  'CDM'..i, 100)

					--local ref=refFrames(i)
					local i=sop.map(phase, firstRef, lastRef, startF, lastF)

					local w=sop.map(phase, firstRef, lastRef, 0, 1)


					-- errorToRemove를 글로벌에서 적용하여 글로벌 y축 회전만 하도록 한다. 
					local CDMtf=simGlobal:sampleTransf(i)

					if false then

						--if phase==lastRef then
						--	dbg.console()
						--end
						--local test=accumulatedError1:Interpolate(1, accumulatedError2)
						--dbg.console()
						RE.output2("CDMy", CDMtf.translation.y)
						local startPos=simGlobal:sampleTransf(startF).translation:copy()
						startPos.y=0
						dbg.namedDraw('Axes', transf(quater(1,0,0,0), -startPos)*CDMtf,  'CDM_orig', 100)
						dbg.namedDraw('Axes', applyDeltaTransf(mMotionDOF:row(phase):toTransf(0)), 'real', 100)
						renderOneFrame()
						RE.usleep(1/10.0 * 1e6); -- three-times slower than real-time
					end
					MotionDOF.setRootTransformation(CDMrootTraj:row(phase-firstRef), CDMtf)


					local V=vectorn()
					simGlobal:sub(0,0, 13,19):sampleRow(i, V)
					local w=V:toVector3(0)
					local v=V:toVector3(3)
					local simR=CDMtf.rotation
					CDMbodyVel:row(phase-firstRef):setVec3(0, simR:inverse()*w)
					CDMbodyVel:row(phase-firstRef):setVec3(3, simR:inverse()*v)
				end

				local rootTraj=mMotionDOF:matView(): sub(firstRef, lastRef+1, 0,0):copy()
				local realCOM=vector3N(lastRef-firstRef+1)
				local footPos={ 
					vector3N(lastRef-firstRef+1),
					vector3N(lastRef-firstRef+1),
					vector3N(lastRef-firstRef+1),
					vector3N(lastRef-firstRef+1),
				}

				for phase=firstRef, lastRef do
					local loader=mLoader
					loader:setPoseDOF(mMotionDOF:row(phase))
					realCOM(phase-firstRef):assign(applyDelta(loader:calcCOM()))
					local footbone={MotionLoader.LEFTANKLE, MotionLoader.LEFTANKLE, MotionLoader.RIGHTANKLE, MotionLoader.RIGHTANKLE}
					local lpos={feet_config.toe, feet_config.heel, feet_config.toe, feet_config.heel, }

					local feetinfo=vectorn(3*#lpos+1)
					for icontact=0,#lpos-1 do
						local footpos=loader:getBoneByVoca(footbone[icontact+1]):getFrame()*lpos[icontact+1]
						footPos[icontact+1](phase-firstRef):assign(applyDelta(footpos))
					end
					rootTraj:row(phase-firstRef):setTransf(0, applyDeltaTransf(rootTraj:row(phase-firstRef):toTransf(0)))
				end
				
				for phase=firstRef, lastRef do
					local sim2=MotionDOF.rootTransformation(CDMrootTraj:row( phase-firstRef))
				--local lines=vector3N()
					--local sim2=simGlobal:row(i):toTransf(0) -- for debugging



					--lines:pushBack(sim2.translation)
					--lines:pushBack(mMotionDOF:row(phase):toVector3(0))
					-- sim*simToReal=real
					local simToReal=sim2:inverse()*rootTraj:row(phase-firstRef):toTransf(0)
					simToReal.rotation:align(quater(1,0,0,0))

					CDMtoRoot:row(phase):radd(simToReal:toLogVec())


					-- comdof error
					do
						local com_error=vectorn(9)

						local real_com=realCOM:row(phase-firstRef)
						local simR=sim2.rotation
						com_error:setVec3(0, simR:inverse()*(real_com-sim2.translation))
						
						local simBodyVel=CDMbodyVel:row( phase-firstRef)
						--local realGlobalVel=invI*Hcom computed from the fullbody motion
						local realBodyVel=g_realGlobalVel:row(phase):copy()
						local w=realBodyVel:toVector3(0)
						local v=realBodyVel:toVector3(3)
						realBodyVel:setVec3(0, simR:inverse()*delta.rotation*w)
						realBodyVel:setVec3(3, simR:inverse()*delta.rotation*v)

						--print('real_wv:', realBodyVel, 'sim:', simBodyVel)
						com_error:slice(3,0):assign(realBodyVel-simBodyVel)

						comdof_error:row(phase):radd(com_error)
					end

				

					local feetinfo=vectorn(3*#footPos+1)
					for icontact=0,#footPos-1 do
						ifoot=math.floor(icontact/2)
						local footpos=footPos[icontact+1](phase-firstRef)
						local refCoord=CDMfeetTraj:row(phase-firstRef):toTransf(ifoot*7)
						feetinfo:setVec3(icontact*3, refCoord:toLocalPos(footpos))
					end

					
					feetinfo:set(3*#footPos, refFrames_aligned:sample(sop.map(phase, firstRef, lastRef, 0, lastF-startF)))

					CDMfeetToHumanFeetPositions:row(phase):radd(feetinfo)

					count:set(phase, count(phase)+1)
				end
				--lines:drawLines('CDMtoRoot')
			end
		end

		local cycles={}
		for i=0, refFrames:size()-1 do

			local refFrame=refFrames(i)
			
			if refFrame> prevRef then

				if refFrame==prevRef+1 then
					-- typical case
				else
					-- early touch-down
				end
				if not startF then
					startF=i
				end
				lastF=i
			else
				if startF then
					if startF~=0 or motionInfo.noLooping then 
						-- 이전 path의 오차 저장. 단 누적 오차는 제거함.
						-- loop인경우 첫 사이클은 제외.
						table.insert(cycles, {startF, lastF})
					end
				end
				startF=i
			end
			prevRef=refFrame
		end
		if #cycles>10 then
			table.remove(cycles, 1)
			table.remove(cycles, 1)
		end
		if not motionInfo.noLooping then
			-- remove short cycles
			local minRef=mMotionDOF:numFrames()
			local maxRef=0
			local function ref(v)
				return refTraj.refFrames(v)
			end
			for i, v in ipairs(cycles) do
				minRef=math.min(minRef, ref(v[1]))
				maxRef=math.max(maxRef, ref(v[2]))
			end
			local cycles_old=cycles
			cycles={}
			for i, v in ipairs(cycles_old) do
				if minRef==ref(v[1]) and maxRef==ref(v[2]) then
					table.insert(cycles, {v[1], v[2]})
					saveDelta(v[1], v[2]) 
				end
			end
			assert(#cycles>0)
		end
		if motionInfo.noLooping then
			--saveDelta(startF, lastF) 
			saveDelta(cycles[1][1], cycles[1][2]) 
		else
			-- 마지막 패스는 끝까지 진행 안됬을수도 있으니 무시.
			-- saveDelta(startF, lastF)
		end

		local invalid=boolN(count:size())
		invalid:clearAll()
		for i=0, count:size()-1 do
			if count(i)==0 then
				invalid:set(i, true)
			else
				CDMtoRoot:row(i):rmult(1.0/count(i))
				CDMfeetToHumanFeetPositions:row(i):rmult(1.0/count(i))
				comdof_error:row(i):rmult(1.0/count(i))
			end
		end

		for i=1, CDMfeetToHumanFeetPositions:rows()-1 do
			local prev_ref=CDMfeetToHumanFeetPositions(i-1, 6)
			local ref=CDMfeetToHumanFeetPositions(i, CDMfeetToHumanFeetPositions:cols()-1)
			assert(ref>=prev_ref)
		end
		local invalidInterval=invalid:runLengthEncode()

		for i=0, invalidInterval:size()-1 do
			local startI=invalidInterval:startI(i)
			local endI=invalidInterval:endI(i)

			if startI==0 and endI==1 then
				-- typical case.
				CDMtoRoot:row(0):assign(CDMtoRoot:row(1))
				CDMfeetToHumanFeetPositions:row(0):slice(0,-1):assign(CDMfeetToHumanFeetPositions:row(1):slice(0,-1))
				comdof_error:row(0):assign(comdof_error:row(1))
				-- 마지막 컬럼에는 phase들어 있어서.
				
			end
			for j=0, CDMtoRoot:cols()-1 do
				--CDMtoRoot:column(j):range(startI-1, endI+1):linspace(CDMtoRoot(startI-1,j), CDMtoRoot(endI,j))
			end
		end


		refTraj.CDMtoRoot=CDMtoRoot

		if model.option_refCDMtraj.scaleYdelta then
			local midY=refTraj.CDMtoRoot:column(4):avg()
			local delta=refTraj.CDMtoRoot:column(4)-midY
			delta:rmult(model.option_refCDMtraj.scaleYdelta)
			refTraj.CDMtoRoot:column(4):assign(delta+midY)
		end

		refTraj.CDMfeetToHumanFeetPositions=CDMfeetToHumanFeetPositions
		if motionInfo.option_poseSampler then
			local ops=motionInfo.option_poseSampler
			if ops.cyclicRoot then
				math.filter(refTraj.CDMtoRoot, 5)
				refTraj.CDMtoRoot:makeCyclic(5)
			end
			if ops.cyclicFoot then
				refTraj.CDMfeetToHumanFeetPositions:sub(0,0,0,-1):makeCyclic(5)
			end
		end
		refTraj.comdof_error=comdof_error
		refTraj.bodyVel=g_realBodyVel

		util.saveTable(refTraj, fn2..'.processed')
		mTimeline=Timeline("Timeline", refTraj.globalTraj:rows(), 1/30)
		print('finished exporting', refTraj, fn2..'.processed')

	end


	mMotionDOF:align(mMotionDOF:copy(), mMotionDOF:copy())

	mSkin= RE.createVRMLskin(mLoader, false);	-- to create character
	mSkin:scale(100,100,100);					-- motion data is in meter unit while visualization uses cm unit.
	mSkin:applyMotionDOF(mMotionDOF)
	mSkin:setMaterial('lightgrey_transparent')

	RE.motionPanel():motionWin():addSkin(mSkin)

	local refFrame=refTraj.CDMfeetToHumanFeetPositions:column(refTraj.CDMfeetToHumanFeetPositions:cols()-1)
	local phase=CT.linspace(refFrame(0), refFrame(refFrame:size()-1),refFrame:size())
	refFrameToPhase=math.PiecewiseLinearCurve(refFrame, phase)


	if testReconstruction then
		mSkin2= RE.createVRMLskin(mLoader, false);	-- to create character
		mSkin2:scale(100,100,100);					-- motion data is in meter unit while visualization uses cm unit.
		mSkin2:setMaterial('lightgrey_transparent')

		-- test fullbody reconstruction


		require('gym_cdm2/module/CDMphaseFilter')
		mPhaseFilter=CDMphaseFilter(3, refFrameToPhase) -- only the fullbody-phase will be filtered so that the timing flows fluently.
	end


	if param and param.exitAt==0 then
		this('exit!',0)
	end
end
function dtor()
end

function frameMove()
	return 0
end
function handleRendererEvent()
	return 0
end

function EVR:onFrameChanged(win, i)
	if noFrameChange then return end
	print(i)

	local COMtraj=CDMtraj.COMtraj

	--dbg.namedDraw('Axes', transf(quater(1,0,0,0), COMtraj:row(i)), 'root', 100)
	if refTraj then
		--dbg.namedDraw('Axes', MotionDOF.rootTransformation(refTraj.globalTraj:row(i)),  'CDM', 100)

		if i<refTraj.CDMtoRoot:rows()*2-1 then
			local ii=i
			if i>=refTraj.CDMtoRoot:rows() then
				ii=i-(refTraj.CDMtoRoot:rows()-1) -- the last frame equals to the first frame (one-frame overlap)
			end
			-- sim*CDMtoRoot=real
			--> real*CDMtoRoot:inverse()=sim
			local CDMtoRoot=refTraj.CDMtoRoot:row(ii):to_se3():exp()
			local root=MotionDOF.rootTransformation(mMotionDOF:row(i))
			root.rotation:normalize()
			dbg.namedDraw('Axes', root, 'ROOT', 100)

			local CDM2=root*CDMtoRoot:inverse()
			dbg.namedDraw('Axes', CDM2, 'CDM_fitted', 100)


			local loader=mLoader
			loader:setPoseDOF(mMotionDOF:row(i))

			if true then

				local toe=feet_config.toe
				local heel=feet_config.heel
				local lfoot=loader:getBoneByVoca(MotionLoader.LEFTANKLE):getFrame()*toe
				local rfoot=loader:getBoneByVoca(MotionLoader.RIGHTANKLE):getFrame()*toe
				local lfoot2=loader:getBoneByVoca(MotionLoader.LEFTANKLE):getFrame()*heel
				local rfoot2=loader:getBoneByVoca(MotionLoader.RIGHTANKLE):getFrame()*heel

				-- simFoot+cdm_rotY*footOffset=realFoot
				--> simFoot=realFoot- CDM2.rotation:rotationY()*footOffset
				dbg.draw('SphereM',lfoot-CDM2.rotation:rotationY()*refTraj.CDMfeetToHumanFeetPositions:row(ii):toVector3(0), 'lfoot','red')
				dbg.draw('SphereM',lfoot2-CDM2.rotation:rotationY()*refTraj.CDMfeetToHumanFeetPositions:row(ii):toVector3(3), 'lfoot2','red')
				dbg.draw('SphereM',rfoot-CDM2.rotation:rotationY()*refTraj.CDMfeetToHumanFeetPositions:row(ii):toVector3(6), 'rfoot', 'blue')
				dbg.draw('SphereM',rfoot2-CDM2.rotation:rotationY()*refTraj.CDMfeetToHumanFeetPositions:row(ii):toVector3(9), 'rfoot2', 'blue')
			else
				dbg.draw('SphereM',refTraj.CDMfeetToHumanFeetPositions:row(ii):toVector3(0), 'lfoot','red')
				dbg.draw('SphereM',refTraj.CDMfeetToHumanFeetPositions:row(ii):toVector3(6), 'rfoot', 'blue')
			end
			--dbg.namedDraw('Sphere', simFoot*100, 'lpos_sim','blue')
			-- 위의 레퍼런스 모션과 아래 시뮬레이티드 모션은 타이밍이 다름.
		else
			dbg.erase('Axes', 'ROOT')
			dbg.erase('Axes', 'CDM_fitted')
		end
		if i<refTraj.globalTraj:rows() then

			local drawOffsetX=1
			local CDM=refTraj.globalTraj:row(i):toTransf()
			CDM.translation.x=CDM.translation.x+drawOffsetX
			dbg.namedDraw('Axes', CDM, 'CDM_sim', 100)

			local gi=refTraj.globalTraj:row(i)
			local lpos=gi:toVector3(7)+vector3(drawOffsetX,0,0)
			local lori=quater(gi(13+6), vector3(0,1,0))
			local rpos=gi:toVector3(10)+vector3(drawOffsetX,0,0)
			local rori=quater(gi(13+7), vector3(0,1,0))

			local LrefCoord=transf(lori, lpos)
			local RrefCoord=transf(rori, rpos)

			local function drawCon(index, refCoord, name, color, ii)
				local realFootL=refCoord*refTraj.CDMfeetToHumanFeetPositions:sample(ii):toVector3(index)
				dbg.draw('SphereM',realFootL, name,color)
			end



			local sim_phase=refTraj.refFrames(i)

			local ii=refFrameToPhase:sample(sim_phase)


			local CDMtoRoot=refTraj.CDMtoRoot:sample(ii):to_se3():exp()
			local root=CDM*CDMtoRoot

			dbg.namedDraw('Axes', root, 'root_fitted', 100)

			dbg.namedDraw('Axes', LrefCoord,'lpos',100)
			dbg.namedDraw('Axes', RrefCoord,'rpos',100)

			drawCon(0,LrefCoord, 'lfoot_fitted', 'blue', ii)
			drawCon(3,LrefCoord, 'lfoot_fitted2', 'blue', ii)
			drawCon(6,RrefCoord, 'rfoot_fitted', 'red', ii)
			drawCon(9,RrefCoord, 'rfoot_fitted2', 'red', ii)

			if testReconstruction then
				-- redraw using phase filter
				local CDMdq=CT.zeros(6)
				mPhaseFilter:setState(CDM, CDMdq, LrefCoord, RrefCoord, 0,0, sim_phase) 
				local prev_ii=ii
				CDM, LrefCoord, RrefCoord, ii=mPhaseFilter:getState()
				print(prev_ii, ii)
				CDM.translation.x=CDM.translation.x+drawOffsetX
				LrefCoord.translation.x=LrefCoord.translation.x+drawOffsetX
				RrefCoord.translation.x=RrefCoord.translation.x+drawOffsetX

				local CDMtoRoot=refTraj.CDMtoRoot:sample(ii):to_se3():exp()
				local root=CDM*CDMtoRoot
				dbg.namedDraw('Axes', root, 'root_fitted2', 100)
				dbg.namedDraw('Axes', LrefCoord,'lpos2',100)
				dbg.namedDraw('Axes', RrefCoord,'rpos2',100)

				drawCon(0,LrefCoord, '_lfoot_fitted', 'blue', ii)
				drawCon(3,LrefCoord, '_lfoot_fitted2', 'blue', ii)
				drawCon(6,RrefCoord, '_rfoot_fitted', 'red', ii)
				drawCon(9,RrefCoord, '_rfoot_fitted2', 'red', ii)

				local pose=mMotionDOF:sample(ii)
				pose:setTransf(0, root)
				mSkin2:setPoseDOF(pose)
			end

		end
	end
end
function renderOneFrame()
	noFrameChange=true
	RE.renderOneFrame(true)
	noFrameChange=false
end

