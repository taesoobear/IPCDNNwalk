
require("config")
require("module")
require("common")

package.projectPath='./gym_walkCDM'
package.path=package.path..";../../taesooLib/Samples/scripts/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/RigidBodyWin/subRoutines/?.lua" --;"..package.path

require("Timeline")
require("gym_walkCDM/testWalkV2Playback")
FE=require("gym_walkCDM/featureExtractor")

onCallback_orig=onCallback



function ctor()
	RE.viewpoint():setFOVy(45.000000)
	RE.viewpoint():setZoom(1.000000)
	RE.viewpoint().vpos:set(-341.368391, 122.750331, 62.135002)
	RE.viewpoint().vat:set(21.295332, 91.836266, 57.736808)
	RE.viewpoint():update()

	init_planner()

	_createTerrain()
	local datasets
	if true then
		-- evaluation dataset
		datasets={ 
			{'_terrainRandom_eval_0.dat', useTerrain=true},
			{'_terrainRandom_eval_1.dat', useTerrain=true},
			{'_terrainRandom_eval_2.dat', useTerrain=true},
			{'_terrainRandom_eval_3.dat', useTerrain=true},
			{'_terrainRandom_eval_4.dat', useTerrain=true},
			{'_terrainRandom_eval_5.dat', useTerrain=true},
			{'_terrainRandom_eval_6.dat', useTerrain=true},
			{'_terrainRandom_eval_7.dat', useTerrain=true},
			{'_terrainRandom_eval_8.dat', useTerrain=true},

			outfile='feature_data_eval.dat'
		}
	else
		datasets={ 
			{'dataset1.dat',},
			{'dataset2.dat', },
			{'data/dataset1.dat',},
			{'data/dataset2.dat', },
			{'data/dataset3.dat', },
			--{'dataset3.dat', },
			--{'_terrain0.dat', useTerrain=true},
			--{'_terrain1.dat', useTerrain=true},
			{'terrain2.dat', useTerrain=true},
			{'terrain3.dat', useTerrain=true},
			{'terrain4.dat', useTerrain=true},
			--{'terrain5.dat', useTerrain=true},
			--{'terrain6.dat', useTerrain=true},
			{'terrain7.dat', useTerrain=true},
			{'terrain8.dat', useTerrain=true},
			{'terrain9.dat', useTerrain=true},
			{'terrain10.dat', useTerrain=true}, 
			{'terrain11.dat', useTerrain=true}, 
			--{'terrain12.dat', useTerrain=true}, 
			--{'terrain13.dat', useTerrain=true}, 
			--{'terrain16.dat', useTerrain=true}, 
			{'terrain17.dat', useTerrain=true}, 
			{'terrain18.dat', useTerrain=true}, 
			{'terrain19.dat', useTerrain=true}, 
			{'terrain20.dat', useTerrain=true},
			--{'terrain21.dat', useTerrain=true},
			{'terrain22.dat', useTerrain=true},
			--{'terrain25.dat', useTerrain=true},
			--{'terrain27.dat', useTerrain=true},
			--{'terrain28.dat', useTerrain=true},
			--{'terrain29.dat', useTerrain=true},
			{'terrain30.dat', useTerrain=true},
			--{'terrain31.dat', useTerrain=true},
			--{'terrain32.dat', useTerrain=true},
			--{'terrain33.dat', useTerrain=true},
			--{'terrain34.dat', useTerrain=true},
			--{'terrain35.dat', useTerrain=true},
			--{'terrain36.dat', useTerrain=true},
			--{'terrain37.dat', useTerrain=true},
			{'data/_terrain0.dat', useTerrain=true},
			--data/{'_terrain1.dat', useTerrain=true},
			{'data/_terrain2.dat', useTerrain=true},
			{'data/_terrain3.dat', useTerrain=true},
			{'data/_terrain4.dat', useTerrain=true},
			{'data/_terrain5.dat', useTerrain=true},
			{'data/_terrain6.dat', useTerrain=true},
			{'data/_terrain7.dat', useTerrain=true},
			{'data/_terrain8.dat', useTerrain=true},
			{'data/_terrain9.dat', useTerrain=true},
			{'data/_terrain10.dat', useTerrain=true}, 
			{'data/_terrain11.dat', useTerrain=true}, 
			{'data/_terrain12.dat', useTerrain=true}, 
			{'data/_terrain13.dat', useTerrain=true}, 
			{'data/_terrain16.dat', useTerrain=true}, 
			{'data/_terrain17.dat', useTerrain=true}, 
			{'data/_terrain18.dat', useTerrain=true}, 
			{'data/_terrain19.dat', useTerrain=true}, 
			{'data/_terrain20.dat', useTerrain=true},
			{'data/_terrain21.dat', useTerrain=true},
			{'data/_terrain22.dat', useTerrain=true},
			{'data/_terrain25.dat', useTerrain=true},
			{'data/_terrain27.dat', useTerrain=true},
			{'data/_terrain28.dat', useTerrain=true},
			{'data/_terrain29.dat', useTerrain=true},
			{'data/_terrain30.dat', useTerrain=true},
			{'data/_terrain31.dat', useTerrain=true},
			{'data/_terrain32.dat', useTerrain=true},
			{'data/_terrain33.dat', useTerrain=true},
			{'data/_terrain34.dat', useTerrain=true},
			{'data/_terrain35.dat', useTerrain=true},
			{'data/_terrain36.dat', useTerrain=true},
			{'data/_terrain37.dat', useTerrain=true},

			outfile='feature_data.dat'
		}
	end
	local count=0
	for idataset, vdataset in ipairs(datasets) do
		local dataset=vdataset[1]
		if not os.isFileExist(dataset) then
			print(dataset..' does not exist!')
			count=count+1
		end
	end
	if count>0 then
		dbg.console()
	end
	if false then
		-- count total # of frames
		local total_nf=0
		for idataset, vdataset in ipairs(datasets) do
			local dataset=vdataset[1]
			if os.isFileExist(dataset) then
				g_history=nil
				collectgarbage('collect')
				g_history=util.loadTable(dataset)
				local phases=g_history.phases
				total_nf=total_nf+phases:rows()
			end
		end
		dbg.console()
	end
	
	local matfeatures=matrixn()
	local matdata=matrixn()
	local matdotdata=matrixn()

	local numSampleOutput=30
	local outputCols={}
	for idataset, vdataset in ipairs(datasets) do
		local dataset=vdataset[1]
		g_history=nil
		collectgarbage('collect')
		g_COMerr=nil
		initializeDataset(dataset,{getPrevStride=true}, vdataset.useTerrain)

		g_mot=MotionDOFcontainer(mMot.loader.dofInfo)
		g_mot:resize(g_history.dtheta:rows()-1)
		for i=0, g_mot:numFrames()-1 do
			local rp=planner:drawFrame(i)
			g_mot.mot:row(i):assign(rp)
			if math.fmod(i,10)==0 then
				RE.renderOneFrame(true)
			end
		end
		local dmot=g_mot.mot:calcDerivative(30) -- returns body velocity

		local cutState=boolN(g_history.stridePhase[1]:rows())
		cutState:setAllValue(false)
		local isL=boolN(g_history.stridePhase[1]:rows())

		local max_i=1e5
		for ileg=0,1 do
			local prev_sp=g_history.prevStride:column(ileg)
			local s_f=g_history.prevStrideFrame:column(ileg)
			local sp=g_history.stridePhase[ileg+1]:column(0)
			local pi=0

			for i=1, s_f:size()-1 do
				if s_f(i)~=s_f(i-1) then
					if(sp(pi+1)>0.3) then
						max_i=math.min(max_i, i) 
						break
					end
					assert(sp(i)>=0.6)
					if(sp(i+1)>1.0) then
						-- last segment
						max_i=math.min(max_i, i) 
						break
					end
					if(sp(i+1)>0.3) then
						max_i=math.min(max_i, i)
						break
					end
					print(pi, i)
					cutState:set(pi, true)
					cutState:set(i, true)
					if ileg==0 then
						isL:set(pi, true)
						isL:set(i, true)
					end
					pi=i
				end
			end
		end
		RE.motionPanel():scrollPanel():addPanel(cutState, CPixelRGB8(255,0,0))


		local seg=intIntervals()
		seg:runLengthEncodeCut(cutState)
		for iseg=0, seg:numInterval()-3 do -- discard last
			local sf=seg:startI(iseg)
			local ef=seg:endI(iseg+1)
			local _isL=isL(sf)
			print(seg:startI(iseg), seg:endI(iseg), seg:endI(iseg+1), _isL)

			local sp=
			{
				g_history.stridePhase[1]:column(0),
				g_history.stridePhase[2]:column(0),
			}

			local sp2={}
			local function isSimilar(a,b)
				return math.abs(a-b)<1e-3
			end

			local function isSimilarPhase(a, b)
				return isSimilar(a,b) or isSimilar(a, b+1) or isSimilar(a, b-1) 
			end

			local motionType=g_history.state
			local vfeature, refTerrainHeight, regressor=FE.extractFeature(sp, g_history.pendstate:sub(0,0,0,6), g_history.pendstate:sub(0,0,9,12),g_history.pendstate:sub(0,0,12,0), motionType, sf, ef, vdataset.useTerrain)

			local data=matrixn()
			local dotdata=matrixn()
			local cdmtrajR=vector3N(ef-sf+1)
			local cdmtrajT=vector3N(ef-sf+1)

			--local offsetQm=g_history.pendstate:row(math.round(sf*0.5+ef*0.5)):toQuater(12):offsetQ()

			for iframe=sf, ef do
				local pendstate=g_history.pendstate:row(iframe)
				local pendROOTQ=pendstate:toQuater(12)
				local pendCOM=pendstate:toVector3(0)
				local pendZMP=pendstate:toVector3(9)

				refTerrainHeight=regressor:getHeight(pendZMP)

				pendCOM.y=pendCOM.y+refTerrainHeight
				pendZMP.y=pendZMP.y+refTerrainHeight

				--local pend2Dtf=transf(pendROOTQ:rotationY()*offsetQm, pendCOM)
				local pend2Dtf=transf(pendROOTQ, pendCOM)
				local pend2Dtf_foot=transf(pendROOTQ:rotationY(), pendZMP)
				pend2Dtf_foot.translation.y=0

				local comdof=g_history.input[4]:row(iframe)
				local cdmtf=MotionDOF.rootTransformation(comdof)
				local pendCOMVEL=pendstate:toVector3(3)
				local pendANGVEL=pendstate:toVector3(6)

				local comangvel=comdof:toVector3(7) -- in global coord
				local comvel=comdof:toVector3(10) -- in global coord
				local dpose=dmot:row(iframe)
				local dotdata_row=vectorn(dpose:size()+6)
				dotdata_row:setVec3(0,  pend2Dtf.rotation:inverse()*comangvel)
				dotdata_row:setVec3(3,  pend2Dtf.rotation:inverse()*comvel)
				dotdata_row:slice(6,0):assign(dpose)
				dotdata:pushBack(dotdata_row)

				-- pend2Dtf*delta=cdmtf
				local lcdmtf=pend2Dtf:inverse()*cdmtf
				lcdmtf.rotation:align(quater(1,0,0,0))
				cdmtrajR(iframe-sf):assign(lcdmtf.rotation:rotationVector())
				cdmtrajT(iframe-sf):assign(lcdmtf.translation)

				local pose=g_mot.mot:row(iframe):copy()
				local posetf=MotionDOF.rootTransformation(pose)
				local lposetf=cdmtf:inverse()*posetf

				pose=pose:slice(1,0):copy()
				pose:slice(0,3):assign(lposetf.rotation:rotationVector())
				pose:slice(3,6):assign(lposetf.translation)

				dbg.draw("Axes", cdmtf, 'cdm', 100, 2.0)
				dbg.draw("Axes", pend2Dtf, 'pendZ', 100, 2.0)

				--cdmtf==comdof[0:7]
				local numCon=mSolverInfo.numCon
				local localfoot=vectorn(numCon*3)

				-- pend world velocity
				local pendV=Liegroup.se3(pendANGVEL, pendCOMVEL)

				-- CDM world velocity
				local V=Liegroup.se3(comdof:toVector3(7), comdof:toVector3(10))
				local footdof=g_history.input[3]:row(iframe)
				local numCon=mSolverInfo.numCon
				for i=0, numCon-1 do
					dbg.draw('Sphere', footdof:toVector3(i*3)*100, 'ball'..i,'red',5)

					local gfoot=footdof:toVector3(i*3)
					gfoot.y=gfoot.y-regressor:getHeight(gfoot)
					localfoot:setVec3(i*3, pend2Dtf_foot:toLocalPos(gfoot))
				end
				outputCols[1]= localfoot:size()
				outputCols[2]= pose:size()
				data:pushBack(localfoot..pose)
				if iframe==sf then
					-- posefeature (cdm, feet)  + pend-traj feature (vfeature)
					matfeatures:pushBack(cdmtrajT:matView():row(iframe-sf).. localfoot..vfeature)
				end
			end

			outputCols[3]=cdmtrajR:matView():cols()
			outputCols[4]=cdmtrajT:matView():cols()
			local mdata=data..cdmtrajR:matView()..cdmtrajT:matView()
			local mdata2=matrixn()
			mdata2:resample(mdata, numSampleOutput)
			matdata:pushBack(mdata2:toVector())
			mdata2:resample(dotdata, numSampleOutput)
			matdotdata:pushBack(mdata2:toVector())
		end

	end
	assert(matfeatures:rows()==matdata:rows())

	util.saveTable({matfeatures, matdata, matdotdata, dataInfo={numSampleOutput, outputCols}, }, datasets.outfile)

	mMot.skin:applyMotionDOF(g_mot.mot)
	RE.motionPanel():motionWin():detachSkin(mMot.skin)
	RE.motionPanel():motionWin():addSkin(mMot.skin)
end

function EVR:onFrameChanged(win, iframe)
	if g_history.dtheta:rows()<=iframe then
		return 
	end
	local comdof=g_history.input[4]:row(iframe)
	local cdmtf=MotionDOF.rootTransformation(comdof)
	local pendstate=g_history.pendstate:row(iframe)
	local pendCOM=pendstate:toVector3(0)
	local pendCOMVEL=pendstate:toVector3(3)
	local pendANGVEL=pendstate:toVector3(6)
	local pendZMP=pendstate:toVector3(9)

	local pendROOTQ=pendstate:toQuater(12)
	local pendtf=transf(pendROOTQ, pendCOM)
	dbg.draw("Axes", pendtf, 'pend', 100, 1.5)
	dbg.draw("Axes", cdmtf, 'cdm', 100, 2.0)

	local numCon=mSolverInfo.numCon

	-- pend world velocity
	local pendV=Liegroup.se3(pendANGVEL, pendCOMVEL)

	-- CDM world velocity
	local V=Liegroup.se3(comdof:toVector3(7), comdof:toVector3(10))
	local footdof=g_history.input[3]:row(iframe)
	local numCon=mSolverInfo.numCon
	for i=0, numCon-1 do
		dbg.draw('Sphere', footdof:toVector3(i*3)*100, 'ball'..i,'red',5)
	end
	planner:drawContactForce(iframe)
	if boolean_options.attachCamera then
		local curPos= cdmtf.translation*100
		curPos.y=0
		if g_prevRootPos then
			--curPos=filterXZ:step(curPos)
			RE.viewpoint().vpos:assign(RE.viewpoint().vpos+curPos-g_prevRootPos)
			RE.viewpoint().vat:assign(RE.viewpoint().vat+curPos-g_prevRootPos)
			RE.viewpoint():update()     
			print(curPos)
		end
		g_prevRootPos=curPos:copy()
	end
end
