useGMBSsimulator=true
if true then
	-- for compatibility with gmbs simulator
	Physics.DynamicsSimulator_TRL_QP.calcMassMatrix3=Physics.DynamicsSimulator_TRL_QP.calcMassMatrix
	function Physics.DynamicsSimulator_TRL_QP:calcBodyJacobianAt(ichar, ibone, J, localpos)
		self:calcJacobianAt(ichar, ibone, J, localpos)

		local T=self:getWorldState(ichar):globalFrame(1)
		local AdR=liegroup.Ad(transf(T.rotation, vector3(0,0,0)))

		-- [J0 J1] * [dq0] == [J0  J1] * [AdR *bdq]== [J0*AdR  J1] * [bdq]
		--           [dq1]               [dq1]                       [dq1]
		-- , where J0 == J:sub(0,6,0,6)
		--         J1 == J:sub(0,6,6,0)
		J:sub(0,6,0,6):assign(J:sub(0,6,0,6)*AdR)
	end
	function Physics.DynamicsSimulator_TRL_QP:calcBodyDotJacobianAt(ichar, ibone, J, DJ, localpos)
		self:calcJacobianAt(ichar, ibone, J, localpos);
		self:calcDotJacobianAt(ichar, ibone, DJ, localpos);

		local T=self:getWorldState(ichar):globalFrame(1)
		local AdR=liegroup.Ad(transf(T.rotation, vector3(0,0,0)))
		local dotAdR=AdR:copy()
		-- dotR=skew(w)*R where w is the world angular velocity
		local skewW=CT.skew(self:getWorldAngVel(ichar,self:skeleton(ichar):VRMLbone(1)))
		dotAdR:sub(0,3,0,3):assign(skewW* AdR:sub(0,3,0,3))
		dotAdR:sub(3,6,3,6):assign(dotAdR:sub(0,3,0,3))

		DJ:sub(0,6,0,6):assign(DJ:sub(0,6,0,6)*AdR+J:sub(0,6,0,6)*dotAdR)
		J:sub(0,6,0,6):assign(J:sub(0,6,0,6)*AdR)
	end

	function Physics.DynamicsSimulator_TRL_QP:calcBoneDotJacobian(ichar, ibone, localpos, J, DJ)
		--assert(false)
		local jacobian=matrixn()
		local dotjacobian=matrixn()
		self:calcBodyDotJacobianAt(ichar, ibone, jacobian, dotjacobian, localpos);
		J:assign(jacobian:sub(3,6,0,0))
		DJ:assign(dotjacobian:sub(3,6,0,0))
	end
	function Physics.DynamicsSimulator_TRL_QP:calcBoneDotJacobian2(ichar, ibone, localpos, jacobian, dotjacobian)
		self:calcBodyDotJacobianAt(ichar, ibone, jacobian, dotjacobian, localpos)
	end

	function Physics.DynamicsSimulator_TRL_QP:calcJacobian(ichar, ibone, J)
		self:calcJacobianAt(ichar, ibone, J, vector3(0,0,0));

		local T=self:getWorldState(ichar):globalFrame(1)
		local AdR=liegroup.Ad(transf(T.rotation, vector3(0,0,0)))

		J:sub(0,0,0,6):assign(J:sub(0,0,0,6)*AdR)

	end
	function Physics.DynamicsSimulator_TRL_QP:setCollisionMargin(i1,i2)
		-- does nothing
	end
	function Physics.DynamicsSimulator_TRL_QP:calcDotJacobian(ichar, ibone, DJ)
		local J=matrixn()
		self:calcJacobianAt(ichar, ibone, J, vector3(0,0,0));
		self:calcDotJacobianAt(ichar, ibone, DJ, vector3(0,0,0));

		local T=self:getWorldState(ichar):globalFrame(1)
		local AdR=liegroup.Ad(transf(T.rotation, vector3(0,0,0)))
		local dotAdR=AdR:copy()
		-- dotR=skew(w)*R where w is the world angular velocity
		local skewW=CT.skew(self:getWorldAngVel(ichar,self:skeleton(ichar):VRMLbone(1)))
		dotAdR:sub(0,3,0,3):assign(skewW* AdR:sub(0,3,0,3))
		dotAdR:sub(3,6,3,6):assign(dotAdR:sub(0,3,0,3))

		DJ:sub(0,0,0,6):assign(DJ:sub(0,0,0,6)*AdR+J:sub(0,0,0,6)*dotAdR)
	end
	function Physics.DynamicsSimulator_TRL_QP:stepKinematic(ddbq, tau, integrate)
		assert(integrate)

		local T=self:getWorldState(0):globalFrame(1)
		local AdR=liegroup.Ad(transf(T.rotation, vector3(0,0,0)))
		self:stepKinematic2(0, (AdR*ddbq:column()):column(0))
	end
	function Physics.DynamicsSimulator_TRL_QP:calcMassMatrix3(ichar, M, b)
		self:calcMassMatrix(0, M, b)

		local T=self:getWorldState(0):globalFrame(1)
		local AdR=liegroup.Ad(transf(T.rotation, vector3(0,0,0)))
		local invAdR=AdR:T()

		-- M ddq + b = u
		--
		-- but dq= AdR*dbq 
		--
		-- ddq = AdR*ddbq 
		--  u  = AdR* bu

		-- thus,
		--
		-- M*AdR * ddbq  + b= AdR bu
		--

		M:sub(0,6,0,6):assign(invAdR*M:sub(0,6,0,6)*AdR)
		b:slice(0,6):assign((invAdR*b:slice(0,6):column()):column(0))


		if b:size()>6 then
			-- [M0    M1] [ddq  ]     +     [b0]  =       [u0]
			-- [M2    M3] [ddq1 ]     +     [b1 ]  =      [u1]
			
			--> 
			-- [M0    M1] [AdR dbq  ]     +     [ b0 ]  =      [AdR bu]
			-- [M2    M3] [ddq1 ]     +     [b1 ]  =      [u1]
			
			--> 
			-- [invAdR*M0   invAdR*M1] [AdR*dbq  ]     + [ invAdR b0 ]  = [bu]
			-- [M2                 M3] [ddq1 ]     +     [b1 ]  =      [u1]


			--> 이미 M0와 b0는 위에서 변환이 되어 있으므로.
			-- [M   invAdR*M1] [dbq ]     +      [b  ]  = [bu]
			-- [M2*AdR     M3] [ddq1 ]     +     [b1 ]  = [u1]

			M:sub(0,6,6,0):assign(invAdR*M:sub(0,6,6,0))
			M:sub(6,0,0,6):assign(M:sub(6,0,0,6)*AdR)
		end
	end

	if not Physics.DynamicsSimulator_Trbdl_penalty then
		Physics.DynamicsSimulator_Trbdl_penalty= Physics.DynamicsSimulator_Trdl_penalty -- only temporalily
	end
	function Physics.DynamicsSimulator_Trbdl_penalty:calcJacobian(ichar, ibone, J)
		self:_calcJacobianAt(ichar, ibone, J, vector3(0,0,0))
		-- swap force and torque packing
		local temp=J:sub(0,0,3,6):copy()
		J:sub(0,0,3,6):assign(J:sub(0,0,0,3))
		J:sub(0,0,0,3):assign(temp)

		-- R*bf (world force) to bf (body force)
		-- J[0:6]=(I  0)*J[0:6] 
		--        (0  R)        
		local R=self:getWorldState(ichar):globalFrame(1).rotation:M()
		J:sub(3,6,3,6):assign33(R)
	end
	function Physics.DynamicsSimulator_Trbdl_penalty:calcDotJacobian(ichar, ibone, DJ)
		self:_calcDotJacobianAt(ichar, ibone, DJ, vector3(0,0,0))
		-- swap force and torque packing
		local temp=DJ:sub(0,0,3,6):copy()
		DJ:sub(0,0,3,6):assign(DJ:sub(0,0,0,3))
		DJ:sub(0,0,0,3):assign(temp)

		local w=self:_bodyW(ichar, 1)
		local R=self:getWorldState(ichar):globalFrame(1).rotation:M()
		local skew_w=matrix3()
		skew_w:setTilde(w)
		local dotR=R*skew_w -- when w is the body angular velocity

		DJ:sub(3,6,3,6):assign33(dotR)
	end
	function Physics.DynamicsSimulator_Trbdl_penalty:setCollisionMargin(i1,i2)
		-- does nothing
	end
	function Physics.DynamicsSimulator_Trbdl_penalty:calcMassMatrix3(ichar, M, b)
		-- QDot[0:3] == R* bv
		-- QDot[3:6] == bw
		-- Tau[0:3] == R* bf
		-- Tau[3:6] == btau
		
		-- H * QDDot + C = Tau
		--
		--
		-- but in taesooLib
		--
		-- dq[0:3] == bw
		-- dq[3:6] == bv
		-- u[0:3] == btau
		-- u[3:6] == bf
		--

		local H=matrixn()
		local C=vectorn()
		self:_calcMassMatrix(ichar, H, C)

		if true then
			-- swap force and torque packing
			-- after swapping,
			-- QDot[0:3] == bw 
			-- QDot[3:6] == R* bv
			-- Tau[0:3] == btau 
			-- Tau[3:6] == R* bf
			local temp=matrixn()
			temp:assign(H:sub(0,0,0,3));
			H:sub(0,0,0,3):assign(H:sub(0,0,3,6));
			H:sub(0,0,3,6):assign(temp);
			temp:assign(H:sub(0,3,0,0))
			H:sub(0,3,0,0):assign(H:sub(3,6,0,0));
			H:sub(3,6,0,0):assign(temp);
			local tempb=C:toVector3(0);
			C:range(0,3):assign(C:range(3,6));
			C:setVec3(3, tempb)
		end

		--> QDot = (  I   0 )*bw  = S * W
		--         (  0   R ) bv
		--> QDDot = S *dW + dS *W
		

		-- [H0    H1] [qdd  ]     +     [C0]  =       [T0]
		-- [H2    H3] [qdd1 ]     +     [C1]  =       [T1]
		-->
		-- [H0    H1] [S * dW + dS* W]  + [C0]  =      [S*U]
		-- [H2    H3] [qdd1          ]  + [C1]  =      [T1]
		-->
		-- [invS*H0   invS*H1] [S * dW + dS* W]  + [invS*C0]  =  [U]
		-- [H2             H3] [qdd1          ]  + [C1]  =      [T1]
		-->
		-- [invS*H0   invS*H1] [S * dW]  + [invS*C0+invS*H0*dS*W]  =  [U]
		-- [H2             H3] [qdd1  ]  + [C1 + H2*dS*W]  =      [T1]
		-->
		-- [invS*H0*S   invS*H1] [dW]  + [invS*C0+invS*H0*dS*W]  =  [U]
		-- [H2*S             H3] [qdd1  ]  + [C1 + H2*dS*W]  =      [T1]
		

		local w=self:_bodyW(ichar, 1)
		local R=self:getWorldState(ichar):globalFrame(1).rotation:M()
		local bv=self:_bodyV(ichar,1)
		local skew_w=matrix3()
		skew_w:setTilde(w)
		local dotR=R*skew_w -- when w is the body angular velocity

		local H0_0=H:toMat33(0,0)
		local H0_1=H:toMat33(0,3)
		local H0_2=H:toMat33(3,0)
		local H0_3=H:toMat33(3,3)

		-- M0 = invS   *     H0      * S
		--    = [I 0 ] * [H0_0 H0_1] *[I 0]
		--      [0 R']   [H0_2 H0_3] *[0 R]
		--     = [H0_0        H0_1] * [I 0]
		--       [R'*H0_2  R'*H0_3]   [0 R]
		--     = [H0_0       H0_1*R]
		--       [R'*H0_2    R'*H0_3*R]

		local M0_1=H0_1*R
		local M0_3=R:T()*H0_3*R
		M:setSize(H:rows(), H:cols())
		M:sub(0,3,0,3):assign33(H0_0)
		M:sub(0,3,3,6):assign33(M0_1)
		M:sub(3,6,0,3):assign33(M0_1:T())
		M:sub(3,6,3,6):assign33(M0_3)

		b:setSize(C:size())
		-- b0=invS*C0+invS*H0*dS*W
		--    = [I 0 ] * [C0_0] + [I 0 ]*[H0_0 H0_1]*[ 0       ]
		--      [0 R']   [C0_1]   [0 R'] [H0_2 H0_3] [ dotR*bv]
		--    = [C0_0   ] + [I 0 ]*[H0_1*dotR*bv]
		--      [R'*C0_1]   [0 R'] [H0_3*dotR*bv]
		--
		local C0_0=C:toVector3(0)
		local C0_1=C:toVector3(3)
		--dS=[0    0]
		--   [0 dotR]
		local dS_W_1=dotR*bv
		b:setVec3(0, C0_0 + H0_1*dS_W_1)
		b:setVec3(3, R:T()*(C0_1 + H0_3*dS_W_1))

		if b:size()>6 then
			-- b1 = C1 + H2*dS*W
			--    = C1 + H2 * [   0  ]
			--                [dS_W_1]
			for j=6, M:cols()-1 do
				local H2_j1=H:row(j):toVector3(3)
				b:set(j, C(j)+ H2_j1:dotProduct(dS_W_1))
			end

			-- [invS*H0*S   invS*H1] [dW]  + [invS*C0+invS*H0*dS*W]  =  [U]
			-- [H2*S             H3] [qdd1  ]  + [C1]  =      [T1]
			
			-- M1=invS*H1
			-- =[I 0 ] * [H1_0]
			--  [0 R']    H1_1
			M:sub(0,3, 6,0):assign(H:sub(0,3, 6,0))
			for j=6, M:cols()-1 do
				M:column(j):setVec3(3, R:T()*H:column(j):toVector3(3))
			end

			-- M2
			M:sub(6,0, 0,6):transpose(M:sub(0,6, 6,0))

			-- M3
			M:sub(6,0, 6,0):assign(H:sub(6,0, 6,0))
		end
	end
	function Physics.DynamicsSimulator_Trbdl_penalty:stepKinematic(ddbq, tau, integrate)
		assert(integrate)

		--> QDot = (  I   0 )*bw  = S * W
		--         (  0   R ) bv
		--> QDDot = S *dW + dS *W
		--
		
		local ichar=0
		local w=self:_bodyW(ichar, 1)
		local R=self:getWorldState(ichar):globalFrame(1).rotation:M()
		local bv=self:_bodyV(ichar, 1)

		local skew_w=matrix3()
		skew_w:setTilde(w)
		local dotR=R*skew_w -- when w is the body angular velocity

		local dW0=ddbq:toVector3(0)
		local dW1=ddbq:toVector3(3)
		local QDDot0= dW0
		local QDDot1= R*dW1 + dotR*bv
		local QDDot=ddbq:copy()

		-- swap 0 and 3
		QDDot:setVec3(3, QDDot0)
		QDDot:setVec3(0, QDDot1)
		
		self:_stepKinematic(ichar, QDDot)
	end
	Physics.DynamicsSimulator_TRL_LCP.calcJacobian= Physics.DynamicsSimulator_TRL_QP.calcJacobian
	Physics.DynamicsSimulator_TRL_LCP.calcDotJacobian= Physics.DynamicsSimulator_TRL_QP.calcDotJacobian
	Physics.DynamicsSimulator_TRL_LCP.calcBodyDotJacobianAt= Physics.DynamicsSimulator_TRL_QP.calcBodyDotJacobianAt
	Physics.DynamicsSimulator_TRL_LCP.calcBoneDotJacobian= Physics.DynamicsSimulator_TRL_QP.calcBoneDotJacobian
	Physics.DynamicsSimulator_TRL_LCP.calcMassMatrix3= Physics.DynamicsSimulator_TRL_QP.calcMassMatrix3

	if not Physics.DynamicsSimulator_Trbdl_LCP then
		Physics.DynamicsSimulator_Trbdl_penalty=nil
	else
		Physics.DynamicsSimulator_Trbdl_LCP.calcJacobian= Physics.DynamicsSimulator_Trbdl_penalty.calcJacobian
		Physics.DynamicsSimulator_Trbdl_LCP.calcDotJacobian= Physics.DynamicsSimulator_Trbdl_penalty.calcDotJacobian
		Physics.DynamicsSimulator_Trbdl_LCP.calcMassMatrix3= Physics.DynamicsSimulator_Trbdl_penalty.calcMassMatrix3
		Physics.DynamicsSimulator_Trbdl_LCP.stepKinematic= Physics.DynamicsSimulator_Trbdl_penalty.stepKinematic
	end
end
function vector3N:set(i,v)
	self(i):assign(v)
end
HessianQuadratic.add=QuadraticFunctionHardCon.add
-- add objective function term weight*( x_i -value)^2
function HessianQuadratic:addD(weight, index, value)
	local sqw=math.sqrt(weight)
	local i=CT.ivec(index)
	local v=CT.vec(sqw,sqw*-1*value)
	self:addSquared(i,v)
end
function HessianQuadratic:addSquaredW(weight, i, v)
	self:addSquared(i, math.sqrt(weight)*v)
end
-- minimize weight*(M * x[si:ei) +b)^2
function HessianQuadratic:addV(weight, M, si, ei,b)
	local i=CT.colon(si,ei,1)
	assert(M:cols()==ei-si)
	local v=vectorn(M:cols()+1)
	local sqw=math.sqrt(weight)
	for j=0,M:rows()-1 do
		v:range(0,v:size()-1):assign(M:row(j))
		v:set(v:size()-1, b(j,0))
		v:rmult(sqw)
		self:addSquared(i,v)
	end
	--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"addv",self.R:copy()}) --##dos end 
end 

function HessianQuadratic:addVselective(weight, M, si, ei,b, index)
	local i=CT.colon(si,ei,1)
	assert(M:cols()==ei-si)
	local v=vectorn(M:cols()+1)
	local sqw=math.sqrt(weight)
	for j=0,index:size()-1 do
		v:range(0,v:size()-1):assign(M:row(index(j)))
		v:set(v:size()-1, b(index(j),0))
		v:rmult(sqw)
		self:addSquared(i,v)
	end
	--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"vsel",self.R:copy()}) --##dos end 
end 
function Physics.ContactBasis:__tostring()
	return string.format("%d %d (%s) n:(%s) f:%d %d %d\n", self.ibody, self.ibone,tostring( self.globalpos), tostring(self.normal), self.frictionNormal:size(), self.globalIndex, self.globalFrictionIndex)
end
function Physics.Vec_ContactBasis:__tostring()
	local out=""
	for i=0, self:size()-1 do
		out=out..i..": "..tostring(self(i))
	end
	return out
end

--class 'QPservo'
QPservo=LUAclass()

function QPservo:setCoef(dofInfo,kp, kd, tgtVelScale, k_scale)
	kp:setSize(dofInfo:numDOF())
	kp:setAllValue(k_p)
	kd:setSize(dofInfo:numDOF())
	kd:setAllValue(k_d)
	tgtVelScale:setSize(dofInfo:numDOF())
	tgtVelScale:setAllValue(1)
	
	if self.excludeRoot then
		-- exclude root joint
		kp:range(0,7):setAllValue(0)
		kd:range(0,7):setAllValue(0)
	end
	
	--print("initQPservo:"..dofInfo:skeleton():bone(1):name())
	for i=2,dofInfo:skeleton():numBone()-1 do
		local bone=dofInfo:skeleton():bone(i)
		local vbone=bone:treeIndex()
		local nJoint=dofInfo:numDOF(vbone)
		--      print("initQPservo:"..bone:name())
		for j=0, nJoint-1 do
			
			local dofIndex=dofInfo:DOFindex(vbone,j)
			
			kp:set(dofIndex, k_p*k_scale.default[1])
			kd:set(dofIndex, k_d*k_scale.default[2])
			tgtVelScale:set(dofIndex, k_scale.default[3])

			if bone:voca()==MotionLoader.LEFTANKLE or bone:voca()==MotionLoader.RIGHTANKLE then
				if k_scale.ankle~=nil then
					kp:set(dofIndex, k_p*k_scale.ankle[1])
					kd:set(dofIndex, k_d*k_scale.ankle[2])
					tgtVelScale:set(dofIndex, k_scale.ankle[3])
				end
			elseif bone:voca()==MotionLoader.LEFTWRIST or bone:voca()==MotionLoader.RIGHTWRIST then
				if k_scale.wrist~=nil then
					kp:set(dofIndex, k_p*k_scale.wrist[1])
					kd:set(dofIndex, k_d*k_scale.wrist[2])
					tgtVelScale:set(dofIndex, k_scale.wrist[3])
				end
			elseif bone:voca()==MotionLoader.LEFTCOLLAR or bone:voca()==MotionLoader.RIGHTCOLLAR then
				if k_scale.collar~=nil then
					kp:set(dofIndex, k_p*k_scale.collar[1])
					kd:set(dofIndex, k_d*k_scale.collar[2])
					tgtVelScale:set(dofIndex, k_scale.collar[3])
				end
			elseif bone:voca()==MotionLoader.LEFTSHOULDER or bone:voca()==MotionLoader.RIGHTSHOULDER then
				if k_scale.shoulder~=nil then
					kp:set(dofIndex, k_p*k_scale.shoulder[1])
					kd:set(dofIndex, k_d*k_scale.shoulder[2])
					tgtVelScale:set(dofIndex, k_scale.shoulder[3])
				end
			elseif bone:voca()==MotionLoader.LEFTELBOW or bone:voca()==MotionLoader.RIGHTELBOW then
				if k_scale.elbow~=nil then
					kp:set(dofIndex, k_p*k_scale.elbow[1])
					kd:set(dofIndex, k_d*k_scale.elbow[2])
					tgtVelScale:set(dofIndex, k_scale.elbow[3])
				end
			elseif bone:voca()==MotionLoader.LEFTKNEE or bone:voca()==MotionLoader.RIGHTKNEE then
				if k_scale.knee~=nil then
					kp:set(dofIndex, k_p*k_scale.knee[1])
					kd:set(dofIndex, k_d*k_scale.knee[2])
					tgtVelScale:set(dofIndex, k_scale.knee[3])
				end
			elseif bone:voca()==MotionLoader.LEFTHIP or bone:voca()==MotionLoader.RIGHTHIP then
				if k_scale.hip~=nil then
					kp:set(dofIndex, k_p*k_scale.hip[1])
					kd:set(dofIndex, k_d*k_scale.hip[2])
					tgtVelScale:set(dofIndex, k_scale.hip[3])
				end
			elseif bone:voca()==MotionLoader.CHEST then
				if k_scale.chest~=nil then
					kp:set(dofIndex, k_p*k_scale.chest[1])
					kd:set(dofIndex, k_d*k_scale.chest[2])
					tgtVelScale:set(dofIndex, k_scale.chest[3])
				end
			elseif bone:voca()==MotionLoader.CHEST2 then
				if k_scale.chest2~=nil then
					kp:set(dofIndex, k_p*k_scale.chest2[1])
					kd:set(dofIndex, k_d*k_scale.chest2[2])
					tgtVelScale:set(dofIndex, k_scale.chest2[3])
				end
			elseif bone:voca()==MotionLoader.NECK then
				if k_scale.neck~=nil then
					kp:set(dofIndex, k_p*k_scale.neck[1])
					kd:set(dofIndex, k_d*k_scale.neck[2])
					tgtVelScale:set(dofIndex, k_scale.neck[3])
				end
			elseif bone:voca()==MotionLoader.HEAD then
				if k_scale.head~=nil then
					kp:set(dofIndex, k_p*k_scale.head[1])
					kd:set(dofIndex, k_d*k_scale.head[2])
					tgtVelScale:set(dofIndex, k_scale.head[3])
				end
			end
			if dofInfo:DOFtype(vbone, j)==MotionDOFinfo.SLIDE then
				local dofIndex=dofInfo:DOFindex(vbone,j)
				kp:set(dofIndex, k_p_slide)
				kd:set(dofIndex, k_d_slide)
				tgtVelScale:set(dofIndex, 0)
			end
		end
	end
end

function QPservo:updateCoef()
	local dofInfo=self.dofInfo

	k_p=1
	k_d=1
	k_p_slide=5
	k_d_slide=5

	-- self:setIDGain(dofInfo:skeleton(), self.kp_id, self.kd_id, k_p, k_d, k_p_slide or k_p*5, k_d_slide or k_d*5)
	self.weight=vectorn()
	local bigJointsAccCoef=QPparam.bigJointsAccCoef or 1
	model.k_scale_id.hip={bigJointsAccCoef,bigJointsAccCoef,1}
	model.k_scale_id.chest={bigJointsAccCoef,bigJointsAccCoef,1}
	--model.k_scale_id.ankle={0.5,0.5,1}
	self:setCoef(dofInfo, self.kp_id, self.kd_id, self.weight, model.k_scale_id)

	self.weight2=vectorn()

	local temp=vectorn()

	local bigJointsTorqueCoef=QPparam.bigJointsTorqueCoef or 1
	local k_torque_bound={
		default={1,1,1}, 
		hip={bigJointsTorqueCoef,1,1},
		chest={bigJointsTorqueCoef,1,1},
		--ankle={1/bigJointsTorqueCoef,1,1},
		--ankle={0.5,1,1},
	}

	self:setCoef(dofInfo, self.weight2,temp,temp, k_torque_bound)

end


function QPservo:__init(dofInfo, timestep,integrator, simulator)
	self.simulator=simulator
	self.dtinv=1/timestep
	self.invfricCoef=QPparam.invFricCoef or 1
	-- settings
	self.excludeRoot=QPparam.excludeRoot
	self.excludeRootFlight=QPparam.excludeRootFlight or false
	-- defalt
	self.state={
		previousFlightPhase=false, flightPhase=false, supportPhaseElapsed=100, flightPhaseElapsed=0, 
		-- added for QPparam TRL
		desiredSpeedX=0,
		desiredSpeedZ=0,
		dthetaX=0,
		dthetaZ=0,
	}
	self.numCLinks=simulator:getNumAllLinkPairs()
	local index=intvectorn()
	simulator:getContactLinkBoneIndex(0,index)

	--_checkpoints:pushBack(deepCopyTable({'clinks', self.numCLinks, index}))

	if QPparam.createSim then
		QPparam.createSim(self, simulator)
	end


	--_checkpoints:pushBack(deepCopyTable({'state', state}))
	self.theta=vectorn()
	self.dtheta=vectorn()
	-- HD servo
	self.theta_d=vectorn() -- desired q
	self.dtheta_d=vectorn() -- desired dq
	self.ddtheta_d=vectorn() -- desired ddq

	-- PD servo
	--self.theta_d_pd=vectorn()

	self.desiredacceleration=vectorn()
	self.controlforce=vectorn()
	self.kp=vectorn()
	self.kd=vectorn()
	self.kp_id=vectorn()
	self.kd_id=vectorn()

	self.tgtVelScale=vectorn()
	self.mask_slide=vectorn()
	

	-- lleg+rleg+upperbody=all
	self.mask_lleg=vectorn() -- excluding sliding joints
	self.mask_rleg=vectorn() -- excluding sliding joints
	self.mask_upperbody=vectorn()
	self.scale_lleg=1
	self.scale_rleg=1
	self.scale_upperbody=1
	
	self.mask_slide:setSize(dofInfo:numDOF())
	self.mask_slide:setAllValue(0)
	self.mask_lleg:setSize(dofInfo:numDOF())
	self.mask_rleg:setSize(dofInfo:numDOF())
	self.mask_upperbody:setSize(dofInfo:numDOF())
	self.mask_lleg:setAllValue(0)
	self.mask_rleg:setAllValue(0)
	self.mask_upperbody:setAllValue(1)

	self.dofInfo=dofInfo
	self:updateCoef()
	print ("kp=",self.kp)
	print ("kd=",self.kd)

	local skel=dofInfo:skeleton()

	local function setClampMax(clampTorque)
		local clampMax=vectorn(dofInfo:numDOF())
		clampMax:setAllValue(clampTorque)
		return clampMax
	end

	local clampTorque=model.clampTorqueID or 400

	self.clampMaxID=setClampMax(clampTorque)

	
	self.clampMinID=self.clampMaxID*-1

	self.numActualDOF=dofInfo:numActualDOF()
	self.workspace={}
	local w=self.workspace
	w.M=matrixn()
	w.b=vectorn(self.numActualDOF)
	w.JtV=matrixn()
	-- for friction cones
	w.J=matrixn()
	w.dotJ=matrixn()
	w.V=matrixn()
	w.dotV=matrixn()


	w.Mlcp=matrixn()
	w.Mlcp_bias=vectorn()
	w.CE=matrixn()
	w.ce0=vectorn()
	w.CI=matrixn()
	w.ci0=vectorn()
	w.x=vectorn()
	w.x_lcp=vectorn()
	w.CElcp=matrixn()
	w.ce0lcp=vectorn()
	w.CIinvM=matrixn()

	w.clearCI=function(self, max_numCon, totalDIM) -- max_numCon은 정확할 필요 없음. 대충 적당히 넉넉히 주면 메모리 재할당이 줄듯.
		self.CI:setSize(max_numCon, totalDIM)
		self.CI:resize(0,totalDIM)
		self.ci0:setSize(max_numCon)
		self.ci0:resize(0)
	end
	w.addCI=function(self, CI, startIndex)
		local nrow=self.CI:rows()
		self.CI:resize(nrow+CI:rows(), self.CI:cols())
		self.CI:sub(nrow, nrow+CI:rows(),startIndex, startIndex+CI:cols()):assign(CI)
	end
	w.addCI0=function(self, ci0)
		local nrow=self.ci0:size()
		self.ci0:resize(nrow+ci0:size())
		self.ci0:range(nrow, nrow+ci0:size()):assign(ci0)
		assert(self.ci0:size()==self.CI:rows())
	end

	return o
end

function QPservo:initQPservo(startf, endf,motionDOF, dmotionDOF, ddmotionDOF)--, motionDOF_pdtarget)

	self.startFrame=startf
	self.endFrame=endf
	self.currFrame=startf
	self.deltaTime=0
	self.motionDOF=motionDOF
	self.dmotionDOF=dmotionDOF
	self.ddmotionDOF=ddmotionDOF
	--self.motionDOF_pdtarget=motionDOF_pdtarget or motionDOF

end
function QPservo:_updateCurrFrame()
	local simulator=self.simulator
	self.currFrame=(simulator:currentTime()+self.deltaTime)*model.frame_rate+self.startFrame
	if self.currFrame> self.motionDOF:rows()-1 then
		-- rewind
		self.deltaTime=-1*simulator:currentTime()
	end
end

-- generate FBtorque
function QPservo:buildProblem(maxForce, contactpos)
	self:_updateCurrFrame()
	self:sampleCurrPose()
	self:calcDesiredAcceleration(self.currFrame)
	self:_buildProblem( maxForce, contactpos)
	return true
end
function QPservo:_calcDesiredAcceleration()
	local state=self.theta
	local dstate=self.dtheta
	self.desiredacceleration:setSize(self.dofInfo:numDOF())
	
	local delta=self.theta_d-state
	MainLib.VRMLloader.projectAngles(delta) -- [-pi, pi]

	if true then
		local T1=MotionDOF.rootTransformation(state)
		local T2=MotionDOF.rootTransformation(self.theta_d)
		T2.rotation:align(T1.rotation)
		local V=T1:twist(T2,1)

		delta:setVec3(0, V.v)
		delta:set(3,0)
		delta:setVec3(4, V.w)
	else
		local q=state:toQuater(3)
		local q_d=self.theta_d:toQuater(3)
		q:align(q_d)
		local v=q:inverse()*(self.theta_d:toVector3(0)-state:toVector3(0))
		--q*w=theta_d
		local w=(q:inverse()*q_d):rotationVector()

		delta:setVec3(0, v)
		delta:set(3,0)
		delta:setVec3(4, w)
	end

	--self.desiredacceleration:setAllValue(0)
	--self.dtheta_d:range(0,7):setAllValue(0)

	local ddelta=(self.dtheta_d-dstate)

	self.desiredacceleration:assign(self.kp_id*delta*QPparam.k_p_ID +  self.kd_id*ddelta*QPparam.k_d_ID)

	--self.desiredacceleration:range(0,7):setAllValue(0)
	local accClamp=QPparam.desiredAccThr or 400
	self.desiredacceleration:smoothClamp(-accClamp, accClamp)

	self.ddtheta_d:range(0,7):setAllValue(0)
	self.desiredacceleration:radd(self.ddtheta_d)

	--print(self.ddtheta_d)
	--self.desiredacceleration:clamp(-400, 400)

--##dos if g_debugOneStep then --##dos --##dos g_debugOneStep:pushBack({"theta",self.theta:copy()}) --##dos g_debugOneStep:pushBack({"dtheta",self.dtheta:copy()}) --##dos g_debugOneStep:pushBack({"dtheta_d",self.dtheta_d:copy()}) --##dos g_debugOneStep:pushBack({"kp_id",self.kp_id:copy()}) --##dos g_debugOneStep:pushBack({"kd_id",self.kd_id:copy()}) --##dos g_debugOneStep:pushBack(QPparam.QPservoDScaleCoef ) --##dos g_debugOneStep:pushBack({"delta",delta:copy()}) --##dos g_debugOneStep:pushBack({"ddtheta_d",self.ddtheta_d:copy()}) --##dos g_debugOneStep:pushBack({"desiredAcc",self.desiredacceleration:copy()}) --##dos end
end


-- deprecated: use _calcDesiredAcceleration
function QPservo:calcDesiredAcceleration(frame)

	local simulator=self.simulator

	--[[ continuous sampling ]]--
	--   print("theta",self.theta)

	self:sampleTargetPoses(frame)

	--   self.dtheta_d:setAllValue(0)
	self:_calcDesiredAcceleration()
end

function QPservo:sampleCurrPose()
	local simulator=self.simulator
	simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
	simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
end
function QPservo:restoreSampledPose()
	local simulator=self.simulator
	simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
	simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
	simulator:initSimulation()
end
function QPservo:sampleTargetPoses( frame)
	-- desired (target) pose
	self.motionDOF:samplePose(frame, self.theta_d)
	if debug_mode then
		print('sample', frame,'\n', self.theta_d)
	end
--	self.motionDOF_pdtarget:samplePose(frame, self.theta_d_pd)
	self.dmotionDOF:sampleRow(frame, self.dtheta_d)
	self.ddmotionDOF:sampleRow(frame, self.ddtheta_d)
end

function QPservo:stepSimul( _option, impulse)
	local QPparam=QPparam
	local simulator=self.simulator
	if false then
		self:stepSimul_old( impulse)
	else -- use QP solution directly
		local qp=self.qp
		local w=self.workspace
		local numActualDOF=self.numActualDOF
		--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"qpR",qp.H:copy(), qp.R:copy()}) --##dos end
		
		if impulse then
			local impulses
			if impulse.lf then
				impulses={impulse}
			else
				impulses=impulse
			end

			for i, impulse in ipairs(impulses) do
				-- actually, external forces (not impulses)
				local J=matrixn()
				simulator:calcJacobian(0,impulse.treeIndex,J)
				local GT=simulator:getWorldState(0):globalFrame(impulse.treeIndex)
				-- spatial coordinate to bone global coordinate
				local vk=vectorn(6)

				local p=GT:toGlobalPos(impulse.lpos)
				local n=GT:toGlobalDir(impulse.lf)
				local pcn=p:cross(n)

				-- transform the global generalized force (0, n) to spatial coordinate
				-- local f=liegroup.dse3(vector3(0,0,0), n)
				-- vk=f:invdAd(transf(quater(1,0,0,0), p)))
				vk:setVec3(0, pcn)
				vk:setVec3(3, n)
				local R_dAd= liegroup.invdAd(transf(GT.rotation,vector3(0,0,0))*GT:inverse())

				local tau=matrixn()
				tau:multAtB(J, R_dAd*vk:column())

				w.ce0:range(0,numActualDOF):rsub(tau:column(0))
			end
		end
		if w.CI_additional and w.CI:cols()> 0 then
			for i,v in ipairs(w.CI_additional) do
				w:addCI(v[1], v[2])
				w:addCI0(v[3])
			end
		end
		--timerQP2:start()
		Eigen.solveQuadprog(qp, w.CE, w.ce0, w.CI, w.ci0, w.x)
		--timerQP2:pause()
		--Eigen.solveQuadprog(qp,w.CE, w.ce0, w.CI, w.ci0, w.x, true)
		--_checkpoints:pushBack(deepCopyTable({'quadprog', w.CE, w.ce0, w.CI, w.ci0, w.x}))

		if false then
			if _count==1 then
				if util.isFileExist('_checkpoints.tbl') then
					tbl2=util.loadTable('_checkpoints.tbl')
					util.compareTable(_checkpoints, tbl2)
				else
					util.saveTable(_checkpoints, '_checkpoints.tbl')
				end
				dbg.console()
			end
			_count=_count+1
		end

		assert(w.x==w.x)
		self.controlforce:range(0,7):setAllValue(0)
		self.controlforce:range(7,self.controlforce:size()):assign(w.x:range(numActualDOF+6,numActualDOF*2))

		--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"qpcontrolForce", w.x:copy(), w.CE:copy(), w.ce0:copy(), w.CI:copy(), w.ci0:copy()}) end 
		local w=self.workspace
		local numDOF=self.numActualDOF
		local ddq=w.x:range(0,numDOF)
		local tau = w.x:range(numDOF, numDOF*2)
		local lambda= w.x:range(numDOF*2, w.x:size())

		local aggCF=vector3(0,0,0)

		local mat=nil -- do not draw contact forces
		if true then -- draw contact force
			local lambda=w.x:range(self.numActualDOF*2, w.x:size())
			local skel=self.dofInfo:skeleton()

			
			if lambda:size()>0 then 

				local w=self.workspace
				assert(w.bases and  w.bases:size()>0)
				local collector
				if mrd_info and mrd_info.outputContactForce then
					collector=mrd_info.outputContactForce[2]
				else
					collector={vector3(0,0,0), vector3(0,0,0), vector3(0,0,0)}
				end
				local limbs={L=1,R=2,O=3}
				for i=1,3 do collector[i]:assign(vector3(0,0,0)) end
				if self.drawDebugInformation then
					mat=vector3N(w.bases:size()) -- draw contact forces

				end

				for i=0, w.bases:size()-1 do
					local b=w.bases(i)
					local limb=limbs.O

					local cb=collector[limb]
					local cf=vector3(0,0,0)
					for j=0, b.normals:size()-1 do
						local gi=b.globalIndex+j
						cf:radd(lambda(gi)*b.normals(j))
					end
					cb:radd(cf)
					if mat then
						mat(i):assign(cf)
					end
				end
				for i=1,3 do 
					aggCF:radd(collector[i])
				end
			else
				mat=vector3N(0)
				mat:matView():setAllValue(0)
			end
		end
		--print('ddq', ddq)
		--print('before 2', self.theta)
		if QPparam.stepSimulation then
			QPparam.stepSimulation(self, tau)
		elseif true then
			simulator:stepKinematic(ddq, vectorn(), true)
		else
			local link_pair_count=w.link_pair_count

			local numDOF=self.numActualDOF
			local rootR = self.simulator:getWorldState(0):globalFrame(1).rotation
			local function packTau( tau)
				local genForce=vectorn(numDOF+1)
				local M = tau:toVector3(0)
				local F = tau:toVector3(3)
				--M:rotate(rootR)
				--F:rotate(rootR)
				genForce:setVec3(4, M)
				genForce:setVec3(0, F)
				genForce:range(7,genForce:size()):assign(tau:range(6,tau:size()))
				return genForce
			end
			local controlForce=packTau( tau)
			if link_pair_count>0 then
				local cf=w.JtV*lambda:column()
				local genContactForce=packTau( cf:column(0))

				controlForce:radd(genContactForce)
			end
			simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, controlForce)
			simulator:stepSimulation()
		end
		return mat
	end
end
function QPservo:_buildProblem(maxForce, contactpos )

	if not contactpos then
		self:QPsolve()
	else
		local link_pair_count=contactpos:size()
		if link_pair_count>0 then
			local w=self.workspace
			w.bases = vector()
			for i=0, link_pair_count-1 do
				local cinfo={
					--depth=0,
					--ilinkpair=i,
					globalIndex=i,
					globalpos=vector3(0,0,0),
					ibody=0,
					ibone=1,
					normal=vector3(0,1,0),
					normals=vector3N(),
					relvel=vector3(0,0,0),
				}
				cinfo.normals=vector3N(5)
				cinfo.normals:set(0, vector3(0,1,0))

				local angle=math.atan(1/(QPparam.invFricCoef or 1))

				cinfo.normals:set(1, quater(-angle, vector3(1,0,0))*vector3(0,1,0))
				cinfo.normals:set(2, quater(-angle, vector3(0,0,1))*vector3(0,1,0))
				cinfo.normals:set(3, quater(angle, vector3(1,0,0))*vector3(0,1,0))
				cinfo.normals:set(4, quater(angle, vector3(0,0,1))*vector3(0,1,0))
				w.bases:pushBack(cinfo)
			end

			for i=0, contactpos:size()-1 do
				w.bases(i).globalpos:assign(contactpos(i))
			end
		end

		self:_QPsolve(link_pair_count)
	end
end

function QPservo:calcDQ()
	local dq=vectorn(self.dtheta:size()-1)
	self:_calcDQ(dq)
	return dq
end
function QPservo:_calcDQ(dq)
	-- now use body frame for both TRL and gmbs
	local simulator=self.simulator
	-- gmbs
	dq:range(0,3):assign(self.dtheta:range(4,7)) 
	dq:range(3,6):assign(self.dtheta:range(0,3)) 
	dq:range(6,dq:size()):assign(self.dtheta:range(7,self.dtheta:size()))
end
function QPservo:addCOMobjective(sim, desiredCOMacc, weight)
	dbg.console()
end
function QPservo:addMomentumObjective(sim_unused,desiredDotAngMomentum, desiredDotLinMomentum,weight_ang, weight_lin)
	dbg.console()
end
function QPservo:_getContactBases()
	local simulator=self.simulator
	local link_pair_count=simulator:getNumContactLinkPairs()
	if link_pair_count==0 then
		return link_pair_count, nil
	end
	local function updateFrictionIndex(basesin)
		local fi=0
		for i=0, basesin:size()-1 do
			local b=basesin(i)
			b.globalIndex=fi
			fi=fi+b.normals:size()
		end
	end
	local function convertBasis(b)
		local cinfo={
			depth=b.depth,
			ilinkpair=b.ilinkpair,
			globalpos=b.globalpos:copy(),
			normal=b.normal:copy(),
			ibone=b.ibone,
			ibody=b.ibody,
			relvel=b.relvel:copy(),
		}
		local nbasis=b.frictionNormal:size()+1
		cinfo.normals=vector3N(nbasis)
		cinfo.normals:set(0, b.normal)
		for j=0, nbasis-2 do
			cinfo.normals:set(j+1, b.frictionNormal(j):copy())
		end
		return cinfo
	end
	local function convertBases(basesout, basesin)
		basesout:resize(basesin:size())
		for i=0, basesin:size()-1 do
			local b=basesin(i)
			assert(basesout(i)==nil)
			local cinfo=convertBasis(b)
			basesout:set(i, cinfo)
		end
		updateFrictionIndex(basesout)
	end
	local w=self.workspace
	do
		local bases=Physics.Vec_ContactBasis()
		simulator:getContactBases(bases,self.invfricCoef)

		w.bases = vector()

		convertBases(w.bases, bases)
		return link_pair_count
	end
end
function QPservo:QPsolve()
		--timerQP2:pause()
	local link_pair_count=self:_getContactBases()
	return self:_QPsolve(link_pair_count)
end
function QPservo:_QPsolve(link_pair_count)
	--_checkpoints:pushBack(deepCopyTable({'mass', w.M, w.b, link_pair_count}))
	--print('state', self.theta, self.dtheta)
	--print('mass', w.M, w.b, link_pair_count)
	--self.simulator:skeleton(0):printHierarchy()
	--for i=1, self.simulator:skeleton(0):numBone()-1 do print(self.simulator:skeleton(0):VRMLbone(i):mass()) end

--##dos if g_debugOneStep then --##dos print('_qpsolv') --##dos g_debugOneStep:pushBack({'state', state:copy(), dstate:copy()}) --##dos g_debugOneStep:pushBack({'desiredacc', self.desiredacceleration:copy()}) --##dos end

	local simulator=self.simulator

	
	local numActualDOF=self.numActualDOF
	local w=self.workspace
		--timerQP2:start()
	simulator:calcMassMatrix3(0, w.M, w.b)
	--simulator:calcMassMatrix_local(0, w.M, w.b)

	if false then
		self:testMassMatrix()
	end
	self.controlforce:setSize(numActualDOF+1)
	self.controlforce:setAllValue(0)
	w.link_pair_count=link_pair_count

	local state=self.state
	w.CI_additional=nil
	w.CE_additional={}
	if link_pair_count>0 then
		function calcContactJacobianAll(bases, J, dotJ, V, dotV, linkPairCount, frictionCoef)
			local V_temp = matrixn()
			local dotV_temp = matrixn()
			local N = numActualDOF
			local numTorqueVector 
			local a -- the last basis index +1
			if bases:size()>0 then
				local lastBasis=bases(bases:size()-1)
				a= lastBasis.globalIndex+lastBasis.normals:size()
			else
				a=0
			end


			J:setSize(6*linkPairCount,N)
			dotJ:setSize(6*linkPairCount,N)
			V:setSize(6*linkPairCount,a)
			dotV:setSize(6*linkPairCount,a)
			V_temp:setSize(6,a)
			dotV_temp:setSize(6,a)

			local dot_R_dAd=matrixn()
			dot_R_dAd:setSize(6,6)
			dot_R_dAd:setAllValue(0)

			for i=0, bases:size()-1 do
				local b=bases(i)
				local JDJ_i=w.jCache[b.ibody+1][b.ibone+1]
				local J_i,dotJ_i
				if JDJ_i then
					J_i=JDJ_i[1]
					dotJ_i=JDJ_i[2]
				else
					J_i=matrixn()
					dotJ_i=matrixn()
					J_i:setSize(6,N)
					dotJ_i:setSize(6,N)
					simulator:calcJacobian(b.ibody, b.ibone, J_i)
					simulator:calcDotJacobian(b.ibody, b.ibone, dotJ_i)
					w.jCache[b.ibody+1][b.ibone+1]={J_i, dotJ_i}
				end

				assert((i+1)*6<=J:rows())
				assert(N==J_i:cols())
				J:range( i*6, (i+1)*6, 0, N):assign(J_i)
				dotJ:range( i*6, (i+1)*6, 0, N):assign(dotJ_i)

				do
					V_temp:setAllValue(0)
					dotV_temp:setAllValue(0)

					local p = b.globalpos:copy()

					local globalIndex = b.globalIndex
					local vk 
					local dot_vk 
					assert(globalIndex>=0 and globalIndex<V_temp:cols())
					local dot_p = vector3(0,0,0)
					dot_p = -1*b.relvel:copy()

					for j=0, b.normals:size()-1 do
						local vk=V_temp:column(globalIndex+j);
						local dot_vk=dotV_temp:column(globalIndex+j)
						local n=b.normals(j)
						local pcn=p:cross(n)

						-- transform the global generalized force (0, n) to spatial coordinate
						-- local f=liegroup.dse3(vector3(0,0,0), n)
						-- vk+=f:invdAd(transf(quater(1,0,0,0), p)))
						vk:setVec3(0, vk:toVector3(0)+pcn)
						vk:setVec3(3, vk:toVector3(3)+n)
						pcn=dot_p:cross(n)
						dot_vk:setVec3(0,dot_vk:toVector3(0)+pcn)
					end

				end
				local GT=simulator:getWorldState(0):globalFrame(b.ibone)
				-- spatial coordinate to bone global coordinate
				local R_dAd= liegroup.invdAd(transf(GT.rotation,vector3(0,0,0))*GT:inverse())

				V:range(i*6, (i+1)*6, 0, a):assign(R_dAd*V_temp)

				local iboneVel=-1*simulator:getWorldVelocity(0, simulator:skeleton(0):VRMLbone(b.ibone), vector3(0,0,0)):copy()
				dot_R_dAd:range(0,3,3,6):assign(CT.skew(iboneVel):copy())
				dotV:range(i*6, (i+1)*6, 0, a):assign(dot_R_dAd*V_temp + R_dAd*dotV_temp)
			end
		end
		w.jCache={{},{},{},{}} 
		if true then
			calcContactJacobianAll(w.bases, w.J, w.dotJ, w.V, w.dotV, w.bases:size(), self.invfricCoef)
			w.JtV:multAtB(w.J, w.V)
		else
			simulator:calcContactJacobian(w.JtV, link_pair_count)
		end
		if w.VtJ==nil then w.VtJ=matrixn() end
		if w.VtDotJ==nil then w.VtDotJ=matrixn() end
		if w.dotVtJ==nil then w.dotVtJ=matrixn() end
		do
			w.VtJ:transpose(w.JtV)
			w.VtDotJ:multAtB(w.V, w.dotJ)
			w.dotVtJ:multAtB(w.dotV, w.J)
		end
		local cdim=w.JtV:cols()
		local totalDIM=numActualDOF*2+cdim -- ddq, tau, lambda
		local qp=HessianQuadratic(totalDIM)
		self.qp=qp
		local ddqObjWeight=QPparam.ddqObjWeight or 10000
		local ddqObjWeightR=QPparam.ddqObjWeightR or ddqObjWeight
		local ddqObjWeight2=QPparam.ddqObjWeight2 or ddqObjWeight
		local weight=self.weight
		assert(ddqObjWeight)
		-- minimize desired acc error
		if self.excludeRoot then
			local w=1
			for i=0,3 do -- root
				qp:addD(w,i,0)
			end
			for i=3,6 do -- root
				qp:addD(w,i,0)
			end
		else
			for i=0,2 do -- root
				qp:addD(ddqObjWeightR*self.weight(i+4),i,self.desiredacceleration(i+4))
			end
			if QPparam.excludeRootPos then
				for i=3,5 do -- root
					qp:addD(1,i,self.desiredacceleration(i-3))
				end
			else
				for i=3,5 do -- root
					qp:addD(ddqObjWeight*self.weight(i-3),i,self.desiredacceleration(i-3))
				end

				--local dv=self.desiredacceleration:toVector3(0)
				--local R=simulator:getWorldState(0):globalFrame(1).rotation
				--dbg.draw('Arrow', vector3(0,100,0), vector3(0,100,0)+rotate(dv,R)*100,'R*dv')
			end
		end

		--_checkpoints:pushBack(deepCopyTable({'wi', self.weight, self.desiredacceleration}))
		do
			for i=6,numActualDOF-1 do
				local imp=1 
				local w=sop.map(imp, 0,1, ddqObjWeight2, ddqObjWeight)
				qp:addD(w*self.weight(i+1),i,self.desiredacceleration(i+1))
			end
		end

		--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"qp1",qp.H:copy(), qp.R:copy()}) --##dos end

		-- minimize joint torque
		if QPparam.tauObjWeight>1 then
			local w=QPparam.tauObjWeight or 0.00001
			for i=0,5 do
				qp:addD(0.00001,i+numActualDOF,0)
			end
			for i=6,numActualDOF-1 do
				qp:addD(w*self.weight2(i+1),i+numActualDOF,self.desiredacceleration(i+1)*2)
			end
		else
			local w=QPparam.tauObjWeight or 0.00001
			assert(w)
			for i=0,numActualDOF-1 do
				qp:addD(w,i+numActualDOF,0)
			end
		end
		if true then
			-- minimize contact force
			local lw=QPparam.lambdaObjWeight or 0.00001
			local lw2=QPparam.lambdaObjWeight2 or 10

			--lw=10 으로 세팅해도 잘 동작하는 것 확인함.
			assert(lw)
			for i=0,cdim-1 do
				qp:addD(lw,i+numActualDOF*2,0)
			end
		end


		-- set inequality constraints
		do
			-- use [de Lasa et al, SIGGRAPH2010]
			-- a_c=V'J ddq + V'dotJ dq + dotV' J dq >= 0

			local link_pair_count=w.link_pair_count
			--assert(w.J:rows()==link_pair_count*6)
			--assert(w.J:cols()==numActualDOF)
			--assert(w.V:rows()==link_pair_count*6)
			--assert(w.V:cols()==cdim)


			local numConTau=numActualDOF-6
			local clampTorque=true
			local maxTorque= QPparam.maxTorque or 400
			if clampTorque==false then
				maxTorque=160 --  actually maxAcceleration (clampAcceleration)
			end

			w:clearCI(w.VtJ:rows()+cdim+numConTau*2+1, totalDIM)
			-- w.bases(i).ilinkpair has those values for each w.bases(i)
			-- print(w.bases(1).normal:dotProduct(vector3(1,1,1)))

			local dq=w.dq
			if dq==nil then w.dq=vectorn(self.dtheta:size()-1) dq=w.dq end
			self:_calcDQ(dq)

			-- VtJ*ddq +VtDotJ*dq+dotVtJ*dq == contact_accelerations 
			-- contact_accelerations+prevVelocity >= 0
			
			assert(w.CI:rows()==0)
			if QPparam.useVelocityCone then
				w:addCI(w.VtJ, 0) -- ddq (contact acceleration constraints)
				w:addCI0((w.VtDotJ*dq:column()):column(0)+(w.dotVtJ*dq:column()):column(0), 0)
			end
			w:addCI(CT.eye(cdim), numActualDOF*2) --lambdas (friction cone coefs). These constrain contact forces.
			w:addCI0(CT.zeros(cdim))

			--_checkpoints:pushBack(deepCopyTable({'w', w.VtJ,w.dotVtJ}))
			--_checkpoints:pushBack(deepCopyTable({'dq',dq, w.CI, w.ci0}))

			if numConTau>0 then
				local startc=numActualDOF
				if not clampTorque then startc=0 end
				-- -tau + maxTorque>0
				w:addCI(CT.eye(numConTau)*-1, startc+6)
				local www=self.weight2:range(7,numActualDOF+1)
				w:addCI0(maxTorque*www)
				-- tau+maxTorque>0
				w:addCI(CT.eye(numConTau), startc+6)
				w:addCI0(maxTorque*www)
			end

			if QPparam.useVelocityCone then
				-- cmargin dependent
				local cmargin=0.005
				local velMarginStrength=QPparam.velMarginStrength or 1.0
				local maxPenetratingVel=QPparam.maxPenetratingVel or 0
				local dtinv=self.dtinv
				-- add velocity-dependent margin
				for i=0, w.bases:size()-1 do
					local b=w.bases(i)
					local dp=b.normal:dotProduct(b.relvel)

					local cmargin_dep_max_penetrating_vel
					if b.depth>cmargin then
						--cmargin_dep_max_penetrating_vel =maxPenetratingVel 
						cmargin_dep_max_penetrating_vel =maxPenetratingVel +sop.clampMap(b.depth, cmargin, cmargin*1.5, cmargin*dtinv, 0)
					else
						--cmargin_dep_max_penetrating_vel =maxPenetratingVel + sop.clampMap(b.depth ,0, cmargin, cmargin*dtinv, 0) 
						cmargin_dep_max_penetrating_vel =maxPenetratingVel + cmargin*dtinv 
					end
					local relvel=b.relvel:copy()+cmargin_dep_max_penetrating_vel*b.normal
					local GI=b.globalIndex
					for j=0, b.normals:size()-1 do
						local projectionOfRelvel=b.normals(j):dotProduct(relvel)*dtinv*velMarginStrength -- do not allow foot slipping
						local gi=GI+j
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel)
					end
				end
			end

		end
		-- set equality constraints  
		--
		-- w.M*ddq - tau - JtV* lambda = w.b
		--> (w.M -I -JtV)(ddq;tau;lambda) = w.b
		--
		local reftime=self.state.reftime
		if QPparam.allowSmallRootTorque then
			-- without root force
			w.CE:setSize(numActualDOF+3, totalDIM)
			w.CE:sub(0,numActualDOF,0,numActualDOF):assign(w.M)
			local minusI=w.CE:sub(0,numActualDOF,numActualDOF,numActualDOF*2)
			minusI:identity()
			minusI:rmult(-1)
			local minusJtV=w.CE:sub(0,numActualDOF, numActualDOF*2, totalDIM)
			minusJtV:assign(w.JtV)
			minusJtV:rmult(-1)
			-- constrain tau[3:6]=0
			w.CE:sub(numActualDOF, numActualDOF+3):setAllValue(0)
			w.CE:sub(numActualDOF, numActualDOF+3, numActualDOF+3, numActualDOF+6):identity()
			w.ce0:setSize(numActualDOF+3)
			w.ce0:range(0,numActualDOF):assign(w.b)
			w.ce0:range(numActualDOF,numActualDOF+3):setAllValue(0)

			-- constrain  tau[0:3]+thr>= 0
			local thr=QPparam.allowSmallRootTorque
			w:addCI(CT.eye(3), numActualDOF)
			w:addCI0(CT.ones(3)*thr)

			-- constrain  -tau[0:3]+thr>= 0
			-- --> tau <= thr
			w:addCI(CT.eye(3)*-1, numActualDOF)
			w:addCI0(CT.ones(3)*thr)
		else
			-- without root force/torque
			w.CE:setSize(numActualDOF+6, totalDIM)
			w.CE:sub(0,numActualDOF,0,numActualDOF):assign(w.M)
			local minusI=w.CE:sub(0,numActualDOF,numActualDOF,numActualDOF*2)
			minusI:identity()
			minusI:rmult(-1)
			local minusJtV=w.CE:sub(0,numActualDOF, numActualDOF*2, totalDIM)
			minusJtV:assign(w.JtV)
			minusJtV:rmult(-1)
			-- constrain tau[0:6]=0
			w.CE:sub(numActualDOF, numActualDOF+6):setAllValue(0)
			w.CE:sub(numActualDOF, numActualDOF+6, numActualDOF, numActualDOF+6):identity()
			w.ce0:setSize(numActualDOF+6)
			w.ce0:range(0,numActualDOF):assign(w.b)
			w.ce0:range(numActualDOF,numActualDOF+6):setAllValue(0)
		end
		if #w.CE_additional >0 then
			local ncon=#w.CE_additional
			assert(ncon==1)
			local con=w.CE_additional[1]
			local prevRow=w.CE:rows()
			w.CE:resize(prevRow+con[1]:rows(), w.CE:cols())
			w.ce0:resize(prevRow+con[1]:rows())
			w.CE:sub(prevRow,w.CE:rows(),0,con[1]:cols()):assign(con[1])
			w.ce0:range(prevRow,w.CE:rows()):assign(con[2])
		end
		--]]

		--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({qp.H:copy(), qp.R:copy()}) --##dos end
		-- print(self.controlforce)
		-- self:addPDtorque(simulator)
		-- dbg.console()
		
	else
		local totalDIM=numActualDOF*2 -- ddq and tau
		local qp=HessianQuadratic(totalDIM)
		self.qp=qp
		-- dbg.console()
		-- minimize desired acc error
		local ddqObjWeight_flight=QPparam.ddqObjWeight_flight or 1000
		local weight=self.weight
		if self.excludeRootFlight then --self.excludeRoot then
			local w=1
			for i=0,2 do -- root
				qp:addD(w,i,self.desiredacceleration(i+4))
			end
			for i=3,5 do -- root
				qp:addD(w,i,self.desiredacceleration(i-3))
			end
		else
			--self.desiredacceleration:setVec3(4, vector3(0, -9.8,0))
			for i=0,2 do -- root
				qp:addD(10000,i,self.desiredacceleration(i+4))
			end
			for i=3,5 do -- root
				qp:addD(10000,i,self.desiredacceleration(i-3))
			end
		end

		local weight=self.weight
		for i=6,numActualDOF-1 do
			qp:addD(ddqObjWeight_flight*weight(i+1),i,self.desiredacceleration(i+1))
		end
		for i=0,numActualDOF-1 do
			qp:addD(1,i+numActualDOF,0)
		end
		w.CE:setSize(numActualDOF+6, numActualDOF*2)
		w.CE:sub(0,numActualDOF,0,numActualDOF):assign(w.M)
		local minusI=w.CE:sub(0,numActualDOF,numActualDOF,numActualDOF*2)
		minusI:identity()
		minusI:rmult(-1)
		-- constrain tau[0:6]=0
		w.CE:sub(numActualDOF, numActualDOF+6):setAllValue(0)
		w.CE:sub(numActualDOF, numActualDOF+6, numActualDOF, numActualDOF+6):identity()
		w.ce0:setSize(numActualDOF+6)
		w.ce0:range(0,numActualDOF):assign(w.b)
		w.ce0:range(numActualDOF,numActualDOF+6):setAllValue(0)
		w.CI:setSize(0,0)
		w.ci0:setSize(0)
	end

end

function QPservo:rewindTargetMotion(sim)
	local simulator=self.simulator
	self.deltaTime=-1*simulator:currentTime()
end

QPsim=LUAclass()

function QPsim.registerContactPairAll(model, loader, floor, simulator)
	param=vectorn ()
	param:setValues(0.5,0.5, 30000, 3000)
	for i=1,loader:numBone()-1 do

		local bone_i=loader:VRMLbone(i)
		simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param)
	end
end
function QPsim:setReferenceMotion(info)
	self.motionDOF=info.CDMmot
	self.DMotionDOF=info.CDM_DMotionDOF
	self.DDMotionDOF=info.CDM_DDMotionDOF

	if self.qpservo then
		self.qpservo.motionDOF=self.motionDOF
		self.qpservo.dmotionDOF=self.DMotionDOF
		self.qpservo.ddmotionDOF=self.DDMotionDOF
	end
end
function QPsim:setNoReferenceMotion()
	self.motionDOF=nil
	self.DMotionDOF=nil
	self.DDMotionDOF=nil

	if self.qpservo then
		self.qpservo.motionDOF=nil
		self.qpservo.dmotionDOF=nil
		self.qpservo.ddmotionDOF=nil
	end
end
function QPsim:__init(loader, info, simulatorParam)
	self.drawOffset=vector3(0,0,0)
	simulatorParam=simulatorParam or self.createSimulatorParam()
	if simulatorParam.drawSkeleton==nil then simulatorParam.drawSkeleton = true end
	self.skin=RE.createVRMLskin(loader, simulatorParam.drawSkeleton)
	self.skin:setThickness(0.03)
	self.skin:scale(100,100,100)

	if useGMBSsimulator then
		--self.simulator= Physics.DynamicsSimulator_gmbs()
		self.simulator= Physics.DynamicsSimulator_Trbdl_penalty('libccd')
	else
		assert(false)
		self.simulator= Physics.DynamicsSimulator_TRL_QP("libccd")
	end
	self.simulator:registerCharacter(loader)
	if QPparam.collisionCheck then
		local floor=mFloor or VRMLloader("../Resource/mesh/floor_y.wrl")
		self.simulator:registerCharacter(floor)
		QPsim.registerContactPairAll(model, loader, floor, self.simulator)   
	end

	if self.simulator._enableDotJocobianComputation then
		self.simulator:_enableDotJocobianComputation(0)
	end

	self.simulator:init(simulatorParam.timestep, Physics.DynamicsSimulator.EULER)

	local debugContactParam={10, 0, 0.01, 0, 0}-- size, tx, ty, tz, tfront
	self.simulator:setSimulatorParam("debugContact", debugContactParam)
	self.simulator:setSimulatorParam("contactForceVis", {0.001,0.001,0.001})
	self.simulator:setSimulatorParam("debugContact", debugContactParam)
	-- adjust initial positions

	if info and info.motionDOF then
		self:setReferenceMotion(info)
	else
		self:setNoReferenceMotion()
	end
	self.simulationParam=simulatorParam

	self.controlforce=vectorn(loader.dofInfo:numDOF())
	do

		self.qpservo=QPservo(loader.dofInfo,simulatorParam.timestep, nil, self.simulator)
		if info and info.motionDOF then
			self.motionDOF=info.motionDOF
			self.DMotionDOF=info.DMotionDOF
			self.DDMotionDOF=info.DDMotionDOF
			self.qpservo:initQPservo(model.start, self.motionDOF:numFrames(),
			self.motionDOF, self.DMotionDOF, self.DDMotionDOF)
		end


		self.qpservo.drawDebugInformation=simulatorParam.drawDebugInformation
	end

	local motdof=self.motionDOF
	if motdof then
		model.start=math.min(model.start, self.motionDOF:numFrames()-1)
		initialState=vectorn()
		initialState:assign(self.motionDOF:row(model.start))
		-- set global position
		-- initialState:set(0,0)
		-- initialState:set(1,initialState:get(1)+(initialHeight or 0) )
		-- initialState:set(2,0)

		print("initialState=",initialState)
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)

		if self.DMotionDOF then
			local initialVel=self.DMotionDOF:row(model.start):copy()
			self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, initialVel)
		end
		self.simulator:initSimulation()
	else
	end
	--	debug.debug()
	Physics.DynamicsSimulator.setPose(self.skin,self.simulator,0)

	self.skin:setMaterial("lightgrey_transparent")

	--self.simulator.setGVector(vector3(0,0,9.8))
	self.simulator:setGVector(vector3(0,9.8,0))
	self.simulator:initSimulation()
	self.loader=loader
	self.floor=floor -- have to be a member to prevent garbage collection

	self.refCoord=transf()
	self.refCoord:identity()
	self.refCoord.rotation:assign(quater(math.rad(0), vector3(0,1,0)))
	--self.refCoord.translation.y=-0.3
end
function QPsim:setFrame(iframe)
	self.qpservo.startFrame=iframe
	self.qpservo:rewindTargetMotion(self.simulator)
	local initialState=vectorn()
	initialState:assign(self.motionDOF:row(iframe))

	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)

	if self.DMotionDOF then
		local initialVel=self.DMotionDOF:row(iframe):copy()
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, initialVel)
	end
	self.simulator:initSimulation()
end
function QPsim:__finalize()
	-- remove objects that are owned by C++
	if self.skin~=nill then
		RE.remove(self.skin)
		self.skin=nil
	end
	self.simulator=nil

end
timer=util.Timer()
function QPsim:frameMove_simple(niter, contactpos, iframe, action, daction, impulseInfo)
	local cf
	for iter=1,niter do
		do
			local maxForce=9.8*80
			local qpservo=self.qpservo
			if debug_mode then
				--print(qpservo.currFrame)
				print('b',qpservo.theta_d, qpservo.dtheta_d)
			end
			qpservo:sampleCurrPose()
			if iframe then
				if iframe>=0 then
					qpservo.currFrame=iframe
					qpservo:sampleTargetPoses(qpservo.currFrame)
				end
			else
				qpservo:_updateCurrFrame()
				qpservo:sampleTargetPoses(qpservo.currFrame)
			end
			if action then
				qpservo.theta_d:setVec3(0, qpservo.theta_d:toVector3(0)+action:toVector3(0)) --desired body pos
				local dq=quater() dq:setRotation(action:toVector3(3))
				qpservo.theta_d:setQuater(3, qpservo.theta_d:toQuater(3)*dq) -- ori
			end
			if daction then
				qpservo.dtheta_d:setVec3(0, qpservo.dtheta_d:toVector3(0)+daction:toVector3(0)) --desired body v
				qpservo.dtheta_d:setVec3(4, qpservo.dtheta_d:toVector3(4)+daction:toVector3(3)) --desired body w
			end
			qpservo:_calcDesiredAcceleration()
			if debug_mode then
				--print(qpservo.currFrame)
				print('',qpservo.theta_d, qpservo.dtheta_d)
				print(qpservo.theta, qpservo.dtheta)
				print('daction', action, daction)
				print('da', qpservo.desiredacceleration, qpservo.theta(1), model.maxContactY, QPparam.k_d_ID)
			end
			if qpservo.theta(1)>0 and qpservo.theta(1)<(model.maxContactY or 1.1) then
				qpservo:_buildProblem( maxForce, contactpos)
			else
				qpservo:_buildProblem( maxForce, vector3N())
			end
			local impulse
			if impulseInfo and impulseInfo.impulse>0 then
				RE.output2("impulse",impulseInfo.impulse)

				local gf=impulseInfo.impulseDir
				local chest=self.loader:VRMLbone(1)
				local frame= self.simulator:getWorldState(0):globalFrame(1)
				local lf=frame:toLocalDir(gf)

				local dir=gf:copy()
				dir:normalize()
				if impulseInfo.gizmo then
					impulseInfo.gizmo[3]=gf -- will be drawn in showHuman.lua
				else
					local pos=frame*chest:localCOM()
					if model.delayedVis then
						--dbg.delayedDraw('Arrow',model.delayedVis-1, pos*100-50*dir,pos*100,'impulseGizmo')
						dbg.delayedDraw('Arrow',model.delayedVis, pos*100-50*dir,pos*100,'impulseGizmo')
					else
						dbg.draw('Arrow',pos*100-50*dir,pos*100,'impulseGizmo')
					end
				end
				impulse={treeIndex=1, lf=lf, lpos=impulseInfo.impulse_lpos}
				impulseInfo.impulse=impulseInfo.impulse-1
			end
			cf=qpservo:stepSimul(self.drawDebugInformation, impulse)
		end
	end
	self.skin:setPose(self.simulator,0)				

	if cf and cf:size()>0 and (not draw_option or draw_option.contactforce) then 

		local out=vector3N()
		for i=0, contactpos:size()-1 do
			local contactPosL=contactpos(i)
			local contactForceL=cf(i)
			if contactForceL:length()>0 then
				out:pushBack(contactPosL+self.drawOffset)
				out:pushBack(contactPosL+contactForceL*0.002+self.drawOffset)
			end
		end
		if model.delayedVis then
			dbg.delayedDraw('Traj', model.delayedVis, out:matView()*100, "contacts") 
		else
			dbg.namedDraw('Traj', out:matView()*100, "contacts") 
		end
	else
		if model.delayedVis then
			dbg.delayedErase('Traj', model.delayedVis, 'contacts')
		else
			dbg.erase('Traj', 'contacts')
		end
	end

	return mat
end

-- 좌표계 독립적인 새 버젼
function QPsim:frameMove(niter, contactpos, iframe, action, daction, impulseInfo, interactionGenerator)
	-- todo : dtheta
	local qpservo=self.qpservo

	qpservo:sampleCurrPose()

	local refCoord=self.refCoord
	local invRefCoord=refCoord:inverse()

	local function transformTheta(tf, theta)
		MotionDOF.setRootTransformation(theta, tf*MotionDOF.rootTransformation(theta))
	end
	transformTheta(invRefCoord, qpservo.theta)
	qpservo:restoreSampledPose() -- reference 좌표계에서 QP풀기.

	if iframe then
		assert(iframe==-1)
		-- 이 경우 qpservo.theta_d는 외부에서 주어짐.
		transformTheta(invRefCoord, qpservo.theta_d)
	end

	local function mult(q, v)
		local out=vector3N(v:size())
		for i=0, v:size()-1 do
			out(i):assign(q*v(i))
		end
		return out
	end

	local cf
	for iter=1,niter do
		do
			local maxForce=9.8*80
			qpservo:sampleCurrPose()
			if iframe then
				assert(iframe==-1)
				-- 이 경우 qpservo.theta_d는 외부에서 주어짐.
			else
				qpservo:_updateCurrFrame()
				qpservo:sampleTargetPoses(qpservo.currFrame)
				transformTheta(invRefCoord, qpservo.theta_d)
			end
			if action then
				qpservo.theta_d:setVec3(0, qpservo.theta_d:toVector3(0)+action:toVector3(0)) --desired body pos
				local dq=quater() dq:setRotation(action:toVector3(3))
				qpservo.theta_d:setQuater(3, qpservo.theta_d:toQuater(3)*dq) -- ori
			end
			if daction then
				qpservo.dtheta_d:setVec3(0, qpservo.dtheta_d:toVector3(0)+daction:toVector3(0)) --desired body v
				qpservo.dtheta_d:setVec3(4, qpservo.dtheta_d:toVector3(4)+daction:toVector3(3)) --desired body w
			end
			--print(qpservo.theta_d, qpservo.dtheta_d)
			--print('->',qpservo.theta, qpservo.dtheta)
			qpservo:_calcDesiredAcceleration()
			if qpservo.theta(1)>0 and qpservo.theta(1)<(model.maxContactY or 1.1) then
				qpservo:_buildProblem( maxForce, mult(invRefCoord,contactpos))
				--print(mult(invRefCoord,contactpos))
			else
				qpservo:_buildProblem( maxForce, vector3N())
			end
			local impulse
			if impulseInfo and impulseInfo.impulse>0 then
				RE.output2("impulse",impulseInfo.impulse)

				local gf=invRefCoord:toLocalDir(impulseInfo.impulseDir)
				local chest=self.loader:VRMLbone(1)
				local frame= self.simulator:getWorldState(0):globalFrame(1)
				local lf=(refCoord*frame):toLocalDir(gf)

				local dir=gf:copy()
				dir:normalize()
				if impulseInfo.gizmo then
					--local t=transf()
					--t:axisToAxis(vector3(0,0,0), vector3(0,1,0), refCoord*(frame.translation)*100-10*dir, dir)
					--RE.output2("gizmo", t.rotation,t.translation)
					--impulseInfo.impulseGizmo:transform(t)
					impulseInfo.gizmo[3]=gf
					--dbg.namedDraw('Sphere', t.translation*100, 'impulse',"red", 5)
				else
					local pos=refCoord*(frame*chest:localCOM())
					if model.delayedVis then
						dbg.delayedDraw('Arrow',model.delayedVis, pos*100-50*dir,pos*100-0*dir,'impulseGizmo')
					else
						dbg.draw('Arrow',pos*100-50*dir,pos*100-0*dir,'impulseGizmo')
					end
				end
				impulse={treeIndex=1, lf=lf, lpos=impulseInfo.impulse_lpos}
				impulseInfo.impulse=impulseInfo.impulse-1
			end
			if interactionGenerator then
				impulse=interactionGenerator:step( refCoord*self.simulator:getWorldState(0):globalFrame(1), qpservo.dtheta, impulse )
			end
			cf=qpservo:stepSimul(self.drawDebugInformation, impulse)
		end
	end
	qpservo:sampleCurrPose()
	transformTheta(refCoord, qpservo.theta)
	qpservo:restoreSampledPose() --항상 월드 좌표계에서 시뮬레이터 상태, 목표 상태 저장.
	transformTheta(refCoord, qpservo.theta_d)

	self.skin:setPoseDOF(qpservo.theta)
	local isTraining=not hasGUI

	if CDMdrawOffset then
		self.drawOffset=CDMdrawOffset
	end
	if false then
		self.drawOffset=self.drawOffset:copy()
		local rotY=qpservo.theta:toQuater(3):rotationY()
		self.drawOffset:radd(rotY*draw_option.draw_offset)
	end

	if cf and cf:size()>0 and not isTraining and (not draw_option or draw_option.contactforce) then 
		local out=vector3N()
		if math.fmod(contactpos:size(),2)==0 then
			-- merge two forces into one
			for i=0, contactpos:size()-1,2 do
				local contactPosL=contactpos(i)*0.5+contactpos(i+1)*0.5
				local contactForceL=cf(i)*0.5+ cf(i+1)*0.5
				if contactForceL:length()>0 then
					out:pushBack(contactPosL+self.drawOffset)
					out:pushBack(contactPosL+contactForceL*0.002+self.drawOffset)
				end
			end
		else
			for i=0, contactpos:size()-1 do
				local contactPosL=contactpos(i)
				local contactForceL=refCoord.rotation*cf(i)
				if contactForceL:length()>0 then
					out:pushBack(contactPosL+self.drawOffset)
					out:pushBack(contactPosL+contactForceL*0.002+self.drawOffset)
				end
			end
		end
		if model.delayedVis then
			dbg.delayedDraw('Traj', model.delayedVis, out:matView()*100, "contacts") 
		else
			dbg.namedDraw('Traj', out:matView()*100, "contacts") 
		end
	else
		if model.delayedVis then
			dbg.delayedErase('Traj', model.delayedVis, 'contacts')
		else
			dbg.erase('Traj', 'contacts')
		end
	end
end



if Physics.DynamicsSimulator_gmbs then
function Physics.DynamicsSimulator_gmbs:getDQ(ichar)
	local q=self:getWorldState(0):globalFrame(1).rotation
	local dpose=vectorn()
	self:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dpose)
	return MotionDOF.dposeToDQ(q, dpose)
end
end
