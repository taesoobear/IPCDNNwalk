   markerFile="NONE"
   if false then
   Start(
   "../Resource/jae/human/humanHanyang_T.wrl", -- 종아리 짧음
  --"../Resource/jae/human/humanHanyang_T_male.mesh.wrl",
    "../Resource/jae/human/humanHanyang_T_humanHanyang.dof",
   --"../Resource/jae/human/humanHanyang_T.dof",
   0.0
   )

   else
   Start(
   --"../Resource/jae/human/humanHanyang_T.wrl", -- 종아리 짧음
   "../Resource/motion/humanHanyang/humanHanyang_T.wrl", -- 정상 비율
  --"../Resource/jae/human/humanHanyang_T_male.mesh.wrl",
   "../Resource/motion/humanHanyang/humanHanyang_T.dof",
   --"../Resource/jae/human/humanHanyang_T_locomotion_hl_upperbody.dof",
   0.0
   )
   end
