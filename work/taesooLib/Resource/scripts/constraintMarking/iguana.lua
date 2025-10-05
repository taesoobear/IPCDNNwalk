mot=RE.motionPanel():currMotion()
bCleanup=false
bFillGap=true
minConDuration=1
reduceCon=4
footstepDetector=FootstepDetection(mot, bCleanup, bFillGap, minConDuration, reduceCon)

RE.motionPanel():scrollPanel():removeAllPanel()

-- left toe
bone=mot:skeleton():getBoneByVoca(MotionLoader.LEFTANKLE):childHead()
height_thr=5
speed_thr=1
footstepDetector:calcConstraint(bone, FootstepDetection.CONSTRAINT_LEFT_TOE, height_thr, speed_thr, "left_toe.bmp") 
RE.motionPanel():scrollPanel():addPanel("left_toe.bmp")
RE.motionPanel():drawCon(FootstepDetection.CONSTRAINT_LEFT_TOE)

-- right toe
bone=mot:skeleton():getBoneByVoca(MotionLoader.RIGHTANKLE):childHead()
height_thr=5
speed_thr=1
footstepDetector:calcConstraint(bone, FootstepDetection.CONSTRAINT_RIGHT_TOE, height_thr, speed_thr, "right_toe.bmp") 
RE.motionPanel():scrollPanel():addPanel("right_toe.bmp")
RE.motionPanel():drawCon(FootstepDetection.CONSTRAINT_RIGHT_TOE)


-- left wrist
bone=mot:skeleton():getBoneByVoca(MotionLoader.LEFTWRIST):childHead()
height_thr=6
speed_thr=1
footstepDetector:calcConstraint(bone, FootstepDetection.CONSTRAINT_LEFT_FINGERTIP, height_thr, speed_thr, "left_FINGERTIP.bmp") 
RE.motionPanel():scrollPanel():addPanel("left_FINGERTIP.bmp")
RE.motionPanel():drawCon(FootstepDetection.CONSTRAINT_LEFT_FINGERTIP)

-- right wrist
bone=mot:skeleton():getBoneByVoca(MotionLoader.RIGHTWRIST):childHead()
height_thr=6
speed_thr=1
footstepDetector:calcConstraint(bone, FootstepDetection.CONSTRAINT_RIGHT_FINGERTIP, height_thr, speed_thr, "right_FINGERTIP.bmp") 
RE.motionPanel():scrollPanel():addPanel("right_FINGERTIP.bmp")
RE.motionPanel():drawCon(FootstepDetection.CONSTRAINT_RIGHT_FINGERTIP)