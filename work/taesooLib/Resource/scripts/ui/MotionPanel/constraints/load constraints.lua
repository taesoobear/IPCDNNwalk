

mp=RE.motionPanel()
mot=mp:currMotion()

leftFoot=boolN()
rightFoot=boolN()

fn=Fltk.chooseFile("Choose con", "../Resource/motion/", "*.con", false)

file=util.BinaryFile()
file:openRead(fn)
file:unpack(leftFoot)
file:unpack(rightFoot)
file:close()

for i=0,mot:numFrames()-1 do
   mot:setConstraint(i, FootstepDetection.CONSTRAINT_LEFT_FOOT, leftFoot(i))
   mot:setConstraint(i, FootstepDetection.CONSTRAINT_RIGHT_FOOT,rightFoot(i))
end

mp:scrollPanel():addPanel(leftFoot:bit(), CPixelRGB8(255,255,0))
mp:scrollPanel():addPanel(rightFoot:bit(), CPixelRGB8(255,0,255))

