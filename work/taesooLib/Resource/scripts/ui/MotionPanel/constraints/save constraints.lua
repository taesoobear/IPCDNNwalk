

mp=RE.motionPanel()
mot=mp:currMotion()

leftFoot=boolN(mot:numFrames())
rightFoot=boolN(mot:numFrames())

for i=0,mot:numFrames()-1 do
   leftFoot:set(i, mot:isConstraint(i, FootstepDetection.CONSTRAINT_LEFT_FOOT))
   rightFoot:set(i, mot:isConstraint(i, FootstepDetection.CONSTRAINT_RIGHT_FOOT))
end

mp:scrollPanel():addPanel(leftFoot:bit(), CPixelRGB8(255,255,0))
mp:scrollPanel():addPanel(rightFoot:bit(), CPixelRGB8(255,0,255))

fn=Fltk.chooseFile("Choose con", "../Resource/motion/", "*.con", true)

file=util.BinaryFile()
file:openWrite(fn)
file:pack(leftFoot)
file:pack(rightFoot)
file:close()

