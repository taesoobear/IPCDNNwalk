

mp=RE.motionPanel()
mot=mp:currMotion()

discontinuity=boolN()

fn=Fltk.chooseFile("Choose con", "../Resource/motion/", "*.dis", false)

file=util.BinaryFile()
file:openRead(fn)
file:unpack(discontinuity)
file:close()

for i=0,mot:numFrames()-1 do
   mot:setConstraint(i, FootstepDetection.IS_DISCONTINUOUS, discontinuity(i))
end

mp:scrollPanel():addPanel(discontinuity:bit(), CPixelRGB8(255,255,0))

