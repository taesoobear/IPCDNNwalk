a=Motion(1,39488,39531)
b=Motion(1,52177,52271)
e=stitch(a,b, "stitch", 40,true, false, true, true)
e:show()
f=stitch(a,b, "concat")
f:show("green")


test("PositionStitch", 53536, 53601, 25908, 25960)