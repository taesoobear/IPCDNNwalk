a=Motion(1,53536,53601)
b=Motion(1,25908,25960)
e=stitch(a,b, "stitch", 50)
e:show()
f=stitch(a,b, "concat")
f:show("green")

test("PositionStitch", 53536, 53601, 25908, 25960)