mot=RE.motionPanel():currMotion()
kernelTrans=0.2
kernelRot=0.1
mot:smooth(kernelTrans, kernelRot)
