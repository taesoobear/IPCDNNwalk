
--class 'ConvexHull2D'
ConvexHull2D=LUAclass()
function ConvexHull2D:__init()
	self.grahamScan=math.GrahamScan()
	self.N=0
end

function ConvexHull2D:_addTestPoints()
	self:addPoint(1,0)
--	self:addPoint(0,1)
--	self:addPoint(1,1)
	self:addPoint(0,0.5)
	self:addPoint(1,0.5)
	self:addPoint(0.5,1.2)
	self:addPoint(0,0)
end

function ConvexHull2D:debugDraw()
	local mat=matrixn(self.supportPlanes:size(), 3)
	for i, sp in ipairs(self.supportPlanes) do
		local a=self.supportVertex[i]
		mat:set(i-1,0, a.x*100)
		mat:set(i-1,1, 0.02*100)
		mat:set(i-1,2, a.y*100)
	end

	dbg.draw('Quads', "convexHull", mat)
end
function ConvexHull2D:test()
	print('wait for processing...')
	self:_addTestPoints()
	self:buildHull()

	local img=CImage()
	local RES=512
	img:create(RES, RES);
	local dist=matrixn(RES,RES)
	img:drawBox(TRect(0,0,RES,RES), 255,255,255)

	local centroid=math.Point2D(0,0)
	local count=0
	for i=0,RES-1 do
		for j=0, RES-1 do
			local pp=math.Point2D(sop.map(i, 0, RES-1, -0.5, 1.5), sop.map(j, 0, RES-1, -0.5, 1.5))
			if self:project(pp.x, pp.y, 0.3)~=pp then
				img:setPixel(i,j,0,0,0)
			else
				centroid=centroid+math.Point2D(i,j)
				count=count+1
			end
			local d=self:distance(pp.x, pp.y)
			dist:set(i,j,d)
		end
	end
	local COM=self:calcCentroid()
	local comix=sop.map(COM.x, -0.5, 1.5, 0, RES-1)
	local comiy=sop.map(COM.y, -0.5, 1.5, 0, RES-1)
	img:setPixel(comix, comiy, 255,0,0, 5)
	img:setPixel(comix, comiy, 0,0,0, 3)
	img:setPixel(math.round(centroid.x/count), math.round(centroid.y/count), 0,255,255,3)
	print('see work/test.jpg')
	local range={dist:minimum(), dist:maximum()}
	local imgDist=CImage()
	imgDist:create(RES, RES);
	for i=0,RES-1 do
		for j=0, RES-1 do
			local grayL=sop.map(dist(i,j), range[1], range[2], 0, 255)
			imgDist:setPixel(i,j,grayL, grayL, grayL )
		end
	end
	img:Save('test.jpg')
	imgDist:Save('testDist.jpg')
end

function ConvexHull2D:addVector3(v)
	self:addPoint(v.x, v.z)
end

function ConvexHull2D:addPoint(x,y)
	if self.N==0 then
		self.firstPoint=math.Point2D(x,y)
	end
	self.grahamScan:add_point(math.Point2D(x,y))
	self.N=self.N+1
end

--       | planeL  | normal  | planeR
--       b <---------------- a
function ConvexHull2D:createDiscriminatingPlane()
	self.planes={}
	for i, sp in ipairs(self.supportPlanes) do
		local a=self.supportVertex[i]
		local b=self.supportVertex[i+1]
		local normal=sp.normal

		local planeR=math.Plane2D:new(a, a+normal)
		local planeL=math.Plane2D:new(b, b-normal)
		self.planes[i]={planeR, planeL,0,0,0}
	end
	self.planes[table.getn(self.planes)+1]=self.planes[1] -- make cyclic
end

function ConvexHull2D:calcCentroid()
	if self.N==1 then
		local point = self.firstPoint
		return point, 0
	elseif self.N==2 then

		local sum=math.Point2D(0,0)

		for i=1, self.supportPlanes:size() do
			sum=sum+self.supportVertex[i]
		end
		return sum*(1/self.supportPlanes:size()), 0
	else 
		local sum=math.Point2D(0,0)
		local refVertex=self.supportVertex[1]
		local summedArea=0
		for i=2, self.supportPlanes:size()-1 do
			local a=self.supportVertex[i]
			local b=self.supportVertex[i+1]

			local area=(a-refVertex):cross(b-refVertex)*0.5
			summedArea=area+summedArea
			sum=sum+((a+b+refVertex)*area*(1/3))
		end
		return sum*(1/summedArea), summedArea
	end
end
if true then
	-- use approximate circle to calculate area
	ConvexHull2D.calcCentroid_old=ConvexHull2D.calcCentroid
	function ConvexHull2D:calcCentroid()
		local center, area=self:calcCentroid_old()
		local MIN_AREA=3.14*0.02*0.02  -- square_cm
		if self.N==1 then
			return center, MIN_AREA
		else
			local R=0
			for i=1, self.supportPlanes:size() do
				R=R+center:distance(self.supportVertex[i])
			end
			R=R/self.N
			local area=3.14*R*R
			return center, math.max(area, MIN_AREA)
		end
	end
end
function ConvexHull2D:buildHull()
	if self.N>1 then
		self.grahamScan:partition_points()
		self.grahamScan:build_hull()
		local out=matrixn()
		self.grahamScan:get_hull(out)
		self.hull=out
		self.supportVertex={}
		self.supportPlanes=array:new()
			
		if self.N>2 then
			for i=0, out:rows()-1 do
				self.supportVertex[i+1]=math.Point2D(out(i,0), out(i,1))
			end
		
		else
			local p1=math.Point2D(out(0,0), out(0,1))
			local p2=math.Point2D(out(1,0), out(1,1))
			
			--                   | normal
			--         1--------------------2
			--
			local nor=(p2-p1):normalize():rotate90()*0.0001

			self.supportVertex[1]=p1-nor
			self.supportVertex[2]=p2-nor
			self.supportVertex[3]=p2+nor
			self.supportVertex[4]=p1+nor
			self.supportVertex[5]=p1-nor
		end
		local sv=self.supportVertex
		for i=1, table.getn(sv)-1 do
			self.supportPlanes:pushBack(math.Plane2D:new(sv[i+1], sv[i]))
		end
		self:createDiscriminatingPlane()
	end
end

function ConvexHull2D:distanceVec3(p)
	return self:distance(p.x, p.z)
end
-- negative: inside, positive : outside
function ConvexHull2D:distance(x,y)
	local pp=math.Point2D(x,y)
	if self.N==1 then
		self.supportVertex[1]=self.firstPoint
		return self.firstPoint:distance(pp), {'v', 1}
	end
	-- check edge support
	for i, sp in ipairs(self.supportPlanes) do
		local dist=sp:distance(pp)
		local dp=self.planes[i]
		dp[3]=dp[1]:distance(pp)
		dp[4]=dp[2]:distance(pp)
		
		if dp[3]>=0 and dp[4]>=0 and dist>=0 then
			-- support detected
			return dist, {'e', i}
		end
		dp[5]=dist
	end
	-- check inside
	local inside=true
	local max_dist=-1e10
	local arg_max
	for i=1, self.supportPlanes:size() do
		local dd=self.planes[i][5]
		if dd>max_dist then
			max_dist=dd
			arg_max=i
		end
	end

	if max_dist<=0.0001 then
		return max_dist, {'e', arg_max}
	end

	-- check vertex support
	for i=1, self.supportPlanes:size() do
		local dist1=self.planes[i][4]
		local dist2=self.planes[i+1][3]

		if dist1<=0 and dist2<=0 then
			return self.supportVertex[i+1]:distance(pp), {'v', i+1}
		end
	end
	assert(false)
	return 0
end
function ConvexHull2D:project(x,y, margin)
	
	local function clampV2(a, len)
		if a:length()>len then
			return a:normalize()*len
		end
		return a
	end
	if margin==nil then
		margin=0
	end
	local pp=math.Point2D(x,y)
	if self.N==1 then
		return self.firstPoint+clampV2(pp-self.firstPoint, margin)
	end

	-- check edge support
	for i, sp in ipairs(self.supportPlanes) do
		local dist=sp:distance(pp)
		local dp=self.planes[i]
		dp[3]=dp[1]:distance(pp)
		dp[4]=dp[2]:distance(pp)
		
		if dp[3]>=0 and dp[4]>=0 and dist>=0 then
			-- support detected
			return sp:project(pp,margin)
		end
		dp[5]=dist
	end
	-- check inside
	local inside=true
	for i=1, self.supportPlanes:size() do
		if self.planes[i][5]>0 then
			inside=false
		end
	end

	if inside then return pp:copy() end


	-- check vertex support
	for i=1, self.supportPlanes:size() do
		local dist1=self.planes[i][4]
		local dist2=self.planes[i+1][3]

		if dist1<=0 and dist2<=0 then
			return self.supportVertex[i+1]+clampV2(pp-self.supportVertex[i+1], margin)
		end
	end
	assert(false)
	return nil
end


function math.Point2D.__eq(a,b)
	return a.x==b.x and a.y==b.y
end
function math.Point2D.__sub(a,b)
	return math.Point2D(a.x-b.x, a.y-b.y)
end
function math.Point2D.__add(a,b)
	return math.Point2D(a.x+b.x, a.y+b.y)
end
function math.Point2D:__tostring()
	return '( '..self.x..","..self.y..')'
end
function math.Point2D.distance(a,b)
	return math.sqrt((a.x-b.x)^2+(a.y-b.y)^2)
end

function math.Point2D.cross(a,b)
	return a.x*b.y-a.y*b.x
end

function math.Point2D.copy(a)
	return math.Point2D(a.x, a.y)
end

function math.Point2D.__mul(a, b)
	return math.Point2D(a.x*b, a.y*b)
end
function math.Point2D:length()
	local x=self.x
	local y=self.y
	return math.sqrt(x*x+y*y)
end

function math.Point2D:normalize()
	local x=self.x
	local y=self.y
	local len=math.sqrt(x*x+y*y)
	if len==0 then return math.Point2D(0,0) end
	return math.Point2D(x/len, y/len)
end

function math.Point2D:dot(a)
	return self.x*a.x+self.y*a.y
end

function math.Point2D.rotate90(a)
	return math.Point2D(-a.y, a.x)
end

math.Plane2D=LUAclass()
-- two points: normal will face rotate90(b-a)
function math.Plane2D:__init(a,b)
	self.dir=b-a
	self.normal=self.dir:rotate90():normalize()
	self.d=-(self.normal:dot(a))
end

function math.Plane2D:distance(point)
	return self.normal:dot(point)+self.d
end

function math.Plane2D:project(point, margin)
	local dist=self:distance(point)-margin
	if dist>0 then
		return point+self.normal*(dist*-1)
	end
	return point:copy()
end

function math.projectTrajectoryToItsHull(tpos, option)
	local function buildHullZT(tpos)
		local out={}
		local zmpHull=ConvexHull2D()
		local sampling = tpos:size()-1
		for i=0, sampling  do
			zmpHull:addPoint(i, tpos(i).y)
		end
		zmpHull:buildHull()
		if zmpHull.supportPlanes:size() >  3 then
			local sv=zmpHull.supportVertex

			local function search_vtx(v2, tpos)
				for  i=0, tpos:size()-1 do
					if v2.x==i and v2.y==tpos(i).y then
						return i
					end
				end
				assert(false)
				return -1
			end

			local arrPi={}
			for j=1, #sv do -- support
				table.insert(arrPi, search_vtx(sv[j], tpos))
			end
			assert(#arrPi>=2)
			local results={spprt_vtx_idx=arrPi, vtx_orig=tpos}
			return results
		end
	end
	local function projectConvexHull(arr_tpos,  res)
		local spprt_vtx_idx=res.spprt_vtx_idx
		local vtx_orig=res.vtx_orig
		local err_thr=1e-3

		local function findLines(z) 
			local lines={}
			for i=1, #spprt_vtx_idx-1 do
				local s1=spprt_vtx_idx[i]
				local s2=spprt_vtx_idx[i+1]

				local z1=math.min(s1, s2)
				local z2=math.max(s1, s2)

				if z1-err_thr <z and z< z2+err_thr then
					table.insert(lines, i)
				end
			end
			return lines
		end
		local z_min=0
		local z_max=vtx_orig:size()-1 

		for i=0, arr_tpos:size()-1 do
			local z=i
			local l=findLines(z)
			if #l==0 then
				if z<z_min+err_thr then
					l=findLines(z_min)
				elseif z>z_max-err_thr then
					l=findLines(z_max)
				else
					assert(false)
				end
			end
			local y_max=-10000
			for ii, v in ipairs(l) do
				local s1=spprt_vtx_idx[v]
				local s2=spprt_vtx_idx[v+1]

				local z1=s1
				local z2=s2
				local y1=vtx_orig(s1).y
				local y2=vtx_orig(s2).y

				local y=sop.map(z, z1, z2, y1, y2)
				y_max=math.max(y_max, y)
			end

			assert(y_max~=-10000)
			arr_tpos(i).y=math.max(arr_tpos(i).y, y_max)
		end
	end
	option=option or {}
	local res=buildHullZT(tpos)
	if res then
		if option.drawHull then
			local function drawConvexHull( res, name, scalef)
				local indices=res.spprt_vtx_idx
				local tpos=res.vtx_orig
				local lines=matrixn((table.getn(indices)-1)*2, 3)
				for j = 2 ,table.getn(indices)  do
					lines:row((j-2)*2):setVec3(0, tpos(indices[j-1])*scalef)
					lines:row((j-2)*2+1):setVec3(0,tpos(indices[j])*scalef)
				end
				dbg.namedDraw('Traj',lines, 'currlineR'..name, 'solidred', 8, 'BillboardLineList')
			end
			drawConvexHull(res,  'test',1)
		end

		projectConvexHull(tpos, res)
		return true
	end
	return false
end
