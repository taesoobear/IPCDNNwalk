local scilua={}

if false then
function scilua.quadprog(H,f)
	local x=H%(-f)
	return x
end
function LVecView:copy()
	return LVec(self)
end
function LVec:copy()
	return LVec(self)
end
function LMat:copy()
	return LMat(self)
end
function LMatView:copy()
	return LMat(self)
end

function LMat:__tostring()
	local out="{"
	local a=self
	local function tostr(v)
		local str=string.format("%.3g",v)
		local lstr=string.len(str)
		if lstr<10 then
			str=string.rep(" ", 9-lstr)..str
		end
		return str
	end
	local function printRow(i)
		out=out.."\n ["..i.."]={"
		if a:cols()<10 then
			for j=1, a:cols() do
				out=out..tostr(a(i,j))..","
			end
		else
			for j=1, 5 do
				out=out..tostr(a(i,j))..","
			end
			out=out..'..., '
			for j=a:cols()-2, a:cols() do
				out=out..tostr(a(i,j))..","
			end
		end
		out=out.."}"
	end
	if a:rows()<10 then
		for i=1,a:rows() do
			printRow(i)
		end
	else
		for i=1,5 do
			printRow(i)
		end
		out=out..'\n ...\n'
		for i=a:rows()-4, a:rows() do
			printRow(i)
		end
	end

	return out.."\n}\n"
end
LMatView.__tostring=LMat.__tostring

function scilua.ismatrix(vec)
	local typename=getmetatable(vec).luna_class
	return typename=='LMat'
end
function scilua.fromMatrixn(a)
	local ma=LMat(a:rows(), a:cols())
	for i=1,a:rows() do
		for j=1,a:cols() do
			ma:set(i,j,a(i-1,j-1))
		end
	end
	return ma
end
function scilua.toMatrixn(ma)
	local a=matrixn(ma:rows(), ma:cols())
	for i=1,a:rows() do
		for j=1,a:cols() do
			a:set(i-1,j-1,ma(i,j))
		end
	end
	return a
end

function scilua.trace(a)
	local sum=0
	for i=1,a:rows() do
		sum=sum+a(i,i)
	end
	return sum
end
function scilua.length(a)
	return a:cols()
end

function scilua.norm(a,P)
	if P=='inf' then
		assert(scilua.ismatrix(a))
		local maxRowSum=0
		for i=1,a:rows() do
			local rowSum=0
			for j=1,a:cols() do
				rowSum=rowSum+math.abs(a(i,j))
			end

			if rowSum>maxRowSum then
				maxRowSum=rowSum
			end
		end
		return maxRowSum
	else
		assert(false)
	end
end
-- local f, e=scilua.log2(a)
function scilua.log2(a)
	return test.math.frexp(a)
end

function scilua.expm(a)
	a=a:copy()
	if a:rows()~=a:cols() then
		error('expm requires a square matrix')
	end
	local n=a:rows()
	-- Trace reduction.
	local trshift = scilua.trace (a) / scilua.length (a);
	if (trshift > 0) then
		a :rsub( trshift*scilua.eye (n));
	end
	-- Balancing.
	local d, p, aa = scilua.balance (a);
	local f,e=scilua.log2(scilua.norm(a,'inf'))
	local s = math.max (0, e);
	s = math.min (s, 1023);
	aa:rmult( 2^(-s));

	-- Pade approximation for exp(A).
	local c = {
		5.0000000000000000e-1,
		1.1666666666666667e-1,
		1.6666666666666667e-2,
		1.6025641025641026e-3,
		1.0683760683760684e-4,
		4.8562548562548563e-6,
		1.3875013875013875e-7,
		1.9270852604185938e-9
	};
	local a2 = aa*aa;
	local id = scilua.eye (n);
	local x = (((c[8] * a2 + c[6] * id) * a2 + c[4] * id) * a2 + c[2] * id) * a2 + id;
	local y = (((c[7] * a2 + c[5] * id) * a2 + c[3] * id) * a2 + c[1] * id) * aa;

	local r = (x - y) % (x + y);

	-- Undo scaling by repeated squaring.
	for k = 1,s do
		r = r*r;
	end

	-- inverse balancing.
	d = scilua.diag (d);
	r = d * r / d;
	r:set(p, p, r);
	-- Inverse trace reduction.
	if (trshift >0) then
		r :rmult( math.exp (trshift));
	end
	
	return r
end

-- matlab_a=1:2:3 -> a=M.range(1,2,3)
function scilua.range(start, step, finish)
	-- mimicks matlab 0:T:fin syntax
	if finish==nil then finish=step step=1 end

	local c=start
	local out=LVec()
	while c<=finish do

		out:pushBack(c)
		c=c+step
	end
	return out
end

function scilua.diag(vec)
	if scilua.ismatrix(vec) then
		local m=vec
		assert(m:rows()==m:cols())
		local a=LVec(m:rows())
		m:getDiagonal(a)
		return a
	else
		local mat=LMat(vec:size(), vec:size())
		mat:setAll(0)
		mat:setDiagonal(vec)
		return mat
	end
end

function scilua.balance(a)
	local dd=LMat()
	local p=LVec()
	local s=LVec()
	local ac=a:copy()
	ac:balance(dd,p,s)
	return s, p, ac
end

-- inplace
function scilua.c2d(a,b,T)
	local n=a:rows()
	local m=b:cols()
	local mat=LMat(n+m,n+m)
	mat:sub(1,n,1,n):assign(a)
	mat:sub(1,n,n+1,n+m):assign(b)
	mat:sub(n+m,n+m,1,n+m):setAll(0)
	local matexp=scilua.expm(mat*T);
	a:assign(matexp:sub(1,n,1,n))
	b:assign(matexp:sub(1,n,n+1,n+m))
end

function scilua.block(nr, nc, ...)

	local tbl={...}

	local nrow=0
	local ncol=0

	for j=1,nc do
		ncol=ncol+tbl[j]:cols()
	end

	for i=1,nr do
		nrow=nrow+tbl[(i-1)*nc+1]:rows()
	end

	local out=LMat(nrow, ncol)

	crow=1
	local rsize
	for i=1,nr do
		ccol=1
		rsize=tbl[(i-1)*nc+1]:rows()
		for j=1,nc do
			local mat=tbl[(i-1)*nc+j]
			assert(mat:rows()==rsize)
			out:sub(crow, crow+rsize-1, ccol, ccol+mat:cols()-1):assign(mat)
			ccol=ccol+mat:cols()
		end
		assert(ccol==ncol+1)
		crow=crow+rsize
	end
	assert(crow==nrow+1)

   return out
end

function scilua.mat(n,m, ...)
   local mat=LMat(n,m)
   mat:setValues(...)
   return mat
end

function scilua.GEMM(a,opa,b,opb,alpha,beta)
	local mat=LMat()
	if beta then
		mat:mult(a,opa,b,opb,alpha,beta)
	else
		mat:mult(a,opa,b,opb,alpha)
	end
	return mat
end

function scilua.mult(a,opa,b,opb)
	local mat=LMat()
	if type(opa)~='string' then
		opb=b
		b=opa
		opa='n'
	end
	if type(opb)~='string' then
		opb='n'
	end
	mat:mult(a,opa,b,opb)
	return mat
end

function scilua.vec(...)
	local vec=LVec()
	if type(select(1,...))=="table" then 
		vec:setValues(unpack(select(1,...)))
	else
		vec:setValues(...)
	end
	return vec
end

function scilua.tblVec(vec)
   local tbl={}
   for i=1, vec:size() do
      tbl[i]=vec(i)
   end
   return tbl
end

function scilua.eye(n)
   local mat=LMat(n,n)
   mat:setAll(0)
   mat:setDiagonal(1)
   return mat
end

function scilua.rand(m,n)
	if n==nil then
		-- vectorn
		assert(false)-- not implemented yet
	else
		local mat=LMat(m,n)
		for i=1,m do
			for j=1,n do
				mat:set(i,j, math.random())
			end
		end
		return mat
	end
end

function scilua.inv(a)
	local mat=LMat()
	mat:invert(a)
	return mat
end

function scilua.skew(w)
	return scilua.mat(3,3, 
	0, -w.z, w.y,
	w.z, 0, -w.x,
	-w.y, w.x, 0)
end


function scilua.linspace(a,b,size)
   vec=LVec()
   vec:linspace(a,b,size)
   return vec
end

function scilua.zeros(n,m)
	if m~=nil then
		local mat=LMat(n,m)
		mat:setAll(0)
		return mat
	else
		local vec=LVec(n)
		vec:setAll(0)
		return vec
	end
end

function scilua.ones(n,m)
	if m~=nil then
		local mat=LMat(n,m)
		mat:setAll(1)
		return mat
	else
		local vec=LVec(n)
		vec:setAll(1)
		return vec
	end
end
return scilua
end
