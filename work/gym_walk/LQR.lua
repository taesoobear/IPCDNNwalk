function issquare(a)
   if a:rows()==a:cols() then
      return a:rows()
   end
   return 0
end



function issymmetric(a)
   for i=0,a:rows()-1 do
      for j=i+1, a:cols()-1 do
	 if a(i,j)~=a(j,i) then
	    return false
	 end
      end
   end
   return true
end

function size(a)
   return a:rows(), a:cols()
end

function zeros(r,c)
   local a=matrixn(r,c)
   a:setAllValue(0)
   return a
end

function lqr (a, b, q, r, s)
   local k,p,e

   -- ## disp("lqr: entry");

   
   if (a ==nil or b==nil or q==nil) then
      error ("lqr: invalid number of arguments");
   end

   -- ## Check a.
   local n = issquare (a)
   if n==0 then
      error ("lqr: requires 1st parameter(a) to be square");
   end

   -- ## Check b.
   local n1,m=size(b)

   if (n1 ~= n) then
      error ("lqr: a,b not conformal");
   end

   -- ## Check q.
   n1 = issquare (q)
   if (n1 == 0 or n1 ~= n) then
      error ("lqr: q must be square and conformal with a");
   end

   -- ## Check r.
   local m1=issquare(r)
   if (m1 == 0 or m1 ~= m) then
      error ("lqr: r must be square and conformal with column dimension of b");
   end

   -- ## Check if n is there.
   if (s) then
      n1, m1 = size (s);
      if (n1 ~= n or m1 ~= m) then
	 error ("lqr: z must be identically dimensioned with b");
      end

      -- ## Incorporate cross term into a and q.
      ao = a - (b/r)*s:Transpose();
      qo = q - (s/r)*s:Transpose();
   else
      s = zeros (n, m);
      ao = a;
      qo = q;
   end


   -- ## Check that q, (r) are symmetric, positive (semi)definite

   -- if (issymmetric (q) and issymmetric (r)
--       && all (eig (q) >= 0) && all (eig (r) > 0)) 
   if(true
    )
   then
--      p = are (ao, (b/r)*b:Transpose(), qo);

      -- print(b)
      -- print(r)
      -- print((b/r)*b:Transpose())

      -- p=octave:call("are",1,{ao, (b/r)*b:Transpose(), qo})[1];

      -- print("p0",p)
      p=are(ao, (b/r)*b:Transpose(), qo)
      -- print("p1",p)
      -- print("pp",b:Transpose()*p + s:Transpose())
      k = r:LeftDiv(b:Transpose()*p + s:Transpose());

      -- print(k)
      -- dbg.console()
--      e = eig (a - b*k);
   else
      error ("lqr: q (r) must be symmetric positive (semi) definite");
   end

   return k,p,e
end


function are (a, b, c)
   local x

   local n = issquare(a)
   local m = issquare(b)
   local p = issquare(c)
   if (n == 0) then
      error ("are: a is not square");
   end

   if (m == 0) then
      b = b * b:Transpose();
      m = b:rows()
   end

   if (p == 0) then
      c = c:Transpose() * c;
      p = c:rows ();
   end

   if (n ~= m or n ~= p) then 
      error ("are: a, b, c not conformably dimensioned.");
   end

   local maT=a:Transpose()
   maT:rmult(-1)
   local H=CT.block(2,2, a,-b,-c,maT)
   local u=matrixn()
   balancedSchurDecomposition(H, u)

   x = u:range (n,2*n, 0,n) / u:range (0,n,0,n);

   return x
end




