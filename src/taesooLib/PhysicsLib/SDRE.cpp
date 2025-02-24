#include "physicsLib.h"
#include "clapack_wrap.h"
#include "../BaseLib/math/Operator_NR.h"

static int issquare(matrixn const& a)
{
	if( a.rows()==a.cols()) return a.rows();
	return 0;
}

static void clamp(matrixn& mat, double val)
{
	for( int i=0;i<mat.rows()-1;i++ ) {
		for(int j=0; j<mat.cols()-1;j++ ) {
			double v=mat(i,j);
			if(v>val ) {
				mat.set(i,j,val);
			}
			else if(v<val*-1 ) 
			{
				mat.set(i,j,val*-1);
			}
		}
	}
}

static void multAdiagB(matrixn& out, matrixn const& matA, vectorn const& diagB)
{
   assert(matA.rows()==diagB.size());

   out.assign(matA);
   for (int i=0;i<out.rows(); i++)
   {
      out.column(i)*=diagB(i);
   }
}

static bool issymmetric(matrixn const& a)
{
	for( int i=0; i<a.rows()-1 ;i++) {
		for( int j=i+1; j< a.cols(); j++ ) {
			if(a(i,j)!=a(j,i)) {
				return false;
			}
		}
	}
	return true;
}


static void zeros(matrixn& a, int r, int c)
{
	a.setSize(r,c);
	a.setAllValue(0);
}

static void div(matrixn& out, matrixn const& a, matrixn const& b)
{
	matrixn invB;
	m::LUinvert(invB,b);
	out.mult(a, invB);
}

static void leftDiv(matrixn& out, matrixn const& a, matrixn const& b)
{
	matrixn invA;
	m::LUinvert(invA,a);
	out.mult(invA,b);
}

static void raddT(matrixn & out, matrixn const& in)
{
	for (int i=0; i<out.rows(); i++)
	{
		double* ptr=out[i];
		for(int j=0; j<out.cols(); j++)
		{
			ptr[j]+=in[j][i];
		}
	}
}

void block22(matrixn & out, matrixn const& a, matrixn const& b, matrixn const& c, matrixn const& d)
{
	out.resize(a.rows()+c.rows(), b.cols()+d.cols());

	out.range(0, a.rows(), 0, a.cols()).assign(a);
	out.range(0, a.rows(), a.cols(), out.cols()).assign(b);
	out.range(a.rows(), out.rows(), 0, a.cols()).assign(c);
	out.range(a.rows(), out.rows(), a.cols(), out.cols()).assign(d);
}

static void are (matrixn & x, matrixn const& a, matrixn & b, matrixn & c)
{

	int n = issquare(a);
	int m = issquare(b);
	int p = issquare(c);
	if(n == 0) {
		Msg::error ("are. a is not square");
	}

	matrixn temp;
	if (m == 0) {
		temp.assign(b);
		b.multABt(temp, temp);
		m = b.rows();
	}

	if (p == 0) {
		temp.assign(c);
		c.multAtB(c,c);
		p = c.rows ();
	}

	if (n != m || n != p) { 
		Msg::error ("are. a, b, c not conformably dimensioned.");
	}

	matrixn maT;
	maT.transpose(a);
	maT*=-1;

	matrixn H;
	block22(H, a,b,c,maT);
	H.range(0, a.rows(), a.cols(), H.cols())*=-1;
	H.range(a.rows(), H.rows(), 0, a.cols())*=-1;
	matrixn u;
	CLapack::balancedSchurDecomposition(H, u);

	div(x, u.range (n,2*n, 0,n) , u.range (0,n,0,n));
}


void lqr (matrixn &k, matrixn const& a, matrixn const&b, matrixn &q, matrixn const&r)
{
#ifdef EXCLUDE_CLAPACK

   TString fn("work/_LQR_CACHE_", a.rows());
   BinaryFile f(false, (TString(RE::taesooLibPath().c_str())+fn+TString(".DAT")).ptr());
   Msg::verify(f.readable(), "failed to open: %s%s", RE::taesooLibPath().c_str(), (fn+TString(".DAT")).ptr());


   f.unpack( k);
   matrixn aa;
   f.unpack( aa);
   matrixn bb;
   f.unpack( bb); 
   f.unpack(q);
   matrixn rr;
   f.unpack(rr);
   f.close();

	return;
#endif

	matrixn p;

	// ## Check a.
	int n = issquare (a);
	if(n==0) {
		Msg::error ("lqr. requires 1st parameter(a) to be square");
	}

	// ## Check b.
	int n1=b.rows();
	int m= b.cols();

	if(n1 != n) {
		Msg::error ("lqr. a,b not conformal");
	}

	// ## Check q.
	n1 = issquare (q);
	if(n1 == 0 || n1 != n) {
		Msg::error ("lqr. q must be square and conformal with a");
	}

	// ## Check r.
	int m1=issquare(r);
	if(m1 == 0 || m1 != m) {
		Msg::error ("lqr. r must be square and conformal with column dimension of b, b.cols:%d r:%d",m,m1);
	}

	// ## Check if n is there.
	matrixn s(n,m);
	s.setAllValue(0);
      // ao = a;
      // qo = q;


   // ## Check that q, (r) are symmetric, positive (semi)definite

   // if((issymmetric (q) and issymmetric (r)
//       && all (eig (q) >= 0) && all (eig (r) > 0)) 
   if(true)
   {
//      p = are (ao, (b/r)*b.Transpose(), qo);

      // print(b)
      // print(r)
      // print((b/r)*b.Transpose())

      // p=octave.call("are",1,{ao, (b/r)*b.Transpose(), qo})[1];

      // print("p0",p)
	   matrixn bdivr;
	   div(bdivr, b,r);
	   matrixn temp;
	   temp.multABt(bdivr, b);
	   are(p, a, temp, q);
      // print("p1",p)
      // print("pp",b.Transpose()*p + s.Transpose())

	   temp.multAtB(b,p);
	   raddT(temp, s);

	   leftDiv(k, r, temp );

	   // print(k)
      // dbg.console()
	   //      e = eig (a - b*k);
   }
   else {
	   Msg::error ("lqr. q (r) must be symmetric positive (semi) definite");
   }
#if 0
   TString fn("_LQR_CACHE_", a.rows());
   BinaryFile f(true, (fn+TString(".DAT")).ptr());
   f.pack( k);
   f.pack( a);
   f.pack( b); 
   f.pack(q);
   f.pack(r);
   f.close();
#endif
}


