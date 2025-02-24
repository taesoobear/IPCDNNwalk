#include "stdafx.h"
#include "mathclass.h"
#include "optimize.h"
Optimize::Method::Method()
{
}
Optimize::Method::~Method()
{
}
void Optimize::Method::_getCurPos(vectorn &opos)
{
	std::vector<Opt_dimension> &_opt_dimension=_optimizer->_opt_dimension;
	int N_opt_dimension=_opt_dimension.size();
	opos.setSize(N_opt_dimension);


	for( int i=0; i<N_opt_dimension; i++){
		opos[i]=_opt_dimension[i].curval;
	}
}

void Optimize::Method::_normalize(vectorn& pos, vectorn const& opos)
{
	std::vector<Opt_dimension> &_opt_dimension=_optimizer->_opt_dimension;
	for( int i=0; i<pos.size(); i++){
		pos[i]= opos(i)/_opt_dimension[i].max_step;
	}
}

void Optimize::Method::_unnormalize(vectorn& opos, vectorn const& pos)
{
	std::vector<Opt_dimension> &_opt_dimension=_optimizer->_opt_dimension;
	for (int i=0; i<pos.size(); i++){
		opos[i]=pos(i)*_opt_dimension[i].max_step;
	}
}


class Optimize_impl
{
public:
	
	Optimize::Method &_method;
	Optimize* _interface;
	Optimize_impl( Optimize::Method & method)
	:_method(method)
	{
	}
	void optimize()	
	{
		_method.prepareOptimization(*_interface);
		_method.optimize(_method.getInout());
		_method.finalizeOptimization();
	}
};

double Optimize::Method::func(const vectorn& x)
{
	if(!_optimizer->_useNormalization)
	{
		return _optimizer->objectiveFunction(x);
	}

	_unnormalize(_opos, x);
	double out=_optimizer->objectiveFunction(_opos);
   
	if (out==100000 )
		return 100000;
	
	return out/_oeval;
}


double Optimize::Method::func_dfunc(const vectorn& x,vectorn& dx)
{
	if(!_optimizer->_useNormalization)
	{
		return _optimizer->gradientFunction(x, dx);
	}

	_unnormalize(_opos, x);
	double e=_optimizer->gradientFunction(_opos, _gradient) ;
	dx.setSize(_gradient.size());
	for(int i=0; i<dx.size(); i++){
		dx[i]=_gradient(i)/_oeval;
	}
	

	if( e==100000 )
		return 100000;
  
	return e/_oeval;
}



void Optimize::Method::prepareOptimization(Optimize& objectiveFunction)
{
	_optimizer=&objectiveFunction;
	int N_opt_dimension=_optimizer->_opt_dimension.size();
	_x.resize(N_opt_dimension);


	if(_optimizer->_useNormalization)
	{
		_opos.resize(N_opt_dimension);
		_getCurPos(_opos);
		_oeval=_optimizer->objectiveFunction(_opos);
		if (_oeval<=0.0001 ) _oeval=0.0001;

#ifdef DEBUG_Optimize
		printf("oeval: %g\n", _oeval);
#endif
		_normalize(_x, _opos); // actual optimization is done in the normalized space.
	}
	else
	{
		_getCurPos(_x);
		_oeval=1.0;
	}
}
void Optimize::Method::finalizeOptimization()
{
	// save result to _opos.
	if(_optimizer->_useNormalization)
		_unnormalize(_opos, _x);
	//info(-1,0);
}

double Optimize::gradientFunction(vectorn const& _pos, vectorn& gradient)
{
	static vectorn pos;
	pos.assign(_pos);
	gradient.setSize(_opt_dimension.size() );
   
	double p0, p1;
	p0=objectiveFunction(pos);

	for (int i=0; i<_opt_dimension.size(); i++){

		Optimize::Opt_dimension& optvar=_opt_dimension[i];
		pos[i]=pos[i]+optvar.grad_step;
		p1=objectiveFunction(pos);
		pos[i]=pos[i]-optvar.grad_step;//recover pos

		if( p1==100000 )
			gradient[i]=0; //-- turn off the dimension
		else
			gradient[i]= (p1-p0)/(optvar.grad_step);
	}

#ifdef DEBUG_Optimize
	printf("grad: %s\n", gradient.output().ptr());
#endif
	return p0;
}
Optimize::Optimize()
{
	_useNormalization=true;
	_data=NULL;
}
void Optimize::init(double stepSize, int ndim, double max_step, double grad_step, Method & method)
{
	std::vector<Optimize::Opt_dimension>&	dim=_opt_dimension;
	dim.resize(ndim);

	if(max_step==1.0)
		_useNormalization=false;


	for(int i=0; i<ndim; i++)
	{
		dim[i].curval=0;
		dim[i].max_step=max_step;
		dim[i].grad_step=grad_step;
	}
	_data=(void*) new Optimize_impl( method);
	((Optimize_impl*)_data)->_interface=this;
}


Optimize::Optimize(double stepSize, std::vector<Opt_dimension> const& dim, Method & method)
{
	_useNormalization=true;
	_opt_dimension=dim;
	_data=(void*) new Optimize_impl( method);
	((Optimize_impl*)_data)->_interface=this;
}

Optimize::Optimize(double stepSize, int ndim, double max_step, double grad_step, Method & method)
{
	_useNormalization=true;
	_data=NULL;
	init(stepSize,ndim,max_step,grad_step,method);
}

Optimize::~Optimize()
{
	delete (Optimize_impl*)_data;
}

void Optimize::optimize(vectorn const& initialSolution)
{
	for(int i=0; i<_opt_dimension.size(); i++)
		_opt_dimension[i].curval=initialSolution[i];
	
	((Optimize_impl*)_data	)->optimize();	
}

vectorn& Optimize::getResult()
{
	if(_useNormalization)
		return ((Optimize_impl*)_data	)->_method._opos;	
	else
		return ((Optimize_impl*)_data	)->_method._x;	
}
void* _NRSolver::mFunc=NULL;
static Optimize::Method * g_func=NULL;
namespace NR_OLD
{
//	extern void (*opt_info)(int iter, m_real fp);
}
static double NRfunc(const vectorn& x)
{
    return g_func->func(x);
}
static double NRdfunc(const vectorn& x, vectorn& dx)
{
    return g_func->func_dfunc(x,dx);
}
static void NRdfunc2(const vectorn& x, vectorn& dx)
{
    g_func->func_dfunc(x,dx);
}

static void NRoptInfo(int iter, m_real fp)
{
    //g_func->info(iter, fp);
}
	inline void shft3(double &a, double &b, double &c, const double d)
	{
		a=b;
		b=c;
		c=d;
	}
double NR_brent(const double ax, const double bx, const double cx, double f(const double),
	const double tol, double &xmin, const int ITMAX)
{
	//const int ITMAX=100;
	const double CGOLD=0.3819660;
	const double ZEPS=DBL_EPSILON*1.0e-3;
	int iter;
	double a,b,d=0.0,etemp,fu,fv,fw,fx;
	double p,q,r,tol1,tol2,u,v,w,x,xm;
	double e=0.0;

	a=(ax < cx ? ax : cx);
	b=(ax > cx ? ax : cx);
	x=w=v=bx;
	fw=fv=fx=f(x);
	for (iter=0;iter<ITMAX;iter++) {
		xm=0.5*(a+b);
		tol2=2.0*(tol1=tol*fabs(x)+ZEPS);
		if (fabs(x-xm) <= (tol2-0.5*(b-a))) {
			xmin=x;
			return fx;
		}
		if (fabs(e) > tol1) {
			r=(x-w)*(fx-fv);
			q=(x-v)*(fx-fw);
			p=(x-v)*q-(x-w)*r;
			q=2.0*(q-r);
			if (q > 0.0) p = -p;
			q=fabs(q);
			etemp=e;
			e=d;
			if (fabs(p) >= fabs(0.5*q*etemp) || p <= q*(a-x) || p >= q*(b-x))
				d=CGOLD*(e=(x >= xm ? a-x : b-x));
			else {
				d=p/q;
				u=x+d;
				if (u-a < tol2 || b-u < tol2)
					d=SIGN(tol1,xm-x);
			}
		} else {
			d=CGOLD*(e=(x >= xm ? a-x : b-x));
		}
		u=(fabs(d) >= tol1 ? x+d : x+SIGN(tol1,d));
		fu=f(u);
		if (fu <= fx) {
			if (u >= x) a=x; else b=x;
			shft3(v,w,x,u);
			shft3(fv,fw,fx,fu);
		} else {
			if (u < x) a=u; else b=u;
			if (fu <= fw || w == x) {
				v=w;
				w=u;
				fv=fw;
				fw=fu;
			} else if (fu <= fv || v == x || v == w) {
				v=u;
				fv=fu;
			}
		}
	}
	//printf("Too many iterations in brent\n");
	xmin=x;
	return fx;
}

int ncom;
vectorn *pcom_p,*xicom_p;
double (*nrfunc)(vectorn &);
void (*nrdfun)(vectorn &, vectorn &);
void NR_linmin(vectorn &p, vectorn &xi, double &fret, double func(vectorn &), const double TOL, const int ITMAX)
{
	int j;
	double xx,xmin,fx,fb,fa,bx,ax;

	int n=p.size();
	ncom=n;
	pcom_p=new vectorn(n);
	xicom_p=new vectorn(n);
	nrfunc=func;
	vectorn &pcom=*pcom_p,&xicom=*xicom_p;
	for (j=0;j<n;j++) {
		pcom[j]=p[j];
		xicom[j]=xi[j];
	}
	ax=0.0;
	xx=1.0;
#ifdef USE_NR
	NR::mnbrak(ax,xx,bx,fa,fx,fb,NR::f1dim);
	fret=NR_brent(ax,xx,bx,NR::f1dim,TOL,xmin, ITMAX);
#else
	Msg::error("mnbrak");
#endif
	for (j=0;j<n;j++) {
		xi[j] *= xmin;
		p[j] += xi[j];
	}
	delete xicom_p;
	delete pcom_p;
}

void NR_frprmn(Vec_IO_DP &p, const DP ftol, int &iter, DP &fret,
	DP func(Vec_I_DP &), void dfunc(Vec_I_DP &, Vec_O_DP &))
{
	//const DP TOL=1.0e-8;
	const DP TOL=fret*1.0e-2;
	//const int ITMAX=2000;
	const int ITMAX=iter;
	const DP eps=1.0e-18;
	int j,its;
	DP gg,gam,fp,dgg;

	int n=p.size();
	Vec_DP g(n),h(n),xi(n);
	fp=func(p);
	dfunc(p,xi);
	for (j=0;j<n;j++) {
		g[j] = -xi[j];
		xi[j]=h[j]=g[j];
	}
	for (its=0;its<ITMAX;its++) {
		iter=its;
		NR_linmin(p,xi,fret,func,TOL, ITMAX);
		if (2.0*fabs(fret-fp) <= ftol*(fabs(fret)+fabs(fp)+eps))
			return;
		fp=fret;
		dfunc(p,xi);
		dgg=gg=0.0;
		for (j=0;j<n;j++) {
			gg += g[j]*g[j];
//		  dgg += xi[j]*xi[j];
			dgg += (xi[j]+g[j])*xi[j];
		}
		if (gg == 0.0)
			return;
		gam=dgg/gg;
		for (j=0;j<n;j++) {
			g[j] = -xi[j];
			xi[j]=h[j]=g[j]+gam*h[j];
		}
	}
//	nrerror("Too many iterations in frprmn");
	//printf("Error!!!! Too many iterations in frprmn\n");
}
void Optimize::ConjugateGradient::optimize(vectorn & initial)
{
	g_func=this;
	vectorn& x=getInout();
	int iter=int(max_iter);
#ifdef USE_NR
	//			NR_OLD::opt_info=&NRoptInfo;
	NR_frprmn(x, tol,iter, thr, NRfunc, NRdfunc2);
#else
	Msg::error("frprmn");
#endif
}


void Optimize::_initSquareTerms(int ndim)
{
	terms.resize(0);
	termCoeffs.resize(ndim);
	for(int i=0; i<ndim; i++)
		termCoeffs[i].resize(0);
}
double Optimize::_updateSquareTermsGradient(vectorn const& pos, vectorn& grad)
{
	double o=0.0;
	// objective function value
	for(int i=0; i<terms.size(); i++)
	{
		auto& term=terms[i];
		auto& ii=term.indices;
		auto& w=term.coeffs;
		double v=0.0;
		for(int j=0;j< ii.size(); j++)
			v+=pos(ii(j))*w(j);
		v+=w(ii.size());
		term.v=v;
		o+=v*v;
	}

	// gradient
	for (int i=0; i<grad.size();i++){
		auto& termCoeff=termCoeffs[i];
		for (int j=0; j< termCoeff.size(); j++)
		{
			auto& tc=termCoeff[j];
			int ti=tc.termIndex;
			grad(i)+= terms[ti].v*2*tc.termCoef;
		}
	}
	return o;
}
void Optimize::addSquared(intvectorn const& index, vectorn const& coef)
{
	int iterm=terms.size();
	terms.push_back({index, coef});
	for(int i=0; i<index.size(); i++)
		termCoeffs[index(i)].push_back({iterm, coef(i)});
}

