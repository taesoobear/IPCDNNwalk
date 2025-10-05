#include "stdafx.h"
#include "Function.h"
#include "statistics.h"
#include "GnuPlot.h"
#include "../BaseLib/utility/tfile.h"
#include "../BaseLib/utility/operatorString.h"
#include "../BaseLib/math/Operator_NR.h"
#include "../BaseLib/math/Operator.h"
//#include "../BaseLib/math/matrix3.h"

void Function::test(const char* id)
{
	TString tid=id;
	vectorn x;
	x.setValues(10, 0.1 , 0.15 ,0.2 ,0.25 ,0.65 ,0.7 ,0.75 ,0.8 ,0.85 ,0.9);

	vectorn noise;
	noise.setValues(10, 0.1, 0.3, 0.7, 0.2, 0.8, 0.4, 0.2, 0.9, 0.5, 0.3);
	int ndata = x.size();
	vectorn t;
	t.each1(s1::SIN, x*2.0*M_PI);
	t+=noise*0.05;

	gnuPlotQueue q(tid, 2, tid, "x", "t");

	matrixn xx;
	xx.concatColumn(x.column(), t.column());

	q.plotScattered(xx, "source");

	learn(x.column(), t.column());

	vectorn xtest;
	v::linspace(xtest ,0, 1, 100);
	vectorn ytest(100);

	vectorn source(1),target(1);
	printf("\n");
	for(int i=0; i<xtest.size();i++)
	{
		source[0]=xtest[i];
		mapping(source, target);
		ytest[i]=target[0];
	}

	matrixn xt;
	xt.concatColumn(xtest.column(), ytest.column());

	q.plotParametric(xt, "test");
}

void Function::mapping(const matrixn& sources, matrixn& targets) const
{
	targets.setSize(sources.rows(), dimRange());

	for(int i=0; i<sources.rows(); i++)
	{
	    vectornView sri=sources.row(i);
	    vectornView tri=targets.row(i);
		mapping(sri, tri);
	}
}

void Function::test2(const char* id, int numSample)
{
	TString tid(id);

	matrixn x,t;
	x.setSize(14,2);
	t.setSize(14,1);
	//numSample=7일때 gptest2.m과 동일)
	x.column(0).setValues(14,0.1, 0.2, 0.3, 0.4, 0.7, 0.7, 0.75,
		0.1, 0.2, 0.3, 0.9, 0.2, 0.2, 0.1);
	x.column(1).setValues(14, 0.2, 0.7, 0.3, 0.9, 0.3, 0.2, 0.1,
		0.6, 0.3, 0.2, 0.7, 0.9, 0.8, 0.1);  // 어려운 케이스.
	t.column(0).setValues(14, 0.1, 0.2,  0.3, 0.6,  0.45,  0.45, 0.3 ,
		0.3, 0.8, 0.7, 0.6 , 0.6, 0.2, 0.1);

	x.resize(numSample,2);
	t.resize(numSample,1);

	learn(x, t);

	matrixn xtest;
	xtest.setSize(50*50,2);

	gnuPlotQueue q(tid, 3, tid, "x1", "x2", "t");

	int k=0;
	for (int i=0; i<50; i++)
	{
		for(int j=0; j<50; j++)
		{
			xtest[k][0]=m_real(i)/50.0;
			xtest[k][1]=m_real(j)/50.0;
			k=k+1;
		}
	}

	vectorn ytest(xtest.rows());
	vectorn ytest1(1);

	vectorn errorbar,var;
	errorbar.setAllValue(xtest.rows(), 0.0);

	for(int i=0; i<xtest.rows(); i++)
	{
		//printf("%d ", i);
		//fflush(stdout);
		mapping(xtest.row(i), ytest1);
		ytest[i]=ytest1[0];

		variance(xtest.row(i), var);
		if(var.size())
			errorbar[i]=sqrt(var[0]);
	}

	matrixn xx;
	xx.concatColumn(x, t);

	q.plotScattered(xx, "source");

	matrixn xt;
	xt.concatColumn(xtest, ytest.column());

	q.plotParametric(xt, "test", 50);

	if(errorbar[0]>0.00001)
	{
		matrixn et;
		et.concatColumn(xtest, (ytest+errorbar).column());
		q.plotParametric(et, "errorbar", 50);
	}
}

BijectiveFunction::BijectiveFunction(void)
{
}

BijectiveFunction::~BijectiveFunction(void)
{
}

void Function::plot(const matrixn& domain, const matrixn& range, const char* identifier)
{

	bool bYup;
	bYup=gnuPlot::setYUP(false);

	// split id into dir+lfn
	TString id, lfn, dir;
	id=identifier;
	int currDir;
	if((currDir=id.findChar(0, '/'))!=-1)
	{
		lfn=id.right(-1*currDir-1);
		dir=id.left(currDir);
	}
	else
		lfn=id;

	if(mSourceDimName.size()!=domain.cols() && domain.cols()>0)
	{
		mSourceDimName.init(domain.cols());

		for(int i=0; i<domain.cols(); i++)
			mSourceDimName[i].format("%d",i);
	}

	if(mTargetDimName.size()!=range.cols()&& range.cols()>0)
	{
		mTargetDimName.init(range.cols());

		for(int i=0; i<range.cols(); i++)
			mTargetDimName[i].format("%d",i);
	}

	for(int dparam=0; dparam<range.cols(); dparam++)
	{
		int numTrace=domain.rows();

		for(int col=0; col<domain.cols(); col++)
		{
			matrixn trace(numTrace, 2);

			for(int i=0; i<numTrace; i++)
			{
				trace[i][0]=domain.value(i, col);
				trace[i][1]=range.value(i,dparam);
			}

			if(dir.length())
				gnuPlot::plotScattered(TString(sz0::format("%s/function_%s%s_%s", dir.ptr(), mSourceDimName[col].ptr(), mTargetDimName[dparam].ptr(), lfn.ptr())), trace, lfn, mSourceDimName[col], mTargetDimName[dparam]);
			else
				gnuPlot::plotScattered(TString(sz0::format("function_%s%s_%s", mSourceDimName[col].ptr(), mTargetDimName[dparam].ptr(), lfn.ptr())), trace, lfn, mSourceDimName[col], mTargetDimName[dparam]);
		}

		if(domain.cols()==2)
		{
			matrixn traceSpeedTurn_Delta(numTrace, 3);
			int col1=0;
			int col2=1;
			for(int i=0; i<numTrace; i++)
			{
				traceSpeedTurn_Delta[i][0]=domain.value(i, col1);
				traceSpeedTurn_Delta[i][1]=domain.value(i,col2);
				traceSpeedTurn_Delta[i][2]=range.value(i,dparam);
			}

			if(dir.length())
				gnuPlot::plotScattered(TString(sz0::format("%s/function3d_%s%s%s_%s", dir.ptr(), mSourceDimName[col1].ptr(), mSourceDimName[col2].ptr(), mTargetDimName[dparam].ptr(), lfn.ptr())), traceSpeedTurn_Delta, lfn, mSourceDimName[col1], mSourceDimName[col2], mTargetDimName[dparam]);
			else
				gnuPlot::plotScattered(TString(sz0::format("function3d_%s%s%s_%s", mSourceDimName[col1].ptr(), mSourceDimName[col2].ptr(), mTargetDimName[dparam].ptr(), lfn.ptr())), traceSpeedTurn_Delta, lfn, mSourceDimName[col1], mSourceDimName[col2], mTargetDimName[dparam]);

		}

	}

	gnuPlot::setYUP(bYup);


}

void LinearFunction ::learn(const matrixn& sources, const matrixn& targets)
{
	Function::learn(sources, targets);

	// source linearCoef   a column of target
	// (v1 t1 1)  a         k1
	// (v2 t2 1)  b         k2
	// ...       *c    =
	// (vn tn 1)            kn

	// linear system
	//  L         x    =    k
	//  source * mLinearCoef = target

	matrixn L;
	L.setSize(sources.rows(), sources.cols()+1);

	L.setValue(0,0, sources);

	// turning speed는 abs에 비례하도록 모델을 세웠다.
	for(int i=0; i<L.rows(); i++)
		L[i][1]=ABS(L[i][1]);

	L.setColumn(sources.cols(), 1.f);

	const matrixn& k=targets;

	matrixn invL;
	// use pseudo inverse for the least square solution.
	m::pseudoInverse(invL,L);

	mLinearCoef.mult(invL, k);


	// linear system
	//  (source,1) * (mLinearCoef,(0,0,0,...,1)^T) = (target,1)^T
	// inverse linear system
	// (source,1)=(target,1)*inv(mLinearCoef)

	matrixn linearCoef;
	linearCoef.setSize(mLinearCoef.rows(), mLinearCoef.cols()+1);
	linearCoef.setValue(0,0, mLinearCoef);
	vectorn zeros_one(mLinearCoef.rows());
	zeros_one.setAllValue(0.f);
	zeros_one[mLinearCoef.rows()-1]=1.f;
	linearCoef.setColumn(mLinearCoef.cols(), zeros_one);
	m::pseudoInverse(mInvLinearCoef,linearCoef);
}

void LinearFunction ::mapping(const vectorn& source, vectorn& target) const
{
	target.setSize(mLinearCoef.cols());

	// linear system
	//  (source,1) * mLinearCoef = target

	for(int i=0; i<mLinearCoef.cols(); i++)
	{
		target[i]=0;
		for(int j=0; j<mLinearCoef.rows()-1; j++)
			target[i]+=source[j]*mLinearCoef[j][i];

		target[i]+=mLinearCoef[mLinearCoef.rows()-1][i];
	}
}

void LinearFunction ::inverseMapping(const vectorn& target, vectorn& source) const
{
	// linear system
	//  (source,1) * (mLinearCoef,(0,0,0,...,1)^T) = (target,1)
	// inverse linear system
	// (source,1)=(target,1)*inv(mLinearCoef)

	matrixn matSource;
	matrixn vecTarget(1, target.size()+1);
	vecTarget.row(0).range(0, target.size())=target;
	vecTarget[0][target.size()]=1.f;
	matSource.mult(vecTarget, mInvLinearCoef);

	Msg::verify(matSource.rows()==1 && matSource.cols()== target.size()+1 && isSimilar(matSource[target.size()][0], 1.f), "inverseMapping Error");

	source.setSize(target.size());
	for(int i=0; i<target.size(); i++)
		source[i]=matSource[i][0];
}
void LeastSquareFitting ::learn(const matrixn& sources, const matrixn& targets)
{

	Function::learn(sources, targets);
	Msg::verify(targets.cols()==1, "not imp");
	Msg::verify(sources.cols()==1, "not imp");

	matrixn nsources(targets.rows(), 2);
	for (int i=0; i<targets.rows(); i++)
	{
		double ti=sources(i,0);
		nsources[i][0]=ti;
		nsources[i][1]=ti*ti;
	}
	LinearFunction cc;
	cc.learn(nsources, targets);
	//printf("%s\n", cc.mLinearCoef.output().ptr());
	c.y=cc.mLinearCoef[0][0]; // linear
	c.z=cc.mLinearCoef[1][0]; // quadratic
	c.x=cc.mLinearCoef[2][0]; // constant
	/*
	matrix3 m;
	int n=sources.rows();
	m._11=n;
	m._12=0;
	m._13=0;
	m._23=0;
	m._33=0;
	vector3 d(0,0,0);
	for (int i=0; i<n ;i++)
	{
		double ti=sources(i,0);
		double di=targets(i,0);

		// ti
		d.x+=di;
		m._12+=ti; 
		d.y+=di*ti;
		
		// ti*ti
		ti*=ti;
		m._13+=ti; 
		d.z+=di*ti;

		// ti^3
		ti*=ti;
		m._23+=ti; 

		// ti^4
		ti*=ti;
		m._33+=ti; 
	}
	m._21=m._12;
	m._31=m._22=m._13;
	m._32=m._23;
	matrix3 o;
	o.inverse(m);
	c=o*d;
	*/
}

void LeastSquareFitting ::mapping(const vectorn& source, vectorn& target) const
{
	//printf("%s %s\n", source.output().ptr(), target.output().ptr());
	target.resize(source.size());
	for (int i=0,n=source.size(); i<n; i++)
	{
		double ti=source(i);
		target[i]=c.x+c.y*ti+c.z*ti*ti;
	}
}

void NormalizedFunction ::_initialize(const matrixn& sourceSamples)
{
	if(m_cNormalizingInterval.size()==0)
		m_cNormalizingInterval.calcRange(sourceSamples);

	m_aNormalizedSamples.assign(sourceSamples);
	m_aNormalizedSamples.normalize(m_cNormalizingInterval.start(), m_cNormalizingInterval.end());

	m_vResultMin.minimum(m_aNormalizedSamples);
	m_vResultMax.maximum(m_aNormalizedSamples);

	m_vMean.setSize(m_aNormalizedSamples.cols());
	m_vStddev.setSize(m_aNormalizedSamples.cols());
	vectorn column;
	for(int i=0; i<m_aNormalizedSamples.cols(); i++)
	{
		m_aNormalizedSamples.getColumn(i,column);
		statistics::NormalDistribution n(column);
		m_vMean[i]=n.sampleMean();
		m_vStddev[i]=n.sampleStdDev();
	}
}

void NonlinearFunctionIDW::learn(const matrixn& sources, const matrixn& targets)
{
	Function::learn(sources, targets);

	// parameter normalize
	//mInterpolation.setMinMax(vectorn(), vectorn());

	intervalN intervalParam;
	intervalParam.setSize(sources.cols());
	// parameter normalize는 하지 않는다.
	intervalParam.start_ref().setAllValue(0);
	intervalParam.end_ref().setAllValue(1);
	mInterpolation.setMinMax(intervalParam.start(), intervalParam.end());

	mInterpolation.initialize(sources);
	mTarget=targets;
}

void NonlinearFunctionIDW::mapping(const vectorn& source, vectorn& target, intvectorn & index, vectorn& weight) const
{
	((KNearestInterpolation&)mInterpolation).calcWeight(source, index, weight);
	target.setSize(mTarget.cols());
	target.setAllValue(0);

	for(int i=0; i<index.size(); i++)
	{
		target.multAdd(mTarget.row(index[i]), weight[i]);
	}
}
void NonlinearFunctionIDW::mapping(const vectorn& source, vectorn& target, vectorn& weights) const
{
	intvectorn index;
	vectorn weight;
	((KNearestInterpolation&)mInterpolation).calcWeight(source, index, weight);

	weights.setSize(mTarget.rows());
	weights.setAllValue(0);
	for(int i=0; i<weight.size(); i++)
		weights[index[i]]=weight[i];

	target.setSize(mTarget.cols());
	target.setAllValue(0);

	for(int i=0; i<index.size(); i++)
	{
		target.multAdd(mTarget.row(index[i]), weight[i]);
	}
}
void NonlinearFunctionIDW::mapping(const vectorn& source, vectorn& target) const
{
	intvectorn index;
	vectorn weight;
	((KNearestInterpolation&)mInterpolation).calcWeight(source, index, weight);

	target.setSize(mTarget.cols());
	target.setAllValue(0);


	for(int i=0; i<index.size(); i++)
	{
		target.multAdd(mTarget.row(index[i]), weight[i]);
	}
}



void InterpolationFunction ::learn(const matrixn& sources, const matrixn& targets)
{
	Function::learn(sources, targets);

	intervalN intervalParam;
	intervalParam.setSize(sources.cols());

	// parameter normalize는 하지 않는다.
	intervalParam.start_ref().setAllValue(0);
	intervalParam.end_ref().setAllValue(1);
	mInterpolation->setMinMax(intervalParam.start(), intervalParam.end());
	mInterpolation->initialize(sources);
	mTarget=targets;
}

void InterpolationFunction ::mapping(const vectorn& source, vectorn& target) const
{
	intvectorn index;
	vectorn weight;
	mInterpolation->calcWeightNormalized(source, index, weight);

	target.setSize(mTarget.cols());
	target.setAllValue(0);

	for(int i=0; i<index.size(); i++)
		target.multAdd(mTarget.row(index[i]), weight[i]);
}


void UniversalFunction::learn(const matrixn & sources, const matrixn& targets)
{
	Function::learn(sources, targets);

	mTrendEstimator->learn(sources, targets);

	mTrends.setSameSize(targets);

	for(int i=0; i<targets.rows(); i++)
	{
	    vectornView sri=sources.row(i);
	    vectornView tri=mTrends.row(i);
		mTrendEstimator->mapping(sri, tri);
	}

	mResiduals.subtract(targets, mTrends);

	mResidualEstimator->learn(sources, mResiduals);
}

void UniversalFunction::mapping(const vectorn& source, vectorn& target) const
{
	vectorn trend;
	mTrendEstimator->mapping(source, trend);
	mResidualEstimator->mapping(source, target);
	target+=trend;
}

#include "tpros/tpros.h"

TPros::TPros(const char* option):Function()
{
	mOption=option;
}

TPros::~TPros()
{
	for(int i=0;i<pGP.size();++i){
		delete pGP[i];
	}
}

void TPros::pack(BinaryFile& file)
{
	assert(mIsLearned);

	file.packInt(pGP.size());
	file.packInt(mDimSource);
	file.packInt(mDimTarget);
	file.packInt(mNumTrainingData);
	for(int i=0; i<pGP.size(); i++)
		pGP[i]->save(file);
}
void TPros::unpack(BinaryFile& file)
{
	for(int i=0;i<pGP.size();++i){
		delete pGP[i];
	}
	pGP.resize(file.unpackInt());
	file.unpackInt(mDimSource);
	file.unpackInt(mDimTarget);
	file.unpackInt(mNumTrainingData);
	mIsLearned=true;

	for(int i=0;i<pGP.size();++i){
		pGP[i]=new tpros(mOption.ptr(),mDimSource, mNumTrainingData);
	}

	for(int i=0; i<pGP.size(); i++)
		pGP[i]->load(file);
}

BinaryFile* TProsFromFile::mpFile=NULL;
bool TProsFromFile::mbSave=true;

void TProsFromFile::learn(const matrixn& sources, const matrixn& targets)
{
	if(sources.rows()<2) mOption="tpros spec/tspecSmall";
	if(mbSave)
	{
		TPros::learn(sources, targets);
		pack(*mpFile);
	}
	else
	{
		try
		{
			unpack(*mpFile);
		}
		catch(...)
		{
			printf("unpack failed.. perform full leaning.\n");
			printf("sources: %s\ntargets:%s\n", sources.output().ptr(), targets.output().ptr());
			BinaryFile bf;
			bf.openWrite("tprosDebug.data");
			bf.pack(sources);
			bf.pack(targets);
			bf.close();
			TPros::learn(sources, targets);
		}
	}
}

void TProsFromFile::testPackUnpack()
{


	BinaryFile ff(true, "gptest.tpros");

	TPros f;
	f.test ("1gp");
	f.pack(ff);
	f.test2("2gp",7);
	f.pack(ff);
	f.test2("3gp",2);
	f.pack(ff);
	f.test2("4gp",1);
	f.pack(ff);
	f.test2("5gp",14);
	f.pack(ff);

	ff.close();

	{
		BinaryFile ff;
		ff.openRead("gptest.tpros");
		matrixn xtest;
		xtest.setSize(50*50,2);

		int k=0;
		for (int i=0; i<50; i++)
		{
			for(int j=0; j<50; j++)
			{
				xtest[k][0]=m_real(i)/50.0;
				xtest[k][1]=m_real(j)/50.0;
				k=k+1;
			}
		}

		for(int itid=1; itid<=5; itid++)
		{
			f.unpack(ff);

			vectorn ytest(xtest.rows());
			vectorn ytest1(1);
			for(int i=0; i<xtest.rows(); i++)
			{
				f.mapping(xtest.row(i), ytest1);
				ytest[i]=ytest1[0];
			}

			matrixn xt;
			xt.concatColumn(xtest, ytest.column());

			TString tid;
			tid.format("%dgpl", itid);
			gnuPlot::plotScattered(tid+"test", xt, tid+"test", "xtest1", "xtest2", "ytest");
		}
	}


}
void TPros::learn(const matrixn& sources, const matrixn& targets)
{
	Function::learn(sources, targets);
	mDimSource=sources.cols();mDimTarget=targets.cols();
	mNumTrainingData=sources.rows();
	int i,j;

	pGP.resize(mDimTarget);
	for(i=0;i<pGP.size();++i){
		pGP[i]=new tpros(mOption.ptr(),sources.cols(), sources.rows());
	}

	for(i=0;i<pGP.size();++i)
	{
		vectorn targetVec;
		targets.getColumn(i,targetVec);
		pGP[i]->learn(sources, targetVec);
	}
}

void TPros::mapping(const vectorn& source, vectorn& target) const
{
	int i;
	if(!isLearned())
	{
		// learning이 안되어 있음.
		ASSERT(0);
	}
	target.setSize(mDimTarget);
	for(i=0;i<pGP.size();++i){

		m_real mean, sig;
		pGP[i]->test(source,mean,sig);
		target[i]=mean;
	}
}

m_real TPros::bestProb(const vectorn& source) const
{
	int i;
	if(!isLearned())
	{
		// learning이 안되어 있음.
		ASSERT(0);
	}
	m_real bestProb=1.0;
//	printf("sig ");

	for(i=0;i<pGP.size();++i)
	{
		m_real mean, sig;
		pGP[i]->test(source,mean,sig);
		bestProb*=1.0/sig;

//		printf("%f ", sig);
	}
//	printf("\n");
	return bestProb;
}


m_real TPros::prob(const vectorn& source, const vectorn& target) const
{
	int i;
	if(!isLearned())
	{
		// learning이 안되어 있음.
		ASSERT(0);
	}
	m_real bestProb=1.0;


	for(i=0;i<pGP.size();++i)
	{
		m_real mean, sig;
		pGP[i]->test(source,mean,sig);

		m_real z=(target[i]-mean)/sig;

		bestProb*=exp(-0.5*SQR(z))/sig;


	}

	return bestProb;
}

void FunctionUtil::_dump(matrixn const& source, matrixn const& target)
{
	BinaryFile file(true, mName);
	file.pack(source);
	file.pack(target);
	file.close();
}

void FunctionUtil::learnFromDumpfile(const char* filename, Function& func, int outdim)
{
	matrixn source, target;

	BinaryFile file(false, filename);
	file.unpack(source);
	file.unpack(target);
	file.close();

	if(outdim==-1)
		func.learn(source, target);
	else
		func.learn(source, target.range(0, target.rows(), outdim, outdim+1));
}
BayesianLinearRegression::BayesianLinearRegression(int order, m_real noisePrior)
:mOrder(order), v(noisePrior)
{
}

BayesianLinearRegression::~BayesianLinearRegression(void)
{
}

static void v0_pow(vectorn &c, double mK) 
{
	for(int i=0; i<c.size(); i++) c[i]=::pow(c[i], mK);
}
void BayesianLinearRegression::learn(const matrixn& sources, const matrixn& targets)
{
	Function::learn(sources, targets);

	
	// set data matrices X (m by N) and Y (d by N)
	m= 1+sources.cols()*mOrder;
	d=targets.cols();
	N=sources.rows();

	X.setSize(m, N);	
	X.row(0).setAllValue(1.0);
	
	for(int i=1; i<=mOrder; i++)
	{
		X.range(1+dimDomain()*(i-1), 1+dimDomain()*i).transpose(sources);
		if(i>1)
		{
			for (int r=1+dimDomain()*(i-1); r<1+dimDomain()*i; r++)
				v0_pow(X.row(r).lval(), i);
		}
	}
	Y.transpose(targets);

	// find alpha that maximizes the evidence P(Y|X, V, alpha). See Equation (35)

	m_real md=(m_real)m*d;
	matrixn invV;
	invV.identity(d);
	invV*=1.0/v;
	XXt.multABt( X, X);
	m::LUinvert(InvXXt,XXt);
	YXt.multABt (Y, X);
	m::multABAt(cross, YXt, InvXXt);
	m_real tr=(invV*cross).trace();
	ASSERT(tr!=md);
	alpha=md/(tr-md);	
	ASSERT(alpha==alpha);
	
	YXtInvXXt.mult(YXt, InvXXt);
	YYt.multABt( Y, Y);

	Sy_x.subtract(YYt, cross*(1.0/(1.0+alpha)));

	printf("Evidence %f Likelihood %f\n", calcEvidence(), logProb(sources.row(0), targets.row(0)));
}

m_real BayesianLinearRegression::calcEvidence()
{
	m_real md=(m_real)m*d;
	m_real minusNover2=-1.0*m_real(N)/2.0;
	
	if(v<FERR) v=FERR;
	matrixn invV;
	invV.identity(d);
	invV*=1.0/v;
	
	m_real tr=(invV*Sy_x).trace();

	m_real t1=md/2*log(alpha/(alpha+1.0));
	m_real t2=minusNover2*log(2.0*M_PI*v);
	m_real t3=-0.5*tr;
	return t1+t2+t3;
}

void BayesianLinearRegression::mapping(const vectorn& source, vectorn& target) const
{
	vectorn x;
	mean(source, x, target);
	
}

void BayesianLinearRegression::mean(const vectorn& source, vectorn & x, vectorn& target) const
{
	assert(dimDomain()==source.size());
	x.setSize(1+dimDomain()*mOrder);
	x[0]=1.0;
	
	for(int i=1; i<=mOrder; i++)
	{
		x.range(1+dimDomain()*(i-1), 1+dimDomain()*i)=source;
		if(i>1)
		{
			for (int r=1+dimDomain()*(i-1); r<1+dimDomain()*i; r++)
				x[r]=::pow(x[r], i);
		}
	}

	target.multmat(YXtInvXXt, x);
	target*=1.0/(1.0+alpha);
}

void BayesianLinearRegression::prepare(const vectorn& source) const
{
	vectorn x;
	this->mean(source, x, __mean);

	m_real invC;
	matrixn xSx;
	m::multAtBA(xSx, x.column(), InvXXt);
	xSx*=1.0/(1.0+alpha);
	invC=1.0+xSx[0][0];

	m_real logprob=0.0;

	m_real Var=MAX(0.0001, invC*v);
	__stdev=sqrt(Var);
}

void BayesianLinearRegression::variance(const vectorn& source, vectorn& var) const			
{
	vectorn mean, x;
	this->mean(source, x, mean);

	m_real invC;
	matrixn xSx;
	m::multAtBA(xSx, x.column(), InvXXt);
	xSx*=1.0/(1.0+alpha);
	invC=1.0+xSx[0][0];

	var.setAllValue(dimRange(), MAX(0.01,invC*v));
}

m_real BayesianLinearRegression::logProbPrepared(const vectorn& target) const
{
	m_real logprob=0.0;

	for(int i=0; i<target.size(); i++)
	{

		statistics::NormalDistribution n(__mean[i], __stdev);
		logprob+=n.logProb(target[i]);
	}
	return logprob;
}

