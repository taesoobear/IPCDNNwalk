void demo1();
int clust(int  argc,TStrings const& argv);

// refactored from original source codes
namespace Clust
{
	/* SigSet (Signature Set) data stucture used throughout package.         */
	/*   ClassSig (Class Signature) data stucture holds the parameters       */
	/*       of a single Gaussian mixture model. SigSet.nclasses is the      */
	/*       number of ClassSig's in a SigSet.                               */
	/*     SubSig (Subsignature) data stucture holds each component of a     */
	/*         Gaussian mixture model. SigSet.classSig[k].nsubclasses is the */
	/*         number of SubSig's in a ClassSig.                             */

	struct ClassData
	{
		int npixels() const	{return x.rows();}
		double SummedWeights;
		matrixn x; /* list of pixel vectors:     x[npixels][nbands] */

		// temporary variables. (used for passing informations from regroup() to reestimate())
		matrixn p; /* prob pixel is in subclass: p[npixels][subclasses] */
		vectorn w; /* weight of pixel:           w[npixels] */
	};

	// A single Gaussian distribution
	class SubSigFull;
	class SubSigDiag;
	class SubSig
	{
	public:
		SubSig(){}
		double N;       /* expected number of pixels in subcluster */
		double pi;      /* probability of component in GMM */
		vectorn means;  /* mean of component in GMM */
		double cnst;    /* normalizing constant for multivariate Gaussian */
		int used;

		virtual void init(int nbands);
		virtual double loglike(vectorn const& x){ASSERT(0); return 0;}
		virtual void setCov(matrixn const& R, double Rmin){ASSERT(0);}
		virtual void estimateCov(ClassData* Data, int subSigIndex, double Rmin){ASSERT(0);}
		virtual void compute_constants(){ASSERT(0);}
		virtual void add(SubSig const&SubSig1, SubSig const& SubSig2){ASSERT(0);}
		virtual double distance(SubSig const& SubSig2){ASSERT(0);return 0;}

		int nbands() const	{return means.size();}

		virtual SubSig& operator=(const SubSig& other){ASSERT(0);return *this;}
		virtual SubSig& operator=(const SubSigFull& other){ASSERT(0);return *this;}
		virtual SubSig& operator=(const SubSigDiag& other){ASSERT(0);return *this;}

		virtual void writeToFile(FILE *fd){ASSERT(0);}

	};

	class SubSigFull : public SubSig
	{
	public:
		SubSigFull(){}
		matrixn R;     /* convarance of component in GMM */
		matrixn Rinv;  /* inverse of R */

		virtual void init(int nbands);
		virtual double loglike(vectorn const& x);
		virtual void setCov(matrixn const& R, double Rmin);
		virtual void estimateCov(ClassData* Data, int subSigIndex, double Rmin);
		virtual void compute_constants();
		virtual void add(SubSig const&SubSig1, SubSig const& SubSig2);
		virtual double distance(SubSig const& SubSig2);
		virtual SubSigFull& operator=(const SubSig& other);
		virtual SubSigFull& operator=(const SubSigFull& other){return this->operator =(other);}

		virtual void writeToFile(FILE *fd);
	};

	class SubSigDiag : public SubSig
	{
	public:
		SubSigDiag(){}
		vectorn diagR; /* convarance of component in GMM */
		vectorn diagRinv;/* inverse of R */
		virtual void init(int nbands);
		virtual double loglike(vectorn const& x);
		virtual void setCov(matrixn const& R, double Rmin);
		virtual void estimateCov(ClassData* Data, int subSigIndex, double Rmin);
		virtual void compute_constants();
		virtual void add(SubSig const&SubSig1, SubSig const& SubSig2);
		virtual double distance(SubSig const& SubSig2);
		virtual SubSigDiag& operator=(const SubSig& other);
		virtual SubSigDiag& operator=(const SubSigDiag& other){return this->operator =(other);}
		virtual SubSigDiag& operator=(const SubSigFull& other){ASSERT(0); return *this;}

		virtual void writeToFile(FILE *fd);
	};

	class SubSigSphr : public SubSig
	{
	public:
		SubSigSphr(){}
		m_real var;
		virtual void init(int nbands);
		virtual double loglike(vectorn const& x);
		virtual void setCov(matrixn const& R, double Rmin);
		virtual void estimateCov(ClassData* Data, int subSigIndex, double Rmin);
		virtual void compute_constants();
		virtual void add(SubSig const&SubSig1, SubSig const& SubSig2);
		virtual double distance(SubSig const& SubSig2);
		virtual SubSigSphr& operator=(const SubSig& other);
		virtual SubSigSphr& operator=(const SubSigSphr& other){return this->operator =(other);}

		virtual void writeToFile(FILE *fd);
	};

	class ClassSig
	{
	public:
		//  option1 - (optional) controls clustering model\n");
		//    full - (default) use full convariance matrices
		//    diag - use diagonal convariance matrices
		//    sphr - use spherical covariance matrices
		//  option2 - (optional) controls number of clusters\n");
		//    0 - (default) estimate number of clusters\n");
		//    n - use n clusters in mixture model with n<#_subclasses");
		ClassSig(int init_num_of_subclasses, matrixn const& samples, const char* option1="full", int  option2=0, const char* outfile=0);
		ClassSig();
		virtual ~ClassSig();

		// retrieve results.
		int numGaussian() const						{return mSubSig.size();}
		double pi(int iGaussian) const				{return mSubSig[iGaussian].pi;}
		m_real logLikelihood(vectorn const& sample) const;
		void logLikelihoods(vectorn const&sample, vectorn& loglikelihoods) const;

	private:
		void init(int nbands, const char* title);
		long classnum;
		TString title;
		int used;
		int type;

		// store a GMM ( a set of Gaussian distributions)
		class SubSigs : protected TArray<SubSig>
		{
		public:
			SubSigs ():TArray<SubSig>(){mSize=0;}
			SubSigs& operator=(const SubSigs& other);
			void insertNewSubSig();
			int nbands;

			int mSize;
			void changeFactory(TFactory<SubSig>* pF)	{ TArray<SubSig>::changeFactory(pF);}
			int size() const							{ return mSize;}
			SubSig& operator[](int nIndex) const		{ return TArray<SubSig>::operator [](nIndex);}
			void remove(int start, int end);
			void resize(int nsize);
		};
		SubSigs mSubSig;

		int nbands()	const	{return mSubSig.nbands;}
		int nsubclasses() const {return mSubSig.size();}

		void allocClassData(matrixn const& samples);
		double averageVariance() const;
		int subcluster(
			int desired_num,  /* Input: desired number of subclusters. */
							  /*      0=>ignore this input. */
			int option,       /* Input: type of clustering to use */
							  /*      option=1=CLUSTER_FULL=>full covariance matrix */
							  /*      option=0=CLUSTER_DIAG=>diagonal covariance matrix */
			double Rmin,      /* Minimum value for diagonal elements of convariance */
			int *Max_num);     /* Output: maximum number of allowed subclusters */

		void seed(double Rmin, int option);

		// perform EM steps
		double refine_clusters(double Rmin, int option);
		void reduce_order(int *min_ii,  int *min_jj);
		void reestimate(double Rmin, int option);
		void compute_constants();
		void normalize_pi();
		double regroup();
		void writeToFile(FILE *fd);

		ClassData classData;

		// temporary variable
		Clust::SubSig* mpSubSig3;			// used in reduce_order
		Clust::ClassSig::SubSigs * mpSmin;	// used in subCluster
	};

}
