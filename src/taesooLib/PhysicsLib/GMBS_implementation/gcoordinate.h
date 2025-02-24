//================================================================================
//         BASIC ELEMENTS FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_COORDINATE_
#define _GMBS_GEOMETRIC_COORDINATE_


//=============================================================
//                 GCoordinate
//=============================================================
class GCoordinate
{
public:
	double q;								// displacement
	double dq;								// velocity
	double ddq;								// acceleration
	double tau;								// torque or force

	double DqDp, DdqDp, DddqDp, DtauDp;		// derivatives w.r.t. an arbitrary scalar variable p

	bool bPrescribed;						// set bPrescribed = true if q,dq,ddq are prescribed

	double qLL, qUL;						// lower and upper limits of q
	double dqLL, dqUL;						// lower and upper limits of dq
	double ddqLL, ddqUL;					// lower and upper limits of ddq
	double tauLL, tauUL;					// lower and upper limits of tau
	int system_idx_coord; // taesoo
public:
	GCoordinate() : q(0), dq(0), ddq(0), tau(0), DqDp(0), DdqDp(0), DddqDp(0), DtauDp(0), bPrescribed(false)
		, qLL(-1E20), qUL(1E20), dqLL(-1E20), dqUL(1E20), ddqLL(-1E20), ddqUL(1E20), tauLL(-1E20), tauUL(1E20), system_idx_coord(-1) {}
	~GCoordinate() {}

	void initValues() {
		q = dq = ddq = tau = DqDp = DdqDp = DddqDp = DtauDp = 0.0;
		qLL = dqLL = ddqLL = tauLL = -1E20;
		qUL = dqUL = ddqUL = tauUL = 1E20;
	}
};


#endif

