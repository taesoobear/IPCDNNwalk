#include <algorithm>
#include <fstream>

#include "gsystem_ik.h"
#include "rmatrix3j.h"
#include "liegroup_rmatrix3_ext.h"

using namespace std;


//=============================================================
//                 GSystemIK
//=============================================================

bool GSystemIK::buildConstrIK_dq(RMatrix &J, RMatrix &V, vector<GBodyRigid*> pbodies, vector<Vec3> pos, vector<se3> V_target, vector< vector<int> > idxC)
{
	int i, j, k;
	int cnt;			// a counter
	int nb;				// number of bodies
	int ncik;			// number of IK constraints
	list<GCoordinate*>::iterator iter_pcoord, iter_pcoord2;

	nb = int(pbodies.size());

	if ( pos.size() != nb || V_target.size() != nb || idxC.size() != nb ) return false;

	// counts the number of IK constraints
	ncik = 0;
	for (i=0; i<nb; i++) {
		for ( j=0; j<int(idxC[i].size()); j++) { 
			if ( idxC[i][j] < 0 || idxC[i][j] > 5 ) return false;
		}
		ncik += int(idxC[i].size());
	}
	J.SetZero(ncik, getNumCoordinates());
	V.SetZero(ncik, 1);

	// build J, V
	cnt = 0;
	for (i=0; i<nb; i++) {
		// update Jacobian
		pbodies[i]->fJL.setM(SE3(pos[i]));
		pbodies[i]->fJL.setJointLoopConstraintType(JOINTLOOP_ORIENTATION_POSITION);
		pbodies[i]->fJL.update_J();

		// transformed Jacobian, Jg = [R 0; 0 R] * J
		RMatrix Jg(pbodies[i]->fJL.J.RowSize(), pbodies[i]->fJL.J.ColSize());
		RMatrix R = convert_to_RMatrix(pbodies[i]->T_global.GetRotation());
		Jg.Push(0, 0, R * pbodies[i]->fJL.J.Sub(0, 2, 0, pbodies[i]->fJL.J.ColSize()-1));
		Jg.Push(3, 0, R * pbodies[i]->fJL.J.Sub(3, 5, 0, pbodies[i]->fJL.J.ColSize()-1));

		// build J
		for (j=0, iter_pcoord = pbodies[i]->fJL.pCoordinates.begin(); iter_pcoord != pbodies[i]->fJL.pCoordinates.end(); j++, iter_pcoord++) {

			// find index of pbodies[i]->fJL.pCoordinates[j] in pCoordinates
			int idx = -1;
			for (k=0, iter_pcoord2 = pCoordinates.begin(); iter_pcoord2 != pCoordinates.end(); k++, iter_pcoord2++) {
				if ( *iter_pcoord2 == *iter_pcoord ) { idx = k; break; }
			}
			if ( idx < 0 ) return false;

			// insert j-th column of the body Jacobian to the right place
			for (k=0; k<int(idxC[i].size()); k++) {
				J(cnt+k, idx) = Jg(idxC[i][k], j);
			}
		}

		// build V
		for (j=0; j<int(idxC[i].size()); j++) {
			V(cnt+j, 0) = V_target[i][idxC[i][j]];
		}

		cnt += int(idxC[i].size());
	}

	return true;
}

bool GSystemIK::solveIK_dq(RMatrix &dq, vector<GBodyRigid*> pbodies_primary, vector<GBodyRigid*> pbodies_secondary, vector<Vec3> p_primary, vector<Vec3> p_secondary, vector<se3> V_primary, vector<se3> V_secondary, vector< vector<int> > idxC_primary, vector< vector<int> > idxC_secondary, double alpha_primary, double alpha_secondary)
{
	RMatrix Jp, Js;		// Jacobian matrices for primary/secondary constraints
	RMatrix Vp, Vs;		// the righthand side of the constraints

	if ( !buildConstrIK_dq(Jp, Vp, pbodies_primary, p_primary, V_primary, idxC_primary) ) return false;
	if ( !buildConstrIK_dq(Js, Vs, pbodies_secondary, p_secondary, V_secondary, idxC_secondary) ) return false;

	dq.SetZero(getNumCoordinates(), 1);
	if ( Jp.RowSize() > 0 ) {
		RMatrix dq0, N;
		dq0 = srInv(Jp, N, alpha_primary) * Vp;
		if ( Js.RowSize() > 0 ) {
			dq = dq0 + N * ( srInv(Js * N, alpha_secondary) * (Vs - Js * dq0) );
		} else {
			dq = dq0;
		}
	} else {
		if ( Js.RowSize() > 0 ) {
			dq = srInv(Js, alpha_secondary) * Vs;
		}
	}

	return true;
}

/*
bool GSystemIK::solveIK_dq(RMatrix &dq, vector<GBodyRigid*> pbodies_primary, vector<GBodyRigid*> pbodies_secondary, vector<Vec3> p_primary, vector<Vec3> p_secondary, vector<se3> V_primary, vector<se3> V_secondary, vector< vector<int> > idxC_primary, vector< vector<int> > idxC_secondary, vector<GCoordinate*> pcoords_disabled, double alpha)
{
	int idx;
	vector<GCoordinate*>::iterator viter_pcoord;
	list<GCoordinate*>::iterator iter_pcoord;
	RMatrix Jp, Js;		// Jacobian matrices for primary/secondary constraints
	RMatrix Vp, Vs;		// the righthand isde of the constraints

	if ( !buildConstrIK_dq(Jp, Vp, pbodies_primary, p_primary, V_primary, idxC_primary) ) return false;
	if ( !buildConstrIK_dq(Js, Vs, pbodies_secondary, p_secondary, V_secondary, idxC_secondary) ) return false;

	for (viter_pcoord = pcoords_disabled.begin(); viter_pcoord != pcoords_disabled.end(); viter_pcoord++) {
		for (idx = 0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); idx++, iter_pcoord++) {
			if ( *iter_pcoord == *viter_pcoord ) {
				for (int i=0; i<Jp.RowSize(); i++) { Jp(i,idx) = 0.0; }
				for (int i=0; i<Js.RowSize(); i++) { Js(i,idx) = 0.0; }
			}
		}
	}

	if ( Jp.RowSize() > 0 ) {
		RMatrix dq0, N;
		//dq0 = srInv(Jp, N, alpha) * Vp;
		dq0 = pInv(Jp, N) * Vp;
		dq = dq0 + N * ( srInv(Js * N, alpha) * (Vs - Js * dq0) );
	} else {
		dq = pInv(Js) * Vs;
	}

	return true;
}
*/

bool GSystemIK::solveIK_dq(RMatrix &dq, vector<GBodyRigid*> pbodies_primary, vector<GBodyRigid*> pbodies_secondary, vector<Vec3> p_primary, vector<Vec3> p_secondary, vector<se3> V_primary, vector<se3> V_secondary, vector< vector<int> > idxC_primary, vector< vector<int> > idxC_secondary, vector<GCoordinate*> pcoords_prescribed, double alpha_primary, double alpha_secondary)
{
	if ( pcoords_prescribed.size() == 0 ) {
		return solveIK_dq(dq, pbodies_primary, pbodies_secondary, p_primary, p_secondary, V_primary, V_secondary, idxC_primary, idxC_secondary, alpha_primary, alpha_secondary);
	}

	vector<GCoordinate*>::iterator viter_pcoord;
	list<GCoordinate*>::iterator iter_pcoord;
	RMatrix Jp, Js;		// Jacobian matrices for primary/secondary constraints
	RMatrix Vp, Vs;		// the righthand isde of the constraints

	if ( !buildConstrIK_dq(Jp, Vp, pbodies_primary, p_primary, V_primary, idxC_primary) ) return false;
	if ( !buildConstrIK_dq(Js, Vs, pbodies_secondary, p_secondary, V_secondary, idxC_secondary) ) return false;

	int num_prescribed = 0;
	vector<bool> b_prescribed;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		if ( find(pcoords_prescribed.begin(), pcoords_prescribed.end(), *iter_pcoord) == pcoords_prescribed.end() ) {
			b_prescribed.push_back(false);
		} else {
			b_prescribed.push_back(true);
			num_prescribed++;
		}
	}

	vector<GCoordinate*> pcoords_all(pCoordinates.begin(), pCoordinates.end());
	int nx = getNumCoordinates() - num_prescribed;
	RMatrix Jp_(Jp.RowSize(), nx), Js_(Js.RowSize(), nx);
	RMatrix Vp_(Vp), Vs_(Vs);

	int cnt = 0;
	for (int i=0; i<getNumCoordinates(); i++) {
		if ( b_prescribed[i] ) {
			Vp_ -= pcoords_all[i]->dq * Jp.Sub(0, Jp.RowSize()-1, i, i);
			Vs_ -= pcoords_all[i]->dq * Js.Sub(0, Js.RowSize()-1, i, i);
		} else {
			Jp_.Push(0, cnt, Jp.Sub(0, Jp.RowSize()-1, i, i));
			Js_.Push(0, cnt, Js.Sub(0, Js.RowSize()-1, i, i));
			cnt++;
		}
	}

	RMatrix x;
	x.SetZero(nx,1);
	if ( Jp_.RowSize() > 0 ) {
		RMatrix x0, N;
		x0 = srInv(Jp_, N, alpha_primary) * Vp_;
		if ( Js_.RowSize() > 0 ) {
			x = x0 + N * ( srInv(Js_ * N, alpha_secondary) * (Vs_ - Js_ * x0) );
		} else {
			x = x0;
		}
	} else {
		if ( Js_.RowSize() > 0 ) {
			x = srInv(Js_, alpha_secondary) * Vs_;
		}
	}

	dq.ReSize(getNumCoordinates(), 1);
	cnt = 0;
	for (int i=0; i<getNumCoordinates(); i++) {
		if ( b_prescribed[i] ) {
			dq[i] = pcoords_all[i]->dq;
		} else {
			dq[i] = x[cnt++];
		}
	}

	return true;
}

bool GSystemIK::solveIK_dq(RMatrix &dq, vector<GBodyRigid*> pbodies_primary, vector<GBodyRigid*> pbodies_secondary, vector<Vec3> p_primary, vector<Vec3> p_secondary, vector<se3> V_primary, vector<se3> V_secondary, vector< vector<int> > idxC_primary, vector< vector<int> > idxC_secondary, vector<GCoordinate*> pcoords_prescribed, ofstream *pfout, double alpha_primary, double alpha_secondary)
{
	if ( pcoords_prescribed.size() == 0 ) {
		return solveIK_dq(dq, pbodies_primary, pbodies_secondary, p_primary, p_secondary, V_primary, V_secondary, idxC_primary, idxC_secondary, alpha_primary, alpha_secondary);
	}

	vector<GCoordinate*>::iterator viter_pcoord;
	list<GCoordinate*>::iterator iter_pcoord;
	RMatrix Jp, Js;		// Jacobian matrices for primary/secondary constraints
	RMatrix Vp, Vs;		// the righthand isde of the constraints

	if ( !buildConstrIK_dq(Jp, Vp, pbodies_primary, p_primary, V_primary, idxC_primary) ) return false;
	if ( !buildConstrIK_dq(Js, Vs, pbodies_secondary, p_secondary, V_secondary, idxC_secondary) ) return false;

	int num_prescribed = 0;
	vector<bool> b_prescribed;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		if ( find(pcoords_prescribed.begin(), pcoords_prescribed.end(), *iter_pcoord) == pcoords_prescribed.end() ) {
			b_prescribed.push_back(false);
		} else {
			b_prescribed.push_back(true);
			num_prescribed++;
		}
	}

	vector<GCoordinate*> pcoords_all(pCoordinates.begin(), pCoordinates.end());
	int nx = getNumCoordinates() - num_prescribed;
	RMatrix Jp_(Jp.RowSize(), nx), Js_(Js.RowSize(), nx);
	RMatrix Vp_(Vp), Vs_(Vs);

	int cnt = 0;
	for (int i=0; i<getNumCoordinates(); i++) {
		if ( b_prescribed[i] ) {
			Vp_ -= pcoords_all[i]->dq * Jp.Sub(0, Jp.RowSize()-1, i, i);
			Vs_ -= pcoords_all[i]->dq * Js.Sub(0, Js.RowSize()-1, i, i);
		} else {
			Jp_.Push(0, cnt, Jp.Sub(0, Jp.RowSize()-1, i, i));
			Js_.Push(0, cnt, Js.Sub(0, Js.RowSize()-1, i, i));
			cnt++;
		}
	}

	RMatrix x;
	x.SetZero(nx,1);
	if ( Jp_.RowSize() > 0 ) {
		RMatrix x0, N;
		x0 = srInv(Jp_, N, alpha_primary) * Vp_;
		if ( Js_.RowSize() > 0 ) {
			x = x0 + N * ( srInv(Js_ * N, alpha_secondary) * (Vs_ - Js_ * x0) );
		} else {
			x = x0;
		}
	} else {
		if ( Js_.RowSize() > 0 ) {
			x = srInv(Js_, alpha_secondary) * Vs_;
		}
	}

	dq.ReSize(getNumCoordinates(), 1);
	cnt = 0;
	for (int i=0; i<getNumCoordinates(); i++) {
		if ( b_prescribed[i] ) {
			dq[i] = pcoords_all[i]->dq;
		} else {
			dq[i] = x[cnt++];
		}
	}

	if ( pfout != NULL ) {
		*pfout << "x = " << x << "Jp_ = " << Jp_ << "Js_ = " << Js_ << "Vp_ = " << Vp_ << "Vs_ = " << Vs_;
		*pfout << "Jp = " << Jp << "Js = " << Js << "Vp = " << Vp << "Vs = " << Vs;
		*pfout << "dq_prescribed = (";
		for (int i=0; i<pcoords_prescribed.size(); i++) {
			*pfout << pcoords_prescribed[i]->dq << ", ";
		}
		*pfout << endl;
	}

	return true;
}
