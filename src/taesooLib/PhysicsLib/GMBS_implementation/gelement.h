//================================================================================
//         BASIC ELEMENTS FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_ELEMENT_
#define _GMBS_GEOMETRIC_ELEMENT_

#include <string>
#include <sstream>
#include <list>
#include <vector>
#include "rmatrix3j.h"


class GCoordinate;

//=============================================================
//                 GElement
//=============================================================
class GElement
{
public:
	std::string name;						// name
	int id;									// id

	std::list<GCoordinate *> pCoordinates;	// pointer to coordinates
	
public:
	GElement() : id(0), name("") {}
	~GElement() {}

public:
	virtual void initialize();

	void setName(std::string name_) { name = name_; }
	void setID(int id_) { id = id_; }

	std::string getName() { return name; }
	int getID() { return id; }

	int getNumCoordinates() { return int(pCoordinates.size()); }

	std::vector<GCoordinate*> getPrescribedCoordinates();
	std::vector<GCoordinate*> getUnprescribedCoordinates();

	std::vector<int> getIndexOfPrescribedCoordinates();
	std::vector<int> getIndexOfUnprescribedCoordinates();
	
 	virtual	int getIndexOfCoordinate(GCoordinate *pcoord_); // taesoo : gsystem class override this function to speed up the search.

	void set_q(const double *x_);
	void set_dq(const double *x_);
	void set_ddq(const double *x_);
	void set_tau(const double *x_);

	void set_DqDp(const double *x_);
	void set_DdqDp(const double *x_);
	void set_DddqDp(const double *x_);
	void set_DtauDp(const double *x_);

	void set_qLL(const double *x_);
	void set_dqLL(const double *x_);
	void set_ddqLL(const double *x_);
	void set_tauLL(const double *x_);

	void set_qUL(const double *x_);
	void set_dqUL(const double *x_);
	void set_ddqUL(const double *x_);
	void set_tauUL(const double *x_);

	bool set_q(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_q(in_.GetPtr()); return true; }
	bool set_dq(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_dq(in_.GetPtr()); return true; }
	bool set_ddq(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_ddq(in_.GetPtr()); return true; }
	bool set_tau(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_tau(in_.GetPtr()); return true; }

	bool set_DqDp(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_DqDp(in_.GetPtr()); return true; }
	bool set_DdqDp(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_DdqDp(in_.GetPtr()); return true; }
	bool set_DddqDp(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_DddqDp(in_.GetPtr()); return true; }
	bool set_DtauDp(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_DtauDp(in_.GetPtr()); return true; }

	bool set_qLL(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_qLL(in_.GetPtr()); return true; }
	bool set_dqLL(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_dqLL(in_.GetPtr()); return true; }
	bool set_ddqLL(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_ddqLL(in_.GetPtr()); return true; }
	bool set_tauLL(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_tauLL(in_.GetPtr()); return true; }

	bool set_qUL(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_qUL(in_.GetPtr()); return true; }
	bool set_dqUL(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_dqUL(in_.GetPtr()); return true; }
	bool set_ddqUL(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_ddqUL(in_.GetPtr()); return true; }
	bool set_tauUL(RMatrix const &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_tauUL(in_.GetPtr()); return true; }

	void set_q(double x_) { set_q(x_*Ones(getNumCoordinates(),1)); }
	void set_dq(double x_) { set_dq(x_*Ones(getNumCoordinates(),1)); }
	void set_ddq(double x_) { set_ddq(x_*Ones(getNumCoordinates(),1)); }
	void set_tau(double x_) { set_tau(x_*Ones(getNumCoordinates(),1)); }

	void set_DqDp(double x_) { set_DqDp(x_*Ones(getNumCoordinates(),1)); }
	void set_DdqDp(double x_) { set_DdqDp(x_*Ones(getNumCoordinates(),1)); }
	void set_DddqDp(double x_) { set_DddqDp(x_*Ones(getNumCoordinates(),1)); }
	void set_DtauDp(double x_) { set_DtauDp(x_*Ones(getNumCoordinates(),1)); }

	void set_qLL(double x_) { set_qLL(x_*Ones(getNumCoordinates(),1)); }
	void set_dqLL(double x_) { set_dqLL(x_*Ones(getNumCoordinates(),1)); }
	void set_ddqLL(double x_) { set_ddqLL(x_*Ones(getNumCoordinates(),1)); }
	void set_tauLL(double x_) { set_tauLL(x_*Ones(getNumCoordinates(),1)); }

	void set_qUL(double x_) { set_qUL(x_*Ones(getNumCoordinates(),1)); }
	void set_dqUL(double x_) { set_dqUL(x_*Ones(getNumCoordinates(),1)); }
	void set_ddqUL(double x_) { set_ddqUL(x_*Ones(getNumCoordinates(),1)); }
	void set_tauUL(double x_) { set_tauUL(x_*Ones(getNumCoordinates(),1)); }

	void get_q(double *x_);
	void get_dq(double *x_);
	void get_ddq(double *x_);
	void get_tau(double *x_);

	void get_DqDp(double *x_);
	void get_DdqDp(double *x_);
	void get_DddqDp(double *x_);
	void get_DtauDp(double *x_);

	void get_qLL(double *x_);
	void get_dqLL(double *x_);
	void get_ddqLL(double *x_);
	void get_tauLL(double *x_);

	void get_qUL(double *x_);
	void get_dqUL(double *x_);
	void get_ddqUL(double *x_);
	void get_tauUL(double *x_);

	RMatrix get_q() { RMatrix re(getNumCoordinates(), 1); get_q(re.GetPtr()); return re; }
	RMatrix get_dq() { RMatrix re(getNumCoordinates(), 1); get_dq(re.GetPtr()); return re; }
	RMatrix get_ddq() { RMatrix re(getNumCoordinates(), 1); get_ddq(re.GetPtr()); return re; }
	RMatrix get_tau() { RMatrix re(getNumCoordinates(), 1); get_tau(re.GetPtr()); return re; }

	RMatrix get_DqDp() { RMatrix re(getNumCoordinates(), 1); get_DqDp(re.GetPtr()); return re; }
	RMatrix get_DdqDp() { RMatrix re(getNumCoordinates(), 1); get_DdqDp(re.GetPtr()); return re; }
	RMatrix get_DddqDp() { RMatrix re(getNumCoordinates(), 1); get_DddqDp(re.GetPtr()); return re; }
	RMatrix get_DtauDp() { RMatrix re(getNumCoordinates(), 1); get_DtauDp(re.GetPtr()); return re; }

	RMatrix get_qLL() { RMatrix re(getNumCoordinates(), 1); get_qLL(re.GetPtr()); return re; }
	RMatrix get_dqLL() { RMatrix re(getNumCoordinates(), 1); get_dqLL(re.GetPtr()); return re; }
	RMatrix get_ddqLL() { RMatrix re(getNumCoordinates(), 1); get_ddqLL(re.GetPtr()); return re; }
	RMatrix get_tauLL() { RMatrix re(getNumCoordinates(), 1); get_tauLL(re.GetPtr()); return re; }

	RMatrix get_qUL() { RMatrix re(getNumCoordinates(), 1); get_qUL(re.GetPtr()); return re; }
	RMatrix get_dqUL() { RMatrix re(getNumCoordinates(), 1); get_dqUL(re.GetPtr()); return re; }
	RMatrix get_ddqUL() { RMatrix re(getNumCoordinates(), 1); get_ddqUL(re.GetPtr()); return re; }
	RMatrix get_tauUL() { RMatrix re(getNumCoordinates(), 1); get_tauUL(re.GetPtr()); return re; }

	RMatrix get_q(std::vector<int> idx_);
	RMatrix get_dq(std::vector<int> idx_);
	RMatrix get_ddq(std::vector<int> idx_);
	RMatrix get_tau(std::vector<int> idx_);

public:
	virtual bool getReady() { return true; }
	virtual std::string getInfoStr();
	virtual void render() {}
};


#endif

