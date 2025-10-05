#define SWAP_FORCE_AND_TORQUE


inline TRL::Link* getTRLlink(TRL::Body* cinfo, int hrpjointindex)
{
	return cinfo->link(hrpjointindex);
}


inline quater toBase(matrix33 const& m)
{
	quater q;
	q.setRotation(m);
	return q;
}

inline matrix3 toOpenHRP(quater const& q )
{
	matrix3 out;
	out.setRotation(q);
	return out;
}

