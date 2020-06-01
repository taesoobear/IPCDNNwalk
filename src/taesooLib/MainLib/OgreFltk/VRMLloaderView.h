#ifndef VRMLLOADER_VIEW_H_
#define VRMLLOADER_VIEW_H_

#pragma once
#include "VRMLloader.h"
#include "../../BaseLib/math/optimize.h"
class VRMLloaderView: public VRMLloader
{
	public:
	VRMLloaderView(VRMLloader const& source, Bone& newRootBone, vector3 const& localPos); // newRootBone in the source skeleton will become the new root.

	IndexMapping _treeIndex2sourceTree;	//!< m_aTreeIndex2sourceTree[tree index]=source joint index
	IndexMapping _DOFIndex2sourceTree; //!< m_aTreeIndex2sourceTree[tree index]=source joint index
	vectorn _conversionSign; // indexed by DOFIndex. reverted: -1, otherwise: 1
	VRMLloader const& _sourceSkel;
	Bone* _newRootBone; // the bone in the source skeleton that corresponds to the root in the new skeleton.
	Bone* _srcRootTransf; // the bone in the new skeleton that corresponds to the root in the source skeleton.
	vector3 _srcRootOffset;

	VRMLloader const & getSourceSkel() {return _sourceSkel;}
	void convertSourcePose(vectorn const& srcPose, vectorn& pose) const;
	void convertPose(vectorn const& pose, vectorn& srcPose) const;

	// much faster. This ignores the root joint (ball joints).
	void convertSourceDOFexceptRoot(vectorn const& srcPose, vectorn& pose) const;
	void convertDOFexceptRoot(vectorn const& pose, vectorn& srcPose) const;
	// uses DQ index instead of DOF index (see class MotionDOFinfo).
	void convertSourceDQexceptRoot(vectorn const& src_dq, vectorn& dq) const;
	void convertDQexceptRoot(vectorn const& dq, vectorn& src_dq) const;
};


#endif
