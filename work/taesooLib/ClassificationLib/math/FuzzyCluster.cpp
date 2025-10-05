
#include "stdafx.h"
#include "../BaseLib/math/mathclass.h"
#include "FuzzyCluster.h"
#include <deque>
#include <float.h>
#include "../BaseLib/utility/TArray.h"
#include "../BaseLib/utility/util.h"
#include <vector>
#include <list>
#include "../BaseLib/image/ImageProcessor.h"
#include "PCA.H"
#include "../BaseLib/math/Operator.h"


CFuzzyCluster::CFuzzyCluster(void)
{
}

CFuzzyCluster::~CFuzzyCluster(void)
{
}

m_real CFuzzyCluster::min_value(vectorn& x)
{
	m_real min_val = FLT_MAX;

	for (int i = 0; i < x.size(); i++) {
		if (x[i] < min_val) {
			min_val = x[i];
		}
	}

	return min_val;
}

m_real CFuzzyCluster::max_value(vectorn& x)
{
	m_real max_val = INT_MIN;

	for (int i = 0; i < x.size(); i++) {
		if (x[i] > max_val) {
			max_val = x[i];
		}
	}

	return max_val;
}

m_real CFuzzyCluster::min_value(m_real* x, int n, int& index)
{
	m_real min_val = FLT_MAX;

	for (int i = 0; i < n; i++) {
		if (x[i] < min_val) {
			min_val = x[i];
			index = i;
		}
	}

	return min_val;
}

m_real CFuzzyCluster::max_value(m_real* x, int n, int& index)
{
	m_real max_val = INT_MIN;

	for (int i = 0; i < n; i++) {
		if (x[i] > max_val) {
			max_val = x[i];
			index = i;
		}
	}

	return max_val;
}

void CFuzzyCluster::min_value(vectorn* x, int n, vectorn& minX)
{
	minX.setSize(x[0].size());

	for (int i = 0; i < x[0].size(); i++) {
		m_real min_comp = FLT_MAX;
		for (int j = 0; j < n; j++) {
			if (x[j].getValue(i) < min_comp) {
				min_comp = x[j].getValue(i);
			}
		}

		minX.setValue(i, min_comp);
	}
}

void CFuzzyCluster::max_value(vectorn* x, int n, vectorn& maxX)
{
	maxX.setSize(x[0].size());

	for (int i = 0; i < x[0].size(); i++) {
		m_real max_comp = INT_MIN;
		for (int j = 0; j < n; j++) {
			if (x[j].getValue(i) > max_comp) {
				max_comp = x[j].getValue(i);
			}
		}

		maxX.setValue(i, max_comp);
	}
}

void CFuzzyCluster::Subtractive(matrixn& x, int n, std::deque<vectorn>& centers)
{
	m_real *radii = new m_real[x.row(0).size()];

	for (int i = 0; i < x.cols(); i++) {
//		radii[i] = 1.5;		// for free-style dance
//		radii[i] = 3.0;		// for waltz	2002. 1. 17.
//		radii[i] = 1.6;		// for Backdance.. optimal path of length 20  (2002. 4. 8.)
//		radii[i] = 2.0;		// for free-style.. optimal path of length 20  (2002. 4. 8.)
//		radii[i] = 2.0;		// for BackDance010
		radii[i] = 3.0;		// for BackDance010
	}

	Subtractive(x, n, radii, centers);

	delete radii;
}

void CFuzzyCluster::Subtractive(matrixn& x, int n, m_real radii, std::deque<vectorn>& centers)
{
	m_real *radiiList = new m_real[x.row(0).size()];

	for (int i = 0; i < x.cols(); i++) {
		radiiList[i] = radii;
	}

	Subtractive(x, n, radiiList, centers);

	delete[] radiiList;
}

void CFuzzyCluster::Subtractive2(vectorn* x, int n, m_real radii, std::deque<vectorn>& centers)
{
	m_real *radiiList = new m_real[x[0].size()];

	for (int i = 0; i < x[0].size(); i++) {
		radiiList[i] = radii;
	}

	Subtractive2(x, n, radiiList, centers);

	delete[] radiiList;
}

void CFuzzyCluster::Subtractive(matrixn& x, int n, m_real* radii, std::deque<vectorn>& centers)
{
	ASSERT(n==x.rows());

	const m_real sqshFactor = 1.25f;
	const m_real acceptRatio = 0.5f;
	const m_real rejectRatio = 0.15f;
	const int numParams = x.cols();

	//
	vectorn accumMultp(numParams);
	vectorn sqshMultp(numParams);

	for (int i = 0; i < numParams; i++) {
		accumMultp[i] = 1.0 / radii[i];
		sqshMultp[i] = 1.0 / (sqshFactor * radii[i]);
	}

	//
	vectorn minX;
	minX.minimum(x);
	vectorn maxX;
	maxX.maximum(x);

/*	TRACE("minX = ");
	for (i = 0; i < numParams; i++) {
		TRACE("%lf ", minX[i]);
	}
	TRACE("\n");
	TRACE("maxX = ");
	for (i = 0; i < numParams; i++) {
		TRACE("%lf ", maxX[i]);
	}
	TRACE("\n");
*/
	for (int i = 0; i < numParams; i++) {
		if (minX[i] == maxX[i]) {
			minX[i] = minX[i] - 0.0001 * (1 + ABS(minX[i]));
			maxX[i] = maxX[i] + 0.0001 * (1 + ABS(maxX[i]));
		}
	}
/*
	TRACE("minX = ");
	for (i = 0; i < numParams; i++) {
		TRACE("%lf ", minX[i]);
	}
	TRACE("\n");
	TRACE("maxX = ");
	for (i = 0; i < numParams; i++) {
		TRACE("%lf ", maxX[i]);
	}
	TRACE("\n");
*/
	// normalize the data into a unit hyperbox using the verified minX and maxX
	m_real normalized_val;

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < numParams; j++) {
			normalized_val = (x[i][j] - minX[j]) / (maxX[j] - minX[j]);
			x[i][j] = normalized_val;
		}
	}

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < numParams; j++) {
			normalized_val = MIN(MAX(x[i][j], 0), 1);
			x[i][j] = normalized_val;
		}
	}
/*
	TRACE("X = ");
	for (i = 0; i < n; i++) {
		for (int j = 0; j < numParams; j++) {
			TRACE("%lf ", x[i][j]);
		}
		TRACE("\n");
	}
	TRACE("\n");
*/
	// compute the initial potentials for each data point
	m_real *potVals = new m_real [n];
	memset(potVals, 0, n * sizeof(m_real));

	vectorn nextPoint(numParams);
	vectorn dx(numParams);
	vectorn thePoint(numParams);
	for (int i = 0; i < n; i++) {
		thePoint= x.row(i);
		potVals[i] = potVals[i] + 1.0;

		for (int j = i + 1; j < n; j++) {
			nextPoint = x.row(j);
			for (int k = 0; k < numParams; k++) {
				dx[k] = (thePoint[k] - nextPoint[k]) * accumMultp[k];
			}
			m_real dxSq = SQR(dx.length());
			m_real mu = exp(-4.0 * dxSq);

			potVals[i] = potVals[i] + mu;
			potVals[j] = potVals[j] + mu;
		}
	}

	// find the data point with highest potential value
	// refPotVal : the highest potential value,
	//             used as a reference for accepting/rejecting other data points as cluster centers
	int maxPotIndex;
	m_real refPotVal = max_value(potVals, n, maxPotIndex);
	m_real maxPotVal = refPotVal;

	if (centers.size() > 0) {
		centers.clear();
	}

	int numClusters = 0;
	int findMore = 1;

	vectorn maxPoint(numParams);

	while (findMore && maxPotVal != 0) {
		findMore = 0;
		maxPoint = x.row(maxPotIndex);
		m_real maxPotRatio = maxPotVal / refPotVal;

		if (maxPotRatio > acceptRatio) {
			// the new peak value is significant, accept it
			findMore = 1;
		}
		else {
			if (maxPotRatio > rejectRatio) {
				// accept this data point only if it achieves a good balance between having
				// a reasonable potential and being far from all existing cluster centers
				m_real minDistSq = -1;

				for (int i = 0; i < numClusters; i++) {
					m_real dxSq = 0;

					for (int k = 0; k < numParams; k++) {
						dx[k] = (maxPoint[k] - centers[i][k]) * accumMultp[k];
						dxSq += SQR(dx[k]);
					}

					if (minDistSq < 0 || dxSq < minDistSq) {
						minDistSq = dxSq;
					}
				}

				m_real minDist = sqrt(minDistSq);
				if (maxPotRatio + minDist >= 1) {
					// tentatively accept this data point as a cluster center
					findMore = 1;
				}
				else {
					// remove this point from further consideration, and continue
					findMore = 2;
				}
			}
		}

		if (findMore == 1) {
			// add the data point to the list of cluster centers
			centers.push_back(maxPoint);
			numClusters++;

			// subtract potential from data points near the new cluster center
			for (int i = 0; i < n; i++) {
				nextPoint = x.row(i);
				m_real potVal = potVals[i];

				m_real dxSq = 0;

				for (int k = 0; k < numParams; k++) {
					dx[k] = (maxPoint[k] - nextPoint[k]) * sqshMultp[k];
					dxSq += SQR(dx[k]);
				}

				potVal = potVal - (maxPotVal * exp(-4.0 * dxSq));
				if (potVal < 0) {
					potVal = 0;
				}

				potVals[i] = potVal;
			}

			// find the data point with the highest remaining potential
			maxPotVal = max_value(potVals, n, maxPotIndex);
		}
		else {
			if (findMore == 2) {
				potVals[maxPotIndex] = 0;
				maxPotVal = max_value(potVals, n, maxPotIndex);
			}
		}
	}

	// scale the cluster centers from the normalized values
	// back to values in the original range
	for (int j = 0; j < numClusters; j++) {
		for (int i = 0; i < numParams; i++) {
			centers[j][i] = centers[j][i] * (maxX[i] - minX[i]) + minX[i];
		}
	}

	// compute the sigma values for the clusters
	m_real *sigmas = new m_real [numParams];

	for (int i = 0; i < numParams; i++) {
		sigmas[i] = (radii[i] * (maxX[i] - minX[i])) / sqrt(8.0);
	}

	// centers for return value
	vectorn *ret_center = new vectorn [numClusters];

	for (int i = 0; i < numClusters; i++) {
		ret_center[i] = centers[i];
	}

	//
	delete[] sigmas;
	delete[] potVals;
}

m_real CFuzzyCluster::Distance(vectorn &v1, vectorn &v2, vectorn &factor)
{
	int numParams = v1.size();
	vectorn dx(numParams);
	m_real dxSq = 0;
	int conSize = (int)v1[numParams - 1];
	char str1[256];
	char str2[256];

	for (int i = 0; i < conSize * 2; i++) {
		str1[i] = (char)v1[numParams - (conSize * 2 + 1) + i];
		str2[i] = (char)v2[numParams - (conSize * 2 + 1) + i];
	}
	str1[conSize * 2 - 2] = str2[conSize * 2 - 2] = 0;
/*
	if (str1[conSize * 2 - 1] != str2[conSize * 2 - 1]) {
		return INT_MAX;
	}
	for (int i = 0; i < conSize * 2 - 2; i++) {
		if ((str1[i] == 'L' && str2[i] == 'R') || (str1[i] == 'R' && str2[i] == 'L')) {
			return INT_MAX;
		}
	}*/

	if (strcmp(str1, str2)) {
		return FLT_MAX;
	}

	numParams -= conSize * 2 + 1;
	for (int k = 0; k < numParams; k++) {
		dx[k] = (v1[k] - v2[k]) * factor[k];
	}
	dxSq = SQR(dx.length());

	return dxSq;
}

void CFuzzyCluster::Subtractive2(vectorn* x, int n, m_real* radii, std::deque<vectorn>& centers)
{
	const m_real sqshFactor = 1.25f;
	const m_real acceptRatio = 0.5f;
	const m_real rejectRatio = 0.15f;
	int numParams = x[0].size();
	numParams -= (int)x[0][numParams - 1] * 2 + 1;

	//
	vectorn accumMultp(numParams);
	vectorn sqshMultp(numParams);

	for (int i = 0; i < numParams; i++) {
		accumMultp[i] = 1.0 / radii[i];
		sqshMultp[i] = 1.0 / (sqshFactor * radii[i]);
	}

	//
	vectorn minX;
	min_value(x, n, minX);
	vectorn maxX;
	max_value(x, n, maxX);

	for (int i = 0; i < numParams; i++) {
		if (minX[i] == maxX[i]) {
			minX[i] = minX[i] - 0.0001 * (1 + ABS(minX[i]));
			maxX[i] = maxX[i] + 0.0001 * (1 + ABS(maxX[i]));
		}
	}

	// normalize the data into a unit hyperbox using the verified minX and maxX
	m_real normalized_val;

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < numParams; j++) {
			normalized_val = (x[i][j] - minX[j]) / (maxX[j] - minX[j]);
			x[i][j] = normalized_val;
		}
	}

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < numParams; j++) {
			normalized_val = MIN(MAX(x[i][j], 0), 1);
			x[i][j] = normalized_val;
		}
	}

	// compute the initial potentials for each data point
	m_real *potVals = new m_real [n];
	memset(potVals, 0, n * sizeof(m_real));

	vectorn nextPoint;
	vectorn dx(numParams);

	for (int i = 0; i < n; i++) {
		vectorn thePoint = x[i];
		potVals[i] = potVals[i] + 1.0;

		for (int j = i + 1; j < n; j++) {
			nextPoint = x[j];
/*			for (int k = 0; k < numParams; k++) {
				dx[k] = (thePoint[k] - nextPoint[k]) * accumMultp[k];
			}
			m_real dxSq = SQR(dx.length());*/
			m_real dxSq = Distance(thePoint, nextPoint, accumMultp);
			m_real mu = exp(-4.0 * dxSq);

			potVals[i] = potVals[i] + mu;
			potVals[j] = potVals[j] + mu;
		}
	}

	// find the data point with highest potential value
	// refPotVal : the highest potential value,
	//             used as a reference for accepting/rejecting other data points as cluster centers
	int maxPotIndex;
	m_real refPotVal = max_value(potVals, n, maxPotIndex);
	m_real maxPotVal = refPotVal;

	if (centers.size() > 0) {
		centers.clear();
	}

	int numClusters = 0;
	int findMore = 1;

	while (findMore && maxPotVal != 0) {
		findMore = 0;
		vectorn maxPoint = x[maxPotIndex];
		m_real maxPotRatio = maxPotVal / refPotVal;

		if (maxPotRatio > acceptRatio) {
			// the new peak value is significant, accept it
			findMore = 1;
		}
		else {
			if (maxPotRatio > rejectRatio) {
				// accept this data point only if it achieves a good balance between having
				// a reasonable potential and being far from all existing cluster centers
				m_real minDistSq = -1;

				for (int i = 0; i < numClusters; i++) {
					m_real dxSq = 0;
					vectorn dx(numParams);
/*					for (int k = 0; k < numParams; k++) {
						dx[k] = (maxPoint[k] - centers[i][k]) * accumMultp[k];
						dxSq += SQR(dx[k]);
					}*/
					dxSq = Distance(maxPoint, centers[i], accumMultp);

					if (minDistSq < 0 || dxSq < minDistSq) {
						minDistSq = dxSq;
					}
				}

				m_real minDist = sqrt(minDistSq);
				if (maxPotRatio + minDist >= 1) {
					// tentatively accept this data point as a cluster center
					findMore = 1;
				}
				else {
					// remove this point from further consideration, and continue
					findMore = 2;
				}
			}
		}

		if (findMore == 1) {
			// add the data point to the list of cluster centers
			centers.push_back(maxPoint);
			numClusters++;

			// subtract potential from data points near the new cluster center
			for (int i = 0; i < n; i++) {
				vectorn nextPoint = x[i];
				m_real potVal = potVals[i];

				m_real dxSq = 0;
				vectorn dx(numParams);
/*				for (int k = 0; k < numParams; k++) {
					dx[k] = (maxPoint[k] - nextPoint[k]) * sqshMultp[k];
					dxSq += SQR(dx[k]);
				}*/
				dxSq = Distance(maxPoint, nextPoint, sqshMultp);

				potVal = potVal - (maxPotVal * exp(-4.0 * dxSq));
				if (potVal < 0) {
					potVal = 0;
				}

				potVals[i] = potVal;
			}

			// find the data point with the highest remaining potential
			maxPotVal = max_value(potVals, n, maxPotIndex);
		}
		else {
			if (findMore == 2) {
				potVals[maxPotIndex] = 0;
				maxPotVal = max_value(potVals, n, maxPotIndex);
			}
		}
	}

	// scale the cluster centers from the normalized values
	// back to values in the original range
	for (int j = 0; j < numClusters; j++) {
		for (int i = 0; i < numParams; i++) {
			centers[j][i] = centers[j][i] * (maxX[i] - minX[i]) + minX[i];
		}
	}

	// compute the sigma values for the clusters
	m_real *sigmas = new m_real [numParams];

	for (int i = 0; i < numParams; i++) {
		sigmas[i] = (radii[i] * (maxX[i] - minX[i])) / sqrt(8.0);
	}

	// centers for return value
	vectorn *ret_center = new vectorn [numClusters];

	for (int i = 0; i < numClusters; i++) {
		ret_center[i] = centers[i];
	}

	//
	delete sigmas;
	delete potVals;
}

void CFuzzyCluster::InitFcm(matrixn& x, vectorn* U, int cluster_n, int expo, std::deque<vectorn>& centers)
{
	int n = U[0].size();

	// fill the distance matrix
	vectorn *dist = new vectorn [cluster_n];
	vectorn tempV;

	for (int i = 0; i < cluster_n; i++) {
		dist[i].setSize(n);

		for (int j = 0; j < n; j++) {
			tempV = x.row(j);
			tempV -= centers[i];
			dist[i][j] = tempV.length();
			if (dist[i][j] == 0) {
				dist[i][j] = 0.0000001f;
			}
		}
	}

	// calculate new U
	assert(expo != 1);

	vectorn *tmp = new vectorn [cluster_n];
	m_real sum_tmp = 0;

	for (int i = 0; i < cluster_n; i++) {
		tmp[i].setSize(n);
		for (int j = 0; j < n; j++) {
			tmp[i][j] = pow(dist[i][j], -2 / (expo - 1));
			sum_tmp += tmp[i][j];
		}
	}

	for (int i = 0; i < cluster_n; i++) {
		for (int j = 0; j < n; j++) {
			U[i][j] = tmp[i][j] / sum_tmp;
		}
	}
	delete[] dist;
	delete[] tmp;
}

// return value : value of objective function
m_real CFuzzyCluster::StepFcm(matrixn& x, vectorn* U, int cluster_n, int expo, std::deque<vectorn>& centers)
{
	int n = U[0].size();

	// MF matrix after exponential modification
	vectorn *mf = new vectorn [cluster_n];

	for (int i = 0; i < cluster_n; i++) {
		mf[i].setSize(n);

		for (int j = 0; j < n; j++) {
			mf[i][j] = pow(U[i][j], expo);
		}
	}

	// new center
	vectorn v(x.cols());
	vectorn tempV;

	for (int i = 0; i < cluster_n; i++) {
		m_real div = 0;

		for (int k = 0; k < v.size(); k++) {
			v[k] = 0;
		}

		for (int k = 0; k < n; k++) {
			tempV = x.row(k);
			tempV *= mf[i][k];
			v += tempV;
			div += mf[i][k];
		}
		v /= div;

		centers[i] = v;
	}

	// fill the distance matrix
	vectorn *dist = new vectorn [cluster_n];

	for (int i = 0; i < cluster_n; i++) {
		dist[i].setSize(n);

		for (int j = 0; j < n; j++) {
			tempV = x.row(j);
			tempV -= centers[i];
			dist[i][j] = tempV.length();
			if (dist[i][j] == 0) {
				dist[i][j] = 0.0000001f;
			}
		}
	}

	// objective function
	m_real obj_fcn = 0;
	for (int i = 0; i < cluster_n; i++) {
		for (int j = 0; j < n; j++) {
			obj_fcn += dist[i][j] * mf[i][j];
		}
	}

	// calculate new U
	assert(expo != 1);

	vectorn *tmp = new vectorn [cluster_n];
	m_real sum_tmp = 0;

	for (int i = 0; i < cluster_n; i++) {
		tmp[i].setSize(n);
		for (int j = 0; j < n; j++) {
			tmp[i][j] = pow(dist[i][j], -2 / (expo - 1));
			sum_tmp += tmp[i][j];
		}
	}

	for (int i = 0; i < cluster_n; i++) {
		for (int j = 0; j < n; j++) {
			U[i][j] = tmp[i][j] / sum_tmp;
		}
	}

	delete [] mf;
	delete [] tmp;
	delete [] dist;


	return obj_fcn;
}

void CFuzzyCluster::Fcm(matrixn& x, int n, int cluster_n, std::deque<vectorn>& centers, int* group_index)
{
	const int max_iter = 100;
	const m_real min_impro = 0.00001f;
	const int expo = 2;

	m_real last_objFcn_val;
	m_real cur_objFcn_val;


	// initialize
	vectorn *U = new vectorn [cluster_n];
	for (int i = 0; i < cluster_n; i++) {
		U[i].setSize(n);

		for (int j = 0; j < n; j++) {
			U[i][j] = (m_real)rand() / (m_real)RAND_MAX;
		}
	}

	m_real col_sum;
	for (int i = 0; i < n; i++) {
		col_sum = 0;
		for (int j = 0; j < cluster_n; j++) {
			col_sum += U[j][i];
		}
		for (int j = 0; j < cluster_n; j++) {
			U[j][i] /= col_sum;
		}
	}
	//

	// FCM
	InitFcm(x, U, cluster_n, expo, centers);

	for (int i = 0; i < max_iter; i++) {
		cur_objFcn_val = StepFcm(x, U, cluster_n, expo, centers);

		if (i > 0 && ABS(cur_objFcn_val - last_objFcn_val) < min_impro) {
			break;
		}

		last_objFcn_val = cur_objFcn_val;
	}

	for (int i = 0; i < n; i++) {
		m_real max_u = INT_MIN;
		int max_index = INT_MIN;

		for (int j = 0; j < cluster_n; j++) {
			if (U[j][i] > max_u) {
				max_u = U[j][i];
				max_index = j;
			}
		}

		assert(max_index != INT_MIN);
		//
		group_index[i] = max_index;
	}

	delete[] U;
}

void CFuzzyCluster::Fcm(matrixn& x, int n, std::deque<vectorn>& centers, int* group_index)
{
//	TRACE("CLUSTER DEBUG\n");

	matrixn tempX;
	tempX.assign(x);

	Subtractive(tempX, n, centers);

//	TRACE("CENTERS\n");
	for (int i = 0; i < centers.size(); i++) {
//		for (int j = 0; j < centers[i].size(); j++) {
//			TRACE("%lf ", centers[i][j]);
//		}
//		TRACE("\n");
	}

	Fcm(x, n, centers.size(), centers, group_index);
}

void CFuzzyCluster::FcmRadii(matrixn& x, int n, m_real radii, std::deque<vectorn>& centers, int* group_index)
{
	matrixn tempX(x);

	Subtractive(tempX, n, radii, centers);

	Fcm(x, n, centers.size(), centers, group_index);
}


/************************************************************
	k-means clustering for 1d
	In	: data, input, num_data, num_input, k
	Out	: group, result
	data		: data points
	input		: initial guess
	group		: cluster index of each data points
	n			: number of data points
	num_input	: number of initial guessing points
	k			: number of clusters should be more than 0
	result		: center of each cluster
*************************************************************/

int compareFloat( const void *arg1, const void *arg2 )
{
	m_real* p1=(m_real*)arg1;
	m_real* p2=(m_real*)arg2;

	m_real diff=*p1-*p2;

	if(diff<0) return -1;
	else if(diff>0) return 1;
	else return 0;
}

void CFuzzyCluster::Subtractive(const matrixn& inputvectors, m_real radii, std::deque<vectorn>& centers)
{
	matrixn temp(inputvectors);
	CFuzzyCluster::Subtractive(temp, inputvectors.rows(), radii, centers);
}

void MakeGroupIndex(CTArray<intVector>& aVectorGroup, int* pGroupIndex)
{
	for(int i=0; i<aVectorGroup.Size(); i++)
	{
		for(int j=0; j<aVectorGroup[i].size(); j++)
		{
			pGroupIndex[aVectorGroup[i][j]]=i;
		}
	}
}

bool CFuzzyCluster::CheckNonEmpty(const matrixn& aInputVec, int &numCluster, matrixn& aCenter, int *pGroupIndex)
{
	bool bChanged=false;
	// Empty클러스터가 있으면 그거 없앤다.
	CTArray<intVector> aVectorGroup;
	aVectorGroup.Init(numCluster);
	for(int i=0; i<aInputVec.rows(); i++)
		aVectorGroup[pGroupIndex[i]].push_back(i);

	matrixn aCenterCpy;
	for(int i=0; i<numCluster; i++)
	{
		if(aVectorGroup[i].size() ==0)
		{
			// delete empty class
			aVectorGroup.Swap(i,numCluster-1);
			aCenter.setRow(i, aCenter.row(numCluster-1));
			aVectorGroup.Resize(numCluster-1);
			aCenter.setSize(numCluster-1, aCenter.cols());
			MakeGroupIndex(aVectorGroup, pGroupIndex);
			numCluster--;
			i--;
			bChanged=true;
		}
	}
	return bChanged;
}

void CFuzzyCluster::KMeanCluster(const matrixn& inputvectors, int cluster_n, matrixn& centers, int *group_index, bool bUseInitialCenters)
{
	if(!bUseInitialCenters)
		centers.setSize(0,0);

	KMeanCluster(inputvectors, cluster_n, centers, group_index, centers);
}

void CFuzzyCluster::KMeanCluster(const matrixn& inputvectors, int cluster_n, matrixn& centers, int *group_index, const matrixn& input_centers)
{
	ASSERT(cluster_n<=inputvectors.rows());
	bool again;						// stable?
	int l;

	m_real factor;
	int max_iter=1000;	// maximum number of iteration

	matrixn presult;	// result of previous iteration
	matrixn cresult;	// result of current iteration
	matrixn distMat;	// distance between inputvectors and centers
	m_real dist;			// distance from center, variables
	vectorn direction;
	m_real mindist, thresh;
	matrixn sum;		// used for calculating centers

	int num_input=inputvectors.rows();
	vectorn min;
	vectorn max;

	intvectorn groupNum;
	groupNum.setSize(cluster_n);

	min.minimum(inputvectors);
	max.maximum(inputvectors);

	m_real interval=min.distance(max);

	if(inputvectors.rows() < 1)
		return;
	else
	{
		ASSERT(input_centers.rows()==0 || input_centers.cols()==inputvectors.cols());

		cresult.setSize(cluster_n, inputvectors.cols());

		int num_initial_centers=MIN(input_centers.rows(), cluster_n);

		for(int i=0; i< num_initial_centers; i++)
		{
			// copy from initial values
			cresult.setRow(i, input_centers.row(i));
		}

		int randv=rand();
		factor=(m_real)inputvectors.rows()/(m_real)cluster_n;

		for(int i=num_initial_centers; i<cluster_n; i++)
		{
			// random select
			int data_index=(int(factor*i+0.5)+randv)%num_input;
			cresult.setRow(i, inputvectors.row(data_index));
		}
	}

	centers.setSize(cluster_n, inputvectors.cols());
	for(int i=0;i<max_iter;i++)
	{
		again=false;

		m::distanceMat(distMat, inputvectors, cresult);
		for(int j=0;j<num_input;j++)
		{
			// for each data point, 가장 가까운 cluster로 지정
			distMat.row(j).findMin(mindist, l);
			group_index[j]=l;
		}

		sum.setSize(cluster_n, inputvectors.cols());
		sum.setAllValue(0);
		groupNum.setAllValue(0);
		for(int j=0; j<num_input; j++)
		{
			sum.row(group_index[j])+=inputvectors.row(j);
			groupNum[group_index[j]]++;
		}
		presult.assign(cresult);

		for(int l=0; l<cluster_n; l++)
		{
			if(groupNum[l]!=0)
				cresult.row(l).div(sum.row(l), groupNum[l]);
		}
		for(int l=0; l<cluster_n; l++)
		{
			if(groupNum[l]==0)
			{
				m_real max_dist = 0;
				int max_index = -1;
				//
				for ( int j = 0; j < num_input; j++) {
					dist=inputvectors.row(j).distance(cresult.row(group_index[j]));
					if (dist > max_dist && groupNum[group_index[j]]>1) {
						max_dist = dist;
						max_index = j;
					}
				}
				//
				ASSERT(max_index >= 0 && max_index < num_input);
				groupNum[group_index[max_index]] --;
				group_index[max_index] = l;
				cresult.setRow(l,inputvectors.row(max_index));
			}
		}

		thresh=interval/1000.f;
		for(int l=0;l<cluster_n;l++)
		{
			if(presult.row(l).distance(cresult.row(l))>thresh)
			{
				again=true;
				break;
			}
		}
		if(!again) break;
	}

	centers.assign(cresult);
}

int Guess(const matrixn &aInputVec, m_real radii, std::deque<vectorn>& centers, int numCluster, int thr)
{
	CFuzzyCluster::Subtractive(aInputVec, radii, centers);

	Msg::print("radii = %8.6f, Num_cluster = %d\n", radii, centers.size());

	if(centers.size()<numCluster-thr) return -1;
	else if(centers.size()>numCluster+thr) return 1;
	else return 0;
}

m_real CFuzzyCluster::FcmKCenter(const matrixn& aInputVec,  int numCluster, int thr, matrixn& aCenter, int* group_index, int max_iter)
{
	std::deque<vectorn> centers;

	matrixn distMat;
	//distMat.op0(m0::draw("aInputVec.bmp"));
	m::distanceMat(distMat,aInputVec);
	//distMat.op0(m0::draw("distance.bmp"));
	m_real distMin=distMat.minimum();
	m_real distMax=distMat.maximum();

	Msg::print("%f %f\n", distMin, distMax);
	// binary search로 찾기
	m_real guess_l=(distMin+distMax)/4.0f, guess_u=(distMin+distMax)/2.0f, half;

	int result;
	for(int i=0; i<max_iter; i++)
	{
		if((result=Guess(aInputVec, guess_u, centers, numCluster, thr))==1)
		{
			// guess_u 계속 키움
			guess_l=guess_u;
			guess_u*=2;
		}
		else if(result==0)
			break;
		else if((result=Guess(aInputVec, guess_l, centers, numCluster, thr))==-1)
		{
			// guess_l 계속 줄임.
			guess_u=guess_l;
			guess_l*=0.5f;
		}
		else
		{
			// guess_l, guess_u사이에 있음
			for(int j=0; j<max_iter; j++)
			{
				half=(guess_l+guess_u)/2;
				if((result=Guess(aInputVec,half, centers, numCluster, thr))==1)
					guess_l=half;
				else if(result==-1)
					guess_u=half;
				else
					goto END;
			}
			goto END;
		}
	}
END:
	Msg::print("\nNo_Fuzzycluster = %d \n", centers.size());

	matrixn tempX(aInputVec);
	Fcm(tempX, aInputVec.rows(), centers.size(), centers, group_index);

	aCenter.setSize(centers.size(), centers[0].size());
	for(int i=0; i<centers.size(); i++)
		aCenter.setRow(i, centers[i]);


	return half;
}

void CFuzzyCluster::CalcVectorGroup(int numInput, int numCluster, int* pGroupIndex, CTArray<intVector>& aVectorGroup)
{
	aVectorGroup.Init(numCluster);
	for(int i=0; i<numInput; i++)
	{
#ifdef _DEBUG
		ASSERT(0<=pGroupIndex[i]);
		ASSERT(pGroupIndex[i]<numInput);
#endif
		aVectorGroup[pGroupIndex[i]].push_back(i);
	}
#ifdef _DEBUG
	for(int i=0; i<numCluster; i++)
		ASSERT(aVectorGroup[i].size() !=0);
#endif
}


void CFuzzyCluster::FcmKMeanCluster(const matrixn &aInputVec, int numCluster, matrixn& aCenters, int *pGroupIndex)
{
#ifdef OUTPUT_TO_FILE
	OutputToFile("optseg.txt","fcmkmeancluster");
#endif
	/////////////////////////////////////////////////////////////////////
	//	Clustering by K-means algorithm
	//  clustering
	matrixn initial_center;

	// binary search FCM-> make initial solution
	FcmKCenter(aInputVec, numCluster+3, 5, initial_center, pGroupIndex);

	KMeanCluster(aInputVec, numCluster, aCenters, pGroupIndex, initial_center);

	OutputToFile("optseg.txt","FCMKCenter Ended");
}

void CFuzzyCluster::GenerateOptimumCluster(const matrixn &aInputVec, int &GroupNum, int *pGroupIndex, m_real thr, matrixn& aCenter)
{
#ifdef OUTPUT_TO_FILE
	OutputToFile("optseg.txt","GenerateOptimumCluster");
#endif
	int i;
	/////////////////////////////////////////////////////////////////////
	//	Clustering by K-means algorithm
	//  clustering
	matrixn initial_center;
	int numCluster = 1;

	/*
	// binary search FCM-> make initial solution
	FcmKCenter(aInputVec, numCluster, 3, initial_center, pGroupIndex);
	OutputToFile("optseg.txt","FCMKCenter Ended");
	*/
	vectorn temp;
	vectorn temp1;

	m_real max_length = 0.0f;int max_index1 = 0;	int max_index2 = 0;
	m_real min_length = FLT_MAX; int min_index1 = 0; int min_index2 = 1;

	//m_real thresh_max = 3.0f;
	//m_real thresh_min = 1.0f;
	m_real thresh_max = thr;
	m_real thresh_min = thr/2.f;


	int dimension = aInputVec.cols();

	int max_iter = 100;

	CTArray<intVector> aVectorGroup;

	/////////////////////////////////////////////////////////////////////
	//	Clustering by K-means algorithm
	CFuzzyCluster::KMeanCluster(aInputVec, numCluster, aCenter, pGroupIndex, initial_center);
	CheckNonEmpty(aInputVec, numCluster, aCenter, pGroupIndex);
	for(int iter=0; iter<max_iter; iter++)
	{

		CalcVectorGroup(aInputVec.rows(), numCluster, pGroupIndex, aVectorGroup);
		CalcInnerClusterScore(aInputVec, aCenter, aVectorGroup, max_index1, max_index2, max_length);
		CalcInterClusterScore(aInputVec, aCenter, numCluster, min_index1, min_index2, min_length);

		/////////////////////////////////////////////////////////////////////
		// check score

		int numCluster_old = numCluster;

		if(max_length>thresh_max)
		{
			/////////////////////////////////////////////////////////////////////
			// for cluster split
			ASSERT(max_index1<numCluster);
			initial_center.setSize(numCluster+1, aInputVec.cols());
			for(i=0; i<numCluster; i++)
				initial_center.setRow(i, aCenter.row(i));
			initial_center.setRow(numCluster, aInputVec.row(aVectorGroup[max_index1][max_index2]));
			numCluster ++;
		}
		else if(min_length<thresh_min)
		{
			/////////////////////////////////////////////////////////////////////
			// for cluster merge
			initial_center.setSize(numCluster-1, aInputVec.cols());
			ASSERT(min_index1 != min_index2);
			ASSERT(min_index1 < numCluster && min_index2 < numCluster);
			int ind = 0;
			for(i=0; i<numCluster; i++){
				if(i != min_index1 && i != min_index2){
					initial_center.setRow(ind, aCenter.row(i));
					ind ++;
				}
			}
			ASSERT(ind == numCluster-2);
			m_real t1 = aVectorGroup[min_index1].size();
			m_real t2 = aVectorGroup[min_index2].size();

			temp.mult(aCenter.row(min_index1), t1/(t1+t2));
			temp1.mult(aCenter.row(min_index2), t2/(t1+t2));
			initial_center.row(numCluster-2).add(temp, temp1);

			numCluster --;
		}
		else
			break;
#ifdef OUTPUT_TO_FILE
		MOutputToFile("optseg.txt", ("numCluster %d  ", numCluster_old) );
		MOutputToFile("optseg.txt", ("max_len %6.3f min_len %6.3f \n", max_length, min_length) );
#endif
		/////////////////////////////////////////////////////////////////////
		//	Clustering by K-means algorithm
		KMeanCluster(aInputVec, numCluster, aCenter, pGroupIndex, initial_center);
		if(CheckNonEmpty(aInputVec, numCluster, aCenter, pGroupIndex))
#ifdef OUTPUT_TO_FILE
			OutputToFile("optseg.txt","zero cluster occurred");
#else
		;
#endif

		if(numCluster>=aInputVec.rows()) break;
		if(numCluster<2) break;
	}
	GroupNum = numCluster;
#ifdef OUTPUT_TO_FILE
	OutputToFile("optseg.txt", "OptSegEnded");
#endif
}

void CFuzzyCluster::FcmRadii(const matrixn& inputvectors,  m_real radii, matrixn& aCenter, int* group_index)
{
	// fuzzy cluster
	matrixn inputvec(inputvectors);

	std::deque<vectorn> centers;

	FcmRadii(inputvec, inputvectors.rows(), radii, centers, group_index);

	aCenter.setSize(centers.size(), centers[0].size());
	for(int i=0; i<centers.size(); i++)
		aCenter.setRow(i, centers[i]);


}

m_real CFuzzyCluster::CalcInnerClusterScore(const matrixn& aInputVec, const matrixn& aCenter, const CTArray<intVector>& aVectorGroup, int& maxGroupIndex, int& maxEltIndex, m_real& maxLength)
{
	/////////////////////////////////////////////////////////////////////
	// Inner-Cluster Score;
	maxLength = 0.0f; maxGroupIndex = maxEltIndex = 0;
	m_real distance;
	int numCluster=aVectorGroup.Size();
	m_real cluster_score, score_inner=0.f;
	for(int i=0; i<numCluster; i++)
	{
		cluster_score = 0.0f;
		for(int j=0; j<aVectorGroup[i].size(); j++)
		{
			distance=aCenter.row(i).distance(aInputVec.row(aVectorGroup[i][j]));
			cluster_score +=distance;
			if(maxLength<distance)
			{maxGroupIndex = i; maxEltIndex=j; maxLength = distance;}
		}
		score_inner += (cluster_score / (aVectorGroup[i].size()));
	}
	return score_inner/(m_real)numCluster;
}

m_real CFuzzyCluster::CalcInterClusterScore(const matrixn& aInputVec, const matrixn& aCenter, int numCluster, int& minGroup1, int& minGroup2, m_real& minLength)
{
	/////////////////////////////////////////////////////////////////////
	// Inter-Cluster Score;
	minLength = FLT_MAX; minGroup1 = 1; minGroup2 = 0;
	m_real distance;
	m_real cluster_score=0.f;
	for(int i=0; i<numCluster; i++)
	{
		for(int j=i+1; j<numCluster; j++)
		{
			distance=aCenter.row(i).distance(aCenter.row(j));
			cluster_score+=distance;
			if(minLength>distance)
			{minGroup1 = i; minGroup2 = j; minLength = distance;}
		}
	}
	return cluster_score/m_real(numCluster*numCluster);
}

#include "../BaseLib/math/DynamicTimeWarping.h"
void CFuzzyCluster::CalcDTWDistanceMatrix(const matrixn& aInputVec, int numRow, int numColumn, matrixn& distMat)
{
	matrixn sample1, sample2;
	CDynamicTimeWarping dtw;
	m_real distance;
	distMat.setSize(aInputVec.rows(),aInputVec.rows());
	ASSERT(numRow*numColumn==aInputVec.cols());

	for(int i=0; i<aInputVec.rows(); i++)
	{
		for(int j=i; j<aInputVec.rows(); j++)
		{
			sample1.fromVector(aInputVec.row(i), numColumn);
			sample2.fromVector(aInputVec.row(j), numColumn);
			//sample1.save("sample1.txt");
			distance=dtw.CalcDTWDistance(sample1, sample2);
			distMat[i][j]=distMat[j][i]=distance;
		}
	}
}

typedef std::list<int> CCluster;


void CFuzzyCluster::AggloCluster(const matrixn& distMat, m_real inter_cluster_thr, int& cluster_n, int* group_index, int eLinkage)
{
	cluster_n=1;
	matrixn interDist;
	vectorn innerDist;
	AggloClusterCore(distMat, interDist, innerDist,inter_cluster_thr, cluster_n, group_index, eLinkage);
}

void CFuzzyCluster::AggloCluster(const matrixn& distMat, int cluster_n, int* group_index, int eLinkage)
{
	matrixn interDist;
	vectorn innerDist;
	AggloClusterCore(distMat, interDist, innerDist, FLT_MAX, cluster_n, group_index, eLinkage);
}

void FindMin(const matrixn& interDist, int nCluster, m_real& fMinDist, int& min_i, int& min_j)
{
	fMinDist=interDist[0][1];
	min_i=0; min_j=1;

	for(int i=0; i<nCluster; i++)
	{
		for(int j=i+1; j<nCluster; j++)
		{
			if(interDist[i][j]<fMinDist)
			{
				fMinDist=interDist[i][j];
				min_i=i;
				min_j=j;
			}
		}
	}
}

m_real CalcLinkage(const matrixn& interDist, const CTArray<CCluster>& aClusters, int i, int j, int mode=1)
{
	// mode 0: min, 1: average, 2: max linkage
	m_real fLinkage;
	switch(mode)
	{
	case 0:
		fLinkage=FLT_MAX;
		break;
	case 1:
	case 2:
		fLinkage=0;
		break;
	}

	for(CCluster::iterator il=aClusters[i].begin(); il!=aClusters[i].end(); il++)
	{
		for(CCluster::iterator jl=aClusters[j].begin(); jl!=aClusters[j].end(); jl++)
		{
			switch(mode)
			{
			case 0:
				fLinkage=MIN(fLinkage,interDist[*il][*jl]);
				break;
			case 1:
				fLinkage+=interDist[*il][*jl];
				break;
			case 2:
				fLinkage=MAX(fLinkage,interDist[*il][*jl]);
				break;
			}
		}
	}
	if(mode==1)		fLinkage/=aClusters[i].size()*aClusters[j].size();
	return fLinkage;
}

m_real CalcMergedInnerLinkage(CTArray<CCluster>& aClusters, const vectorn& innerDist, const matrixn& interDist,int i, int j, int eLinkage)
{
	// merge group i + group j
	switch(eLinkage)
	{
	case 0:// min
		return MIN3(innerDist[i], innerDist[j], interDist[i][j]);
	case 1:// average
		{
			m_real ni, nj;
			ni=aClusters[i].size();
			nj=aClusters[j].size();

			m_real ci=ni*(ni-1)/2.0;
			m_real cj=nj*(nj-1)/2.0;

			return (innerDist[i]*ci+innerDist[j]*cj+interDist[i][j]*ni*nj)/(ci+cj+ni*nj);
		}
		break;
	case 2:// max
		return MAX3(innerDist[i], innerDist[j], interDist[i][j]);
	}
	return 0;
}

m_real CalcMergedInterLinkage(CTArray<CCluster>& aClusters, const vectorn& innerDist, const matrixn& interDist,int i, int j, int k, int eLinkage)
{
	switch(eLinkage)
	{
	case 0:// min
		return MIN(interDist[i][k], interDist[j][k]);
		break;
	case 1:// average
		{
			m_real ni, nj, nk;
			ni=(m_real)aClusters[i].size();
			nj=(m_real)aClusters[j].size();
			nk=(m_real)aClusters[k].size();

			return (interDist[i][k]*ni*nk+interDist[j][k]*nj*nk)/((ni+nj)*nk);
		}
		break;
	case 2:// max
		return MAX(interDist[i][k], interDist[j][k]);
	}
	return 0;
}


void CFuzzyCluster::AggloClusterCore(const matrixn& distMat, matrixn& interDist, vectorn& innerDist, m_real inner_cluster_thr, int& cluster_n, int* group_index, int eLinkage)
{
	// brute force implementation
	// cluster tree
	// 어떻게 구현할까..흠..

	CTArray<CCluster> aClusters;

	ASSERT(distMat.rows()==distMat.cols());
	int numElts=distMat.rows();
	aClusters.Init(numElts);

	// make initial clusters (one elt per one cluser )
	for(int i=0; i<numElts; i++)
		// add first element.
		aClusters[i].push_back(i);

	// inter distance matrix , initially는 distMat과 같지만 점점 작아진다. 실제로 매트릭스 크기를 줄이진 않고, 바깥쪽을 그냥 사용하지 않는다.
	interDist.assign(distMat);
	innerDist.setSize(numElts);
	innerDist.setAllValue(0);

	int min_i, min_j;
	m_real fMinDist,fInnerLinkage;
	vectorn colVec, rowVec;

	// linkage 를 뜻한다 0:min, 1: average, 2: max linkage

	// agglomerative clustering
	for(int iteration=0; iteration< numElts-cluster_n; iteration++)
	{
		// find min_interCluster_distance pair (i,j)
		FindMin(interDist, aClusters.Size(), fMinDist, min_i, min_j);

		fInnerLinkage=CalcMergedInnerLinkage(aClusters, innerDist, interDist,min_i, min_j, eLinkage);
		if(fInnerLinkage>inner_cluster_thr)
			break;
		// merge cluster min_i and min_j into min_i
		innerDist[min_i]=fInnerLinkage;

		for(int k=0; k<aClusters.Size(); k++)
		{
			if(k!=min_i)
				interDist[k][min_i]=interDist[min_i][k]=CalcMergedInterLinkage(aClusters,innerDist, interDist, min_i, min_j, k, eLinkage);
		}
		// merge i and j
		aClusters[min_i].insert(aClusters[min_i].end(),aClusters[min_j].begin(), aClusters[min_j].end());

		// empty min_j 와 마지막 element를 바꾸기. (빈공간 없애기)
		// min_j는 없어졌고 마지막이 min_j로 왔으니까 distance matrix도 마지막 컬럼과 로우가 min_j컬럼과 min_j로우로 와야 된다.
		innerDist[min_j]=innerDist[aClusters.Size()-1];
		aClusters.Swap(aClusters.Size()-1, min_j);
		interDist.getRow(aClusters.Size()-1, rowVec);
		interDist.setRow(min_j, rowVec);
		interDist.getColumn(aClusters.Size()-1, colVec);
		interDist.setColumn(min_j, colVec);
		aClusters.Resize(aClusters.Size()-1);

		/*
#ifdef _DEBUG
		for(int j=0; j<aClusters.Size(); j++)
		{
			m_real fAverageLinkage;
			if(j==min_i) continue;
			// 마지막 parameter는 linkage 를 뜻한다 0:min, 1: average, 2: max linkage
			fAverageLinkage=CalcLinkage(distMat, aClusters, min_i, j, eLinkage);
			ASSERT(isSimilar(interDist[min_i][j],fAverageLinkage));
			ASSERT(isSimilar(interDist[j][min_i],fAverageLinkage));
		}
#endif*/
	}

	Msg::verify(inner_cluster_thr!=FLT_MAX || aClusters.Size()==cluster_n, "clustering failed! %d", aClusters.Size());
	cluster_n=aClusters.Size();

	for(int i=0; i<numElts; i++)
		group_index[i]=-1;

	for(int i=0; i<aClusters.Size(); i++)
	{
		for(CCluster::iterator il=aClusters[i].begin(); il!=aClusters[i].end(); il++)
		{
			group_index[*il]=i;
		}
	}
}

void CFuzzyCluster::LexicoCluster(const matrixn& aInputVec, int& cluster_n, int* group_index)
{
	matrixn aCenter;
	aCenter.setSize(aInputVec.rows(), aInputVec.cols());

	cluster_n=0;

	for(int i=0; i<aInputVec.rows(); i++)
	{
		bool bFound=false;
		for(int j=0; j<cluster_n; j++)
		{
			if(aCenter.row(j).isSimilar(aInputVec.row(i)))
			{
				group_index[i]=j;
				bFound=true;
				break;
			}
		}

		if(!bFound)
		{
			aCenter.setRow(cluster_n, aInputVec.row(i));
			group_index[i]=cluster_n;
			cluster_n++;
		}
	}
}

void CFuzzyCluster::DTWAggloCluster(const matrixn &aInputVec, int numRow, int numColumn, m_real inner_cluster_thr, int& cluster_n, int* group_index, m_real *pfMaxInner, m_real* pfMinInter)
{
static	matrixn DTWdistMat;
static  matrixn interDist;
static  vectorn innerDist;

	if(pfMaxInner)
		*pfMaxInner=FLT_MIN;
	if(pfMinInter)
		*pfMinInter=FLT_MAX;

	CalcDTWDistanceMatrix(aInputVec, numRow, numColumn, DTWdistMat);

//#define USE_TRANSFORMATION
#ifdef USE_TRANSFORMATION
	// taesoo test
	// 속도가 큰놈은 distance를 줄여줄 필요가 있다. 빠른 놈은 좀 덜 비슷해도 블렌딩 가능하기 때문
	vectorn aSpeed(aInputVec.rows());

	matrixn pos,vel;
	vectorn allSpeed;
	for(int i=0; i<aInputVec.rows(); i++)
	{
		pos.fromVector(aInputVec.row(i), numColumn);
		vel.derivative(pos);
		allSpeed.aggregateRows(OP_RMS, vel);
		aSpeed[i]=allSpeed.avg();
	}

	for(i=0; i<DTWdistMat.rows(); i++)
	{
		for(int j=i; j<DTWdistMat.cols(); j++)
		{
			DTWdistMat[i][j]/=aSpeed[i]*aSpeed[j];
			DTWdistMat[j][i]=DTWdistMat[i][j];
		}
	}
#endif

	// cluster후 minInner, maxInter계산
	{
		cluster_n=1;
		AggloClusterCore(DTWdistMat, interDist, innerDist,inner_cluster_thr, cluster_n, group_index);
		ASSERT(cluster_n<=DTWdistMat.rows());
		if(pfMaxInner)
		{
			for(int j=0; j<cluster_n; j++)
			{
				if(innerDist[j]>*pfMaxInner)
					*pfMaxInner=innerDist[j];
			}
		}
		if(pfMinInter)
		{
			for(int k=0; k<cluster_n; k++)
				for(int j=k+1; j<cluster_n; j++)
				{
					if(interDist[k][j]<*pfMinInter)
						*pfMinInter=interDist[k][j];
				}
		}
	}
}

void CFuzzyCluster::DTWKmeanCluster(const matrixn &aInputVec, int numRow, int numColumn, m_real inner_cluster_thr, int& cluster_n, int* group_index, matrixn& aCenter, matrixn& refPattern, m_real *pfMaxInner, m_real* pfMinInter)
{
	int refPatternIndex=0;

	if(aInputVec.rows()>1)
	{
		static matrixn DTWdistMat;
		CalcDTWDistanceMatrix(aInputVec, numRow, numColumn, DTWdistMat);

		m_real MIN_SSE=FLT_MAX;
		// find reference pattern
		for(int i=0; i<aInputVec.rows(); i++)
		{
			m_real SSE=0;
			for(int j=0; j<aInputVec.rows(); j++)
			{
				if(j!=i)
					SSE+=DTWdistMat[i][j];
			}
			if(SSE<MIN_SSE)
			{
				MIN_SSE=SSE;
				refPatternIndex=i;
			}
		}
	}

	CDynamicTimeWarping dtw;
	matrixn sample;
	matrixn timeWarpedPattern;
	timeWarpedPattern.setSize(aInputVec.rows(), aInputVec.cols());

	refPattern.fromVector(aInputVec.row(refPatternIndex), numColumn);

	for(int i=0; i<aInputVec.rows(); i++)
	{
		if(i==refPatternIndex)
		{
			timeWarpedPattern.setRow(i, aInputVec.row(i));
		}
		else
		{
			sample.fromVector(aInputVec.row(i), numColumn);
			dtw.TimeWarpToReferencePattern(refPattern, sample);
			vectornView tri=timeWarpedPattern.row(i);
			sample.toVector(tri);
		}
	}
//#define USE_MOTION_PCA
#ifdef USE_MOTION_PCA
	if(aInputVec.rows()>1)
	{
		matrixn postureFeatures;
		postureFeatures.assign(timeWarpedPattern);

		// do PCA on posture features
		PCA motionPCA;
		motionPCA.getPCA(postureFeatures);
		timeWarpedPattern.setSize(postureFeatures.rows(), motionPCA.m_ReducedDim);
		for(int i=0; i<aInputVec.rows(); i++)
			motionPCA.GetReducedData(i, timeWarpedPattern.row(i));
	}
#endif
	if(timeWarpedPattern.rows()==1)
	{
		cluster_n=1;
		group_index[0]=0;
		aCenter.assign(timeWarpedPattern);
	}
	else
		GenerateOptimumCluster(timeWarpedPattern, cluster_n, group_index, inner_cluster_thr, aCenter);
	if(pfMaxInner&& pfMinInter)
	{
		CTArray<intVector> aVectorGroup;

		CalcVectorGroup(timeWarpedPattern.rows(), cluster_n, group_index, aVectorGroup);
		int MGI,MGI2, MEI;
		CalcInnerClusterScore(timeWarpedPattern, aCenter, aVectorGroup, MGI,MEI,*pfMaxInner);
		CalcInterClusterScore(timeWarpedPattern, aCenter, cluster_n, MGI, MGI2,*pfMinInter);
	}
}

void CFuzzyCluster::SubClustering::AddElement(int targetIndex, int GroupIndex, const matrixn& temporalFeature)
{
	int numElt=aTemporal.rows();
	ASSERT(GroupIndex<numGroup);
	ASSERT(aTargetIndex.size()==numElt);
	ASSERT(aGroupIndex.size()==numElt);
	ASSERT(aTargetIndex.findFirstIndex(targetIndex)==-1);

	aTemporal.resize(numElt+1, aTemporal.cols());
	aTargetIndex.resize(numElt+1);
	aGroupIndex.resize(numElt+1);

	aTemporal.row(numElt).fromMatrix(temporalFeature);
	aTargetIndex[numElt]=targetIndex;
	aGroupIndex[numElt]=GroupIndex;
	ASSERT(aTemporal.row(numElt).distance(aCenter.row(GroupIndex))<100);
	// aCenter update를 해야하지만, 생략.(귀차니즘)
}

int CFuzzyCluster::SubClustering::FindCluster(matrixn& postureFeature, m_real inner_thr)
{
	CDynamicTimeWarping cDTW;
	cDTW.TimeWarpToReferencePattern(referencePattern, postureFeature);
	vectorn vecPostureFeature;
	vecPostureFeature.fromMatrix(postureFeature);

	for(int j=0; j<aCenter.rows(); j++)
	{
		if(vecPostureFeature.distance(aCenter.row(j))<inner_thr)
			return j;
	}
	return -1;
}

int CFuzzyCluster::SubClustering::CountSameGroupElements(int targetIndex)
{
	int group=aGroupIndex[aTargetIndex.findFirstIndex(targetIndex)];
	return aGroupIndex.count(s2::INT_EQUAL, group);
}
void CFuzzyCluster::SubClustering::Init()
{
	numGroup=1;
	aTemporal.setSize(0,0);
	aTargetIndex.setSize(0);
	aGroupIndex.setSize(0);
}

void CFuzzyCluster::SubClustering::RemoveEmpty()
{
	// 빈그룹 없애주기.
	int numElt=aTargetIndex.size();
	intvectorn groupEltCount(numGroup);

	while(1)
	{
		groupEltCount.setAllValue(0);

		for(int i=0; i<numElt; i++)
			groupEltCount[aGroupIndex[i]]++;

		intvectorn indexV;
		indexV.findIndex(groupEltCount, 0);

		if(indexV.size()>0)
		{
			if(aTemporal.rows()==0)
				numGroup=0;
			else
			{
				int emptyGroup=indexV[0];
				aCenter.setRow(emptyGroup, aCenter.row(numGroup-1));
				indexV.findIndex(aGroupIndex, numGroup-1);
				aGroupIndex.setAt(indexV, emptyGroup);
				ASSERT(aCenter.rows()==numGroup);
				numGroup--;
				aCenter.resize(aCenter.rows()-1, aCenter.cols());
			}
			// invalidate
			globalGroupStartIndex=-1;
		}
		else
			break;
	}
}

void CFuzzyCluster::SubClustering::DelElement(int targetIndex)
{
	int numElt=aTemporal.rows();
	ASSERT(aTargetIndex.size()==numElt);
	ASSERT(aGroupIndex.size()==numElt);

	intvectorn indexV;
	indexV.findIndex(aTargetIndex, targetIndex);
	ASSERT(indexV.size()==1);
	int index=indexV[0];
	aTemporal.setRow(index, aTemporal.row(numElt-1));
	aTargetIndex[index]=aTargetIndex[numElt-1];
	aGroupIndex[index]=aGroupIndex[numElt-1];

	aTemporal.resize(numElt-1, aTemporal.cols());
	aTargetIndex.resize(numElt-1);
	aGroupIndex.resize(numElt-1);
	// aCenter update를 해야하지만, 생략.(귀차니즘)
	numElt--;
}

void CFuzzyCluster::SubClustering::Cluster(int nNumSample, int nSizePostureFeature, m_real maxInner)
{
	DTWKmeanCluster(aTemporal, nNumSample, nSizePostureFeature, maxInner, numGroup, aGroupIndex.dataPtr(), aCenter, referencePattern);
}

void MakeGroupIndex(intvectorn& aGroupIndex, CTArray<CFuzzyCluster::SubClustering>& subClusters, int& numGroup)
{
	int currGlobalGroupStartIndex=0;
	for(int i=0; i<subClusters.Size(); i++)
	{
		ASSERT(subClusters[i].numGroup!=0);
		subClusters[i].globalGroupStartIndex=currGlobalGroupStartIndex;
		currGlobalGroupStartIndex+=subClusters[i].numGroup;
	}
	numGroup=currGlobalGroupStartIndex;

	intvectorn globalGroupIndex;
	for(int i=0; i<subClusters.Size(); i++)
	{
		globalGroupIndex.add(subClusters[i].aGroupIndex,subClusters[i].globalGroupStartIndex);
		aGroupIndex.setAt(subClusters[i].aTargetIndex, globalGroupIndex);
	}
}

void CFuzzyCluster::Classify(const matrixn& aInputVec, int nNumSample, int nSizePostureFeature, m_real INNER_THR, int& numGroup, int* _aGroupIndex, CTArray<SubClustering>& clusters, m_real *pfMaxInner, m_real* pfMinInter, bool bUseAgglo)
{
	// exact-cluster using just non-temporal features.(num-step, interaction ...)
	intvectorn columns, globalGroupIndex;

	// temporal features
	columns.colon(0, nNumSample*nSizePostureFeature);
	matrixn aTemporal;
	aTemporal.extractColumns(aInputVec,columns);

	// non-temporal features
	columns.colon(nNumSample*nSizePostureFeature, aInputVec.cols());
	matrixn aNonTemporal;
	aNonTemporal.setSize(aInputVec.rows(), columns.size());
	aNonTemporal.extractColumns(aInputVec, columns);

	// lexico-graphical exact clustering
//	CImageProcessor::SaveMatrix(aNonTemporal,"feature.bmp");
	int nSubCluster;
	intvectorn aSubGroupIndex;
	aSubGroupIndex.setSize(aNonTemporal.rows());
	CFuzzyCluster::LexicoCluster(aNonTemporal, nSubCluster, aSubGroupIndex.dataPtr());

	clusters.Init(nSubCluster);

	if(pfMaxInner)
		*pfMaxInner=FLT_MIN;
	if(pfMinInter)
		*pfMinInter=FLT_MAX;

	// agglo+DTW clustering in subClusters
	numGroup=0;
	intvectorn aGroupIndex(aInputVec.rows());
	for(int i=0; i<nSubCluster; i++)
	{
		clusters[i].aTargetIndex.findIndex(aSubGroupIndex,i);
		// init subclusters
		clusters[i].aTemporal.extractRows(aTemporal,clusters[i].aTargetIndex);
		clusters[i].aGroupIndex.setSize(clusters[i].aTargetIndex.size());

		m_real maxInner, minInter;
		// clustering
		if(bUseAgglo)
			DTWAggloCluster(clusters[i].aTemporal, nNumSample, nSizePostureFeature, INNER_THR, clusters[i].numGroup, clusters[i].aGroupIndex.dataPtr(), &maxInner, &minInter);
		else
			DTWKmeanCluster(clusters[i].aTemporal, nNumSample, nSizePostureFeature, INNER_THR, clusters[i].numGroup, clusters[i].aGroupIndex.dataPtr(), clusters[i].aCenter, clusters[i].referencePattern, &maxInner, &minInter);

		if(pfMaxInner&& pfMinInter)
		{
			*pfMaxInner=MAX(maxInner, *pfMaxInner);
			*pfMinInter=MIN(minInter, *pfMinInter);
		}
	}
	// sub-clusters 반영하기.
	MakeGroupIndex(aGroupIndex, clusters, numGroup);

	memcpy(_aGroupIndex, aGroupIndex.dataPtr(), sizeof(int)*aInputVec.rows());
}
