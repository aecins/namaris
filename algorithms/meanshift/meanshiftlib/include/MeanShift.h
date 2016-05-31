/*******************************************************

                 Mean Shift Class
	=============================================

	This is a set of functions for performing mean shift 
	over Riemannian manifolds.

Implemented by Raghav Subbarao
********************************************************/

#ifndef _C_MEAN_SHIFT_
#define _C_MEAN_SHIFT_

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "algorithms/meanshift/geometry/include/Geometry.h"
#include "algorithms/meanshift/meanshiftlib/include/PointSet.h"

#include <iostream>
#include <vector>
#include <algorithm>

// TODO : add usual distance finding function. Basically make it adaptive ms
// TODO : add pilot density estimation function
// #ifndef min
// #define min(a,b) ((a) <= (b) ? (a) : (b))
// #endif
// #ifndef max
// #define max(a,b) ((a) >= (b) ? (a) : (b))
// #endif

#define RSMSMINSTEP 1.0e-7
#define RSMAXMSITER 200

template<typename T> class CMeanShift{

public:

	enum EKernelType {
		EEpanechnikovKernel = 0,
		EGaussianKernel = 1
	};

	/*/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/
	/* Class Constructor and Destructor */
	/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/

	// Default Constructor
	CMeanShift(EKernelType kernel = EGaussianKernel);

	// Destructor
	virtual ~CMeanShift();

	/*/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/
	/*				Other Public Methods			 */
	/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/

	// Bandwidth Initialization Function
	inline void setBandwidth(double h) { m_ms_h = h; };
	
	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	doMeanShift							|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Public mean shift function. Performs mean shift	|//
	//|   with a given set of data points with a given		|//
	//|   geometry.											|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <CGeometry<T> geom>								|//
	//|   A geometry class which is used for computing		|//
	//|	  exponentials and logarithms.						|//
	//|														|//
	//|   <CPointSet<T>& x>									|//
	//|   The data points which define the kernel density.	|//
	//|														|//
	//|   <CPointSet<T>& modes>								|//
	//|   The modes are returned in this. They are sorted	|//
	//|   in decreasing order of kernel density.			|//
	//|														|//
	//|   <double frac>										|//
	//|   The fraction of points from which iterations are	|//
	//|   initialized.										|//
	//|														|//
	//|   <int jump>										|//
	//|   The jump between points used to start iterations.	|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		doMeanShift(geom, x, modes, frac, jump)			|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	double doMeanShift(CGeometry<T>& geom, CPointSet<T>& x, CPointSet<T>& modes, std::vector<double> &probs, double frac = 1.0, int jump = 1);

	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	doMeanShift							|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Public mean shift function. Performs mean shift	|//
	//|   with a given set of data points and a given		|//
	//|   geometry. Only one set of iterations are done		|//
	//|   starting at a specified point.					|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <CGeometry<T> geom>								|//
	//|   A geometry class which is used for computing		|//
	//|	  exponentials and logarithms.						|//
	//|														|//
	//|   <CPointSet<T>& x>									|//
	//|   The data points which define the kernel density.	|//
	//|														|//
	//|   <T* p>											|//
	//|   On entry this specifies the point from which		|//
	//|   iterations are started. On exit this contains the	|//
	//|   point of convergence of the iterations.			|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		doMeanShift(geom, x, p)							|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	double doSingleMeanShift(CGeometry<T>& geom, CPointSet<T>& x, T *p);

	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	pruneModes							|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Mode Pruning function. Removes modes which are	|//
	//|   too close to each other or modes with not enough	|//
	//|   support. The returned modes are organized in		|//
	//|   decreasing order of kernel scores.				|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <CGeometry<T> geom>								|//
	//|   A geometry class which is used for computing		|//
	//|	  distances.										|//
	//|														|//
	//|   <CPointSet<T>& unprunedModes>						|//
	//|   The modes to be pruned.							|//
	//|														|//
	//|   <CPointSet<T>& prunedModes>						|//
	//|   The pruned modes.									|//
	//|														|//
	//|   <int minSize>										|//
	//|   The minimum size required for a mode.				|//
	//|														|//
	//|   <int mindis>										|//
	//|   The minimum distance required between modes.		|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		pruneModes(geom, unprunedModes, prunedModes,	|//
	//|					minSize, mindis);					|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	int pruneModes(CGeometry<T>& geom, CPointSet<T>& unprunedModes, CPointSet<T>& prunedModes, std::vector<std::vector<int> > &prunedModeSupport, int minSize, double mindis = 1.0);

	void getKernelDensities(CGeometry<T>& geom, CPointSet<T>& x, CPointSet<T>& modes, double *kernelDensities);

protected:
	
	// sorting functions
	void sort(double* r, int nvals, int* ridx);
	void sort(T* points, double* probs, int n, int d);

	// Evaluates score/probability at given point
	inline double probability(T* point, CGeometry<T>& geom, CPointSet<T>& x);

	// Kernel function
	inline double kernel(const T* x1, const T* x2, CGeometry<T>& geom){ return profile(geom.dism(x1, x2) / (m_ms_h * m_ms_h)); };

	// Single Mean Shift step
	double doMeanShift(const T* currentPosition, T* nextPosition, CGeometry<T>& geom, CPointSet<T>& x);

	/*/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/
	/*					Virtual Methods				 */
	/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/

	// Profile functions. Currently handle Epanechnikov or Gaussian kernels.
	inline virtual double profile(double x);
	inline virtual double profileDiff(double x);

	EKernelType m_kernel;	// Type of kernel

	double m_ms_h;			// bandwidth

	T *m_delta;
	T *m_ms_delta;

};

template<typename T> CMeanShift<T>::CMeanShift(EKernelType kernel) :
m_kernel(kernel), m_ms_h(0), m_delta(0), m_ms_delta(0)
{}

template<typename T> CMeanShift<T>::~CMeanShift()
{

	if(m_delta)
		delete [] m_delta; 
	if(m_ms_delta)
		delete [] m_ms_delta; 
	m_delta = m_ms_delta = 0;

}


template<typename T> double CMeanShift<T>::doMeanShift(CGeometry<T>& geom, CPointSet<T>& x, CPointSet<T>& modes, std::vector<double> &probs, double frac, int jump){

	if(geom.datapointsize() != x.dimension()){
		printf("Geometry and Data Set not of same size!!!\n");
		return 0;
	}

	if(m_delta)		delete [] m_delta;		m_delta		= new T[geom.tangentsize()];
	if(m_ms_delta)	delete [] m_ms_delta;	m_ms_delta	= new T[geom.tangentsize()];

	// select points to initialize iterations at
	int npoints = x.size();//x.selectPoints(frac, jump);
	int dim = x.dimension();

	probs.resize(npoints);
	T *umodes = new T[npoints * dim];

	T *currentPosition	= new T[dim];
	T *nextPosition		= new T[dim];	memset(nextPosition, 0, dim * sizeof(T));
	double dis = 1.0e100;
	int i, iter;

	for(i = 0; i < npoints; i++){

		// get point to start iterations
		memcpy(currentPosition, x[i], sizeof(T) * dim);

		iter = 0;
		do{
			iter++;

			dis = doMeanShift(currentPosition, nextPosition, geom, x);

			memcpy(currentPosition, nextPosition, sizeof(T) * dim);

		}while (dis > RSMSMINSTEP && iter < RSMAXMSITER);

// 		printf("%03d\t%03d\r", i, iter);

		// store mode and compute its probability
		memcpy(umodes + i * dim, currentPosition, sizeof(T) * dim);
		probs[i] = probability(currentPosition, geom, x);
	}
	
// 	printf("\n");

  // NOTE: this is supposed to sort modes in decreasing order of probability,
  // but it seems that it always sorts things in a fixed order. Thus this line
  // is disabled
// 	sort(umodes, probs, npoints, dim);

	for(i = 0; i < npoints; i++)
		modes.addPoint(umodes + i * dim);

	double high = *std::max_element(probs.begin(),probs.end());

	delete [] currentPosition;
	delete [] nextPosition;
	delete [] umodes;

	return high;

}


template<typename T> double CMeanShift<T>::doSingleMeanShift(CGeometry<T>& geom, CPointSet<T>& x, T *p){

	int dim = x.dimension();
	T *currentPosition = new T[dim];
	T *nextPosition = new T[dim];	memset(nextPosition, 0, dim * sizeof(T));
	double dis = 1.0e100;
	int iter;

	memcpy(currentPosition, p, sizeof(T) * dim);

	iter = 0;
	do{
		iter++;
		
		dis = doMeanShift(currentPosition, nextPosition, geom, x);
		
		memcpy(currentPosition, nextPosition, sizeof(T) * dim);
		
	}while (dis > RSMSMINSTEP && iter < RSMAXMSITER);
	
	memcpy(p, currentPosition, sizeof(T) * dim);
	
	delete [] currentPosition;
	delete [] nextPosition;

	return probability(p, geom, x);

}

// This is a greedy procedure to prunne the set of points. Given a set of points
// it goes through the points one by one. If current point is closer than some
// minimum distance to one of the existing clusters add that point to the 
// support of that cluster. Otherwise create a new cluster. Finally remove all
// clusters who's support is less than some minimal support size.
template<typename T> int CMeanShift<T>::pruneModes(CGeometry<T>& geom, CPointSet<T>& unprunedModes, CPointSet<T>& prunedModes, std::vector<std::vector<int> > &prunedModeSupport, int minSize, double mindis){

	int i, j, dim = unprunedModes.dimension(), n = unprunedModes.size();
	int flag;
	int	tLocalModeCount;
	int* tLocalModeSizes;
	T*	tLocalModes;
	
	tLocalModeCount = 0;
	tLocalModeSizes = new int[n];
	tLocalModes = new T[n * dim];

	memset(tLocalModeSizes, 0, sizeof(int) * n);

	unprunedModes.first();
  std::vector<std::vector<int> > prunedModeSupport_all (0);
  prunedModeSupport.resize(0);
  
	for (i = 0; i < n; i++, ++unprunedModes){

		flag = 1;

		for (j = 0; j < tLocalModeCount; j++){
			geom.logm(unprunedModes, tLocalModes + j * dim, m_delta);
			if ((geom.norm(m_delta) / m_ms_h) < mindis){
				tLocalModeSizes[j]++;
        prunedModeSupport_all[j].push_back(i);
				flag = 0;
				break;
			}
		}

		if (flag){
			unprunedModes.getPoint(tLocalModes + tLocalModeCount * dim);
			tLocalModeSizes[tLocalModeCount]++;
			tLocalModeCount++;
      prunedModeSupport_all.push_back(std::vector<int> (0));
      prunedModeSupport_all.back().push_back(i);
		}
	}
	
	for (i = 0; i < tLocalModeCount; i++){
		if (tLocalModeSizes[i] >= minSize){
// 			printf("%03d\n", tLocalModeSizes[i]);
			prunedModes.addPoint(tLocalModes + i * dim);
      prunedModeSupport.push_back(prunedModeSupport_all[i]);
		}
	}
	delete [] tLocalModeSizes;
	delete [] tLocalModes;

	return prunedModes.size();

}


template<typename T> void CMeanShift<T>::getKernelDensities(CGeometry<T>& geom, CPointSet<T>& x, CPointSet<T>& modes, double *kernelDensities){

	if(geom.datapointsize() != x.dimension()){
		printf("Geometry and Data Set not of same size!!!\n");
		return;
	}

	if(m_delta)		delete [] m_delta;		m_delta		= new T[geom.tangentsize()];
	if(m_ms_delta)	delete [] m_ms_delta;	m_ms_delta	= new T[geom.tangentsize()];

	int npoints = modes.size();
	int dim = x.dimension();

	T *currentPosition	= new T[dim];

	for(int i = 0; i < npoints; i++){
		// get point to start iterations
		memcpy(currentPosition, modes[i], sizeof(T) * dim);
		kernelDensities[i] = probability(currentPosition, geom, x);
	}

	delete [] currentPosition;

}


template<typename T> void CMeanShift<T>::sort(double* r, int nvals, int* ridx){
	
	unsigned long n, l, ir, i, j;
	n = nvals;
	double rra;
	int irra;
	
	if (n < 2)
		return;
	
	l = (n >> 1) + 1;
	ir = n;
	
	for (;;){
		
		if (l > 1){
			irra = ridx[(--l) - 1];
			rra = r[l - 1];
		}
		
		else{
			irra = ridx[ir - 1];
			rra = r[ir - 1];
			
			ridx[ir - 1] = ridx[1 - 1];
			r[ir - 1] = r[1 - 1];
			
			if (--ir == 1){
				ridx[1 - 1] = irra;
				r[1 - 1] = rra;
				break;
			}
		}
		
		i = l;
		j = l+l;
		
		while (j<=ir){
			
			if (j < ir && r[j - 1] < r[j + 1 - 1])
				j++;
			
			if (rra < r[j - 1]){
				ridx[i - 1] = ridx[j - 1];
				r[i - 1] = r[j - 1];
				
				i = j;
				j <<= 1;
			}
			else
				j = ir + 1;
		}
		ridx[i - 1] = irra;
		r[i - 1] = rra;
	}

}

template<typename T> void CMeanShift<T>::sort(T* points, double* probs, int n, int d){

	int *index = new int[n], i;
	for(i = 0; i < n; i++)
		index[i] = i;
  
	T *spoints = new T[n * d];
	double *sprobs = new double[n];

	memcpy(spoints, points, sizeof(T) * n * d);
	memcpy(sprobs, probs, sizeof(double) * n);

	sort(sprobs, n, index);

	for(i = 0; i < n; i++){
		probs[n - i - 1] = sprobs[i];

		memcpy(&(points[(n - i - 1) * d]), &(spoints[index[i] * d]), sizeof(T) * d);
	}

	delete [] index;
	delete [] spoints;
	delete [] sprobs;

}

template<typename T> double CMeanShift<T>::doMeanShift(const T* currentPosition, T* nextPosition, CGeometry<T>& geom, CPointSet<T>& x){

	int i, k, m = geom.tangentsize(), n = x.size(), d = geom.datapointsize();
	double sum = 0, dis = 0, w;
	memset(nextPosition, 0, sizeof(T) * d);
	memset(m_ms_delta, 0, sizeof(T) * m);

	for(i = 0, x.first(); i < n; i++, ++x){

		geom.logm(currentPosition, x, m_delta);

        dis = geom.norm(m_delta) / (m_ms_h * m_ms_h);

		if(dis < 1.0){
			w = profileDiff(dis);
			for(k = 0; k < m; k++)
				m_ms_delta[k] += w * m_delta[k];
			sum += w;
		}

	}

	// no update!!!
	if(sum == 0){
		memcpy(nextPosition, currentPosition, sizeof(T) * d);
		return 0;
	}

	// compute mean shift vector
	for(k = 0; k < m; k++)
		m_ms_delta[k] /= sum;

	geom.expm(currentPosition, m_ms_delta, nextPosition);

	return geom.norm(m_ms_delta);

}

template<typename T> inline double CMeanShift<T>::probability(T* point, CGeometry<T>& geom, CPointSet<T>& x){

	double p = 0.0, n = x.size();
	x.first();
	for(int i = 0; i < n; i++, ++x)
		p += kernel(point, x, geom);

	return (p / n);

}

template<typename T> inline double CMeanShift<T>::profile(double x){

	switch (m_kernel){

	case EEpanechnikovKernel:
		return (fabs(x) > 1.0 ? 0.0 : 1.0 - x);
		break;

	case EGaussianKernel:
		return (fabs(x) > 10.0 ? 0.0 : exp(-0.5 * x));
		break;

	default:
		return 0;
		break;
	}

}

template<typename T> inline double CMeanShift<T>::profileDiff(double x){

	switch (m_kernel){

	case EEpanechnikovKernel:
		return (fabs(x) > 1.0 ? 0.0 : 1.0);
		break;

	case EGaussianKernel:
		return (fabs(x) > 3.0 ? 0.0 : exp(-0.5 * x));
		break;

	default:
		return 0;
		break;
	}

}

#endif
