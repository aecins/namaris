/////////////////////////////////////////////////////////////////////////////
// Name:        VectorPointSet.h
// Purpose:     Container for points represented as they are
// Author:      Raghav Subbarao
// Modified by:
// Created:     10/01/2008
// Copyright:   (c) Raghav Subbarao
// Version:     v0.1
/////////////////////////////////////////////////////////////////////////////

#ifndef _C_VECTOR_POINT_SET_
#define _C_VECTOR_POINT_SET_

#include "PointSet.h"

template<typename T> class CVectorPointSet : public CPointSet<T>{

public:

	// Empty Constructor
	CVectorPointSet();

	// Standard Constructor
	// d = dimension of data
	// n = number of points to allocate memory for.
	CVectorPointSet(int d, int n);

	// Standard Constructor with data
	// d = dimension of data
	// n = number of points to allocate memory for.
	// data = data to be copied.
	CVectorPointSet(int d, int n, const T* data);

	// TODO: change for reference coutning
	// Copy Constructor
	CVectorPointSet(CVectorPointSet<T>& ps);

	// Destructor
	virtual ~CVectorPointSet(){};

	// Copies point directly
	inline void loadPoint(T *x);

	// Copies point directly into x.
	// Note: assumes space has been allocated for x.
	inline void returnPoint(T *x, int n);

};

template<typename T> CVectorPointSet<T>::CVectorPointSet():
CPointSet<T>()
{}

template<typename T> CVectorPointSet<T>::CVectorPointSet(int d, int n):
CPointSet<T>(d, d, n)
{}

template<typename T> CVectorPointSet<T>::CVectorPointSet(int d, int n, const T *data):
CPointSet<T>(d, d, n, data)
{}

template<typename T> CVectorPointSet<T>::CVectorPointSet(CVectorPointSet<T>& ps):
CPointSet<T>(ps.m_ps_dim, ps.m_ps_dim, ps.m_ps_nmax)
{
	this->m_ps_n			=	ps.m_ps_n;
	this->m_ps_loc		=	ps.m_ps_loc;
	memcpy(this->m_ps_points, ps.m_ps_points, sizeof(T) * this->m_ps_n * this->m_ps_dim);
}

template<typename T> inline void CVectorPointSet<T>::loadPoint(T* x){

	if(this->m_ps_n == this->m_ps_nmax)
		return;

	memcpy(this->m_ps_points + this->m_ps_n * this->m_ps_dim, x, sizeof(T) * this->m_ps_dim);
	this->m_ps_n++;

}

template<typename T> inline void CVectorPointSet<T>::returnPoint(T* x, int n){

	if(n >= this->m_ps_n)
		return;

	memcpy(x, this->m_ps_points + n * this->m_ps_dim, sizeof(T) * this->m_ps_dim);

}

#endif
