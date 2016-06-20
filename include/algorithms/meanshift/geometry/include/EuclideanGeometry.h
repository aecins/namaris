/****************************************************************

                 Euclidean Geometry Library
	=============================================

Implementation of manifold operations for Euclidean Space

Implemented by Raghav Subbarao
****************************************************************/

#ifndef _C_EUCLIDEAN_GEOMETRY_H_
#define _C_EUCLIDEAN_GEOMETRY_H_

//include needed libraries
#include <math.h>
#include <assert.h>
#include <memory.h>

#include "Geometry.h"

template<typename T> class CEuclideanGeometry : public CGeometry<T>{

public:

	/*/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/
	/* Class Constructor and Destructor */
	/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/

	// Default Constructor
	// scale = dimension of Euclidean space.
	CEuclideanGeometry(int d);

	// Virtual Destructor
	virtual ~CEuclideanGeometry(){};

	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	g									|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Computes the dot product of tangents.				|//
	//|   It always	returns a double value.					|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <const T* gamma>									|//
	//|   An array of type T containing the tangent.		|//
	//|														|//
	//|   <const T* delta>									|//
	//|   An array of type T containing the tangent.		|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		g(delta, gamma)									|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	inline double g(const T *gamma, const T *delta);
	


	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	expm								|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Compute the sum of x and delta. The memory for y	|//
	//|   is assumed to have been allocated already.		|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <T* x>											|//
	//|   An array of type T containing the point x.		|//
	//|														|//
	//|   <const T* delta>									|//
	//|   An array of type T containing the tangent vector.	|//
	//|														|//
	//|   <T* y>											|//
	//|   The result is returned in y.						|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		expm(delta, y)									|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	inline void expm(const T *x, const T *delta, T *y);



	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	logm								|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Computes the difference of x and y. The memory for|//
	//|   delta is assumed to have been allocated already.	|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <T* x>											|//
	//|   An array of type T containing the point x.		|//
	//|														|//
	//|   <const T* y>										|//
	//|   An array of type T containing the point.			|//
	//|														|//
	//|   <T* delta>										|//
	//|   The tangent is returned in delta.					|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		logm(y, delta)									|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	inline void logm(const T *x, const T *y, T *delta);

};



/********************************************************/
//Class Constructor										//
//*******************************************************/
//Post:													//
//      The EuclideanGeometry class is initialized.		//
/********************************************************/
template<typename T> CEuclideanGeometry<T>::CEuclideanGeometry(int d):
CGeometry<T>(d, d)
{}

/********************************************************/
//Inner Product											//
//******************************************************//
//Computes the dot product of tangents.					//
/********************************************************/
//Pre:													//
//      - gamma is a tangent of size m_d.				//
//      - delta is a tangent of size m_d.				//
//Post:													//
//      - the product is returned as a double value.	//
//*******************************************************/
template<typename T> double CEuclideanGeometry<T>::g(const T *gamma, const T *delta){

	double d = 0;
	for(int i = 0; i < this->m_d; i++)
		d += gamma[i] * delta[i];
	return d;

}



/********************************************************/
//Manifold Exponential									//
//******************************************************//
//Computes the exponential by adding delta to x.		//
/********************************************************/
//Pre:													//
//      - x is a vector of size m_d.					//
//      - delta is a tangent of size m_d.				//
//Post:													//
//      - the returned vector is y = x + delta			//
//*******************************************************/
template<typename T> void CEuclideanGeometry<T>::expm(const T* x, const T *delta, T *y){

	for(int i = 0; i < this->m_d; i++)
		y[i] = x[i] + delta[i];
	
}



/********************************************************/
//Manifold Logarithm									//
//******************************************************//
//Computes the logarithm as the difference of x and y.	//
/********************************************************/
//Pre:													//
//      - x is a vector of size m_d.					//
//      - y is a vector of size m_d.					//
//Post:													//
//      - the returned tangent is delta = y - x			//
//*******************************************************/
template<typename T> void CEuclideanGeometry<T>::logm(const T* x, const T *y, T *delta){

	for(int i = 0; i < this->m_d; i++)
		delta[i] = y[i] - x[i];
	
}


#endif
