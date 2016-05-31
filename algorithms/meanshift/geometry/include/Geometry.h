/*******************************************************

                 Riemannian Geometry Library
	=============================================

	This set of files if a collection of routines for 
	simple operations over Riemannian manifolds.

Implemented by Raghav Subbarao
********************************************************/

#ifndef _C_GEOMETRY_H_
#define _C_GEOMETRY_H_

//include needed libraries
#include <math.h>
#include <assert.h>
#include <memory.h>

// Geometry Prototype
template<typename T> class CGeometry{

public:
	
	/*/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/
	/* Class Constructor and Destructor */
	/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/

	// Default Constructor
	// d = dimensionality of ambient space or data representation
	// m = size of tangent respresentation.
	CGeometry(int d, int m);

    // Virtual Destructor
	virtual ~CGeometry();

	/*/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/
	/*				Other Public Methods			 */
	/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/

	int datapointsize() const { return m_d; };

	int tangentsize() const { return m_m; };


	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	setBasePoint						|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Sets a base point on the manifold. In the			|//
	//|   future, if an alternate point is'nt specified		|//
	//|   exp and log are computed from this point.			|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <const T* x>										|//
	//|   An array of type T containing the data point.		|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		setBasePoint(x)									|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	inline void setBasePoint(const T *x){ memcpy(m_x, x, sizeof(T) * m_d); };



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
	//|   Compute manifold exponential at m_x. Uses the 	|//
	//|   virtual function expm(x, delta, y). The memory	|//
	//|   is assumed to have been allocated already.		|//
	//|														|//
	//|   The following arguments must be provided:			|//
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
	inline void expm(const T *delta, T *y) { expm(m_x, delta, y); };



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
	//|   Compute manifold logarithm at m_x. Uses the	 	|//
	//|   virtual function logm(x, y, delta). The memory	|//
	//|   is assumed to have been allocated already.		|//
	//|														|//
	//|   The following arguments must be provided:			|//
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
	inline void logm(const T *y, T *delta) { logm(m_x, y, delta); };



	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	dism								|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Computes Riemannian distance from the base point.	|//
	//|   Uses the virtual function dism(x, y).	It always	|//
	//|   returns a double value.							|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <const T* y>										|//
	//|   An array of type T containing the point.			|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		dism(y)											|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	inline double dism(const T *y) { return dism(m_x, y); };



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
	//|   Computes the Riemannian norm of a tangnet.		|//
	//|   Uses the virtual function g(x, y). It always		|//
	//|   returns a double value.							|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <const T* delta>									|//
	//|   An array of type T containing the tangent.		|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		g(delta)										|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	inline double norm(const T *delta) { return g(delta, delta); };

	

	/*/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/
	/*					Virtual Methods				 */
	/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/


	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	setTangent							|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Sets a tangent to be stored in m_del.	Should be 	|//
	//|   redefined if any precomputation is required such	|//
	//|   as for Grassmann manifolds.						|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <const T* del>									|//
	//|   An array of type T containing the tangent.		|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		setTangent(delta)								|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	virtual inline void setTangent(const T *del){ memcpy(m_del, del, sizeof(T) * m_m); };



	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	dism								|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Computes Riemannian distance between x and y by  	|//
	//|   estimating the tangent and computing its norm.	|//
	//|   This is always numerically right but can be		|//
	//|	  changed for efficiency reasons.					|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <const T* x>										|//
	//|   An array of type T containing the point x.		|//
	//|														|//
	//|   <const T* y>										|//
	//|   An array of type T containing the point y.		|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		dism(x, y)										|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	virtual inline double dism(const T *x, const T *y){ logm(x, y, m_del); return norm(m_del); };



	/*/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/
	/*				Pure Virtual Methods			 */
	/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/
	

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
	//|   Computes the Riemannian inner product of tangents.|//
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
	virtual double g(const T *gamma, const T *delta) = 0;



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
	//|   Compute manifold exponential. The memory for y	|//
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
	virtual void expm(const T *x, const T *delta, T *y) = 0;



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
	//|   Compute manifold logarithm at x. The memory for	|//
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
	virtual void logm(const T *x, const T *y, T *delta) = 0;

	// parallel transport function
	// transport g from m_x along m_del a magnitued of a 
//	virtual void parallelTransport(T a, T *g, T *tg) = 0;
//	virtual void selfTransport(T a, T *th) { memcpy(th, m_del, sizeof(T) * m_m); };

protected:
	
	int m_d;			// size of data points
	int m_m;			// size of tangents

	T *m_x;				// base point at which operations are carried out
	T *m_del;			// tangent for certain operations

};


/********************************************************/
//Class Constructor										//
//*******************************************************/
//Post:													//
//      The Geometry class is properly initialized.		//
/********************************************************/
template<typename T> CGeometry<T>::CGeometry(int d, int m):
m_d(d), m_m(m)
{

	m_x		= new T[m_d];
	m_del	= new T[m_m];

}


/********************************************************/
//Virtual Destructor									//
//*******************************************************/
//Post:													//
//      The Geometry class is destroyed.				//
/********************************************************/
template<typename T> CGeometry<T>::~CGeometry(){

	if(m_x)
		delete [] m_x;
	if(m_del)
		delete [] m_del;

	m_x = m_del = 0;
	m_d = m_m = 0;

}

#endif
