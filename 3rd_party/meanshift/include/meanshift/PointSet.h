/*******************************************************

                 Point Set Class
	=============================================

	This is a container class for data points stored as a
	data array.

Implemented by Raghav Subbarao
********************************************************/

#ifndef _C_POINT_SET_
#define _C_POINT_SET_

#include <assert.h>
#include <string.h>
#include <stdio.h>

// needed in selectPoints
#ifndef UNIX
#define drand48() (rand() * 1.0 / RAND_MAX)
#endif

template<typename T> class CPointSet{

public:

	/*/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/
	/* Class Constructor and Destructor */
	/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/

	// Empty Constructor
	CPointSet();

	// Standard Constructor
	// d = dimension of data
	// n = number of points to allocate memory for.
	CPointSet(int d, int m, int n);

	// Standard Constructor with data
	// d = dimension of data
	// n = number of points to allocate memory for.
	// data = data to be copied.
	CPointSet(int d, int m, int n, T *data);

	// Virtual Destructor
	virtual ~CPointSet();
	void clearData();

	/*/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/
	/*				Other Public Methods			 */
	/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/

	int size() const {return m_ps_n; };
	int dimension() const {return m_ps_dim; };

	// go to first point
	void first() { m_ps_loc = 0; };

	// move to desired location
	void move(int i) { assert(i < m_ps_n); if(i < m_ps_n) m_ps_loc = i; };

	// move to next location. prefix operator
	CPointSet& operator++(){ if(m_ps_loc < m_ps_n - 1) m_ps_loc++; return *this; };

	// move to previous location. prefix operator
	CPointSet& operator--(){ if(m_ps_loc > 0) m_ps_loc--; };

	// Pointers to current location.
	// Note: accesses data as is!!!
	inline operator const T* () const{ return m_ps_points + m_ps_loc * m_ps_dim; };
	inline const T* operator[](int i) const { return m_ps_points + i * m_ps_dim; };

    	
	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	returnPoint							|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Returns the data point at the current location.	|//
	//|   The data format is converted from the internal	|//
	//|   representation to the external representation and	|//
	//|   written into x. Uses the virtual function			|//
	//|   returnPoint(x, i).								|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <const T* x>										|//
	//|   An array of T which contains the point on exit.	|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		returnPoint(x)									|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	void returnPoint(T *x) { returnPoint(x, m_ps_loc); };
	


	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	getPoint							|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Copies the data point at the required location	|//
	//|   into the array x. This array is assumed to have	|//
	//|   been allocated already. This function returns the	|//
	//|   point in its internal representation.				|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <const T* x>										|//
	//|   An array of T which contains the point on exit.	|//
	//|														|//
	//|   <int i>											|//
	//|   Location of required data point.					|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		getPoint(x, i)									|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	void getPoint(T *x, int i = -1) const { if(i == -1) i = m_ps_loc; memcpy(x, m_ps_points + i * m_ps_dim, m_ps_dim * sizeof(T)); };




	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	setPoint							|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Copies the data point at x to the required place	|//
	//|   specified by i. The data in x is assumed to be in	|//
	//|   the internal data representation.					|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <const T* x>										|//
	//|   An array of T which contains the point on exit.	|//
	//|														|//
	//|   <int i>											|//
	//|   Location of required data point.					|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		setPoint(x, i)									|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	void setPoint(T *x, int i = -1){ if(i == -1) i = m_ps_loc; memcpy(m_ps_points + i * m_ps_dim, x, m_ps_dim * sizeof(T)); };




	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	addPoint							|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Adds a new data point to the class. The new point	|//
	//|   is given by x and it is assumed to be in the		|//
	//|   internal data representation.						|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <const T* x>										|//
	//|   An array of T which contains the point on exit.	|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		addPoint(x)										|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	void addPoint(T *x){ if(m_ps_n < m_ps_nmax) memcpy(m_ps_points + (m_ps_n++) * m_ps_dim, x, sizeof(T) * m_ps_dim); };
		
	void addReference(CPointSet& ps);
	void removeReference();
	

	// TODO: implement this
	int selectPoints(double frac, int jump);

	void write(char *filename);
	void write(char *filename, double *kernelDensities);

	/*/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/
	/*				Pure Virtual Methods			 */
	/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/

	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	returnPoint							|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Returns the data point at the current location.	|//
	//|   The data format is converted from the internal	|//
	//|   representation to the external representation and	|//
	//|   written into x. Uses the virtual function			|//
	//|   returnPoint(x, i).								|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <const T* x>										|//
	//|   An array of T which contains the point on exit.	|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		returnPoint(x)									|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	// load point. do any preprocessing here.
	inline virtual void loadPoint(T *x) = 0;

	/*\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\\--/
	//<---------------------------------------------------->|//
	//|														|//
	//|	Method Name:	returnPoint							|//
	//|   ============										|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Description:										|//
	//|	============										|//
	//|														|//
	//|   Returns the data point at the current location.	|//
	//|   The data format is converted from the internal	|//
	//|   representation to the external representation and	|//
	//|   written into x. Uses the virtual function			|//
	//|   returnPoint(x, i).								|//
	//|														|//
	//|   The following arguments must be provided:			|//
	//|														|//
	//|   <const T* x>										|//
	//|   An array of T which contains the point on exit.	|//
	//|														|//
	//<---------------------------------------------------->|//
	//|														|//
	//|	Usage:												|//
	//|   ======											|//
	//|		returnPoint(x)									|//
	//|														|//
	//<---------------------------------------------------->|//
	//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||//--\\||\*/
	// return point.
	// do any post processing here.
	inline virtual void returnPoint(T *x, int n) = 0;

protected:
	
	// DATA
	int m_ps_dim;			// Dimensionality of data in internal representation
	int m_ps_m;				// Dimensionality of data in external representation
	int m_ps_n;				// Number of point present
	int m_ps_nmax;			// Number of points allocated for

	int m_ps_loc;			// current location

	int m_ps_refcounter;	// reference counter

	T *m_ps_points;	// data

};


template<typename T> CPointSet<T>::CPointSet():
m_ps_dim(0), m_ps_m(0), m_ps_n(0), m_ps_nmax(0), m_ps_loc(0), m_ps_refcounter(0), m_ps_points(0)
{}

template<typename T> CPointSet<T>::CPointSet(int d, int m, int n):
m_ps_dim(d), m_ps_m(m), m_ps_nmax(n), m_ps_n(0), m_ps_refcounter(1), m_ps_loc(0)
{

	m_ps_points = new T[m_ps_nmax * m_ps_dim];

}

template<typename T> CPointSet<T>::CPointSet(int d, int m, int n, T *data):
m_ps_dim(d), m_ps_m(m), m_ps_nmax(n), m_ps_n(n), m_ps_refcounter(1), m_ps_loc(0)
{

	m_ps_points = new T[m_ps_nmax * m_ps_dim];
	memcpy(m_ps_points, data, sizeof(T) * m_ps_nmax * m_ps_dim);

}


template<typename T> CPointSet<T>::~CPointSet(){

	clearData();

}

template<typename T> void CPointSet<T>::clearData(){

	if(m_ps_points)
		delete [] m_ps_points;
	m_ps_dim = m_ps_m = m_ps_n = m_ps_nmax = m_ps_loc = m_ps_refcounter = 0;
	m_ps_points = 0;

}

template<typename T> void CPointSet<T>::addReference(CPointSet& ps){

	if(this){
		m_ps_refcounter--;
		if(!m_ps_refcounter)
			clearData();
	}
	
	*this = ps;
	this->m_ps_refcounter++;

}

template<typename T> void CPointSet<T>::removeReference(){

	if(this){
		m_ps_refcounter--;
		if(!m_ps_refcounter)
			clearData();
	}

}

template<typename T> void CPointSet<T>::write(char *filename){

	int i, j;
	FILE* fu = fopen(filename, "w");
	fprintf(fu, "%d %d\n", m_ps_n, m_ps_dim);
	T *p = new T[m_ps_m];
	
	for (i = 0; i < m_ps_n; i++){
		returnPoint(p, i);
		for (j = 0; j < m_ps_m; j++)
			fprintf(fu, "%f ", p[j]);
		fprintf(fu, "%f\n");
	}
	fclose(fu);

	delete [] p;

}

template<typename T> void CPointSet<T>::write(char *filename, double *kernelDensities){

	int i, j;
	FILE* fu = fopen(filename, "w");
	fprintf(fu, "%d %d\n", m_ps_n, m_ps_dim);
	T *p = new T[m_ps_m];
	
	for (i = 0; i < m_ps_n; i++){
		returnPoint(p, i);
		for (j = 0; j < m_ps_m; j++)
			fprintf(fu, "%f ", p[j]);
		fprintf(fu, "%f\n", kernelDensities[i]);
	}
	fclose(fu);

	delete [] p;

}


#endif