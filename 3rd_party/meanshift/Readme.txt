===============================================================================
Nonlinear Mean Shift over Riemannian Manifolds
Author: Raghav Subbarao
Robust Image Understanding Laboratory, Rutgers University
===============================================================================

Implements the Nonlinear Mean Shift algorithm:
R. Subbarao, P. Meer, "Nonlinear Mean Shift",
Submitted to, IEEE Transactions on Pattern Analysis and MAchine Intelligence

Examples for using the base class for mean shift over SE(3), the essential manifold anf the Grassmann Manifold are provided in the code. The base case Euclidean Mean Shift has also been implemented.

Using the binary:

Euclidean:
	newmeanshift -euc [DATAFILE] [band width]


SE(3):
	newmeanshift -se3 [DATAFILE] [band width] [Translation scaling]

	The translation scaling can be used to scale the units of translation. This will default to 25 if nothing is given.


Essential:
	newmeanshift -ess [DATAFILE] [band width]

Grassmann:
	newmeanshift -gmn [DATAFILE] [band width] [n] [k]
	The [n] and [k] are the parameters of the Grassmann manifold.

DATAFILE format (see also the examples in MeanShift/Data/):
nRows nColumns
input data points, each row is a point.

The pruned modes are written out into a file name prunedModes.txt

Extending mean shift to new manifolds:
	Derive a new class from the CGeometry abstract Class for performing operations over manifolds. Three pure virutal 	functions have to be defined for each concrete subclass, expm(manifold exponential), logm(manifold log) and 	g(Riemannian inner product).
	Derive a new class from the CPointSet class which is a contained for the data. Any perprocessing can be done while 		loading points into this class.
	The mean shift function can then be used to perform mean shift.


Raghav Subbarao
rsubbara@caip.rutgers.edu

Robust Image Understanding Laboratory
www.caip.rutgers.edu/riul