
#define FILTERBRUTEFORCE 0
#define FILTEROPTIMIZED  1

#define MSFILTER_STEP_THRESH	0.0001

#define NODE_MULTIPLE 50

//#define _USE_MASK_

enum EKernelType {
	EEpanechnikovKernel = 0,
	EGaussianKernel = 1
};

#ifndef min
#define min(a,b) ((a) <= (b) ? (a) : (b))
#endif
#ifndef max
#define max(a,b) ((a) >= (b) ? (a) : (b))
#endif