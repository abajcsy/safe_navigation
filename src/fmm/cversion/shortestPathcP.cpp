#include <math.h>
#include "matrix.h"
#include "mex.h"   //--This one is required
#include <float.h>
#include <stdio.h>
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

double BilinearInterp(double x, double y, int N, double L, double** u, double numInfty) {
	double dx, a, b;
	int xl, xu, yl, yu;
	double interpVal;
	double LD,LU,RD,RU;
	
	// grid spacing
	dx = (double) 2.0*L/(N-1);
	
	// indices of the box
	xl = (int)floor( (x+L)/dx );
	xu = (int) ceil( (x+L)/dx );
	yl = (int)floor( (y+L)/dx );
	yu = (int) ceil( (y+L)/dx );
	//printf("%d %d %d %d \n", xl, xu, yl, yu);
    
	if (xl < 0 || xu > N-1 || yl < 0 || yu > N-1) {
		return numInfty;
	}
	
	// location of (x,y) relative to (xl, yl)
	a = (double) (x+L)/dx - xl; // in [0,1]
	b = (double) (y+L)/dx - yl; // in [0,1]
	//printf("%f %f \n", a, b);
	
	if (a>1.0 || a<0.0 || b>1.0 || b<0.0) {
		printf("a and b are not in [0,1]! a=%f, b=%f \n", a, b);
	}
	
    LD = u[xl][yl]; //if (LD >= numInfty) {return numInfty;}
    LU = u[xl][yu]; //if (LU >= numInfty) {return numInfty;}
    RD = u[xu][yl]; //if (RD >= numInfty) {return numInfty;}
    RU = u[xu][yu]; //if (RU >= numInfty) {return numInfty;}
    
	if (LD*LU*RD*RU >= numInfty) {
		int xx, yy;
		xx = (int)floor((x+L)/dx + 0.5);
		yy = (int)floor((y+L)/dx + 0.5);
		interpVal = u[xx][yy];
		// printf("(%d, %d)", xx, yy);
	} else {
		// bilinear interpolation
		interpVal =             (1-a)*(1-b)*LD;
		interpVal = interpVal + (1-a)*(  b)*LU;
		interpVal = interpVal + (  a)*(1-b)*RD;
		interpVal = interpVal + (  a)*(  b)*RU;
	}
	
	return interpVal;
}

void computePath(double **u, double *pathx, double *pathy, int N, double L, int T, double** P1, double** P2, double **speed, double numInfty) {
int tstep; // time step variable
double dirx, diry; // directions
double dx, dt; // discretizations
double x, y; // positions
double speedij, P1ij, P2ij;
double uij;
double xt, yt, dirxt, diryt; // temporary positions and directions

x = pathx[0];
y = pathy[0];


speedij = BilinearInterp(x,	y, N, L, speed, numInfty);
P1ij 	= BilinearInterp(x,	y, N, L, P1, numInfty);
P2ij 	= BilinearInterp(x,	y, N, L, P2, numInfty);

dx = 2.0*(double)L/((double)N-1.0);
dt = 0.5*dx/speedij;

for (tstep=1; tstep<T; tstep++) { // HEUN'S METHOD
	// ----- FIRST STEP -----
	// Directions
	dirx = -P1ij/sqrt(P1ij*P1ij + P2ij*P2ij);
	diry = -P2ij/sqrt(P1ij*P1ij + P2ij*P2ij);
	
	// Update position

	xt = x + speedij*dirx*dt;
	yt = y + speedij*diry*dt;
	
	// ----- SECOND STEP -----
	P1ij = BilinearInterp(xt, yt, N, L, P1, numInfty);
	P2ij = BilinearInterp(xt, yt, N, L, P2, numInfty);
	dirxt = -P1ij/sqrt(P1ij*P1ij + P2ij*P2ij);
	diryt = -P2ij/sqrt(P1ij*P1ij + P2ij*P2ij);
	
	x = x + 0.5*dt* (speedij*dirx + speedij*dirxt);
	y = y + 0.5*dt* (speedij*diry + speedij*diryt);
	
	
	pathx[tstep] = x;
	pathy[tstep] = y;

	uij 	= BilinearInterp(x,	y, N, L, u, numInfty);
	if (uij < dt) { 
		printf("Very close to target!\n");
		return; 
	}
	
	// Update time increment
	speedij = BilinearInterp(x,	y, N, L, speed, numInfty);
	P1ij 	= BilinearInterp(x,	y, N, L, P1, numInfty);
	P2ij 	= BilinearInterp(x,	y, N, L, P2, numInfty);
	dt 	= 0.5*dx/speedij;
}
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //---Inside mexFunction---
    //Declarations
	double *uValues;
	double **u;
	double *P1Values, *P2Values;
	double **P1, **P2;
    const int *dims; 
	int N; 		// total number of grid points in each dimension
	double *pathxValues, *pathyValues;
	double *pathx, *pathy;
    int i,j; // ith grid in x, jth grid in y
	double L;
	double *speedValues; // Maximum speed
	double **speed;
	int T; // maximum number of time points
	double numInfty;
	
    //Get the input
	uValues 		= mxGetPr(prhs[0]);
	speedValues 	= mxGetPr(prhs[1]);
	pathxValues 	= mxGetPr(prhs[2]);
	pathyValues		= mxGetPr(prhs[3]);
	P1Values 		= mxGetPr(prhs[4]);
	P2Values 		= mxGetPr(prhs[5]);
	L 				= mxGetScalar(prhs[6]);
	numInfty 		= mxGetScalar(prhs[7]);

	// Dimensions
    dims    = mxGetDimensions(prhs[0]);
	N       = dims[0];
	dims    = mxGetDimensions(prhs[2]); 
	T       = dims[0];
	
    // memory allocation
	u 	= (double **) malloc ( N * sizeof(double*));
	speed 	= (double **) malloc ( N * sizeof(double*));
	P1	 = (double **) malloc ( N * sizeof(double*));
	P2	 = (double **) malloc ( N * sizeof(double*));
	
	for (i=0;i<N;i++){
		u[i]    = (double *) malloc (N * sizeof(double));
		speed[i]    = (double *) malloc (N * sizeof(double));
		P1[i]  = (double *) malloc (N * sizeof(double));
		P2[i]  = (double *) malloc (N * sizeof(double));
    }

    // Initialize u
    for (i=0; i < N; i++) {
		for (j=0; j < N; j++) {
			u[i][j] = uValues[j*N+i];
			speed[i][j] = speedValues[j*N+i];
			P1[i][j] = P1Values[j*N+i];
			P2[i][j] = P2Values[j*N+i];
        }
	}

	// Allocate memory for path
	pathx =  (double*)malloc(T*sizeof(double));
	pathy =  (double*)malloc(T*sizeof(double));
	
	// Initialize path
	for (i=0; i<T; i++) {
		pathx[i] = pathxValues[i];
		pathy[i] = pathyValues[i];
	}
    
	// Compute path
	computePath(u, pathx, pathy, N, L, T, P1, P2, speed, numInfty);
   
	// Send result to output
	for (i=0; i<T; i++) {
		pathxValues[i] = pathx[i];
		pathyValues[i] = pathy[i];
	}	
   
    // free memory;
	for(i=0; i<N; i++){
		free(u[i]);
		free(speed[i]);
		free(P1[i]);
		free(P2[i]);
  	}
	free(u);
	free(speed);
	free(P1);
	free(P2);
   
    free(pathx);
	free(pathy);
}
