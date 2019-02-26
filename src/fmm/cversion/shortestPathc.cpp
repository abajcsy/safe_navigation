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

void computePath(double** u, double *pathx, double *pathy, int N, double L, int T, double **speed, double numInfty) {
int tstep; // time step variable
double uxm, uxy, uxp, uym, uyp; // values
double duxm, dux, duxp, duym, duy, duyp;   // derivatives
double dirx, diry; // directions
double dx, dt; // discretizations
double x, y; // positions
double speedij;
double ssize; // stencil size

// "x" stencil
double umm, upp, ump, upm;
double dumm, dupp, dump, dupm, duxy, duyx;
double dirxy, diryx;

x = pathx[0];
y = pathy[0];

speedij = BilinearInterp(x,	y, N, L, speed, numInfty);

dx = 2.0*(double)L/((double)N-1);
dt = 0.5*dx/speedij;

ssize = 0.5;
uxy	= BilinearInterp(x, 	y, N, L, u, numInfty);
// "+" stencil
uxm = BilinearInterp(x-ssize*dx, 	y, N, L, u, numInfty);
uxp = BilinearInterp(x+ssize*dx, 	y, N, L, u, numInfty);
uym = BilinearInterp(x,	y-ssize*dx, 	N, L, u, numInfty);
uyp = BilinearInterp(x,	y+ssize*dx, 	N, L, u, numInfty);

for (tstep=1; tstep<T; tstep++) {
	
	// ----- COMPUTE DIRECTION -----
	// Gradient in the x direction
	duxm = uxy - uxm;
	duxp = uxp - uxy;

	if (duxm * duxp > 0.0) {
		if (uxp > uxm) { dux = duxm;}
		else           { dux = duxp;}
	} else {
		if (duxm > 0.0) {
			if (uxp > uxm) 	{ dux = duxm; }
			else           	{ dux = duxp; }
		} else 				{ dux = 0.0; }
	}

	// Gradient in y direction
	duym = uxy - uym;
	duyp = uyp - uxy;

	if (duym * duyp > 0.0) {
		if (uyp > uym) 	{ duy = duym; }
		else            { duy = duyp; }
	} else {
		if (duym > 0.0) {
			if (uyp > uym) 	{ duy = duym; }
			else            { duy = duyp; }
		} else              { duy = 0.0; }
	}

	// Directions
	if (dux*dux + duy*duy <= 0.0) { // "+" stencil doesn't lead anywhere, try "x" stencil
		// printf("(uxy,uxm,uxp,uym,uyp)=(%f,%f,%f,%f,%f)\n",uxy,uxm,uxp,uym,uyp);
		
		// "x" stencil
		umm = BilinearInterp(x-ssize*dx, 	y-ssize*dx, N, L, u, numInfty);
		ump = BilinearInterp(x-ssize*dx, 	y+ssize*dx, N, L, u, numInfty);
		upm = BilinearInterp(x+ssize*dx,	y-ssize*dx, 	N, L, u, numInfty);
		upp = BilinearInterp(x+ssize*dx,	y+ssize*dx, 	N, L, u, numInfty);		
		
		// y=x direction
		dumm = uxy - umm;
		dupp = upp - uxy;
		
		if (dumm * dupp > 0.0) {
			if (upp > umm) { duxy = dumm;}
			else           { duxy = dupp;}
		} else {
			if (dumm > 0.0) {
				if (upp > umm) 	{ duxy = dumm; }
				else           	{ duxy = dupp; }
			} else 				{ duxy = 0.0; }
		}
	
		// y=-x direction
		dump = ump-uxy;
		dupm = uxy-upm;
		if (dump * dupm > 0.0) {
			if (ump > upm) { duyx = dupm;}
			else           { duyx = dump;}
		} else {
			if (dupm > 0.0) {
				if (ump > upm) 	{ duyx = dupm; }
				else           	{ duyx = dump; }
			} else 				{ duyx = 0.0; }
		}
		
		if (duxy*duxy + duyx*duyx <= 0.0) { // Both "+" and "x" stencils lead nowhere, we're stuck
			printf("tstep = %d. Can't go anywhere!\n", tstep);
			return;
		} else {
			dirxy = -duxy / sqrt(duxy*duxy + duyx*duyx);
			diryx = -duyx / sqrt(duxy*duxy + duyx*duyx);
		}
		
		// Convert to x-y direction
		dirx = 1.0/sqrt(2.0)*(dirxy - diryx);
		diry = 1.0/sqrt(2.0)*(dirxy + diryx);
		
		dirx = dirx / sqrt(dirx*dirx + diry*diry); // Redundant, but makes sure direction is normalized
		diry = diry / sqrt(dirx*dirx + diry*diry);
		
	} else {
		dirx = -dux / sqrt(dux*dux + duy*duy);
		diry = -duy / sqrt(dux*dux + duy*duy);
	}
	
	// Update position
	x = x + speedij*dirx*dt;
	y = y + speedij*diry*dt;
	
	pathx[tstep] = x;
	pathy[tstep] = y;

	uxy 	= BilinearInterp(x, 	y, N, L, u, numInfty);

	// printf("uxy=%f\n", uxy);
	if (uxy < dt) { 
		printf("Very close to target!\n");
		return; 
	}
	
	uxm = BilinearInterp(x-ssize*dx, 	y, N, L, u, numInfty);
	uxp = BilinearInterp(x+ssize*dx, 	y, N, L, u, numInfty);
	uym = BilinearInterp(x,	y-ssize*dx, 	N, L, u, numInfty);
	uyp = BilinearInterp(x,	y+ssize*dx, 	N, L, u, numInfty);	
	
	// Update time increment
	speedij = BilinearInterp(x,	y, N, L, speed, numInfty);
	dt = 0.5*dx/speedij;
}
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //---Inside mexFunction---
    //Declarations
    double *uValues;
    double **u;
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
	L 				= mxGetScalar(prhs[4]);
	numInfty 		= mxGetScalar(prhs[5]);

	//numInfty = 100000.0;
	
	// Dimensions
    dims    = mxGetDimensions(prhs[0]);
	N       = dims[0];
	dims    = mxGetDimensions(prhs[2]); 
	T       = dims[0];
	
    // memory allocation
	u = (double **) malloc ( N * sizeof(double*));
	speed    = (double **) malloc ( N * sizeof(double*));

	
	for (i=0;i<N;i++){
		u[i] = (double *) malloc (N * sizeof(double));
		speed[i]    = (double *) malloc (N * sizeof(double));
    }

    // Initialize u
    for (i=0; i < N; i++) {
		for (j=0; j < N; j++) {
			u[i][j] 	= uValues[j*N+i];
			speed[i][j] = speedValues[j*N+i];
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
	computePath(u, pathx, pathy, N, L, T, speed, numInfty);
   
	// Send result to output
	for (i=0; i<T; i++) {
		pathxValues[i] = pathx[i];
		pathyValues[i] = pathy[i];
	}	
   
    // free memory;
	for(i=0; i<N; i++){
		free(u[i]);
		free(speed[i]);
  	}
	
	free(u);
	free(speed);
   
    free(pathx);
	free(pathy);
}
