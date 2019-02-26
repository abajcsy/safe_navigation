#include <math.h>
#include "matrix.h"
#include "mex.h"   //--This one is required

#define PI 3.1415926535
#define SQRT2 1.414213562373
#define SQRT5 2.236067977499

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define abs(a) ((a > 0) ? a : (-1)*a)
#define sgn(a) (a>=0 ? 1 : -1)
#define MAXVAL 999999

struct quadruple {
	int x;
	int y;
	int th;
	double value;
};

struct heapAugmented {
	int Ni;
    int Nj;
    int Nk;
	int numOfElements;
	struct quadruple *heap;
	int ***hashMap;
};

struct heapAugmented* build(int Ni,int Nj,int Nk){
	struct heapAugmented *hh;
	int i,j,k;
	
	hh = (struct heapAugmented *)malloc(sizeof(struct heapAugmented));
	hh->Ni=Ni;
    hh->Nj=Nj;
    hh->Nk=Nk;
	hh->numOfElements = 0;
	hh->heap = (struct quadruple *)malloc((Ni*Nj*Nk+1)*sizeof(struct quadruple));
	struct quadruple firstQuad;
	firstQuad.x = 0;
	firstQuad.y = 0;
	firstQuad.th = 0;
	firstQuad.value = -MAXVAL;
	(hh->heap)[0] = firstQuad;
	hh->hashMap = (int ***)malloc(Ni*sizeof(int **));
	for(i=0; i< Ni; i++){
		*(hh->hashMap + i) = (int **)malloc(Nj*sizeof(int *));
        for(j=0;j<Nj;j++){
            *(*(hh->hashMap + i)+j) = (int *)malloc(Nk*sizeof(int));
        }
	}
    //initialize hash table! -rrtakei
    for(i=0; i< Ni; i++)
        for(j=0; j< Nj; j++)
            for(k=0;k<Nk;k++)
                (hh->hashMap)[i][j][k] = 0;
    
	return hh;
}

void freeHeap(struct heapAugmented* hh){
    int i,j;
    for(i=0;i<hh->Ni;i++){
        for(j=0;j<hh->Nj;j++){
            free((hh->hashMap)[i][j]);
        }
        free((hh->hashMap)[i]);
    }
    free(hh->hashMap);
    free(hh->heap);
    free(hh);
}

void printHeapAugmented(struct heapAugmented *h){
    int i;
    
	if(h->numOfElements == 0){
		printf("This is an empty heap");
	}
	else{
        printf("***");
		printf("The heap now has %d elements. The heapArray looks like the following:  \n",h->numOfElements);
		
		for(i=1;i <= h->numOfElements;i++){
			printf("(%d,%d,%d,%f) at position %d;\n",(h->heap[i]).x,(h->heap[i]).y,(h->heap[i]).th,(h->heap[i]).value,i);
			//printf("The hashMap at (%d,%d,%d) points to the %d th element in the hashArray.\n",(h->heap)[i].x,((h->heap)[i]).y,(h->heap[i]).th,(h->hashMap)[((h->heap[i])).x][((h->heap[i])).y][(h->heap[i]).th]);
		}
	}
}

void printHashMap(struct heapAugmented *h){
	int i,j,k;
	for(i=0;i<h->Ni;i++){
		for(j=0;j<h->Nj;j++){
            for(k=0;k<h->Nk;k++)
                //printf("(%d,%d,%d): %f\n",i,j,k,(h->hashMap)[i][j][k]);
                printf("(%d,%d,%d): %f\n",i,j,k,((h->heap)[1]).value);
		}
	}
}

// name changed by rrtakei
double minValue(struct heapAugmented *h) {
	return ((h->heap)[1]).value;
}

// added by rrtakei
int minIndexI(struct heapAugmented *h) {
	return ((h->heap)[1]).x;
}
// added by rrtakei
int minIndexJ(struct heapAugmented *h) {
	return ((h->heap)[1]).y;
}

int minIndexTh(struct heapAugmented *h){
    return ((h->heap)[1]).th;
}

void add(int i,int j,int th,double value,struct heapAugmented *h){
    /*Assumption: When you call add,the (i,j) must be distinct from each of the pairs already in the heap structure,
	 otherwise, you should call update.
	 */
    if((h->hashMap)[i][j][th] == 0){
        /*
         * It appears to me that C initializes everything in the hashMap to be 0.
         */
        struct quadruple newQuadruple;
        newQuadruple.x = i;
        newQuadruple.y = j;
        newQuadruple.th=th;
        
        newQuadruple.value = value;
        h->numOfElements = h->numOfElements + 1;
        int currentPosition=h->numOfElements;
		
        while(((h->heap)[currentPosition/2]).value >= value){
            (h->heap)[currentPosition] = (h->heap)[currentPosition/2];
            (h->hashMap)[((h->heap)[currentPosition]).x][((h->heap)[currentPosition]).y][((h->heap)[currentPosition]).th] = currentPosition;
            currentPosition = currentPosition/2;
        }
        (h->heap)[currentPosition] = newQuadruple;
        (h->hashMap)[i][j][th] = currentPosition;
    }
    else{
        printf("The element in (%d,%d) is already in the list, you cannot add them!\n",i,j);
    }
}

double removeMin(struct heapAugmented *h){
	if(h->numOfElements == 1){
		double minValue = (h->heap)[1].value;
		h->numOfElements = 0;
		(h->hashMap)[((h->heap)[1]).x][((h->heap)[1]).y][((h->heap)[1]).th] = 0;
		return minValue;
	}
	
	else{
		double minValue = (h->heap)[1].value;
		(h->hashMap)[((h->heap)[1]).x][((h->heap)[1]).y][((h->heap)[1]).th] = 0;
		struct quadruple newQuadruple = (h->heap)[h->numOfElements];
		(h->numOfElements)--;
		int currentPosition = 1;
		
		while(2*currentPosition <= h->numOfElements){
			//so long as the element updated has at least one child,loop
			if(2*currentPosition + 1 <= h->numOfElements){
				//the updated element has two children
				if((h->heap)[2*currentPosition].value < (h->heap)[2*currentPosition+1].value){
					if((h->heap)[2*currentPosition].value >= newQuadruple.value){
						break;
					}
					else{
						/*the left child of the updated element has smaller value than the right child
						 * of the updated element,in which case we shall bubble up the left child so that
						 * the left child will now have smaller or equal to value than both
						 */
						(h->heap)[currentPosition] = (h->heap)[2*currentPosition];
						(h->hashMap)[(h->heap)[currentPosition].x][(h->heap)[currentPosition].y][(h->heap)[currentPosition].th] = currentPosition;
						currentPosition = 2*currentPosition;
					}
				}
				else {
					if((h->heap)[2*currentPosition + 1].value >= newQuadruple.value){
						break;
					}
					else{
						/*the left child of the updated element has larger or equal to value than the right child
						 * of the updated element,in which case we shall bubble up the right child so that
						 * the right child will now have smaller or equal to value than both
						 */
						(h->heap)[currentPosition] = (h->heap)[2*currentPosition + 1];
						(h->hashMap)[(h->heap)[currentPosition].x][(h->heap)[currentPosition].y][(h->heap)[currentPosition].th] = currentPosition;
						currentPosition = 2*currentPosition +1 ;
					}
				}
				
			}
			else{
				//the updated element only has one child
				if(newQuadruple.value > (h->heap)[2*currentPosition].value){
					(h->heap)[currentPosition] = (h->heap)[2*currentPosition];
					(h->hashMap)[(h->heap)[currentPosition].x][(h->heap)[currentPosition].y][(h->heap)[currentPosition].th] = currentPosition;
					currentPosition = 2*currentPosition;
				}
				
				else { // else case modified by rrtakei
					break;
				}
				
			}
		}
		(h->heap)[currentPosition] = newQuadruple;
		(h->hashMap)[newQuadruple.x][newQuadruple.y][newQuadruple.th] = currentPosition; //we can make it more efficient by using pointer notation instead of array notation
		return minValue;
	}
}

void update(int i,int j,int th,double newValue,struct heapAugmented *h,int***  Nodes){
	int position = (h->hashMap)[i][j][th];
	if(position == 0){
		printf("Error 001: The value you want to update doesn't exist, (i,j,k) = (%d,%d,%d). Nodes = %f.\n",i,j,th,Nodes[i][j][th]);
	}
	else{
		double oldValue = (h->heap)[position].value;
		(h->heap)[position].value = newValue;
		if(newValue < oldValue){
			//you bubble the updated element up, if necessary
			struct quadruple newQuadruple = (h->heap)[position];
			int currentPosition = position;
			while(((h->heap)[currentPosition/2]).value >= newValue){
				(h->heap)[currentPosition] = (h->heap)[currentPosition/2];
				(h->hashMap)[((h->heap)[currentPosition]).x][((h->heap)[currentPosition]).y][((h->heap)[currentPosition]).th] = currentPosition;
				currentPosition = currentPosition/2;
			}
			(h->heap)[currentPosition] = newQuadruple;
			(h->hashMap)[i][j][th] = currentPosition;
		}
		else{
			//you bubble the updated element down,if necessary
			struct quadruple newQuadruple = (h->heap)[position];
			int currentPosition = position;
			while(2*currentPosition <= h->numOfElements){
				//so long as the element updated has at least one child,loop
				if(2*currentPosition + 1 <= h->numOfElements){
					//the updated element has two children
					if((h->heap)[2*currentPosition].value < (h->heap)[2*currentPosition+1].value){
						if((h->heap)[2*currentPosition].value >= newQuadruple.value){
							break;
						}
						else{
							/*the left child of the updated element has smaller value than the right child
							 * of the updated element,in which case we shall bubble up the left child so that
							 * the left child will now have smaller or equal to value than both
							 */
							(h->heap)[currentPosition] = (h->heap)[2*currentPosition];
							(h->hashMap)[(h->heap)[currentPosition].x][(h->heap)[currentPosition].y][(h->heap)[currentPosition].th] = currentPosition;
							currentPosition = 2*currentPosition;
						}
					}
					else {
						if((h->heap)[2*currentPosition + 1].value >= newQuadruple.value){
							break;
						}
						else{
							/*the left child of the updated element has larger or equal to value than the right child
							 * of the updated element,in which case we shall bubble up the right child so that
							 * the right child will now have smaller or equal to value than both
							 */
							(h->heap)[currentPosition] = (h->heap)[2*currentPosition + 1];
							(h->hashMap)[(h->heap)[currentPosition].x][(h->heap)[currentPosition].y][(h->heap)[currentPosition].th] = currentPosition;
							currentPosition = 2*currentPosition +1 ;
						}
                    }
					
				}
				else{
					//the updated element only has one child
					if(newQuadruple.value > (h->heap)[2*currentPosition].value){
						(h->heap)[currentPosition] = (h->heap)[2*currentPosition];
						(h->hashMap)[(h->heap)[currentPosition].x][(h->heap)[currentPosition].y][(h->heap)[currentPosition].th] = currentPosition;
						currentPosition = 2*currentPosition;
					}
					
					
					else { // else case modified by rrtakei
                        break;
					}
					
				}
			}
			(h->heap)[currentPosition] = newQuadruple;
			(h->hashMap)[newQuadruple.x][newQuadruple.y][newQuadruple.th] = currentPosition; //we can make it more efficient by using pointer notation instead of array notation
		}
	}
	
}

// added by rrtakei
double LinearInterpX(double x, int j, int k, const int* N, double L, double*** unext, double numInfty) {
    int Ni;
    double dx, a;
    int xl,xu;
    double LEFT, RIGHT;
            
    Ni = N[0];
    
    // grid spacing
	dx = (double) 2*L/(Ni-1);
    
    xl = (int)floor( (x+L)/dx );
	xu = (int) ceil( (x+L)/dx );
    if (xl < 0 || xu > Ni-1) {
		return numInfty;
	}
    a = (double) (x+L)/dx - xl; // in [0,1]

	if (a>1 || a<0) {
		printf("LinearInterpX: a is not in [0,1]! a=%f \n", a);
	} 
    
    LEFT  = unext[xl][j][k]; //if (LEFT  >= numInfty) {return numInfty;}
    RIGHT = unext[xu][j][k]; //if (RIGHT >= numInfty) {return numInfty;}
    
    return (1-a)*LEFT + a*RIGHT;
}

// added by rrtakei
double LinearInterpY(int i, double y, int k, const int* N, double L, double*** unext, double numInfty) {
    int Nj;
    double dy, a;
    int yl,yu;
    double UP, DOWN;
            
    Nj = N[1];
    
    // grid spacing
	dy = (double) 2*L/(Nj-1);
    
    yl = (int)floor( (y+L)/dy );
	yu = (int) ceil( (y+L)/dy );
    if (yl < 0 || yu > Nj-1) {
		return numInfty;
	}
	
    a = (double) (y+L)/dy - yl; // in [0,1]
 
	if (a>1 || a<0) {
		printf("LinearInterpY: a is not in [0,1]! a=%f \n", a);
	} 
    DOWN  = unext[i][yl][k]; //if (DOWN >= numInfty) {return numInfty;}
    UP    = unext[i][yu][k]; //if (UP   >= numInfty) {return numInfty;}
    
    return (1-a)*DOWN + a*UP;
}

double LinearInterpDiag(double ux, double uy, int i, int j, int k, const int* N, double L, double*** unext, double*** v, double numInfty) {
    int Ni, Nj, Nk;
    double dx, dy, dth;
    double r, psi;
    double theta;
    double dt;
    
    Ni        = N[0];
    Nj        = N[1];
    Nk        = N[2];
	
	// grid spacing
	dx = (double) 2*L/(Ni-1);
	dy = (double) 2*L/(Nj-1);
    dth= (double) 2*PI/(Nk);
    
    theta = k*dth;
    
    r   = dx*dy/(dx*abs(sin(theta)) + dy*abs(cos(theta)));
    psi = r*abs(cos(theta))/dx;
    
    dt = r/v[i][j][k];
    return uy*(1-psi) + ux*psi + dt;
}

// MO!!
double BilinearInterp(double x, double y, int k, const int* N, double L, double*** unext, double numInfty) {
	int Ni,Nj;
	double dx, dy, a, b;
	int xl, xu, yl, yu;
	double interpVal;
	double LD,LU,RD,RU;
	
	Ni        = N[0];
    Nj        = N[1];
	
	// grid spacing
	dx = (double) 2*L/(Ni-1);
	dy = (double) 2*L/(Nj-1);
	
	// indices of the box
	xl = (int)floor( (x+L)/dx );
	xu = (int) ceil( (x+L)/dx );
	yl = (int)floor( (y+L)/dy );
	yu = (int) ceil( (y+L)/dy );
	//printf("%d %d %d %d \n", xl, xu, yl, yu);
    
	if (xl < 0 || xu > Ni-1 || yl < 0 || yu > Nj-1) {
		return numInfty;
	}
	
	// location of (x,y) relative to (xl, yl)
	a = (double) (x+L)/dx - xl; // in [0,1]
	b = (double) (y+L)/dy - yl; // in [0,1]
	//printf("%f %f \n", a, b);
	
	if (a>1 || a<0 || b>1 || b<0) {
		printf("a and b are not in [0,1]! a=%f, b=%f \n", a, b);
	}
	
    // if any of the 4 neighbors are infinity, return infinity
    LD = unext[xl][yl][k]; //if (LD >= numInfty) {return numInfty;}
    LU = unext[xl][yu][k]; //if (LU >= numInfty) {return numInfty;}
    RD = unext[xu][yl][k]; //if (RD >= numInfty) {return numInfty;}
    RU = unext[xu][yu][k]; //if (RU >= numInfty) {return numInfty;}
    
    
	// bilinear interpolation
	interpVal =             (1-a)*(1-b)*LD;
	interpVal = interpVal + (1-a)*(  b)*LU;
	interpVal = interpVal + (  a)*(1-b)*RD;
	interpVal = interpVal + (  a)*(  b)*RU;
	
	return interpVal;
}

double updateU(double uxp, double uxm, double uyp, double uym, double utp, double utm, 
				int n, double theta, double h, double ht, double pijk, double vijk, double numInfty) {
				
	double ux, uy, ut, AC, AS, us, uss, R;
	
	R = 1.0/pijk;
	

	// choose upwind direction
	if (n > 0 ) {
		if 		(cos(theta) > 0){ ux = uxp; }
		else if (cos(theta) < 0){ ux = uxm; }
		else 					{ ux = 0; }
		
		if (sin(theta) > 0) 	{ uy = uyp; }
		else if (sin(theta) < 0){ uy = uym; }
		else 					{ uy = 0; }
		
	} else {
		if (cos(theta) < 0) 	{ ux = uxp; }
		else if (cos(theta) > 0){ ux = uxm; }
		else 					{ ux = 0;	}
		
		if (sin(theta) < 0) 	{ uy = uyp; }
		else if (sin(theta) > 0){ uy = uym; }
		else 					{ uy = 0; }		
	}

	// upwind for u_theta
	if (utp<utm) { ut = utp; } else { ut = utm;	}
	
	AC = abs(cos(theta));
	AS = abs(sin(theta));

	// general formula
	if (vijk > 0) {
		us = (AC*ux + AS*uy + R*h/ht*ut + h/vijk) / (R*h/ht + AC + AS);
		// special case when u_theta = 0
		uss = (AC*ux + AS*uy + h/vijk) / (AC + AS);
	} else {
		return numInfty;
	}
	
	if (us < uss) {
		return us;
	} else {
		return uss;
	}
}

double updateUdiag(double uNE, double uNW, double uSW, double uSE, double utp, double utm, 
				int n, double theta, double h, double ht, double pijk, double vijk, double numInfty) {
	double ux, uy, ut, AC, AS, us, uss, R;

	R = 1.0/pijk;
	
	// choose upwind direction
	if (n > 0) {
		if (cos(theta) + sin(theta) > 0) 		{ ux = uNE; }
		else if (cos(theta) + sin(theta) < 0) 	{ ux = uSW; }
		else 									{ ux = 0; }

		if (sin(theta) - cos(theta) > 0) 		{ uy = uNW; }
		else if (sin(theta) - cos(theta) < 0) 	{ uy = uSE; } 
		else 									{ uy = 0; }
	} else {
		if (cos(theta) + sin(theta) < 0) 		{ ux = uNE; }
		else if (cos(theta) + sin(theta) > 0) 	{ ux = uSW; }
		else 									{ ux = 0; }

		if (sin(theta) - cos(theta) < 0) 		{ uy = uNW; }
		else if (sin(theta) - cos(theta) > 0) 	{ uy = uSE; } 
		else 									{ uy = 0; }	
	}

	// upwind for u_theta
	if (utp<utm) { ut = utp; } else { ut = utm; }
    
    AC = cos(theta) + sin(theta);
    AS = sin(theta) - cos(theta);
    
	AC = abs(AC);
	AS = abs(AS);

    AC = AC/SQRT2;
    AS = AS/SQRT2;
    
	// scale h accordingly
	h = h*SQRT2;

	// general formula
	if (vijk > 0) {
		us = (AC*ux + AS*uy + R*h/ht*ut + h/vijk) / (R*h/ht + AC + AS);
		// special case when u_theta = 0
		uss = (AC*ux + AS*uy + h/vijk) / (AC + AS);
	} else {
		return numInfty;
	}
	
	if (us < uss) {
		return us;
	} else {
		return uss;
	}
}

double updateU16a(double uNEE,double uNNW,double uSSE,double uSWW, double utp, double utm, 
				int n, double theta, double h, double ht, double pijk, double vijk, double numInfty) {
	double t, ux, uy, ut, AC, AS, us, uss, R;
	R = 1.0/pijk;
	
	t = 0.463647609;

	// choose upwind direction
	if (n > 0) {
		if (cos(t)*cos(theta) + sin(t)*sin(theta) > 0) 			{ ux = uNEE; }
		else if (cos(t)*cos(theta) + sin(t)*sin(theta) < 0) 	{ ux = uSWW; }
		else 													{ ux = 0; }

		if (cos(t)*sin(theta) - sin(t)*cos(theta) > 0) 			{ uy = uNNW; }
		else if (cos(t)*sin(theta) - sin(t)*cos(theta) < 0) 	{ uy = uSSE; }
		else 													{ uy = 0; }
	} else {
		if (cos(t)*cos(theta) + sin(t)*sin(theta) < 0) 			{ ux = uNEE; }
		else if (cos(t)*cos(theta) + sin(t)*sin(theta) > 0) 	{ ux = uSWW; }
		else 													{ ux = 0; }

		if (cos(t)*sin(theta) - sin(t)*cos(theta) < 0) 			{ uy = uNNW; }
		else if (cos(t)*sin(theta) - sin(t)*cos(theta) > 0) 	{ uy = uSSE; }
		else 													{ uy = 0; }	
	}

	// upwind for u_theta
	ut = min(utp, utm);
    
    AC = cos(t)*cos(theta) + sin(t)*sin(theta);
    AS = cos(t)*sin(theta) - sin(t)*cos(theta);
    
	AC = abs(AC);
	AS = abs(AS);

	// scale h accordingly
	h = h*SQRT5;

	// general formula
	if (vijk > 0) {
		us = (AC*ux + AS*uy + R*h/ht*ut + h/vijk) / (R*h/ht + AC + AS);
		// special case when u_theta = 0
		uss = (AC*ux + AS*uy + h/vijk) / (AC + AS);
	} else {
		return numInfty;
	}
	
	if (us < uss) {
		return us;
	} else {
		return uss;
	}
}

double updateU16b(double uNNE,double uNWW,double uSEE,double uSSW, double utp, double utm, 
				int n, double theta, double h, double ht, double pijk, double vijk, double numInfty) {
	double u_ijk, t, ux, uy, ut, AC, AS, us, uss, R;
	R = 1/pijk;
	
	t = 1.107148717794;

	// choose upwind direction
	if (n>0) {
		if (cos(t)*cos(theta) + sin(t)*sin(theta) > 0)		{ ux = uNNE; } 
		else if (cos(t)*cos(theta) + sin(t)*sin(theta) < 0) { ux = uSSW; }
		else												{ ux = 0; }

		if (cos(t)*sin(theta) - sin(t)*cos(theta) > 0)		{ uy = uNWW; }
		else if (cos(t)*sin(theta) - sin(t)*cos(theta) < 0)	{ uy = uSEE; }
		else                                        		{ uy = 0; }
	} else {
		if (cos(t)*cos(theta) + sin(t)*sin(theta) < 0)		{ ux = uNNE; } 
		else if (cos(t)*cos(theta) + sin(t)*sin(theta) > 0) { ux = uSSW; }
		else												{ ux = 0; }

		if (cos(t)*sin(theta) - sin(t)*cos(theta) < 0)		{ uy = uNWW; }
		else if (cos(t)*sin(theta) - sin(t)*cos(theta) > 0)	{ uy = uSEE; }
		else                                        		{ uy = 0; }	
	}

	// upwind for u_theta
	ut = min(utp, utm);
    
    AC = cos(t)*cos(theta) + sin(t)*sin(theta);
    AS = cos(t)*sin(theta) - sin(t)*cos(theta);
    
	AC = abs(AC);
	AS = abs(AS);

	// scale h accordingly
	h = h*SQRT5;
	
	// general formula
	if (vijk > 0) {
		us = (AC*ux + AS*uy + R*h/ht*ut + h/vijk) / (R*h/ht + AC + AS);
		// special case when u_theta = 0
		uss = (AC*ux + AS*uy + h/vijk) / (AC + AS);
	} else {
		return numInfty;
	}

	if (us < uss) {
		return us;
	} else {
		return uss;
	}	
}

double updateFormulaPDE(int i, int j, int k, const int* N, double L, double*** unext, 
						double*** v, double*** p, int*** sc, double numInfty, int dubin) {
	/*
	Finite difference
		uses all 4 grid orientations for updating
			1. [1 0], [0 1]
			2. [1 1], [1 -1]
			3. [2 1], [-1 2]
			4. [1 2], [-2 1]
	*/

	/* 
	Limitations:	v_forward = v_backward = v
					dx = dy = h
	*/
	int Ni,Nj,Nk;
	double theta, vijk, pijk;
	double dx, dy, dth;
	double u1a, u2a, u3a, u4a, u1b, u2b, u3b, u4b;
	double uxp,uyp,utp,utm,uxm,uym,uNW,uNE,uSW,uSE,
			uNEE,uNNW,uSWW,uSSE,uNNE,uNWW,uSSW,uSEE;
	double ureturna, ureturnb, ureturn;

	double deltax, deltay, deltath;
	double unew;
	double dt;
	double xi, yj;
	int n;
	double ux, uy, uth;
	double t;
	
	u1a = numInfty;	u2a = numInfty;	u3a = numInfty;	u4a = numInfty;
	u1b = numInfty;	u2b = numInfty;	u3b = numInfty;	u4b = numInfty;
	
	Ni        = N[0];    Nj        = N[1];    Nk        = N[2];
	
	// grid spacing
	dx  = 2.0*L/(double)(Ni-1);	dy  = 2.0*L/(double)(Nj-1);	dth = 2.0*PI/(double)(Nk);
	
	theta = k*dth;	vijk = v[i][j][k];	pijk = p[i][j][k];
	
	if (i < Ni-1) 	{ uxp = unext[i+1][j][k]; } else { uxp = numInfty; }
	if (i > 0) 		{ uxm = unext[i-1][j][k]; } else { uxm = numInfty; }
	if (j < Nj-1) 	{ uyp = unext[i][j+1][k]; } else { uyp = numInfty; }
	if (j > 0) 		{ uym = unext[i][j-1][k]; } else { uym = numInfty; }
	
	if (i<Ni-1&&j<Nj-1)	{ uNE = unext[i+1][j+1][k]; } else { uNE = numInfty; }
	if (i>0 && j<Nj-1)	{ uNW = unext[i-1][j+1][k]; } else { uNW = numInfty; }
	if (i>0 && j>0)		{ uSW = unext[i-1][j-1][k]; } else { uSW = numInfty; }
	if (i<Ni-1&&j>0)	{ uSE = unext[i+1][j-1][k]; } else { uSE = numInfty; }
	
	if (i<Ni-1&&j<Nj-2) { uNNE = unext[i+1][j+2][k]; } else { uNNE = numInfty; }
	if (i<Ni-2&&j>0)	{ uSEE = unext[i+2][j-1][k]; } else { uSEE = numInfty; }
	if (i>0 && j>1) 	{ uSSW = unext[i-1][j-2][k]; } else { uSSW = numInfty; }
	if (i>1 && j<Nj-1) 	{ uNWW = unext[i-2][j+1][k]; } else { uNWW = numInfty; }

	if (i<Ni-2&&j<Nj-1) { uNEE = unext[i+2][j+1][k]; } else { uNEE = numInfty; }
	if (i<Ni-1&& j>1 )	{ uSSE = unext[i+1][j-2][k]; } else { uSSE = numInfty; }
	if (i>1 && j>0 )	{ uSWW = unext[i-2][j-1][k]; } else { uSWW = numInfty; }
	if (i>0 && j<Nj-2) 	{ uNNW = unext[i-1][j+2][k]; } else { uNNW = numInfty; }
	
	// periodic boundary condition for theta
	if     (k==Nk-1) {
		utp = unext[i][j][0]; 
		utm = unext[i][j][k-1];
	} else if (k==0) {	
		utp = unext[i][j][k+1]; 
		utm = unext[i][j][Nk-1];
	} else {
		utp = unext[i][j][k+1]; 
		utm = unext[i][j][k-1];
	}
	
	ureturn = unext[i][j][k];
	
	u1a = updateU(uxp, uxm, uyp, uym, utp, utm, 1, theta, dx, dth, pijk, vijk, numInfty);
	u2a = updateUdiag(uNE, uNW, uSW, uSE, utp, utm, 1, theta, dx, dth, pijk, vijk, numInfty);	
	u3a = updateU16a(uNEE, uNNW, uSSE, uSWW,  utp,  utm, 1, theta, dx, dth, pijk, vijk, numInfty);
	u4a = updateU16b(uNNE, uNWW, uSEE, uSSW,  utp,  utm, 1, theta, dx, dth, pijk, vijk, numInfty);
	
	if (dubin == -1) {
	u1b = updateU(uxp, uxm, uyp, uym, utp, utm, -1, theta, dx, dth, pijk, vijk, numInfty);
	u2b = updateUdiag(uNE, uNW, uSW, uSE, utp, utm, -1, theta, dx, dth, pijk, vijk, numInfty);
	u3b = updateU16a(uNEE, uNNW, uSSE, uSWW,  utp,  utm, -1, theta, dx, dth, pijk, vijk, numInfty);
	u4b = updateU16b(uNNE, uNWW, uSEE, uSSW,  utp,  utm, -1, theta, dx, dth, pijk, vijk, numInfty);
	}
	
	ureturna = min(u1a,u2a);
	ureturna = min(u3a,ureturna);
	ureturna = min(u4a,ureturna);
	
	ureturnb = min(u1b,u2b);
	ureturnb = min(u3b,ureturnb);
	ureturnb = min(u4b,ureturnb);
	
	if (ureturna < ureturn || ureturnb < ureturn) {
		if (ureturna < ureturnb) {
			ureturn = ureturna;
			sc[i][j][k] = 1;
		} else {
			ureturn = ureturnb;
			sc[i][j][k] = -1;
		}
	}
	
	return ureturn;
}

double updateFormula(int i, int j, int k, const int* N, double L, double*** unext, 
						double*** v, double*** p, int*** sc, double numInfty, int dubin) {
	/*
	Semi-Lagrangian update scheme:
		Going straight: trajectory goes until it hits an edge of a grid box; 
						value is linearly interpolated from the two nodes connecting the edge
						(2 known nodes needed for updating 1 node)

		Turning:		trajectory goes until it hits a theta ceiling or floor;
						value is bilinearly interpolated from 4 nodes of the ceiling or floor
						(4 known nodes needed for updating 1 node)
	*/

	/* 
	Limitations:	v = 1
					dx = dy = h
	*/
	int Ni,Nj,Nk;
	double theta, vijk, pijk;
	double dx, dy, dth;

	double ureturn;

	double deltax, deltay, deltath;
	double x, y, th;
	double unew;
	double dt;
	double xi, yj;
	int n, m, ii, jj, kk;
	double A, B;
	double ux, uy, uth;
	double t;
	
	
	ureturn = unext[i][j][k];
	

	Ni        = N[0];    Nj        = N[1];    Nk        = N[2];
	
	// grid spacing
	dx  = 2.0*L/(double)(Ni-1);	dy  = 2.0*L/(double)(Nj-1);	dth = 2.0*PI/(double)(Nk);
	
	theta = k*dth;	vijk = v[i][j][k];	pijk = p[i][j][k];
	
	if (vijk == 0) { return numInfty; }
	// grid location
	xi = -L + dx*i;
	yj = -L + dy*j;	
	
    /*
     * n = +1 .... forward
     * n = -1 .... backward
     *
     * m = +1 .... right turn (theta decreases)
     * m =  0 .... straight (theta no change)
     * m = -1 .... left turn (theta increases)
     *
     */
	
	for(n=dubin;n<=1;n=n+2) {
		for(m=-1;m<=1;m++) {
			if (fabs((double)m) > 0.5) { // right (m=1) or left (m=-1) turn
				// Mo changed to abs(m) > 0.5
				// case m != 0 formulas verified by Mo
				
				if (n == 1 && m == -1) {
					deltax = pijk*(sin(dth)*cos(theta) - sin(theta)*(1-cos(dth)));
					deltay = pijk*(sin(dth)*sin(theta) + cos(theta)*(1-cos(dth)));
				} 
				
				if (n == 1 && m == 1) {
					deltax = pijk*(sin(dth)*cos(theta) + sin(theta)*(1-cos(dth)));
					deltay = pijk*(sin(dth)*sin(theta) - cos(theta)*(1-cos(dth)));				
				}
				
				if (n == -1 && m == 1) {
					deltax = pijk*(-sin(dth)*cos(theta) + sin(theta)*(1-cos(dth)));
					deltay = pijk*(-sin(dth)*sin(theta) - cos(theta)*(1-cos(dth)));					
				}	
				
				if (n == -1 && m == -1) {
					deltax = pijk*(-sin(dth)*cos(theta) - sin(theta)*(1-cos(dth)));
					deltay = pijk*(-sin(dth)*sin(theta) + cos(theta)*(1-cos(dth)));					
				}				

				// absolute update location (on the surface of accepted region)
				x = xi + deltax;
				y = yj + deltay;

				// compute extra time
				dt = pijk/vijk*dth;
								
				// compute potential value
                if  ( (n == 1 && m == -1) || (n==-1 && m == 1) ) {// if n*m == -1
                    if (k < Nk-1) 	{ unew = BilinearInterp(x, y, k+1, N, L, unext, numInfty) + dt; } 
					else 			{ unew = BilinearInterp(x, y,   0, N, L, unext, numInfty) + dt; }
                } else { // if n*m == 1
					if ( (n == 1 && m == 1) || (n == -1 && m == -1) ) {
						if (k > 0) 	{ unew = BilinearInterp(x, y,  k-1, N, L, unext, numInfty) + dt; }
						else 		{ unew = BilinearInterp(x, y, Nk-1, N, L, unext, numInfty) + dt; }
					} else { printf("Something's wrong! (m n) = (%f %f) \n", m, n); }
                }
							
			} else { // goin' straight
				
				// Case m == 0 formulas also confirmed by Mo
				if (cos(theta) == 0) { // up or down
					//dt = fabs(dy/vijk/sin(theta));
                    dt = dy/vijk;
					if (sin(theta) > 0) { jj = j + n; }
					
					if (sin(theta) < 0) { jj = j - n; }
					
					// jj = j+n*sgn(sin(theta));
					if (jj < 0 || jj > Nj-1) { unew = numInfty; } 
					else { unew = unext[i][jj][k] + dt; }
					
				} else if (sin(theta) == 0) {	// right or left			
					//dt = fabs(dx/vijk/cos(theta));
                    dt = dx/vijk;
					
					if (cos(theta) > 0) { ii = i + n; }
					
					if (cos(theta) < 0) { ii = i - n; }
					
					//ii = i+n*sgn(cos(theta));
					if (ii < 0 || ii > Ni-1) { unew = numInfty; } 
					else { unew = unext[ii][j][k] + dt; }

				} else { // some angle...
                    A = fabs(dx/vijk/cos(theta)); // time to hit left or right edge
                    B = fabs(dy/vijk/sin(theta)); // time to hit top or bottom edge

					if(A <= B) { // if left edge takes shorter time to hit
                        dt     = A;
						
						if ( sin(theta) > 0 ) { deltay =   n*fabs(tan(theta))*dx; }
						
						if ( sin(theta) < 0 ) { deltay =   -n*fabs(tan(theta))*dx; }
						
                        y = yj + deltay;
                        
						if (cos(theta) > 0) { ii = i + n; }
						
						if (cos(theta) < 0) { ii = i - n; }						
						
						if (ii < 0 || ii > Ni-1) { unew = numInfty; } 
						else { unew = LinearInterpY(ii,y,k,N,L,unext,numInfty) + dt; }
                        
					} else {
                        dt     = B;
						//printf("Not turning: dt = %f\n", dt);
						
						if (cos(theta) > 0) { deltax = n*1/fabs(tan(theta))*dy; }
						
						if (cos(theta) < 0) { deltax = -n*1/fabs(tan(theta))*dy; }					
						
                        x = xi + deltax;
						
						if (sin(theta) > 0) { jj = j + n; }
						
						if (sin(theta) < 0) { jj = j - n; }
						
                        //jj = j+n*sgn(sin(theta));
						if (jj < 0 || jj > Nj-1) { unew = numInfty; } 
						else { unew = LinearInterpX(x,jj,k,N,L,unext,numInfty) + dt; }
                        
					}
				}
			}
			
			// update value if it's smaller than before
			if (unew < ureturn) {
				ureturn = unew;
				sc[i][j][k] = n;	
			} 
			
		}
	}
	
	return ureturn;

	
}


double updateFormulaAll(int i, int j, int k, const int* N, double L, double*** unext, 
						double*** v, double*** p, int*** sc, double numInfty, int dubin) {
	/*
	Uses both semi-Lagrangian and finite difference update schemes
	
	Semi-Lagrangian
		going straight: uses horizontal, vertical, and diagonal lines for interpolation (2 known nodes needed for updating 1 node)
		turning:		uses ceiling or floor for interpolation (4 known nodes needed for updating 1 node)

	Finite difference
		uses all 4 grid orientations for updating
			1. [1 0], [0 1]
			2. [1 1], [1 -1]
			3. [2 1], [-1 2]
			4. [1 2], [-2 1]
	*/

	/* 
	Limitations:	v = 1
					dx = dy = h
	*/
	int Ni,Nj,Nk;
	double theta, vijk, pijk;
	double dx, dy, dth;

	double deltax, deltay, deltath;
	double x, y, th;
	double unew, unew2;
	double dt;
	double xi, yj;
	int n, m, ii, jj, kk;
	double A, B;
	double ux, uy, uth;
	double t;
	
	double u_vert, u_hori;
	
	double u1a, u2a, u3a, u4a, u1b, u2b, u3b, u4b;
	double uxp,uyp,utp,utm,uxm,uym,uNW,uNE,uSW,uSE,
			uNEE,uNNW,uSWW,uSSE,uNNE,uNWW,uSSW,uSEE;
	double ureturna, ureturnb, ureturn;

	Ni = N[0]; Nj = N[1]; Nk = N[2];
	
	// grid spacing
	dx  = 2.0*L/(double)(Ni-1);	dy  = 2.0*L/(double)(Nj-1);	dth = 2.0*PI/(double)(Nk);
	
	theta = k*dth;	vijk = v[i][j][k];	pijk = p[i][j][k];

	if (vijk == 0) { return numInfty; }	
	
	ureturn = unext[i][j][k];

    // ---- PDE -----
	/* 
	Limitations:	v_forward = v_backward = v
					dx = dy = h
	*/

	
	u1a = numInfty;	u2a = numInfty;	u3a = numInfty;	u4a = numInfty;
	u1b = numInfty;	u2b = numInfty;	u3b = numInfty;	u4b = numInfty;
	
	if (i < Ni-1) 	{ uxp = unext[i+1][j][k]; } else { uxp = numInfty; }
	if (i > 0) 		{ uxm = unext[i-1][j][k]; } else { uxm = numInfty; }
	if (j < Nj-1) 	{ uyp = unext[i][j+1][k]; } else { uyp = numInfty; }
	if (j > 0) 		{ uym = unext[i][j-1][k]; } else { uym = numInfty; }
	
	if (i<Ni-1&&j<Nj-1)	{ uNE = unext[i+1][j+1][k]; } else { uNE = numInfty; }
	if (i>0 && j<Nj-1)	{ uNW = unext[i-1][j+1][k]; } else { uNW = numInfty; }
	if (i>0 && j>0)		{ uSW = unext[i-1][j-1][k]; } else { uSW = numInfty; }
	if (i<Ni-1&&j>0)	{ uSE = unext[i+1][j-1][k]; } else { uSE = numInfty; }
	
	if (i<Ni-1&&j<Nj-2) { uNNE = unext[i+1][j+2][k]; } else { uNNE = numInfty; }
	if (i<Ni-2&&j>0)	{ uSEE = unext[i+2][j-1][k]; } else { uSEE = numInfty; }
	if (i>0 && j>1) 	{ uSSW = unext[i-1][j-2][k]; } else { uSSW = numInfty; }
	if (i>1 && j<Nj-1) 	{ uNWW = unext[i-2][j+1][k]; } else { uNWW = numInfty; }

	if (i<Ni-2&&j<Nj-1) { uNEE = unext[i+2][j+1][k]; } else { uNEE = numInfty; }
	if (i<Ni-1&& j>1 )	{ uSSE = unext[i+1][j-2][k]; } else { uSSE = numInfty; }
	if (i>1 && j>0 )	{ uSWW = unext[i-2][j-1][k]; } else { uSWW = numInfty; }
	if (i>0 && j<Nj-2) 	{ uNNW = unext[i-1][j+2][k]; } else { uNNW = numInfty; }
	
	// periodic boundary condition for theta
	if     (k==Nk-1) {
		utp = unext[i][j][0]; 
		utm = unext[i][j][k-1];
	} else if (k==0) {	
		utp = unext[i][j][k+1]; 
		utm = unext[i][j][Nk-1];
	} else {
		utp = unext[i][j][k+1]; 
		utm = unext[i][j][k-1];
	}
	
	ureturn = unext[i][j][k];
	
	u1a = updateU(uxp, uxm, uyp, uym, utp, utm, 1, theta, dx, dth, pijk, vijk, numInfty);
	u2a = updateUdiag(uNE, uNW, uSW, uSE, utp, utm, 1, theta, dx, dth, pijk, vijk, numInfty);	
	u3a = updateU16a(uNEE, uNNW, uSSE, uSWW,  utp,  utm, 1, theta, dx, dth, pijk, vijk, numInfty);
	u4a = updateU16b(uNNE, uNWW, uSEE, uSSW,  utp,  utm, 1, theta, dx, dth, pijk, vijk, numInfty);
	
	if (dubin == -1) {
	u1b = updateU(uxp, uxm, uyp, uym, utp, utm, -1, theta, dx, dth, pijk, vijk, numInfty);
	u2b = updateUdiag(uNE, uNW, uSW, uSE, utp, utm, -1, theta, dx, dth, pijk, vijk, numInfty);
	u3b = updateU16a(uNEE, uNNW, uSSE, uSWW,  utp,  utm, -1, theta, dx, dth, pijk, vijk, numInfty);
	u4b = updateU16b(uNNE, uNWW, uSEE, uSSW,  utp,  utm, -1, theta, dx, dth, pijk, vijk, numInfty);
	}
	
	ureturna = min(u1a,u2a);
	ureturna = min(u3a,ureturna);
	ureturna = min(u4a,ureturna);
	
	ureturnb = min(u1b,u2b);
	ureturnb = min(u3b,ureturnb);
	ureturnb = min(u4b,ureturnb);
	
	if (ureturna < ureturn || ureturnb < ureturn) {
		if (ureturna < ureturnb) {
			ureturn = ureturna;
			sc[i][j][k] = 1;
		} else {
			ureturn = ureturnb;
			sc[i][j][k] = -1;
		}
	}

	// ----- Semi-Lagrangian -----
	// grid location
	xi = -L + dx*i;
	yj = -L + dy*j;	
	
    /*
     * n = +1 .... forward
     * n = -1 .... backward
     *
     * m = +1 .... right turn (theta decreases)
     * m =  0 .... straight (theta no change)
     * m = -1 .... left turn (theta increases)
     *
     */

	for(n=dubin;n<=1;n=n+2) {
		for(m=-1;m<=1;m++) {
			if (fabs((double)m) > 0.5) { // right (m=1) or left (m=-1) turn
				// Mo changed to abs(m) > 0.5
				// case m != 0 formulas verified by Mo
				
				if (n == 1 && m == -1) {
					deltax = pijk*(sin(dth)*cos(theta) - sin(theta)*(1-cos(dth)));
					deltay = pijk*(sin(dth)*sin(theta) + cos(theta)*(1-cos(dth)));
				} 
				
				if (n == 1 && m == 1) {
					deltax = pijk*(sin(dth)*cos(theta) + sin(theta)*(1-cos(dth)));
					deltay = pijk*(sin(dth)*sin(theta) - cos(theta)*(1-cos(dth)));				
				}
				
				if (n == -1 && m == 1) {
					deltax = pijk*(-sin(dth)*cos(theta) + sin(theta)*(1-cos(dth)));
					deltay = pijk*(-sin(dth)*sin(theta) - cos(theta)*(1-cos(dth)));					
				}	
				
				if (n == -1 && m == -1) {
					deltax = pijk*(-sin(dth)*cos(theta) - sin(theta)*(1-cos(dth)));
					deltay = pijk*(-sin(dth)*sin(theta) + cos(theta)*(1-cos(dth)));					
				}				

				// absolute update location (on the surface of accepted region)
				x = xi + deltax;
				y = yj + deltay;

				// compute extra time
				dt = pijk/vijk*dth;
								
				// compute potential value
                if  ( (n == 1 && m == -1) || (n==-1 && m == 1) ) {// if n*m == -1
                    if (k < Nk-1) 	{ unew = BilinearInterp(x, y, k+1, N, L, unext, numInfty) + dt; } 
					else 			{ unew = BilinearInterp(x, y,   0, N, L, unext, numInfty) + dt; }
                } else { // if n*m == 1
					if ( (n == 1 && m == 1) || (n == -1 && m == -1) ) {
						if (k > 0) 	{ unew = BilinearInterp(x, y,  k-1, N, L, unext, numInfty) + dt; }
						else 		{ unew = BilinearInterp(x, y, Nk-1, N, L, unext, numInfty) + dt; }
					} else { printf("Something's wrong! (m n) = (%f %f) \n", m, n); }
                }
							
			} else { // goin' straight
				
				// Case m == 0 formulas also confirmed by Mo
				if (cos(theta) == 0) { // up or down
					//dt = fabs(dy/vijk/sin(theta));
                    dt = dy/vijk;
					if (sin(theta) > 0) { jj = j + n; }
					
					if (sin(theta) < 0) { jj = j - n; }
					
					// jj = j+n*sgn(sin(theta));
					if (jj < 0 || jj > Nj-1) { unew = numInfty; } 
					else { unew = unext[i][jj][k] + dt; }
					
				} else if (sin(theta) == 0) {	// right or left			
					//dt = fabs(dx/vijk/cos(theta));
                    dt = dx/vijk;
					
					if (cos(theta) > 0) { ii = i + n; }
					
					if (cos(theta) < 0) { ii = i - n; }
					
					//ii = i+n*sgn(cos(theta));
					if (ii < 0 || ii > Ni-1) { unew = numInfty; } 
					else { unew = unext[ii][j][k] + dt; }

				} else { // some angle...
                    A = fabs(dx/vijk/cos(theta)); // time to hit left or right edge
                    B = fabs(dy/vijk/sin(theta)); // time to hit top or bottom edge

					if(A <= B) { // if left edge takes shorter time to hit
                        dt     = A;
						
						if ( sin(theta) > 0 ) { deltay =   n*fabs(tan(theta))*dx; }
						
						if ( sin(theta) < 0 ) { deltay =   -n*fabs(tan(theta))*dx; }
						
                        y = yj + deltay;
                        
						if (cos(theta) > 0) { ii = i + n; }
						
						if (cos(theta) < 0) { ii = i - n; }						
						
						if (ii < 0 || ii > Ni-1) { unew = numInfty; } 
						else { unew = LinearInterpY(ii,y,k,N,L,unext,numInfty) + dt; }
                        
					} else {
                        dt     = B;
						//printf("Not turning: dt = %f\n", dt);
						
						if (cos(theta) > 0) { deltax = n*1/fabs(tan(theta))*dy; }
						
						if (cos(theta) < 0) { deltax = -n*1/fabs(tan(theta))*dy; }					
						
                        x = xi + deltax;
						
						if (sin(theta) > 0) { jj = j + n; }
						
						if (sin(theta) < 0) { jj = j - n; }
						
                        //jj = j+n*sgn(sin(theta));
						if (jj < 0 || jj > Nj-1) { unew = numInfty; } 
						else { unew = LinearInterpX(x,jj,k,N,L,unext,numInfty) + dt; }
                        
					}
					
					if (cos(theta) > 0) { ii = i + n; }
					
					if (cos(theta) < 0) { ii = i - n; }	
						
					if (sin(theta) > 0) { jj = j + n; }
					
					if (sin(theta) < 0) { jj = j - n; }
					
					if (ii < 0 || ii > Ni-1) {
						unew2 = numInfty;
					} else if (jj < 0 || jj > Nj-1){
						unew2 = numInfty;
					} else {
						u_vert = unext[i ][jj][k];
						u_hori = unext[ii][j ][k];
						
						// no need to add dt! its implemented inside procedure
						unew2 = LinearInterpDiag(u_hori, u_vert, i,j,k, N, L, unext, v, numInfty);
					}

                    
                    if (unew2 < unew){
                        unew = unew2;
                        //printf("msg 001\n");
                    }
				}
			}
			
			// update value if it's smaller than before
			if (unew < ureturn) {
				ureturn = unew;
				sc[i][j][k] = n;	
			} 
			
		}
	}
	
	return ureturn;

	
}

void updateNeighborPDE(int i, int j,int k, const int* N, double L, double*** unext,double*** v,double*** p,
						int*** sc,struct heapAugmented *h,int***  Nodes, double numInfty, int dubin) {
	// finite difference update scheme for fast marching
    double newValue;

	if(Nodes[i][j][k] < 1) {
		// compute candidate value at node
		newValue = updateFormulaPDE(i,j,k,N,L,unext,v,p,sc,numInfty, dubin);
		
		// newValue = updateFormulaPDE(i,j,k,N,L,u,v,p,sc,numInfty);
		//printf("newValue = %f\n", newValue);
	
        //if (newValue < 0.8*numInfty) { // add to heap only if its less than numInfty
            if (Nodes[i][j][k] == 0) {
                // overwrite node already in the heap
                update(i,j,k,newValue,h,Nodes);
				
            } else {
                // add new node to narrow band heap
                add(i,j,k,newValue,h);
                //update to narrow band
                Nodes[i][j][k] = 0;
				
            }
        //}
	}
}

void updateNeighbor(int i, int j,int k, const int* N, double L, double*** unext,double*** v,double*** p,
					int*** sc,struct heapAugmented *h,int***  Nodes, double numInfty, int dubin) {
	// semi-lagrangian update scheme for fast marching

	double newValue;
	if(Nodes[i][j][k] < 1) {
		// compute candidate value at node
		newValue = updateFormula(i,j,k,N,L,unext,v,p,sc,numInfty, dubin);
		
		// newValue = updateFormulaPDE(i,j,k,N,L,u,v,p,sc,numInfty);
		//printf("newValue = %f\n", newValue);
	
		if (Nodes[i][j][k] == 0) {
			// overwrite node already in the heap
			update(i,j,k,newValue,h,Nodes);
			
		} else {
			// add new node to narrow band heap
			add(i,j,k,newValue,h);
			//update to narrow band
			Nodes[i][j][k] = 0;
			
		}
	}
}


void updateNeighborAll(int i, int j,int k, const int* N, double L, double*** unext,double*** v,double*** p,
					int*** sc,struct heapAugmented *h,int***  Nodes, double numInfty, int dubin) {
    // finite difference and semi-Lagrangian update scheme
	double newValue;

	if(Nodes[i][j][k] < 1) {
		// compute candidate value at node
		newValue = updateFormulaAll(i,j,k,N,L,unext,v,p,sc,numInfty, dubin);
		
		// newValue = updateFormulaPDE(i,j,k,N,L,u,v,p,sc,numInfty);
		//printf("newValue = %f\n", newValue);
	
		if (Nodes[i][j][k] == 0) {
			// overwrite node already in the heap
			update(i,j,k,newValue,h,Nodes);
			
		} else {
			// add new node to narrow band heap
			add(i,j,k,newValue,h);
			//update to narrow band
			Nodes[i][j][k] = 0;
			
		}
	}
}


void CCM_sweepPDE(double*** unext, double*** v, double*** p, int*** sc, const int* N, double L, double numInfty, int dubin) {
	printf("Fast sweeping with finite difference update \n");
	
	// Fast sweeping, using finite difference update
	int Ni,Nj,Nk,n;
    int s1,s2,s3,i,j,k;
		
    Ni        = N[0];
    Nj        = N[1];
    Nk        = N[2];

	for(s1=-1;s1<=1;s1+=2) {
	for(s2=-1;s2<=1;s2+=2) {
	for(s3=-1;s3<=1;s3+=2) {
	for(i=(s1<0?Ni-2:1);(s1<0?i>=1:i<Ni-1);i+=s1) {
	for(j=(s2<0?Nj-2:1);(s2<0?j>=1:j<Nj-1);j+=s2) {
	for(k=(s3<0?Nk-1:0);(s3<0?k>=0:k<Nk-1);k+=s3) {
			unext[i][j][k] = updateFormulaPDE(i, j, k, N, L, unext, v, p, sc,numInfty,dubin);
			//u[i][j][k] = updateFormulaPDE(i, j, k, N, L, u, v, p, sc,numInfty);
	}
	}}}}}
}

void CCM_sweep(double*** unext, double*** v, double*** p, int*** sc, const int* N, double L, double numInfty, int dubin) {
	printf("Fast sweeping with semi-Lagrangian update \n");

	// fast sweeping, using semi-Lagrangian update
	int Ni,Nj,Nk,n;
    int s1,s2,s3,i,j,k;
		
    Ni        = N[0];
    Nj        = N[1];
    Nk        = N[2];

	for(s1=-1;s1<=1;s1+=2) {
	for(s2=-1;s2<=1;s2+=2) {
	for(s3=-1;s3<=1;s3+=2) {
	for(i=(s1<0?Ni-2:1);(s1<0?i>=1:i<Ni-1);i+=s1) {
	for(j=(s2<0?Nj-2:1);(s2<0?j>=1:j<Nj-1);j+=s2) {
	for(k=(s3<0?Nk-1:0);(s3<0?k>=0:k<Nk-1);k+=s3) {
			unext[i][j][k] = updateFormula(i, j, k, N, L, unext, v, p, sc,numInfty,dubin);
	}
	}}}}}
}

void CCM_sweepAll(double*** unext, double*** v, double*** p, int*** sc, const int* N, double L, double numInfty, int dubin) {
	printf("Fast sweeping with semi-Lagrangian and finite difference update \n");
	
	// fast sweeping, using both semi-Lagrangian and finite difference updating schemes
	int Ni,Nj,Nk,n;
    int s1,s2,s3,i,j,k;
		
    Ni        = N[0];
    Nj        = N[1];
    Nk        = N[2];

	for(s1=-1;s1<=1;s1+=2) {
	for(s2=-1;s2<=1;s2+=2) {
	for(s3=-1;s3<=1;s3+=2) {
	for(i=(s1<0?Ni-2:1);(s1<0?i>=1:i<Ni-1);i+=s1) {
	for(j=(s2<0?Nj-2:1);(s2<0?j>=1:j<Nj-1);j+=s2) {
	for(k=(s3<0?Nk-1:0);(s3<0?k>=0:k<Nk-1);k+=s3) {
			unext[i][j][k] = updateFormulaAll(i, j, k, N, L, unext, v, p, sc,numInfty,dubin);
	}
	}}}}}
}

void CCM(double*** unext, double*** v, double*** p, int***  Nodes, int*** sc, const int* N, double L, double numInfty, int dubin) { 
    // fast marching, using semi-Lagrangian update
	printf("Fast marching with semi-Lagrangian update \n");

	int Ni,Nj,Nk;
    int i,j,k;
    int numAlive;
	double dx,dy,dth;
	
	double value;
		
	struct heapAugmented* h;
	
    Ni        = N[0];
    Nj        = N[1];
    Nk        = N[2];
	
	// build heap 
	h   = build(Ni,Nj,Nk); // heap value should be all inf at this time
	
	dx  = (double) 2*L/(Ni-1); 
	dy  = (double) 2*L/(Nj-1); 
	dth = (double) 2*PI/Nk;
	
	//value = numInfty;//////////////////////////////////
	numAlive = 0;
	
	// initialize set of narrow band around b.c.
	for ( i=1; i<Ni-1; i++ ) {
        for ( j=1; j<Nj-1; j++ ) {
			for (k=0; k<Nk; k++) {
				if(Nodes[i][j][k] == 1) { // if (i,j,k) is a accepted node...
					numAlive++;
					
					// update all neighbors of (i,j,k)...
					//... unless its on the boundary
					//if(i < Ni-2)
					if(i<Ni-1) { updateNeighbor(i+1,j,k, N, L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					
					//if(i > 1)
					if(i>0) { updateNeighbor(i-1,j,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					
					//if(j < Nj-2)
					if (j<Nj-1) { updateNeighbor(i,j+1,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					
					//if(j > 1)
					if (j>0) { updateNeighbor(i,j-1,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					
					// be careful of how to treat periodic b.c. in theta!! 
					if(k < Nk-1) { updateNeighbor(i,j,k+1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
                    else { updateNeighbor(i,j,0,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
							
					if(k > 0) { updateNeighbor(i,j,k-1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
                    else { updateNeighbor(i,j,Nk-1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
				}
			}
		}
	}
    
	// stopping condition check 
	if (numAlive == (Ni-2)*(Nj-2)*(Nk)){ //then all points have been computed!
		// changed from numAlive == (Ni-2)*(Nj-2)*(Nk-2)
		//free memory for heap
		freeHeap(h);
		return;
	} 
	
	// do while loop
	while (numAlive  < (Ni-2)*(Nj-2)*(Nk)) {
		// changed from numAlive < (Ni-2)*(Nj-2)*(Nk-2)
		
		//printf("numAlive = %d\n",numAlive);
		
		if (h->numOfElements == 0) {
			printf("empty heap! numAlive = %d\n",numAlive);
			return;
		}
		
		// extract min value in NB
		i = minIndexI(h);
        j = minIndexJ(h);
		k = minIndexTh(h);
        value = minValue(h);
		
		// set value function to correct value
        unext[i][j][k] = value;
		
		// update narrow band info
		Nodes[i][j][k] = 1;
		
		numAlive++; 
		
        // remove the minimum value from heap
        removeMin(h);
        //printf("Code runs fine until here. \n");	
		
		// stopping condition check (1. all nodes done, OR 2. smallest NB element is infinity)
        //if ( numAlive == (Ni-2)*(Nj-2)*(Nk) || (value > 0.8*numInfty) ){ //then all points have been computed!
		if ( numAlive == (Ni-2)*(Nj-2)*(Nk) ){
			// changed from numAlive == (Ni-2)*(Nj-2)*(Nk-2)
			//free memory for heap
			freeHeap(h);
			//printf("Finished. numAlive = %d\n", numAlive);
            return;
        } 

		if(i<Ni-1) { updateNeighbor(i+1,j,k, N, L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
		
		if(i>0) { updateNeighbor(i-1,j,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
		
		if (j<Nj-1) { updateNeighbor(i,j+1,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
		
		//if(j > 1)
		if (j>0) { updateNeighbor(i,j-1,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }		
		
		// be careful of how to treat periodic b.c. in theta!! 
		if(k < Nk-1) { updateNeighbor(i,j,k+1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
        else { updateNeighbor(i,j,0,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
				
		if(k > 0) { updateNeighbor(i,j,k-1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
        else { updateNeighbor(i,j,Nk-1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
        
        //if(numAlive < 5)
         //   printHeapAugmented(h);
        
	}
    freeHeap(h);//dependability via redundancy, it should never get outside of the while loop without returning 
	
}

void CCM_PDE(double*** unext, double*** v, double*** p, int***  Nodes, int*** sc, const int* N, double L, double numInfty, int dubin) { 
	printf("Fast marching with finite difference update \n");
	// fast marching, using finite difference update
    int Ni,Nj,Nk;
    int i,j,k;
    int numAlive;
	double dx,dy,dth;
	
	double value;
		
	struct heapAugmented* h;
	
    Ni        = N[0];
    Nj        = N[1];
    Nk        = N[2];
	
	// build heap 
	h   = build(Ni,Nj,Nk); // heap value should be all inf at this time
	
	dx  = (double) 2*L/(Ni-1); 
	dy  = (double) 2*L/(Nj-1); 
	dth = (double) 2*PI/Nk;
	
	//value = numInfty;//////////////////////////////////
	numAlive = 0;
	
	// initialize set of narrow band around b.c.
	for ( i=1; i<Ni-1; i++ ) {
        for ( j=1; j<Nj-1; j++ ) {
			for (k=0; k<Nk; k++) {
				if(Nodes[i][j][k] == 1) { // if (i,j,k) is a accepted node...
					numAlive++;
					
					// update all neighbors of (i,j,k)...
					//... unless its on the boundary
					//if(i < Ni-2)
					if(i<Ni-1) { updateNeighborPDE(i+1,j,k, N, L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					
					//if(i > 1)
					if(i>0) { updateNeighborPDE(i-1,j,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					
					//if(j < Nj-2)
					if (j<Nj-1) { updateNeighborPDE(i,j+1,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					
					//if(j > 1)
					if (j>0) { updateNeighborPDE(i,j-1,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					
					// be careful of how to treat periodic b.c. in theta!! 
					if(k < Nk-1) { updateNeighborPDE(i,j,k+1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					// this is the case k == Nk-1
                    else { updateNeighborPDE(i,j,0,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
                        
							
					if(k > 0) { updateNeighborPDE(i,j,k-1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
                    // This is the case k == 0
					else { updateNeighborPDE(i,j,Nk-1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
				}
			}
		}
	}
	
    //printHeapAugmented(h);
    
	// stopping condition check 
	if (numAlive == (Ni-2)*(Nj-2)*(Nk)){ //then all points have been computed!
		// changed from numAlive == (Ni-2)*(Nj-2)*(Nk-2)
		//free memory for heap
		freeHeap(h);
		return;
	} 
	
	// do while loop
	while (numAlive  < (Ni-2)*(Nj-2)*(Nk)) {
		// changed from numAlive < (Ni-2)*(Nj-2)*(Nk-2)
		
		//printf("numAlive = %d\n",numAlive);
		
		if (h->numOfElements == 0) {
			printf("empty heap! numAlive = %d\n",numAlive);
			return;
		}
		
		// extract min value in NB
		i = minIndexI(h);
        j = minIndexJ(h);
		k = minIndexTh(h);
        value = minValue(h);
		//printf("value = %f \n",value);
		//printf("i,j,k = %d,%d,%d\n",i,j,k);
		
		// set value function to correct value
        unext[i][j][k] = value;
		
		// update narrow band info
		Nodes[i][j][k] = 1;
		
		numAlive++; 
		
        // remove the minimum value from heap
        removeMin(h);
        //printf("Code runs fine until here. \n");	
		
		// stopping condition check (1. all nodes done, OR 2. smallest NB element is infinity)
        //if ( numAlive == (Ni-2)*(Nj-2)*(Nk) || (value > 0.8*numInfty) ){ //then all points have been computed!
		if ( numAlive == (Ni-2)*(Nj-2)*(Nk) ){
			// changed from numAlive == (Ni-2)*(Nj-2)*(Nk-2)
			//free memory for heap
			freeHeap(h);
			//printf("Finished. numAlive = %d\n", numAlive);
            return;
        } 
		
		if(i<Ni-1) { updateNeighborPDE(i+1,j,k, N, L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
		
		//if(i > 1)
		if(i>0) { updateNeighborPDE(i-1,j,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
		
		//if(j < Nj-2)
		if (j<Nj-1) { updateNeighborPDE(i,j+1,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
		
		//if(j > 1)
		if (j>0) { updateNeighborPDE(i,j-1,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }		
		
		// be careful of how to treat periodic b.c. in theta!! 
		if(k < Nk-1) { updateNeighborPDE(i,j,k+1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
        // this is the case k == Nk-1
		else { updateNeighborPDE(i,j,0,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
				
		if(k > 0) { updateNeighborPDE(i,j,k-1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
        // This is the case k == 0
		else { updateNeighborPDE(i,j,Nk-1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
        
        //if(numAlive < 5)
         //   printHeapAugmented(h);
        
	}
    freeHeap(h);//dependability via redundancy, it should never get outside of the while loop without returning 
}

void CCM_all(double*** unext, double*** v, double*** p, int***  Nodes, int*** sc, const int* N, double L, double numInfty, int dubin) { 
    printf("Fast marching with semi-Lagrangian and finite difference update \n");

	// fast marching, using both finite difference and semi-Lagrangian updating schemes
	int Ni,Nj,Nk;
    int i,j,k;
    int numAlive;
	double dx,dy,dth;
	
	double value;
		
	struct heapAugmented* h;
	
    Ni        = N[0];
    Nj        = N[1];
    Nk        = N[2];
	
	// build heap 
	h   = build(Ni,Nj,Nk); // heap value should be all inf at this time
	
	dx  = (double) 2*L/(Ni-1); 
	dy  = (double) 2*L/(Nj-1); 
	dth = (double) 2*PI/Nk;
	
	//value = numInfty;//////////////////////////////////
	numAlive = 0;
	
	// initialize set of narrow band around b.c.
	for ( i=1; i<Ni-1; i++ ) {
        for ( j=1; j<Nj-1; j++ ) {
			for (k=0; k<Nk; k++) {
				if(Nodes[i][j][k] == 1) { // if (i,j,k) is a accepted node...
					numAlive++;
					
					// update all neighbors of (i,j,k)...
					//... unless its on the boundary
					//if(i < Ni-2)
					if(i<Ni-1) { updateNeighborAll(i+1,j,k, N, L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					
					//if(i > 1)
					if(i>0) { updateNeighborAll(i-1,j,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					
					//if(j < Nj-2)
					if (j<Nj-1) { updateNeighborAll(i,j+1,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					
					//if(j > 1)
					if (j>0) { updateNeighborAll(i,j-1,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
					
					// be careful of how to treat periodic b.c. in theta!! 
					if(k < Nk-1) { updateNeighborAll(i,j,k+1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
                    else { updateNeighborAll(i,j,0,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
							
					if(k > 0) { updateNeighborAll(i,j,k-1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
                    else { updateNeighborAll(i,j,Nk-1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
				}
			}
		}
	}
    
	// stopping condition check 
	if (numAlive == (Ni-2)*(Nj-2)*(Nk)){ //then all points have been computed!
		// changed from numAlive == (Ni-2)*(Nj-2)*(Nk-2)
		//free memory for heap
		freeHeap(h);
		return;
	} 
	
	// do while loop
	while (numAlive  < (Ni-2)*(Nj-2)*(Nk)) {
		// changed from numAlive < (Ni-2)*(Nj-2)*(Nk-2)
		
		//printf("numAlive = %d\n",numAlive);
		
		if (h->numOfElements == 0) {
			printf("empty heap! numAlive = %d\n",numAlive);
			return;
		}
		
		// extract min value in NB
		i = minIndexI(h);
        j = minIndexJ(h);
		k = minIndexTh(h);
        value = minValue(h);
		
		// set value function to correct value
        unext[i][j][k] = value;
		
		// update narrow band info
		Nodes[i][j][k] = 1;
		
		numAlive++; 
		
        // remove the minimum value from heap
        removeMin(h);
        //printf("Code runs fine until here. \n");	
		
		// stopping condition check (1. all nodes done, OR 2. smallest NB element is infinity)
        //if ( numAlive == (Ni-2)*(Nj-2)*(Nk) || (value > 0.8*numInfty) ){ //then all points have been computed!
		if ( numAlive == (Ni-2)*(Nj-2)*(Nk) ){
			// changed from numAlive == (Ni-2)*(Nj-2)*(Nk-2)
			//free memory for heap
			freeHeap(h);
			//printf("Finished. numAlive = %d\n", numAlive);
            return;
        } 

		if(i<Ni-1) { updateNeighborAll(i+1,j,k, N, L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
		
		if(i>0) { updateNeighborAll(i-1,j,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
		
		if (j<Nj-1) { updateNeighborAll(i,j+1,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
		
		//if(j > 1)
		if (j>0) { updateNeighborAll(i,j-1,k,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }		
		
		// be careful of how to treat periodic b.c. in theta!! 
		if(k < Nk-1) { updateNeighborAll(i,j,k+1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
        else { updateNeighborAll(i,j,0,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
				
		if(k > 0) { updateNeighborAll(i,j,k-1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
        else { updateNeighborAll(i,j,Nk-1,N,L,unext,v,p,sc,h,Nodes,numInfty, dubin); }
        
        //if(numAlive < 5)
         //   printHeapAugmented(h);
        
	}
    freeHeap(h);//dependability via redundancy, it should never get outside of the while loop without returning 
	
}

double fndiff(double*** fold, double*** fnew, const int* N, double numInfty ){
    int Ni,Nj,Nk;
	int i, j, k;
	double diff, maxdiff, avgdiff, totaldiff;
	int numBigDiff;
	
    Ni        = N[0];
    Nj        = N[1];
    Nk        = N[2];	

	diff = 0;
	maxdiff = 0;
	avgdiff = 0;
	totaldiff = 0;
	
	numBigDiff = 0;
	
	// Calculate difference between the two value functions
	for (i=0; i<Ni; i++) {
		for (j=0; j<Nj; j++) {
			for (k=0; k<Nk; k++) {
				diff = fabs(fold[i][j][k] - fnew[i][j][k]);
				
				totaldiff += diff;
				
				if (diff > maxdiff) { maxdiff = diff; }
				
				if (diff > 1.0) { numBigDiff++; }
			}
				
		}
	}
	
	avgdiff = totaldiff/Ni/Nj/Nk;
	
	//return (double) numBigDiff;
	return avgdiff;
	//return maxdiff;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	// Main function
	// MatLab function call: [uf,sc] = CCMotion(u,v,p,L,numInfty);

    // for the output matrix
    #define B_OUT	plhs[0]
	#define B_OUT2	plhs[1]
    
    //---Inside mexFunction---
    //Declarations
    double *uValues,*vValues,*pValues;
    double ***u,***v,***p,***unext;
    double L,numInfty; 
	int Ni,Nj,Nk;
    int i,j,k;
    int numIter, minIter, maxIter;
	int dubin; // 1 for Dubins car, -1 for RS car
	int march; // 1 to enable fast marching, 0 to disable
	int sweep; // 1 to enable fast sweeping, 0 to disable
	int march_method; // 0 for SL, 1 for FD, 2 for both
	int sweep_method; // 0 for SL, 1 for FD, 2 for both
    double *B,*B2;
    
	int ***Nodes, ***sc;
	
    int K = mxGetNumberOfDimensions(prhs[0]); 
    const int *N = mxGetDimensions(prhs[0]);
	
	double difference; 
	
    // ========= INPUTS ===========
    uValues			= mxGetPr(prhs[0]); // first LHS variable
    vValues			= mxGetPr(prhs[1]);
	pValues			= mxGetPr(prhs[2]);
	L				= (double)mxGetScalar(prhs[3]);
    numInfty		= (double)mxGetScalar(prhs[4]);
	dubin			= (int)mxGetScalar(prhs[5]);  
	march			= (int)mxGetScalar(prhs[6]);  
	sweep			= (int)mxGetScalar(prhs[7]);
	march_method	= (int)mxGetScalar(prhs[8]);
	sweep_method	= (int)mxGetScalar(prhs[9]);
	// ===========================
 
 
    Ni        = N[0];
    Nj        = N[1];
    Nk        = N[2];
    
    // memory allocation for u, v, p
    //u represents the time-to-reach value function, a function of space. v is the speed profile, encoding the obstacles. p is the turning radius profile.
    u    = (double ***) malloc ( Ni * sizeof(double**));
	v    = (double ***) malloc ( Ni * sizeof(double**));
	p    = (double ***) malloc ( Ni * sizeof(double**));
	unext= (double ***) malloc ( Ni * sizeof(double**));
	for (i=0;i<Ni;i++) {
		u[i]    = (double **) malloc (Nj * sizeof(double*));
		v[i]    = (double **) malloc (Nj * sizeof(double*));
		p[i]    = (double **) malloc (Nj * sizeof(double*));
		unext[i]    = (double **) malloc (Nj * sizeof(double*));
		for (j=0;j<Nj;j++) {
			u[i][j]    = (double *) malloc (Nk * sizeof(double));
			v[i][j]    = (double *) malloc (Nk * sizeof(double));
			p[i][j]    = (double *) malloc (Nk * sizeof(double));
			unext[i][j]    = (double *) malloc (Nk * sizeof(double));
        }
	}
    
    // assignment u, v, and p
    for (i=0; i < Ni; i++) {
		for (j=0; j < Nj; j++) {
			for (k=0; k < Nk; k++) {
                    u[i][j][k]    =    uValues[(k*Nj + j)*Ni + i];
					unext[i][j][k]=    u[i][j][k];
					v[i][j][k]    =    vValues[(k*Nj + j)*Ni + i];
					p[i][j][k]    =    pValues[(k*Nj + j)*Ni + i];
            }
		}
	}
           
	// pointers for accepted (1), narrow band (0), far away (-1) nodes.
	Nodes    = (int ***) malloc ( Ni * sizeof(int**));
	sc       = (int ***) malloc ( Ni * sizeof(int**));
	for (i=0;i<Ni;i++) {
		Nodes[i]    = (int **) malloc (Nj * sizeof(int*));
		sc[i]       = (int **) malloc (Nj * sizeof(int*));
		for (j=0;j<Nj;j++) {
			Nodes[i][j]    = (int *) malloc (Nk * sizeof(int));
			sc[i][j]       = (int *) malloc (Nk * sizeof(int));
        }
	}
	
	
	// initialize all nodes as far away
	for (i=0; i < Ni; i++) {
        for (j=0; j < Nj; j++) {
            for (k=0; k < Nk; k++) {
				// if value is currently 0, it is the b.c., so it is accepted
				if (u[i][j][k] <= 0) {
					Nodes[i][j][k] = 1; // accepted
				} else {
					Nodes[i][j][k] = -1; // otherwise, far away
				}
                
                //initialize switching control to zeros
                sc[i][j][k] = 0;
            }
        }
	}
    
	// ========= MARCH =========
	// CCM_all(unext, v, p, Nodes, sc, N, L, numInfty, dubin);
	if (march) {
		if (march_method == 0) { CCM(unext, v, p, Nodes, sc, N, L, numInfty, dubin); }
		else if (march_method == 1) { CCM_PDE(unext, v, p, Nodes, sc, N, L, numInfty, dubin); } 
		else if (march_method == 2) { CCM_all(unext, v, p, Nodes, sc, N, L, numInfty, dubin); }
		else { printf("Warning: unknown march_method!"); }

		// CCM(unext, v, p, Nodes, sc, N, L, numInfty, dubin);
		// Update u
		for (i=0; i < Ni; i++) {
			for (j=0; j < Nj; j++) {
				for (k=0; k < Nk; k++) {
					u[i][j][k] = unext[i][j][k];
				}
			}
		}
	}
	// ==========================

	// ========== SWEEP =========
	if (sweep) {
	difference = numInfty;
	
	numIter = 0;
	minIter = 10;
	maxIter = numInfty;
	// for (numIter = 0; numIter<10; numIter+=0) {
	//while (difference > 0.01*Ni*Nj*Nk && numIter < maxIter) {
	while (numIter < minIter || (difference > 1 && numIter < maxIter) ) {
		numIter++;
		printf("numIter = %d\n", numIter);

		if (sweep_method == 0) { CCM_sweep(unext, v, p, sc, N, L, numInfty, dubin); }
		else if (sweep_method == 1) { CCM_sweepPDE(unext, v, p, sc, N, L, numInfty, dubin); } 
		else if (sweep_method == 2) { CCM_sweepAll(unext, v, p, sc, N, L, numInfty, dubin); }
		else { printf("Warning: unknown sweep_method!"); }

		//CCM_sweepAll(unext, v, p, sc, N, L, numInfty, dubin); 
		
		difference = fndiff(u, unext, N, numInfty );	// calculate difference
		printf("difference is %f\n", difference);
		
		// Update u
		for (i=0; i < Ni; i++) {
			for (j=0; j < Nj; j++) {
				for (k=0; k < Nk; k++) {
					u[i][j][k] = unext[i][j][k];
				}
			}
		}
	}
	}
	// ===========================

	// ======= OUTPUT =======
    // create output function
    B_OUT = mxCreateNumericArray(K, N, mxDOUBLE_CLASS, mxREAL);
	B_OUT2= mxCreateNumericArray(K, N, mxDOUBLE_CLASS, mxREAL);
    B     = mxGetPr(B_OUT);
	B2    = mxGetPr(B_OUT2);
    
    for (i=0; i < Ni; i++) {
        for (j=0; j < Nj; j++) {
            for (k=0; k < Nk; k++) {
                B[(k*Nj+j)*Ni+i]  = u[i][j][k];
				B2[(k*Nj+j)*Ni+i] = (double) sc[i][j][k];
            }
        }
	}
    // =======================
    

    // free memory;
	for(i=0; i< Ni; i++) {
		for(j=0; j< Nj; j++) {
			free(u[i][j]);
			free(v[i][j]);
			free(p[i][j]);
			free(Nodes[i][j]);
			free(sc[i][j]);
        }
		free(u[i]);
		free(v[i]);
		free(p[i]);
		free(Nodes[i]);
		free(sc[i]);
	}
	free(u);
    free(v);
	free(p);
	free(Nodes);
	free(sc);
    
}
