#include <math.h>
#include "matrix.h"
#include "mex.h"   //--This one is required
#include <float.h>
#include <stdio.h>
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))


struct triple
{
	int x;
	int y;
	double value;
};

struct heapAugmented
{
	int LIMIT;
	int numOfElements;
	struct triple *heap;
	int **hashMap;
};

struct heapAugmented* build(int N){
	struct heapAugmented *hh;
	int i,j;
	hh = (struct heapAugmented *)malloc(sizeof(struct heapAugmented));
	hh->LIMIT=N;
	hh->numOfElements = 0;
	hh->heap = (struct triple *)malloc((N*N+1)*sizeof(struct triple));
	struct triple firstTriple;
	firstTriple.x = 0;
	firstTriple.y = 0;
	firstTriple.value = -DBL_MAX;
	(hh->heap)[0] = firstTriple;
	hh->hashMap = (int **)malloc(N*sizeof(int *));
	for(i=0; i< N; i++){
		*(hh->hashMap + i) = (int *)malloc(N*sizeof(int));
	}
    //initialize hash table! -rrtakei
    for(i=0; i< N; i++)
        for(j=0; j< N; j++)
            (hh->hashMap)[i][j] = 0;
    
	return hh;
}

void printHeapAugmented(struct heapAugmented *h){
	if(h->numOfElements == 0){
		printf("This is an empty heap");
	}
	else{
		printf("The heap now has %d elements. The heapArray looks like the following:  \n",h->numOfElements);
		int i;
		for(i=1;i <= h->numOfElements;i++){
			printf("(%d,%d,%f) at position %d;",(h->heap[i]).x,(h->heap[i]).y,(h->heap[i]).value,i);
			printf("The hashMap at (%d,%d) points to the %d th element in the hashArray.\n",(h->heap)[i].x,((h->heap)[i]).y,(h->hashMap)[((h->heap[i])).x][((h->heap[i])).y]);
		}
	}
}

void printHashMap(struct heapAugmented *h){
	int i,j;
	for(i=0;i<h->LIMIT;i++){
		for(j=0;j<h->LIMIT;j++){
			printf("The element at (%d,%d) is %d",i,j,(h->hashMap)[i][j]);
		}
	}
}

// name changed by rrtakei
double minValue(struct heapAugmented *h)
{
	return ((h->heap)[1]).value;
}

// added by rrtakei
int minIndexI(struct heapAugmented *h)
{
	return ((h->heap)[1]).x;
}
// added by rrtakei
int minIndexJ(struct heapAugmented *h)
{
	return ((h->heap)[1]).y;
}



void add(int i,int j,double value,struct heapAugmented *h){
    /*Asumption: When you call add,the (i,j) must be distinct from each of the pairs already in the heap structure,
      otherwise, you should call update.
    */
    if((h->hashMap)[i][j] == 0){
        /*
         * It appears to me that C initializes everything in the hashMap to be 0.
         */
        struct triple newTriple;
        newTriple.x = i;
        newTriple.y = j;
        newTriple.value = value;
        h->numOfElements = h->numOfElements + 1;
        int currentPosition=h->numOfElements;

        while(((h->heap)[currentPosition/2]).value >= value){
            (h->heap)[currentPosition] = (h->heap)[currentPosition/2];
            (h->hashMap)[((h->heap)[currentPosition]).x][((h->heap)[currentPosition]).y] = currentPosition;
            currentPosition = currentPosition/2;
        }
        (h->heap)[currentPosition] = newTriple;
        (h->hashMap)[i][j] = currentPosition;
    }
    else{
        printf("The element in (%d,%d) is already in the list, you cannot add them!\n",i,j);
    }
}

double removeMin(struct heapAugmented *h){
	if(h->numOfElements == 1){
	     double minValue = (h->heap)[1].value;
         h->numOfElements = 0;
         (h->hashMap)[((h->heap)[1]).x][((h->heap)[1]).y] = 0;
         return minValue;
	}

	else{
		double minValue = (h->heap)[1].value;
		(h->hashMap)[((h->heap)[1]).x][((h->heap)[1]).y] = 0;
		struct triple newTriple = (h->heap)[h->numOfElements];
		(h->numOfElements)--;
		int currentPosition = 1;

		while(2*currentPosition <= h->numOfElements){
		    			 //so long as the element updated has at least one child,loop
		    			 if(2*currentPosition + 1 <= h->numOfElements){
		    				 //the updated element has two children
		    				 if((h->heap)[2*currentPosition].value < (h->heap)[2*currentPosition+1].value){
                                 if((h->heap)[2*currentPosition].value >= newTriple.value){
                                     break;
                                 }
                                 else{
                                 /*the left child of the updated element has smaller value than the right child
                                  * of the updated element,in which case we shall bubble up the left child so that
                                  * the left child will now have smaller or equal to value than both
                                  */
                                 (h->heap)[currentPosition] = (h->heap)[2*currentPosition];
                                 (h->hashMap)[(h->heap)[currentPosition].x][(h->heap)[currentPosition].y] = currentPosition;
                                 currentPosition = 2*currentPosition;
                                 }
                             }
                             else {
                                 if((h->heap)[2*currentPosition + 1].value >= newTriple.value){
                                     break;
                                 }
                                 else{
                                 /*the left child of the updated element has larger or equal to value than the right child
                                  * of the updated element,in which case we shall bubble up the right child so that
                                  * the right child will now have smaller or equal to value than both
                                  */
                                 (h->heap)[currentPosition] = (h->heap)[2*currentPosition + 1];
                                 (h->hashMap)[(h->heap)[currentPosition].x][(h->heap)[currentPosition].y] = currentPosition;
                                 currentPosition = 2*currentPosition +1 ;
                                 }
                            }

		    			 }
		    			 else{
                            //the updated element only has one child
                            if(newTriple.value > (h->heap)[2*currentPosition].value){
                                 (h->heap)[currentPosition] = (h->heap)[2*currentPosition];
                                 (h->hashMap)[(h->heap)[currentPosition].x][(h->heap)[currentPosition].y] = currentPosition;
                                 currentPosition = 2*currentPosition;
                            }
		    				 
                             else { // else case modified by rrtakei
                                break;
                             }

		    			 }
		    		 }
		(h->heap)[currentPosition] = newTriple;
		(h->hashMap)[newTriple.x][newTriple.y] = currentPosition; //we can make it more efficient by using pointer notation instead of array notation
		return minValue;
	}
}

void update(int i,int j,double newValue,struct heapAugmented *h){
	 int position = (h->hashMap)[i][j];
     if(position == 0){
    	 printf("The value you want to update doesn't exist\n");
     }
     else{
    	 double oldValue = (h->heap)[position].value;
    	 (h->heap)[position].value = newValue;
    	 if(newValue < oldValue){
    		 //you bubble the updated element up, if necessary
    		 struct triple newTriple = (h->heap)[position];
    		 int currentPosition = position;
    		 while(((h->heap)[currentPosition/2]).value >= newValue){
    		 		(h->heap)[currentPosition] = (h->heap)[currentPosition/2];
    		 		(h->hashMap)[((h->heap)[currentPosition]).x][((h->heap)[currentPosition]).y] = currentPosition;
    		 		currentPosition = currentPosition/2;
    		 	}
    		 	(h->heap)[currentPosition] = newTriple;
    		 	(h->hashMap)[i][j] = currentPosition;
    	 }
    	 else{
    		 //you bubble the updated element down,if necessary
    		 struct triple newTriple = (h->heap)[position];
    		 int currentPosition = position;
    		 while(2*currentPosition <= h->numOfElements){
    			 //so long as the element updated has at least one child,loop
    			 if(2*currentPosition + 1 <= h->numOfElements){
    				 //the updated element has two children
    				 if((h->heap)[2*currentPosition].value < (h->heap)[2*currentPosition+1].value){
                         if((h->heap)[2*currentPosition].value >= newTriple.value){
                             break;
                         }
                         else{
    					 /*the left child of the updated element has smaller value than the right child
    					  * of the updated element,in which case we shall bubble up the left child so that
    					  * the left child will now have smaller or equal to value than both
    					  */
    					 (h->heap)[currentPosition] = (h->heap)[2*currentPosition];
    					 (h->hashMap)[(h->heap)[currentPosition].x][(h->heap)[currentPosition].y] = currentPosition;
    					 currentPosition = 2*currentPosition;
                         }
    				 }
    				 else {
                         if((h->heap)[2*currentPosition + 1].value >= newTriple.value){
                             break;
                         }
                         else{
    					 /*the left child of the updated element has larger or equal to value than the right child
    					  * of the updated element,in which case we shall bubble up the right child so that
    					  * the right child will now have smaller or equal to value than both
    					  */
    					 (h->heap)[currentPosition] = (h->heap)[2*currentPosition + 1];
    					 (h->hashMap)[(h->heap)[currentPosition].x][(h->heap)[currentPosition].y] = currentPosition;
    					 currentPosition = 2*currentPosition +1 ;
                         }
                    }

    			 }
    			 else{
    				 //the updated element only has one child
    				 if(newTriple.value > (h->heap)[2*currentPosition].value){
    					 (h->heap)[currentPosition] = (h->heap)[2*currentPosition];
    					 (h->hashMap)[(h->heap)[currentPosition].x][(h->heap)[currentPosition].y] = currentPosition;
                         currentPosition = 2*currentPosition;
                     }
    					 
    				 
                     else { // else case modified by rrtakei
                        break;
                     }

    			 }
    		 }
    		 (h->heap)[currentPosition] = newTriple;
    		 (h->hashMap)[newTriple.x][newTriple.y] = currentPosition; //we can make it more efficient by using pointer notation instead of array notation
    	 }
     }

}

double updateFormula(int i, int j, double** u, double** speed, double dx, double numInfty, int N){
    double c, DET, ux, uy, u_new;    
    
   
    c = speed[i][j];

    if ( c <= 0) {// inside obstacle
        return numInfty;
    } else {
        c = 1/c;

        if      (j==0)      ux = u[i][j+1];
        else if (j==N-1)    ux = u[i][j-1];
        else                ux = min(u[i][j+1], u[i][j-1]);

        if      (i==0)      uy = u[i+1][j];
        else if (i==N-1)    uy = u[i-1][j];
        else                uy = min(u[i+1][j], u[i-1][j]);

        DET = 2*dx*dx*c*c - (ux-uy)*(ux-uy);

        if (DET < 0)
            u_new = dx*c + min(ux,uy);
        else  {  
            u_new = 0.5*(ux + uy + sqrt(DET));
            if (uy > u_new )        u_new = dx*c + ux; 
            else if (ux > u_new)    u_new = dx*c + uy;
        }
        
        return u_new;
    }
    
    
}
    
void runEikonal(double** u, double** bdryCond, double** speed, bool** Alive, bool** NarrowBand, int N, double L, double numInfty)
{
    double ux,uy,error,DET,u_old,u_new,c,dx,value,newValue;
    int i,j,s1,s2,m,k;
    int numAlive;
    
    struct heapAugmented* h;
	
	h = build(N);
    dx = (double) 2*L/(N-1);

    numAlive = 0;
    
    // initial set of narrow band around boundary condition
    for ( i=0; i<N; i++ )
        for ( j=0; j<N; j++ ) {
            if(Alive[i][j]) { 
                // if Alive, u has been updated already
                numAlive++;
            } else { 
                //if not in Alive, and is close to another Alive, then add to Narrow Band
                if(i>0) {
                    if(Alive[i-1][j]) {
                        newValue = updateFormula(i, j, u, speed, dx, numInfty,N); 
                        if (NarrowBand[i][j]) {
                            update(i,j,newValue,h);
                        } else {
                            add(i,j,newValue,h);
                            NarrowBand[i][j] = true;
                        }
                    }
                }
                if(i<N-1) {
                    if(Alive[i+1][j]) {
                        newValue = updateFormula(i, j, u, speed, dx, numInfty,N); 
                        if (NarrowBand[i][j]) {
                            update(i,j,newValue,h);
                        } else {
                            add(i,j,newValue,h);
                            NarrowBand[i][j] = true;
                        }
                    }
                }
                if(j<N-1) {
                    if(Alive[i][j+1]) {
                        newValue = updateFormula(i, j, u, speed, dx, numInfty,N); 
                        if (NarrowBand[i][j]) {
                            update(i,j,newValue,h);
                        } else {
                            add(i,j,newValue,h);
                            NarrowBand[i][j] = true;
                        }
                    }
                }
                if(j>0) {
                    if(Alive[i][j-1]) {
                        newValue = updateFormula(i, j, u, speed, dx, numInfty,N); 
                        if (NarrowBand[i][j]) {
                            update(i,j,newValue,h);
                        } else {
                            add(i,j,newValue,h);
                            NarrowBand[i][j] = true;
                        }
                    }
                }
            }
        }
    
    
    if (numAlive == N*N){ //then all points have been computed!
		//free memory for heap
		for(i=0;i<h->LIMIT;i++){
                free((h->hashMap)[i]);
		}
		free(h->hashMap);
		free(h->heap);
		free(h);
		
        return;
	}
    
    while (numAlive < N*N) {
        
        //extract the smallest value in the Narrow band
        i = minIndexI(h);
        j = minIndexJ(h);
        value = minValue(h);
        
        // set value function to correct value
        u[i][j] = value;
        
        Alive[i][j] = true;         // now this index is Alive...
        NarrowBand[i][j] = false;   // ...and not in the Narrow Band...
        removeMin(h);               // ...and should be removed from the heap.
        
        //add to the counter of Alive nodes
        numAlive++; 
		
        if (numAlive == N*N){ //then all points have been computed!
			//free memory for heap
			for(i=0;i<h->LIMIT;i++){
                free((h->hashMap)[i]);
			}
			free(h->hashMap);
			free(h->heap);
			free(h);
			
            return;
        }
        
        // next, update all neighbours of (i,j) to grow Narrow band if not already
        if(i>0) {
            if(Alive[i-1][j]==false){
                newValue = updateFormula(i-1, j, u, speed, dx, numInfty,N); 
                if(NarrowBand[i-1][j]) { 
                    // it it is already in the Narrow Band overwrite the new update value 
                    update(i-1,j,newValue,h);
                } else {   
                    // if not already in the Narrow Band, just add it
                    add(i-1,j,newValue,h);
                    NarrowBand[i-1][j] = true;
                }
            }
        }
        
        
        if(i<N-1) {
            if(Alive[i+1][j]==false){
                newValue = updateFormula(i+1, j, u, speed, dx, numInfty,N); 
                if(NarrowBand[i+1][j]) { 
                    // it it is already in the Narrow Band overwrite the new update value 
                    update(i+1,j,newValue,h);
                } else {  
                    // if not already in the Narrow Band, just add it:
                    add(i+1,j,newValue,h);
                    NarrowBand[i+1][j] = true;
                }
            }
        }
        
        
        
        if(j<N-1) {
            if(Alive[i][j+1]==false){
                newValue = updateFormula(i, j+1, u, speed, dx, numInfty,N); 
                if(NarrowBand[i][j+1]) { 
                    // it it is already in the Narrow Band overwrite the new update value
                    update(i,j+1,newValue,h);
                } else {   
                    // if not already in the Narrow Band, just add it:
                    add(i,j+1,newValue,h);
                    NarrowBand[i][j+1] = true;
                }
            }
        }
        
        
        if(j>0) {
            if(Alive[i][j-1]==false){
                newValue = updateFormula(i, j-1, u, speed, dx, numInfty,N); 
                if(NarrowBand[i][j-1]) { 
                    // it it is already in the Narrow Band overwrite the new update value 
                    update(i,j-1,newValue,h);
                } else {   
                    // if not already in the Narrow Band, just add it:
                    add(i,j-1,newValue,h);
                    NarrowBand[i][j-1] = true;
                }
            }
        }
        
       
    
	
	}
	
    
		//free memory for heap
		for(i=0;i<h->LIMIT;i++){
                free((h->hashMap)[i]);
		}
		free(h->hashMap);
		free(h->heap);
		free(h);
		
		return;
	
    
    
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //---Inside mexFunction---
    //Declarations
    double *speedValues, *uValues, *bdryCondValues;
    double **u, **speed, **bdryCond;
    bool **Alive, **NarrowBand;
    const mwSize *dims;
	int N, iP,jP;
    double L,TOL,error,bc;
    int i,j,k;
    
    
    //double L = 1;
    double numInfty = 1000;
    
    //Get the input
    uValues        = mxGetPr(prhs[0]);
    bdryCondValues = mxGetPr(prhs[1]);
    speedValues    = mxGetPr(prhs[2]);
    dims      = mxGetDimensions(prhs[0]);
    N         = dims[0];
    L         = (double)mxGetScalar(prhs[3]);
    
    
    
    // memory allocation for u, speed, bdryCond
	u        = (double **) malloc ( N * sizeof(double*));
    speed    = (double **) malloc ( N * sizeof(double*));
    bdryCond = (double **) malloc ( N * sizeof(double*));
    Alive    = (bool **) malloc ( N * sizeof(bool*));
    NarrowBand  = (bool **) malloc ( N * sizeof(bool*));
	for (i=0;i<N;i++){
		u[i]        = (double *) malloc (N * sizeof(double));
        speed[i]    = (double *) malloc (N * sizeof(double));
        bdryCond[i] = (double *) malloc (N * sizeof(double));
        Alive[i]    = (bool *) malloc (N * sizeof(bool));
        NarrowBand[i]  = (bool *) malloc (N * sizeof(bool));
    }
    
    
    
    
    // assignment u, speed and bdryCond values 
    for (i=0; i < N; i++) 
		for (j=0; j < N; j++) {
            speed[i][j]    = speedValues[(j*N)+i];
            bdryCond[i][j] = bdryCondValues[(j*N)+i];
        }
   
    // set u to numerical infinity, except at the boundary condition
    for (i=0; i < N; i++)
		for (j=0; j < N; j++) {
                bc = bdryCond[i][j];
                if ( bc >= 0) {
                    u[i][j] = bc;
                    Alive[i][j] = true;
                    
                } else if(speed[i][j] <= 0) {
                    u[i][j] = numInfty;
                    Alive[i][j] = true;
                } else { 
                    u[i][j] = numInfty;
                    Alive[i][j] = false;
                }
                NarrowBand[i][j] = false;
            }
   
    
    // run FMM algorithm
	runEikonal(u, bdryCond, speed, Alive, NarrowBand, N, L, numInfty);
  
    // send the processed u to the output  
    for (i=0; i < N; i++)
		for (j=0; j < N; j++)
            uValues[(j*N)+i] = u[i][j];
           
    
    // delete u;
	for(i=0; i< N; i++){
		free(u[i]);
        free(bdryCond[i]);
        free(speed[i]);
        free(Alive[i]);
        free(NarrowBand[i]);
	}
	free(u);
    free(bdryCond);
    free(speed);
    free(Alive);
    free(NarrowBand);
    
}
