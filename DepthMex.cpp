#include "mex.h"
#include <algorithm>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    int rows = (int)mxGetScalar(prhs[0]);
    int columns = (int)mxGetScalar(prhs[1]);
    int nPts    = mxGetN(prhs[2]);
    double* index = mxGetPr(prhs[2]);
    double* value = mxGetPr(prhs[3]);
    
    plhs[0] = mxCreateNumericMatrix(rows, columns, mxDOUBLE_CLASS, mxREAL);
    double* depth = (double*)mxGetData(plhs[0]);
    
    double* indexEnd = index + nPts;
    while(index<indexEnd){
        double* d = depth + (int)(*index++);
        if (*d==0){
            *d = *value++;
        }else{
            *d = std::min(*d,*value++);
        }
    }
}