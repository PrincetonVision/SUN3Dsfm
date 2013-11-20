#include <libfreenect/libfreenect.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>

int get_data_size(FILE *fp) {
  int orig = ftell(fp);
  fseek(fp, 0L, SEEK_END);
  int out = ftell(fp);
  fseek(fp, orig, SEEK_SET);
  return out;
}

void mexFunction(int nlhs, mxArray* plhs[], const int nrhs, const mxArray* prhs[]) {
  if (nrhs != 1) {
    mexErrMsgTxt("Number of arguments must be exactly 1.");
  } else if (!mxIsChar(prhs[0])) {
    mexErrMsgTxt("Input must be a string.");
  }

  // get the length of the filename.
  mwSize filename_length = (mxGetM(prhs[0]) * mxGetN(prhs[0])) + 1;
  char *filename = mxArrayToString(prhs[0]);
  
  FILE* fp = fopen(filename, "r");
  if (fp == NULL) {
    mexErrMsgIdAndTxt("filename:notFound", "file %s not found", filename);
  }
  
  int data_size = get_data_size(fp);
  if (data_size != sizeof(freenect_raw_tilt_state)) {
    mexErrMsgIdAndTxt("filename:notAccel",
        "file %s's size doesnt match freenect_raw_tilt_state.", filename);
  }
  
  freenect_raw_tilt_state state;
  fread(&state, sizeof(state), 1, fp);
  
  mwSize ndim = 2;
  const mwSize dim_size[] = {4, 1};
  plhs[0] = mxCreateNumericArray(ndim, dim_size, mxDOUBLE_CLASS, mxREAL);
  double* output_data = (double*) mxGetData(plhs[0]);
  output_data[0] = state.accelerometer_x;
  output_data[1] = state.accelerometer_y;
  output_data[2] = state.accelerometer_z;
  output_data[3] = state.tilt_angle;
  
  fclose(fp);
}
