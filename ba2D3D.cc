
#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"



double fx, fy, px, py, w3Dv2D;

#define EPS T(0.00001)

int exe_time=0;



struct AlignmentErrorTriangulate {
  AlignmentErrorTriangulate(double* observed_in, double* camera_extrinsic_in): observed(observed_in), camera_extrinsic(camera_extrinsic_in) {}

  template <typename T>
  bool operator()(const T* const point, T* residuals) const {
                  
    // camera_extrinsic[0,1,2] are the angle-axis rotation.
    T p[3];
    
    ceres::AngleAxisRotatePoint((T*)(camera_extrinsic), point, p);

    // camera_extrinsic[3,4,5] are the translation.
    p[0] += T(camera_extrinsic[3]);
    p[1] += T(camera_extrinsic[4]);
    p[2] += T(camera_extrinsic[5]);
    
    // let p[2] ~= 0
    if (T(0.0)<=p[2]){
        if(p[2]<EPS){
            p[2] = EPS;
        }
    }else{
        if (p[2]>-EPS){
            p[2] = -EPS;
        }
    }
    
    // project it
    p[0] = T(fx) * p[0] / p[2] + T(px);
    p[1] = T(fy) * p[1] / p[2] + T(py);
    
    // reprojection error
    residuals[0] = (p[0] - T(observed[0]));
    residuals[1] = (p[1] - T(observed[1]));     

    /*
    std::cout<<"p[0]="<<p[0]<<std::endl;
    std::cout<<"p[1]="<<p[1]<<std::endl;
    std::cout<<"observed[0]="<<observed[0]<<std::endl;
    std::cout<<"observed[1]="<<observed[1]<<std::endl;
    std::cout<<"residuals[0]="<<residuals[0]<<std::endl;
    std::cout<<"residuals[1]="<<residuals[1]<<std::endl;
    std::cout<<"--------------------------"<<std::endl;
    */
    return true;
  }
  
  double* observed;
  double* camera_extrinsic;
};


struct AlignmentError2D {
  AlignmentError2D(double* observed_in): observed(observed_in) {}

  template <typename T>
  bool operator()(const T* const camera_extrinsic,
                  const T* const point,
                  T* residuals) const {
                  
    // camera_extrinsic[0,1,2] are the angle-axis rotation.
    T p[3];
    
    ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
    /*
    T x = camera_extrinsic[0];
    T y = camera_extrinsic[1];
    T z = camera_extrinsic[2];
    T x2 = x*x;
    T y2 = y*y;
    T z2 = z*z;    
    T w2 = T(1.0) - x2 - y2 - z2;
    T w  = sqrt(w2);
    
    p[0] = point[0]*(w2 + x2 - y2 - z2) - point[1]*(T(2.0)*w*z - T(2.0)*x*y) + point[2]*(T(2.0)*w*y + T(2.0)*x*z);
    p[1] = point[1]*(w2 - x2 + y2 - z2) + point[0]*(T(2.0)*w*z + T(2.0)*x*y) - point[2]*(T(2.0)*w*x - T(2.0)*y*z);
    p[2] = point[2]*(w2 - x2 - y2 + z2) - point[0]*(T(2.0)*w*y - T(2.0)*x*z) + point[1]*(T(2.0)*w*x + T(2.0)*y*z);
    */
    
    // camera_extrinsic[3,4,5] are the translation.
    p[0] += camera_extrinsic[3];
    p[1] += camera_extrinsic[4];
    p[2] += camera_extrinsic[5];
    
    // let p[2] ~= 0
    if (T(0.0)<=p[2]){
        if(p[2]<EPS){
            p[2] = EPS;
        }
    }else{
        if (p[2]>-EPS){
            p[2] = -EPS;
        }
    }
    
    // project it
    p[0] = T(fx) * p[0] / p[2] + T(px);
    p[1] = T(fy) * p[1] / p[2] + T(py);
    
    // reprojection error
    residuals[0] = p[0] - T(observed[0]);
    residuals[1] = p[1] - T(observed[1]);     
    
    /*
    if (exe_time<10000){
        exe_time++;
        std::cout<<"p[0]="<<p[0]<<std::endl;
        std::cout<<"p[1]="<<p[1]<<std::endl;
        std::cout<<"observed[0]="<<observed[0]<<std::endl;
        std::cout<<"observed[1]="<<observed[1]<<std::endl;
        std::cout<<"residuals[0]="<<residuals[0]<<std::endl;
        std::cout<<"residuals[1]="<<residuals[1]<<std::endl;        
        std::cout<<"--------------------------"<<std::endl;
    }
    */
    
    return true;
  }
  
  double* observed;

};


struct AlignmentError3D {
  AlignmentError3D(double* observed_in): observed(observed_in) {}

  template <typename T>
  bool operator()(const T* const camera_extrinsic,
                  const T* const point,
                  T* residuals) const {
                  
    // camera_extrinsic[0,1,2] are the angle-axis rotation.
    T p[3];
    
    ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
    /*
    T x = camera_extrinsic[0];
    T y = camera_extrinsic[1];
    T z = camera_extrinsic[2];
    T x2 = x*x;
    T y2 = y*y;
    T z2 = z*z;    
    T w2 = T(1.0) - x2 - y2 - z2;
    T w  = sqrt(w2);
    
    p[0] = point[0]*(w2 + x2 - y2 - z2) - point[1]*(T(2.0)*w*z - T(2.0)*x*y) + point[2]*(T(2.0)*w*y + T(2.0)*x*z);
    p[1] = point[1]*(w2 - x2 + y2 - z2) + point[0]*(T(2.0)*w*z + T(2.0)*x*y) - point[2]*(T(2.0)*w*x - T(2.0)*y*z);
    p[2] = point[2]*(w2 - x2 - y2 + z2) - point[0]*(T(2.0)*w*y - T(2.0)*x*z) + point[1]*(T(2.0)*w*x + T(2.0)*y*z);
    */
    
    // camera_extrinsic[3,4,5] are the translation.
    p[0] += camera_extrinsic[3];
    p[1] += camera_extrinsic[4];
    p[2] += camera_extrinsic[5];
    
    // The error is the difference between the predicted and observed position.
    residuals[0] = (p[0] - T(observed[2]));
    residuals[1] = (p[1] - T(observed[3]));
    residuals[2] = (p[2] - T(observed[4]));


    /*    
     if (exe_time<10){
        exe_time ++;
        std::cout<<"fx="<<fx<<std::endl;
        std::cout<<"fy="<<fy<<std::endl;
        std::cout<<"px="<<px<<std::endl;
        std::cout<<"py="<<py<<std::endl;
        std::cout<<"w3Dv2D="<<w3Dv2D<<std::endl;        
        std::cout<<"p[0]="<<p[0]<<std::endl;
        std::cout<<"p[1]="<<p[1]<<std::endl;
        std::cout<<"p[2]="<<p[2]<<std::endl;
        std::cout<<"observed[0]="<<observed[0]<<std::endl;
        std::cout<<"observed[1]="<<observed[1]<<std::endl;
        std::cout<<"observed[2]="<<observed[2]<<std::endl;
        std::cout<<"observed[3]="<<observed[3]<<std::endl;
        std::cout<<"observed[4]="<<observed[4]<<std::endl;
        std::cout<<"residuals[0]="<<residuals[0]<<std::endl;
        std::cout<<"residuals[1]="<<residuals[1]<<std::endl;
        std::cout<<"residuals[2]="<<residuals[2]<<std::endl;
        std::cout<<"--------------------------"<<std::endl;
    }
    */


    return true;
  }
  double* observed;
};


struct AlignmentError2D3D {
  AlignmentError2D3D(double* observed_in): observed(observed_in) {}

  template <typename T>
  bool operator()(const T* const camera_extrinsic,
                  const T* const point,
                  T* residuals) const {
                  
    // camera_extrinsic[0,1,2] are the angle-axis rotation.
    T p[3];
    
    ceres::AngleAxisRotatePoint(camera_extrinsic, point, p);
    /*
    T x = camera_extrinsic[0];
    T y = camera_extrinsic[1];
    T z = camera_extrinsic[2];
    T x2 = x*x;
    T y2 = y*y;
    T z2 = z*z;    
    T w2 = T(1.0) - x2 - y2 - z2;
    T w  = sqrt(w2);
    
    p[0] = point[0]*(w2 + x2 - y2 - z2) - point[1]*(T(2.0)*w*z - T(2.0)*x*y) + point[2]*(T(2.0)*w*y + T(2.0)*x*z);
    p[1] = point[1]*(w2 - x2 + y2 - z2) + point[0]*(T(2.0)*w*z + T(2.0)*x*y) - point[2]*(T(2.0)*w*x - T(2.0)*y*z);
    p[2] = point[2]*(w2 - x2 - y2 + z2) - point[0]*(T(2.0)*w*y - T(2.0)*x*z) + point[1]*(T(2.0)*w*x + T(2.0)*y*z);
    */
    
    
    // camera_extrinsic[3,4,5] are the translation.
    p[0] += camera_extrinsic[3];
    p[1] += camera_extrinsic[4];
    p[2] += camera_extrinsic[5];
    
    // The error is the difference between the predicted and observed position.
    residuals[2] = (p[0] - T(observed[2]))*w3Dv2D;
    residuals[3] = (p[1] - T(observed[3]))*w3Dv2D;
    residuals[4] = (p[2] - T(observed[4]))*w3Dv2D;

    // let p[2] ~= 0
    if (T(0.0)<=p[2]){
        if(p[2]<EPS){
            p[2] = EPS;
        }
    }else{
        if (p[2]>-EPS){
            p[2] = -EPS;
        }
    }
    
    // project it
    p[0] = T(fx) * p[0] / p[2] + T(px);
    p[1] = T(fy) * p[1] / p[2] + T(py);
    
    // reprojection error
    residuals[0] = p[0] - T(observed[0]);
    residuals[1] = p[1] - T(observed[1]);     
    
    /*
     if (exe_time<10){
        exe_time ++;
        std::cout<<"fx="<<fx<<std::endl;
        std::cout<<"fy="<<fy<<std::endl;
        std::cout<<"px="<<px<<std::endl;
        std::cout<<"py="<<py<<std::endl;
        std::cout<<"w3Dv2D="<<w3Dv2D<<std::endl;
        std::cout<<"p[0]="<<p[0]<<std::endl;
        std::cout<<"p[1]="<<p[1]<<std::endl;
        std::cout<<"observed[0]="<<observed[0]<<std::endl;
        std::cout<<"observed[1]="<<observed[1]<<std::endl;
        std::cout<<"residuals[0]="<<residuals[0]<<std::endl;
        std::cout<<"residuals[1]="<<residuals[1]<<std::endl;
        std::cout<<"residuals[2]="<<residuals[2]<<std::endl;
        std::cout<<"residuals[3]="<<residuals[3]<<std::endl;
        std::cout<<"residuals[4]="<<residuals[4]<<std::endl;
        std::cout<<"--------------------------"<<std::endl;
    }
    */
    return true;
  }
  double* observed;
};


int main(int argc, char** argv)
{
  //std::cout<<"sizeof(unsigned int)="<<sizeof(unsigned int)<<std::endl;
  //std::cout<<"sizeof(double)="<<sizeof(double)<<std::endl;
  std::cout<<"Ba2D3D bundle adjuster in 2D and 3D. Writen by Jianxiong Xiao."<<std::endl;
  std::cout<<"Usage: EXE mode(1,2,3,5) w3Dv2D input_file_name output_file_name"<<std::endl;


  int mode = atoi(argv[1]);
  w3Dv2D = atof(argv[2]);

  // start reading input file
  FILE* fp = fopen(argv[3],"rb");
  if (fp==NULL) { std::cout<<"fail to open file"<<std::endl; return false;}
  // read header count
  unsigned int nCam;  fread((void*)(&nCam), sizeof(unsigned int), 1, fp);
  unsigned int nPts;  fread((void*)(&nPts), sizeof(unsigned int), 1, fp);
  unsigned int nObs;  fread((void*)(&nObs), sizeof(unsigned int), 1, fp);
  // read camera intrinsic
  fread((void*)(&fx), sizeof(double), 1, fp);
  fread((void*)(&fy), sizeof(double), 1, fp);
  fread((void*)(&px), sizeof(double), 1, fp);
  fread((void*)(&py), sizeof(double), 1, fp);
  // read camera extrinsic
  double* cameraRt = new double [12*nCam];
  fread((void*)(cameraRt), sizeof(double), 12*nCam, fp);
  // read initial 3D point position
  double* pointCloud = new double [3*nPts];
  fread((void*)(pointCloud), sizeof(double), 3*nPts, fp);
  // observation
  unsigned int* pointObservedIndex = new unsigned int [2*nObs];
  double* pointObservedValue = new double [5*nObs];
  fread((void*)(pointObservedIndex), sizeof(unsigned int), 2*nObs, fp);
  fread((void*)(pointObservedValue), sizeof(double), 5*nObs, fp);
  // finish reading
  fclose(fp);

  // output info
  std::cout<<"Parameters: ";
  std::cout<<"mode="<<mode<<" ";
  std::cout<<"w3Dv2D="<<w3Dv2D<<"\t"; //<<std::endl;  

  std::cout<<"Meta Info: ";
  std::cout<<"nCam="<<nCam<<" ";
  std::cout<<"nPts="<<nPts<<" ";
  std::cout<<"nObs="<<nObs<<"\t"; //<<std::endl;

  std::cout<<"Camera Intrinsic: ";
  std::cout<<"fx="<<fx<<" ";
  std::cout<<"fy="<<fy<<" ";
  std::cout<<"px="<<px<<" ";
  std::cout<<"py="<<py<<"\t"<<std::endl;


  // construct camera parameters from camera matrix
  double* cameraParameter = new double [6*nCam];
  for(int cameraID=0; cameraID<nCam; ++cameraID){
      double* cameraPtr = cameraParameter+6*cameraID;
      double* cameraMat = cameraRt+12*cameraID;
      if (!(std::isnan(*cameraPtr))){
          ceres::RotationMatrixToAngleAxis<double>(cameraMat, cameraPtr);
          cameraPtr[3] = cameraMat[9];
          cameraPtr[4] = cameraMat[10];
          cameraPtr[5] = cameraMat[11];
          //std::cout<<"cameraID="<<cameraID<<" : ";
          //std::cout<<"cameraPtr="<<cameraPtr[0]<<" "<<cameraPtr[1]<<" "<<cameraPtr[2]<<" "<<cameraPtr[3]<<" "<<cameraPtr[4]<<" "<<cameraPtr[5]<<std::endl;
      }
  }


  //exe_time = 0;    
  
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  
  ceres::LossFunction* loss_function = NULL; // squared loss
  //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
  //ceres::LossFunction* loss_function = new ceres::ArctanLoss(10.0);
  
  //----------------------------------------------------------------

  for (unsigned int idObs=0; idObs<nObs; ++idObs){

    double* cameraPtr = cameraParameter + pointObservedIndex[2*idObs] * 6;
    double* pointPtr  = pointCloud + pointObservedIndex[2*idObs+1] * 3;
    double* observePtr = pointObservedValue+5*idObs;

    ceres::CostFunction* cost_function;
    switch (mode){
      case 1:
        // 2D triangulation
        cost_function = new ceres::AutoDiffCostFunction<AlignmentErrorTriangulate, 2, 3>(new AlignmentErrorTriangulate(observePtr,cameraPtr));
        problem.AddResidualBlock(cost_function,loss_function,pointPtr);
        break;
      case 2:
        // 2D bundle adjustment
        cost_function = new ceres::AutoDiffCostFunction<AlignmentError2D, 2, 6, 3>(new AlignmentError2D(observePtr));
        problem.AddResidualBlock(cost_function,loss_function,cameraPtr,pointPtr);
        break;
      case 3:
        // 3D bundle adjustment
        cost_function = new ceres::AutoDiffCostFunction<AlignmentError3D, 3, 6, 3>(new AlignmentError3D(observePtr));
        problem.AddResidualBlock(cost_function,loss_function,cameraPtr,pointPtr);
        break;
      case 5:
        // 5D bundle adjustment
        if (std::isnan(observePtr[2])){
          cost_function = new ceres::AutoDiffCostFunction<AlignmentError2D,   2, 6, 3>(new AlignmentError2D(observePtr));
        }else{
          cost_function = new ceres::AutoDiffCostFunction<AlignmentError2D3D, 5, 6, 3>(new AlignmentError2D3D(observePtr));
        }
        problem.AddResidualBlock(cost_function,loss_function,cameraPtr,pointPtr);
        break;
    }
  }

  //----------------------------------------------------------------

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.max_num_iterations = 200;  
  options.minimizer_progress_to_stdout = true;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;  //ceres::SPARSE_SCHUR;  //ceres::DENSE_SCHUR;
  //options.ordering_type = ceres::SCHUR;
  
  /*
  options.linear_solver_type = ceres::DENSE_SCHUR; //ceres::SPARSE_SCHUR; //ceres::DENSE_SCHUR; //ceres::SPARSE_NORMAL_CHOLESKY; //
  options.ordering_type = ceres::SCHUR;
  options.minimizer_progress_to_stdout = true;
  // New options
  //options.preconditioner_type = ceres::JACOBI; // ceres::IDENTITY
  options.num_linear_solver_threads = 12;
  //options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  //options.use_block_amd = true;
  //options.eta=1e-2;
  //options.dogleg_type = ceres::TRADITIONAL_DOGLEG;
  //options.use_nonmonotonic_steps=false;
  */
  
/*
  options.trust_region_strategy_type =  ceres::LEVENBERG_MARQUARDT; // DEFINE_string(trust_region_strategy, "lm", "Options are: lm, dogleg");
  options.eta = 1e-2; //  DEFINE_double(eta, 1e-2, "Default value for eta. Eta determines the accuracy of each linear solve of the truncated newton step. Changing this parameter can affect solve performance ");
  options.linear_solver_type = ceres::SPARSE_SCHUR; //DEFINE_string(solver_type, "sparse_schur", "Options are:  sparse_schur, dense_schur, iterative_schur, sparse_cholesky,  dense_qr, dense_cholesky and conjugate_gradients");
  options.preconditioner_type = ceres::JACOBI; //DEFINE_string(preconditioner_type, "jacobi", "Options are:  identity, jacobi, schur_jacobi, cluster_jacobi,  cluster_tridiagonal");
  options.sparse_linear_algebra_library =  ceres::SUITE_SPARSE; //DEFINE_string(sparse_linear_algebra_library, "suitesparse", "Options are: suitesparse and cxsparse");
  options.ordering_type = ceres::SCHUR; //DEFINE_string(ordering_type, "schur", "Options are: schur, user, natural");
  options.dogleg_type =  ceres::TRADITIONAL_DOGLEG; //DEFINE_string(dogleg_type, "traditional", "Options are: traditional, subspace");
  options.use_block_amd = true; //DEFINE_bool(use_block_amd, true, "Use a block oriented fill reducing ordering.");
  options.num_threads = 1; //DEFINE_int32(num_threads, 1, "Number of threads");
  options.linear_solver_min_num_iterations = 5; //DEFINE_int32(num_iterations, 5, "Number of iterations");
  options.use_nonmonotonic_steps = false; //DEFINE_bool(nonmonotonic_steps, false, "Trust region algorithm can use nonmonotic steps");
//DEFINE_double(rotation_sigma, 0.0, "Standard deviation of camera rotation perturbation.");
//DEFINE_double(translation_sigma, 0.0, "Standard deviation of the camera translation perturbation.");
//DEFINE_double(point_sigma, 0.0, "Standard deviation of the point perturbation");
//DEFINE_int32(random_seed, 38401, "Random seed used to set the state of the pseudo random number generator used to generate the pertubations.");
*/  
  
  //ceres::Solve(options, &problem, NULL);
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //std::cout << summary.FullReport() << std::endl;
  std::cout << summary.BriefReport() << std::endl;
  
  // obtain camera matrix from parameters
  for(int cameraID=0; cameraID<nCam; ++cameraID){
      double* cameraPtr = cameraParameter+6*cameraID;
      double* cameraMat = cameraRt+12*cameraID;
      if (!(std::isnan(*cameraPtr))){
          ceres::AngleAxisToRotationMatrix<double>(cameraPtr, cameraMat);
          cameraMat[9]  = cameraPtr[3];
          cameraMat[10] = cameraPtr[4];
          cameraMat[11] = cameraPtr[5];
          //std::cout<<"cameraID="<<cameraID<<" : ";
          //std::cout<<"cameraPtr="<<cameraPtr[0]<<" "<<cameraPtr[1]<<" "<<cameraPtr[2]<<" "<<cameraPtr[3]<<" "<<cameraPtr[4]<<" "<<cameraPtr[5]<<std::endl;
      }
  }

  // write back result files

  FILE* fpout = fopen(argv[4],"wb");
  fwrite((void*)(&nCam), sizeof(unsigned int), 1, fpout);
  fwrite((void*)(&nPts), sizeof(unsigned int), 1, fpout);
  fwrite((void*)(cameraRt), sizeof(double), 12*nCam, fpout);
  fwrite((void*)(pointCloud), sizeof(double), 3*nPts, fpout);
  fclose (fpout);

  // clean up
  delete [] cameraRt;
  delete [] pointCloud;
  delete [] pointObservedIndex;
  delete [] pointObservedValue;
  delete [] cameraParameter;

  return 0;  
}
