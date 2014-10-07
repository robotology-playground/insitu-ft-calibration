#include  "offset_estimator.h"

#include <cstdlib>
#include <cmath>
#include <iostream>

void custom_assert_true(bool condition, std::string msg)
{
    if(!condition)
    {
        std::cout << "[ERR] " << msg << std::endl;
        exit(EXIT_FAILURE);
    }
}

void custom_assert_close(double op1, double op2, double tol, std::string msg)
{
    if((fabs(op1-op2)>tol))
    {
        std::cout << "[ERR] " << msg << std::endl;
        exit(EXIT_FAILURE);
    }
}

Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d & v)
{
    Eigen::Matrix3d ret;
    ret << 0, -v[2], v[1],
          v[2], 0, -v[0],
          -v[1], v[0], 0;
    return ret;
}

Eigen::Matrix<double,6,3> GravityRegressor(double mass, Eigen::Vector3d com)
{
    Eigen::Matrix<double,6,3> ret;
    ret.block<3,3>(0,0) = mass*Eigen::Matrix3d::Identity();
    ret.block<3,3>(3,0) = mass*crossProductMatrix(com);
    return ret;
}

int main()
{
    InSituFTCalibration::ForceTorqueOffsetEstimator offset_estimator;
    
    custom_assert_true(offset_estimator.getNrOfSamples() == 0,"Sample in a new dataset are different from zero");
    
    double tol = 1e-6;
    int n = 300;
    double g = 9.8;
    double mass = 1.0;
    Eigen::Vector3d com;
    com << 1.0, -2.0, 3.0;
    //com.setZero();
    
    Eigen::Matrix<double,6,1>  true_offset;
    true_offset << 1.0, -2.0, 3.0, -4.0, 5.0, -6.0;
    //true_offset.setZero();
    
    
    for( int i=0; i < n; i++ )
    {
        //Create random gravity vector
        Eigen::Vector3d acc = Eigen::Vector3d::Random();
        acc *= g/acc.norm();
        
        custom_assert_close(acc.norm(),g,tol,"Generated random gravity has not a proper norm");

        Eigen::Matrix<double,6,1>  ft = GravityRegressor(mass,com)*acc + true_offset;
                
        //std::cout << "[INFO] acc: " << acc.transpose() << std::endl;
        //std::cout << "[INFO] ft:  " << ft.transpose() << std::endl;
        
        offset_estimator.addMeasurements(ft,acc);
    }
    
    custom_assert_true(offset_estimator.getNrOfSamples() == n,"Number of sample in dataset is not consistent");
    
    custom_assert_true(offset_estimator.computeOffsetEstimation(),"Offset computation failed");
    
    Eigen::Matrix<double,6,1>  estimated_offset = offset_estimator.getOffset();
    
    std::cout << "[INFO] true offset: " << true_offset.transpose() << std::endl;
    std::cout << "[INFO] estimated offset: " << estimated_offset.transpose() << std::endl;
    std::cout << "[INFO] true-estimated offset: " << (true_offset-estimated_offset).transpose() << std::endl;
    
    
    for(int j=0; j < 6; j++ ) 
    {
        custom_assert_close(true_offset(j),estimated_offset(j),tol,"Different between true offset and estimated offset");
    }
    
    offset_estimator.reset();
    
    custom_assert_true(offset_estimator.getNrOfSamples() == 0,"Number of sample after reset is not consistent");

    return EXIT_SUCCESS;
}