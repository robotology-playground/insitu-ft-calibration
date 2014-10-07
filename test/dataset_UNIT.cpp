#include  "dataset.h"

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



int main()
{
    InSituFTCalibration::ForceTorqueAccelerometerDataset dataset;
    
    custom_assert_true(dataset.getNrOfSamples() == 0,"Sample in a new dataset are different from zero");
    
    int n = 10;
    double tol = 1e-5;
    for( int i=0; i < n; i++ )
    {
        Eigen::Vector3d acc, acc2;
        acc << 1,2,3;
        Eigen::Matrix<double,6,1> ft, ft2;
        ft << 1,2,3, 1,2,3;
        
        custom_assert_true(dataset.getNrOfSamples() == i,"Testing size of dataset before adding sample");
        dataset.addMeasurements(ft,acc);
        custom_assert_true(dataset.getNrOfSamples() == i+1,"Testing size of dataset before adding sample");

        
        dataset.getMeasurements(i,ft2,acc2);
        
        //std::cout << "[INFO] ft inserted:  " << ft.transpose() << std::endl;
        //std::cout << "[INFO] ft retrieved: " << ft2.transpose() << std::endl;

        custom_assert_true((ft-ft2).norm()<tol,"Bug in addMeasurements, getMeasurements");
        custom_assert_true((acc-acc2).norm()<tol,"Bug in addMeasurements, getMeasurements");
        
    }
    
    custom_assert_true(dataset.getNrOfSamples() == n,"Number of sample in dataset is not consistent");
    
    dataset.reset();
    
    custom_assert_true(dataset.getNrOfSamples() == 0,"Number of sample after reset is not consistent");

    return EXIT_SUCCESS;
}