#include  "force_calibration_matrix_estimator.h"

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <sstream>

#include <Eigen/Dense>
#include <Eigen/LU>

#include <Eigen/StdVector>

#include "eigen_wrappers.h"

using namespace InSituFTCalibration;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<double,6,1>)

template<typename T>
std::string toString(const T& value)
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

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
    InSituFTCalibration::ForceCalibrationMatrixEstimator calibration_matrix_estimator;

    custom_assert_true(calibration_matrix_estimator.getNrOfSamples() == 0,"Sample in a new dataset are different from zero");

    double tol = 1e-4;
    int n = 1000;
    double g = 0.098;
    int n_datasets = 20;

    double unknown_mass = 0.5;
    Eigen::Vector3d unknown_com;
    unknown_com << 1,2,3;

    std::vector<double> masses;
    std::vector< Eigen::Vector3d> coms;
    std::vector< Eigen::Matrix<double,6,1> > offsets;
    std::vector< int > n_samples;

    masses.resize(n_datasets);
    coms.resize(n_datasets);
    offsets.resize(n_datasets);
    n_samples.resize(n_datasets);

    for(int i=0; i < n_datasets; i++ )
    {
        masses[i] = Eigen::Vector3d::Random()[0];
        coms[i] = Eigen::Vector3d::Random();
        offsets[i] << Eigen::Matrix<double,6,1>::Random();
        n_samples[i] = n;
    }

    //Eigen::Matrix<double,3,6> calibration_matrix = Eigen::Matrix<double,6,6>::Random();
    Eigen::Matrix<double,6,6> shape_matrix =  Eigen::Matrix<double,6,6>::Random();

    for(int dataset=0; dataset < n_datasets; dataset++ )
    {
        InSituFTCalibration::ForceTorqueOffsetEstimator * p_offset_estimator = 0;

        std::string dataset_name = "dataset" + toString(dataset);
        int dataset_id = calibration_matrix_estimator.addCalibrationDataset(dataset_name,masses[dataset]);
        double return_mass;
        Eigen::Vector3d return_com;
        std::string return_name;

        custom_assert_true(dataset_id == dataset,"Value return by addDataset method is inconsistent");


        custom_assert_true(calibration_matrix_estimator.getCalibrationDataset(dataset_name,p_offset_estimator),"getDataset with name not working");

        custom_assert_true(calibration_matrix_estimator.getCalibrationDataset(dataset_id,p_offset_estimator),"getDataset with id not working");

        custom_assert_true(p_offset_estimator->getNrOfSamples() == 0,"get samples before adding the calibration samples is not coherent");
        //std::cout << "[INFO] p_offset_estimator->getNrOfSamples() before adding samples " << p_offset_estimator->getNrOfSamples() << std::endl;


        //std::cout << "[INFO]  n_samples[dataset]" << n_samples[dataset] << std::endl;
        for( int smp=0; smp < n_samples[dataset]; smp++ )
        {
            //Create random gravity vector
            Eigen::Vector3d acc = Eigen::Vector3d::Random();
            acc *= g/acc.norm();

            custom_assert_close(acc.norm(),g,tol,"Generated random gravity has not a proper norm");

            Eigen::Matrix<double,6,1>  ft = shape_matrix*(GravityRegressor(unknown_mass+masses[dataset],
                                                                           unknown_com+coms[dataset])*acc) + offsets[dataset];


            //std::cout << "[INFO] acc: " << acc.transpose() << std::endl;
            //std::cout << "[INFO] ft:  " << ft.transpose() << std::endl;

            //std::cout << "[INFO] p_offset_estimator->getNrOfSamples() before adding sample  " << p_offset_estimator->getNrOfSamples() << std::endl;
            p_offset_estimator->addMeasurements(wrapVec(ft),wrapVec(acc));
            //std::cout << "[INFO] p_offset_estimator->getNrOfSamples() before after adding sample  " << p_offset_estimator->getNrOfSamples() << std::endl;
        }

        //std::cout << "[INFO] p_offset_estimator->getNrOfSamples() after adding samples  " << p_offset_estimator->getNrOfSamples() << std::endl;
        custom_assert_true(p_offset_estimator->getNrOfSamples() == n_samples[dataset],"get samples after adding the calibration samples is not coherent");

        //std::cout << "[INFO] total samples used of calibration: " << calibration_matrix_estimator.getNrOfSamples() << " expected " << n*n_datasets << std::endl;
        //custom_assert_true(calibration_matrix_estimator.getNrOfSamples() == (dataset+1)*n_datasets,"Number of sample in dataset is not consistent");
    }

    // Validation for checking number of samples in each dataset
    for(int dataset=0; dataset < n_datasets; dataset++ )
    {
        InSituFTCalibration::ForceTorqueOffsetEstimator * p_offset_estimator = 0;
        custom_assert_true(calibration_matrix_estimator.getCalibrationDataset(dataset,p_offset_estimator),"getDataset with id not working");

        //std::cout << "[INFO] p_offset_estimator->getNrOfSamples() for dataset " << dataset << " :  " << p_offset_estimator->getNrOfSamples() << std::endl;
        custom_assert_true(p_offset_estimator->getNrOfSamples() == n_samples[dataset],"get samples after adding the calibration samples is not coherent");

        p_offset_estimator->computeOffsetEstimation();

        Eigen::Matrix<double,6,1> estimated_offset;
        p_offset_estimator->getEstimatedOffset(wrapVec(estimated_offset));

        /*
        std::cout << "[INFO] offset information for dataset " << dataset << std::endl;
        std::cout << "[INFO] true offset: " << offsets[dataset].transpose() << std::endl;
        std::cout << "[INFO] estimated offset: " << estimated_offset.transpose() << std::endl;
        std::cout << "[INFO] true-estimated offset: " << (offsets[dataset]-estimated_offset).transpose() << std::endl;
        */
    }

    std::cout << "[INFO] total samples used of calibration: " << calibration_matrix_estimator.getNrOfSamples() << " expected " << n*n_datasets << std::endl;
    custom_assert_true(calibration_matrix_estimator.getNrOfSamples() == n*n_datasets,"Number of sample in dataset is not consistent");

    std::string dummy;
    custom_assert_true(calibration_matrix_estimator.computeForceCalibrationMatrixEstimation(dummy),"Calibration matrix computation failed");

    Eigen::Matrix<double,3,6>  estimated_calibration_matrix;
    custom_assert_true(calibration_matrix_estimator.getEstimatedForceCalibrationMatrix(wrapMat(estimated_calibration_matrix)),"Calibration matrix retrieval failed");

    std::cout << "[INFO] estimated calibration matrix:  \n" << estimated_calibration_matrix << std::endl;
    std::cout << "[INFO] estimated calibration mat * true shape_matrix :\n" << estimated_calibration_matrix*shape_matrix << std::endl;
    std::cout << "[INFO] true shape_matrix :\n" << shape_matrix << std::endl;

    Eigen::Matrix<double,3,6> obtained_matrix, desired_matrix;
    obtained_matrix = estimated_calibration_matrix*shape_matrix;
    desired_matrix.setZero();
    desired_matrix.block<3,3>(0,0) = Eigen::Matrix3d::Identity();


    for(int i=0; i < 3; i++ )
    {
        for(int j=0; j < 6; j++ )
        {
            custom_assert_close(obtained_matrix(i,j),desired_matrix(i,j),tol,"Difference between desired calib*shaped and obtained one");
        }
    }

    calibration_matrix_estimator.reset();

    custom_assert_true(calibration_matrix_estimator.getNrOfSamples() == 0,"Number of sample after reset is not consistent");

    return EXIT_SUCCESS;
}
