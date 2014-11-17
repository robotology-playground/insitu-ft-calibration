#include "calibration_matrix_estimator.h"

#include "eigen_wrappers.h"

#include <Eigen/StdVector>

#include <iostream>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)

namespace InSituFTCalibration
{

struct CalibrationMatrixEstimator::CalibrationMatrixEstimatorPrivateAttributes
{
    std::vector<ForceTorqueOffsetEstimator> datasets; ///< Vector of the considered datasets
    std::vector<double> added_masses;
    std::vector<Eigen::Vector3d> added_coms;
    std::vector<std::string> dataset_names;
    Eigen::Matrix<double,6,6>  estimated_calibration_matrix; //< Offset value estimated by the algorithm
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CalibrationMatrixEstimator::CalibrationMatrixEstimator():
    pimpl(new CalibrationMatrixEstimatorPrivateAttributes)
{
}

CalibrationMatrixEstimator::CalibrationMatrixEstimator(const CalibrationMatrixEstimator& other)
    : pimpl(new CalibrationMatrixEstimatorPrivateAttributes(*(other.pimpl)))
{
}


CalibrationMatrixEstimator& CalibrationMatrixEstimator::operator=(const CalibrationMatrixEstimator &other) {
    if(this != &other)
    {
        *pimpl = *(other.pimpl);
    }
    return *this;
}


CalibrationMatrixEstimator::~CalibrationMatrixEstimator()
{
    delete pimpl;
    this->pimpl = 0;
}

bool CalibrationMatrixEstimator::reset()
{
    pimpl->datasets.resize(0);
    pimpl->added_masses.resize(0);
    pimpl->added_coms.resize(0);
    pimpl->dataset_names.resize(0);
    return true;
}

int CalibrationMatrixEstimator::getNrOfDatasets() const
{
    return pimpl->datasets.size();
}

int CalibrationMatrixEstimator::getNrOfSamples() const
{
    int totalNrOfSamples = 0;
    for(int i=0; i < getNrOfDatasets(); i++ )
    {
        totalNrOfSamples += pimpl->datasets[i].getNrOfSamples();
    }
    return totalNrOfSamples;
}

int CalibrationMatrixEstimator::addCalibrationDataset(std::string dataset_name,
                                                      double added_mass,
                                                      const VecWrapper _added_com)
{
    Eigen::Vector3d added_com = toEigen(_added_com);

    pimpl->dataset_names.push_back(dataset_name);
    pimpl->added_masses.push_back(added_mass);
    pimpl->added_coms.push_back(added_com);
    pimpl->datasets.push_back(ForceTorqueOffsetEstimator());

    return pimpl->dataset_names.size()-1;
}

bool CalibrationMatrixEstimator::getCalibrationDataset(const int dataset_id,
                                                       ForceTorqueOffsetEstimator *& p_dataset) const
{
    double dummy_double;
    Eigen::Vector3d dummy;
    std::string dummy_string;
    return this->getCalibrationDataset(dataset_id,p_dataset,dummy_double,wrapVec(dummy),dummy_string);
}

bool CalibrationMatrixEstimator::getCalibrationDataset(const std::string & dataset_name,
                                 ForceTorqueOffsetEstimator *& p_dataset) const
{
    double dummy_double;
    Eigen::Vector3d dummy;
    int dummy_int;
    return this->getCalibrationDataset(dataset_name,p_dataset,dummy_double,wrapVec(dummy),dummy_int);
}

bool CalibrationMatrixEstimator::getCalibrationDataset(const int dataset_id,
                                                       ForceTorqueOffsetEstimator *& p_dataset,
                                                       double & added_mass,
                                                       const VecWrapper added_com,
                                                       std::string & dataset_name) const
{
    if(dataset_id < 0 || dataset_id >= this->getNrOfDatasets())
    {
        return false;
    }

    p_dataset = &(pimpl->datasets[dataset_id]);
    added_mass = pimpl->added_masses[dataset_id];
    toEigen(added_com) = pimpl->added_coms[dataset_id];
    dataset_name = pimpl->dataset_names[dataset_id];

    return true;
}

bool CalibrationMatrixEstimator::getCalibrationDataset(const std::string & dataset_name,
                                                       ForceTorqueOffsetEstimator *& p_dataset,
                                                       double & added_mass,
                                                       const VecWrapper added_com,
                                                       int & dataset_id) const
{
    for(int dataset=0; dataset < this->getNrOfDatasets(); dataset++ )
    {
        if( pimpl->dataset_names[dataset] == dataset_name )
        {
            dataset_id = dataset;
            std::string dummy;
            return getCalibrationDataset(dataset_id,
                                         p_dataset,
                                         added_mass,
                                         added_com,
                                         dummy);
        }
    }
    return false;
}

Eigen::Matrix<double,18,4> P_matrix()
{
    Eigen::Matrix<double,18,4> wrench_regressor;
    wrench_regressor.setZero();
    wrench_regressor(0,0) = 1;
    wrench_regressor(7,0) = 1;
    wrench_regressor(14,0) = 1;
    wrench_regressor(11,1) = 1;
    wrench_regressor(16,1) = -1;
    wrench_regressor(15,2) = 1;
    wrench_regressor(5,2) = -1;
    wrench_regressor(4,3) = 1;
    wrench_regressor(9,3) = -1;
    return wrench_regressor;
}

Eigen::Matrix<double,6,40>  calibration_matrix_regressor(const Eigen::Matrix<double,6,1> & raw_measures,
                                                         const Eigen::Vector3d & grav,
                                                         const Eigen::Matrix<double,18,4> & P)
{
    Eigen::Matrix<double,6,40> regr;
    regr.setZero();
    for(int i=0; i < 6; i++ )
    {
        regr.block<6,6>(0,6*i) = raw_measures(i)*Eigen::Matrix<double,6,6>::Identity();
    }
    Eigen::Matrix<double,6,18> gKronI;
    for(int i=0; i< 3; i++ )
    {
        gKronI.block<6,6>(0,6*i) = grav(i)*Eigen::Matrix<double,6,6>::Identity();
    }
    //FIXME find a more compact (and efficient) expression
    regr.block<6,4>(0,36) = -gKronI*P;
    return regr;
}

Eigen::Matrix<double,6,1> getAddedMassesWrench(double mass,
                                              const Eigen::Vector3d & com,
                                              const Eigen::Vector3d & grav)
{
    Eigen::Matrix<double,6,1> ret;

    ret.segment<3>(0) = mass*grav;
    ret.segment<3>(3) = mass*com.cross(grav);

    return ret;
}

bool CalibrationMatrixEstimator::computeCalibrationMatrixEstimation(std::string & error_msg)
{
    Eigen::Matrix<double,40,40> ThetaTransposeTheta;
    Eigen::Matrix<double,40,1> x;
    Eigen::Matrix<double,40,1> ThetaTransposeBeta;
    Eigen::Matrix<double,18,4> P = P_matrix();

    ThetaTransposeTheta.setZero();
    x.setZero();
    ThetaTransposeBeta.setZero();

    for(int dataset=0; dataset < this->getNrOfDatasets(); dataset++ )
    {
        double added_mass;
        Eigen::Vector3d added_com;
        std::string dummy;
        ForceTorqueOffsetEstimator * p_dataset;
        this->getCalibrationDataset(dataset,
                                    p_dataset,
                                    added_mass,
                                    wrapVec(added_com),
                                    dummy);

        p_dataset->computeOffsetEstimation();

        for(int smpl=0; smpl < p_dataset->getNrOfSamples(); smpl++ )
        {
            Eigen::Matrix<double,6,1> raw;
            Eigen::Vector3d grav;

            p_dataset->getMeasurementsWithoutFTOffset(smpl,wrapVec(raw),wrapVec(grav));

            Eigen::Matrix<double,6,40> Theta = calibration_matrix_regressor(raw,grav,P);
            Eigen::Matrix<double,6,1> Beta = getAddedMassesWrench(added_mass,added_com,grav);

            ThetaTransposeTheta += Theta.transpose()*Theta;
            ThetaTransposeBeta  += Theta.transpose()*Beta;
        }
    }

    Eigen::JacobiSVD<Eigen::Matrix<double,40,40>, Eigen::HouseholderQRPreconditioner>
          svd_TTT(ThetaTransposeTheta, Eigen::ComputeFullU | Eigen::ComputeFullV);

    x = svd_TTT.solve(ThetaTransposeBeta);

    std::cout << "[INFO] svd problems singular value " << svd_TTT.singularValues() << std::endl;
    std::cout << "[INFO] TTT " << std::endl << ThetaTransposeTheta << std::endl;
    std::cout << "[INFO] estimated mass and m*com " << x.segment<4>(36) << std::endl;

    for(int i=0; i < 6; i++ )
    {
        for(int j=0; j < 6; j++ )
        {
            this->pimpl->estimated_calibration_matrix(i,j) = x(j*6+i);
        }
    }

    return true;
}

bool CalibrationMatrixEstimator::getEstimatedCalibrationMatrix(const MatWrapper _estimated_calibration_matrix)
{
    if( _estimated_calibration_matrix.rows != 6 ||
        _estimated_calibration_matrix.cols != 6 ||
        _estimated_calibration_matrix.storage_order != COLUMN_MAJOR )
    {
        return false;
    }

    toEigen(_estimated_calibration_matrix) = this->pimpl->estimated_calibration_matrix;
    return true;
}



}