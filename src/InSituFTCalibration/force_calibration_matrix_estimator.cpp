#include "force_calibration_matrix_estimator.h"

#include "eigen_wrappers.h"

#include <Eigen/StdVector>

#include <iostream>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)


namespace InSituFTCalibration
{

const int EstimatedParameters = 6*3+1; //3 6 elements rows + unknown mass
const int CalibrationOutputs   = 3;
const int CalibrationInputs    = 6;

struct ForceCalibrationMatrixEstimator::ForceCalibrationMatrixEstimatorPrivateAttributes
{
    std::vector<ForceTorqueOffsetEstimator> datasets; ///< Vector of the considered datasets
    std::vector<double> added_masses;
    std::vector<std::string> dataset_names;
    Eigen::Matrix<double,CalibrationOutputs,CalibrationInputs>  estimated_calibration_matrix;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

ForceCalibrationMatrixEstimator::ForceCalibrationMatrixEstimator():
    pimpl(new ForceCalibrationMatrixEstimatorPrivateAttributes)
{
}

ForceCalibrationMatrixEstimator::ForceCalibrationMatrixEstimator(const ForceCalibrationMatrixEstimator& other)
    : pimpl(new ForceCalibrationMatrixEstimatorPrivateAttributes(*(other.pimpl)))
{
}


ForceCalibrationMatrixEstimator& ForceCalibrationMatrixEstimator::operator=(const ForceCalibrationMatrixEstimator &other) {
    if(this != &other)
    {
        *pimpl = *(other.pimpl);
    }
    return *this;
}


ForceCalibrationMatrixEstimator::~ForceCalibrationMatrixEstimator()
{
    delete pimpl;
    this->pimpl = 0;
}

bool ForceCalibrationMatrixEstimator::reset()
{
    pimpl->datasets.resize(0);
    pimpl->added_masses.resize(0);
    pimpl->dataset_names.resize(0);
    return true;
}

int ForceCalibrationMatrixEstimator::getNrOfDatasets() const
{
    return pimpl->datasets.size();
}

int ForceCalibrationMatrixEstimator::getNrOfSamples() const
{
    int totalNrOfSamples = 0;
    for(int i=0; i < getNrOfDatasets(); i++ )
    {
        totalNrOfSamples += pimpl->datasets[i].getNrOfSamples();
    }
    return totalNrOfSamples;
}

int ForceCalibrationMatrixEstimator::addCalibrationDataset(std::string dataset_name,
                                                           double added_mass)
{
    pimpl->dataset_names.push_back(dataset_name);
    pimpl->added_masses.push_back(added_mass);
    pimpl->datasets.push_back(ForceTorqueOffsetEstimator());

    return pimpl->dataset_names.size()-1;
}

bool ForceCalibrationMatrixEstimator::getCalibrationDataset(const int dataset_id,
                                                       ForceTorqueOffsetEstimator *& p_dataset) const
{
    double dummy_double;
    std::string dummy_string;
    return this->getCalibrationDataset(dataset_id,p_dataset,dummy_double,dummy_string);
}

bool ForceCalibrationMatrixEstimator::getCalibrationDataset(const std::string & dataset_name,
                                 ForceTorqueOffsetEstimator *& p_dataset) const
{
    double dummy_double;
    int dummy_int;
    return this->getCalibrationDataset(dataset_name,p_dataset,dummy_double,dummy_int);
}

bool ForceCalibrationMatrixEstimator::getCalibrationDataset(const int dataset_id,
                                                       ForceTorqueOffsetEstimator *& p_dataset,
                                                       double & added_mass,
                                                       std::string & dataset_name) const
{
    if(dataset_id < 0 || dataset_id >= this->getNrOfDatasets())
    {
        return false;
    }

    p_dataset = &(pimpl->datasets[dataset_id]);
    added_mass = pimpl->added_masses[dataset_id];
    dataset_name = pimpl->dataset_names[dataset_id];

    return true;
}

bool ForceCalibrationMatrixEstimator::getCalibrationDataset(const std::string & dataset_name,
                                                       ForceTorqueOffsetEstimator *& p_dataset,
                                                       double & added_mass,
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
                                         dummy);
        }
    }
    return false;
}

Eigen::Matrix<double,CalibrationOutputs,EstimatedParameters>  calibration_matrix_regressor(const Eigen::Matrix<double,6,1> & raw_measures,
                                                                                           const Eigen::Vector3d & grav)
{
    Eigen::Matrix<double,CalibrationOutputs,EstimatedParameters> regr;
    regr.setZero();
    for(int i=0; i < CalibrationInputs; i++ )
    {
        regr.block<CalibrationOutputs,CalibrationOutputs>(0,CalibrationOutputs*i) =
             raw_measures(i)*Eigen::Matrix<double,CalibrationOutputs,CalibrationOutputs>::Identity();
    }
    //FIXME find a more compact (and efficient) expression
    regr.block<CalibrationOutputs,1>(0,CalibrationOutputs*CalibrationInputs) = -grav;
    return regr;
}

Eigen::Matrix<double,3,1> getAddedMassesForce(double mass,
                                              const Eigen::Vector3d & grav)
{
    Eigen::Matrix<double,3,1> ret;

    ret.segment<3>(0) = mass*grav;

    return ret;
}

bool ForceCalibrationMatrixEstimator::computeForceCalibrationMatrixEstimation(std::string & error_msg)
{
    Eigen::Matrix<double,EstimatedParameters,EstimatedParameters> ThetaTransposeTheta;
    Eigen::Matrix<double,EstimatedParameters,1> x;
    Eigen::Matrix<double,EstimatedParameters,1> ThetaTransposeBeta;

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
                                    dummy);

        p_dataset->computeOffsetEstimation();

        for(int smpl=0; smpl < p_dataset->getNrOfSamples(); smpl++ )
        {
            Eigen::Matrix<double,CalibrationInputs,1> raw;
            Eigen::Vector3d grav;

            p_dataset->getMeasurementsWithoutFTOffset(smpl,wrapVec(raw),wrapVec(grav));

            Eigen::Matrix<double,CalibrationOutputs,EstimatedParameters> Theta = calibration_matrix_regressor(raw,grav);
            Eigen::Matrix<double,CalibrationOutputs,1> Beta = getAddedMassesForce(added_mass,grav);

            ThetaTransposeTheta += Theta.transpose()*Theta;


            ThetaTransposeBeta  += Theta.transpose()*Beta;
        }
    }

    Eigen::Matrix<double,EstimatedParameters,EstimatedParameters> reg_term =
           Eigen::Matrix<double,EstimatedParameters,EstimatedParameters>::Identity();
    Eigen::JacobiSVD<Eigen::Matrix<double,EstimatedParameters,EstimatedParameters>, Eigen::HouseholderQRPreconditioner>
          svd_TTT(ThetaTransposeTheta, Eigen::ComputeFullU | Eigen::ComputeFullV);

    x = svd_TTT.solve(ThetaTransposeBeta);

    //std::cout << "[INFO] svd problems singular value " << svd_TTT.singularValues() << std::endl;
    //std::cout << "[INFO] TTT " << std::endl << ThetaTransposeTheta << std::endl;
    //std::cout << "[INFO] estimated mass " << x.segment<1>(CalibrationInputs*CalibrationOutputs) << std::endl;

    for(int i=0; i < CalibrationOutputs; i++ )
    {
        for(int j=0; j < CalibrationInputs; j++ )
        {
            this->pimpl->estimated_calibration_matrix(i,j) = x(j*CalibrationOutputs+i);
        }
    }

    return true;
}

bool ForceCalibrationMatrixEstimator::getEstimatedForceCalibrationMatrix(const MatWrapper _estimated_calibration_matrix)
{
    if( _estimated_calibration_matrix.rows != CalibrationOutputs ||
        _estimated_calibration_matrix.cols != CalibrationInputs ||
        _estimated_calibration_matrix.storage_order != COLUMN_MAJOR )
    {
        std::cerr << "[ERR] mismatch in _estimated_calibration_matrix parameter of getEstimatedForceCalibrationMatrix method" << std::endl;
        return false;
    }

    toEigen(_estimated_calibration_matrix) = this->pimpl->estimated_calibration_matrix;
    return true;
}



}
