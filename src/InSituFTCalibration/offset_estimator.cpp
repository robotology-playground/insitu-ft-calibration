#include "offset_estimator.h"

#include <Eigen/Dense>

#include "eigen_wrappers.h"


namespace InSituFTCalibration {
    
struct ForceTorqueOffsetEstimator::ForceTorqueOffsetEstimatorPrivateAttributes
{   
    Eigen::Matrix<double,6,1>  offset; //< Offset value estimated by the algorithm
};


ForceTorqueOffsetEstimator::ForceTorqueOffsetEstimator(): 
    pimpl(new ForceTorqueOffsetEstimatorPrivateAttributes)
{
}

ForceTorqueOffsetEstimator::ForceTorqueOffsetEstimator(const ForceTorqueOffsetEstimator& other)
    : pimpl(new ForceTorqueOffsetEstimatorPrivateAttributes(*other.pimpl))
{
}

/*
ForceTorqueOffsetEstimator::ForceTorqueOffsetEstimator(ForceTorqueOffsetEstimator&& other) 
    : pimpl(0)
{
    std::swap(pimpl, other.pimpl);
}*/
 
ForceTorqueOffsetEstimator& ForceTorqueOffsetEstimator::operator=(const ForceTorqueOffsetEstimator &other) {
    if(this != &other) {
        *pimpl = *(other.pimpl);
    }
    return *this;
}
 

ForceTorqueOffsetEstimator::~ForceTorqueOffsetEstimator()
{
    delete pimpl;
    pimpl = 0;
}

Eigen::Matrix<double,9,1> vec_operator(const Eigen::Matrix3d & mat)
{
    Eigen::Matrix<double,9,1> ret;
    ret.segment<3>(0) = mat.block<3,1>(0,0);
    ret.segment<3>(3) = mat.block<3,1>(0,1);
    ret.segment<3>(6) = mat.block<3,1>(0,2);
    return ret;
}

Eigen::Matrix<double,3,12>  offset_regressor(const Eigen::Vector3d & g)
{
    Eigen::Matrix<double,3,12> regr;
    regr.setZero();
    regr.block<3,3>(0,0) = g(0)*Eigen::Matrix3d::Identity();
    regr.block<3,3>(0,3) = g(1)*Eigen::Matrix3d::Identity();
    regr.block<3,3>(0,6) = g(2)*Eigen::Matrix3d::Identity();
    regr.block<3,3>(0,9) = -Eigen::Matrix3d::Identity();
    return regr;
}

bool ForceTorqueOffsetEstimator::computeOffsetEstimation()
{
    if( this->getNrOfSamples() == 0 )
    {
        return false;
    }
    
    //Compute the mean of the ft measurements (\f[ r_m \f])
    Eigen::Matrix<double,6,1> r_m;
    r_m.setZero();
    
    for(int i=0; i < this->getNrOfSamples(); i++)
    {
        Eigen::Matrix<double,6,1> ft_sample;
        Eigen::Vector3d acc_sample;
        
        this->getMeasurements(i,wrapVec(ft_sample),wrapVec(acc_sample)); 
             
        Eigen::Matrix<double,6,1> delta = ft_sample - r_m;
        r_m += delta/(i+1);

    }
        
    //Compute the 3D subspace in which the measurement lie
    //\todo this implementation is probably numerically unstable
    //      substitute it with welford algorithm
    Eigen::Matrix<double,6,6> RTR;
    RTR.setZero();
    
    for(int i=0; i < this->getNrOfSamples(); i++)
    {
        Eigen::Matrix<double,6,1> ft_sample;
        Eigen::Vector3d acc_sample;
        
        this->getMeasurements(i,wrapVec(ft_sample),wrapVec(acc_sample));
        
        Eigen::Matrix<double,6,1> ft_sample_without_mean = ft_sample-r_m;
        RTR += (ft_sample_without_mean)*(ft_sample_without_mean).transpose();   
    }
    
    //Compute the SVD (or eigenvalue decomposition) of RTR
    //For getting the estimate of the subspace
    Eigen::JacobiSVD<Eigen::Matrix<double,6,6>, Eigen::HouseholderQRPreconditioner> 
          svd(RTR, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    svd.computeU();
    svd.computeV();
        
    Eigen::Matrix<double,6,3> U1 = svd.matrixU().block<6,3>(0,0);
    
    
    //Solve offset least square problem
    Eigen::Matrix<double,12,12> GammaTransposeGamma = Eigen::Matrix<double,12,12>::Zero();
    Eigen::Matrix<double,12,1> GammaTranspose_r = Eigen::Matrix<double,12,1>::Zero();
    
    for(int i=0; i < this->getNrOfSamples(); i++)
    {
        Eigen::Matrix<double,6,1> ft_sample;
        Eigen::Vector3d acc_sample;
        this->getMeasurements(i,wrapVec(ft_sample),wrapVec(acc_sample));

        Eigen::Matrix<double,3,12> regr_matrix = offset_regressor(acc_sample);
        
        Eigen::Vector3d ft_sample_without_mean_projected = U1.transpose()*(ft_sample-r_m);

        GammaTransposeGamma += regr_matrix.transpose()*regr_matrix;
        GammaTranspose_r += regr_matrix.transpose()*ft_sample_without_mean_projected;
    }
    
     Eigen::JacobiSVD<Eigen::Matrix<double,12,12>, Eigen::HouseholderQRPreconditioner> 
          svd_GTG(GammaTransposeGamma, Eigen::ComputeFullU | Eigen::ComputeFullV);
     
     Eigen::Matrix<double,12,1> x = svd_GTG.solve(GammaTranspose_r);
     
     
     //Get o_first
     Eigen::Vector3d o_first = x.segment<3>(9);
          
     //Get final offset
     //FIXME the paper has a plus in this formula, probably we need to fix it in the paper
     this->pimpl->offset = r_m - U1*o_first;
   
     return true;
}

bool ForceTorqueOffsetEstimator::getEstimatedOffset(const VecWrapper offset) const
{
    if( offset.size != 6 )
    {
        return false;
    }
    
    toEigen(offset) = this->pimpl->offset;
    
    return true;
}

}