#include "dataset.h"

#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "eigen_wrappers.h"

namespace InSituFTCalibration {
    
/**
 * Structure representing a joint ft 
 * and accelerometer measurement.
 */ 
struct INSITU_FT_CALIBRATION_EXPORT ForceTorqueAccelerometerMeasurement
{
    Eigen::Matrix<double,6,1> ft_measure;
    Eigen::Vector3d acc_measure;
};
    
struct ForceTorqueAccelerometerDataset::ForceTorqueAccelerometerDatasetPrivateAttributes
{   
    std::vector<ForceTorqueAccelerometerMeasurement> samples; //< storage of the time series of measurements of the dataset
};


ForceTorqueAccelerometerDataset::ForceTorqueAccelerometerDataset(): 
    pimpl(new ForceTorqueAccelerometerDatasetPrivateAttributes)
{
}

ForceTorqueAccelerometerDataset::ForceTorqueAccelerometerDataset(const ForceTorqueAccelerometerDataset& other)
    : pimpl(new ForceTorqueAccelerometerDatasetPrivateAttributes(*(other.pimpl)))
{
}

/*
ForceTorqueAccelerometerDataset::ForceTorqueAccelerometerDataset(ForceTorqueAccelerometerDataset&& other) 
    : pimpl(0)
{
    std::swap(pimpl, other.pimpl);
}*/
 
ForceTorqueAccelerometerDataset& ForceTorqueAccelerometerDataset::operator=(const ForceTorqueAccelerometerDataset &other) {
    if(this != &other) {
        *pimpl = *(other.pimpl);
    }
    return *this;
}
 

ForceTorqueAccelerometerDataset::~ForceTorqueAccelerometerDataset()
{
    delete pimpl;
    pimpl = 0;
}

int ForceTorqueAccelerometerDataset::getNrOfSamples() const
{
    return (int)this->pimpl->samples.size();
}  


bool ForceTorqueAccelerometerDataset::addMeasurements(const VecWrapper _ft_measure,
                                                      const VecWrapper _acc_measure)
{
    ForceTorqueAccelerometerMeasurement sample;
    sample.ft_measure = toEigen(_ft_measure);
    sample.acc_measure = toEigen(_acc_measure);
    
    this->pimpl->samples.push_back(sample);
    
    return true;
}

bool ForceTorqueAccelerometerDataset::getMeasurements(const int sample,
                                                      const VecWrapper ft_measure,
                                                      const VecWrapper acc_measure)
{
    if( !(sample >= 0 && sample < this->getNrOfSamples()) )
    {
        return false;
    }
    
    toEigen(ft_measure) = this->pimpl->samples[sample].ft_measure;
    toEigen(acc_measure) = this->pimpl->samples[sample].acc_measure;   
    return true;
}
    




}