#include "dataset.h"
#include <cmath>

namespace InSituFTCalibration {
    
ForceTorqueAccelerometerDataset::ForceTorqueAccelerometerDataset()
{
    samples.resize(0);
}

ForceTorqueAccelerometerDataset::~ForceTorqueAccelerometerDataset()
{
    samples.resize(0);
}

void ForceTorqueAccelerometerDataset::reset()
{
    samples.resize(0);
}


int ForceTorqueAccelerometerDataset::getNrOfSamples() const
{
    return (int)samples.size();
}  


bool ForceTorqueAccelerometerDataset::addMeasurements(const Eigen::Matrix<double,6,1> & _ft_measure,
                                                     const Eigen::Vector3d & _acc_measure)
{
    ForceTorqueAccelerometerMeasurement sample;
    sample.ft_measure = _ft_measure;
    sample.acc_measure = _acc_measure;
    
    samples.push_back(sample);
    
    return true;
}

bool ForceTorqueAccelerometerDataset::getMeasurements(const int sample,
                     Eigen::Matrix<double,6,1> & ft_measure,
                     Eigen::Vector3d & acc_measure)
{
    if( !(sample >= 0 && sample < this->getNrOfSamples()) )
    {
        ft_measure.setConstant(-1e10);
        acc_measure.setConstant(-1e10);
        return false;
    }
    
    ft_measure = this->samples[sample].ft_measure;
    acc_measure = this->samples[sample].acc_measure;   
    return true;
}
    




}