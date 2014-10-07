#ifndef INSITU_FT_ACC_DATASET_H
#define INSITU_FT_ACC_DATASET_H

#include <vector>

#define EIGEN_NO_STATIC_ASSERT
#include <Eigen/Dense>


#include "insitu-ft-calibration-export.h"

namespace InSituFTCalibration {
    
template class INSITU_FT_CALIBRATION_EXPORT Eigen::Matrix<double,6,1>;
template class INSITU_FT_CALIBRATION_EXPORT Eigen::Matrix<double,3,1>;

/**
 * Structure representing a joint ft 
 * and accelerometer measurement.
 */ 
struct INSITU_FT_CALIBRATION_EXPORT ForceTorqueAccelerometerMeasurement
{
    Eigen::Matrix<double,6,1> ft_measure;
    Eigen::Vector3d acc_measure;
};

/**
 * \class InSituFTCalibration::ForceTorqueAccelerometerDataset
 * \headerfile dataset.h <InSituFTCalibration/dataset.h>
 *
 * \brief A class for storing joint measurement of an accelerometer 
 *        and a force torque sensor.
 * 
 * \note For the ForceTorque sensor, raw measurement are stored.
 *
 */
class INSITU_FT_CALIBRATION_EXPORT ForceTorqueAccelerometerDataset
{
private:
    std::vector<ForceTorqueAccelerometerMeasurement> samples; //< storage of the time series of measurements of the dataset
    
public:
    /**
     * Constructor
     */
    ForceTorqueAccelerometerDataset();

    /**
     * Destructory
     */
    virtual ~ForceTorqueAccelerometerDataset();

    /**
     * Clean the class, deleting all added samples.
     */
    virtual void reset();
    
    /**
     * Add a sample of FT/accelerometer measurements. 
     */
    virtual bool addMeasurements(const Eigen::Matrix<double,6,1> & ft_measure,
                                 const Eigen::Vector3d & acc_measure);
    
    /**
     * Get a sample of FT/accelerometer measurements.
     * 
     * @return true if the sample was available, false otherwise
     */
    virtual bool getMeasurements(const int sample,
                                 Eigen::Matrix<double,6,1> & ft_measure,
                                 Eigen::Vector3d & acc_measure);
    
    
    /**
     * Get total number of samples added to this object
     */
    virtual int getNrOfSamples() const;
    
    
};

}

#endif // INSITU_FT_ACC_DATASET_H
