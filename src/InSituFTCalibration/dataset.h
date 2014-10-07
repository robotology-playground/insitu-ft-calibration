#ifndef INSITU_FT_ACC_DATASET_H
#define INSITU_FT_ACC_DATASET_H

#include <vector>

#include <Eigen/Dense>


#include "insitu-ft-calibration-export.h"

namespace Eigen {
    typedef Matrix<double,6,1> Vector6d;
}

namespace InSituFTCalibration {
    

/**
 * Structure representing a joint ft 
 * and accelerometer measurement.
 */ 
struct INSITU_FT_CALIBRATION_EXPORT ForceTorqueAccelerometerMeasurement
{
    INSITU_FT_CALIBRATION_EXPORT Eigen::Vector6d ft_measure;
    INSITU_FT_CALIBRATION_EXPORT Eigen::Vector3d acc_measure;
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
    virtual bool addMeasurements(const Eigen::Vector6d & ft_measure,
                                 const Eigen::Vector3d & acc_measure);
    
    /**
     * Get a sample of FT/accelerometer measurements.
     * 
     * @return true if the sample was available, false otherwise
     */
    virtual bool getMeasurements(const int sample,
                                 Eigen::Vector6d & ft_measure,
                                 Eigen::Vector3d & acc_measure);
    
    
    /**
     * Get total number of samples added to this object
     */
    virtual int getNrOfSamples() const;
    
    
};

}

#endif // INSITU_FT_ACC_DATASET_H
