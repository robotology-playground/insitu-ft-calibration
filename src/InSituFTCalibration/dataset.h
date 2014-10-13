#ifndef INSITU_FT_ACC_DATASET_H
#define INSITU_FT_ACC_DATASET_H


#include "insitu-ft-calibration-export.h"

#include "wrappers.h"


namespace InSituFTCalibration {

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
    struct ForceTorqueAccelerometerDatasetPrivateAttributes; 
    ForceTorqueAccelerometerDatasetPrivateAttributes * pimpl;
    
public:
    /**
     * Constructor
     */
    ForceTorqueAccelerometerDataset();

    /**
     * Destructory
     */
    virtual ~ForceTorqueAccelerometerDataset();
    
    ForceTorqueAccelerometerDataset(const ForceTorqueAccelerometerDataset& other);

    ForceTorqueAccelerometerDataset& operator=(const ForceTorqueAccelerometerDataset &other);

    /**
     * Clean the class, deleting all added samples.
     */
    virtual void reset();
    
    /**
     * Add a sample of FT/accelerometer measurements. 
     * 
     * @param ft_measure vector of size 6 of ft_measurements
     * @param acc_measure vector of size 3 of acc_measure
     */
    virtual bool addMeasurements(const VecWrapper ft_measure,
                                 const VecWrapper acc_measure);
    
    /**
     * Get a sample of FT/accelerometer measurements.
     * 
     * @param ft_measure vector of size 6 of ft_measurements
     * @param acc_measure vector of size 3 of acc_measure
     * @return true if the sample was available, false otherwise
     */
    virtual bool getMeasurements(const int sample,
                                 const VecWrapper ft_measure,
                                 const VecWrapper acc_measure) const;
    
    
    /**
     * Get total number of samples added to this object
     */
    virtual int getNrOfSamples() const;
    
    
};

}

#endif // INSITU_FT_ACC_DATASET_H
