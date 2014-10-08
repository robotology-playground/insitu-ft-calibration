#ifndef INSITU_OFFSET_ESTIMATOR_H
#define INSITU_OFFSET_ESTIMATOR_H

#include "insitu-ft-calibration-export.h"

#include "wrappers.h"

#include "dataset.h"

namespace InSituFTCalibration {
    

/**
 * \class InSituFTCalibration::OffsetEstimator
 * \headerfile dataset.h <InSituFTCalibration/offset_estimator.h>
 *
 * \brief A class for estimating offset in a dataset.
 * 
 * \note 
 */
class INSITU_FT_CALIBRATION_EXPORT ForceTorqueOffsetEstimator : public ForceTorqueAccelerometerDataset
{    
private:
    struct ForceTorqueOffsetEstimatorPrivateAttributes; 
    ForceTorqueOffsetEstimatorPrivateAttributes * pimpl;
    
public:
    /**
     * Constructor
     */
    ForceTorqueOffsetEstimator();
    
    ForceTorqueOffsetEstimator(const ForceTorqueOffsetEstimator& other);

    /**
     * Destructor
     */
    virtual ~ForceTorqueOffsetEstimator();
    
    
    virtual ForceTorqueOffsetEstimator& operator=(const ForceTorqueOffsetEstimator &other);

    
    /**
     * Get total number of samples added to this object
     */
   // virtual int getNrOfSamples() const;
    
     
    /**
     * Add a sample of FT/accelerometer measurements. 
     */
    //virtual bool addMeasurements(const Eigen::Vector6d & ft_measure,
    //                             const Eigen::Vector3d & acc_measure);
    
    /**
     * Get a sample of FT/accelerometer measurements.
     * 
     * @return true if the sample was available, false otherwise
     */
    //virtual bool getMeasurements(const int sample,
    //                             Eigen::Vector6d & ft_measure,
    //                             Eigen::Vector3d & acc_measure);
    
    /**
     * Perform the offset estimation
     */
    virtual bool computeOffsetEstimation();

    /**
     * Compute offset for the dataset, using algorithm from Traversaro2015
     * 
     * \note this function returns the value stored by the computeOffsetEstimation
     *       method. If necessary this method is called before returning the result.
     * \param VecWrapper a Vector of six elements, used to return the offset
     */   
    virtual bool  getEstimatedOffset(const VecWrapper offset) const;
    
    /**
     * As getMeasurements(), but remove the offset estimated from the ft measurement.
     */
    //virtual bool getMeasurementsWithoutFTOffset(Eigen::Matrix<double,6,1>  & ft_measure,
    //                                            Eigen::Vector3d & acc_measure) const;
    
    /**
     * 
     * 
     */
    //virtual Vector3d getSemiAxesLengthOfAssociatedEllipsoid() const;
    
    /**
     * The raw measurs should lay on a 3d subspace of the 6d space. 
     * Return the basis matrix of the estimated subspace (\f[ U_1 \f])
     *
     */
    //virtual Eigen::Matrix<double,6,3> getBasisMatrixOf3DSubspace() const;
};

}

#endif // INSITU_FT_ACC_DATASET_H