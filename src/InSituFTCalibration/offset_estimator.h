#ifndef INSITU_OFFSET_ESTIMATOR_H
#define INSITU_OFFSET_ESTIMATOR_H

#include <vector>

#include <Eigen/Dense>

#include "insitu-ft-calibration-export.h"

#include "dataset.h"

namespace InSituFTCalibration {
    

/**
 * \class InSituFTCalibration::OffsetEstimator
 * \headerfile dataset.h <InSituFTCalibration/dataset.h>
 *
 * \brief A class for estimating offset in a dataset.
 * 
 * \note 
 */
class INSITU_FT_CALIBRATION_EXPORT ForceTorqueOffsetEstimator : public ForceTorqueAccelerometerDataset
{    
private:
    Eigen::Matrix<double,6,1>  offset; //< Offset value estimated by the algorithm
public:
    /**
     * Constructor
     */
    ForceTorqueOffsetEstimator();

    /**
     * Destructor
     */
    virtual ~ForceTorqueOffsetEstimator();
    
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
     */   
    virtual Eigen::Matrix<double,6,1>  getOffset() const;
    
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