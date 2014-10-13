#ifndef INSITU_CALIBRATION_MATRIX_ESTIMATOR_H
#define INSITU_CALIBRATION_MATRIX_ESTIMATOR_H

#include <vector>

#include <Eigen/Dense>

#include "insitu-ft-calibration-export.h"

#include "dataset.h"
#include "offset_estimator.h"

namespace InSituFTCalibration {
    

/**
 * \class InSituFTCalibration::CalibrationMatrixEstimator
 * \headerfile dataset.h <InSituFTCalibration/calibration_matrix_estimator.h>
 *
 * \brief A class for estimating the calibration matrix of a six-axes FT sensor.
 * 
 * A class for estimating the calibration matrix of a six-axes FT sensor, using 
 *  multiple datasets (i.e. static readings of the FT sensors under different loads) . 
 * 
 * \note 
 */
class INSITU_FT_CALIBRATION_EXPORT CalibrationMatrixEstimator
{    
private:
    struct CalibrationMatrixEstimatorPrivateAttributes;
    CalibrationMatrixEstimatorPrivateAttributes * pimpl;
    
public:
 /**
     * Constructor
     */
    CalibrationMatrixEstimator();
    
    /**
     * Copy constructor
     */
    CalibrationMatrixEstimator(const CalibrationMatrixEstimator& other);

    /**
     * Copy operator
     */
    CalibrationMatrixEstimator& operator=(const CalibrationMatrixEstimator &other);

    /**
     * Destructory
     */
    virtual ~CalibrationMatrixEstimator();
    

    
    /**
     * Get total number of datasets considered
     * for the calibration matrix estimation.
     */
    virtual int getNrOfDatasets() const;
    
    virtual int getNrOfSamples() const;
     
    /**
     * Add a dataset of FT/accelerometer measurements.
     * 
     * For each dataset we assume that an additional 
     * mass is added to the system, to have a reference
     * on which to calibrate, for more information check
     * the paper.
     * 
     * \note the added_com is the vector connecting the origin 
     *       of the sensor frame with the COM of the added mass,
     *       expressed in the sensor frame with respect to which
     *       we are calibrating the FT sensor.
     * 
     * @param dataset_name a conventional name for the dataset
     * @param added_mass the known additional mass (in Kg)
     * @param added_com  the known com of the additional mass(in m)
     * @return the ID of the dataset, a integer assigned to the dataset
     *         for internal use. The dataset are assigned progressivly from 0.
     */
    virtual int addCalibrationDataset(std::string dataset_name,
                           double added_mass, 
                           const VecWrapper added_com);
    
    /**
     * Get a pointer to a given dataset
     * 
     * @return true if dataset_id is positive and lower than getNrOfDatasets(), false otherwise.
     */
    virtual bool getCalibrationDataset(const int dataset_id,
                                       ForceTorqueOffsetEstimator *& p_dataset) const;
    
    /**
     * Get a pointer to a given dataset
     * 
     * @return true if dataset_id is positive and lower than getNrOfDatasets(), false otherwise.
     */
    virtual bool getCalibrationDataset(const int dataset_id,
                                       ForceTorqueOffsetEstimator *& p_dataset,
                                       double & added_mass,
                                       const VecWrapper added_com,
                                       std::string & dataset_name) const;
                                       
        
    /**
     * Get a pointer to a given dataset
     * 
     * @return true if dataset_name is the name of a dataset, false otherwise.
     */
    virtual bool getCalibrationDataset(const std::string & dataset_name,
                                       ForceTorqueOffsetEstimator *& p_dataset) const;
    
    /**
     * Get a pointer to a given dataset
     * 
     * @return true if dataset_name is the name of a dataset, false otherwise.
     */
    virtual bool getCalibrationDataset(const std::string & dataset_name,
                                       ForceTorqueOffsetEstimator *& p_dataset,
                                       double & added_mass,
                                       const VecWrapper added_com,
                                       int & dataset_id) const;
    
    /**
     * Perform the estimation algoritmh for the given dataset,
     * estimation results can be queried using the getEstimated* methods in the next
     * section.  
     * 
     * @param error_msg if an error occurs, fill this string with a human readable error message 
     * @return true if all went well, false otherwise.
     */
    bool computeCalibrationMatrixEstimation(std::string & error_msg);
    
    bool getEstimatedCalibrationMatrix(const MatWrapper estimated_calibration_matrix);
    
    bool reset();
    
};

}

#endif // INSITU_FT_ACC_DATASET_H