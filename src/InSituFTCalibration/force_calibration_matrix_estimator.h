#ifndef INSITU_FORCE_CALIBRATION_MATRIX_ESTIMATOR_H
#define INSITU_FORCE_CALIBRATION_MATRIX_ESTIMATOR_H

#include <vector>

#include <Eigen/Dense>

#include "insitu-ft-calibration-export.h"

#include "dataset.h"
#include "offset_estimator.h"

namespace InSituFTCalibration {


/**
 * \class InSituFTCalibration::ForceCalibrationMatrixEstimator
 * \headerfile dataset.h <InSituFTCalibration/force_calibration_matrix_estimator.h>
 *
 * \brief A class for estimating the force calibration matrix of a six-axes FT sensor.
 *
 * A class for estimating the force calibration matrix of a six-axes FT sensor, using
 *  multiple datasets (i.e. static readings of the FT sensors under different loads).
 *
 * \note for now we assume that the six-axes FT sensor is equipped with 6 raw strain gauges sensors
 *
 * \note this is a reduced version of the CalibrationMatrixEstimator, that estimates only
 *       the rows of the calibration matrix related to the force measurements.
 */
class INSITU_FT_CALIBRATION_EXPORT ForceCalibrationMatrixEstimator
{
private:
    struct ForceCalibrationMatrixEstimatorPrivateAttributes;
    ForceCalibrationMatrixEstimatorPrivateAttributes * pimpl;

public:
    /**
     * Constructor
     */
    ForceCalibrationMatrixEstimator();

    /**
     * Copy constructor
     */
    ForceCalibrationMatrixEstimator(const ForceCalibrationMatrixEstimator& other);

    /**
     * Copy operator
     */
    ForceCalibrationMatrixEstimator& operator=(const ForceCalibrationMatrixEstimator &other);

    /**
     * Destructory
     */
    virtual ~ForceCalibrationMatrixEstimator();



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
     * the algorithm explained in the paper.
     *
     * @param dataset_name a conventional name for the dataset
     * @param added_mass the known additional mass (in Kg)
     * @return the ID of the dataset, a integer assigned to the dataset
     *         for internal use. The dataset are assigned progressivly from 0.
     */
    virtual int addCalibrationDataset(std::string dataset_name,
                                      double added_mass);

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
                                       int & dataset_id) const;

    /**
     * Perform the estimation algoritmh for the given dataset,
     * estimation results can be queried using the getEstimated* methods in the next
     * section.
     *
     * @param error_msg if an error occurs, fill this string with a human readable error message
     * @return true if all went well, false otherwise.
     */
    bool computeForceCalibrationMatrixEstimation(std::string & error_msg);

    /**
     * Get the estimated 3 times 6 force calibration matrix
     *
     */
    bool getEstimatedForceCalibrationMatrix(const MatWrapper estimated_calibration_matrix);

    bool reset();

};

}

#endif // INSITU_FT_ACC_DATASET_H