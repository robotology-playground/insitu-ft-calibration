#ifndef INSITU_FT_WRAPPERS_H
#define INSITU_FT_WRAPPERS_H

#include <cstdlib>

#include "insitu-ft-calibration-export.h"

namespace InSituFTCalibration {

/**
 * Structure representing an array of double elements.
 * It holds a pointer to the raw area of memory where the
 * first elements of the array is, and some additional information
 * to permit a basic form of error checking.
 *
 */
struct INSITU_FT_CALIBRATION_EXPORT VecWrapper
{
    double * data;
    size_t size;
};

enum StorageOrder { ROW_MAJOR,
                    COLUMN_MAJOR };

/**
 * Structure representing a 2D array (matrix)
 * of double elements.
 * It holds a pointer to the raw area of memory where the
 * first elements of the array is, and some additional information
 * to permit a basic form of error checking and interpretation of the data.
 */
struct INSITU_FT_CALIBRATION_EXPORT MatWrapper
{
    double * data;
    size_t rows;
    size_t cols;
    StorageOrder storage_order;
};

}

#endif // INSITU_FT_WRAPPERS_H
