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
class INSITU_FT_CALIBRATION_EXPORT VecWrapper
{
public:
    double * data;
    size_t size;
    VecWrapper() {};
    VecWrapper(double * _data,
               size_t   _size):
               data(_data),
               size(_size) {};
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
class INSITU_FT_CALIBRATION_EXPORT MatWrapper
{
public:
    double * data;
    size_t rows;
    size_t cols;
    StorageOrder storage_order;
    MatWrapper() {};
    MatWrapper(double * _data,
               size_t   _rows,
               size_t   _cols,
               StorageOrder _storage_order):
               data(_data),
               rows(_rows),
               cols(_cols),
               storage_order(_storage_order) {};
};

}

#endif // INSITU_FT_WRAPPERS_H
