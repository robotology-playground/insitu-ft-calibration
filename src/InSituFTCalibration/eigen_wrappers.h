#ifndef INSITU_FT_EIGEN_WRAPPERS_H
#define INSITU_FT_EIGEN_WRAPPERS_H

#include "wrappers.h"

#include <Eigen/Dense>

namespace InSituFTCalibration {

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline VecWrapper wrapVec(Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols > & eigen_vec)
{
    VecWrapper ret;
    ret.data = eigen_vec.data();
    ret.size = eigen_vec.size();
    return ret;
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline MatWrapper wrapMat(Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols > & eigen_mat)
{
    MatWrapper ret;
    ret.data = eigen_mat.data();
    ret.rows = eigen_mat.rows();
    ret.cols = eigen_mat.cols();
    ret.storage_order = eigen_mat.IsRowMajor ? ROW_MAJOR : COLUMN_MAJOR;
    return ret;
}

inline Eigen::Map<Eigen::MatrixXd> toEigen(const VecWrapper vec_wrapper)
{
    return Eigen::Map<Eigen::MatrixXd>(vec_wrapper.data,vec_wrapper.size,1);
}

inline Eigen::Map< Eigen::Matrix<double,Dynamic,Dynamic,RowMajor> > toEigenRowMajor(const MatWrapper mat_wrapper)
{
    assert(mat_wrapper.storage_order == ROW_MAJOR);
    return Eigen::Map< Eigen::Matrix<double,Dynamic,Dynamic,RowMajor> >(mat_wrapper.data,mat_wrapper.rows,mat_wrapper.cols);
}

inline Eigen::Map< Eigen::MatrixXd > toEigenColMajor(const MatWrapper mat_wrapper)
{
    assert(mat_wrapper.storage_order == COL_MAJOR);
    return Eigen::Map< Eigen::MatrixXd >(mat_wrapper.data,mat_wrapper.rows,mat_wrapper.cols);
}

}


#endif // INSITU_FT_WRAPPERS_H
