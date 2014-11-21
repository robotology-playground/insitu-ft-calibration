#ifndef INSITU_FT_YARP_WRAPPERS_H
#define INSITU_FT_YARP_WRAPPERS_H

#include "wrappers.h"

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

namespace InSituFTCalibration {

inline VecWrapper wrapVec(yarp::sig::Vector & vec)
{
    VecWrapper ret;
    ret.data = vec.data();
    ret.size = vec.size();
    return ret;
}

inline MatWrapper wrapMat(yarp::sig::Matrix & mat)
{
    MatWrapper ret;
    ret.data = mat.data();
    ret.rows = mat.rows();
    ret.cols = mat.cols();
    ret.storage_order = ROW_MAJOR;
    return ret;
}

}


#endif // INSITU_FT_WRAPPERS_H
