#include "lidar_mapping/lm.h"
#include "nano_gicp/nanoflann_adaptor.h"

template class nanoflann::KdTreeFLANN<PointType>;
