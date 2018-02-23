
// Yarp-related blocks
#include "YarpClock.h"
#include "YarpRead.h"
#include "YarpWrite.h"

// Generic blocks
#include "CentroidalMomentum.h"
#include "DotJNu.h"
#include "ForwardKinematics.h"
#include "GetLimits.h"
#include "GetMeasurement.h"
#include "InverseDynamics.h"
#include "Jacobian.h"
#include "MassMatrix.h"
#include "ModelPartitioner.h"
#include "RealTimeSynchronizer.h"
#include "SetLowLevelPID.h"
#include "SetReferences.h"
#include "SimulatorSynchronizer.h"

#ifdef WBT_USES_ICUB
// iCub-related blocks
#include "DiscreteFilter.h"
#include "MinimumJerkTrajectoryGenerator.h"
#endif

#ifdef WBT_USES_IPOPT
// #include "InverseKinematics.h"
#endif
// #include "RemoteInverseKinematics.h"
