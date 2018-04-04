// YARP-dependent blocks
#include "GetLimits.h"
#include "GetMeasurement.h"
#include "ModelPartitioner.h"
#include "RealTimeSynchronizer.h"
#include "SetLowLevelPID.h"
#include "SetReferences.h"
#include "SimulatorSynchronizer.h"
#include "YarpClock.h"
#include "YarpRead.h"
#include "YarpWrite.h"
// #include "RemoteInverseKinematics.h"

// iDyntree-dependent blocks
#include "CentroidalMomentum.h"
#include "DotJNu.h"
#include "ForwardKinematics.h"
#include "InverseDynamics.h"
#include "Jacobian.h"
#include "MassMatrix.h"

// iCub-depent blocks
#ifdef WBT_USES_ICUB
#include "DiscreteFilter.h"
#include "MinimumJerkTrajectoryGenerator.h"
#endif
#ifdef WBT_USES_IPOPT
// #include "InverseKinematics.h"
#endif
