#include "Log.h"
//General Yarp utilities
#include "YarpRead.h"
#include "YarpWrite.h"
#include "ModelPartitioner.h"
#include "YarpClock.h"
//WBI-related stuff
#include "MassMatrix.h"
#include "ForwardKinematics.h"
#include "SetReferences.h"
#include "RealTimeSynchronizer.h"
#include "SimulatorSynchronizer.h"
#include "Jacobian.h"
#include "GetMeasurement.h"
#include "InverseDynamics.h"
#include "DotJNu.h"
#include "GetLimits.h"
#include "CentroidalMomentum.h"
#include "SetLowLevelPID.h"
#ifdef WBT_USES_ICUB
#include "MinimumJerkTrajectoryGenerator.h"
#include "DiscreteFilter.h"
#endif
#ifdef WBT_USES_IPOPT
// #include "InverseKinematics.h"
#endif
// #include "RemoteInverseKinematics.h"
