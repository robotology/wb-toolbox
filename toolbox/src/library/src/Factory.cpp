/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

// YARP-dependent blocks
#include "Block/GetLimits.h"
#include "Block/GetMeasurement.h"
#include "Block/ModelPartitioner.h"
#include "Block/RealTimeSynchronizer.h"
#include "Block/SetMotorParameters.h"
#include "Block/SetReferences.h"
#include "Block/SimulatorSynchronizer.h"
#include "Block/YarpClock.h"
#include "Block/YarpRead.h"
#include "Block/YarpWrite.h"
// #include "Block/RemoteInverseKinematics.h"

// iDyntree-dependent blocks
#include "Block/CentroidalMomentum.h"
#include "Block/DotJNu.h"
#include "Block/ForwardKinematics.h"
#include "Block/InverseDynamics.h"
#include "Block/Jacobian.h"
#include "Block/MassMatrix.h"
#include "Block/RelativeTransform.h"

// iCub-dependent blocks
#ifdef WBT_USES_ICUB
#include "Block/DiscreteFilter.h"
#include "Block/MinimumJerkTrajectoryGenerator.h"
#endif

// Other blocks
#ifdef WBT_USES_QPOASES
#include "Block/QpOases.h"
#endif
#ifdef WBT_USES_IPOPT
// #include "Block/InverseKinematics.h"
#endif

#include <string>

// Class factory API
#include "SharedLibraryClassApi.h"

// YARP-dependent blocks
SHLIBPP_DEFINE_SHARED_SUBCLASS(GetLimits, wbt::GetLimits, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(GetMeasurement, wbt::GetMeasurement, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(ModelPartitioner, wbt::ModelPartitioner, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(RealTimeSynchronizer, wbt::RealTimeSynchronizer, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(SetMotorParameters, wbt::SetMotorParameters, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(SetReferences, wbt::SetReferences, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(SimulatorSynchronizer, wbt::SimulatorSynchronizer, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(YarpClock, wbt::YarpClock, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(YarpRead, wbt::YarpRead, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(YarpWrite, wbt::YarpWrite, wbt::Block)

// iDyntree-dependent blocks
SHLIBPP_DEFINE_SHARED_SUBCLASS(CentroidalMomentum, wbt::CentroidalMomentum, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(DotJNu, wbt::DotJNu, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(ForwardKinematics, wbt::ForwardKinematics, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(InverseDynamics, wbt::InverseDynamics, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(Jacobian, wbt::Jacobian, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(MassMatrix, wbt::MassMatrix, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(RelativeTransform, wbt::RelativeTransform, wbt::Block)

// iCub-dependent blocks
#ifdef WBT_USES_ICUB
SHLIBPP_DEFINE_SHARED_SUBCLASS(DiscreteFilter, wbt::DiscreteFilter, wbt::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(MinimumJerkTrajectoryGenerator,
                               wbt::MinimumJerkTrajectoryGenerator,
                               wbt::Block)
#endif

// Other blocks
#ifdef WBT_USES_QPOASES
SHLIBPP_DEFINE_SHARED_SUBCLASS(QpOases, wbt::QpOases, wbt::Block)
#endif
#ifdef WBT_USES_IPOPT
SHLIBPP_DEFINE_SHARED_SUBCLASS(InverseKinematics, wbt::InverseKinematics, wbt::Block)
#endif
