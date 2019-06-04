/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

// YARP-dependent blocks
#include "WBToolbox/Block/GetLimits.h"
#include "WBToolbox/Block/GetMeasurement.h"
#include "WBToolbox/Block/RealTimeSynchronizer.h"
#include "WBToolbox/Block/SetMotorParameters.h"
#include "WBToolbox/Block/SetReferences.h"
#include "WBToolbox/Block/SimulatorSynchronizer.h"
#include "WBToolbox/Block/YarpClock.h"
#include "WBToolbox/Block/YarpRpc.h"
#include "WBToolbox/Block/YarpRead.h"
#include "WBToolbox/Block/YarpWrite.h"
// #include "Block/RemoteInverseKinematics.h"

// iDyntree-dependent blocks
#include "WBToolbox/Block/CentroidalMomentum.h"
#include "WBToolbox/Block/DotJNu.h"
#include "WBToolbox/Block/ForwardKinematics.h"
#include "WBToolbox/Block/InverseDynamics.h"
#include "WBToolbox/Block/Jacobian.h"
#include "WBToolbox/Block/MassMatrix.h"
#include "WBToolbox/Block/RelativeTransform.h"

// iCub-dependent blocks
#ifdef WBT_USES_ICUB
#include "WBToolbox/Block/DiscreteFilter.h"
#include "WBToolbox/Block/MinimumJerkTrajectoryGenerator.h"
#endif

// Other blocks
#ifdef WBT_USES_QPOASES
#include "WBToolbox/Block/QpOases.h"
#endif
#ifdef WBT_USES_IPOPT
// #include "Block/InverseKinematics.h"
#endif

#include <string>

// Class factory API
#include "shlibpp/SharedLibraryClassApi.h"

// YARP-dependent blocks
SHLIBPP_DEFINE_SHARED_SUBCLASS(GetLimits, wbt::block::GetLimits, blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(GetMeasurement,
                               wbt::block::GetMeasurement,
                               blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(RealTimeSynchronizer,
                               wbt::block::RealTimeSynchronizer,
                               blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(SetMotorParameters,
                               wbt::block::SetMotorParameters,
                               blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(SetReferences, wbt::block::SetReferences, blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(SimulatorSynchronizer,
                               wbt::block::SimulatorSynchronizer,
                               blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(YarpClock, wbt::block::YarpClock, blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(YarpRpc, wbt::block::YarpRpc, blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(YarpRead, wbt::block::YarpRead, blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(YarpWrite, wbt::block::YarpWrite, blockfactory::core::Block)

// iDyntree-dependent blocks
SHLIBPP_DEFINE_SHARED_SUBCLASS(CentroidalMomentum,
                               wbt::block::CentroidalMomentum,
                               blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(DotJNu, wbt::block::DotJNu, blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(ForwardKinematics,
                               wbt::block::ForwardKinematics,
                               blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(InverseDynamics,
                               wbt::block::InverseDynamics,
                               blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(Jacobian, wbt::block::Jacobian, blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(MassMatrix, wbt::block::MassMatrix, blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(RelativeTransform,
                               wbt::block::RelativeTransform,
                               blockfactory::core::Block)

// iCub-dependent blocks
#ifdef WBT_USES_ICUB
SHLIBPP_DEFINE_SHARED_SUBCLASS(DiscreteFilter,
                               wbt::block::DiscreteFilter,
                               blockfactory::core::Block)
SHLIBPP_DEFINE_SHARED_SUBCLASS(MinimumJerkTrajectoryGenerator,
                               wbt::block::MinimumJerkTrajectoryGenerator,
                               blockfactory::core::Block)
#endif

// Other blocks
#ifdef WBT_USES_QPOASES
SHLIBPP_DEFINE_SHARED_SUBCLASS(QpOases, wbt::block::QpOases, blockfactory::core::Block)
#endif
#ifdef WBT_USES_IPOPT
SHLIBPP_DEFINE_SHARED_SUBCLASS(InverseKinematics,
                               wbt::block::InverseKinematics,
                               blockfactory::core::Block)
#endif
