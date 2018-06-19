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

using namespace wbt;

Block* instantiateBlockWithClassName(const std::string& blockClassName)
{
    Block* block = nullptr;

    if (blockClassName == YarpRead::ClassName) {
        block = new YarpRead();
    }
    else if (blockClassName == YarpWrite::ClassName) {
        block = new YarpWrite();
    }
    else if (blockClassName == ModelPartitioner::ClassName) {
        block = new ModelPartitioner();
    }
    else if (blockClassName == YarpClock::ClassName) {
        block = new YarpClock();
    }
    else if (blockClassName == MassMatrix::ClassName) {
        block = new MassMatrix();
    }
    else if (blockClassName == ForwardKinematics::ClassName) {
        block = new ForwardKinematics();
    }
    else if (blockClassName == RelativeTransform::ClassName) {
        block = new RelativeTransform();
    }
    else if (blockClassName == SetReferences::ClassName) {
        block = new SetReferences();
    }
    else if (blockClassName == RealTimeSynchronizer::ClassName) {
        block = new RealTimeSynchronizer();
    }
    else if (blockClassName == SimulatorSynchronizer::ClassName) {
        block = new SimulatorSynchronizer();
    }
    else if (blockClassName == Jacobian::ClassName) {
        block = new Jacobian();
    }
    else if (blockClassName == GetMeasurement::ClassName) {
        block = new GetMeasurement();
    }
    else if (blockClassName == InverseDynamics::ClassName) {
        block = new InverseDynamics();
    }
    else if (blockClassName == DotJNu::ClassName) {
        block = new DotJNu();
    }
    else if (blockClassName == GetLimits::ClassName) {
        block = new GetLimits();
    }
    else if (blockClassName == CentroidalMomentum::ClassName) {
        block = new CentroidalMomentum();
    }
    else if (blockClassName == SetLowLevelPID::ClassName) {
        block = new SetLowLevelPID();
    }
#ifdef WBT_USES_ICUB
    else if (blockClassName == MinimumJerkTrajectoryGenerator::ClassName) {
        block = new MinimumJerkTrajectoryGenerator();
    }
    else if (blockClassName == DiscreteFilter::ClassName) {
        block = new wbt::DiscreteFilter();
    }
#endif
#ifdef WBT_USES_QPOASES
    else if (blockClassName == QpOases::ClassName) {
        block = new QpOases();
    }
#endif
#ifdef WBT_USES_IPOPT
    // else if (blockClassName == InverseKinematics::ClassName) {
    //     block = new InverseKinematics();
    // }
#endif
    // else if (blockClassName == RemoteInverseKinematics::ClassName) {
    //     block = new RemoteInverseKinematics();
    // }
    return block;
}
