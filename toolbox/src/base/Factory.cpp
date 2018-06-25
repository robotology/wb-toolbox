/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

// YARP-dependent blocks
#include "GetLimits.h"
#include "GetMeasurement.h"
#include "ModelPartitioner.h"
#include "RealTimeSynchronizer.h"
#include "SetMotorParameters.h"
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
#include "RelativeTransform.h"

// iCub-dependent blocks
#ifdef WBT_USES_ICUB
#include "DiscreteFilter.h"
#include "MinimumJerkTrajectoryGenerator.h"
#endif

// Other blocks
#ifdef WBT_USES_QPOASES
#include "QpOases.h"
#endif
#ifdef WBT_USES_IPOPT
// #include "InverseKinematics.h"
#endif

#include <string>

using namespace wbt;

Block* Block::instantiateBlockWithClassName(const std::string& blockClassName)
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
    else if (blockClassName == SetMotorParameters::ClassName) {
        block = new SetMotorParameters();
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
