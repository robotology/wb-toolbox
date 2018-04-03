#include "GetLimits.h"
#include "BlockInformation.h"
#include "Log.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>
#include <yarp/dev/IControlLimits2.h>

#include <limits>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

using namespace wbt;

const std::string GetLimits::ClassName = "GetLimits";

double GetLimits::deg2rad(const double& v)
const unsigned PARAM_IDX_BIAS = WBBlock::NumberOfParameters - 1;
const unsigned PARAM_IDX_LIMIT_SRC = PARAM_IDX_BIAS + 1;

{
    return v * M_PI / 180.0;
}

unsigned GetLimits::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool GetLimits::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!WBBlock::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    // INPUTS
    // ======
    //
    // No inputs
    //

    if (!blockInfo->setNumberOfInputPorts(0)) {
        wbtError << "Failed to configure the number of input ports.";
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) vector with the information asked (1xDoFs)
    //

    if (!blockInfo->setNumberOfOutputPorts(2)) {
        wbtError << "Failed to configure the number of output ports.";
        return false;
    }

    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    bool success = true;
    success = success && blockInfo->setOutputPortVectorSize(0, dofs); // Min limit
    success = success && blockInfo->setOutputPortVectorSize(1, dofs); // Max limit

    blockInfo->setOutputPortType(0, DataType::DOUBLE);
    blockInfo->setOutputPortType(1, DataType::DOUBLE);

    if (!success) {
        wbtError << "Failed to configure output ports.";
        return false;
    }

    return true;
}

bool GetLimits::initialize(BlockInformation* blockInfo)
{
    using namespace yarp::os;

    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // Read the control type
    std::string limitType;
    if (!blockInfo->getStringParameterAtIndex(WBBlock::numberOfParameters() + 1, limitType)) {
        wbtError << "Failed to get parameters after their parsing.";
        return false;
    }

    // Initialize the structure that stores the limits
    const unsigned dofs = getConfiguration().getNumberOfDoFs();
    m_limits.reset(new Limit(dofs));

    // Initializes some buffers
    double min = 0;
    double max = 0;

    // From the RemoteControlBoardRemapper
    // ===================================
    //
    // In the next methods, the values are asked using joint index and not string.
    // The ControlBoardRemapper internally uses the same joints ordering of its
    // initialization. In this case, it matches 1:1 the joint vector. It is hence
    // possible using i to point to the correct joint.

    // Get the RemoteControlBoardRemapper and IControlLimits2 interface if needed
    yarp::dev::IControlLimits2* iControlLimits2 = nullptr;
    if (limitType == "ControlBoardPosition" || limitType == "ControlBoardVelocity") {
        // Retain the control board remapper
        if (!getRobotInterface()->retainRemoteControlBoardRemapper()) {
            wbtError << "Couldn't retain the RemoteControlBoardRemapper.";
            return false;
        }
        // Get the interface
        if (!getRobotInterface()->getInterface(iControlLimits2) || !iControlLimits2) {
            wbtError << "Failed to get IControlLimits2 interface.";
            return false;
        }
    }

    if (limitType == "ControlBoardPosition") {
        for (auto i = 0; i < dofs; ++i) {
            if (!iControlLimits2->getLimits(i, &min, &max)) {
                wbtError << "Failed to get limits from the interface.";
                return false;
            }
            m_limits->min[i] = deg2rad(min);
            m_limits->max[i] = deg2rad(max);
        }
    }
    else if (limitType == "ControlBoardVelocity") {
        for (auto i = 0; i < dofs; ++i) {
            if (!iControlLimits2->getVelLimits(i, &min, &max)) {
                wbtError << "Failed to get limits from the interface.";
                return false;
            }
            m_limits->min[i] = deg2rad(min);
            m_limits->max[i] = deg2rad(max);
        }
    }

    // From the URDF model
    // ===================
    //
    // For the time being, only position limits are supported.

    else if (limitType == "ModelPosition") {
        iDynTree::IJointConstPtr p_joint;

        // Get the KinDynComputations pointer
        const auto& kindyncomp = getRobotInterface()->getKinDynComputations();
        if (!kindyncomp) {
            wbtError << "Failed to retrieve the KinDynComputations object.";
            return false;
        }

        // Get the model
        const iDynTree::Model model = kindyncomp->model();

        for (auto i = 0; i < dofs; ++i) {
            // Get the joint name
            std::string joint = getConfiguration().getControlledJoints()[i];

            // Get its index
            iDynTree::JointIndex jointIndex = model.getJointIndex(joint);

            if (jointIndex == iDynTree::JOINT_INVALID_INDEX) {
                wbtError << "Invalid iDynTree joint index.";
                return false;
            }

            // Get the joint from the model
            p_joint = model.getJoint(jointIndex);

            if (!p_joint->hasPosLimits()) {
                m_limits->min[i] = -std::numeric_limits<double>::infinity();
                m_limits->max[i] = std::numeric_limits<double>::infinity();
                wbtWarning << "Joint " << joint << " has no model limits.";
            }
            else {
                if (!p_joint->getPosLimits(0, min, max)) {
                    wbtError << "Failed to get joint limits from the URDF model "
                             << "for the joint " << joint + ".";
                    return false;
                }
                m_limits->min[i] = min;
                m_limits->max[i] = max;
            }
        }
    }
    // TODO
    // else if (limitType == "ModelVelocity") {
    // }
    // else if (limitType == "ModelEffort") {
    // }
    else {
        wbtError << "Limit type " + m_limitType + " not recognized.";
        return false;
    }

    return true;
}

bool GetLimits::terminate(const BlockInformation* blockInfo)
{
    bool ok = true;

    // Read the control type
    std::string limitType;
    ok = ok && blockInfo->getStringParameterAtIndex(WBBlock::numberOfParameters() + 1, limitType);
    if (!ok) {
        Log::getSingleton().error("Could not read estimate type parameter.");
        // Don't return false here. WBBlock::terminate must be called in any case
    }

    // Release the RemoteControlBoardRemapper
    if (limitType == "ControlBoardPosition" || limitType == "ControlBoardVelocity") {
        ok = ok && getRobotInterface()->releaseRemoteControlBoardRemapper();
        if (!ok) {
            wbtError << "Failed to release the RemoteControlBoardRemapper.";
            // Don't return false here. WBBlock::terminate must be called in any case
        }
    }

    return ok && WBBlock::terminate(blockInfo);
}

bool GetLimits::output(const BlockInformation* blockInfo)
{
    if (!m_limits) {
        return false;
    }

    Signal minPort = blockInfo->getOutputPortSignal(0);
    Signal maxPort = blockInfo->getOutputPortSignal(1);

    minPort.setBuffer(m_limits->min.data(), getConfiguration().getNumberOfDoFs());
    maxPort.setBuffer(m_limits->max.data(), getConfiguration().getNumberOfDoFs());

    return true;
}
