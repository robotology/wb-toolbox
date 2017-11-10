#include "GetLimits.h"

#include "Log.h"
#include "BlockInformation.h"
#include "Signal.h"
#include "RobotInterface.h"
#include "iDynTree/KinDynComputations.h"
#include "iDynTree/Model/Model.h"
#include <yarp/dev/IControlLimits2.h>
#include <vector>
#include <limits>

using namespace wbt;

const std::string GetLimits::ClassName = "GetLimits";

unsigned GetLimits::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool GetLimits::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!WBBlock::configureSizeAndPorts(blockInfo)) return false;

    // INPUTS
    // ======
    //
    // No inputs
    //

    if (!blockInfo->setNumberOfInputPorts(0)) {
        Log::getSingleton().error("Failed to configure the number of input ports.");
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) vector with the information asked (1xDoFs)
    //

    if (!blockInfo->setNumberOfOutputPorts(2)) {
        Log::getSingleton().error("Failed to configure the number of output ports.");
        return false;
    }

    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    bool success = true;
    success = success && blockInfo->setOutputPortVectorSize(0, dofs); // Min limit
    success = success && blockInfo->setOutputPortVectorSize(1, dofs); // Max limit

    blockInfo->setOutputPortType(0, PortDataTypeDouble);
    blockInfo->setOutputPortType(1, PortDataTypeDouble);

    if (!success) {
        Log::getSingleton().error("Failed to configure output ports.");
        return false;
    }

    return true;
}

bool GetLimits::initialize(const BlockInformation* blockInfo)
{
    using namespace yarp::os;
    if (!WBBlock::initialize(blockInfo)) return false;

    // Read the control type
    std::string limitType;
    if (!blockInfo->getStringParameterAtIndex(WBBlock::numberOfParameters() + 1, limitType)) {
        Log::getSingleton().error("Could not read estimate type parameter.");
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
    std::weak_ptr<yarp::dev::IControlLimits2> iControlLimits2;
    if (limitType == "ControlBoardPosition" || limitType == "ControlBoardVelocity") {
        // Retain the control board remapper
        if (!getRobotInterface()->retainRemoteControlBoardRemapper()) {
            Log::getSingleton().error("Failed to initialize the Robot Interface containing the Control Board Remapper.");
            return false;
        }
        // Get the interface
        if (!getRobotInterface()->getInterface(iControlLimits2)) {
            Log::getSingleton().error("Failed to get IControlLimits2 interface.");
            return false;
        }
    }

    if (limitType == "ControlBoardPosition") {
        for (auto i = 0; i < dofs; ++i) {
            if (!iControlLimits2.lock()->getLimits(i, &min, &max)) {
                Log::getSingleton().error("Failed to get limits from the interface.");
                return false;
            }
            m_limits->min[i] = min;
            m_limits->max[i] = max;
        }
    }
    else if (limitType == "ControlBoardVelocity") {
        for (auto i = 0; i < dofs; ++i) {
            if (!iControlLimits2.lock()->getVelLimits(i, &min, &max)) {
                Log::getSingleton().error("Failed to get limits from the interface.");
                return false;
            }
            m_limits->min[i] = min;
            m_limits->max[i] = max;
        }
    }

    // From the URDF model
    // ===================
    //
    // For the time being, only position limits are supported.

    else if (limitType == "ModelPosition") {
        iDynTree::IJointConstPtr p_joint;
        const iDynTree::Model model = getRobotInterface()->getKinDynComputations()->model();

        for (auto i = 0; i < dofs; ++i) {
            // Get the joint name
            std::string joint = getConfiguration().getControlledJoints()[i];

            // Get its index
            iDynTree::LinkIndex jointIndex = model.getLinkIndex(joint);

            // Get the joint from the model
            p_joint = model.getJoint(jointIndex);

            if (!p_joint->hasPosLimits()) {
                Log::getSingleton().warning("Joint " + joint + " has no model limits.");
                // TODO: test how these values are interpreted by Simulink
                min = -std::numeric_limits<double>::infinity();
                max = std::numeric_limits<double>::infinity();
            }
            else {
                p_joint->getPosLimits(0, min, max);
            }
        }
    }
    // TODO
    // else if (limitType == "ModelVelocity") {
    // }
    // else if (limitType == "ModelEffort") {
    // }
    else {
        Log::getSingleton().error("Limit type " + limitType + " not recognized.");
        return false;
    }

    return true;
}

bool GetLimits::terminate(const BlockInformation* blockInfo)
{
    // Release the RemoteControlBoardRemapper
    bool ok = true;
    ok = ok & getRobotInterface()->releaseRemoteControlBoardRemapper();
    if (!ok) {
        Log::getSingleton().error("Failed to release the RemoteControlBoardRemapper.");
    }

    return ok && WBBlock::terminate(blockInfo);
}

bool GetLimits::output(const BlockInformation* blockInfo)
{
    if (!m_limits) return false;

    Signal minPort = blockInfo->getOutputPortSignal(0);
    Signal maxPort = blockInfo->getOutputPortSignal(1);

    minPort.setBuffer(m_limits->min.data(), getConfiguration().getNumberOfDoFs());
    maxPort.setBuffer(m_limits->max.data(), getConfiguration().getNumberOfDoFs());

    return true;
}
