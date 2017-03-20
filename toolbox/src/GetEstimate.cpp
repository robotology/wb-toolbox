#include "GetEstimate.h"

#include "Error.h"
#include "WBInterface.h"
#include "BlockInformation.h"
#include "Signal.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>

namespace wbt {

    std::string GetEstimate::ClassName = "GetEstimate";

    GetEstimate::GetEstimate()
    : m_estimate(0)
    , m_estimateType(wbi::ESTIMATE_JOINT_POS) {}

    unsigned GetEstimate::numberOfParameters()
    {
        return WBIBlock::numberOfParameters() + 1;
    }

    bool GetEstimate::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (!WBIBlock::configureSizeAndPorts(blockInfo, error)) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        if (!blockInfo->setNumberOfInputPorts(0)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }

        // Output port:
        // - DoFs vector with the information asked
        if (!blockInfo->setNumberOfOuputPorts(1)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        bool success = blockInfo->setOutputPortVectorSize(0, dofs);
        blockInfo->setOutputPortType(0, PortDataTypeDouble);
        if (!success) {
            if (error) error->message = "Failed to configure output ports";
            return false;
        }


        return true;
    }

    bool GetEstimate::initialize(BlockInformation *blockInfo, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIBlock::initialize(blockInfo, error)) return false;

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        m_estimate = new double[dofs];

        // Reading the control type
        std::string informationType;
        if (!blockInfo->getStringParameterAtIndex(WBIBlock::numberOfParameters() + 1, informationType)) {
            if (error) error->message = "Could not read estimate type parameter";
            return false;
        }

        if (informationType == "Joints Position") {
            m_estimateType = wbi::ESTIMATE_JOINT_POS;
        } else if (informationType == "Joints Velocity") {
            m_estimateType = wbi::ESTIMATE_JOINT_VEL;
        } else if (informationType == "Joints Acceleration") {
            m_estimateType = wbi::ESTIMATE_JOINT_ACC;
        } else if (informationType == "Joints Torque") {
            m_estimateType = wbi::ESTIMATE_JOINT_TORQUE;
        } else {
            if (error) error->message = "Estimate not supported";
            return false;
        }
        return m_estimate;
    }

    bool GetEstimate::terminate(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (m_estimate) {
            delete [] m_estimate;
            m_estimate = 0;
        }
        return WBIBlock::terminate(blockInfo, error);
    }

    bool GetEstimate::output(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        //get input
        wbi::wholeBodyInterface * const interface = WBInterface::sharedInstance().interface();
        if (interface) {
            interface->getEstimates(m_estimateType, m_estimate);
            Signal signal = blockInfo->getOutputPortSignal(0);
            signal.setBuffer(m_estimate, blockInfo->getOutputPortWidth(0));

            return true;
        }
        return false;
    }
}
