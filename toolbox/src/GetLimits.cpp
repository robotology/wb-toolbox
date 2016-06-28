#include "GetLimits.h"

#include "Error.h"
#include "BlockInformation.h"
#include "Signal.h"
#include "WBInterface.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>

namespace wbt {

    std::string GetLimits::ClassName = "GetLimits";

    struct GetLimits::Limit {
        double *min;
        double *max;

        Limit(unsigned size) : min(0), max(0)
        {
            min = new double[size]();
            max = new double[size]();
        }

        ~Limit()
        {
            if (min) {
                delete [] min;
                min = 0;
            }
            if (max) {
                delete [] max;
                max = 0;
            }
        }
    };

    GetLimits::GetLimits()
    : m_limits(0) {}

    unsigned GetLimits::numberOfParameters()
    {
        return WBIBlock::numberOfParameters() + 1;
    }

    bool GetLimits::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
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
        if (!blockInfo->setNumberOfOuputPorts(2)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        bool success = blockInfo->setOutputPortVectorSize(0, dofs); //Min limit
        success = success && blockInfo->setOutputPortVectorSize(1, dofs); //Max limit

        blockInfo->setOutputPortType(0, PortDataTypeDouble);
        blockInfo->setOutputPortType(1, PortDataTypeDouble);

        if (!success) {
            if (error) error->message = "Failed to configure output ports";
            return false;
        }


        return true;
    }

    bool GetLimits::initialize(BlockInformation *blockInfo, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIBlock::initialize(blockInfo, error)) return false;

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        wbi::wholeBodyInterface * const interface = WBInterface::sharedInstance().interface();

        // Reading the control type
        std::string limitType;
        if (!blockInfo->getStringParameterAtIndex(WBIBlock::numberOfParameters() + 1, limitType)) {
            if (error) error->message = "Could not read estimate type parameter";
            return false;
        }

        bool success = true;
        if (limitType == "Position") {
            m_limits = new Limit(dofs);
            if (m_limits) {
                success = interface->getJointLimits(m_limits->min, m_limits->max);
            }
        } else {
            if (error) error->message = "Limit type not supported";
            return false;
        }

        return m_limits && success;
    }

    bool GetLimits::terminate(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (m_limits) {
            delete m_limits;
            m_limits = 0;
        }
        return WBIBlock::terminate(blockInfo, error);
    }

    bool GetLimits::output(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        if (!m_limits) return false;

        Signal minPort = blockInfo->getOutputPortSignal(0);
        Signal maxPort = blockInfo->getOutputPortSignal(1);

        for (int i = 0; i < blockInfo->getOutputPortWidth(0); ++i) {
            minPort.set(i, m_limits->min[i]);
            maxPort.set(i, m_limits->max[i]);
        }
        return true;
    }
}
