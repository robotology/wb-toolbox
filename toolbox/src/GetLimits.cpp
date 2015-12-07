#include "GetLimits.h"

#include "Error.h"
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

    bool GetLimits::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        if (!WBIBlock::configureSizeAndPorts(S, error)) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        if (!ssSetNumInputPorts (S, 0)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }

        // Output port:
        // - DoFs vector with the information asked
        if (!ssSetNumOutputPorts (S, 2)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        bool success = ssSetOutputPortVectorDimension(S, 0, dofs); //Min limit
        success = success && ssSetOutputPortVectorDimension(S, 1, dofs); //Max limit

        ssSetOutputPortDataType (S, 0, SS_DOUBLE);
        ssSetOutputPortDataType (S, 1, SS_DOUBLE);

        if (!success) {
            if (error) error->message = "Failed to configure output ports";
            return false;
        }


        return true;
    }

    bool GetLimits::initialize(SimStruct *S, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIBlock::initialize(S, error)) return false;

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        wbi::wholeBodyInterface * const interface = WBInterface::sharedInstance().interface();

        // Reading the control type
        std::string limitType;
        if (!Block::readStringParameterAtIndex(S, WBIBlock::numberOfParameters() + 1, limitType)) {
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

    bool GetLimits::terminate(SimStruct *S, wbt::Error *error)
    {
        if (m_limits) {
            delete m_limits;
            m_limits = 0;
        }
        return WBIBlock::terminate(S, error);
    }

    bool GetLimits::output(SimStruct *S, wbt::Error */*error*/)
    {
        if (!m_limits) return false;

        real_T *minPort = ssGetOutputPortRealSignal(S, 0);
        real_T *maxPort = ssGetOutputPortRealSignal(S, 1);

        int_T widthPort = ssGetOutputPortWidth(S, 0);
        for (int i = 0; i < widthPort; ++i) {
            minPort[i] = m_limits->min[i];
            maxPort[i] = m_limits->max[i];
        }
        return true;
    }
}
