#include "GetEstimate.h"

#include "Error.h"
#include "WBInterface.h"
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

    bool GetEstimate::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
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
        if (!ssSetNumOutputPorts (S, 1)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        bool success = ssSetOutputPortVectorDimension(S, 0, dofs);
        ssSetOutputPortDataType (S, 0, SS_DOUBLE);
        if (!success) {
            if (error) error->message = "Failed to configure output ports";
            return false;
        }


        return true;
    }

    bool GetEstimate::initialize(SimStruct *S, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIBlock::initialize(S, error)) return false;

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        m_estimate = new double[dofs];

        // Reading the control type
        std::string informationType;
        if (!Block::readStringParameterAtIndex(S, WBIBlock::numberOfParameters() + 1, informationType)) {
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

    bool GetEstimate::terminate(SimStruct *S, wbt::Error *error)
    {
        if (m_estimate) {
            delete [] m_estimate;
            m_estimate = 0;
        }
        return WBIBlock::terminate(S, error);
    }

    bool GetEstimate::output(SimStruct *S, wbt::Error */*error*/)
    {
        //get input
        wbi::wholeBodyInterface * const interface = WBInterface::sharedInstance().interface();
        if (interface) {
            interface->getEstimates(m_estimateType, m_estimate);
            real_T *signal = ssGetOutputPortRealSignal(S, 0);
            int_T widthPort = ssGetOutputPortWidth(S, 0);
            for (int i = 0; i < widthPort; ++i) {
                signal[i] = m_estimate[i];
            }
            return true;
        }
        return false;
    }
}
