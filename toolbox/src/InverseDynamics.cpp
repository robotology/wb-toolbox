#include "InverseDynamics.h"

#include "Error.h"
#include "WBInterface.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>
#include <Eigen/Core>

#define PARAM_IDX_1 5
#define GET_OPT_EXPLICIT_GRAVITY static_cast<bool>(mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_1)))

namespace wbt {

    std::string InverseDynamics::ClassName = "InverseDynamics";

    InverseDynamics::InverseDynamics()
    : m_basePose(0)
    , m_torques(0)
    , m_explicitGravity(false)
    , m_basePoseRaw(0)
    , m_configuration(0)
    , m_baseVelocity(0)
    , m_jointsVelocity(0)
    , m_baseAcceleration(0)
    , m_jointsAcceleration(0)
    {
        m_gravity[0] = 0;
        m_gravity[1] = 0;
        m_gravity[2] = -9.81;
    }

    unsigned InverseDynamics::numberOfParameters()
    {
        return WBIBlock::numberOfParameters() + 1;
    }

    bool InverseDynamics::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        if (!WBIBlock::configureSizeAndPorts(S, error)) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        // Specify I/O
        // Input ports:
        // - 4x4 matrix (homogenous transformation for the base pose w.r.t. world)
        // - DoFs vector for the robot (joints) configurations

        m_explicitGravity = GET_OPT_EXPLICIT_GRAVITY;

        int_T portNumber = 6;
        if (m_explicitGravity) portNumber++;

        if (!ssSetNumInputPorts (S, portNumber)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }
        bool success = true;

        success = success && ssSetInputPortMatrixDimensions(S,  0, 4, 4); //base pose
        success = success && ssSetInputPortVectorDimension(S, 1, dofs); //joint configuration
        success = success && ssSetInputPortVectorDimension(S, 2, 6); //base velocity
        success = success && ssSetInputPortVectorDimension(S, 3, dofs); //joints velocitity
        success = success && ssSetInputPortVectorDimension(S, 4, 6); //base acceleration
        success = success && ssSetInputPortVectorDimension(S, 5, dofs); //joints acceleration
        if (m_explicitGravity)
            success = success && ssSetInputPortVectorDimension(S, 6, 3); //gravity acceleration

        ssSetInputPortDataType (S, 0, SS_DOUBLE);
        ssSetInputPortDataType (S, 1, SS_DOUBLE);
        ssSetInputPortDataType (S, 2, SS_DOUBLE);
        ssSetInputPortDataType (S, 3, SS_DOUBLE);
        ssSetInputPortDataType (S, 4, SS_DOUBLE);
        ssSetInputPortDataType (S, 5, SS_DOUBLE);
        if (m_explicitGravity)
            ssSetInputPortDataType (S, 6, SS_DOUBLE);

        ssSetInputPortDirectFeedThrough (S, 0, 1);
        ssSetInputPortDirectFeedThrough (S, 1, 1);
        ssSetInputPortDirectFeedThrough (S, 2, 1);
        ssSetInputPortDirectFeedThrough (S, 3, 1);
        ssSetInputPortDirectFeedThrough (S, 4, 1);
        ssSetInputPortDirectFeedThrough (S, 5, 1);
        if (m_explicitGravity)
            ssSetInputPortDirectFeedThrough (S, 6, 1);

        if (!success) {
            if (error) error->message = "Failed to configure input ports";
            return false;
        }

        // Output port:
        // - DoFs + 6) vector representing the torques
        if (!ssSetNumOutputPorts (S, 1)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        success = ssSetOutputPortVectorDimension(S, 0, dofs + 6);
        ssSetOutputPortDataType (S, 0, SS_DOUBLE);

        return success;
    }

    bool InverseDynamics::initialize(SimStruct *S, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIModelBlock::initialize(S, error)) return false;

        m_explicitGravity = GET_OPT_EXPLICIT_GRAVITY;

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        m_basePose = new double[16];
        m_torques = new double[6 + dofs];
        m_basePoseRaw = new double[16];
        m_configuration = new double[dofs];
        m_baseVelocity = new double[6];
        m_jointsVelocity = new double[dofs];
        m_baseAcceleration = new double[6];
        m_jointsAcceleration = new double[dofs];

        return m_basePose && m_torques && m_basePoseRaw && m_configuration && m_baseVelocity && m_jointsVelocity && m_baseAcceleration && m_jointsAcceleration;
    }

    bool InverseDynamics::terminate(SimStruct *S, wbt::Error *error)
    {
        if (m_basePose) {
            delete [] m_basePose;
            m_basePose = 0;
        }
        if (m_torques) {
            delete [] m_torques;
            m_torques = 0;
        }
        if (m_basePoseRaw) {
            delete [] m_basePoseRaw;
            m_basePoseRaw = 0;
        }
        if (m_configuration) {
            delete [] m_configuration;
            m_configuration = 0;
        }
        if (m_baseVelocity) {
            delete [] m_baseVelocity;
            m_baseVelocity = 0;
        }
        if (m_jointsVelocity) {
            delete [] m_jointsVelocity;
            m_jointsVelocity = 0;
        }

        if (m_baseAcceleration) {
            delete [] m_baseAcceleration;
            m_baseAcceleration = 0;
        }

        if (m_jointsAcceleration) {
            delete [] m_jointsAcceleration;
            m_jointsAcceleration = 0;
        }

        return WBIModelBlock::terminate(S, error);
    }

    bool InverseDynamics::output(SimStruct *S, wbt::Error */*error*/)
    {
        //get input
        wbi::iWholeBodyModel * const interface = WBInterface::sharedInstance().model();
        if (interface) {
            InputRealPtrsType basePoseRaw = ssGetInputPortRealSignalPtrs(S, 0);
            InputRealPtrsType configuration = ssGetInputPortRealSignalPtrs(S, 1);
            InputRealPtrsType baseVelocity = ssGetInputPortRealSignalPtrs(S, 2);
            InputRealPtrsType jointsVelocity = ssGetInputPortRealSignalPtrs(S, 3);
            InputRealPtrsType baseAcceleration = ssGetInputPortRealSignalPtrs(S, 4);
            InputRealPtrsType jointsAcceleration = ssGetInputPortRealSignalPtrs(S, 5);

            for (unsigned i = 0; i < ssGetInputPortWidth(S, 0); ++i) {
                m_basePoseRaw[i] = *basePoseRaw[i];
            }
            for (unsigned i = 0; i < ssGetInputPortWidth(S, 1); ++i) {
                m_configuration[i] = *configuration[i];
            }
            for (unsigned i = 0; i < ssGetInputPortWidth(S, 2); ++i) {
                m_baseVelocity[i] = *baseVelocity[i];
            }
            for (unsigned i = 0; i < ssGetInputPortWidth(S, 3); ++i) {
                m_jointsVelocity[i] = *jointsVelocity[i];
            }
            for (unsigned i = 0; i < ssGetInputPortWidth(S, 4); ++i) {
                m_baseAcceleration[i] = *baseAcceleration[i];
            }
            for (unsigned i = 0; i < ssGetInputPortWidth(S, 5); ++i) {
                m_jointsAcceleration[i] = *jointsAcceleration[i];
            }

            if (m_explicitGravity) {
                InputRealPtrsType gravity = ssGetInputPortRealSignalPtrs(S, 6);
                for (unsigned i = 0; i < ssGetInputPortWidth(S, 6); ++i) {
                    m_gravity[i] = *gravity[i];
                }
            }

            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor> > basePoseColMajor(m_basePoseRaw);
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > basePose(m_basePose);
            basePose = basePoseColMajor;

            wbi::Frame frame;
            wbi::frameFromSerialization(basePose.data(), frame);

            interface->inverseDynamics(m_configuration,
                                       frame,
                                       m_jointsVelocity,
                                       m_baseVelocity,
                                       m_jointsAcceleration,
                                       m_baseAcceleration,
                                       m_gravity,
                                       m_torques);

            real_T *output = ssGetOutputPortRealSignal(S, 0);
            for (unsigned i = 0; i < ssGetOutputPortWidth(S, 0); ++i) {
                output[i] = m_torques[i];
            }

            return true;
        }
        return false;
    }
}
