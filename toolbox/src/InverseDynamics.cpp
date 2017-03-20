#include "InverseDynamics.h"

#include "Error.h"
#include "BlockInformation.h"
#include "Signal.h"
#include "WBInterface.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>
#include <Eigen/Core>

#define PARAM_IDX_1 5

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

    bool InverseDynamics::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (!WBIBlock::configureSizeAndPorts(blockInfo, error)) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        // Specify I/O
        // Input ports:
        // - 4x4 matrix (homogenous transformation for the base pose w.r.t. world)
        // - DoFs vector for the robot (joints) configurations

        m_explicitGravity = blockInfo->getScalarParameterAtIndex(PARAM_IDX_1).booleanData();

        int portNumber = 6;
        if (m_explicitGravity) portNumber++;

        if (!blockInfo->setNumberOfInputPorts(portNumber)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }
        bool success = true;

        success = success && blockInfo->setInputPortMatrixSize(0, 4, 4); //base pose
        success = success && blockInfo->setInputPortVectorSize(1, dofs); //joint configuration
        success = success && blockInfo->setInputPortVectorSize(2, 6); //base velocity
        success = success && blockInfo->setInputPortVectorSize(3, dofs); //joints velocitity
        success = success && blockInfo->setInputPortVectorSize(4, 6); //base acceleration
        success = success && blockInfo->setInputPortVectorSize(5, dofs); //joints acceleration
        if (m_explicitGravity)
            success = success && blockInfo->setInputPortVectorSize(6, 3); //gravity acceleration

        blockInfo->setInputPortType(0, PortDataTypeDouble);
        blockInfo->setInputPortType(1, PortDataTypeDouble);
        blockInfo->setInputPortType(2, PortDataTypeDouble);
        blockInfo->setInputPortType(3, PortDataTypeDouble);
        blockInfo->setInputPortType(4, PortDataTypeDouble);
        blockInfo->setInputPortType(5, PortDataTypeDouble);

        if (m_explicitGravity)
            blockInfo->setInputPortType(6, PortDataTypeDouble);

        if (!success) {
            if (error) error->message = "Failed to configure input ports";
            return false;
        }

        // Output port:
        // - DoFs + 6) vector representing the torques
        if (!blockInfo->setNumberOfOuputPorts(1)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        success = blockInfo->setOutputPortVectorSize(0, dofs + 6);
        blockInfo->setOutputPortType(0, PortDataTypeDouble);

        return success;
    }

    bool InverseDynamics::initialize(BlockInformation *blockInfo, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIModelBlock::initialize(blockInfo, error)) return false;

        m_explicitGravity = blockInfo->getScalarParameterAtIndex(PARAM_IDX_1).booleanData();

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

    bool InverseDynamics::terminate(BlockInformation *blockInfo, wbt::Error *error)
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

        return WBIModelBlock::terminate(blockInfo, error);
    }

    bool InverseDynamics::output(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        //get input
        wbi::iWholeBodyModel * const interface = WBInterface::sharedInstance().model();
        if (interface) {

            Signal basePoseRaw = blockInfo->getInputPortSignal(0);
            Signal configuration = blockInfo->getInputPortSignal(1);
            Signal baseVelocity = blockInfo->getInputPortSignal(2);
            Signal jointsVelocity = blockInfo->getInputPortSignal(3);
            Signal baseAcceleration = blockInfo->getInputPortSignal(4);
            Signal jointsAcceleration = blockInfo->getInputPortSignal(5);

            for (unsigned i = 0; i < blockInfo->getInputPortWidth(0); ++i) {
                m_basePoseRaw[i] = basePoseRaw.get(i).doubleData();
            }
            for (unsigned i = 0; i < blockInfo->getInputPortWidth(1); ++i) {
                m_configuration[i] = configuration.get(i).doubleData();
            }
            for (unsigned i = 0; i < blockInfo->getInputPortWidth(2); ++i) {
                m_baseVelocity[i] = baseVelocity.get(i).doubleData();
            }
            for (unsigned i = 0; i < blockInfo->getInputPortWidth(3); ++i) {
                m_jointsVelocity[i] = jointsVelocity.get(i).doubleData();
            }
            for (unsigned i = 0; i < blockInfo->getInputPortWidth(4); ++i) {
                m_baseAcceleration[i] = baseAcceleration.get(i).doubleData();
            }
            for (unsigned i = 0; i < blockInfo->getInputPortWidth(5); ++i) {
                m_jointsAcceleration[i] = jointsAcceleration.get(i).doubleData();
            }

            if (m_explicitGravity) {
                Signal gravity = blockInfo->getInputPortSignal(6);
                for (unsigned i = 0; i < blockInfo->getInputPortWidth(6); ++i) {
                    m_gravity[i] = gravity.get(i).doubleData();
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

            Signal output = blockInfo->getOutputPortSignal(0);
            output.setBuffer(m_torques, blockInfo->getOutputPortWidth(0));

            return true;
        }
        return false;
    }
}
