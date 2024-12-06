#include "InverseKinematics.h"
#include "BlockInformation.h"
#include "Error.h"
#include "Signal.h"
#include "WBInterface.h"

#include <Eigen/Core>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <iDynTree/DenavitHartenberg.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/iKinConversions.h>
#include <wbi/wholeBodyInterface.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarpWholeBodyInterface/yarpWbiUtil.h>

#include <iostream>
#include <typeinfo>

namespace wbt {

    struct InverseKinematics::InverseKinematicsPimpl
    {
        bool m_firstTime;
        // input buffers
        double* m_configuration;
        double* m_desiredPoseRaw;
        double* m_currentBasePoseRaw;
        double* m_currentBasePose;

        iCub::iKin::iKinChain* m_leg;
        iCub::iKin::iKinIpOptMin* m_solver;

        yarp::sig::Matrix m_desiredPoseAsMatrix;
        yarp::sig::Vector m_desiredPoseAsAngleAxis;
        yarp::sig::Vector m_desiredAngleAxisOrientation;
        yarp::sig::Vector m_solverSolution;

        std::vector<int> m_jointIndexes;

        int m_rootFrameIndex;

        InverseKinematicsPimpl()
            : m_firstTime(true)
            , m_configuration(0)
            , m_desiredPoseRaw(0)
            , m_currentBasePoseRaw(0)
            , m_currentBasePose(0)
            , m_leg(0)
            , m_solver(0)
        {}

        ~InverseKinematicsPimpl()
        {
            if (m_configuration) {
                delete[] m_configuration;
                m_configuration = 0;
            }
            if (m_desiredPoseRaw) {
                delete[] m_desiredPoseRaw;
                m_desiredPoseRaw = 0;
            }
            if (m_currentBasePoseRaw) {
                delete[] m_currentBasePoseRaw;
                m_currentBasePoseRaw = 0;
            }
            if (m_currentBasePose) {
                delete[] m_currentBasePose;
                m_currentBasePose = 0;
            }
            if (m_solver) {
                delete m_solver;
                m_solver = 0;
            }
            if (m_leg) {
                delete m_leg;
                m_leg = 0;
            }
        }
    };

    std::string InverseKinematics::ClassName = "InverseKinematics";

    InverseKinematics::InverseKinematics()
        : m_piml(0)
    {}

    unsigned InverseKinematics::numberOfParameters()
    {
        return WBIBlock::numberOfParameters() + 3;
        // Parameter 1: base link (string)
        // Parameter 2: end effector link (string)
        // Parameter 3: type of solution (Int)
    }

    bool InverseKinematics::configureSizeAndPorts(BlockInformation* blockInfo, wbt::Error* error)
    {
        if (!WBIBlock::configureSizeAndPorts(blockInfo, error)) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        // Specify I/O
        // Input ports:
        // - 4x4 matrix (homogenous transformation for the desired end effector pose w.r.t. world)

        if (!blockInfo->setNumberOfInputPorts(3)) {
            if (error) {
                error->message = "Failed to configure the number of input ports";
            }
            return false;
        }
        bool success = true;
        success = success && blockInfo->setInputPortMatrixSize(0, 4, 4);
        success = success && blockInfo->setInputPortMatrixSize(1, 4, 4);
        success = success && blockInfo->setInputPortVectorSize(2, dofs); // joint configuration

        blockInfo->setInputPortType(0, PortDataTypeDouble);
        blockInfo->setInputPortType(1, PortDataTypeDouble);
        blockInfo->setInputPortType(2, PortDataTypeDouble);

        if (!success) {
            if (error) {
                error->message = "Failed to configure input ports";
            }
            return false;
        }

        // Output port:
        // - 4 x 4 homogenoues transformation matrix representing the desired base position
        // - DoFs desired joints configuration
        if (!blockInfo->setNumberOfOuputPorts(2)) {
            if (error) {
                error->message = "Failed to configure the number of output ports";
            }
            return false;
        }

        success = blockInfo->setOutputPortMatrixSize(0, 4, 4);
        success = success && blockInfo->setOutputPortVectorSize(1, dofs);
        blockInfo->setOutputPortType(0, PortDataTypeDouble);
        blockInfo->setOutputPortType(1, PortDataTypeDouble);

        return success;
    }

    bool InverseKinematics::initialize(BlockInformation* blockInfo, wbt::Error* error)
    {
        using namespace yarp::os;
        if (!WBIModelBlock::initialize(blockInfo, error)) {
            return false;
        }

        m_piml = new InverseKinematicsPimpl();
        if (!m_piml) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        m_piml->m_configuration = new double[dofs];
        m_piml->m_desiredPoseRaw = new double[16];
        m_piml->m_currentBasePoseRaw = new double[16];
        m_piml->m_currentBasePose = new double[16];
        if (!m_piml->m_configuration || !m_piml->m_desiredPoseRaw) {
            return false;
        }

        int parentParameters = WBIBlock::numberOfParameters() + 1;
        std::string baseLink;
        if (!blockInfo->getStringParameterAtIndex(parentParameters, baseLink)) {
            if (error) {
                error->message = "Cannot retrieve string from base link parameter";
            }
            return false;
        }
        parentParameters++;
        std::string endEffectorLink;
        if (!blockInfo->getStringParameterAtIndex(parentParameters, endEffectorLink)) {
            if (error) {
                error->message = "Cannot retrieve string from endeffector link parameter";
            }
            return false;
        }

        wbi::IDList frameList = WBInterface::sharedInstance().model()->getFrameList();
        int frameIndex;
        if (!frameList.idToIndex(baseLink, frameIndex)) {
            if (error) {
                error->message = "Cannot find " + baseLink + " frame";
            }
            return false;
        }
        if (!frameList.idToIndex(endEffectorLink, frameIndex)) {
            if (error) {
                error->message = "Cannot find " + endEffectorLink + " frame";
            }
            return false;
        }

        std::string urdfFile;
        // wbi config file
        std::string wbiConfigFile;
        if (!blockInfo->getStringParameterAtIndex(3, wbiConfigFile)) {
            if (error) {
                error->message = "Could not read WBI configuration file parameter";
            }
            return false;
        }
        yarp::os::ResourceFinder resourceFinder =
            yarp::os::ResourceFinder::getResourceFinderSingleton();
        yarp::os::Property configurations;
        // loading defaults from configuration file
        if (!configurations.fromConfigFile(resourceFinder.findFile(wbiConfigFile))) {
            return false;
        }
        urdfFile = resourceFinder.findFile(configurations.find("urdf").asString());

        m_piml->m_leg = new iCub::iKin::iKinLimb();
        if (!m_piml->m_leg) {
            return false;
        }

        std::vector<std::string> jointNames;
        iDynTree::ModelLoader loader;
        loader.loadModelFromFile(urdfFile);
        if (!loader.isValid()) {
            if (error) {
                error->message = "Cannot load urdf file";
            }
            return false;
        }

        iDynTree::DHChain dhChain;
        if (!iDynTree::ExtractDHChainFromModel(
                loader.model(), baseLink, endEffectorLink, dhChain)) {
            if (error) {
                error->message = "Cannot extract DH parameters from model";
            }
            return false;
        }
        if (!iDynTree::iKinLimbFromDHChain(dhChain, *((iCub::iKin::iKinLimb*) m_piml->m_leg))) {
            if (error) {
                error->message = "Cannot convert DH parameters to iKin chain";
            }
            return false;
        }

        // Retrieve joint names
        jointNames.reserve(dhChain.getNrOfDOFs());
        for (size_t i = 0; i < dhChain.getNrOfDOFs(); ++i) {
            jointNames.push_back(dhChain.getDOFName(i));
        }

        m_piml->m_solver =
            new iCub::iKin::iKinIpOptMin(*m_piml->m_leg, IKINCTRL_POSE_XYZ, 1e-3, 1e-6, 100);
        if (!m_piml->m_solver) {
            return false;
        }

        m_piml->m_solver->setUserScaling(true, 100.0, 100.0, 100.0);

        int optOption = blockInfo->getScalarParameterAtIndex(parentParameters + 1).int32Data();
        if (optOption == 1) {
            m_piml->m_solver->set_ctrlPose(IKINCTRL_POSE_FULL);
        }
        else {
            m_piml->m_solver->set_ctrlPose(IKINCTRL_POSE_XYZ);
        }

        m_piml->m_desiredPoseAsMatrix.resize(4, 4);
        m_piml->m_desiredPoseAsAngleAxis.resize(7, 0.0);
        m_piml->m_desiredAngleAxisOrientation.resize(4, 0.0);
        m_piml->m_solverSolution.resize(m_piml->m_leg->getDOF(), 0.0);

        // Look for joint indexes
        wbi::IDList jointList = WBInterface::sharedInstance().model()->getJointList();
        int index = -1;

        for (std::vector<std::string>::const_iterator it = jointNames.begin();
             it != jointNames.end();
             ++it) {
            if (jointList.idToIndex(*it, index)) {
                m_piml->m_jointIndexes.push_back(index);
            }
        }

        m_piml->m_firstTime = true;

        return true;
    }

    bool InverseKinematics::terminate(BlockInformation* blockInfo, wbt::Error* error)
    {
        if (m_piml) {
            delete m_piml;
            m_piml = 0;
        }
        return WBIModelBlock::terminate(blockInfo, error);
    }

    bool InverseKinematics::output(BlockInformation* blockInfo, wbt::Error* /*error*/)
    {
        // get input
        wbi::iWholeBodyModel* const interface = WBInterface::sharedInstance().model();
        if (interface) {
            Signal desiredPoseRaw = blockInfo->getInputPortSignal(0);
            Signal currentBaseRaw = blockInfo->getInputPortSignal(1);
            Signal configuration = blockInfo->getInputPortSignal(2);

            for (unsigned i = 0; i < blockInfo->getInputPortWidth(0); ++i) {
                m_piml->m_desiredPoseRaw[i] = desiredPoseRaw.get(i).doubleData();
            }

            for (unsigned i = 0; i < blockInfo->getInputPortWidth(1); ++i) {
                m_piml->m_currentBasePoseRaw[i] = currentBaseRaw.get(i).doubleData();
            }

            for (unsigned i = 0; i < blockInfo->getInputPortWidth(2); ++i) {
                m_piml->m_configuration[i] = configuration.get(i).doubleData();
            }

            Eigen::Map<Eigen::VectorXd> inputConfiguration(m_piml->m_configuration,
                                                           blockInfo->getInputPortWidth(2));

            // Map initial configuration to chain joints
            for (size_t index = 0; index < m_piml->m_jointIndexes.size(); ++index) {
                if (m_piml->m_jointIndexes[index] == -1) {
                    continue;
                }
                m_piml->m_leg->setAng(index,
                                      m_piml->m_configuration[m_piml->m_jointIndexes[index]]);
                if (m_piml->m_firstTime) {
                    m_piml->m_solverSolution(index) =
                        m_piml->m_configuration[m_piml->m_jointIndexes[index]];
                }
            }

            m_piml->m_firstTime = false;

            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> basePoseColMajor(
                m_piml->m_currentBasePoseRaw);
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> basePose(
                m_piml->m_currentBasePose);
            basePose = basePoseColMajor;

            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> desiredPoseColMajor(
                m_piml->m_desiredPoseRaw);
            // Convert desiredPoseColMajor to yarp matrix
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    m_piml->m_desiredPoseAsMatrix(row, col) = desiredPoseColMajor(row, col);
                }
            }

            // To vector
            m_piml->m_desiredPoseAsAngleAxis(0) = m_piml->m_desiredPoseAsMatrix(0, 3);
            m_piml->m_desiredPoseAsAngleAxis(1) = m_piml->m_desiredPoseAsMatrix(1, 3);
            m_piml->m_desiredPoseAsAngleAxis(2) = m_piml->m_desiredPoseAsMatrix(2, 3);

            m_piml->m_desiredAngleAxisOrientation =
                yarp::math::dcm2axis(m_piml->m_desiredPoseAsMatrix);
            m_piml->m_desiredPoseAsAngleAxis.setSubvector(3, m_piml->m_desiredAngleAxisOrientation);

            int exitCode = -5;

            m_piml->m_solverSolution = m_piml->m_solver->solve(m_piml->m_solverSolution,
                                                               m_piml->m_desiredPoseAsAngleAxis,
                                                               0,
                                                               m_piml->m_desiredPoseAsAngleAxis,
                                                               m_piml->m_desiredPoseAsAngleAxis,
                                                               0,
                                                               m_piml->m_desiredPoseAsAngleAxis,
                                                               m_piml->m_desiredPoseAsAngleAxis,
                                                               &exitCode);

            if (exitCode != 0) {
                std::cerr << "Exit code: " << exitCode << "\n";
            }

            Signal output = blockInfo->getOutputPortSignal(1);
            Eigen::Map<Eigen::VectorXd> outputPort((double*) output.getContiguousBuffer(),
                                                   blockInfo->getOutputPortWidth(1));
            outputPort = inputConfiguration;

            for (size_t index = 0; index < m_piml->m_solverSolution.size(); ++index) {
                outputPort(m_piml->m_jointIndexes[index]) = m_piml->m_solverSolution[index];
            }

            return true;
        }
        return false;
    }

} // namespace wbt
