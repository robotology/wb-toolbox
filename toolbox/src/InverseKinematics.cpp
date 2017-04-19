#include "InverseKinematics.h"

#include "Error.h"
#include "WBInterface.h"
#include "BlockInformation.h"
#include "Signal.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>
#include <Eigen/Core>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/ResourceFinder.h>
#include <typeinfo>

//These includes are to convert from URDF to DH
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <iDynTree/ModelIO/impl/urdf_import.hpp>
#include <iDynTree/ModelIO/iKin_export.hpp>
//End of temporary include section

namespace wbt {

    static bool iKinLimbFromUrdfFile(const std::string &urdf_file_name, const std::string &base_link_name, const std::string& end_effector_link_name, iCub::iKin::iKinLimb &convertedChain, std::vector<std::string>& jointNames);

    struct InverseKinematics::InverseKinematicsPimpl {
        bool m_firstTime;
        //input buffers
        double *m_configuration;
        double *m_desiredPoseRaw;
        double *m_currentBasePoseRaw;
        double *m_currentBasePose;

        iCub::iKin::iKinChain *m_leg;
        iCub::iKin::iKinIpOptMin *m_solver;

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
        , m_solver(0) {}

        ~InverseKinematicsPimpl()
        {
            if (m_configuration) {
                delete [] m_configuration;
                m_configuration = 0;
            }
            if (m_desiredPoseRaw) {
                delete [] m_desiredPoseRaw;
                m_desiredPoseRaw = 0;
            }
            if (m_currentBasePoseRaw) {
                delete [] m_currentBasePoseRaw;
                m_currentBasePoseRaw = 0;
            }
            if (m_currentBasePose) {
                delete [] m_currentBasePose;
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
    : m_piml(0) {}

    unsigned InverseKinematics::numberOfParameters()
    {
        return WBIBlock::numberOfParameters() + 3;
        //Parameter 1: base link (string)
        //Parameter 2: end effector link (string)
        //Parameter 3: type of solution (Int)
    }

    bool InverseKinematics::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (!WBIBlock::configureSizeAndPorts(blockInfo, error)) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        // Specify I/O
        // Input ports:
        // - 4x4 matrix (homogenous transformation for the desired end effector pose w.r.t. world)

        if (!blockInfo->setNumberOfInputPorts(3)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }
        bool success = true;
        success = success && blockInfo->setInputPortMatrixSize(0, 4, 4);
        success = success && blockInfo->setInputPortMatrixSize(1, 4, 4);
        success = success && blockInfo->setInputPortVectorSize(2, dofs); //joint configuration

        blockInfo->setInputPortType(0, PortDataTypeDouble);
        blockInfo->setInputPortType(1, PortDataTypeDouble);
        blockInfo->setInputPortType(2, PortDataTypeDouble);

        if (!success) {
            if (error) error->message = "Failed to configure input ports";
            return false;
        }

        // Output port:
        // - 4 x 4 homogenoues transformation matrix representing the desired base position
        // - DoFs desired joints configuration
        if (!blockInfo->setNumberOfOuputPorts(2)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        success = blockInfo->setOutputPortMatrixSize(0, 4, 4);
        success = success && blockInfo->setOutputPortVectorSize(1, dofs);
        blockInfo->setOutputPortType(0, PortDataTypeDouble);
        blockInfo->setOutputPortType(1, PortDataTypeDouble);

        return success;
    }

    bool InverseKinematics::initialize(BlockInformation *blockInfo, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIModelBlock::initialize(blockInfo, error)) return false;

        m_piml = new InverseKinematicsPimpl();
        if (!m_piml) return false;

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        m_piml->m_configuration = new double[dofs];
        m_piml->m_desiredPoseRaw = new double[16];
        m_piml->m_currentBasePoseRaw = new double[16];
        m_piml->m_currentBasePose = new double[16];
        if (!m_piml->m_configuration || !m_piml->m_desiredPoseRaw) return false;

        int parentParameters = WBIBlock::numberOfParameters() + 1;
        std::string baseLink;
        if (!blockInfo->getStringParameterAtIndex(parentParameters, baseLink)) {
            if (error) error->message = "Cannot retrieve string from base link parameter";
            return false;
        }
        parentParameters++;
        std::string endEffectorLink;
        if (!blockInfo->getStringParameterAtIndex(parentParameters, endEffectorLink)) {
            if (error) error->message = "Cannot retrieve string from endeffector link parameter";
            return false;
        }

        wbi::IDList frameList = WBInterface::sharedInstance().model()->getFrameList();
        int frameIndex;
        if (!frameList.idToIndex(baseLink, frameIndex)) {
            if (error) error->message = "Cannot find " + baseLink + " frame";
            return false;
        }
        if (!frameList.idToIndex(endEffectorLink, frameIndex)) {
            if (error) error->message = "Cannot find " + endEffectorLink + " frame";
            return false;
        }
        
        std::string urdfFile;
        //wbi config file
        std::string wbiConfigFile;
        if (!blockInfo->getStringParameterAtIndex(3, wbiConfigFile)) {
            if (error) error->message = "Could not read WBI configuration file parameter";
            return false;
        }
        yarp::os::ResourceFinder resourceFinder = yarp::os::ResourceFinder::getResourceFinderSingleton();
        yarp::os::Property configurations;
        //loading defaults from configuration file
        if (!configurations.fromConfigFile(resourceFinder.findFile(wbiConfigFile))) {
            return false;
        }
        urdfFile = resourceFinder.findFile(configurations.find("urdf").asString());

        m_piml->m_leg = new iCub::iKin::iKinLimb();
        if (!m_piml->m_leg) return false;

        std::vector<std::string> jointNames;
        if (!iKinLimbFromUrdfFile(urdfFile, baseLink, endEffectorLink, *((iCub::iKin::iKinLimb*)m_piml->m_leg), jointNames)) {
            if (error) error->message = "Cannot convert urdf to iKin chain";
            return false;
        }

        m_piml->m_solver = new iCub::iKin::iKinIpOptMin(*m_piml->m_leg, IKINCTRL_POSE_XYZ, 1e-3, 1e-6, 100);
        if (!m_piml->m_solver) return false;

        m_piml->m_solver->setUserScaling(true, 100.0, 100.0, 100.0);

        int optOption = blockInfo->getScalarParameterAtIndex(parentParameters + 1).int32Data();
        if (optOption == 1)
            m_piml->m_solver->set_ctrlPose(IKINCTRL_POSE_FULL);
        else
            m_piml->m_solver->set_ctrlPose(IKINCTRL_POSE_XYZ);

        m_piml->m_desiredPoseAsMatrix.resize(4, 4);
        m_piml->m_desiredPoseAsAngleAxis.resize(7, 0.0);
        m_piml->m_desiredAngleAxisOrientation.resize(4, 0.0);
        m_piml->m_solverSolution.resize(m_piml->m_leg->getDOF(), 0.0);


        //Look for joint indexes
        wbi::IDList jointList = WBInterface::sharedInstance().model()->getJointList();
        int index = -1;

        for (std::vector<std::string>::const_iterator it = jointNames.begin();
             it != jointNames.end(); ++it   ) {
            if (jointList.idToIndex(*it, index))
                m_piml->m_jointIndexes.push_back(index);
        }

        m_piml->m_firstTime = true;

        return true;
    }

    bool InverseKinematics::terminate(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (m_piml) {
            delete m_piml;
            m_piml = 0;
        }
        return WBIModelBlock::terminate(blockInfo, error);
    }

    bool InverseKinematics::output(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        //get input
        wbi::iWholeBodyModel * const interface = WBInterface::sharedInstance().model();
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

            Eigen::Map<Eigen::VectorXd > inputConfiguration(m_piml->m_configuration, blockInfo->getInputPortWidth(2));

            //Map initial configuration to chain joints
            for (size_t index = 0; index < m_piml->m_jointIndexes.size(); ++index) {
                if (m_piml->m_jointIndexes[index] == -1) continue;
                m_piml->m_leg->setAng(index, m_piml->m_configuration[m_piml->m_jointIndexes[index]]);
                if (m_piml->m_firstTime) {
                    m_piml->m_solverSolution(index) = m_piml->m_configuration[m_piml->m_jointIndexes[index]];
                }
            }

            m_piml->m_firstTime = false;

            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor> > basePoseColMajor(m_piml->m_currentBasePoseRaw);
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > basePose(m_piml->m_currentBasePose);
            basePose = basePoseColMajor;

            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor> > desiredPoseColMajor(m_piml->m_desiredPoseRaw);
            //Convert desiredPoseColMajor to yarp matrix
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    m_piml->m_desiredPoseAsMatrix(row, col) = desiredPoseColMajor(row, col);
                }
            }

            //To vector
            m_piml->m_desiredPoseAsAngleAxis(0) = m_piml->m_desiredPoseAsMatrix(0, 3);
            m_piml->m_desiredPoseAsAngleAxis(1) = m_piml->m_desiredPoseAsMatrix(1, 3);
            m_piml->m_desiredPoseAsAngleAxis(2) = m_piml->m_desiredPoseAsMatrix(2, 3);

            m_piml->m_desiredAngleAxisOrientation = yarp::math::dcm2axis(m_piml->m_desiredPoseAsMatrix);
            m_piml->m_desiredPoseAsAngleAxis.setSubvector(3, m_piml->m_desiredAngleAxisOrientation);

            int exitCode = -5;

            m_piml->m_solverSolution = m_piml->m_solver->solve(m_piml->m_solverSolution,
                                                               m_piml->m_desiredPoseAsAngleAxis,
                                                               0, m_piml->m_desiredPoseAsAngleAxis, m_piml->m_desiredPoseAsAngleAxis,
                                                               0, m_piml->m_desiredPoseAsAngleAxis, m_piml->m_desiredPoseAsAngleAxis,
                                                               &exitCode);

            if (exitCode != 0)
                std::cerr << "Exit code: " << exitCode << "\n";

            Signal output = blockInfo->getOutputPortSignal(1);
            Eigen::Map<Eigen::VectorXd > outputPort((double*)output.getContiguousBuffer(), blockInfo->getOutputPortWidth(1));
            outputPort = inputConfiguration;


            for (size_t index = 0; index < m_piml->m_solverSolution.size(); ++index) {
                outputPort(m_piml->m_jointIndexes[index]) =  m_piml->m_solverSolution[index];
            }

            return true;
        }
        return false;
    }


    bool iKinLimbFromUrdfFile(const std::string &urdf_file_name, const std::string &base_link_name, const std::string& end_effector_link_name, iCub::iKin::iKinLimb &convertedChain, std::vector<std::string>& jointNames) {

        KDL::Tree kdl_tree;
        KDL::Chain kdl_chain;
        std::vector<std::string> joint_names;
        KDL::JntArray min,max;

        //
        // URDF --> KDL::Tree
        //
        bool root_inertia_workaround = true;
        if (!iDynTree::treeFromUrdfFile(urdf_file_name,kdl_tree,root_inertia_workaround))
        {
            std::cerr << "Could not parse urdf robot model" << std::endl;
            std::cerr << "Please open an issue at https://github.com/robotology-playground/idyntree/issues " << std::endl;

            return false;
        }

        //
        // URDF --> position ranges
        //
        if (!iDynTree::jointPosLimitsFromUrdfFile(urdf_file_name,joint_names,min,max))
        {
            std::cerr << "Could not parse urdf robot model limits" << std::endl;
            return false;
        }

        if (joint_names.size() != min.rows() ||
           joint_names.size() != max.rows() ||
           joint_names.size() == 0)
        {
            std::cerr << "Inconsistent joint limits got from urdf (nr of joints extracted: " << joint_names.size() << " ) " << std::endl;
            return false;
        }

        //
        // KDL::Tree --> KDL::CoDyCo::UndirectedTree
        // (for extracting arbitrary chains,
        //    using KDL::Tree you can just get chains where the base of the chain
        //    is proximal to the tree base with respect to the end effector.
        //
        KDL::CoDyCo::UndirectedTree undirected_tree(kdl_tree);

        KDL::Tree kdl_rotated_tree = undirected_tree.getTree(base_link_name);

        bool result = kdl_rotated_tree.getChain(base_link_name,end_effector_link_name,kdl_chain);
        if (!result)
        {
            std::cerr << "Impossible to find " << base_link_name << " or "
            << end_effector_link_name << " in the URDF."  << std::endl;
            return false;
        }

        //
        // Copy the limits extracted from the URDF to the chain
        //
        int nj = kdl_chain.getNrOfJoints();
        KDL::JntArray chain_min(nj), chain_max(nj);

        jointNames.clear();
        jointNames.reserve(nj);

        size_t seg_i, jnt_i;
        for (seg_i = 0,jnt_i = 0; seg_i < kdl_chain.getNrOfSegments(); seg_i++)
        {
            const KDL::Segment & seg = kdl_chain.getSegment(seg_i);
            if( seg.getJoint().getType() != KDL::Joint::None )
            {
                std::string jnt_name = seg.getJoint().getName();
                // std::cerr << "searching for joint " << jnt_name << std::endl;
                int tree_jnt = 0;
                for(tree_jnt = 0; tree_jnt < joint_names.size(); tree_jnt++ )
                {
                    //std::cerr << "joint_names[ " << tree_jnt << "] is " << joint_names[tree_jnt] << std::endl;
                    if( joint_names[tree_jnt] == jnt_name )
                    {
                        chain_min(jnt_i) = min(tree_jnt);
                        chain_max(jnt_i) = max(tree_jnt);
                        jnt_i++;
                        jointNames.push_back(jnt_name);
                        break;
                    }
                }
                if( tree_jnt == joint_names.size() )
                {
                    std::cerr << "Failure in converting limits from tree to chain, unable to find joint " << jnt_name << std::endl;
                    return false;
                }
            }
        }
        
        if (jnt_i != nj)
        {
            std::cerr << "Failure in converting limits from tree to chain" << std::endl;
            return false;
        }
        
        //
        // Convert the chain and the limits to an iKin chain (i.e. DH parameters)
        //
        result = iDynTree::iKinLimbFromKDLChain(kdl_chain,convertedChain,chain_min,chain_max);
        if (!result)
        {
            std::cerr << "Could not export KDL::Tree to iKinChain" << std::endl;
            return false;
        }
        return true;
    }
}
