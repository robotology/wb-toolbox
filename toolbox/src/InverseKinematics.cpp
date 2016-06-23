#include "InverseKinematics.h"

#include "Error.h"
#include "WBInterface.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>
#include <Eigen/Core>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

namespace wbt {

    struct InverseKinematics::InverseKinematicsPimpl {
        //input buffers
        double *m_configuration;
        double *m_desiredPoseRaw;
        double *m_currentBasePoseRaw;
        double *m_currentBasePose;

        iCub::iKin::iCubLeg *m_leg;
        iCub::iKin::iKinIpOptMin *m_solver;

        yarp::sig::Matrix m_desiredPoseAsMatrix;
        yarp::sig::Vector m_desiredPoseAsAngleAxis;
        yarp::sig::Vector m_desiredAngleAxisOrientation;
        yarp::sig::Vector m_solverSolution;

        std::vector<int> m_jointIndexes;

        int m_rootFrameIndex;

        InverseKinematicsPimpl()
        : m_configuration(0)
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
        return WBIBlock::numberOfParameters() + 2;
    }

    bool InverseKinematics::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        if (!WBIBlock::configureSizeAndPorts(S, error)) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        // Specify I/O
        // Input ports:
        // - 4x4 matrix (homogenous transformation for the desired end effector pose w.r.t. world)

        if (!ssSetNumInputPorts (S, 3)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }
        bool success = true;
        success = success && ssSetInputPortMatrixDimensions(S,  0, 4, 4);
        success = success && ssSetInputPortMatrixDimensions(S,  1, 4, 4);
        success = success && ssSetInputPortVectorDimension(S, 2, dofs); //joint configuration

        ssSetInputPortDataType (S, 0, SS_DOUBLE);
        ssSetInputPortDataType (S, 1, SS_DOUBLE);
        ssSetInputPortDataType (S, 2, SS_DOUBLE);

        ssSetInputPortDirectFeedThrough (S, 0, 1);
        ssSetInputPortDirectFeedThrough (S, 1, 1);
        ssSetInputPortDirectFeedThrough (S, 2, 1);

        if (!success) {
            if (error) error->message = "Failed to configure input ports";
            return false;
        }

        // Output port:
        // - 4 x 4 homogenoues transformation matrix representing the desired base position
        // - DoFs desired joints configuration
        if (!ssSetNumOutputPorts (S, 2)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        success = ssSetOutputPortMatrixDimensions(S, 0, 4, 4);
        success = ssSetOutputPortVectorDimension(S, 1, dofs);
        ssSetOutputPortDataType (S, 0, SS_DOUBLE);
        ssSetOutputPortDataType (S, 1, SS_DOUBLE);

        return success;
    }

    bool InverseKinematics::initialize(SimStruct *S, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIModelBlock::initialize(S, error)) return false;

        m_piml = new InverseKinematicsPimpl();
        if (!m_piml) return false;

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        m_piml->m_configuration = new double[dofs];
        m_piml->m_desiredPoseRaw = new double[16];
        m_piml->m_currentBasePoseRaw = new double[16];
        m_piml->m_currentBasePose = new double[16];
        if (!m_piml->m_configuration || !m_piml->m_desiredPoseRaw) return false;

        int parentParameters = WBIBlock::numberOfParameters() + 1;
        std::string robotSide;
        if (!Block::readStringParameterAtIndex(S, parentParameters, robotSide)) {
            if (error) error->message = "Cannot retrieve string from robot part parameter";
            return false;
        }

        m_piml->m_leg = new iCub::iKin::iCubLeg(robotSide);
        if (!m_piml->m_leg) return false;

        m_piml->m_solver = new iCub::iKin::iKinIpOptMin(*m_piml->m_leg->asChain(), IKINCTRL_POSE_XYZ, 1e-3, 1e-6, 100);
        if (!m_piml->m_solver) return false;

        m_piml->m_solver->setUserScaling(true, 100.0, 100.0, 100.0);

        int optOption = mxGetScalar(ssGetSFcnParam(S,parentParameters + 1));
        if (optOption == 1)
            m_piml->m_solver->set_ctrlPose(IKINCTRL_POSE_FULL);
        else
            m_piml->m_solver->set_ctrlPose(IKINCTRL_POSE_XYZ);

        m_piml->m_desiredPoseAsMatrix.resize(4, 4);
        m_piml->m_desiredPoseAsAngleAxis.resize(7, 0.0);
        m_piml->m_desiredAngleAxisOrientation.resize(4, 0.0);
        m_piml->m_solverSolution.resize(6, 0.0);


        //Look for joint indexes
        std::string partPrefix = robotSide == "left" ? "l_" : "r_";
        m_piml->m_jointIndexes.reserve(6);
        wbi::IDList jointList = WBInterface::sharedInstance().model()->getJointList();
        int index = -1;

        if (!jointList.idToIndex(partPrefix + "hip_pitch", index)) m_piml->m_jointIndexes.push_back(-1);
        m_piml->m_jointIndexes.push_back(index);
        if (!jointList.idToIndex(partPrefix + "hip_roll", index)) m_piml->m_jointIndexes.push_back(-1);
        m_piml->m_jointIndexes.push_back(index);
        if (!jointList.idToIndex(partPrefix + "hip_yaw", index)) m_piml->m_jointIndexes.push_back(-1);
        m_piml->m_jointIndexes.push_back(index);
        if (!jointList.idToIndex(partPrefix + "knee", index)) m_piml->m_jointIndexes.push_back(-1);
        m_piml->m_jointIndexes.push_back(index);
        if (!jointList.idToIndex(partPrefix + "ankle_pitch", index)) m_piml->m_jointIndexes.push_back(-1);
        m_piml->m_jointIndexes.push_back(index);
        if (!jointList.idToIndex(partPrefix + "ankle_roll", index)) m_piml->m_jointIndexes.push_back(-1);
        m_piml->m_jointIndexes.push_back(index);

        return WBInterface::sharedInstance().model()->getFrameList().idToIndex("root_link", m_piml->m_rootFrameIndex);
    }

    bool InverseKinematics::terminate(SimStruct *S, wbt::Error *error)
    {
        if (m_piml) {
            delete m_piml;
            m_piml = 0;
        }
        return WBIModelBlock::terminate(S, error);
    }

    bool InverseKinematics::output(SimStruct *S, wbt::Error */*error*/)
    {
        //get input
        wbi::iWholeBodyModel * const interface = WBInterface::sharedInstance().model();
        if (interface) {
            InputRealPtrsType desiredPoseRaw = ssGetInputPortRealSignalPtrs(S, 0);
            InputRealPtrsType currentBaseRaw = ssGetInputPortRealSignalPtrs(S, 1);
            InputRealPtrsType configuration = ssGetInputPortRealSignalPtrs(S, 2);

            for (unsigned i = 0; i < ssGetInputPortWidth(S, 0); ++i) {
                m_piml->m_desiredPoseRaw[i] = *desiredPoseRaw[i];
            }

            for (unsigned i = 0; i < ssGetInputPortWidth(S, 1); ++i) {
                m_piml->m_currentBasePoseRaw[i] = *currentBaseRaw[i];
            }

            for (unsigned i = 0; i < ssGetInputPortWidth(S, 2); ++i) {
                m_piml->m_configuration[i] = *configuration[i];
            }

            Eigen::Map<Eigen::VectorXd > inputConfiguration(m_piml->m_configuration, ssGetInputPortWidth(S, 2));

            //Map initial configuration to chain joints
            for (size_t index = 0; index < m_piml->m_jointIndexes.size(); ++index) {
                if (m_piml->m_jointIndexes[index] == -1) continue;
                m_piml->m_leg->asChain()->setAng(index, m_piml->m_configuration[m_piml->m_jointIndexes[index]]);
            }

            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor> > basePoseColMajor(m_piml->m_currentBasePoseRaw);
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > basePose(m_piml->m_currentBasePose);
            basePose = basePoseColMajor;

            wbi::Frame w_H_base;
            wbi::frameFromSerialization(basePose.data(), w_H_base);

            wbi::Frame root_H_w;
            interface->computeH(m_piml->m_configuration, w_H_base, m_piml->m_rootFrameIndex, root_H_w);
            root_H_w.setToInverse();

            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor> > desiredPoseColMajor(m_piml->m_desiredPoseRaw);
            //Convert desiredPoseColMajor to yarp matrix
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    m_piml->m_desiredPoseAsMatrix(row, col) = desiredPoseColMajor(row, col);
                }
            }

            double root_H_w_data[16];
            root_H_w.get4x4Matrix(root_H_w_data);
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > root_H_w_eigen(root_H_w_data);
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > desiredPose(m_piml->m_desiredPoseAsMatrix.data());

            //iKin expect frames in root_link but we provide them in world
            //root_H_foot = root_H_w * w_H_foot
            desiredPose = root_H_w_eigen * desiredPose;

            //To vector
            m_piml->m_desiredPoseAsAngleAxis(0) = m_piml->m_desiredPoseAsMatrix(0, 3);
            m_piml->m_desiredPoseAsAngleAxis(1) = m_piml->m_desiredPoseAsMatrix(1, 3);
            m_piml->m_desiredPoseAsAngleAxis(2) = m_piml->m_desiredPoseAsMatrix(2, 3);

            m_piml->m_desiredAngleAxisOrientation = yarp::math::dcm2axis(m_piml->m_desiredPoseAsMatrix);
            m_piml->m_desiredPoseAsAngleAxis.setSubvector(3, m_piml->m_desiredAngleAxisOrientation);

            m_piml->m_solverSolution = m_piml->m_solver->solve(m_piml->m_leg->asChain()->getAng(), m_piml->m_desiredPoseAsAngleAxis);

            real_T *output = ssGetOutputPortRealSignal(S, 1);
            Eigen::Map<Eigen::VectorXd > outputPort(output, ssGetOutputPortWidth(S, 1));
            outputPort = inputConfiguration;


            for (size_t index = 0; index < m_piml->m_jointIndexes.size(); ++index) {
                if (m_piml->m_jointIndexes[index] == -1) continue;
                outputPort(m_piml->m_jointIndexes[index]) =  m_piml->m_solverSolution[index];
            }

            return true;
        }
        return false;
    }
}
