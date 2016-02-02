#include "MassMatrix.h"

#include "Error.h"
#include "WBInterface.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>
#include <Eigen/Core>

namespace wbt {

    std::string MassMatrix::ClassName = "MassMatrix";

    MassMatrix::MassMatrix()
    : m_basePose(0)
    , m_massMatrix(0)
    , m_basePoseRaw(0)
    , m_configuration(0) {}

    bool MassMatrix::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        if (!WBIBlock::configureSizeAndPorts(S, error)) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        // Specify I/O
        // Input ports:
        // - 4x4 matrix (homogenous transformation for the base pose w.r.t. world)
        // - DoFs vector for the robot (joints) configurations

        if (!ssSetNumInputPorts (S, 2)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }
        bool success = true;
        success = success && ssSetInputPortMatrixDimensions(S,  0, 4, 4);
        success = success && ssSetInputPortVectorDimension(S, 1, dofs);

        ssSetInputPortDataType (S, 0, SS_DOUBLE);
        ssSetInputPortDataType (S, 1, SS_DOUBLE);

        ssSetInputPortDirectFeedThrough (S, 0, 1);
        ssSetInputPortDirectFeedThrough (S, 1, 1);

        if (!success) {
            if (error) error->message = "Failed to configure input ports";
            return false;
        }

        // Output port:
        // - (DoFs + 6)x(DoFs + 6) matrix representing the mass matrix
        if (!ssSetNumOutputPorts (S, 1)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        success = ssSetOutputPortMatrixDimensions(S, 0, dofs + 6, dofs + 6);
        ssSetOutputPortDataType (S, 0, SS_DOUBLE);

        return success;
    }

    bool MassMatrix::initialize(SimStruct *S, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIBlock::initialize(S, error)) return false;

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        m_basePose = new double[16];
        m_massMatrix = new double[(6 + dofs)*(6 + dofs)];
        m_basePoseRaw = new double[16];
        m_configuration = new double[dofs];

        return m_basePose && m_massMatrix && m_basePoseRaw && m_configuration;
    }

    bool MassMatrix::terminate(SimStruct *S, wbt::Error *error)
    {
        if (m_basePose) {
            delete [] m_basePose;
            m_basePose = 0;
        }
        if (m_massMatrix) {
            delete [] m_massMatrix;
            m_massMatrix = 0;
        }
        if (m_basePoseRaw) {
            delete [] m_basePoseRaw;
            m_basePoseRaw = 0;
        }
        if (m_configuration) {
            delete [] m_configuration;
            m_configuration = 0;
        }
        return WBIBlock::terminate(S, error);
    }

    bool MassMatrix::output(SimStruct *S, wbt::Error */*error*/)
    {
        //get input
        std::shared_ptr<wbi::iWholeBodyModel> interface = WBInterface::sharedInstance().model().lock();
        if (interface) {
            InputRealPtrsType basePoseRaw = ssGetInputPortRealSignalPtrs(S, 0);
            InputRealPtrsType configuration = ssGetInputPortRealSignalPtrs(S, 1);
            for (unsigned i = 0; i < ssGetInputPortWidth(S, 0); ++i) {
                m_basePoseRaw[i] = *basePoseRaw[i];
            }
            for (unsigned i = 0; i < ssGetInputPortWidth(S, 1); ++i) {
                m_configuration[i] = *configuration[i];
            }

            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor> > basePoseColMajor(m_basePoseRaw);
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > basePose(m_basePose);
            basePose = basePoseColMajor;

            wbi::Frame frame;
            wbi::frameFromSerialization(basePose.data(), frame);

            interface->computeMassMatrix(m_configuration, frame, m_massMatrix);
            unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > massMatrixRowMajor(m_massMatrix, 6 + dofs, 6 + dofs);

            real_T *output = ssGetOutputPortRealSignal(S, 0);
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> > massMatrixColMajor(output, 6 + dofs, 6 + dofs);
            massMatrixColMajor = massMatrixRowMajor;
            return true;
        }
        return false;
    }
}
