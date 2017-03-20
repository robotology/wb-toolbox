#include "MassMatrix.h"

#include "BlockInformation.h"
#include "Signal.h"
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

    bool MassMatrix::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (!WBIBlock::configureSizeAndPorts(blockInfo, error)) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        // Specify I/O
        // Input ports:
        // - 4x4 matrix (homogenous transformation for the base pose w.r.t. world)
        // - DoFs vector for the robot (joints) configurations

        if (!blockInfo->setNumberOfInputPorts(2)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }
        bool success = true;
        success = success && blockInfo->setInputPortMatrixSize(0, 4, 4);
        success = success && blockInfo->setInputPortVectorSize(1, dofs);
        blockInfo->setInputPortType(0, PortDataTypeDouble);
        blockInfo->setInputPortType(1, PortDataTypeDouble);


        if (!success) {
            if (error) error->message = "Failed to configure input ports";
            return false;
        }

        // Output port:
        // - (DoFs + 6)x(DoFs + 6) matrix representing the mass matrix
        if (!blockInfo->setNumberOfOuputPorts(1)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        success = blockInfo->setOutputPortMatrixSize(0, dofs + 6, dofs + 6);
        blockInfo->setOutputPortType(0, PortDataTypeDouble);

        return success;
    }

    bool MassMatrix::initialize(BlockInformation *blockInfo, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIModelBlock::initialize(blockInfo, error)) return false;

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        m_basePose = new double[16];
        m_massMatrix = new double[(6 + dofs)*(6 + dofs)];
        m_basePoseRaw = new double[16];
        m_configuration = new double[dofs];

        return m_basePose && m_massMatrix && m_basePoseRaw && m_configuration;
    }

    bool MassMatrix::terminate(BlockInformation *blockInfo, wbt::Error *error)
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
        return WBIModelBlock::terminate(blockInfo, error);
    }

    bool MassMatrix::output(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        //get input
        wbi::iWholeBodyModel * const interface = WBInterface::sharedInstance().model();
        if (interface) {
            Signal basePoseRaw = blockInfo->getInputPortSignal(0);
            Signal configuration = blockInfo->getInputPortSignal(1);

            for (unsigned i = 0; i < blockInfo->getInputPortWidth(0); ++i) {
                m_basePoseRaw[i] = basePoseRaw.get(i).doubleData();
            }
            for (unsigned i = 0; i < blockInfo->getInputPortWidth(1); ++i) {
                m_configuration[i] = configuration.get(i).doubleData();
            }

            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor> > basePoseColMajor(m_basePoseRaw);
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > basePose(m_basePose);
            basePose = basePoseColMajor;

            wbi::Frame frame;
            wbi::frameFromSerialization(basePose.data(), frame);

            interface->computeMassMatrix(m_configuration, frame, m_massMatrix);
            unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > massMatrixRowMajor(m_massMatrix, 6 + dofs, 6 + dofs);

            Signal output = blockInfo->getOutputPortSignal(0);
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> > massMatrixColMajor((double*)output.getContiguousBuffer(), 6 + dofs, 6 + dofs);
            massMatrixColMajor = massMatrixRowMajor;
            return true;
        }
        return false;
    }
}
