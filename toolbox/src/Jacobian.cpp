#include "Jacobian.h"

#include "Error.h"
#include "WBInterface.h"
#include "BlockInformation.h"
#include "Signal.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>
#include <Eigen/Core>

namespace wbt {

    std::string Jacobian::ClassName = "Jacobian";

    Jacobian::Jacobian()
    : m_basePose(0)
    , m_jacobian(0)
    , m_basePoseRaw(0)
    , m_configuration(0)
    , m_frameIndex(-1) {}

    unsigned Jacobian::numberOfParameters()
    {
        return WBIBlock::numberOfParameters() + 1;
    }

    bool Jacobian::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
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
        // - (6)x(6+dofs) matrix
        if (!blockInfo->setNumberOfOuputPorts(1)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        success = blockInfo->setOutputPortMatrixSize(0, 6, 6 + dofs);
        blockInfo->setOutputPortType(0, PortDataTypeDouble);

        return success;
    }

    bool Jacobian::initialize(BlockInformation *blockInfo, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIModelBlock::initialize(blockInfo, error)) return false;

        int parentParameters = WBIBlock::numberOfParameters() + 1;
        //robot name
        std::string frame;
        if (!blockInfo->getStringParameterAtIndex(parentParameters, frame)) {
            if (error) error->message = "Cannot retrieve string from frame parameter";
            return false;
        }

        //here obtain joint list and get the frame
        wbi::iWholeBodyModel * const interface = WBInterface::sharedInstance().model();
        if (!interface) {
            if (error) error->message = "Cannot retrieve handle to WBI interface";
            return false;
        }
        wbi::IDList frames =  interface->getFrameList();
        if (frame != "com") {
            if (!frames.idToIndex(wbi::ID(frame), m_frameIndex)) {
                if (error) error->message = "Cannot find " + frame + " in the frame list";
                return false;
            }
        } else {
            m_frameIndex = wbi::wholeBodyInterface::COM_LINK_ID;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        m_basePose = new double[16];
        m_jacobian = new double[6 * (6 + dofs)];
        m_basePoseRaw = new double[16];
        m_configuration = new double[dofs];

        return m_basePose && m_jacobian && m_basePoseRaw && m_configuration;
    }

    bool Jacobian::terminate(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (m_basePose) {
            delete [] m_basePose;
            m_basePose = 0;
        }
        if (m_jacobian) {
            delete [] m_jacobian;
            m_jacobian = 0;
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

    bool Jacobian::output(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
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

            unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
            interface->computeJacobian(m_configuration, frame, m_frameIndex, m_jacobian);

            Eigen::Map<Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor> > jacobianRowMajor(m_jacobian, 6, 6 + dofs);

            Signal output = blockInfo->getOutputPortSignal(0);
            Eigen::Map<Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor> > jacobianColMajor((double*)output.getContiguousBuffer(), 6, 6 + dofs);

            jacobianColMajor = jacobianRowMajor;
            return true;
        }
        return false;
    }
}
