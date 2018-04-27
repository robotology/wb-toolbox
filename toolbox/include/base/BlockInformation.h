/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_BLOCKINFORMATION_H
#define WBT_BLOCKINFORMATION_H

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace wbt {
    class BlockInformation;
    class Signal;
    class ParameterMetadata;
    class Parameters;
    class Configuration;
    class RobotInterface;
    enum class DataType;
    extern const std::string BlockOptionPrioritizeOrder;
} // namespace wbt

namespace iDynTree {
    class KinDynComputations;
}

class wbt::BlockInformation
{
public:
    using Rows = int;
    using Cols = int;
    using PortIndex = int;
    using VectorSize = int;
    using MatrixSize = std::pair<Rows, Cols>;

    BlockInformation() = default;
    virtual ~BlockInformation() = default;

    // BLOCK OPTIONS METHODS
    // =====================

    /**
     * Convert a block option from its Toolbox identifier to a specific implementation
     *
     * @param [in]  key    identifier of the block option
     * @param [out] option implementation-specific block option
     * @return             true if the option has been converted. False otherwise
     */
    virtual bool optionFromKey(const std::string& key, double& option) const;

    // PARAMETERS METHODS
    // ==================

    virtual bool parseParameters(wbt::Parameters& /*parameters*/) { return true; }
    virtual bool addParameterMetadata(const wbt::ParameterMetadata& /*paramMD*/) { return true; }

    // PORT INFORMATION SETTERS
    // ========================

    virtual bool setNumberOfInputPorts(const unsigned& numberOfPorts) = 0;
    virtual bool setNumberOfOutputPorts(const unsigned& numberOfPorts) = 0;

    /**
     * Set data type for the specified input port
    virtual bool setInputPortVectorSize(const PortIndex& idx, const VectorSize& size) = 0;
     *
     * @param portNumber number of input port
     * @param portType   data type
    virtual bool setOutputPortVectorSize(const PortIndex& idx, const VectorSize& size) = 0;
     *
     * @return true if succeded, false otherwise
     */
    virtual bool setInputPortMatrixSize(const PortIndex& idx, const MatrixSize& size) = 0;

    virtual bool setOutputPortMatrixSize(const PortIndex& idx, const MatrixSize& size) = 0;
    virtual bool setInputPortType(const PortIndex& idx, const DataType& type) = 0;
    virtual bool setOutputPortType(const PortIndex& idx, const DataType& type) = 0;
    // PORT INFORMATION GETTERS
    // ========================

    virtual unsigned getInputPortWidth(const PortIndex& idx) const = 0;

    virtual unsigned getOutputPortWidth(const PortIndex& idx) const = 0;
    virtual MatrixSize getInputPortMatrixSize(const PortIndex& idx) const = 0;
    virtual MatrixSize getOutputPortMatrixSize(const PortIndex& idx) const = 0;
    virtual wbt::Signal getInputPortSignal(const PortIndex& idx,
                                           const VectorSize& size = -1) const = 0;
    virtual wbt::Signal getOutputPortSignal(const PortIndex& idx,
                                            const VectorSize& size = -1) const = 0;

    virtual std::weak_ptr<wbt::RobotInterface> getRobotInterface() const = 0;
    virtual std::weak_ptr<iDynTree::KinDynComputations> getKinDynComputations() const = 0;
};

#endif // WBT_BLOCKINFORMATION_H
