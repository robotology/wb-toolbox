/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_BLOCKINFORMATION_H
#define WBT_BLOCKINFORMATION_H

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
    // List of possible key for defining block options:
    extern const std::string BlockOptionPrioritizeOrder;
} // namespace wbt

namespace iDynTree {
    class KinDynComputations;
}

/**
 * @brief Abstract class for storing generic Block properties
 *
 * BlockInformation provides an interface for handling implementation-specific properties such as
 * input / output number, size and type, number of parameters, ...
 *
 * A wbt::Block needs to know on what kind of data it operates, and retrieving this information is
 * often specific on the framework on top of which block runs. In order to allow using the same
 * Block class from different frameworks (e.g. Simulink, C++, etc), different implementation of this
 * interface can provide a transparent translation on functionalities.
 *
 * As an example, take the BlockInformation::parseParameters. In Simulink parameters are read from
 * block's masks and Matlab provides a library for reading them. The SimulinkBlockInformation
 * implementation will be linked against that library. However, if you want to call the same Block
 * class (which is just a wrapper of an algorithm) from a pure C++ main, parameters are read e.g.
 * from an xml file. In this case, BlockInformation::parseParameters will parse the xml and fill the
 * wbt::Parameters argument.
 *
 * @see wbt::Block, wbt::Parameters, wbt::Signal
 */
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

    // =====================
    // BLOCK OPTIONS METHODS
    // =====================

    /**
     * @brief Convert a block option from its string identifier to a specific implementation
     *
     * @param[in]  key Identifier of the block option.
     * @param[out] option Implementation-specific block option.
     * @return True if the option has been converted, false otherwise.
     */
    virtual bool optionFromKey(const std::string& key, double& option) const;

    // ==================
    // PARAMETERS METHODS
    // ==================

    /**
     * @brief Parse the wbt::Block's parameters
     *
     * This method allows defining how to gather block's parameters from a specific implementation.
     *
     * @param[out] parameters A container filled with the parsed parameters.
     * @return True for success, false otherwise.
     */
    virtual bool parseParameters(wbt::Parameters& parameters) { return true; }

    /**
     * @brief Add a parameter metadata
     *
     * In order to parse parameters with BlockInformation::parseParameters, adding in advance their
     * metadata can strongly simplify the entire process.
     *
     * @param paramMD The metadata to add.
     * @return True for success, false otherwise.
     */
    virtual bool addParameterMetadata(const wbt::ParameterMetadata& paramMD) { return true; }

    // ========================
    // PORT INFORMATION SETTERS
    // ========================

    /**
     * @brief Set the number of block's inputs
     *
     * @param numberOfPorts Number of input ports.
     * @return True for success, false otherwise.
     */
    virtual bool setNumberOfInputPorts(const unsigned& numberOfPorts) = 0;

    /**
     * @brief Set the number of block's outputs
     *
     * @param numberOfPorts Number of output ports.
     * @return True for success, false otherwise.
     */
    virtual bool setNumberOfOutputPorts(const unsigned& numberOfPorts) = 0;

    /**
     * @brief Set the size of a 1D input port
     *
     * @param idx The index of the port.
     * @param size The size of the port.
     * @return True for success, false otherwise.
     */
    virtual bool setInputPortVectorSize(const PortIndex& idx, const VectorSize& size) = 0;

    /**
     * @brief Set the size of a 1D output port
     *
     * @param idx The index of the port.
     * @param size The size of the port.
     * @return True for success, false otherwise.
     */
    virtual bool setOutputPortVectorSize(const PortIndex& idx, const VectorSize& size) = 0;

    /**
     * @brief Set the size of a 2D input port
     *
     * @param idx The index of the port.
     * @param size The size of the port.
     * @return True for success, false otherwise.
     */
    virtual bool setInputPortMatrixSize(const PortIndex& idx, const MatrixSize& size) = 0;

    /**
     * @brief Set the size of a 2D output port
     *
     * @param idx The index of the port.
     * @param size The size of the port.
     * @return True for success, false otherwise.
     */
    virtual bool setOutputPortMatrixSize(const PortIndex& idx, const MatrixSize& size) = 0;

    /**
     * @brief Set the data type of an input port
     *
     * @param idx The index of the port.
     * @param type The type of the port.
     * @return True for success, false otherwise.
     */
    virtual bool setInputPortType(const PortIndex& idx, const DataType& type) = 0;

    /**
     * @brief Set the data type of an output port
     *
     * @param idx The index of the port.
     * @param type The type of the port.
     * @return True for success, false otherwise.
     */
    virtual bool setOutputPortType(const PortIndex& idx, const DataType& type) = 0;

    // ========================
    // PORT INFORMATION GETTERS
    // ========================

    /**
     * @brief Get the size of a 1D input port
     *
     * @param idx The index of the port.
     * @return The size of the port.
     */
    virtual unsigned getInputPortWidth(const PortIndex& idx) const = 0;

    /**
     * @brief Get the size of a 1D output port
     *
     * @param idx The index of the port.
     * @return The size of the port.
     */
    virtual unsigned getOutputPortWidth(const PortIndex& idx) const = 0;

    /**
     * @brief Get the size of a 2D input port
     *
     * @param idx The index of the port.
     * @return The size of the port.
     */
    virtual MatrixSize getInputPortMatrixSize(const PortIndex& idx) const = 0;

    /**
     * @brief Get the size of a 2D output port
     *
     * @param idx The index of the port.
     * @return The size of the port.
     */
    virtual MatrixSize getOutputPortMatrixSize(const PortIndex& idx) const = 0;

    // =============
    // BLOCK SIGNALS
    // =============

    /**
     * @brief Get the signal connected to a 1D input port
     *
     * @param idx The index of the port.
     * @param size The size of the signal.
     * @return The signal connected to the input port for success, an invalid signal otherwise.
     * @see Signal::isValid
     */
    virtual wbt::Signal getInputPortSignal(const PortIndex& idx,
                                           const VectorSize& size = -1) const = 0;

    /**
     * @brief Get the signal connected to a 1D output port
     *
     * @param idx The index of the port.
     * @param size The size of the signal.
     * @return The signal connected to the output port for success, an invalid signal otherwise.
     * @see Signal::isValid
     */
    virtual wbt::Signal getOutputPortSignal(const PortIndex& idx,
                                            const VectorSize& size = -1) const = 0;

    // ==========================
    // EXTERNAL LIBRARIES METHODS
    // ==========================

    /**
     * @brief Get the wbt::RobotInterface object associated to the Block
     * @return The pointer to the wbt::RobotInterface object
     */
    virtual std::weak_ptr<wbt::RobotInterface> getRobotInterface() const = 0;

    /**
     * @brief Get the iDynTree::KinDynComputations object associated to the Block
     * @return The pointer to the iDynTree::KinDynComputations object
     */
    virtual std::weak_ptr<iDynTree::KinDynComputations> getKinDynComputations() const = 0;
};

#endif // WBT_BLOCKINFORMATION_H
