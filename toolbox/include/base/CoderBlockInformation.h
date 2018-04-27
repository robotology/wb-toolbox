/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_CODERBLOCKINFORMATION_H
#define WBT_CODERBLOCKINFORMATION_H

#include "BlockInformation.h"
#include "Signal.h"

#include <unordered_map>
#include <vector>

namespace wbt {
    class CoderBlockInformation;
}

class wbt::CoderBlockInformation final : public wbt::BlockInformation
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    CoderBlockInformation();
    ~CoderBlockInformation() override;

    // BLOCK OPTIONS METHODS
    // =====================

    bool optionFromKey(const std::string& key, double& option) const override;

    // PARAMETERS METHODS
    // ==================

    bool addParameterMetadata(const wbt::ParameterMetadata& paramMD) override;
    bool parseParameters(wbt::Parameters& parameters) override;

    // PORT INFORMATION SETTERS
    // ========================

    bool setNumberOfInputPorts(const unsigned& numberOfPorts) override;
    bool setNumberOfOutputPorts(const unsigned& numberOfPorts) override;
    bool setInputPortVectorSize(const PortIndex& idx, const VectorSize& size) override;
    bool setInputPortMatrixSize(const PortIndex& idx, const MatrixSize& size) override;
    bool setOutputPortVectorSize(const PortIndex& idx, const VectorSize& size) override;
    bool setOutputPortMatrixSize(const PortIndex& idx, const MatrixSize& size) override;
    bool setInputPortType(const PortIndex& idx, const DataType& type) override;
    bool setOutputPortType(const PortIndex& idx, const DataType& type) override;

    // PORT INFORMATION GETTERS
    // ========================

    unsigned getInputPortWidth(const PortIndex& idx) const override;
    unsigned getOutputPortWidth(const PortIndex& idx) const override;
    wbt::Signal
    getInputPortSignal(const PortIndex& idx,
                       const VectorSize& size = wbt::Signal::DynamicSize) const override;
    wbt::Signal
    getOutputPortSignal(const PortIndex& idx,
                        const VectorSize& size = wbt::Signal::DynamicSize) const override;
    MatrixSize getInputPortMatrixSize(const PortIndex& idx) const override;
    MatrixSize getOutputPortMatrixSize(const PortIndex& idx) const override;

    std::weak_ptr<wbt::RobotInterface> getRobotInterface() const override;
    std::weak_ptr<iDynTree::KinDynComputations> getKinDynComputations() const override;

    // METHODS OUTSIDE THE INTERFACE
    // =============================

    bool storeRTWParameters(const Parameters& parameters);
    bool setInputSignal(const PortIndex& idx, void* address, const MatrixSize& size);
    bool setOutputSignal(const PortIndex& idx, void* address, const MatrixSize& size);
};

#endif // WBT_CODERBLOCKINFORMATION_H
