#ifndef WBT_CODERBLOCKINFORMATION_H
#define WBT_CODERBLOCKINFORMATION_H

#include "BlockInformation.h"
#include "Parameters.h"
#include "Signal.h"

#include <unordered_map>
#include <vector>

namespace wbt {
    class CoderBlockInformation;
}

class wbt::CoderBlockInformation : public wbt::BlockInformation
{
private:
    unsigned m_numberOfInputs;
    unsigned m_numberOfOutputs;

    std::vector<wbt::ParameterMetadata> m_paramsMetadata;
    std::unordered_map<SignalIndex, wbt::Signal> m_inputSignals;
    std::unordered_map<SignalIndex, wbt::Signal> m_outputSignals;

    std::string m_confBlockName;
    Parameters m_parametersFromRTW;

    std::unordered_map<SignalIndex, std::pair<Rows, Cols>> m_inputSignalSize;
    std::unordered_map<SignalIndex, std::pair<Rows, Cols>> m_outputSignalSize;

public:
    CoderBlockInformation() = default;
    ~CoderBlockInformation() override = default;

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
    bool setInputPortVectorSize(const SignalIndex& idx, const VectorSize& size) override;
    bool setInputPortMatrixSize(const SignalIndex& idx, const MatrixSize& size) override;
    bool setOutputPortVectorSize(const SignalIndex& idx, const VectorSize& size) override;
    bool setOutputPortMatrixSize(const SignalIndex& idx, const MatrixSize& size) override;
    bool setInputPortType(const SignalIndex& idx, const DataType& type) override;
    bool setOutputPortType(const SignalIndex& idx, const DataType& type) override;

    // PORT INFORMATION GETTERS
    // ========================

    unsigned getInputPortWidth(const SignalIndex& idx) const override;
    unsigned getOutputPortWidth(const SignalIndex& idx) const override;
    wbt::Signal
    getInputPortSignal(const SignalIndex& idx,
                       const VectorSize& size = wbt::Signal::DynamicSize) const override;
    wbt::Signal
    getOutputPortSignal(const SignalIndex& idx,
                        const VectorSize& size = wbt::Signal::DynamicSize) const override;
    MatrixSize getInputPortMatrixSize(const SignalIndex& idx) const override;
    MatrixSize getOutputPortMatrixSize(const SignalIndex& idx) const override;

    std::weak_ptr<wbt::RobotInterface> getRobotInterface() const override;
    std::weak_ptr<iDynTree::KinDynComputations> getKinDynComputations() const override;

    // METHODS OUTSIDE THE INTERFACE
    // =============================

    bool storeRTWParameters(const Parameters& parameters);
    bool setInputSignal(const SignalIndex& idx, void* address, const MatrixSize& size);
    bool setOutputSignal(const SignalIndex& idx, void* address, const MatrixSize& size);
};

#endif // WBT_CODERBLOCKINFORMATION_H
