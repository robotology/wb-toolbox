#ifndef WBT_SIMULINKBLOCKINFORMATION_H
#define WBT_SIMULINKBLOCKINFORMATION_H

#include "AnyType.h"
#include "BlockInformation.h"
#include "simstruc.h"
#include "Signal.h"

namespace wbt {
    class SimulinkBlockInformation;
} // namespace wbt

class wbt::SimulinkBlockInformation : public wbt::BlockInformation
{
private:
    SimStruct* simstruct;

    DataType mapSimulinkToPortType(const DTypeId& typeId) const;
    DTypeId mapPortTypeToSimulink(const DataType& dataType) const;

public:
    SimulinkBlockInformation(SimStruct* simstruct);
    ~SimulinkBlockInformation() override = default;

    // BLOCK OPTIONS METHODS
    // =====================

    bool optionFromKey(const std::string& key, double& option) const override;

    // PARAMETERS METHODS
    // ==================

    bool getStringParameterAtIndex(unsigned parameterIndex,
                                   std::string& stringParameter) const override;
    bool getScalarParameterAtIndex(unsigned parameterIndex, double& value) const override;
    bool getBooleanParameterAtIndex(unsigned parameterIndex, bool& value) const override;
    bool getStructAtIndex(unsigned parameterIndex, AnyStruct& map) const override;
    bool getVectorAtIndex(unsigned parameterIndex, std::vector<double>& vec) const override;

    // PORT INFORMATION SETTERS
    // ========================

    bool setNumberOfInputPorts(const unsigned& numberOfPorts) override;
    bool setNumberOfOutputPorts(const unsigned& numberOfPorts) override;
    bool setInputPortVectorSize(const SignalIndex& idx, const VectorSize& size) override;
    bool setInputPortMatrixSize(const SignalIndex& idx, const MatrixSize& size) override;
    bool setOutputPortVectorSize(const SignalIndex& idx, const VectorSize& size) override;
    bool setOutputPortMatrixSize(const SignalIndex& idx, const MatrixSize& size) override;
    bool setInputPortType(const SignalIndex& idx, const DataType& portType) override;
    bool setOutputPortType(const SignalIndex& idx, const DataType& portType) override;

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
};

#endif /* end of include guard: WBT_SIMULINKBLOCKINFORMATION_H */
