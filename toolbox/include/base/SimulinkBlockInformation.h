#ifndef WBT_SIMULINKBLOCKINFORMATION_H
#define WBT_SIMULINKBLOCKINFORMATION_H

#include "AnyType.h"
#include "BlockInformation.h"
#include "simstruc.h"

namespace wbt {
    class SimulinkBlockInformation;
    class Signal;
} // namespace wbt

class wbt::SimulinkBlockInformation : public wbt::BlockInformation
{
private:
    SimStruct* simstruct;

    PortDataType mapSimulinkToPortType(const DTypeId& typeId) const;
    DTypeId mapPortTypeToSimulink(const PortDataType& dataType) const;

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

    bool setNumberOfInputPorts(unsigned numberOfPorts) override;
    bool setNumberOfOutputPorts(unsigned numberOfPorts) override;
    bool setInputPortVectorSize(unsigned portNumber, int portSize) override;
    bool setInputPortMatrixSize(unsigned portNumber, int rows, int columns) override;
    bool setOutputPortVectorSize(unsigned portNumber, int portSize) override;
    bool setOutputPortMatrixSize(unsigned portNumber, int rows, int columns) override;
    bool setInputPortType(unsigned portNumber, PortDataType portType) override;
    bool setOutputPortType(unsigned portNumber, PortDataType portType) override;

    // PORT INFORMATION GETTERS
    // ========================

    unsigned getInputPortWidth(unsigned portNumber) const override;
    unsigned getOutputPortWidth(unsigned portNumber) const override;
    wbt::Signal getInputPortSignal(unsigned portNumber,
                                   int portWidth = DYNAMICALLY_SIZED) const override;
    wbt::Signal getOutputPortSignal(unsigned portNumber,
                                    int portWidth = DYNAMICALLY_SIZED) const override;
};

#endif /* end of include guard: WBT_SIMULINKBLOCKINFORMATION_H */
