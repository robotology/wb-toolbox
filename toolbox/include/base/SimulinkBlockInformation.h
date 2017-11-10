#ifndef WBT_SIMULINKBLOCKINFORMATION_H
#define WBT_SIMULINKBLOCKINFORMATION_H

#include "BlockInformation.h"
#include "simstruc.h"
#include "AnyType.h"

namespace wbt {
    class SimulinkBlockInformation;
    class Signal;
}

class wbt::SimulinkBlockInformation : public wbt::BlockInformation
{
private:
    SimStruct* simstruct;

public:
    SimulinkBlockInformation(SimStruct* simstruct);
    ~SimulinkBlockInformation() override = default;

    bool optionFromKey(const std::string& key, double& option) const override;

    //Parameters methods
    bool getStringParameterAtIndex(unsigned parameterIndex, std::string& stringParameter) const override;
    bool getScalarParameterAtIndex(unsigned parameterIndex, double& value) const override;
    bool getBooleanParameterAtIndex(unsigned parameterIndex, bool& value) const override;
    bool getStructAtIndex(unsigned parameterIndex, AnyStruct& map) const override;
    bool getVectorAtIndex(unsigned parameterIndex, std::vector<double>& vec) const override;

    //Port information methods
    bool setNumberOfInputPorts(unsigned numberOfPorts) override;
    bool setNumberOfOutputPorts(unsigned numberOfPorts) override;
    bool setInputPortVectorSize(unsigned portNumber, int portSize) override;
    bool setInputPortMatrixSize(unsigned portNumber, int rows, int columns) override;
    bool setOutputPortVectorSize(unsigned portNumber, int portSize) override;
    bool setOutputPortMatrixSize(unsigned portNumber, int rows, int columns) override;
    bool setInputPortType(unsigned portNumber, PortDataType portType) override;
    bool setOutputPortType(unsigned portNumber, PortDataType portType) override;

    //Port data
    unsigned getInputPortWidth(unsigned portNumber) const override;
    unsigned getOutputPortWidth(unsigned portNumber) const override;
    wbt::Signal getInputPortSignal(unsigned portNumber) const override;
    wbt::Signal getOutputPortSignal(unsigned portNumber) const override;
};

#endif /* end of include guard: WBT_SIMULINKBLOCKINFORMATION_H */
