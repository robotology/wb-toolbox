#ifndef WBT_SIMULINKBLOCKINFORMATION_H
#define WBT_SIMULINKBLOCKINFORMATION_H

#include "BlockInformation.h"

#include "simstruc.h"

namespace wbt {
    class SimulinkBlockInformation;
    class Signal;
}

class wbt::SimulinkBlockInformation : public wbt::BlockInformation {
    SimStruct *simstruct;

public:
    SimulinkBlockInformation(SimStruct *simstruct);

    virtual ~SimulinkBlockInformation();

    virtual bool optionFromKey(const std::string& key, Data &option) const;

    //Parameters methods
    virtual bool getStringParameterAtIndex(unsigned parameterIndex, std::string& stringParameter);
    virtual Data getScalarParameterAtIndex(unsigned parameterIndex);

    //Port information methods
    virtual bool setNumberOfInputPorts(unsigned numberOfPorts);
    virtual bool setNumberOfOuputPorts(unsigned numberOfPorts);
    virtual bool setInputPortVectorSize(unsigned portNumber, int portSize);
    virtual bool setInputPortMatrixSize(unsigned portNumber, int rows, int columns);
    virtual bool setOutputPortVectorSize(unsigned portNumber, int portSize);
    virtual bool setOutputPortMatrixSize(unsigned portNumber, int rows, int columns);
    virtual bool setInputPortType(unsigned portNumber, PortDataType portType);
    virtual bool setOutputPortType(unsigned portNumber, PortDataType portType);

    //Port data
    virtual unsigned getInputPortWidth(unsigned portNumber);
    virtual unsigned getOutputPortWidth(unsigned portNumber);
    virtual wbt::Signal getInputPortSignal(unsigned portNumber);
    virtual wbt::Signal getOutputPortSignal(unsigned portNumber);
};

#endif /* end of include guard: WBT_SIMULINKBLOCKINFORMATION_H */
