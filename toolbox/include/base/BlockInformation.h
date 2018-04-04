#ifndef WBT_BLOCKINFORMATION_H
#define WBT_BLOCKINFORMATION_H

#include <string>
#include <utility>

namespace wbt {
    class BlockInformation;
    class Signal;
    class ParameterMetadata;
    class Parameters;
    enum class DataType;
    extern const std::string BlockOptionPrioritizeOrder;
} // namespace wbt

enum class wbt::DataType
{
    DOUBLE,
    SINGLE,
    INT8,
    UINT8,
    INT16,
    UINT16,
    INT32,
    UINT32,
    BOOLEAN,
};

class wbt::BlockInformation
{
public:
    typedef int Rows;
    typedef int Cols;
    typedef int SignalIndex;
    typedef int VectorSize;
    typedef std::pair<Rows, Cols> MatrixSize;

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
    virtual bool setInputPortVectorSize(const SignalIndex& idx, const VectorSize& size) = 0;
    virtual bool setInputPortMatrixSize(const SignalIndex& idx, const MatrixSize& size) = 0;
    virtual bool setOutputPortVectorSize(const SignalIndex& idx, const VectorSize& size) = 0;
    virtual bool setOutputPortMatrixSize(const SignalIndex& idx, const MatrixSize& size) = 0;

    /**
     * Set data type for the specified input port
     *
     * @param portNumber number of input port
     * @param portType   data type
     *
     * @return true if succeded, false otherwise
     */
    virtual bool setInputPortType(const SignalIndex& idx, const DataType& type) = 0;
    virtual bool setOutputPortType(const SignalIndex& idx, const DataType& type) = 0;

    // PORT INFORMATION GETTERS
    // ========================

    virtual unsigned getInputPortWidth(const SignalIndex& idx) const = 0;
    virtual unsigned getOutputPortWidth(const SignalIndex& idx) const = 0;
    virtual MatrixSize getInputPortMatrixSize(const SignalIndex& port) const = 0;
    virtual MatrixSize getOutputPortMatrixSize(const SignalIndex& port) const = 0;

    virtual wbt::Signal getInputPortSignal(const SignalIndex& idx,
                                           const VectorSize& size = -1) const = 0;
    virtual wbt::Signal getOutputPortSignal(const SignalIndex& idx,
                                            const VectorSize& size = -1) const = 0;

    virtual std::weak_ptr<wbt::RobotInterface> getRobotInterface() const = 0;
    virtual std::weak_ptr<iDynTree::KinDynComputations> getKinDynComputations() const = 0;
};

#endif /* end of include guard: WBT_BLOCKINFORMATION_H */
