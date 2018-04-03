#ifndef WBT_PARAMETER_H
#define WBT_PARAMETER_H

#include <string>
#include <vector>

namespace wbt {
    class ParameterMetadata;
    template <typename T>
    class Parameter;
    class Parameters;
    enum ParameterType
    {
        // Scalar / Vector / Matrix
        PARAM_INT,
        PARAM_BOOL,
        PARAM_DOUBLE,
        PARAM_STRING,
        // Cell
        PARAM_CELL_INT,
        PARAM_CELL_BOOL,
        PARAM_CELL_DOUBLE,
        PARAM_CELL_STRING,
        // Struct
        PARAM_STRUCT_INT,
        PARAM_STRUCT_BOOL,
        PARAM_STRUCT_DOUBLE,
        PARAM_STRUCT_STRING,
        PARAM_STRUCT_CELL_INT,
        PARAM_STRUCT_CELL_BOOL,
        PARAM_STRUCT_CELL_DOUBLE,
        PARAM_STRUCT_CELL_STRING
    };
} // namespace wbt

class wbt::ParameterMetadata
{
public:
    static const int DynamicSize;

    const unsigned m_index;
    const std::string m_name;

    int m_rows;
    int m_cols;
    wbt::ParameterType m_type;

    ParameterMetadata() = delete;
    ~ParameterMetadata() = default;

    ParameterMetadata(const ParameterType& t,
                      const unsigned& ParamIndex,
                      const int& paramRows,
                      const int& paramCols,
                      const std::string& ParamName = {});
    ParameterMetadata(const ParameterMetadata& paramMD);
    ParameterMetadata(ParameterMetadata&& paramMD);

    ParameterMetadata& operator=(const ParameterMetadata& paramMD);
    ParameterMetadata& operator=(ParameterMetadata&& paramMD);
    bool operator==(const ParameterMetadata& rhs);
    inline bool operator!=(const ParameterMetadata& rhs) { return !(*this == rhs); }
};

template <typename T>
class wbt::Parameter
{
private:
    typedef std::vector<T> ParamVector;

    bool m_isScalar;
    T m_valueScalar;
    ParamVector m_valueVector;
    wbt::ParameterMetadata m_metadata;

public:
    Parameter() = delete;
    ~Parameter() = default;

    Parameter(const T& value, const wbt::ParameterMetadata& md)
        : m_isScalar(true)
        , m_valueScalar(value)
        , m_metadata(md)
    {}
    Parameter(const ParamVector& valueVec, const wbt::ParameterMetadata& md)
        : m_valueVector(valueVec)
        , m_isScalar(false)
        , m_metadata(md)
    {}

    bool isScalar() const { return m_isScalar; }
    T getScalarParameter() const { return m_valueScalar; }
    ParamVector getVectorParameter() const { return m_valueVector; }
    wbt::ParameterMetadata getMetadata() const { return m_metadata; }
};

#endif // WBT_PARAMETER_H
