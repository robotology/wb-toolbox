/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_PARAMETER_H
#define WBT_PARAMETER_H

#include <string>
#include <vector>

namespace wbt {
    class ParameterMetadata;
    template <typename T>
    class Parameter;
    enum class ParameterType;
} // namespace wbt

enum class wbt::ParameterType
{
    // Scalar / Vector / Matrix
    INT,
    BOOL,
    DOUBLE,
    STRING,
    // Cell
    CELL_INT,
    CELL_BOOL,
    CELL_DOUBLE,
    CELL_STRING,
    // Struct
    STRUCT_INT,
    STRUCT_BOOL,
    STRUCT_DOUBLE,
    STRUCT_STRING,
    STRUCT_CELL_INT,
    STRUCT_CELL_BOOL,
    STRUCT_CELL_DOUBLE,
    STRUCT_CELL_STRING
};
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
    using ParamVector = std::vector<T>;

    bool m_isScalar;
    T m_valueScalar;
    ParamVector m_valueVector;
    wbt::ParameterMetadata m_metadata;

public:
    enum class Type;
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
