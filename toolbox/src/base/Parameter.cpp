#include "Parameter.h"

using namespace wbt;

const int ParameterMetadata::DynamicSize = -1;
const std::string defaultParamPrefix = "ParamIndex_";

ParameterMetadata::ParameterMetadata(const ParameterType& t,
                                     const unsigned& paramIndex,
                                     const int& paramRows,
                                     const int& paramCols,
                                     const std::string& paramName)
    : m_index(paramIndex)
    , m_name(paramName.empty() ? (defaultParamPrefix + std::to_string(m_index)) : paramName)
    , m_rows(paramRows)
    , m_cols(paramCols)
    , m_type(t)
{}

ParameterMetadata::ParameterMetadata(const ParameterMetadata& paramMD)
    : m_index(paramMD.m_index)
    , m_name(paramMD.m_name.empty() ? (defaultParamPrefix + std::to_string(m_index))
                                    : paramMD.m_name)
    , m_rows(paramMD.m_rows)
    , m_cols(paramMD.m_cols)
    , m_type(paramMD.m_type)
{}

ParameterMetadata::ParameterMetadata(ParameterMetadata&& other)
    : m_index(other.m_index)
    , m_name(other.m_name.empty() ? (defaultParamPrefix + std::to_string(m_index)) : other.m_name)
    , m_rows(other.m_rows)
    , m_cols(other.m_cols)
    , m_type(other.m_type)
{}

ParameterMetadata& ParameterMetadata::operator=(const ParameterMetadata& other)
{
    *this =
        ParameterMetadata(other.m_type, other.m_index, other.m_rows, other.m_cols, other.m_name);
    return *this;
}

ParameterMetadata& ParameterMetadata::operator=(ParameterMetadata&& other)
{
    ParameterMetadata paramMD(
        other.m_type, other.m_index, other.m_rows, other.m_cols, other.m_name);
    *this = paramMD;
    return *this;
}

bool ParameterMetadata::operator==(const ParameterMetadata& rhs)
{
    bool ok = true;
    ok = ok && (this->m_index == rhs.m_index);
    ok = ok && (this->m_name == rhs.m_name);
    ok = ok && (this->m_rows == rhs.m_rows);
    ok = ok && (this->m_cols == rhs.m_cols);
    ok = ok && (this->m_type == rhs.m_type);
    return ok;
}
