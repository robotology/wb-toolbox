#include "Parameters.h"
#include "ConvertStdVector.h"
#include <algorithm>
#include <string>

using namespace wbt;

Parameters::ParamName Parameters::getParamName(const Parameters::ParamIndex& index) const
{
    if (m_indexToName.find(index) == m_indexToName.end()) {
        return PARAM_INVALID_NAME;
    }

    return m_indexToName.at(index);
}

Parameters::ParamIndex Parameters::getParamIndex(const Parameters::ParamName& name) const
{
    if (m_nameToIndex.find(name) == m_nameToIndex.end()) {
        return PARAM_INVALID_INDEX;
    }

    return m_nameToIndex.at(name);
}

bool Parameters::existName(const Parameters::ParamName& name) const
{
    if (existName(name, PARAM_INT) || existName(name, PARAM_BOOL) || existName(name, PARAM_DOUBLE)
        || existName(name, PARAM_STRING) || existName(name, PARAM_STRUCT_INT)
        || existName(name, PARAM_STRUCT_BOOL) || existName(name, PARAM_STRUCT_DOUBLE)
        || existName(name, PARAM_STRUCT_STRING)) {
        return true;
    }
    return false;
}

bool Parameters::existName(const Parameters::ParamName& name, const wbt::ParameterType& type) const
{
    switch (type) {
        case PARAM_INT:
        case PARAM_CELL_INT:
        case PARAM_STRUCT_INT:
        case PARAM_STRUCT_CELL_INT:
            if (m_paramsInt.find(name) == m_paramsInt.end()) {
                return false;
            }
            break;
        case PARAM_BOOL:
        case PARAM_CELL_BOOL:
        case PARAM_STRUCT_BOOL:
        case PARAM_STRUCT_CELL_BOOL:
            if (m_paramsBool.find(name) == m_paramsBool.end()) {
                return false;
            }
            break;
        case PARAM_DOUBLE:
        case PARAM_CELL_DOUBLE:
        case PARAM_STRUCT_DOUBLE:
        case PARAM_STRUCT_CELL_DOUBLE:
            if (m_paramsDouble.find(name) == m_paramsDouble.end()) {
                return false;
            }
            break;
        case PARAM_STRING:
        case PARAM_CELL_STRING:
        case PARAM_STRUCT_STRING:
        case PARAM_STRUCT_CELL_STRING:
            if (m_paramsString.find(name) == m_paramsString.end()) {
                return false;
            }
            break;
    }
    return true;
}

bool Parameters::existIndex(const Parameters::ParamIndex& index) const
{
    if (m_indexToName.find(index) == m_indexToName.end()) {
        return false;
    }

    return true;
}

unsigned Parameters::getNumberOfParameters() const
{
    const unsigned numIntParams = m_paramsInt.size();
    const unsigned numBoolParams = m_paramsBool.size();
    const unsigned numDoubleParams = m_paramsDouble.size();
    const unsigned numStringParams = m_paramsString.size();

    return numIntParams + numBoolParams + numDoubleParams + numStringParams;
}

std::vector<Parameter<int>> Parameters::getIntParameters() const
{
    std::vector<Parameter<int>> vectorParams;

    for (auto p : m_paramsInt) {
        vectorParams.push_back(p.second);
    }

    return vectorParams;
}

std::vector<Parameter<bool>> Parameters::getBoolParameters() const
{
    std::vector<Parameter<bool>> vectorParams;

    for (auto p : m_paramsBool) {
        vectorParams.push_back(p.second);
    }

    return vectorParams;
}

std::vector<Parameter<double>> Parameters::getDoubleParameters() const
{
    std::vector<Parameter<double>> vectorParams;

    for (auto p : m_paramsDouble) {
        vectorParams.push_back(p.second);
    }

    return vectorParams;
}

std::vector<Parameter<std::string>> Parameters::getStringParameters() const
{
    std::vector<Parameter<std::string>> vectorParams;

    for (auto p : m_paramsString) {
        vectorParams.push_back(p.second);
    }

    return vectorParams;
}

wbt::ParameterMetadata Parameters::getParameterMetadata(const ParamName& name)
{
    if (!existName(name) || !existName(name, m_nameToType.at(name))) {
        // TODO: here dummy metadata are returned. This can be improved.
        return {PARAM_INT, 0, 0, 0, "dummy"};
    }

    switch (m_nameToType[name]) {
        case PARAM_INT:
        case PARAM_CELL_INT:
        case PARAM_STRUCT_INT:
        case PARAM_STRUCT_CELL_INT:
            return m_paramsInt.at(name).getMetadata();
        case PARAM_BOOL:
        case PARAM_CELL_BOOL:
        case PARAM_STRUCT_BOOL:
        case PARAM_STRUCT_CELL_BOOL:
            return m_paramsBool.at(name).getMetadata();
        case PARAM_DOUBLE:
        case PARAM_CELL_DOUBLE:
        case PARAM_STRUCT_DOUBLE:
        case PARAM_STRUCT_CELL_DOUBLE:
            return m_paramsDouble.at(name).getMetadata();
        case PARAM_STRING:
        case PARAM_CELL_STRING:
        case PARAM_STRUCT_STRING:
        case PARAM_STRUCT_CELL_STRING:
            return m_paramsString.at(name).getMetadata();
    }
}

bool Parameters::containConfigurationData(const wbt::Parameters& parameters)
{
    return parameters.existName("ConfBlockName") && parameters.existName("RobotName")
           && parameters.existName("UrdfFile") && parameters.existName("LocalName")
           && parameters.existName("ControlledJoints") && parameters.existName("ControlBoardsNames")
           && parameters.existName("GravityVector");
}

// =========
// TEMPLATES
// =========

// GETPARAMETER
// ============

// SCALAR
// ------

// Instantiate the declared templates
template bool Parameters::getParameter<int>(const Parameters::ParamName& name, int& param) const;
template bool Parameters::getParameter<bool>(const Parameters::ParamName& name, bool& param) const;
template bool Parameters::getParameter<double>(const Parameters::ParamName& name,
                                               double& param) const;

template <typename T>
bool wbt::Parameters::getParameter(const wbt::Parameters::ParamName& name, T& param) const
{
    if (!existName(name) || !existName(name, m_nameToType.at(name))) {
        return false;
    }

    switch (m_nameToType.at(name)) {
        case PARAM_INT:
        case PARAM_CELL_INT:
        case PARAM_STRUCT_INT:
        case PARAM_STRUCT_CELL_INT:
            if (!m_paramsInt.at(name).isScalar()) {
                return false;
            }
            param = static_cast<T>(m_paramsInt.at(name).getScalarParameter());
            break;
        case PARAM_BOOL:
        case PARAM_CELL_BOOL:
        case PARAM_STRUCT_BOOL:
        case PARAM_STRUCT_CELL_BOOL:
            if (!m_paramsBool.at(name).isScalar()) {
                return false;
            }
            param = static_cast<T>(m_paramsBool.at(name).getScalarParameter());
            break;
        case PARAM_DOUBLE:
        case PARAM_CELL_DOUBLE:
        case PARAM_STRUCT_DOUBLE:
        case PARAM_STRUCT_CELL_DOUBLE:
            if (!m_paramsDouble.at(name).isScalar()) {
                return false;
            }
            param = static_cast<T>(m_paramsDouble.at(name).getScalarParameter());
            break;
        case PARAM_STRING:
        case PARAM_CELL_STRING:
        case PARAM_STRUCT_STRING:
        case PARAM_STRUCT_CELL_STRING:
            if (!m_paramsString.at(name).isScalar()) {
                return false;
            }
            param = static_cast<T>(std::stod(m_paramsString.at(name).getScalarParameter()));
            break;
    }
    return true;
}

// VECTOR
// ------

template bool Parameters::getParameter<int>(const Parameters::ParamName& name,
                                            std::vector<int>& param) const;
template bool Parameters::getParameter<bool>(const Parameters::ParamName& name,
                                             std::vector<bool>& param) const;
template bool Parameters::getParameter<double>(const Parameters::ParamName& name,
                                               std::vector<double>& param) const;
template bool Parameters::getParameter<std::string>(const Parameters::ParamName& name,
                                                    std::vector<std::string>& param) const;

template <typename T>
bool wbt::Parameters::getParameter(const wbt::Parameters::ParamName& name,
                                   std::vector<T>& param) const
{
    if (!existName(name) || !existName(name, m_nameToType.at(name))) {
        return false;
    }

    param.clear();

    switch (m_nameToType.at(name)) {
        case PARAM_INT:
        case PARAM_CELL_INT:
        case PARAM_STRUCT_INT:
        case PARAM_STRUCT_CELL_INT: {
            if (m_paramsInt.at(name).isScalar()) {
                return false;
            }
            std::vector<T> output;
            convertStdVector(m_paramsInt.at(name).getVectorParameter(), param);
            break;
        }
        case PARAM_BOOL:
        case PARAM_CELL_BOOL:
        case PARAM_STRUCT_BOOL:
        case PARAM_STRUCT_CELL_BOOL: {
            if (m_paramsBool.at(name).isScalar()) {
                return false;
            }
            std::vector<T> output;
            convertStdVector(m_paramsBool.at(name).getVectorParameter(), param);
            break;
        }
        case PARAM_DOUBLE:
        case PARAM_CELL_DOUBLE:
        case PARAM_STRUCT_DOUBLE:
        case PARAM_STRUCT_CELL_DOUBLE: {
            if (m_paramsDouble.at(name).isScalar()) {
                return false;
            }
            std::vector<T> output;
            convertStdVector(m_paramsDouble.at(name).getVectorParameter(), param);
            break;
        }
        case PARAM_STRING:
        case PARAM_CELL_STRING:
        case PARAM_STRUCT_STRING:
        case PARAM_STRUCT_CELL_STRING: {
            if (m_paramsString.at(name).isScalar()) {
                return false;
            }
            std::vector<T> output;
            convertStdVector(m_paramsString.at(name).getVectorParameter(), param);
            break;
        }
    }
    return true;
}

// STOREPARAMETER
// ============

// SCALAR
// ------

template bool Parameters::storeParameter<int>(const int& param,
                                              const ParameterMetadata& paramMetadata);
template bool Parameters::storeParameter<bool>(const bool& param,
                                               const ParameterMetadata& paramMetadata);
template bool Parameters::storeParameter<double>(const double& param,
                                                 const ParameterMetadata& paramMetadata);

template <typename T>
bool wbt::Parameters::storeParameter(const T& param, const wbt::ParameterMetadata& paramMetadata)
{
    if (existName(paramMetadata.m_name) || existName(paramMetadata.m_name, paramMetadata.m_type)) {
        return false;
    }

    if (paramMetadata.m_rows != 1 && paramMetadata.m_cols != 1) {
        return false;
    }

    switch (paramMetadata.m_type) {
        case PARAM_INT:
        case PARAM_CELL_INT:
        case PARAM_STRUCT_INT:
        case PARAM_STRUCT_CELL_INT:
            m_paramsInt.emplace(std::make_pair(
                paramMetadata.m_name, ParameterInt(static_cast<int>(param), paramMetadata)));
            break;
        case PARAM_BOOL:
        case PARAM_CELL_BOOL:
        case PARAM_STRUCT_BOOL:
        case PARAM_STRUCT_CELL_BOOL:
            m_paramsBool.emplace(std::make_pair(
                paramMetadata.m_name, ParameterBool(static_cast<bool>(param), paramMetadata)));
            break;
        case PARAM_DOUBLE:
        case PARAM_CELL_DOUBLE:
        case PARAM_STRUCT_DOUBLE:
        case PARAM_STRUCT_CELL_DOUBLE:
            m_paramsDouble.emplace(std::make_pair(
                paramMetadata.m_name, ParameterDouble(static_cast<double>(param), paramMetadata)));
            break;
        case PARAM_STRING:
        case PARAM_CELL_STRING:
        case PARAM_STRUCT_STRING:
        case PARAM_STRUCT_CELL_STRING:
            m_paramsString.emplace(std::make_pair(
                paramMetadata.m_name, ParameterString(std::to_string(param), paramMetadata)));
            break;
    }

    m_nameToType[paramMetadata.m_name] = paramMetadata.m_type;
    m_nameToIndex[paramMetadata.m_name] = paramMetadata.m_index;
    m_indexToName[paramMetadata.m_index] = paramMetadata.m_name;

    return true;
}

// VECTOR
// ------

template bool Parameters::storeParameter<int>(const std::vector<int>& param,
                                              const ParameterMetadata& paramMetadata);
template bool Parameters::storeParameter<bool>(const std::vector<bool>& param,
                                               const ParameterMetadata& paramMetadata);
template bool Parameters::storeParameter<double>(const std::vector<double>& param,
                                                 const ParameterMetadata& paramMetadata);
template bool Parameters::storeParameter<std::string>(const std::vector<std::string>& param,
                                                      const ParameterMetadata& paramMetadata);

template <typename T>
bool wbt::Parameters::storeParameter(const std::vector<T>& param,
                                     const wbt::ParameterMetadata& paramMetadata)
{
    if (existName(paramMetadata.m_name) || existName(paramMetadata.m_name, paramMetadata.m_type)) {
        return false;
    }

    if (paramMetadata.m_rows != 1 && paramMetadata.m_cols != param.size()) {
        return false;
    }

    switch (paramMetadata.m_type) {
        case PARAM_INT:
        case PARAM_CELL_INT:
        case PARAM_STRUCT_INT:
        case PARAM_STRUCT_CELL_INT: {
            std::vector<int> paramInt(param.size());
            convertStdVector<T, int>(param, paramInt);
            m_paramsInt.emplace(
                std::make_pair(paramMetadata.m_name, ParameterInt(paramInt, paramMetadata)));
            break;
        }
        case PARAM_BOOL:
        case PARAM_CELL_BOOL:
        case PARAM_STRUCT_BOOL:
        case PARAM_STRUCT_CELL_BOOL: {
            std::vector<bool> paramBool(param.size());
            convertStdVector<T, bool>(param, paramBool);
            m_paramsBool.emplace(
                std::make_pair(paramMetadata.m_name, ParameterBool(paramBool, paramMetadata)));
            break;
        }
        case PARAM_DOUBLE:
        case PARAM_CELL_DOUBLE:
        case PARAM_STRUCT_DOUBLE:
        case PARAM_STRUCT_CELL_DOUBLE: {
            std::vector<double> paramDouble(param.size());
            convertStdVector<T, double>(param, paramDouble);
            m_paramsDouble.emplace(
                std::make_pair(paramMetadata.m_name, ParameterDouble(paramDouble, paramMetadata)));
            break;
        }
        case PARAM_STRING:
        case PARAM_CELL_STRING:
        case PARAM_STRUCT_STRING:
        case PARAM_STRUCT_CELL_STRING: {
            std::vector<std::string> paramString(param.size());
            convertStdVector<T, std::string>(param, paramString);
            m_paramsString.emplace(
                std::make_pair(paramMetadata.m_name, ParameterString(paramString, paramMetadata)));
            break;
        }
    }

    m_nameToType[paramMetadata.m_name] = paramMetadata.m_type;
    m_nameToIndex[paramMetadata.m_name] = paramMetadata.m_index;
    m_indexToName[paramMetadata.m_index] = paramMetadata.m_name;

    return true;
}

// PARAMETER
// ---------

template bool Parameters::storeParameter<int>(const Parameter<int>& parameter);
template bool Parameters::storeParameter<bool>(const Parameter<bool>& parameter);
template bool Parameters::storeParameter<double>(const Parameter<double>& parameter);
template bool Parameters::storeParameter<std::string>(const Parameter<std::string>& parameter);

template <typename T>
bool wbt::Parameters::storeParameter(const wbt::Parameter<T>& parameter)
{
    if (existName(parameter.getMetadata().m_name)) {
        return false;
    }

    if (parameter.isScalar()) {
        return storeParameter(parameter.getScalarParameter(), parameter.getMetadata());
    }
    else {
        return storeParameter(parameter.getVectorParameter(), parameter.getMetadata());
    }
}

// TEMPLATE SPECIALIZATIONS
// ========================

template <>
bool Parameters::getParameter<std::string>(const ParamName& name, std::string& param) const
{
    if (!existName(name) || !existName(name, m_nameToType.at(name))) {
        return false;
    }

    switch (m_nameToType.at(name)) {
        case PARAM_INT:
        case PARAM_CELL_INT:
        case PARAM_STRUCT_INT:
        case PARAM_STRUCT_CELL_INT:
            if (!m_paramsInt.at(name).isScalar()) {
                return false;
            }
            param = std::to_string(m_paramsInt.at(name).getScalarParameter());
            break;
        case PARAM_BOOL:
        case PARAM_CELL_BOOL:
        case PARAM_STRUCT_BOOL:
        case PARAM_STRUCT_CELL_BOOL:
            if (!m_paramsBool.at(name).isScalar()) {
                return false;
            }
            param = std::to_string(m_paramsBool.at(name).getScalarParameter());
            break;
        case PARAM_DOUBLE:
        case PARAM_CELL_DOUBLE:
        case PARAM_STRUCT_DOUBLE:
        case PARAM_STRUCT_CELL_DOUBLE:
            if (!m_paramsDouble.at(name).isScalar()) {
                return false;
            }
            param = std::to_string(m_paramsDouble.at(name).getScalarParameter());
            break;
        case PARAM_STRING:
        case PARAM_CELL_STRING:
        case PARAM_STRUCT_STRING:
        case PARAM_STRUCT_CELL_STRING:
            if (!m_paramsString.at(name).isScalar()) {
                return false;
            }
            param = m_paramsString.at(name).getScalarParameter();
            break;
    }
    return true;
}

template <>
bool wbt::Parameters::storeParameter<std::string>(const std::string& param,
                                                  const wbt::ParameterMetadata& paramMetadata)
{
    if (existName(paramMetadata.m_name) || existName(paramMetadata.m_name, paramMetadata.m_type)) {
        return false;
    }

    if (paramMetadata.m_rows != 1 && paramMetadata.m_cols != 1) {
        return false;
    }

    switch (paramMetadata.m_type) {
        case PARAM_INT:
        case PARAM_CELL_INT:
        case PARAM_STRUCT_INT:
        case PARAM_STRUCT_CELL_INT:
            m_paramsInt.emplace(std::make_pair(paramMetadata.m_name,
                                               ParameterInt(std::stoi(param), paramMetadata)));
            break;
        case PARAM_BOOL:
        case PARAM_CELL_BOOL:
        case PARAM_STRUCT_BOOL:
        case PARAM_STRUCT_CELL_BOOL:
            m_paramsBool.emplace(
                std::make_pair(paramMetadata.m_name,
                               ParameterBool(static_cast<bool>(std::stoi(param)), paramMetadata)));
            break;
        case PARAM_DOUBLE:
        case PARAM_CELL_DOUBLE:
        case PARAM_STRUCT_DOUBLE:
        case PARAM_STRUCT_CELL_DOUBLE:
            m_paramsDouble.emplace(std::make_pair(
                paramMetadata.m_name, ParameterDouble(std::stod(param), paramMetadata)));
            break;
        case PARAM_STRING:
        case PARAM_CELL_STRING:
        case PARAM_STRUCT_STRING:
        case PARAM_STRUCT_CELL_STRING:
            m_paramsString.emplace(
                std::make_pair(paramMetadata.m_name, ParameterString(param, paramMetadata)));
            break;
    }

    m_nameToType[paramMetadata.m_name] = paramMetadata.m_type;
    m_nameToIndex[paramMetadata.m_name] = paramMetadata.m_index;
    m_indexToName[paramMetadata.m_index] = paramMetadata.m_name;

    return true;
}
