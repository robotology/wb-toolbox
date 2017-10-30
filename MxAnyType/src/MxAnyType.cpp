#include "MxAnyType.h"
#include <memory>

// PRIVATE METHODS
// ===============

bool MxAnyType::asScalar(double& d)
{
    if (!mx) return false;
    if (!mxIsScalar(mx)) return false;  // 1x1
    if (!mxIsNumeric(mx)) return false; // Types: https://it.mathworks.com/help/matlab/apiref/mxisnumeric.html

    // Cast to double since even a mxINT8_CLASS is returned as double:
    // https://it.mathworks.com/help/matlab/apiref/mxgetscalar.html
    d = static_cast<double>(mxGetScalar(mx));
    return true;
}

bool MxAnyType::validateClassId(mxClassID id1, mxClassID id2)
{
    if (validate)
        return id1 == id2;
    else
        return true;
}

// PUBLIC METHODS
// ==============

// Constructors
// ============

MxAnyType::MxAnyType() : mx(nullptr), validate(false)
{
    md.id = mxUNKNOWN_CLASS;
}

MxAnyType::MxAnyType(const mxArray* m, bool validateId) : mx(m), validate(validateId)
{
    assert(mx);

    // Get the ID
    md.id = mxGetClassID(mx);
    assert(md.id != mxVOID_CLASS);
    assert(md.id != mxUNKNOWN_CLASS);

    // Get the other metadata
    md.isScalar = mxIsScalar(mx);
    md.rows  = static_cast<unsigned>(mxGetN(mx));
    md.cols  = static_cast<unsigned>(mxGetM(mx));
    md.nElem = static_cast<unsigned>(mxGetNumberOfElements(mx));
    md.nDims = static_cast<unsigned>(mxGetNumberOfDimensions(mx));

    if (md.isScalar)
        assert(md.rows == md.cols == md.nElem == 1);

    // TODO: only 2 dims currently supported
    assert(md.nDims == 2);
    assert(md.rows * md.cols == md.nElem);

    const size_t* size = mxGetDimensions(mx);
    for (unsigned dim = 0; dim < md.nDims; ++dim) {
        md.dims.push_back(static_cast<unsigned>(size[dim]));
    }
    assert(md.dims.size() == 2);
}

MxAnyType::MxAnyType(const MxAnyType& mxAnyType)
: mx(mxAnyType.mx)
, md(mxAnyType.md)
, validate(mxAnyType.validate)
{}

void MxAnyType::enableClassIDValidation()
{
    validate = true;
}

// STRING / CHARS
// ==============

bool MxAnyType::asString(std::string& s)
{
    if (!mx) return false;
    if (md.id != mxCHAR_CLASS) return false;
    char* buffer = mxArrayToString(mx);
    s = std::string(buffer);
    mxFree(buffer);
    return true;
}

// SCALAR TYPES
// ============

// Generic casting
// ---------------

bool MxAnyType::asInt(int& i)
{
    double buffer;
    if (!asScalar(buffer)) return false;
    i = static_cast<int>(buffer);
    return true;
}

bool MxAnyType::asUInt(unsigned& i)
{
    double buffer;
    if (!asScalar(buffer)) return false;
    i = static_cast<unsigned>(buffer);
    return true;
}

// Specific casting
// ----------------

bool MxAnyType::asInt32(int32_t& i)
{
    double buffer;
    if (!asScalar(buffer)) return false;
    i = static_cast<int32_t>(buffer);
    return validateClassId(md.id, mxINT32_CLASS);
}

// TODO: complete with all the other scalar types
// bool asInt64(int64_t& i)
// {
//     double buffer;
//     if (!asScalar(buffer)) return false;
//     i = static_cast<int64_t>(buffer);
//     return validateClassId(md.id, mxINT64_CLASS);
// }

bool MxAnyType::asDouble(double& d)
{
    bool ok = asScalar(d);
    return ok && validateClassId(md.id, mxDOUBLE_CLASS);
}

bool MxAnyType::asBool(bool& b)
 {
    if (!mx) return false;
    if (!mxIsLogicalScalar(mx)) return false;
    b = mxIsLogicalScalarTrue(mx);
    return true;
}

// COMPOSITE DATA TYPES
// ====================

bool MxAnyType::asAnyStruct(AnyStruct& s)
{
    if (!mx) return false;
    if (md.id != mxSTRUCT_CLASS) return false;

    for (unsigned i = 0; i < mxGetNumberOfFields(mx); ++i) {
        const char* fieldName = mxGetFieldNameByNumber(mx,i);
        // TODO multidimensional struct
        mxArray* fieldContent = mxGetFieldByNumber(mx,0,i);
        if (fieldName == nullptr) return false;
        if (fieldContent == nullptr) return false;
        s[std::string(fieldName)] = std::make_shared<MxAnyType>(fieldContent);
    }
    return true;
}

bool MxAnyType::asAnyCell(AnyCell& cell)
{
    if (!mx) return false;
    if (md.id != mxCELL_CLASS) return false;

    // TODO: AnyCell then will have a operator()(3,4) method;
    for (unsigned i=0; i < mxGetNumberOfElements(mx); ++i) {
        mxArray* cellContent = mxGetCell(mx, i);
        if (!cellContent) return false;
        cell.push_back(std::make_shared<MxAnyType>(cellContent));
    }
    return true;
}

// MATRIX
// ======

// VECTOR
// ======

// TODO tests with other types (uint8) https://it.mathworks.com/help/matlab/apiref/mxgetdata.html (Descriptio)
bool MxAnyType::asVectorDouble(Eigen::VectorXd& vec)
{
    if (!mx) return false;
    if (!mxIsDouble(mx)) return false;

    if (md.rows > 1 && md.cols > 1) {
        return false;
    }

    // TODO add method for complex vectors (and move the check into md)
    if (mxIsComplex(mx)) return false;

    double* buffer = mxGetPr(mx);
    if (!buffer) return false;

    vec = Eigen::Map<Eigen::VectorXd>(buffer, md.nElem);
    return true;
}

bool MxAnyType::asVectorDouble(std::vector<double>& vec)
{
    Eigen::VectorXd vecEigen;
    if (!asVectorDouble(vecEigen)) return false;
    vec.assign(vecEigen.data(), vecEigen.data() + vecEigen.rows() * vecEigen.cols());
    return true;
}
