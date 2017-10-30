#ifndef MXANYTYPE_H
#define MXANYTYPE_H

#include <vector>
#include <string>
#include <unordered_map>
#include <cassert>
#include "AnyType.h"
#include "matrix.h"

class MxAnyType;

// If needed in the future
// class MxAnyCell : public AnyCell {};
// class MxAnyStruct : public AnyStruct {};

struct MxArrayMetadata {
    mxClassID id;
    bool isScalar;
    size_t rows;
    size_t cols;
    size_t nElem;
    size_t nDims;
    std::vector<size_t> dims;
};

class MxAnyType : public AnyType
{
private:
    const mxArray* mx;
    MxArrayMetadata md;
    bool validate;

    // TODO: https://it.mathworks.com/help/matlab/apiref/mxgetscalar.html returns a double always
    bool asScalar(double& d);
    bool validateClassId(mxClassID id1, mxClassID id2);

    // mxArray* getMxArrayPtr() const { return static_cast<mxArray*>(getAnyDataPtr()); }

public:
    // void* getAnyDataPtr() const override {
    //     return (void*) mx;
    // }

    MxAnyType();
    MxAnyType(const mxArray* m, bool validateId = false);
    ~MxAnyType() = default;
    MxAnyType(const MxAnyType& mxAnyType);

    void enableClassIDValidation();

    // STRING / CHARS
    // ==============

    bool asString(std::string& s) override;

    // SCALAR TYPES
    // ============

    // Generic casting
    // ---------------

    bool asInt(int& i) override;
    bool asUInt(unsigned& i) override;

    // Specific casting
    // ----------------

    bool asInt32(int32_t& i) override;

    // TODO: complete with all the other scalar types
    // bool asInt64(int64_t& i) override
    // {
    //     double buffer;
    //     if (!asScalar(buffer)) return false;
    //     i = static_cast<int64_t>(buffer);
    //     return validateClassId(md.id, mxINT64_CLASS);
    // }

    bool asDouble(double& d) override;
    bool asBool(bool& b) override;

    // COMPOSITE DATA TYPES
    // ====================

    bool asAnyStruct(AnyStruct& s) override;
    bool asAnyCell(AnyCell& cell) override;

    // MATRIX
    // ======

    // VECTOR
    // ======

    bool asVectorDouble(Eigen::VectorXd& vec) override;
    bool asVectorDouble(std::vector<double>& vec) override;
};

#endif /* MXANYTYPE_H */
