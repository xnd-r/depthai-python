#include "types.hpp"


unsigned size_of_type(const TensorDataType& type)
{
    auto it = c_type_size.find(type);
    assert(it != c_type_size.end());
    return it->second;
};

std::string type_to_npy_format_descriptor(const TensorDataType& type)
{
    auto it = type_to_numpy_format.find(type);
    assert(it != type_to_numpy_format.end());
    return it->second;
};
