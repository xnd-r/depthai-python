#pragma once

#include <assert.h>

#include <string>
#include <map>
#include "cnn_info.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include "half.hpp"

using float16 = half_float::half;
namespace pybind11 { namespace detail {
template <>
    struct npy_format_descriptor<float16> {
    static std::string format() {
        // https://docs.python.org/3/library/struct.html#format-characters
        return "e";
    }   
    };
}} // namespace pybind11::detail



const std::map<TensorDataType, unsigned int> c_type_size = {
    {TensorDataType::_fp16,     sizeof(float16)},
    {TensorDataType::_u8f,      sizeof(std::uint8_t)},
    {TensorDataType::_int,      sizeof(std::int32_t)},
    {TensorDataType::_fp32,     sizeof(float)},
    {TensorDataType::_i8,       sizeof(std::int8_t)},
};

const std::map<TensorDataType, std::string> type_to_numpy_format = {
    {TensorDataType::_fp16,     pybind11::format_descriptor<float16>::format()},
    {TensorDataType::_u8f,      pybind11::format_descriptor<std::uint8_t>::format()},
    {TensorDataType::_int,      pybind11::format_descriptor<std::int32_t>::format()},
    {TensorDataType::_fp32,     pybind11::format_descriptor<float>::format()},
    {TensorDataType::_i8,       pybind11::format_descriptor<std::int8_t>::format()},
};

const std::map<TensorDataType, std::string> type_to_string = {
    {TensorDataType::_fp16,     "float16"},
    {TensorDataType::_u8f,      "uint8"},
    {TensorDataType::_int,      "int32"},
    {TensorDataType::_fp32,     "float32"},
    {TensorDataType::_i8,       "int8"},
};

unsigned int size_of_type   (const TensorDataType& type);
std::string type_to_npy_format_descriptor(const TensorDataType& type);
