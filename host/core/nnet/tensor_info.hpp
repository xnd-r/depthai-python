#pragma once

#include <assert.h>

#include <string>
#include <unordered_map>
#include <vector>
#include "cnn_info.hpp"

#include "../types.hpp"


// Tensor -> Entry -> Propery

struct TensorInfo
{
    TensorInfo() = delete;

    TensorInfo(Tensor_info ti)
    {
        tensor_name = std::string(ti.name);

        // output_data_type = ti.shape.dataType;
        tensor_offset = ti.offset;
        tensor_idx = ti.idx;
        tensor_size = size_of_type(output_data_type);
        for(int i = 0; i < ti.shape.numDims; i ++)
        {
            tensor_dimensions.push_back(ti.shape.dims[i]);
            tensor_size *= ti.shape.dims[i];
        }
    }

    std::string tensor_name;
    std::vector<int> tensor_dimensions;
    Type output_data_type = Type::F16;
    size_t tensor_offset;
    size_t tensor_size;
    size_t tensor_idx;

    int nnet_input_width  = 0;
    int nnet_input_height = 0;


    std::vector<std::vector<std::string>>                               output_property_key_index_to_string;
    std::vector<std::unordered_map<std::string, unsigned>>              output_property_key_string_to_index;

};
