#pragma once

#include <cstdint>

struct nn_to_depth_bbox_map
{
    uint16_t offset_x;
    uint16_t offset_y;
    uint16_t max_width;
    uint16_t max_height;
};

struct cnn_info
{
    uint16_t cnn_input_width;
    uint16_t cnn_input_height;
    uint16_t cnn_input_num_channels;
    uint16_t number_of_cmx_slices;
    uint16_t number_of_shaves;
    uint32_t offsets[7];
    nn_to_depth_bbox_map nn_to_depth;
    int32_t satisfied_resources;
};


typedef enum TensorDataType : uint32_t
{
    _fp16 = 0,                  ///< Half precision floating point
    _u8f  = 1,                  ///< Unsigned byte
    _int  = 2,                  ///< Signed integer (4 byte)
    _fp32 = 3,                  ///< Single precision floating point
    _i8   = 4,                  ///< Signed byte
} TensorDataType;

static int constexpr MAX_DIM_SIZE = 16;
static int constexpr MAX_NAME_LENGTH = 100;
static int constexpr MAX_TENSOR_INPUTS = 4;
static int constexpr MAX_TENSOR_OUTPUTS = 10;

typedef struct
{
    uint32_t numDims;
    TensorDataType dataType;

    uint32_t dims[MAX_DIM_SIZE];
} TensorShape;

struct Tensor_info
{
    uint32_t idx;
    uint32_t offset;
    char name[MAX_NAME_LENGTH];
    TensorShape shape;
};

struct cnn_tensor_infos
{
    uint32_t input_size;
    Tensor_info input_info[MAX_TENSOR_INPUTS];
    uint32_t output_size;
    Tensor_info output_info[MAX_TENSOR_OUTPUTS];
    nn_to_depth_bbox_map nn_to_depth;
    int32_t satisfied_resources;
};