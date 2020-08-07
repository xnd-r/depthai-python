#pragma once

#include <assert.h>

#include <memory>
#include <unordered_map>
#include <vector>

#include "tensor_info.hpp"
#include "../host_data_packet.hpp"


class NNetPacket
{
public:
    NNetPacket(
              std::shared_ptr<HostDataPacket> &tensors_raw_data,
        const std::vector<TensorInfo>         &tensors_info
    )
        : _tensors_raw_data(tensors_raw_data)
        , _tensors_info(tensors_info)
    {
        for (size_t i = 0; i < tensors_info.size(); ++i)
        {
            _tensor_name_to_index[ tensors_info[i].tensor_name ] = i;
        }

        if (_tensor_name_to_index.size() != tensors_info.size())
        {
            printf("There are duplication in tensor names!\n");
        }
    }

#ifdef HOST_PYTHON_MODULE
    py::array* _getPythonNumpyArray(unsigned char *data, TensorInfo ti)
    {
        assert(!ti.tensor_dimensions.empty());
        py::array* result = nullptr;

        ssize_t              ndim    = ti.tensor_dimensions.size();
        ssize_t              element_size = size_of_type(ti.output_data_type);
        std::vector<ssize_t> shape;
        std::vector<ssize_t> strides;

        auto size_div = std::accumulate(std::begin(ti.tensor_dimensions), std::end(ti.tensor_dimensions), 1, std::multiplies<int>());
        for (int i = 0; i < ti.tensor_dimensions.size(); ++i)
        {
            shape.push_back(ti.tensor_dimensions[i]);

            size_div /= ti.tensor_dimensions[i];
            strides.push_back(size_div*element_size);
        }

        try {

            result = new py::array(py::buffer_info(
                        static_cast<void*>(&data[ti.tensor_offset]),                             /* data as contiguous array  */
                        element_size,                          /* size of one scalar        */
                        py::format_descriptor<std::uint16_t>::format(),         /* data type          */
                        ndim, //ndim,                                    /* number of dimensions      */
                        shape, //shape,                                   /* shape of the matrix       */
                        strides //strides                                  /* strides for each axis     */
                    ));
        } catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            result = nullptr;
        }
       
        return result;
    }

    py::array* getTensor(unsigned index)
    {
        assert(index < _tensors_info.size());
        TensorInfo ti = _tensors_info[index];
        unsigned char * data = _tensors_raw_data->data.data();
        return _getPythonNumpyArray(data, ti);
    }

    py::array* getTensorByName(const std::string &name)
    {
        auto it = _tensor_name_to_index.find(name);
        if (it == _tensor_name_to_index.end())
        {
            return nullptr;
        }
        else
        {
            return getTensor(it->second);
        }
    }

    py::object getMetadata() {
        // TODO
        return _tensors_raw_data->getMetadata();
    }

    py::dict getOutputs() {
        py::dict outputs;
        for (size_t i = 0; i < _tensors_info.size(); ++i)
        {
            std::string tensor_name = getTensorName(i);
            outputs[tensor_name.c_str()] = getTensor(i);
        }
        
        return outputs;
    }   

#endif

    int getTensorsSize()
    {
        return _tensors_info.size();
    }

private: 
    std::string getTensorName(int index)
    {
        return _tensors_info[index].tensor_name;
    }




private:

          std::shared_ptr<HostDataPacket> _tensors_raw_data;
    const std::vector<TensorInfo>                     _tensors_info;

    std::unordered_map<std::string, unsigned> _tensor_name_to_index;
};
