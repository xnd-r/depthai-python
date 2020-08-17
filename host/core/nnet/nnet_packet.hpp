#pragma once

#include <assert.h>

#include <memory>
#include <unordered_map>
#include <vector>

#include "tensor_info.hpp"
#include "../host_data_packet.hpp"
#include "cnn_info.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "types.hpp"

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
    py::array* _getTensorPythonNumpyArray(unsigned char *data, TensorInfo ti)
    {
        assert(!ti.tensor_dimensions.empty());
        py::array* result = nullptr;

        ssize_t              ndim    = ti.tensor_dimensions.size();
        ssize_t              element_size = size_of_type(ti.tensor_data_type);
        std::string          numpy_format_descriptor = type_to_npy_format_descriptor(ti.tensor_data_type);
        std::vector<ssize_t> shape;
        std::vector<ssize_t> strides;
        shape.reserve(ndim);
        strides.reserve(ndim);

        for (int i = 0; i < ndim; ++i)
        {
            shape.push_back(ti.tensor_dimensions[i]);
            strides.push_back(ti.tensor_strides[i]);
        }

        try {

            result = new py::array(py::buffer_info(
                        static_cast<void*>(&data[ti.tensor_offset]),                             /* data as contiguous array  */
                        element_size,                          /* size of one scalar        */
                        numpy_format_descriptor,         /* data type          */
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
        return _getTensorPythonNumpyArray(data, ti);
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

    py::list getOutputsList() {
        py::list outputList;
        for (size_t i = 0; i < _tensors_info.size(); ++i)
        {
            outputList.append(getTensor(i));
        }
        return outputList;
    }


    py::dict getOutputsDict() {
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

    int getDetectionCount()
    {
        unsigned char * data = _tensors_raw_data->data.data();
        detection_out_t * detections = (detection_out_t *)data;
        return detections->detection_count;
    }

#ifdef HOST_PYTHON_MODULE
    py::object getDetectedObject(int detected_nr)
    {
        unsigned char * data = _tensors_raw_data->data.data();
        detection_out_t * detections = (detection_out_t *)data;
        assert(detected_nr < detections->detection_count);
        return py::cast<detection_t>(detections->detections[detected_nr]);
    }
#endif

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
