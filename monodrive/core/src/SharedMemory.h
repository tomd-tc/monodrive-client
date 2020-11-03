// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
//#include <boost/interprocess/shared_memory_object.hpp>
#pragma once

#pragma push_macro("TEXT")
#undef TEXT
#pragma warning( push )
#pragma warning( disable: 4668 4191)
#undef check
#pragma push_macro("check")
#include <boost/interprocess/managed_shared_memory.hpp>
//#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

#pragma warning( pop)
#pragma pop_macro("TEXT")
#pragma pop_macro("check")

#include <cstring>
#include <cstdlib>
#include <string>
#include <iostream>


typedef struct
    {
        int8_t payload[100];
    } camera;

template <typename T>
struct sensor_frame
    {
        int32_t time;
        int32_t gametime;
        int32_t length;
        T frame;
    };

template <typename T> 
class SharedMemory
{
    typedef boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager> memory_alloc;
    typedef boost::interprocess::vector<T, memory_alloc> memory_vec;
    public:
        SharedMemory(const char* in_name):name(in_name){

        }
        ~SharedMemory(){
            boost::interprocess::shared_memory_object::remove(name);
        }

        memory_vec* create(const char* segmentName, int size)
        {
            segment =  boost::interprocess::managed_shared_memory(boost::interprocess::create_only, name, size);
            const memory_alloc alloc_inst(segment.get_segment_manager());
            return segment.construct<memory_vec>(segmentName)(alloc_inst);
        }
        void open()
        {
            segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, name);
        }
        memory_vec* find(const char* segmentName)
        {
            memory_vec *image_frame = segment.find<memory_vec>(segmentName).first;
            return image_frame;
        }
        private:
            boost::interprocess::managed_shared_memory segment;
            const char *name;
};