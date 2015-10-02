//=================================
// include guard
#ifndef CONCUR_DATA_H
#define CONCUR_DATA_H

#include <boost/interprocess/managed_shared_memory.hpp>
#include "data_structs.h"
#include <unistd.h>
#include <string>
#include <stdlib>
#include <iostream>

using namespace std;


class concur_data
{
   private:
    <DataType> the_data;  //this is the State struct that we will be putting the array into
    mutable boost::mutex the_mutex;
    std::pair<float*, std::size_t> array;  //this is the array that will be changed in other processes

   public:
   	concur_data(string mem_loc_name, string type_of_input);  // This is the constructor
    void update(const DataType& data);
    DataType retrieve(void);

};


#endif
// __CONCUR_DATA_H_INCLUDED__
