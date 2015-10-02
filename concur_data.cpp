//class wrapping the actual data
    //only these functions can actually chance the
#include "concur_data.h"


template<typename DataType>
class concurrent_queue
{
private:
    <DataType> the_data;
    mutable boost::mutex the_mutex;
    std::pair<float*, std::size_t> array;

public:
    concur_data(string mem_loc_name, string type_of_input){
        //constructor: store interprocess variables:
        //type_of_input is either vicon or imu
        managed_shared_memory managed_shm_v(open_only, mem_loc_name);
        std::pair<float*, std::size_t> array = managed_shm_v.find<float>(type_of_input);

    }
    void update(const char[]& data)   //OR State& data
    {   //lock data, update changes, unlock
        boost::mutex::scoped_lock lock(the_mutex);
        the_queue.push(data);
    }

    DataType retrieve(void)
    {   //lock data, unpack data properly (either imu data or vicon)
        boost::mutex::scoped_lock lock(the_mutex);
        return the_queue.front();
    }
};