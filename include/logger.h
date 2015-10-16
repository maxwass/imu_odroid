//=================================
// include guard
#ifndef LOGGER_H
#define LOGGER_H

//=================================
// included dependencies
#include <stdlib.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
//file io
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <queue>
#include <string>
#include "data_structs.h"

using namespace std;

class logger
{
   private:
    ofstream myfile;
    std::queue<Data_log> q;
    int max_queue_length;
    std::string filename;
    const char *filename_p;
    void unwrap(Data_log d);

   public:
    logger(std::string filename, int cycles_until_log);  // This is the constructor
    void log(Data_log d);
    void write_to_file(void);
    template <typename num> string num2str(num f);
    
};
 

#endif
// __LOGGER_H_INCLUDED__
