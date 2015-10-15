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
      std::queue<Vicon> q;
      int max_queue_length;
      string filename;
      const char *filename_p;

   public:
    logger(string filename);  // This is the constructor
    void log(Vicon v);
    void write_to_file(void);
    template <typename num> const char* num2str(num f, int num_digits);
    template <typename num> string num2str_1(num f);
    
};
 

#endif
// __LOGGER_H_INCLUDED__
