#include "logger.h"
//need sudo to run exec

// Member functions definitions including constructor
logger::logger(string filename)
{
printf("In Constructor\n");
printf("opening file for logging: %s",filename.c_str());
 
    //ofstream myfile (filename.c_str(), ios::out | ios::app);

    this -> filename = filename;
    const char *filename_p = filename.c_str();
    int file_descriptor = open(filename_p,  O_WRONLY| O_APPEND);
    this -> file_descriptor = file_descriptor;

    (this->myfile).open(filename_p, ios::out | ios::app);

}

void logger::log(Vicon v)
{

}
 
void logger::write_to_file( void )
{
//write(filename_p "This is a line.\n");
myfile << "This is a line";
myfile << "This is a line";

}

int main (void){
std::cout << "HIIII" << std::endl;
logger log("file.log");    
log.write_to_file();

}
