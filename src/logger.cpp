#include "logger.h"
//need sudo to run exec

// Member functions definitions including constructor
logger::logger(string filename, int cycles_until_log)
{
printf("opening file for logging: %s\n",filename.c_str());
    //when queue gets larget than this, will flush all
    max_queue_length = cycles_until_log; 
    
    const char *filename_p = filename.c_str();
    (this->myfile).open(filename_p, ios::out | ios::app);

    std::cout.fixed;
}

void logger::log(Data_log d)
{
//push data onto queue
q.push(d);
//when queue gets longer than max length write all to file
if (q.size() > max_queue_length) this->write_to_file();
}

void logger::write_to_file( void )
{
    //write all data in queue to file    
    while(q.size() > 0)
        {
        //get front of queue and remove it
        Data_log d_temp = q.front();
        q.pop();
        
        //unwrap elements and write to file
       this->unwrap(d_temp);
       }
}

void logger::unwrap(Data_log d){

    Vicon v = d.vicon;
    Times t = d.time;
    State imu = d.imu;

    myfile << "Vicon: " <<  num2str(v.x) <<  ", " << num2str(v.y) << ", " << num2str(v.z);
    myfile << "State: " << num2str(imu.theta) << ", " << num2str(imu.phi) << ", " << num2str(imu.psi) << '\n';
}

template <typename num>
string logger::num2str(num f)
{

std::ostringstream ss;
ss <<std::fixed << std::setprecision(3)<< f;
std::string s(ss.str());

return s;
}
/*
int main (void){
logger log("file.log",3);    
Data_log d;
Vicon v = {1.5};
State imu = {2.5};
d.vicon = v;
d.imu = imu;
for(int i = 0; i < 20; i++) logger.log(d);

}
*/

//string logger::data2csv(Vicon v)
//{
//String s = 
//}
