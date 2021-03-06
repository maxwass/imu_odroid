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
    this->format(d.time);            myfile << ", ";    
    this->format(d.vicon_data);      myfile << ", ";
    this->format(d.vicon_data_filt); myfile << ", "; 
    this->format(d.vicon_vel);       myfile << ", "; 
    this->format(d.vicon_vel_filt);  myfile << ", "; 
    this->format(d.vicon_error);     myfile << ", ";
    this->format(d.imu);             myfile << ", ";
    this->format(d.imu_error);       myfile << ", ";
    this->format(d.forces);          myfile << ", ";
    this->format(d.desired_angles); myfile << "\n";
}

template <typename num>
string logger::num2str(num f)
{

std::ostringstream ss;
ss <<std::fixed << std::setprecision(3)<< f;
std::string s(ss.str());

return s;
}
void logger::format(Times t){
    myfile << (t.date_time) ;//<< ", "; //<< t.delta;    
        //myfile <<      << ", " << tv2float(times.delta)
}
void logger::format(Vicon v){
    myfile <<  num2str(v.x) <<  ", " << num2str(v.y) << ", " << num2str(v.z) << ", " << num2str(v.theta) <<  ", " << num2str(v.phi) << ", " << num2str(v.psi);
}
void logger::format(State_Error se){
    format(se.x);     myfile << ", ";
    format(se.y);     myfile << ", ";
    format(se.z);     myfile << ", ";
    format(se.theta); myfile << ", ";
    format(se.phi);   myfile << ", ";
    format(se.psi);  //comma added in main function
}
void logger::format(Errors e){
    myfile <<  num2str(e.prop) <<  ", " << num2str(e.deriv) << ", " << num2str(e.integral);
}
void logger::format(State imu){
    myfile <<  num2str(imu.theta) <<  ", " << num2str(imu.phi) << ", " << num2str(imu.psi) << ", " <<num2str(imu.theta_dot)<<  ", " <<num2str(imu.phi_dot)<< ", "<< num2str(imu.psi_dot);
}
void logger::format(Motor_forces mf){
    myfile << num2str(mf.motor_1) << ", " << num2str(mf.motor_2) << ", " << num2str(mf.motor_3) << ", " << num2str(mf.motor_4);
}
void logger::format(Angles a){
    myfile << num2str(a.theta) << ", " << num2str(a.phi) << ", " << num2str(a.psi);
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

