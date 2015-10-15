#include "logger.h"
//need sudo to run exec

// Member functions definitions including constructor
logger::logger(string filename)
{
printf("opening file for logging: %s\n",filename.c_str());
 
    this -> filename = filename;
    const char *filename_p = filename.c_str();

    (this->myfile).open(filename_p, ios::out | ios::app);

    std::cout.precision(10);
    std::cout.fixed;
}

void logger::log(Vicon v)
{
q.push(v);
}

void logger::write_to_file( void )
{
myfile << "New Entry";
float a = 425534.10230;
/*
const char* b = num2str(a,10);
cout << "IN WRITE" << endl;
cout << "output of function, pointer to char: " << b << endl;
cout << "output of function, deref pointer to char: " << *b << endl;
//myfile.write(  num2str(a,10), 10 );// << num2str(a,10);
*/
cout << std::setprecision(8) << num2str_1(a)<< endl;
//myfile.write(num2str_1(a), sizeof(num2str_1(a)));
myfile << num2str_1(a);
}

/*
template <typename num>
const char* logger::num2str(num f, int num_digits)
{
char buf[num_digits];
cout << "IN NUM2STR" << endl;
cout << "number: " << f << endl;
cout << "num digits: " << num_digits << endl;
snprintf(buf,num_digits, "%f", f);
cout << "buffer form snprintf: " << buf << endl;
return buf;
}
*/
template <typename num>
string logger::num2str_1(num f)
{
std::ostringstream ss;
ss <<std::fixed << std::setprecision(8)<< f;
std::string s(ss.str());

return s;
}

int main (void){
logger log("file.log");    

log.write_to_file();

}

//string logger::data2csv(Vicon v)
//{
//String s = 
//}
