#ifndef MOTOR_H
#define MOTOR_H


class motor
{
   private:
      double ensure_valid_force(double force_in);
      int motor_id;
      double force;
      double max_force = 2047.0;
      double min_force = 0.0;

   public:
   	  motor(int which_motor);  // This is the constructor
      void set_force( double force_in );
      double get_force( void );
      void shut_down(void);
      int which_motor(void);
 
};
 

#endif
