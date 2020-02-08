class Odometry
{
    private:
        double D = 0;       //lenght between wheel
        double r = 0;       //radius of wheel
        

        double *r_wheel_vel_buff;
        double *l_wheel_vel_buff;
        double l_wheel_vel = 0; 
        double r_wheel_vel = 0;
        const char wheel_buff = 2;

        double lin_speed = 0;
        double ang_speed = 0;
        double ang_speed_last = 0;

        double x_speed = 0;
        double x_speed_last = 0;
        double y_speed = 0;
        double y_speed_last = 0;

        double x = 0;       //integral of x_speed
        double y = 0;       //integral of y_speed
        double t = 0;       //integral of ang_speed

        double last_time = 0;
        double actual_time = 0;

        unsigned int number = 0;

    private:
        inline void calcTheta();
        double oneStepIntegral(double last, double now); //time is implicit 
        inline void calcLinearSpeed();
        inline void calcPosition();
        void calcWheelVelocity(double l_last_pos, double l_actual_pos, double r_last_pos, double r_actual_pos);
        inline void calcLinearVelocity();
        inline void calcAngularVelocity();
    public:
        void calculateOdometry(double l_last_pos, double l_actual_pos, double r_last_pos, double r_actual_pos, double time);
        void getVelocity(double* vel);
        void getPosition(double* pos);
        double getTime();
        unsigned int getNumber();
        
        Odometry(double x, double y, double t, double raduis, double D);
        ~Odometry();
};
