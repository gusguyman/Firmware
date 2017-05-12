#ifndef PRINCIPAL_AXIS_C_H
#define PRINCIPAL_AXIS_C_H


class principal_axis_c {
public:
    /** Default constructor */
    principal_axis_c();
    /** Default destructor */
    virtual ~principal_axis_c();
    void SetGains(float in_kp, float in_kd, float in_ki);
    void SetDesired(float in_desired);
    void SetCurrentValue(float in_value);
    void SetBounds(float in_min, float in_max);

    void PID_Update();

    float GetOutput() {return _output;}


private:
    //Struct definitions
    struct current_s{
        float value;
        float integral;
        float err;
        float derr;
    };
    struct prev_s{
        float integral;
        float err;
    };
    struct gains_s{
        float kp;
        float ki;
        float kd;
    };

    current_s _current;
    prev_s _prev;
    gains_s _gains;
    float _desired;
    float _output;
    float _dt;
    float _max;
    float _min;

};

#endif
