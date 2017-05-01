#ifndef HELIOSSOLUTION_H
#define HELIOSSOLUTION_H
//#include "libs/Module.h"
#include "BaseSolution.h"

class Config;

class HeliosSolution : public BaseSolution {
    public:
        HeliosSolution(Config*);
        void cartesian_to_actuator(const float[], ActuatorCoordinates &) const override;
        void actuator_to_cartesian(const ActuatorCoordinates &, float[] ) const override;

        bool set_optional(const arm_options_t& options) override;
        bool get_optional(arm_options_t& options, bool force_all) const override;

    private:
        void init();
        float to_degrees(float radians) const;
        float to_radians(float degrees) const;

        float arm1_length;
        float arm2_length;
        float z_mm_per_rotation;
        float min_radius;
        float max_radius;
        mutable int is_default_arm_mode;
        //float offset_x;
        //float offset_y;
        //float scaling_x;
        //float scaling_y;
        //float undefined_min;
        //float undefined_max;
        float slow_rate;
};

#endif // HELIOSSOLUTION_H
