#include "HeliosSolution.h"
#include <cmath>
#include <fastmath.h>
#include "checksumm.h"
#include "ActuatorCoordinates.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "StreamOutputPool.h"

#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#define arm1_length_checksum          CHECKSUM("arm1_length")
#define arm2_length_checksum          CHECKSUM("arm2_length")
#define z_mm_per_rotation_checksum    CHECKSUM("z_mm_per_rotation")
//#define homing_checksum               CHECKSUM("homing")
#define min_radius_checksum        CHECKSUM("min_radius")
#define max_radius_checksum        CHECKSUM("max_radius")

#define SQ(x) powf(x, 2)
#define SQRT(x) sqrtf(x)
#define ROUND(x, y) (roundf(x * 1e ## y) / 1e ## y)

HeliosSolution::HeliosSolution(Config* config)
{
    // arm1_length is the length of the inner main arm from hinge to hinge
    arm1_length         = config->value(arm1_length_checksum)->by_default(150.0f)->as_number();
    // arm2_length is the length of the inner main arm from hinge to hinge
    arm2_length         = config->value(arm2_length_checksum)->by_default(150.0f)->as_number();
    z_mm_per_rotation         = config->value(z_mm_per_rotation_checksum)->by_default(128.0f)->as_number();
    min_radius= config->value(min_radius_checksum)->by_default(115.0f)->as_number();
    max_radius= config->value(max_radius_checksum)->by_default(299.99f)->as_number();
    is_default_arm_mode=true;
    //is_default_arm_mode=true;
    // undefined is the ratio at which the SCARA position is undefined.
    // required to prevent the arm moving through singularity points
    // min: head close to tower
    //undefined_min  = config->value(undefined_min_checksum)->by_default(0.9999f)->as_number();
    // max: head on maximum reach
    //undefined_max  = config->value(undefined_max_checksum)->by_default(0.9999f)->as_number();

    init();
}

void HeliosSolution::init()
{
}

float HeliosSolution::to_degrees(float radians) const
{
    return radians * (180.0F / 3.14159265359f);
}

float HeliosSolution::to_radians(float degrees) const
{
    return (degrees / 180.0F) * 3.14159265359f;
}

void HeliosSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{

    float SCARA_pos[2],
          SCARA_C2,
          SCARA_S2,
          SCARA_K1,
          SCARA_K2,
          SCARA_theta,
          SCARA_psi,
          r,
          x,
          y;
          
    x=cartesian_mm[X_AXIS];
    y=cartesian_mm[Y_AXIS];
    r=SQRT(SQ(x)+SQ(y));
    float arcLength=3.14159265359f/2*min_radius;
    if(r<min_radius && y>=0)
    {
		float xmax=SQRT(min_radius*min_radius-y*y);
		float arc=3.14159265359f/2-atan2f(y,xmax);
		float p=x/xmax;
		float pArcLength=arc*p;
		x=min_radius*cosf(3.14159265359f/2-pArcLength);
		y=min_radius*sinf(3.14159265359f/2-pArcLength);
	}
	else if((r<min_radius ||abs(x)<min_radius) && y<0)
	{
		float length=-y+arcLength;
		float p=x/min_radius;
		float subLength=p*length;
		if(abs(subLength)>arcLength)
		{
			x=copysignf(min_radius,x);
			y=-abs(subLength)+arcLength;
		}
		else
		{
			float angle=3.14159265359f/2*(1-subLength/arcLength);
			x=min_radius*cosf(angle);
			y=min_radius*sinf(angle);
		}
	}
	else if(r>max_radius)
	{
		x*=max_radius/r;
		y*=max_radius/r;
	}
    
    
    

    SCARA_pos[X_AXIS] = x;  //Translate cartesian to tower centric SCARA X Y AND apply scaling factor from this offset.
    SCARA_pos[Y_AXIS] = y;  // offset not to be confused with home offset. This makes the SCARA math work.
    // Y has to be scaled before subtracting offset to ensure position on bed.

    if (this->arm1_length == this->arm2_length)
        SCARA_C2 = (SQ(SCARA_pos[X_AXIS]) + SQ(SCARA_pos[Y_AXIS]) - 2.0f * SQ(this->arm1_length)) / (2.0f * SQ(this->arm1_length));
    else
        SCARA_C2 = (SQ(SCARA_pos[X_AXIS]) + SQ(SCARA_pos[Y_AXIS]) - SQ(this->arm1_length) - SQ(this->arm2_length)) / (2.0f * SQ(this->arm1_length));

    // SCARA position is undefined if abs(SCARA_C2) >=1
    // In reality abs(SCARA_C2) >0.95 can be problematic.




    SCARA_S2 = sqrtf(1.0f - SQ(SCARA_C2));

    SCARA_K1 = this->arm1_length + this->arm2_length * SCARA_C2;
    SCARA_K2 = this->arm2_length * SCARA_S2;

    SCARA_theta = (atan2f(SCARA_pos[X_AXIS], SCARA_pos[Y_AXIS]) - atan2f(SCARA_K1, SCARA_K2)) * -1.0f; // Morgan Thomas turns Theta in oposite direction
    SCARA_psi   = atan2f(SCARA_S2, SCARA_C2);

    actuator_mm[ALPHA_STEPPER] = to_degrees(SCARA_theta);             // Multiply by 180/Pi  -  theta is support arm angle
    actuator_mm[BETA_STEPPER ] = to_degrees(SCARA_theta + SCARA_psi); // kinematics (dual arm)
    //actuator_mm[BETA_STEPPER ] = to_degrees(SCARA_psi);             // real scara
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS]/z_mm_per_rotation*360.0-actuator_mm[Y_AXIS]; 
    
    if(actuator_mm[BETA_STEPPER]>210)   is_default_arm_mode=true; 
    if(actuator_mm[ALPHA_STEPPER]<-30)   is_default_arm_mode=false;
    
    
    if(!is_default_arm_mode)
    {
		actuator_mm[BETA_STEPPER] = to_degrees(SCARA_theta);             // Multiply by 180/Pi  -  theta is support arm angle
		actuator_mm[ALPHA_STEPPER] = to_degrees(SCARA_theta + SCARA_psi); // kinematics (dual arm)
		actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS]/z_mm_per_rotation*360.0-actuator_mm[Y_AXIS]; 
	}
}

void HeliosSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const
{
    // Perform forward kinematics, and place results in cartesian_mm[]

    float y1, y2,
          actuator_rad[2];

    actuator_rad[X_AXIS] = to_radians(actuator_mm[X_AXIS]);
    actuator_rad[Y_AXIS] = to_radians(actuator_mm[Y_AXIS]);

    y1 = sinf(actuator_rad[X_AXIS]) * this->arm1_length;
    y2 = sinf(actuator_rad[Y_AXIS]) * this->arm2_length + y1;

    cartesian_mm[X_AXIS] = cosf(actuator_rad[X_AXIS]) * this->arm1_length + cosf(actuator_rad[Y_AXIS]) * this->arm2_length  ;
    cartesian_mm[Y_AXIS] = y2;
    cartesian_mm[Z_AXIS] = (actuator_mm[Z_AXIS]+actuator_mm[Y_AXIS])/360.0*z_mm_per_rotation;

    cartesian_mm[0] = ROUND(cartesian_mm[0], 7);
    cartesian_mm[1] = ROUND(cartesian_mm[1], 7);
    cartesian_mm[2] = ROUND(cartesian_mm[2], 7);
}

bool HeliosSolution::set_optional(const arm_options_t& options)
{

    arm_options_t::const_iterator i;

    i = options.find('T');         // Theta arm1 length
    if(i != options.end()) {
        arm1_length = i->second;

    }
    i = options.find('P');         // Psi arm2 length
    if(i != options.end()) {
        arm2_length = i->second;
    }
    i = options.find('A');         // Psi arm2 length
    if(i != options.end()) {
        is_default_arm_mode = i->second;
    }
    /*i = options.find('X');         // Home initial position X
    if(i != options.end()) {
        offset_x = i->second;
    }
    i = options.find('Y');         // Home initial position Y
    if(i != options.end()) {
        offset_y = i->second;
    }
    i = options.find('A');         // Scaling X_AXIS
    if(i != options.end()) {
        scaling_x = i->second;
    }
    i = options.find('B');         // Scaling Y_AXIS
    if(i != options.end()) {
        scaling_y = i->second;
    }
    //i= options.find('C');          // Scaling Z_AXIS
    //if(i != options.end()) {
    //    scaling_z= i->second;
    //}
    i = options.find('D');         // Undefined min
    if(i != options.end()) {
        this->undefined_min = i->second;
    }
    i = options.find('E');         // undefined max
    if(i != options.end()) {
        this->undefined_max = i->second;
    }*/
    i= options.find('C');          // z_mm_per_rotation
    if(i != options.end()) {
        z_mm_per_rotation= i->second;
    }

    init();
    return true;
}

bool HeliosSolution::get_optional(arm_options_t& options, bool force_all) const
{
    options['T'] = this->arm1_length;
    options['P'] = this->arm2_length;
    options['A'] = this->is_default_arm_mode;
    //options['D'] = this->undefined_min;
    //options['E'] = this->undefined_max;

    return true;
};
