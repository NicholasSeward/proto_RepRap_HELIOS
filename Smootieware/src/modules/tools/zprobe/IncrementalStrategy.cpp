

#include "IncrementalStrategy.h"

#include "Kernel.h"
#include "Config.h"
#include "Robot.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "Conveyor.h"
#include "ZProbe.h"
#include "nuts_bolts.h"
#include "utils.h"
#include "platform_memory.h"

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <fastmath.h>

#define max_coordinate_checksum      CHECKSUM("size")
#define grid_size_checksum           CHECKSUM("grid_size")
#define max_count_checksum           CHECKSUM("max_count")


IncrementalStrategy::IncrementalStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
    x_points = nullptr;
    y_points = nullptr;
    z_points = nullptr;
    grid = nullptr;
    probe_count=0;
}

IncrementalStrategy::~IncrementalStrategy()
{
    if(x_points != nullptr) AHB0.dealloc(x_points);
    if(y_points != nullptr) AHB0.dealloc(y_points);
    if(z_points != nullptr) AHB0.dealloc(z_points);
    if(grid != nullptr) AHB0.dealloc(grid);
    
}

bool IncrementalStrategy::handleConfig()
{
	size=THEKERNEL->config->value(leveling_strategy_checksum, incremental_leveling_strategy_checksum, max_coordinate_checksum)->by_default(300.0f)->as_number();
	grid_size=THEKERNEL->config->value(leveling_strategy_checksum, incremental_leveling_strategy_checksum, grid_size_checksum)->by_default(24)->as_number();
	max_count=THEKERNEL->config->value(leveling_strategy_checksum, incremental_leveling_strategy_checksum, max_count_checksum)->by_default(500)->as_number();
    probe_count=0;

    // allocate in AHB0
    x_points = (float *)AHB0.alloc(max_count * sizeof(float));
    y_points = (float *)AHB0.alloc(max_count * sizeof(float));
    z_points = (float *)AHB0.alloc(max_count * sizeof(float));
    grid = (float *)AHB0.alloc(grid_size*grid_size * sizeof(float));

    if(grid == nullptr) {
        THEKERNEL->streams->printf("Error: Not enough memory\n");
        return false;
    }

    fill_grid();

    return true;
}





bool IncrementalStrategy::handleGcode(Gcode *gcode)
{
    if(gcode->has_g) {
        if( gcode->g == 32) { // do a grid probe
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_idle();

            if(!doProbe(gcode)) {
                gcode->stream->printf("Probe failed to complete, check the initial probe height and/or initial_height settings\n");
            } else {
                gcode->stream->printf("Probe completed.\n");
                for(int i=0;i<probe_count;i++)
				{
					float px=x_points[i];
					float py=y_points[i];
					float pz=z_points[i];
					gcode->stream->printf("Point #%d: %f,%f,%f\n",i,px,py,pz);
				}
			}
			return true;
        }

    }
    else if(gcode->has_m) {
        if(gcode->m == 370 || gcode->m == 561) { // M370: Clear bed, M561: Set Identity Transform
            // delete the compensationTransform in robot
            setAdjustFunction(false);
            probe_count=0;
            gcode->stream->printf("grid cleared and disabled\n");
            return true;

        }
    }

    return false;
}


void IncrementalStrategy::setAdjustFunction(bool on)
{
    if(on) {
        THEROBOT->compensationTransform = [this](float *target,bool invert) { doCompensation(target,invert); };
    } else {
        THEROBOT->compensationTransform = nullptr;
    }
}

bool IncrementalStrategy::doProbe(Gcode *gc)
{
	THECONVEYOR->wait_for_idle();
    gc->stream->printf("Incremental Probe...\n");

    setAdjustFunction(false);
    float mm;
    
    /*
     * float x= actuators[0]->get_last_milestone(), y= actuators[1]->get_last_milestone(),z= actuators[2]->get_last_milestone();
                    if(gcode->has_letter('X')) x= gcode->get_value('X');
                    if(gcode->has_letter('Y')) y= gcode->get_value('Y');
                    if(gcode->has_letter('Z')) z= gcode->get_value('Z');
                    ActuatorCoordinates real_actuator_position = {x,y,z};
					THEROBOT->reset_actuator_position(real_actuator_position);
					*/
    
    float pos[3];
    THEROBOT->get_current_machine_position(pos);
    if(!zprobe->run_probe(mm, 6000,100,false)) return false;
    //float z_reference = zprobe->getProbeHeight() - mm; // this should be zero
    //gc->stream->printf("probe at %f,%f is %f mm\n", pos[0], pos[1],z_reference);
    x_points[probe_count]=pos[0];
    y_points[probe_count]=pos[1];
    z_points[probe_count]=pos[2]-mm-zprobe->getProbeHeight();
    probe_count++;
    //fill_grid();
    
    setAdjustFunction(true);

    return true;
}

void IncrementalStrategy::doCompensation(float *target,bool invert)
{
	float offset=interpolateOffset(target[X_AXIS], target[Y_AXIS]);
	if(!invert)target[Z_AXIS]+=offset;
	else target[Z_AXIS]-=offset;
}

float IncrementalStrategy::calculateOffset(float x, float y)
{
	float cell_size=2*size/grid_size;
	int i=floorf((x+size)/cell_size);
	int j=floorf((y+size)/cell_size);
	float ratio_x = x - i*cell_size;
    float ratio_y = y - j*cell_size;
	float z1 = grid[i + j*grid_size];
	float z2 = grid[i+ (j + 1) * grid_size];
	float z3 = grid[i+1 +j * grid_size];
	float z4 = grid[i+1+ (j + 1) * grid_size];
	float left = (1 - ratio_y) * z1 + ratio_y * z2;
	float right = (1 - ratio_y) * z3 + ratio_y * z4;
	float offset = (1 - ratio_x) * left + ratio_x * right;
	return offset;
}

float IncrementalStrategy::interpolateOffset(float x, float y)
{
	if(probe_count==0)return 0;
	float total_weight=0;
	float offset=0;
	
	for(int i=0;i<probe_count;i++)
	{
		float px=x_points[i];
		float py=y_points[i];
		float pz=z_points[i];
		px-=x;
		py-=y;
		px/=size;
		py/=size;
		float weight=1/(px*px+py*py+.0001f);
		total_weight+=weight;
		offset+=weight*pz;		
	}
	
	return offset/total_weight;
}


// Reset calibration results to zero.
void IncrementalStrategy::fill_grid()
{
	float cell_size=2*size/grid_size;
    for (int j = 0; j < grid_size; j++) {
        for (int i = 0; i < grid_size; i++) {
            grid[i + (grid_size * j)] = interpolateOffset(i*cell_size-size,j*cell_size-size);
        }
    }
}
