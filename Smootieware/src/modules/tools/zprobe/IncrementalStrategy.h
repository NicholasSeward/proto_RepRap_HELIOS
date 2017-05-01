#pragma once



#include "LevelingStrategy.h"



#include <string.h>
#include <tuple>

#define incremental_leveling_strategy_checksum CHECKSUM("incremental")

class StreamOutput;
class Gcode;

class IncrementalStrategy : public LevelingStrategy
{
public:
    IncrementalStrategy(ZProbe *zprobe);
    ~IncrementalStrategy();
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

private:

	float *x_points;
	float *y_points;
	float *z_points;
	float *grid;
	int probe_count;
	float size;
	int grid_size;
	int max_count;
	
	void fill_grid();
	float calculateOffset(float x, float y);
	float interpolateOffset(float x, float y);
	void setAdjustFunction(bool on);
    void doCompensation(float *target,bool invert);
	bool doProbe(Gcode *gc);

};
