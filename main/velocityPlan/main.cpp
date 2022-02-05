#include <stdio.h>
#include "velocityPlan.h"
#include "kinematics.h"

int main(void)
{
#ifdef USE_SERIAL_SEND
	serial_test();
#endif // USE_SERIAL_SEND	
	//velocityPlanInit();

	kinematicsDemo();
	while (1) {
	}
	return 0;
}
