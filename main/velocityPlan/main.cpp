#include <stdio.h>
#include "velocityPlan.h"

int main(void)
{
#ifdef USE_SERIAL_SEND
	serial_test();
#endif // USE_SERIAL_SEND	
	velocityPlanInit();
	
	while (1) {
	}
	return 0;
}
