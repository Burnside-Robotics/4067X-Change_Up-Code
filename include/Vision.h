/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature RedBall = vex::vision::signature (1, 2495, 8121, 5308, -1817, 71, -874, 1.5, 0);
vex::vision::signature BlueBall = vex::vision::signature (2, -2831, -47, -1438, 1537, 9775, 5656, 0.9, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision FrontVision = vex::vision (vex::PORT6, 50, RedBall, BlueBall, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/