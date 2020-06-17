/*vex-vision-config:begin*/
vex::vision::signature RedBall = vex::vision::signature (1, 7203, 12003, 9602, -2791, -1123, -1956, 1, 0);
vex::vision::signature BlueBall = vex::vision::signature (2, -1031, 433, -298, -1, 5125, 2562, 0.8, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision IntakeVision = vex::vision (vex::PORT16, 50, RedBall, BlueBall, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/