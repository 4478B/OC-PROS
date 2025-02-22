#ifndef AUTON_ROUTES_H
#define AUTON_ROUTES_H

extern int autonSection;

void drivePIDOdom(double goalInches, bool clamping = false, double clampDistInches = 2);
void endSection(int delay = 0);
void progSkills();
void LocalFullAWP(int i);
void blueGoalSideSugarRush(int i);
void redGoalSideSugarRush(int i);


void ringRush(int mode);
void safeAWP(int mode);
#endif // AUTON_ROUTES_H