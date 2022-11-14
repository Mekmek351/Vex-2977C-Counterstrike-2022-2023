using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor FL_Drive;
extern motor BL_Drive;
extern motor FR_Drive;
extern motor BR_Drive;
extern encoder LeftEncoder;
extern encoder RightEncoder;
extern encoder HorizEncoder;
extern inertial Inertial1;
extern motor Intake;
extern motor Flywheel;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );