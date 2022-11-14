#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FL_Drive = motor(PORT6, ratio18_1, false);
motor BL_Drive = motor(PORT2, ratio18_1, false);
motor FR_Drive = motor(PORT3, ratio18_1, true);
motor BR_Drive = motor(PORT4, ratio18_1, true);
encoder LeftEncoder = encoder(Brain.ThreeWirePort.A);
encoder RightEncoder = encoder(Brain.ThreeWirePort.C);
encoder HorizEncoder = encoder(Brain.ThreeWirePort.E);
inertial Inertial1 = inertial(PORT5);
motor Intake = motor(PORT8, ratio6_1, false);
motor Flywheel = motor(PORT9, ratio6_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}