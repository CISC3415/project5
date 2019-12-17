/*
 *  CISC-3415 Robotics
 *  Project 4 - Part 2
 *  Credit To: Simon Parsons
 *
 ** Group Members *************************************************************
 *    
 *  Benjamin Yi
 *  Emmanuel Desdunes
 *  Montasir Omi
 *  Shahzad Ahmad
 *
 ** Description ***************************************************************
 * 
 *  Simulated robot reads from a travel plan (plan.txt) and travels
 *  through each waypoint until it arrives at its goal destination.
 */


#include <iostream>
#include <fstream>
#include <libplayerc++/playerc++.h>
using namespace PlayerCc;  


/**
 * Function headers
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);
void printLaserData(LaserProxy& sp);

int  readPlanLength(void);
void readPlan(double *, int);
void printPlan(double *,int);  
void writePlan(double *, int);
void createPlan(); 
/**
 * main()
 *
 **/

int main(int argc, char *argv[])
{  

  // Variables
  int counter = 0;
  double speed;            // How fast do we want the robot to go forwards?
  double turnrate;         // How fast do we want the robot to turn?
  player_pose2d_t  pose;   // For handling localization data

  // The set of coordinates that makes up the plan

  int pLength;
  double *plan;
  int curr_pos = 0;
  int started = 1, arrived = 0, bumped = 0;
  int finding_angle = 0, traveling = 0;
  double curr_x, curr_y, curr_a;
  double targ_x=0, targ_y=0, targ_a=0;
  int turn_sign = 1;
  double angle_away, dist_away, dx, dy;
  // Set up proxies. These are the names we will use to connect to 
  // the interface to the robot.
  PlayerClient    robot("localhost");  
  BumperProxy     bp(&robot,0);  
  Position2dProxy pp(&robot,0);
  LocalizeProxy   lp (&robot, 0);
  LaserProxy      sp (&robot, 0);
  
  // Allow the program to take charge of the motors (take care now)
  pp.SetMotorEnable(true);

  // Plan handling
  // 
  // A plan is an integer, n, followed by n doubles (n has to be
  // even). The first and second doubles are the initial x and y
  // (respectively) coordinates of the robot, the third and fourth
  // doubles give the first location that the robot should move to, and
  // so on. The last pair of doubles give the point at which the robot
  // should stop.
  createPlan();               // Create custom plan
  pLength = readPlanLength(); // Find out how long the plan is from plan.txt
  plan = new double[pLength]; // Create enough space to store the plan
  readPlan(plan, pLength);    // Read the plan from the file plan.txt.
  printPlan(plan,pLength);    // Print the plan on the screen
  writePlan(plan, pLength);   // Write the plan to the file plan-out.txt
  // Main control loop
  while(true) 
    {    
      // Update information from the robot.
      robot.Read();
      // Read new information about position
      pose = readPosition(lp);
      // Print data on the robot to the terminal
      // printRobotData(bp, pose);
      // Print information about the laser. Check the counter first to stop
      // problems on startup
      if(counter > 2){
	      printLaserData(sp);
      }

      // Print data on the robot to the terminal --- turned off for now.
      // printRobotData(bp, pose);
      
      // Current position data
      curr_x = pose.px;
      curr_y = pose.py;
      curr_a = pose.pa;
      
      // Target position data
      targ_x = plan[curr_pos];
      targ_y = plan[curr_pos+1];
      targ_a = atan2(targ_y-curr_y, targ_x-curr_x);
      
      // Angle required to face the target destination
      angle_away = rtod(targ_a)-rtod(curr_a);

      // Backs up if bumped until safe, and begins scanning for next angle
      if (bumped) {
        if (counter < 15) {
          speed = -0.5;
        } else if (counter < 25) {
          speed = 0.5;
        } else if (counter < 40) {
          turnrate = 0.0;
        } else {
          counter = 0;
          bumped = 0;
          speed = 0;
          finding_angle = 1;
          traveling = 0;
          arrived = 0;
          turnrate = 0.0;
        }
        std::cout << counter << std::endl;
        counter++;
      // Finds angle required to go to the next point in the plan
      } else if (finding_angle) {
        // If the angle is good enough, start traveling
        if (abs(angle_away) < 2) {
          turnrate = 0;
          speed = 1.0;
          finding_angle = 0;
          traveling = 1;
        } else {
          // If the angle is not good enough, keep adjusting.
          std::cout << "Angle Away: " << angle_away << std::endl; 
          if (angle_away < 0) turnrate = -0.4;
          else turnrate = 0.4;
          speed = 0;
        }
      // Travels until distance is good enough
      } else if (traveling) {
        dx = curr_x-targ_x;
        dy = curr_y-targ_y;
        dist_away = sqrt(dx*dx+dy*dy);
        speed = 1.0;
        if (angle_away < 0) turnrate = -0.4;
        else turnrate = 0.4;
        // Stop if distance is close enough
        if (dist_away < 0.5) {
          arrived = 1;
          speed = 0;
          traveling = 0;
        }
      // If arrived at correct location, start finding the next location
      } else if (arrived) {
        speed = 0.0;
        turnrate = 0.0;
        curr_pos += 2;
        // If the robot is at its final location, stop
        if (curr_pos == pLength) {
          std::cout << "Successfully arrived at location!" << std::endl << std::endl;
          pp.SetSpeed(0, 0);
          break;
        }
        finding_angle = 1;
        arrived = 0;
      } else if (started) {
        started = 0;
        turnrate = 0;
        speed = 0;
        finding_angle = 1;
      }
      // If bumped, start backing up
      if (bp[0] || bp[1] || pp.GetStall()) {
        if (bp[0]) turn_sign = -1;
        if (bp[1]) turn_sign = 1;
        turnrate = 0.4 * turn_sign;
        bumped = 1;
      }
      // What are we doing?
      std::cout << "Speed: " << speed << std::endl;      
      std::cout << "Turn rate: " << turnrate << std::endl << std::endl;

      // Location data
      std::cout << "Heading to location: (" << plan[curr_pos] << ", " << plan[curr_pos+1] << ")" << std::endl;
      std::cout << "Current X: " << curr_x << std::endl;
      std::cout << "Current Y: " << curr_y << std::endl;
      std::cout << "Current A: " << curr_a << std::endl;
      std::cout << "Target X:  " << targ_x << std::endl;
      std::cout << "Target Y:  " << targ_y << std::endl;
      std::cout << "Target A:  " << targ_a << std::endl << std::endl;

      // Send the commands to the robot
      pp.SetSpeed(speed, turnrate);  
    }
} // end of main()

/**
 * readPosition()
 *
 * Read the position of the robot from the localization proxy. 
 *
 * The localization proxy gives us a hypothesis, and from that we extract
 * the mean, which is a pose. 
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp)
{

  player_localize_hypoth_t hypothesis;
  player_pose2d_t          pose;
  uint32_t                 hCount;

  // Need some messing around to avoid a crash when the proxy is
  // starting up.

  hCount = lp.GetHypothCount();

  if(hCount > 0){
    hypothesis = lp.GetHypoth(0);
    pose       = hypothesis.mean;
  }

  return pose;
} // End of readPosition()


void printLaserData(LaserProxy& sp)
{

  double maxRange, minLeft, minRight, range, bearing;
  int points;

  maxRange  = sp.GetMaxRange();
  minLeft   = sp.MinLeft();
  minRight  = sp.MinRight();
  range     = sp.GetRange(5);
  bearing   = sp.GetBearing(5);
  points    = sp.GetCount();

  //Uncomment this to print out useful laser data
  //std::cout << "Laser says..." << std::endl;
  //std::cout << "Maximum distance I can see: " << maxRange << std::endl;
  //std::cout << "Number of readings I return: " << points << std::endl;
  //std::cout << "Closest thing on left: " << minLeft << std::endl;
  //std::cout << "Closest thing on right: " << minRight << std::endl;
  //std::cout << "Range of a single point: " << range << std::endl;
  //std::cout << "Bearing of a single point: " << bearing << std::endl;

  return;
} // End of printLaserData()

/**
 *  printRobotData
 *
 * Print out data on the state of the bumpers and the current location
 * of the robot.
 *
 **/

void printRobotData(BumperProxy& bp, player_pose2d_t pose)
{

  // Print out what the bumpers tell us:
  std::cout << "Left  bumper: " << bp[0] << std::endl;
  std::cout << "Right bumper: " << bp[1] << std::endl;
  // Can also print the bumpers with:
  //std::cout << bp << std::endl;

  // Print out where we are
  std::cout << "We are at" << std::endl;
  std::cout << "X: " << pose.px << std::endl;
  std::cout << "Y: " << pose.py << std::endl;
  std::cout << "A: " << pose.pa << std::endl;

  
} // End of printRobotData()

/**
 * readPlanLength
 *
 * Open the file plan.txt and read the first element, which should be
 * an even integer, and return it.
 *
 **/

int readPlanLength(void)
{
  int length;

  std::ifstream planFile;
  planFile.open("plan.txt");

  planFile >> length;
  planFile.close();

  // Some minimal error checking
  if((length % 2) != 0){
    std::cout << "The plan has mismatched x and y coordinates" << std::endl;
    exit(1);
  }

  return length;

} // End of readPlanLength

/**
 * readPlan
 *
 * Given the number of coordinates, read them in from plan.txt and put
 * them in the array plan.
 *
 **/

void readPlan(double *plan, int length)
{
  int skip;

  std::ifstream planFile;
  planFile.open("plan.txt");

  planFile >> skip;
  for(int i = 0; i < length; i++){
    planFile >> plan[i];
  }

  planFile.close();

} // End of readPlan

/**
 * printPlan
 *
 * Print the plan on the screen, two coordinates to a line, x then y
 * with a header to remind us which is which.
 *
 **/

void printPlan(double *plan , int length)
{
  std::cout << std::endl;
  std::cout << "   x     y" << std::endl;
  for(int i = 0; i < length; i++){
    std::cout.width(5);
    std::cout << plan[i] << " ";
    if((i > 0) && ((i % 2) != 0)){
      std::cout << std::endl;
    }
  }
  std::cout << std::endl;

} // End of printPlan


/**
 * writePlan
 * 
 * Send the plan to the file plan-out.txt, preceeded by the information
 * about how long it is.
 *
 **/

void writePlan(double *plan , int length)
{
  std::ofstream planFile;
  planFile.open("plan-out.txt");

  planFile << length << " ";
  for(int i = 0; i < length; i++){
    planFile << plan[i] << " ";
  }

  planFile.close();

} // End of writePlan

void createPlan() {
  std::ofstream ofs;
  ofs.open("plan.txt");
  ofs << "12 -2.5 -6 -2.5 1.5 -1.5 2.5 2.5 3.5 3.5 5.5 6.5 6.5";
  ofs.close();
}
