// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(18.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ElevatorConstants{
    //Elevator Math
    public static final int COUNTS_PER_ROTATION = 630; // 3402 but 630 rn because no 2nd 9:1 gear thing
    public static final double MAX_EXTENTION_INCHES = 22.875; // 22.875
    public static final double INCHES_PER_ROTATION = 0.28;
    public static final double COUNTS_PER_INCH = COUNTS_PER_ROTATION/INCHES_PER_ROTATION;

    //Wrist Math
    public static final int WRIST_COUNTS_PER_ROTATION = 1050; //126
    public static final int MAX_DEGREES = 0; //need to find
    public static final double DEGREES_PER_ROTATION = 4.8; // 360
    public static final double COUNTS_PER_DEGREE = WRIST_COUNTS_PER_ROTATION/DEGREES_PER_ROTATION;
    
    
    //Elevator Set Points
    public static final int HANGING_LEVEL =(int)(1.5 * COUNTS_PER_INCH);
    public static final int STOWED_LEVEL = (int)(1.5 * COUNTS_PER_INCH);
    public static final int CORAL_STATION = (int)(3.25 * COUNTS_PER_INCH);
    public static final int LEVEL_ONE = (int)(1.5 * COUNTS_PER_INCH);
    public static final int LEVEL_TWO = (int)(4 * COUNTS_PER_INCH); // 11.75 in
    public static final int LEVEL_THREE = (int)(21.375 * COUNTS_PER_INCH);; // 22.875 in

    //Wrist Set Points
    public static final int STOWED_ANGLE = 0; 
    public static final int CORAL_ANGLE = -(int)(76 * COUNTS_PER_DEGREE); // aprx. 50° NEEDS TO BE INVERTED
    public static final int SCORING_ANGLE = -(int)(28 * COUNTS_PER_DEGREE); // aprx. 25° 
    public static final int HANGING_ANGLE = -(int)(185 * COUNTS_PER_DEGREE);

    //Intake Speeds
    public static final double INTAKE_IN = 0.3; //Maybe needs to be reversed
    public static final double INTAKE_STOP = 0.0;
    public static final double INTAKE_OUT = -0.3;
  }

  public static class AlgaeConstants{
    public static final int DEGREES_PER_ROT = 2;
    
    
    //arm limits
    public static final int MAX_HEIGHT = 0;
    public static final int MIN_HEIGHT = -20; //change when determined
  
   //Intake Speeds
    public static final double INTAKE_IN = 0.5; //Maybe needs to be reversed
    public static final double INTAKE_STOP = 0.0;
    public static final double INTAKE_OUT = -0.5;
  
  }

  public static class HangerConstants{
    //Hanger Math
    public static final int COUNTS_PER_ROTATION = 30618;
    public static final double DEGREES_PER_ROTATION = 0.49382716049;
    public static final double COUNTS_PER_DEGREE = (double)(COUNTS_PER_ROTATION/DEGREES_PER_ROTATION);
  
    //Hanger Setpoints
    public static final int STAGE_ZERO = (int)(0 * COUNTS_PER_DEGREE);
    public static final int STAGE_ONE = -(int)(10* COUNTS_PER_DEGREE);
    public static final int STAGE_TWO = (int)(18 * COUNTS_PER_DEGREE);
  }

  public static class VisionConstants{
    public static final double X_REEF_ALIGNMENT_P = 0.3;
    public static final double Y_REEF_ALIGNMENT_P = 0.3;
    public static final double ROT_REEF_ALIGNMENT_P = 0.0058;
  
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34;  // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;  // Horizontal pose
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;
  
    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;
  }
  
}
