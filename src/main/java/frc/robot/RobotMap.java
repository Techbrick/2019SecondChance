/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public final int driveStick = 0;
  public final int opStick = 1;
  public double timingInterval = .02;
  public static boolean verbose = false;

  // CAN IDs
  public final int leftMaster = 4;
  public final int leftFollower1 = 5;
  public final int leftFollower2 = 6;
  public final int rightMaster = 1;
  public final int rightFollower1 = 2;
  public final int rightFollower2 = 3;

  public static final int armMasterLeft1 = 7;
  public static final int armFollowerRight1 = 8;
  public static final int wristMotor1 = 9;
  public static final int intakeMotor1 = 10;

  public final int pigeonID = 11;

  // Values
  public static int desiredHeight = 0;
  public static int toggle = 0;

  //PID
  public double kAngleSetpoint = 0.0;
  public double kp_Angle = 0.02;
  public double ki_Angle = 0.00;
  public double kd_Angle = 0.00;
  public double kp_Angle_Wrist = 0.04;
  public double ki_Angle_Wrist = 0.0;
  public double kd_Angle_Wrist = 0.0;
  public double minWristPower = 0.05;
  public double maxWristPower = 1;
  public double joystickDeadband = 0.05;
  public double pidTurnDeadband = 1;
  public double pidWristDeadband = 1;

  public double distanceSetpoint = 0.0;
  public double kp_distance = 0.025; 
  public double ki_distance = 0.00; 
  public double kd_distance = 0.00; 
  public double pidDistDeadband = .2;

  public double encoderMovementThreshold = 200;

  public double minTurnPower = .1; //WAS .24
  public double minDrivePower = .12;
  public double maxPidPower = .5;
  public double maxVelocity = 10;
  public double maxAccel = 15;
  public double fpsPerVolt = .8;
  public double accelPerVolt = 15.0/12.0;
  public int averageCounterVel = 0;
  public int averageCounterAccel = 0;
  public double KpDistanceFollower = .8;

  // Measurements
  public static final double WHEEL_DIAMETER = 6;
  public static final double ENCODER_PULSE_PER_REV = 4096.0; //WAS 2048
  public static final double driveEncoderTicksPerInch = 437.42;
  public static final double trackWidth = 24;
  public static final int armLength = 26; //real one is 38, 26 is for testbot
  public static final double ArmTicksToDeg = 360.0 / ENCODER_PULSE_PER_REV / 25.0;

  // DEPRECATED
  // public static final int wristHatchSolenoidID = 9;
  // public static final int mc_arm_CANID = 2;
  // public final int maxWristAngle = 130;
  // public final int minWristAngle = 0;
  // public boolean twoSpeedDrive = false;
  // public boolean hasCompressor = false;
  // public int leftShiftChannel = 0;
  // public int rightShiftChannel = 0;
}
