/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */

/* Open Source Software - may be modified and shared by FRC teams. The code */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project. */

/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.WristPid;
import frc.robot.subsystems.*;

public class MoveToHeight1 extends Command {

  private ArmSubsystem arm;
  private Robot robot;
  private int position;
  private double turnpower;
  private WristPid level;
  private int oldPosition;
  private int state;

  private boolean shouldInit;  
  private boolean wristDelay;

  private int pos;
  private float wristThresh;
  private int armThresh;
  private double wistAngle;
  private double targetAngle;
  private int targetEncoder;
  private int armEncoder;

  public MoveToHeight1(Robot r, int pos) {
    robot = r;
    requires(robot.arm_subsystem);
    arm = robot.arm_subsystem;
    position = pos;
    level = new WristPid(robot);
    state = 0;
    oldPosition = -1;
    wristDelay = false;
    shouldInit = true;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    shouldInit = false;
    pos = position + (arm.getToggly() && position != 0 ? 4 : 0);
    wristThresh = 2.5F;
    armThresh = 1000;
    targetAngle = arm.heights[1][pos];
    targetEncoder = arm.heights[0][pos];
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    int pos = position + (arm.getToggly() && position != 0 ? 4 : 0);
    armEncoder = robot.arm_subsystem.getArmEncoderTicks();
    wistAngle = robot.arm_subsystem.getWistAngle();
    turnpower = 0;

    if(shouldInit || oldPosition != pos)
      initialize();

    if(Math.abs(armEncoder - 16000) < armThresh && Math.abs(wistAngle - targetAngle) > wristThresh)
    {
      wristDelay = true;
      if(Math.abs(armEncoder - targetEncoder) < 1000)
      {
        pos = 3;
        oldPosition = -1;
      }
    }
    if(!wristDelay)
    {
      turnpower = level.GetAnglePidOutput(targetAngle);
    }
    arm.moveToHeightWrist(turnpower);

    if(Math.abs(armEncoder - targetEncoder) < 500)
      wristDelay = false;

    if(wristDelay || Math.abs(wistAngle - targetAngle) < wristThresh)
    {
      arm.moveToHeightPreset(pos);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // Keep holding the button in case the belt slips
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    state = 0;
    oldPosition = -1;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
      state = 0;
      oldPosition = -1;
  }
}
