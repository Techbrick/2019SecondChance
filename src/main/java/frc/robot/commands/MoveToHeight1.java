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

  public MoveToHeight1(Robot r, int pos) {
    robot = r;
    requires(robot.arm_subsystem);
    arm = robot.arm_subsystem;
    position = pos;
    level = new WristPid(robot);
    state = 0;
    oldPosition = -1;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    int pos = position + (arm.getToggly() && position != 0 ? 4 : 0);
    float wristThresh = 2.5F;
    int armThresh = 500;
    double wistAngle = robot.arm_subsystem.getWistAngle();

    switch(state){
      case 0:
      {
        if(wistAngle > level.getTargetAngle() - wristThresh && wistAngle < level.getTargetAngle() + wristThresh)
        {
          state = 1;
        }
        level.SetTargetAngle(arm.heights[1][level.getTargetAngle() < -45 ? 1 : 2]);
        
      }
      case 1:
      {
        if(arm.getArmEncoderTicks()  >  arm.heights[0][pos] - armThresh && arm.getArmEncoderTicks()  <  arm.heights[0][pos] + armThresh)
        {
          state = 2;
        }
        if(pos != oldPosition){
          arm.moveToHeightPreset(pos);
          oldPosition = pos;
        }
      }
      default:
      {
          level.SetTargetAngle(arm.heights[1][position + (arm.getToggly() && position != 0 ? 4 : 0)]);
      }
    }
    // level.SetTargetAngle(arm.heights[1][position + (arm.getToggly() && position != 0 ? 4 : 0)]);
    // if(pos == 5)
       turnpower = level.GetAnglePidOutput(robot.arm_subsystem.getWistAngle());
    // else
    //   turnpower = level.GetAnglePidOutput(robot.arm_subsystem.getWistAngle());
    arm.moveToHeightWrist(turnpower);
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
