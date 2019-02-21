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

public class MoveToHeight extends Command {

  private ArmSubsystem arm;
  private Robot robot;
  public int position;
  private double turnpower;
  private WristPid level;

  public MoveToHeight(Robot r, int pos) {
    // Use requires() here to declare subsystem dependencies
    robot = r;
    requires(robot.armSubsystem);
    arm = robot.armSubsystem;
    position = pos;
    level = new WristPid(robot);
    level.SetTargetAngle(arm.heights[1][position]);
  }



  // Called just before this Command runs the first time

  @Override
  protected void initialize() {
    arm.moveToHeightPreset(position);
  }

  // Called repeatedly when this Command is scheduled to run

  @Override
  protected void execute() {
    turnpower = level.GetAnglePidOutput(Math.toDegrees(Math.atan2(robot.wristnavX.getQuaternionY(), robot.wristnavX.getQuaternionW())));
    arm.moveToHeightWrist(turnpower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //return arm.isTurnComplete(Math.asin(position / RobotMap.armLength)) && testCompleted;
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // arm.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

  @Override
  protected void interrupted() {

  }
}
