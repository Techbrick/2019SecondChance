/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */

/* Open Source Software - may be modified and shared by FRC teams. The code */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project. */

/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.WristPid;
import frc.robot.subsystems.*;
import frc.robot.Helpers;

public class MoveToHeight extends Command {

  private ArmSubsystem arm;
  private Robot robot;
  public int position;
  private boolean testCompleted = false;
  private double turnpower;
  private int stoppedCounter = 0;
  private WristPid level;
  private Helpers helper;

  public MoveToHeight(Robot r, int pos) {
    // Use requires() here to declare subsystem dependencies
    robot = r;
    requires(robot.arm_subsystem);
    arm = robot.arm_subsystem;
    position = pos;
    level = new WristPid(robot);
    helper = new Helpers();
    level.SetTargetAngle(robot.robotMap.heights[1][position]);
  }
  // Called just before this Command runs the first time

  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run

  @Override
  protected void execute() {
    turnpower = level.GetAnglePidOutput(Math.atan2(robot.wristnavX.getQuaternionW(),robot.wristnavX.getQuaternionY()) * 180 / 3.14);
    if (turnpower == 0) {
      stoppedCounter++;
    } else {
      stoppedCounter = 0;
    }
    if (stoppedCounter > 5) {
      testCompleted = true;
    }
    arm.moveToHeightPreset(position, turnpower);
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
