/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeBall extends Command {
  private Robot _robot;
  public IntakeBall(Robot robot) {
    _robot = robot;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(_robot.arm_subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(_robot.stick.getRawButton(11)) {
      _robot.arm_subsystem.intakeBall(-1);
    }
    else if(_robot.stick.getRawButton(12)) {
      _robot.arm_subsystem.intakeBall(1);
    }
    else {
      _robot.arm_subsystem.intakeBall(0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
