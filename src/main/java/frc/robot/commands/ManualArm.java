/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArm extends Command {
  Robot _robot;
  public ManualArm(Robot r) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    _robot = r;
    requires(_robot.arm_subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(_robot.stick.getRawButton(5)) {
      _robot.arm_subsystem.rotate(-1);
    } else if(_robot.stick.getRawButton(6)) {
      _robot.arm_subsystem.rotate(1);
    } else {
      _robot.arm_subsystem.rotate(0);
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
    _robot.arm_subsystem.rotate(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}