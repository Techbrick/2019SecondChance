/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ShiftGear extends Command {
  private Robot _robot;
  public ShiftGear(Robot robot) {
    _robot = robot;
    requires(_robot.comp_subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _robot.comp_subsystem.checkPressure();  //Starts compressor
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(_robot.stick.getRawButton(2) && !_robot.comp_subsystem.pressure()) {
      _robot.comp_subsystem.actuateCylinder();  //Shifts to High Gear
    }
    else {
      _robot.comp_subsystem.retractCylinder();
      _robot.comp_subsystem.checkPressure();
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
