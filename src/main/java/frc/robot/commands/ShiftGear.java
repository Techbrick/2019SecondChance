/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ShiftGear extends InstantCommand {
  private Robot _robot;
  public boolean shifty;
  public ShiftGear(Robot robot, boolean shift) {
    _robot = robot;
    requires(_robot.driveTrain);
    shifty = shift;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _robot.comp_subsystem.checkPressure();  //Starts compressor
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      _robot.driveTrain.setShifterSolenoid(shifty);
     //Shifts to High Gear
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
}
