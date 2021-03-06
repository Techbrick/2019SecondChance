/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class HatchEjector extends InstantCommand {
  
  private boolean shouldEject;
  private Robot robot;

  public HatchEjector(Robot parRobot, Boolean parEject) {
    // Use requires() here to declare subsystem dependencies
    super();
    robot = parRobot;
    requires(robot.arm_subsystem);
    shouldEject = parEject;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    robot.arm_subsystem.setHatchEjector(shouldEject);
  }

  // // Called repeatedly when this Command is scheduled to run
  // @Override
  // protected void execute() {
  //   //shouldEject = !shouldEject;
  // }

  // // Make this return true when this Command no longer needs to run execute()
  // @Override
  // protected boolean isFinished() {
  //   return true;
  // }

  // // Called once after isFinished returns true
  // @Override
  // protected void end() {
  // }

  // // Called when another command which requires one or more of the same
  // // subsystems is scheduled to run
  // @Override
  // protected void interrupted() {
  // }
}
