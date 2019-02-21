/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ResetAutoArm extends Command {
  private Robot _robot;
  private boolean finished = false;

  public ResetAutoArm(Robot robot) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    _robot = robot;
    requires(robot.armSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _robot.armSubsystem.moveToHeightWrist(0.5);
    _robot.armSubsystem.setArmSpeed(-0.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(_robot.armSubsystem.getWristSpeed() == 0 && _robot.armSubsystem.getArmSpeed() == 0){
      _robot.armSubsystem.wristStartAngle = (int)Math.toDegrees(Math.atan2(_robot.wristnavX.getQuaternionY(), _robot.wristnavX.getQuaternionW()));
      _robot.armSubsystem.setHeights();
      finished = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
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
