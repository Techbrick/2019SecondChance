/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeBall extends Command {
  private Robot _robot;
  private ArmSubsystem arm;
  private double intakeSpeed;
  private boolean pullIn;
  private boolean isFinished;

  public IntakeBall(Robot robot, boolean willIntake) {
    _robot = robot;
    pullIn = willIntake;
    // Use requires() here to declare subsystem dependencies
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    arm = _robot.arm_subsystem;
    intakeSpeed = 1.0D;
    isFinished = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(pullIn){
      if(!_robot.DI.get()){
        arm.setIntakeSpeed(-intakeSpeed);
      } 
      else{
        isFinished = true;
      }
    }
    else
      arm.setIntakeSpeed(intakeSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    _robot.arm_subsystem.setIntakeSpeed(-0.5D);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    _robot.arm_subsystem.setIntakeSpeed(0.0D);
  }
}
