/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeBall extends Command {
  private Robot _robot;
  private static ArmSubsystem arm;
  private boolean out = false;

  private double intakeSpeed;
  boolean pullIn;
  public IntakeBall(Robot robot, boolean willIntake) {
    _robot = robot;
    pullIn = willIntake;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(_robot.arm_subsystem);
    DigitalInput DO = new DigitalInput(1);
    out = DO.get();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    arm = _robot.arm_subsystem;
    intakeSpeed = 1.0D;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(pullIn){
      if(!out)
      _robot.arm_subsystem.setIntakeSpeed(_robot.arm_subsystem.getIntakeSpeed() + intakeSpeed * -1);
    }
    else
      _robot.arm_subsystem.setIntakeSpeed(_robot.arm_subsystem.getIntakeSpeed() + intakeSpeed);
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
    _robot.arm_subsystem.setIntakeSpeed(0.0D);
  }
}
