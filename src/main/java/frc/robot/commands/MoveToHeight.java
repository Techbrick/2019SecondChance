/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */

/* Open Source Software - may be modified and shared by FRC teams. The code */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project. */

/*----------------------------------------------------------------------------*/



package frc.robot.commands;



import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;

import frc.robot.subsystems.*;

/**

* An example command. You can replace me with your own command.

*/

public class MoveToHeight extends Command {

  private ArmSubsystem arm;
  private Robot robot;
  public int currentencoder;
  public int targetencoder = 0;

  public MoveToHeight(Robot r, int pos) {
  // Use requires() here to declare subsystem dependencies
  requires(Robot.arm_subsystem);
  arm = Robot.arm_subsystem;
  targetencoder = arm.getEncoderTicks();

  }



  // Called just before this Command runs the first time

  @Override

  protected void initialize() {

  currentencoder = arm.getEncoderTicks();

  }



  // Called repeatedly when this Command is scheduled to run

  @Override
  protected void execute() {
    // if(currentencoder>targetencoder)
    // {
    //   arm.moveDown();
    // }
    // else if(currentencoder<targetencoder)
    // {
    //   arm.moveUp();
    // }
    // currentencoder = arm.getEncoderTicks();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return currentencoder==targetencoder;
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
