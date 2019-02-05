/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.AccelerometerSubsystem;


public class AccelerometerAngle extends InstantCommand {
  Robot _robot;
  AccelerometerSubsystem acc;
  public AccelerometerAngle(Robot r) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.accelerometer_subsystem);
    _robot = r;
    acc = Robot.accelerometer_subsystem;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      acc.getAngle();
  }

 }
