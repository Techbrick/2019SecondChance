/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class LiftRobot extends InstantCommand {
  //Gives the robot that extra oomph
  private boolean shouldOpen;
  private Robot robot;
  public LiftRobot(Robot r, boolean parShouldOpen) {
    super();
    robot = r;
    requires(robot.driveTrain);
    shouldOpen = parShouldOpen;
  }

  @Override
  protected void initialize() {
    robot.driveTrain.setLifterSolenoid(shouldOpen);
  }
}
