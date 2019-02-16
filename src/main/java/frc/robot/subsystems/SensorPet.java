/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class SensorPet extends Subsystem {
  private DigitalInput bit1 = new DigitalInput(6);
  private DigitalInput bit2 = new DigitalInput(7);
  private DigitalInput bit3 = new DigitalInput(8);
  private DigitalInput bit4 = new DigitalInput(9);
  private int distance = 0;
  
  public SensorPet(){
    distance = (bit4.get()?8:0) + (bit3.get()?4:0) + (bit2.get()?2:0) + (bit1.get()?1:0);
  }

  public int getDistance(){
    return distance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
