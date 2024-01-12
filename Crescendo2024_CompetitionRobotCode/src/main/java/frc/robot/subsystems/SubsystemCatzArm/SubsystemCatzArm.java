// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SubsystemCatzArm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;


public class SubsystemCatzArm extends SubsystemBase {
  
  private final ArmIO io;
  private static SubsystemCatzArm instance;

  public SubsystemCatzArm() {

            switch (CatzConstants.currentMode) {
            case REAL: io = 
                    new ArmIOReal();
                break;

            case SIM : io = null;
                break;
            default : io = 
                    new ArmIOReal() {};
                break;
        }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.exampleAccessMethod(0);
  }

  // Get the singleton instance of the ClimbSubsystem
  public static SubsystemCatzArm getInstance() {
      return instance;
  }

}
