// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SubsystemCatzClimb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;


public class SubsystemCatzClimb extends SubsystemBase {
  
  private final ClimbIO io;
  private static SubsystemCatzClimb instance;

  public SubsystemCatzClimb() {

            switch (CatzConstants.currentMode) {
            case REAL: io = 
                    new ClimbIOReal();
                break;

            case SIM : io = null;
                break;
            default : io = 
                    new ClimbIOReal() {};
                break;
        }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.exampleAccessMethod(0);
  }

  // Get the singleton instance of the ClimbSubsystem
  public static SubsystemCatzClimb getInstance() {
      return instance;
  }

}
