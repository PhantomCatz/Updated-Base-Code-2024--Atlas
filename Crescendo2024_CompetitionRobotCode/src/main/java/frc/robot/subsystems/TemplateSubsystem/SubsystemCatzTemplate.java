// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.TemplateSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;


public class SubsystemCatzTemplate extends SubsystemBase {
  
  private final TemplateIO io;

  public SubsystemCatzTemplate() {

            switch (CatzConstants.currentMode) {
            case REAL: io = 
                    new TemplateIOReal();
                break;

            case SIM : io = null;
                break;
            default : io = 
                    new TemplateIOReal() {};
                break;
        }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.exampleAccessMethod(0);
  }

}
