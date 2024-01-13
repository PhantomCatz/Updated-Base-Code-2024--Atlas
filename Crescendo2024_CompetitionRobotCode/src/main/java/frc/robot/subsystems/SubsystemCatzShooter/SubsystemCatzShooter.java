// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SubsystemCatzShooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;


public class SubsystemCatzShooter extends SubsystemBase {
  
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private static SubsystemCatzShooter instance;

  public SubsystemCatzShooter() {

            switch (CatzConstants.currentMode) {
            case REAL: io = 
                    new ShooterIOReal();
                break;

            case SIM : io = null;
                break;
            default : io = 
                    new ShooterIOReal() {};
                break;
        }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/shooterinputs ", inputs);
    // This method will be called once per scheduler run
  }

  public Command setShooterActive() {
    return run(()->io.shootWithVelocity());
  }

  public Command setShooterDisabled() {
    return run(()->io.setShooterDisabled());
  }

  // Get the singleton instance of the ShooterSubsystem
  public static SubsystemCatzShooter getInstance() {
      return instance;
  }

}
