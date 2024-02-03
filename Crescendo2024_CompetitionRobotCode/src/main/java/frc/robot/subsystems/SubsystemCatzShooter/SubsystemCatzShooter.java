// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SubsystemCatzShooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;

public class SubsystemCatzShooter extends SubsystemBase {
  
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private static SubsystemCatzShooter instance = new SubsystemCatzShooter();

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
        System.out.println("Shooter Configured");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/shooterinputs ", inputs);
    // This method will be called once per scheduler run

    //Logger.recordOutput("velocityBtmLT", inputs.velocityBtmLT);
    Logger.recordOutput("velocityBtmRT", inputs.velocityBtmRT);
    //Logger.recordOutput("velocityTopLT", inputs.velocityTopLT);
    Logger.recordOutput("velocityTopRT", inputs.velocityTopRT);

    Logger.recordOutput("FeederPercentOUtput", inputs.feederMotorPercentOutput);
    Logger.recordOutput("FeederVelocity", inputs.feederMotorVelocity);
    Logger.recordOutput("Feeder2PercentOutput", inputs.feederMotor2PercentOutput);
    Logger.recordOutput("Feeder2Velocity", inputs.feederMotor2Velocity);
    //SmartDashboard.putNumber("velocityBtmLT", inputs.velocityBtmLT);
    SmartDashboard.putNumber("velocityBtmRT", inputs.velocityBtmRT);
    SmartDashboard.putNumber("velocityTopRT", inputs.velocityTopRT);
    SmartDashboard.putNumber("feederMotro2Velcocity", inputs.feederMotor2Velocity);
    SmartDashboard.putNumber("feederMotroVelcocity", inputs.feederMotorVelocity);
    SmartDashboard.putNumber("FeederMotor%output", inputs.feederMotorPercentOutput);
    SmartDashboard.putNumber("FeederMotor2%output", inputs.feederMotor2PercentOutput);
  }

  public Command setShooterActive() {
    return run(()->io.shootWithVelocity());
  }

  public Command setShooterDisabled() {
    return run(()->io.setShooterDisabled());
  }
  public Command setFeederActive() {
    return run(()->io.shootFeederWithVelocity());
  }

  public Command setFeederReverse() {
    return run(()->io.shootFeederReverse());
  }

  public Command setFeederDisabled() {
    return run(()->io.setFeederDisabled());
  }
  // Get the singleton instance of the ShooterSubsystem
  public static SubsystemCatzShooter getInstance() {
      return instance;
  }

}
