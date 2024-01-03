package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;

public interface ModuleIO 
{
 @AutoLog
 public static class ModuleIOInputs {
    public double gyroAngle = 0.0;
    public double driveMtrVelocity = 0.0;
    public double driveMtrSensorPosition = 0.0;
    public double magEncoderValue = 0.0;
    public double driveAppliedVolts = 0.0;
    public double steerAppliedVolts = 0.0;
    public double driveMtrPercentOutput = 0.0;
 }

 /** Updates the set of loggable inputs. */
 public default void updateInputs(ModuleIOInputs inputs) {}

 public default void setDrivePwrPercentIO(double drivePwrPercent) {}

 public default void setDriveVelocityIO(double velocity) {}

 public default void setSteerPwrIO(double SteerPwr) {}

 public default void setSteerVoltageIO(double steerVoltage) {}

 public default void setSteerCoastModeIO() {}

 public default void setSteerBrakeModeIO() {}

 public default void setDrvSensorPositionIO(double sensorpos) {}

 public default void reverseDriveIO(boolean enable) {}

 public default void setDriveSimPwrIO(double volts) {}

 public default void setSteerSimPwrIO(double volts) {}

 public default void setDriveControlIO(VelocityTorqueCurrentFOC controlValue) {}

 public default void setSteerControlIO(PositionVoltage controlValue) {}

 public default void resetMagEncoderIO() {}

}
