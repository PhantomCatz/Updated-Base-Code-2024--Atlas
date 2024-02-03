package frc.robot.subsystems.SubsystemCatzShooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double dummyVariable;
        public double velocityBtmLT;
        public double velocityBtmRT;
        public double velocityTopRT;
        public double velocityTopLT;
        public double feederMotorPercentOutput;
        public double feederMotorVelocity;
        public double feederMotor2PercentOutput;
        public double feederMotor2Velocity;
        public double shooterTopPercentOutput;
        public double shooterBtmPercentOutput;
        public double shooterTopSupplyCurrent;
        public double shooterBtmSupplyCurrent;
        public double shooterTopStatorCurrent;
        public double shooterBtmStatorCurrent;
        public double shooterTopTorqueCurrent;
        public double shooterBtmTorqueCurrent;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void shootWithVelocity() {}

    public default void setShooterDisabled() {}
    public default void shootFeederWithVelocity() {}

    public default void setFeederDisabled() {}

    public default void shootFeederReverse() {};
}
