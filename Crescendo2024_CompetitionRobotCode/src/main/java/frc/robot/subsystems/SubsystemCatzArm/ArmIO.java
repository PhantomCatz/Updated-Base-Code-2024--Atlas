package frc.robot.subsystems.SubsystemCatzArm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog
    public class ArmIOInputs {
        public double dummyVariable;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void exampleAccessMethod(double test) {}
}
