package frc.robot.subsystems.SubsystemCatzClimb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog
    public class ClimbIOInputs {
        public double dummyVariable;
    }

    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void exampleAccessMethod(double test) {}
}
