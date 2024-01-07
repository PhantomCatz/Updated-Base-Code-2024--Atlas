package frc.robot.subsystems.TemplateSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface TemplateIO {
    @AutoLog
    public class TemplateIOInputs {
        public double dummyVariable;
    }

    public default void updateInputs(TemplateIOInputs inputs) {}

    public default void exampleAccessMethod(double test) {}
}
