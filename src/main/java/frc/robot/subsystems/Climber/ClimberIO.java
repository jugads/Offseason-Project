package frc.robot.subsystems.Climber;

public interface ClimberIO {
    default void runClimber(double speed) {}
    default void updateInputs(ClimberIOInputs inputs) {}

    class ClimberIOInputs {
        public double position = 0.0;
    }
}
