package frc.robot.subsystems.Climber;

import static frc.robot.Constants.ClimberConstants.k90DegreesRotations;

public interface ClimberIO {
    default void runClimber(double speed) {}
    default void updateInputs(ClimberIOInputs inputs) {}

    class ClimberIOInputs {
        public double position = 0.0;
        public double setpoint = k90DegreesRotations;
    }
}
