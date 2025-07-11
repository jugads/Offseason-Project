package frc.robot.subsystems.Elevator;
public interface ElevatorIO {
    default void setSpeed(double speed) {}
    default void setPositionSetpoint(double percentage) {}
    default void updateInputs(ElevatorIOInputs inputs) {}
    default void stall() {}
    class ElevatorIOInputs {
        public double position = 0.0;
        public double velocity = 0.0;
        public double setpoint = 0.0;
    }
}
