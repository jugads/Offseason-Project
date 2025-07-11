package frc.robot.subsystems.Arm;

public interface ArmIO {
    public class ArmIOInputs {
        public double position = 0.0;
        public double velocity = 0.0;
    }
    default void setArmSpeed(double speed){}
    default void updateInputs(ArmIOInputs inputs){}
    default void setPositionSetpoint(double setpoint) {}
    default void holdArm() {}
    default void refreshData() {}
}
