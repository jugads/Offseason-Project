package frc.robot.subsystems.AlgaeGripper;

public interface AlgaeGripperIO {
    public class AlgaeGripperInputs {
        public double distance = 0.0;
        public boolean seeingAlgae = false;
        public double velocity = 0.0;
    }

    default void setMotor(double speed) {}
    default void updateInputs(AlgaeGripperInputs inputs) {}
    default void refreshData() {}
}
