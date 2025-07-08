package frc.robot.subsystems.Hopper;

public interface HopperIO {
    default void setWheelSpeed(double speed) {}
    default void setBeltSpeed(double speed) {}
    public class HopperIOInputs {
        public double wheelVoltage = 0.0;
        public double wheelCurrent = 0.0;
        public double beltVoltage = 0.0;
        public double beltCurrent = 0.0;
    }
    default void updateInputs(HopperIOInputs inputs) {}
    default void refreshData() {}
}
