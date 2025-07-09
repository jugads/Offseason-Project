package frc.robot.subsystems.Bluetooth;

public interface BluetoothIO {
    public class BluetoothIOInputs {
        public double current = 0.0;
        public double velocity = 0.0;
    }
    default void updateInputs(BluetoothIOInputs inputs) {}
    default void refreshData() {}
    default void setMotor(double speed) {}
}
