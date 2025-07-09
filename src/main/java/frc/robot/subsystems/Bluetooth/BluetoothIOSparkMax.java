package frc.robot.subsystems.Bluetooth;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BluetoothIOSparkMax implements BluetoothIO {
    SparkMax motor;
    double current;
    public BluetoothIOSparkMax(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);
    }

    @Override
    public void setMotor(double speed) {
        motor.set(speed);
    }

    @Override
    public void updateInputs(BluetoothIOInputs inputs) {
        inputs.current = motor.getOutputCurrent();
        inputs.velocity = motor.get();
        current = inputs.current;
    }

    @Override
    public void refreshData() {
        SmartDashboard.putNumber("Bluetooth/Current", current);
    }
}
