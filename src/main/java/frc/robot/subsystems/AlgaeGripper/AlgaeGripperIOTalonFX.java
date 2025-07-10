package frc.robot.subsystems.AlgaeGripper;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeGripperIOTalonFX implements AlgaeGripperIO{
    TalonFX motor;
    CANrange sensor;
    double distance;
    boolean seeingAlgae;
    double velocity;

    public AlgaeGripperIOTalonFX (int motorID) {
        motor = new TalonFX(motorID);
        sensor = new CANrange(62);
    }

    @Override
    public void setMotor(double speed) {
        motor.set(speed);
    }

    @Override
    public void updateInputs(AlgaeGripperInputs inputs) {
        inputs.distance = sensor.getDistance().getValueAsDouble();
        inputs.seeingAlgae = sensor.getIsDetected().getValue().booleanValue();
        inputs.velocity = motor.get();
        distance = inputs.distance;
        seeingAlgae = inputs.seeingAlgae;
        velocity = inputs.velocity;

    }

    @Override
    public void refreshData() {
        SmartDashboard.putBoolean("AlgaeGripper/seeingAlgae?", seeingAlgae);
        SmartDashboard.putNumber("AlgaeGripper/distance", distance);
        SmartDashboard.putNumber("AlgaeGripper/velocity", velocity);
    }
}
