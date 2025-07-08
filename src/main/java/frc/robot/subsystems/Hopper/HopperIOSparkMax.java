package frc.robot.subsystems.Hopper;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HopperIOSparkMax implements HopperIO {
    private final SparkMax wheelMotor;
    private final SparkMax beltMotor;

    public HopperIOSparkMax(int wheelID, int beltID) {
        wheelMotor = new SparkMax(wheelID, MotorType.kBrushless);  // Neo 550 or Neo
        beltMotor = new SparkMax(beltID, MotorType.kBrushless);
    }

    @Override
    public void setWheelSpeed(double speed) {
        wheelMotor.set(speed);  // -1.0 to 1.0
    }

    @Override
    public void setBeltSpeed(double speed) {
        beltMotor.set(speed);  // -1.0 to 1.0
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.wheelVoltage = wheelMotor.getAppliedOutput() * wheelMotor.getBusVoltage();
        inputs.wheelCurrent = wheelMotor.getOutputCurrent();

        inputs.beltVoltage = beltMotor.getAppliedOutput() * beltMotor.getBusVoltage();
        inputs.beltCurrent = beltMotor.getOutputCurrent();
    }

    @Override
    public void refreshData() {
        // Not required for Spark MAX, but useful for manual telemetry push or debug logging
        SmartDashboard.putNumber("Hopper/WheelVoltage", wheelMotor.getAppliedOutput() * wheelMotor.getBusVoltage());
        SmartDashboard.putNumber("Hopper/BeltVoltage", beltMotor.getAppliedOutput() * beltMotor.getBusVoltage());
    }
}
