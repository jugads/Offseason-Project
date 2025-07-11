package frc.robot.subsystems.Arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmIOSparkMax implements ArmIO {
    private final SparkMax motor;
    private final DutyCycleEncoder encoder;
    private final ArmFeedforward ff = new ArmFeedforward(0., 0.035, 0);
    PIDController controller= new PIDController(2.25, 0, 0.0);

    public ArmIOSparkMax(int ID) {
        motor = new SparkMax(ID, MotorType.kBrushless);  // Neo 550 or Neo
        encoder = new DutyCycleEncoder(0);
    }

    @Override
    public void setArmSpeed(double speed) {
        motor.set(speed);  // -1.0 to 1.0
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.position = -(encoder.get()-0.355467);
        inputs.velocity = motor.get();
    }
    @Override
    public void setPositionSetpoint(double setpoint) {
        controller.setSetpoint(setpoint);

        double currentPosition = -(encoder.get()-0.355467); // same as in updateInputs()
        double output = controller.calculate(currentPosition);

        motor.set(output); // same direction for both
    }
    @Override
    public void holdArm() {
        double currentPosition = -(encoder.get()-0.355467);
        SmartDashboard.putNumber("Arm/FF", ff.calculate(currentPosition * Math.PI * 2, 0));
        setArmSpeed(ff.calculate(currentPosition * Math.PI * 2, 0));
    }
    @Override
    public void refreshData() {
        // Not required for Spark MAX, but useful for manual telemetry push or debug logging
        SmartDashboard.putNumber("Arm/Position", -(encoder.get()-0.355467));
        SmartDashboard.putNumber("Arm/Velocity", motor.get());
    }
}
