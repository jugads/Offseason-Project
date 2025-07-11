package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final RelativeEncoder encoder;

    private final SparkLimitSwitch bottomSwitch;
    private final SparkLimitSwitch topSwitch;
    ElevatorFeedforward ff = new ElevatorFeedforward(0.02, 0.055, 0);
    PIDController controller= new PIDController(1.69, 0, 0.03
    );
    public ElevatorIOSparkMax(int leftMotorID, int rightMotorID) {
        leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);

        encoder = leftMotor.getEncoder(); // encoder only from one motor

        bottomSwitch = leftMotor.getForwardLimitSwitch();
        topSwitch = leftMotor.getReverseLimitSwitch();

        encoder.setPosition(0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = Math.abs(encoder.getPosition() / 20.7856);  // same conversion from your original code
        inputs.velocity = leftMotor.get();
        inputs.setpoint = controller.getSetpoint();
        if (bottomSwitch.isPressed()) {
            encoder.setPosition(0.0);  // auto-reset if at bottom
        }
        SmartDashboard.putNumber("Position", inputs.position);
        SmartDashboard.putNumber("Setpoint", inputs.setpoint);
        SmartDashboard.putNumber("Velocity", inputs.velocity);
    }

    @Override
    public void setSpeed(double speed) {
        leftMotor.set(-speed);
        rightMotor.set(-speed);
    }
    @Override
    public void stall() {
        double speed = ff.calculate(0.);
        leftMotor.set(-speed);
        rightMotor.set(leftMotor.get());
    }
    @Override
    public void setPositionSetpoint(double percent) {
        double clamped = MathUtil.clamp(percent, 0.0, 1.0);
        controller.setSetpoint(clamped);

        double currentPosition = Math.abs(encoder.getPosition() / 20.7856); // same as in updateInputs()
        double output = controller.calculate(currentPosition);

        leftMotor.set(-output);
        rightMotor.set(-output); // same direction for both
    }

}
