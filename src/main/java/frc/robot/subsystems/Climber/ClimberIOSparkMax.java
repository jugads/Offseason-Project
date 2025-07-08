package frc.robot.subsystems.Climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;


public class ClimberIOSparkMax implements ClimberIO{
    private final SparkMax motorOne;
    private final SparkMax motorTwo;
    private final RelativeEncoder encoder;

    public ClimberIOSparkMax(int motorOneID, int motorTwoID) {
        motorOne = new SparkMax(motorOneID, MotorType.kBrushless);
        motorTwo = new SparkMax(motorTwoID, MotorType.kBrushless);

        encoder = motorOne.getEncoder();

        encoder.setPosition(0.0);
    }
    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.position = encoder.getPosition();
        SmartDashboard.putNumber("Climber Position", inputs.position);
    }

    @Override
    public void runClimber(double speed) {
        if (encoder.getPosition() < 240) {
            motorOne.set(speed);
            motorTwo.set(speed);
            }
            else {
              motorOne.set(0.0);
              motorTwo.set(0.0);
            }
    }

}
