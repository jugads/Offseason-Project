package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

    public enum WantedState {
        IDLE,
        MOVE_UP,
        MOVE_DOWN,
        HOLD,
        GO_TO_SETPOINT
    }

    private WantedState wantedState = WantedState.IDLE;
    private double setpointPercent = 0.0;
    private double manualSpeed = 0.5;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        SmartDashboard.putBoolean("At Setpoint", atPosition());
        switch (wantedState) {
            case MOVE_UP:
                if (!atTop()) {
                    io.setSpeed(manualSpeed);
                } else {
                    io.setSpeed(0);
                }
                break;

            case MOVE_DOWN:
                if (!atBottom()) {
                    io.setSpeed(-manualSpeed);
                } else {
                    io.setSpeed(0);
                }
                break;

            case HOLD:
                io.setSpeed(0.05); // Small hold voltage
                break;

            case GO_TO_SETPOINT:
                io.setPositionSetpoint(setpointPercent);
                break;

            case IDLE:
            default:
                io.setSpeed(0);
                break;
        }
    }

    // Public control methods
    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    public void setManualSpeed(double speed) {
        this.manualSpeed = speed;
    }

    public void setSetpointPercent(double percent) {
        this.setpointPercent = MathUtil.clamp(percent, 0.0, 1.0);
    }

    public boolean atBottom() {
        return inputs.position <= 0.01;
    }

    public boolean atTop() {
        return inputs.position >= 0.99;
    }

    public double getPosition() {
        return inputs.position;
    }
    public boolean atPosition() {
        return Math.abs(setpointPercent-inputs.position) < 0.04;
    }
    public WantedState getWantedState() {
        return wantedState;
    }
}
