package frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private final HopperIO io;
    private final HopperIO.HopperIOInputs inputs = new HopperIO.HopperIOInputs();

    public enum WantedState {
        IDLE,
        FEEDING,
        REVERSE
    }

    private enum SystemState {
        IDLED,
        FEEDING,
        REVERSING
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLED;

    private double wheelSpeedSetpoint = 0.0;
    private double beltSpeedSetpoint = 0.0;

    public HopperSubsystem(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            systemState = newState;
        }

        // Run outputs based on current system state
        switch (systemState) {
            case FEEDING:
                io.setWheelSpeed(0.5);
                io.setBeltSpeed(-0.5);
                break;
            case REVERSING:
                io.setWheelSpeed(-wheelSpeedSetpoint);
                io.setBeltSpeed(-beltSpeedSetpoint);
                break;
            case IDLED:
            default:
                io.setWheelSpeed(0.0);
                io.setBeltSpeed(0.0);
                break;
        }

    }

    private SystemState handleStateTransition() {
        switch (wantedState) {
            case FEEDING:
                return SystemState.FEEDING;
            case REVERSE:
                return SystemState.REVERSING;
            case IDLE:
            default:
                return SystemState.IDLED;
        }
    }

    // Public control methods
    public void feed(double wheelSpeed, double beltSpeed) {
        this.wheelSpeedSetpoint = wheelSpeed;
        this.beltSpeedSetpoint = beltSpeed;
        setWantedState(WantedState.FEEDING);
    }

    public void reverse(double wheelSpeed, double beltSpeed) {
        this.wheelSpeedSetpoint = wheelSpeed;
        this.beltSpeedSetpoint = beltSpeed;
        setWantedState(WantedState.REVERSE);
    }

    public void stop() {
        setWantedState(WantedState.IDLE);
    }

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    public WantedState getWantedState() {
        return wantedState;
    }
    public boolean isJammed() {
        // Replace with your actual sensor logic or return false to test
        return false;
    }
    
    public boolean hasBall() {
        // Replace with sensor or switch input
        return true;
    }
    
}
