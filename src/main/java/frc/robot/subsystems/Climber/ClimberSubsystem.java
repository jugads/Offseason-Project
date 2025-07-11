package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants.*;


public class ClimberSubsystem extends SubsystemBase{
    private final ClimberIO io;
    private final ClimberIO.ClimberIOInputs inputs = new ClimberIO.ClimberIOInputs();

    public enum WantedState {
        IDLE,
        MOVE_UP,
        MOVE_DOWN,
        DEPLOY
    }

    private WantedState wantedState = WantedState.IDLE;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        switch (wantedState) {
            case MOVE_UP:
                io.runClimber(0.5);
                break;
            case MOVE_DOWN:
                io.runClimber(-0.5);
                break;
            case DEPLOY:
                if (inputs.position < k90DegreesRotations) {
                io.runClimber(0.7);
                }
                else {
                    io.runClimber(0.);
                }
                break;
            case IDLE:
            default:
                io.runClimber(0.0);
                break;
        }
    }
    
    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }
    public WantedState getWantedState() {
        return wantedState;
    }
}
