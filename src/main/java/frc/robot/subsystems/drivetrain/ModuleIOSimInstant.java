package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleIOSimInstant implements ModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private SwerveModuleState state = new SwerveModuleState();

    private double position = 0.0;

    public ModuleIOSimInstant() {
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        position += inputs.state.speedMetersPerSecond * LOOP_PERIOD_SECS;

        inputs.state = state;
        inputs.position = new SwerveModulePosition(position, inputs.state.angle);
    }

    @Override
    public void setState(SwerveModuleState state) {
        this.state = state;
    }
}
  