package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final Double chassisAngularOffset = 0.0;

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    }

    public SwerveModuleState runState(SwerveModuleState state) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedDesiredState.angle = state.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
        correctedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                getAngle());
        correctedDesiredState.speedMetersPerSecond *= correctedDesiredState.angle.minus(getAngle()).getCos();
        
        io.setState(correctedDesiredState);
        return correctedDesiredState;
    }

    public Rotation2d getAngle() {
        return inputs.position.angle.plus(new Rotation2d(chassisAngularOffset));
    }

    public double getSpeed() {
        return inputs.state.speedMetersPerSecond;
    }

    public SwerveModuleState getState() {
        return inputs.state;
    }

    public SwerveModulePosition getPosition() {
        return inputs.position;
    }
}
