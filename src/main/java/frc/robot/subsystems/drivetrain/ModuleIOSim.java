package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ModuleConstants;

public class ModuleIOSim implements ModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), ModuleConstants.kDrivingMotorReduction, 0.025);
    private final DCMotorSim turnSim = new DCMotorSim(DCMotor.getNeo550(1), ModuleConstants.kTurningMotorReduction, 0.004);

    private final PIDController drivePID;
    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController turnController;

    public ModuleIOSim() {
        drivePID = new PIDController(ModuleConstants.kDrivingP*2, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD);
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.1);
        turnController = new PIDController(ModuleConstants.kTurningP*8, ModuleConstants.kTurningI, ModuleConstants.kTurningD);
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        inputs.position = new SwerveModulePosition(
            driveSim.getAngularPositionRad() * ModuleConstants.kWheelDiameterMeters / 2,
            new Rotation2d(turnSim.getAngularPositionRad())
        );
        inputs.state = new SwerveModuleState(
            driveSim.getAngularVelocityRadPerSec() * ModuleConstants.kWheelDiameterMeters / 2,
            new Rotation2d(turnSim.getAngularPositionRad())
        );
    }

    @Override
    public void setState(SwerveModuleState state) {
        driveSim.setInputVoltage(driveFeedforward.calculate(state.speedMetersPerSecond / (ModuleConstants.kWheelDiameterMeters / 2)) + drivePID.calculate(driveSim.getAngularVelocityRadPerSec(), state.speedMetersPerSecond / (ModuleConstants.kWheelDiameterMeters / 2)));
        turnSim.setInputVoltage(turnController.calculate(turnSim.getAngularPositionRad(), state.angle.getRadians()));
    }
}
  