package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ModuleConstants;

public class ModuleIOSim implements ModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    // private final DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), ModuleConstants.kDrivingMotorReduction, 0.025);
    // private final DCMotorSim turnSim = new DCMotorSim(DCMotor.getNeo550(1), ModuleConstants.kTurningMotorReduction, 0.004);

    private final DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, ModuleConstants.kDrivingMotorReduction),
        DCMotor.getNEO(1));
    private final DCMotorSim turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, ModuleConstants.kTurningMotorReduction),
        DCMotor.getNeo550(1));

    private final PIDController drivePID;
    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController turnController;

    public ModuleIOSim() {
        drivePID = new PIDController(ModuleConstants.kDrivingPSim.get(), ModuleConstants.kDrivingISim.get(), ModuleConstants.kDrivingDSim.get());
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.1);
        turnController = new PIDController(ModuleConstants.kTurningPSim.get(), ModuleConstants.kTurningISim.get(), ModuleConstants.kTurningDSim.get());
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        drivePID.setP(ModuleConstants.kDrivingPSim.get());
        drivePID.setI(ModuleConstants.kDrivingISim.get());
        drivePID.setD(ModuleConstants.kDrivingDSim.get());
        turnController.setP(ModuleConstants.kTurningPSim.get());
        turnController.setI(ModuleConstants.kTurningISim.get());
        turnController.setD(ModuleConstants.kTurningDSim.get());

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
        driveSim.setInputVoltage(
            driveFeedforward.calculate(Units.MetersPerSecond.of(state.speedMetersPerSecond / (ModuleConstants.kWheelDiameterMeters / 2))).in(Units.Volts) + 
            drivePID.calculate(driveSim.getAngularVelocityRadPerSec(), state.speedMetersPerSecond / (ModuleConstants.kWheelDiameterMeters / 2))
        );
        turnSim.setInputVoltage(turnController.calculate(turnSim.getAngularPositionRad(), state.angle.getRadians()));
    }
}
  