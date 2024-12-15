package frc.robot.subsystems.drivetrain;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import frc.robot.Constants.ModuleConstants;

public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private final PIDController drivePID;
    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController turnController;

    public ModuleIOSim(SwerveModuleSimulation moduleSim) {
        moduleSimulation = moduleSim;

        drivePID = new PIDController(ModuleConstants.kDrivingPSim.get(), ModuleConstants.kDrivingISim.get(), ModuleConstants.kDrivingDSim.get());
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.1);
        turnController = new PIDController(ModuleConstants.kTurningPSim.get(), ModuleConstants.kTurningISim.get(), ModuleConstants.kTurningDSim.get());
        turnController.enableContinuousInput(-Math.PI, Math.PI);
        
        driveMotor = moduleSimulation.useGenericMotorControllerForDrive()
                .withCurrentLimit(Units.Amps.of(ModuleConstants.kDrivingMotorCurrentLimit));
        turnMotor = moduleSimulation.useGenericControllerForSteer()
                .withCurrentLimit(Units.Amps.of(ModuleConstants.kTurningMotorCurrentLimit));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        drivePID.setP(ModuleConstants.kDrivingPSim.get());
        drivePID.setI(ModuleConstants.kDrivingISim.get());
        drivePID.setD(ModuleConstants.kDrivingDSim.get());
        turnController.setP(ModuleConstants.kTurningPSim.get());
        turnController.setI(ModuleConstants.kTurningISim.get());
        turnController.setD(ModuleConstants.kTurningDSim.get());

        inputs.position = new SwerveModulePosition(
            moduleSimulation.getDriveWheelFinalPosition().in(Units.Radians) * moduleSimulation.WHEEL_RADIUS.in(Units.Meters),
            moduleSimulation.getSteerAbsoluteFacing()
        );
        inputs.state = new SwerveModuleState(
            moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond) * moduleSimulation.WHEEL_RADIUS.in(Units.Meters),
            moduleSimulation.getSteerAbsoluteFacing()
        );
    }

    @Override
    public void setState(SwerveModuleState state) {
        Logger.recordOutput("Test", driveFeedforward.calculate(
            Units.RadiansPerSecond.of(state.speedMetersPerSecond * moduleSimulation.DRIVE_GEAR_RATIO / moduleSimulation.WHEEL_RADIUS.in(Units.Meters))
        ));
        driveMotor.requestVoltage(
            driveFeedforward.calculate(
                Units.RadiansPerSecond.of(state.speedMetersPerSecond / moduleSimulation.WHEEL_RADIUS.in(Units.Meters))
            ).plus(
                Units.Volts.of(drivePID.calculate(moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond), state.speedMetersPerSecond / moduleSimulation.WHEEL_RADIUS.in(Units.Meters)))
            )
        );
        turnMotor.requestVoltage(Units.Volts.of(turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians(), state.angle.getRadians())));
    }
}
