// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.TractorToolbox.SparkMaxMaker;
import frc.lib.TractorToolbox.TractorParts.PIDGains;
import frc.lib.TractorToolbox.TractorParts.SwerveConstants;
import frc.lib.TractorToolbox.TractorParts.SwerveModuleConstants;

public class SwerveModule {

	private final CANSparkMax turnMotor;
	private final CANSparkMax leaderDriveMotor;
	private final CANSparkMax followerDriveMotor;

	// private final CANCoder absoluteEncoder;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turnEncoder;
	private final AbsoluteEncoder throughBore;

	private final SparkMaxPIDController drivePID;
	private final SparkMaxPIDController turnPID;

	private final double angleZeroOffset;

	private final String moduleName;

	private SwerveModuleState optimizedState;

	private final SwerveConstants swerveConstants;

	/** Creates a new SwerveModule. */
	public SwerveModule(
			String moduleName,
			SwerveModuleConstants moduleConstants,
			SwerveConstants swerveConstants,
			PIDGains kmoduleturninggains,
			PIDGains kmoduledrivegains) {

		this.swerveConstants = swerveConstants;
		this.moduleName = moduleName;
		this.angleZeroOffset = moduleConstants.kAngleZeroOffset;
		optimizedState = new SwerveModuleState();

		// // Initalize CANcoder
		// absoluteEncoder = new CANCoder(moduleConstants.kCANCoderID,
		// Constants.kCTRECANBusName);
		// absoluteEncoder.configFactoryDefault();
		// absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		// absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		// absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10,
		// 100);
		// absoluteEncoder.clearStickyFaults();

		// Initialize the motors
		// Initialize turning motor, encoder, and PID
		// Turn Motor
		turnMotor = SparkMaxMaker.createSparkMax(moduleConstants.kTurnMotorID);
		turnMotor.setInverted(true);
		turnMotor.setIdleMode(IdleMode.kCoast);
		turnMotor.setSmartCurrentLimit(swerveConstants.kTurnSmartCurrentLimit);
		// Turn Encoder
		turnEncoder = turnMotor.getEncoder();
		turnEncoder.setPositionConversionFactor((2 * Math.PI) * swerveConstants.kTurnGearRatio); // radians
		// radians per second
		turnEncoder.setVelocityConversionFactor((2 * Math.PI) * swerveConstants.kTurnGearRatio * (1d / 60d));
		// Turn PID
		turnPID = turnMotor.getPIDController();
		turnPID.setP(kmoduleturninggains.kP);
		turnPID.setI(kmoduleturninggains.kI);
		turnPID.setD(kmoduleturninggains.kD);
		turnPID.setPositionPIDWrappingEnabled(true);
		turnPID.setPositionPIDWrappingMinInput(-Math.PI);
		turnPID.setPositionPIDWrappingMaxInput(Math.PI);

		// Leader Drive Motor
		leaderDriveMotor = SparkMaxMaker.createSparkMax(moduleConstants.kLeaderDriveMotorID);
		leaderDriveMotor.setIdleMode(IdleMode.kBrake);
		leaderDriveMotor.setSmartCurrentLimit(swerveConstants.kDriveSmartCurrentLimit);
		// Leader Drive Encoder
		driveEncoder = leaderDriveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(
				swerveConstants.kDriveGearRatio * swerveConstants.kWheelCircumference); // meters
		driveEncoder.setVelocityConversionFactor(
				swerveConstants.kDriveGearRatio
						* swerveConstants.kWheelCircumference
						* (1d / 60d)); // meters per second

		// Leader Drive PID
		drivePID = leaderDriveMotor.getPIDController();
		drivePID.setP(kmoduledrivegains.kP);
		drivePID.setI(kmoduledrivegains.kI);
		drivePID.setD(kmoduledrivegains.kD);
		drivePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		drivePID.setSmartMotionMaxAccel(swerveConstants.kMaxModuleAccelMetersPerSecond, 0);
		drivePID.setSmartMotionMaxVelocity(swerveConstants.kMaxModuleSpeedMetersPerSecond, 0);
		drivePID.setFF(swerveConstants.kDriveFeedForward);

		// Follower Drive Motor
		followerDriveMotor = SparkMaxMaker.createSparkMax(moduleConstants.kFolloweDriveMotorID);
		followerDriveMotor.follow(leaderDriveMotor, true);
		followerDriveMotor.setSmartCurrentLimit(swerveConstants.kDriveSmartCurrentLimit);

		throughBore = followerDriveMotor.getAbsoluteEncoder(Type.kDutyCycle);
		throughBore.setPositionConversionFactor((2 * Math.PI));
		// throughBore.setZeroOffset(-angleZeroOffset);

		// turnPID.setFeedbackDevice(throughBore);
		setAngleOffset();
	}

	// region: getters

	/** returns the current heading of the module in radians */
	public double getHeading() {
		return turnEncoder.getPosition(); // % Math.PI;
	}

	/**
	 * returns the current heading of the module from the absolute encoder in
	 * radians
	 */
	public double getAbsoluteHeading() {
		return throughBore.getPosition() - angleZeroOffset;
	}

	public double getDistanceMeters() {
		return driveEncoder.getPosition();
	}

	/**
	 * returns the error from the current angle to optimized angle of the module in
	 * radians
	 */
	public double getAngleError() {
		return optimizedState.angle.getRadians() - getHeading();
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		double moduleAngleRadians = getHeading();

		double distanceMeters = driveEncoder.getPosition();

		return new SwerveModulePosition(distanceMeters, new Rotation2d(moduleAngleRadians));
	}
	// endregion: getters

	// region: Setters
	public void setDesiredState(SwerveModuleState desiredState) {
		setDesiredState(desiredState, false);
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean isTurbo) {

		double moduleAngleRadians = getHeading();

		// Optimize the reference state to avoid spinning further than 90 degrees to the
		// desired state
		optimizedState = SwerveModuleState.optimize(
				desiredState,
				new Rotation2d(moduleAngleRadians));

		if (optimizedState.speedMetersPerSecond <= 0) {
			drivePID.setIAccum(0);
		}

		if (isTurbo) {
			// Squeeze every last bit if power out of turbo
			// leaderDriveMotor.setVoltage(12 *
			// Math.signum(optimizedState.speedMetersPerSecond));
			double turboSpeed = Math.copySign(
					swerveConstants.kMaxModuleSpeedMetersPerSecond,
					optimizedState.speedMetersPerSecond);

			drivePID.setReference(turboSpeed, ControlType.kSmartVelocity);
		} else {
			drivePID.setReference(
					optimizedState.speedMetersPerSecond,
					ControlType.kSmartVelocity);
		}

		turnPID.setReference(
				optimizedState.angle.getRadians(),
				ControlType.kPosition);
	}

	public void stopMotors() {
		leaderDriveMotor.stopMotor();
		turnMotor.stopMotor();
	}

	public void setAngleOffset() {
		turnEncoder.setPosition(throughBore.getPosition() - Units.degreesToRadians(angleZeroOffset));
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber(moduleName + " Angle Error", Units.radiansToDegrees(getAngleError()));
		// SmartDashboard.putNumber(moduleName + " ThroughBore Angle",
		// Units.radiansToDegrees(throughBore.getPosition()));
		SmartDashboard.putNumber(moduleName + " Offset", angleZeroOffset);
		// SmartDashboard.putString(moduleName + " Abs. Status",
		// absoluteEncoder.getLastError().toString());
		SmartDashboard.putNumber(moduleName + " Drive Current Draw", leaderDriveMotor.getOutputCurrent());
		SmartDashboard.putNumber(moduleName + " Optimized Angle", optimizedState.angle.getDegrees());
		SmartDashboard.putNumber(moduleName + " Turn Motor Output", turnMotor.getAppliedOutput());
		SmartDashboard.putNumber(moduleName + " Target Velocity", optimizedState.speedMetersPerSecond);
		SmartDashboard.putNumber(moduleName + " Velocity", driveEncoder.getVelocity());
		SmartDashboard.putNumber(moduleName + " Absolute Angle", Units.radiansToDegrees(getAbsoluteHeading()));
		SmartDashboard.putNumber(moduleName + " SparkEncoder Angle",
				Units.radiansToDegrees(getHeading()));
	}

	// endregion: setters

	// region: doers

	public void attemptAgnleOffset() {

		int attempts = 0;

		System.out.println("ZEROING " + moduleName);

		while (true) {

			if (Units.radiansToDegrees(Math.abs(getHeading()) - Math.abs(getAbsoluteHeading())) > 1) {
				System.out.println("WHEEL ZEROING FAILED! looping...");
				attempts++;
			} else {
				System.out.println(moduleName + " WHEEL ZEROED CORRECTLY. in " + attempts + " attempts");
				break;
			}

			setAngleOffset();

			Timer.delay(.1);

		}

	}

	// endregion: doers
}
