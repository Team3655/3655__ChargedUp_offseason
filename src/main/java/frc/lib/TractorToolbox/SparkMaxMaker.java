package frc.lib.TractorToolbox;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMaxMaker {
	
	public static CANSparkMax createSparkMax(int motorID) {

		CANSparkMax motor = new CANSparkMax(motorID, MotorType.kBrushless);

		motor.restoreFactoryDefaults();

		return motor;
	}

	public static RelativeEncoder createEncoder(CANSparkMax motor, double unit, double gearRatio) {

		RelativeEncoder encoder = motor.getEncoder();

		encoder.setPositionConversionFactor(unit * gearRatio);
		encoder.setVelocityConversionFactor(unit * gearRatio * (1d / 60d));

		return encoder;
	}

}
