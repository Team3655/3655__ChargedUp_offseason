package frc.robot.TractorToolbox;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMaxMaker {
	
	public static CANSparkMax createSparkMax(int motorID) {

		CANSparkMax motor = new CANSparkMax(motorID, MotorType.kBrushless);
		motor.restoreFactoryDefaults();
		return motor;
	}
}
