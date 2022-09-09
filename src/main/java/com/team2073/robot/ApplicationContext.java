package com.team2073.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class ApplicationContext {

    private static ApplicationContext instance;

    private CANSparkMax flywheelMotor;

    public static ApplicationContext getInstance() {
        if (instance == null) {
            instance = new ApplicationContext();
        }
        return instance;
    }

    public CANSparkMax getFlywheelMotor() {
        if(flywheelMotor == null) {
            flywheelMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        }
        return flywheelMotor;
    }

}
