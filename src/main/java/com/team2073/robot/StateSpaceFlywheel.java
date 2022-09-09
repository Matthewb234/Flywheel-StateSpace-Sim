package com.team2073.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team2073.common.periodic.AsyncPeriodicRunnable;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StateSpaceFlywheel implements AsyncPeriodicRunnable {
    private Joystick controller = new Joystick(0);
    private final CANSparkMax motor = ApplicationContext.getInstance().getFlywheelMotor();
    // An encoder set up to measure flywheel velocity in radians per second.
    private final RelativeEncoder encoder = motor.getEncoder();
    private static final double SPINUP_RAD_PER_SEC = Units.rotationsPerMinuteToRadiansPerSecond(3500);

    // Volts per (radian per second)
    //example value
    private double flywheelKv = 0.002;
    // Volts per (radian per second squared)
    //example value
    private double flywheelKa = 0.001;
    // The plant holds a state-space model of our flywheel. This system has the following properties:
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.
    private LinearSystem<N1, N1, N1> sysIdFlywheelPlant = LinearSystemId.identifyVelocitySystem(flywheelKv, flywheelKa);



    private double flywheelMomentOfInertia = 0.00032; // kg * m^2
    // Reduction between motors and encoder, as output over input. If the flywheel spins slower than
    // the motors, this number should be greater than one.
    private double flywheelGearing = 1.0;
    // The plant holds a state-space model of our flywheel. This system has the following properties:
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.
    private LinearSystem<N1, N1, N1> moiFlywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), flywheelMomentOfInertia, flywheelGearing);


    private final KalmanFilter<N1, N1, N1> m_observer =
            new KalmanFilter<>(
                    Nat.N1(),
                    Nat.N1(),
                    sysIdFlywheelPlant,
                    VecBuilder.fill(.01), // How accurate we think our model is
                    VecBuilder.fill(100), // How accurate we think our encoder data is
                    0.020);

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N1, N1, N1> flywheelController =
            new LinearQuadraticRegulator<>(
                    sysIdFlywheelPlant,
                    VecBuilder.fill(1.0), // qelms. Velocity error tolerance, in radians per second. Decrease
                    // this to more heavily penalize state excursion, or make the controller behave more
                    // aggressively.
                    VecBuilder.fill(12), // relms. Control effort (voltage) tolerance. Decrease this to more
                    // heavily penalize control effort, or make the controller less aggressive. 12 is a good
                    // starting point because that is the (approximate) maximum voltage of a battery.
                    0.020);

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    private final LinearSystemLoop<N1, N1, N1> flywheelLoop =
            new LinearSystemLoop<>(sysIdFlywheelPlant, flywheelController, m_observer, 12.0, 0.020);

    private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    private MechanismRoot2d root = mechanism2d.getRoot("root", 30, 30);
    private MechanismLigament2d flywheel = root.append(new MechanismLigament2d("flywheel", 10, Units.radiansToDegrees(encoder.getPosition())));

    public StateSpaceFlywheel() {
        autoRegisterWithPeriodicRunner();
        flywheelLoop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()/42)));
        SmartDashboard.putData("flywheel", mechanism2d);
    }

    @Override
    public void onPeriodicAsync() {
        // Sets the target speed of our flywheel. This is similar to setting the setpoint of a PID contoller
        if (controller.getTriggerPressed()) {
            // We just pressed the trigger, so let's set our next reference
            flywheelLoop.setNextR(VecBuilder.fill(SPINUP_RAD_PER_SEC));
        } else if (controller.getTriggerReleased()) {
            // We just released the trigger, so let's spin down
            flywheelLoop.setNextR(VecBuilder.fill(0.0));
        }

        // Correct our Kalman filter's state vector estimate with encoder data.
        flywheelLoop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()/42)));

        // Update our LQR to generate new voltage commands and use the voltages to predict the next
        // state with out Kalman filter.
        flywheelLoop.predict(0.020);

        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        double nextVoltage = flywheelLoop.getU(0);
        SmartDashboard.putNumber("RPM", encoder.getVelocity());
        SmartDashboard.putNumber("Reference",Units.radiansPerSecondToRotationsPerMinute(SPINUP_RAD_PER_SEC));
        motor.setVoltage(nextVoltage);
        flywheel.setAngle(Units.radiansToDegrees(encoder.getPosition()));
    }
}
