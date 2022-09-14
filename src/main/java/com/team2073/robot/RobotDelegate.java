package com.team2073.robot;

import com.revrobotics.REVPhysicsSim;
import com.team2073.common.robot.AbstractRobotDelegate;
import edu.wpi.first.math.system.plant.DCMotor;

public class RobotDelegate extends AbstractRobotDelegate {

    private ApplicationContext appCTX = ApplicationContext.getInstance();

    private OperatorInterface oi = new OperatorInterface();
    private StateSpaceFlywheel stateSpaceFlywheel = new StateSpaceFlywheel();

    public RobotDelegate(double period) {
        super(period);
    }

    @Override
    public void robotInit() {
        oi.init();
    }

    @Override
    public void robotPeriodic() {}

    @Override
    public void teleopInit() { }

    @Override
    public void autonomousInit() { }

    @Override
    public void autonomousPeriodic() { }

    @Override
    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(appCTX.getFlywheelMotor(), DCMotor.getNEO(1));
//        REVPhysicsSim.getInstance().addSparkMax(appCTX.getArmMotor(), DCMotor.getNEO(1));
//        PhysicsSim.getInstance().addTalonFX(appCTX.getElevatorMotor(), .5, 5100);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void simulationPeriodic() {
//        exampleArm.simulationPeriodic();
        REVPhysicsSim.getInstance().run();
//        PhysicsSim.getInstance().run();
    }
}
