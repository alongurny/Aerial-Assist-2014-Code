package drivetrain;

import edu.wpi.first.wpilibj.templates.Vars;

/**
 *
 * @author Alon
 */
public class Drivetrain {

    private MonitoredGearbox leftGearbox, rightGearbox;

    public Drivetrain(MonitoredGearbox leftGearbox, MonitoredGearbox rightGearbox) {
        this.leftGearbox = leftGearbox;
        this.rightGearbox = rightGearbox;
    }

    /**
     * Drive straight
     *
     * @param speed speed in range of [-1.0,1.0]
     */
    public void straight(double speed) {
        setLeftSpeed(speed);
        setRightSpeed(speed);
        scaleFactors(speed,speed);
    }

    public void rotate(double speed) {
        setLeftSpeed(speed);
        setRightSpeed(-speed);
        scaleFactors(speed,speed);
    }
    // TODO: FIXME
    public void arcade(double angularSpeed, double speed) {
        setLeftSpeed(0.5 * (speed + angularSpeed));
        setRightSpeed(0.5 * (speed - angularSpeed));
    }

    public void stop() {
        straight(0);
    }

    public void twoJoystickDrive(double leftSpeed, double rightSpeed) {
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
        scaleFactors(leftSpeed, rightSpeed);
    }

    public void setLeftSpeed(double speed) {
        leftGearbox.set(speed);
    }

    public void setRightSpeed(double speed) {
        rightGearbox.set(-speed);
    }

    public void setLeftSpeedFactor(double factor) {
        leftGearbox.setSpeedFactor(factor);
    }

    public void setRightSpeedFactor(double factor) {
        rightGearbox.setSpeedFactor(factor);
    }

    private void scaleFactors(double wantedLeftSpeed, double wantedRightSpeed) {
        final double NO_SCALE_SPEED = Vars.Gearbox.NO_SCALE_SPEED;
        if (Math.abs(leftSpeed()) <= NO_SCALE_SPEED || Math.abs(rightSpeed()) <= NO_SCALE_SPEED
                || leftSpeed() == rightSpeed()) {
        } else if (leftSpeed() > rightSpeed()) {
            setLeftSpeedFactor(rightSpeed() / leftSpeed() * wantedLeftSpeed / wantedRightSpeed);
            setRightSpeedFactor(1);
        } else {
            setRightSpeedFactor(leftSpeed() / rightSpeed() * wantedRightSpeed / wantedLeftSpeed);
            setLeftSpeedFactor(1);
        }
    }

    public double leftSpeed() {
        return leftGearbox.getVelocity();
    }

    public double rightSpeed() {
        return rightGearbox.getVelocity();
    }
}
