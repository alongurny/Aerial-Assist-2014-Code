package drivetrain;

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
    }

    public void rotate(double speed) {
        setLeftSpeed(speed);
        setRightSpeed(-speed);
    }

    public void arcade(double moveValue, double rotateValue) {
        double leftMotorSpeed;
        double rightMotorSpeed;
        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }
        twoJoystickDrive(leftMotorSpeed, rightMotorSpeed);
    }

    public void stop() {
        straight(0);
    }

    public void twoJoystickDrive(double leftSpeed, double rightSpeed) {
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
    }

    public void setLeftSpeed(double speed) {
        leftGearbox.set(speed);
    }

    public void setRightSpeed(double speed) {
        rightGearbox.set(-speed);
    }
    
    public double getLeftSpeed() {
        return leftGearbox.getVelocity();
    }

    public double getRightSpeed() {
        return rightGearbox.getVelocity();
    }
}
