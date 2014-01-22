package drivetrain;

import edu.wpi.first.wpilibj.templates.Vars;

/**
 *
 * @author Alon
 */
public class Drivetrain {

    private MonitoredGearbox leftGearbox, rightGearbox;
    private boolean speedScalingAllowed;
    /**
     * 
     * @param leftGearbox Left Gearbox
     * @param rightGearbox Right Gearbox
     * @param speedAutoFixAllowed Whether the speed autofix is allowed or not
     */
    public Drivetrain(MonitoredGearbox leftGearbox, MonitoredGearbox rightGearbox, boolean speedAutoFixAllowed) {
        this.leftGearbox = leftGearbox;
        this.rightGearbox = rightGearbox;
        setAutoFix(speedAutoFixAllowed);
    }

    /**
     * Speed autofix is disabled by default.
     * @param leftGearbox Left Gearbox
     * @param rightGearbox Right Gearbox
     * @param speedAutoFixAllowed Whether the speed autofix is allowed or not
     */
    public Drivetrain(MonitoredGearbox leftGearbox, MonitoredGearbox rightGearbox) {
        this(leftGearbox, rightGearbox, false);
    }

    /**
     * Drive straight in speed
     *
     * @param speed movement speed in range of [-1.0,1.0]
     */
    public void straight(double speed) {
        twoSidesDrive(speed, speed);
    }

    /**
     * Rotate in speed
     *
     * @param speed rotation speed in range of [-1.0,1.0]
     */
    public void rotate(double speed) {
        twoSidesDrive(speed, -speed);
    }

    /**
     * Arcade drive. Use both movement value and rotation value to control the
     * robot.
     *
     * @param moveValue movement speed in range of [-1.0,1.0]
     * @param rotateValue rotation speed in range of [-1.0,1.0]
     */
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
        twoSidesDrive(leftMotorSpeed, rightMotorSpeed);
    }

    /**
     * Stops the robot.
     */
    public void stop() {
        straight(0);
    }

    /**
     * Use this method to move each gearbox separately.
     *
     * @param leftSpeed left gearbox speed
     * @param rightSpeed right gearbox speed
     */
    public void twoSidesDrive(double leftSpeed, double rightSpeed) {
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
        if (speedScalingAllowed) {
            scaleFactors(leftSpeed, rightSpeed);
        } else {
            leftGearbox.setSpeedFactor(1);
            rightGearbox.setSpeedFactor(1);
        }
    }

    /**
     * Same as twoSidesDrive.
     *
     * @param leftSpeed
     * @param rightSpeed
     */
    public void twoJoystickDrive(double leftSpeed, double rightSpeed) {
        twoSidesDrive(leftSpeed, rightSpeed);
    }

    /**
     * This method is used to scale the speed factors of each gearbox. Factors
     * are used, for example, if one gearbox is slower than the other
     * (unintentionally), to reduce the difference between their speeds. The
     * method uses getLeftSpeed/getRightSpeed function, which communicate with
     * the Encoder.
     *
     * @param wantedLeft The wanted speed of the left gearbox.
     * @param wantedRight The wanted speed of the right gearbox.
     */
    private void scaleFactors(double wantedLeft, double wantedRight) {
        if (Math.abs(wantedLeft) < Vars.Drivetrain.NO_SCALE_SPEED || Math.abs(wantedRight) < Vars.Drivetrain.NO_SCALE_SPEED) {
            return;
        }
        double realLeft = getLeftSpeed();
        double realRight = getRightSpeed();
        double leftRatio = Math.abs(realLeft / wantedLeft);
        double rightRatio = Math.abs(realRight / wantedRight);
        if (leftRatio - rightRatio > Vars.COMPARE_DOUBLE_TOLERANCE) {
            leftGearbox.setSpeedFactor(rightRatio);
            rightGearbox.setSpeedFactor(1);
        } else if (leftRatio - rightRatio < Vars.COMPARE_DOUBLE_TOLERANCE) {
            leftGearbox.setSpeedFactor(1);
            rightGearbox.setSpeedFactor(leftRatio);
        } else {
            leftGearbox.setSpeedFactor(1);
            rightGearbox.setSpeedFactor(1);
        }
    }

    /**
     * Set wanted speed to the left Gearbox.
     *
     * @param speed Wanted left speed
     */
    public void setLeftSpeed(double speed) {
        leftGearbox.set(speed);
    }

    /**
     * Set wanted speed to the right Gearbox.
     *
     * @param speed Wanted right speed
     */
    public void setRightSpeed(double speed) {
        rightGearbox.set(-speed);
    }

    /**
     * Get the actual left speed from the encoder.
     *
     * @return The actual speed from the encoder
     */
    public double getLeftSpeed() {
        return leftGearbox.getVelocity();
    }

    /**
     * Get the actual right speed from the encoder.
     *
     * @return The actual speed from the encoder
     */
    public double getRightSpeed() {
        return -rightGearbox.getVelocity();
    }
    
    /**
     * Allow drivetrain speed autofix.
     * @param allowed Whether the autofix allowed or not
     */
    public void setAutoFix(boolean allowed) {
        this.speedScalingAllowed = allowed;
    }
}
