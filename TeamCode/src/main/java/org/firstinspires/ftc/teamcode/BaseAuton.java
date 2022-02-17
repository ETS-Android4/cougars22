package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class BaseAuton extends LinearOpMode
{
    /* Declare OpMode members. */
    OurBot robot = new OurBot();
    BNO055IMU imu = null;
    private ElapsedTime runtime = new ElapsedTime();
    static final double HEADING_THRESHOLD = 0.1; //How close to target angle we need to get when turning

    @Override
    public void runOpMode()
    {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initializing Hardware");    //
        telemetry.update();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d, %7d, %7d, %7d",
                robot.leftFront.getCurrentPosition(),
                robot.leftBack.getCurrentPosition(),
                robot.rightFront.getCurrentPosition(),
                robot.rightBack.getCurrentPosition());
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    protected void encoderDrive(double speed,
                              double leftInches, double rightInches,
                              double timeoutS)
    {
        int leftFrontStart;
        int leftBackStart;
        int rightFrontStart;
        int rightBackStart;

        int leftFrontTarget;
        int leftBackTarget;
        int rightFrontTarget;
        int rightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            leftFrontStart = robot.leftFront.getCurrentPosition();
            leftBackStart = robot.leftBack.getCurrentPosition();
            rightFrontStart = robot.rightFront.getCurrentPosition();
            rightBackStart = robot.rightBack.getCurrentPosition();
            // Determine new target position, and pass to motor controller
            leftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (leftInches * OurBot.COUNTS_PER_INCH);
            leftBackTarget = robot.leftBack.getCurrentPosition() + (int) (leftInches * OurBot.COUNTS_PER_INCH);
            rightFrontTarget = robot.rightFront.getCurrentPosition() + (int) (rightInches * OurBot.COUNTS_PER_INCH);
            rightBackTarget = robot.rightBack.getCurrentPosition() + (int) (rightInches * OurBot.COUNTS_PER_INCH);
            robot.leftFront.setTargetPosition(leftFrontTarget);
            robot.leftBack.setTargetPosition(leftBackTarget);
            robot.rightFront.setTargetPosition(rightFrontTarget);
            robot.rightBack.setTargetPosition(rightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            speed = Math.abs(speed);
            robot.leftFront.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.rightBack.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((robot.leftFront.isBusy() || robot.leftBack.isBusy()) || (robot.rightFront.isBusy() || robot.rightBack.isBusy())))
            {
                double leftFrontScale = Math.max(1 - ((double)robot.leftFront.getCurrentPosition() - leftFrontStart) / (leftFrontTarget - leftFrontStart), 0.1);
                double leftBackScale = Math.max(1 - ((double)robot.leftBack.getCurrentPosition() - leftBackStart) / (leftBackTarget - leftBackStart), 0.1);
                double rightFrontScale = Math.max(1 - ((double)robot.rightFront.getCurrentPosition() - rightFrontStart) / (rightFrontTarget - rightFrontStart), 0.1);
                double rightBackScale = Math.max(1 - ((double)robot.rightBack.getCurrentPosition() - rightBackStart) / (rightBackTarget - rightBackStart), 0.1);
                robot.leftFront.setPower(speed * leftFrontScale);
                robot.leftBack.setPower(speed * leftBackScale);
                robot.rightFront.setPower(speed * rightFrontScale);
                robot.rightBack.setPower(speed * rightBackScale);
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d, %7d, %7d, %7d", leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget);
                telemetry.addData("Path2", "Running at %7d, %7d, %7d, %7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.leftBack.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition(),
                        robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    protected void gyroTurn(double speed, double angle, double Kp, double Ki, double Kd, double maxI, double a)
    {
        double error;
        double lastError;
        double errorChange;
        double filterEstimate = 0;
        double lastFilterEstimate = 0;
        double derivative;
        double integral = 0;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        ElapsedTime timer = new ElapsedTime();

        error = getError(angle);
        while (opModeIsActive() && !onTarget)
        {
            lastError = error;
            error = getError(angle);

            errorChange = error - lastError;

            filterEstimate = (a * lastFilterEstimate) + (1 - a) * errorChange;
            lastFilterEstimate = filterEstimate;

            derivative = filterEstimate / timer.seconds();

            integral += error * timer.seconds();
            if (integral > maxI / Ki)
            {
                integral = maxI / Ki;
            }
            else if (integral < -maxI / Ki)
            {
                integral = -maxI / Ki;
            }

            if (Math.abs(error) <= HEADING_THRESHOLD)
            {
                steer = 0.0;
                leftSpeed = 0.0;
                rightSpeed = 0.0;
                onTarget = true;
            }
            else
            {
                steer = Range.clip(error * Kp + derivative * Kd + integral * Ki, -1, 1);
                if (steer > 0 && steer < 0.10)
                {
                    steer = 0.10;
                }
                else if (steer < 0 && steer > -0.10)
                {
                    steer = -0.10;
                }
                leftSpeed = speed * steer;
                rightSpeed = -leftSpeed;
            }

            // Send desired speeds to motors.
            robot.leftFront.setPower(leftSpeed);
            robot.leftBack.setPower(leftSpeed);
            robot.rightFront.setPower(rightSpeed);
            robot.rightBack.setPower(rightSpeed);

            timer.reset();

            // Display it for the driver.
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Derivative", "%5.2f", derivative);
            telemetry.addData("Integral", "%5.2f", integral);
            telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(double targetAngle)
    {
        double robotError;

        // calculate error in -179 to +180 range
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

}
