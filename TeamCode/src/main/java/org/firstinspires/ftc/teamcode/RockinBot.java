package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;

// All the things that we use and borrow
import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class RockinBot {
    // Motors and sensors
    private LinearOpMode o;
    private Pose2D pos;
    private double xLoc = 0;
    private double yLoc = 0;
    private double hLoc = 0;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    PIDFCoefficients pidf = null;
    private DcMotorEx intake = null;
    private DcMotorEx lifter = null;
    public Servo led = null;
    private double leftLauncherVelocity = 0;
    private double rightLauncherVelocity = 0;
    private double leftLauncherPower = 0;
    private double rightLauncherPower = 0;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    private double intakePower = 0;
    private double lifterVelocity = 0;
    private double max = 0;
    // These do NOT affect anything, but leave them as is! See notes in RemoteControlShooter for more information
    // These should be affecting RC, but they do not, and we fear that if we change them, everything will explode
    private AnalogInput laserAnalog;
    private GoBildaPrismDriver prism;
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;
    double launcherVelocity = 850;
    double intakeSpeed = 1.0;
    double distanceMM = 0;
    public GoBildaPinpointDriver odo = null;
    PrismAnimations.Solid green;
    boolean hasBall = false;
    boolean lightsGreen = true;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    final ElapsedTime runtime = new ElapsedTime();
    final ElapsedTime pinpointTime = new ElapsedTime();

    // During runtime

    public RockinBot(LinearOpMode opMode, String robotType) {
        o = opMode;
        o.telemetry.addData("This code was last updated", "9/23/2025, 2:45 pm"); // Todo: Update this date when the code is updated
        o.telemetry.update();

        if(robotType.equals("Shooter"))
            initializeShooterVar();
    }

    // Allow driving and braking
    public void initializeShooterVar()
    {
        //Launcher + intake variables
        leftLauncher = o.hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = o.hardwareMap.get(DcMotorEx.class, "right_launcher");
        laserAnalog = o.hardwareMap.get(AnalogInput.class, "beam");
        prism = o.hardwareMap.get(GoBildaPrismDriver.class,"prism");
        red = new PrismAnimations.Solid(Color.RED);
        green = new PrismAnimations.Solid(Color.GREEN);
        leftLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftLauncher.setDirection(DcMotorEx.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorEx.Direction.FORWARD);
        leftLauncher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLauncher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pidf = leftLauncher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidf.p = 40;
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        RobotLog.vv("Rockin' Robots", "PIDF changed. New p value: " + pidf.p);
        led = o.hardwareMap.get(Servo.class, "led");

        intake = o.hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Wheel variables
        leftFrontDrive = o.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = o.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = o.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = o.hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //Lifter Variables
        lifter = o.hardwareMap.get(DcMotorEx.class, "lifter");
        lifter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lifter.setDirection(DcMotorEx.Direction.REVERSE);
        lifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setTargetPosition(0);
        lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Initializes the pinpoint
        odo = o.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.resetPosAndIMU();
        odo.update();

        RobotLog.vv("Rockin' Robots", "Hardware Initialized");
    }

    // Remote control driving functions
    public void setWheelPower(double left_y, double left_x, double right_x, boolean park) {
        double wheelMultiplier = 1;

        if(park){
            wheelMultiplier = 0.25;
        }
        leftFrontPower = (left_y + left_x + right_x) * wheelMultiplier;
        rightFrontPower = (left_y - left_x - right_x) * wheelMultiplier;
        leftBackPower = (left_y - left_x + right_x) * wheelMultiplier;
        rightBackPower = (left_y + left_x - right_x) * wheelMultiplier;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void stopMoving() {
        RobotLog.vv("Rockin' Robots", "stopMoving()");
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void driveForward(int ms) {
        leftFrontDrive.setPower(0.5); // counter
        rightFrontDrive.setPower(0.5); // clock
        leftBackDrive.setPower(0.5); // clock
        rightBackDrive.setPower(0.5); // counter
        sleep(ms);
        stopMoving();
    }

    public void driveBack(int ms) {
        leftFrontDrive.setPower(-0.5); // counter
        rightFrontDrive.setPower(-0.5); // clock
        leftBackDrive.setPower(-0.5); // clock
        rightBackDrive.setPower(-0.5); // counter
        sleep(ms);
        stopMoving();
    }

    public void driveRight(int ms) {
        leftFrontDrive.setPower(0.5); // clock
        rightFrontDrive.setPower(0.5); // clock
        leftBackDrive.setPower(-0.5); // clock
        rightBackDrive.setPower(-0.5); // clock
        sleep(ms);
        stopMoving();
    }

    // Intake functions
    public void intakePower(double speed) {
        RobotLog.vv("Rockin' Robots", "intakePower(%.2f)", speed);
        intakeSpeed = speed;
        intake.setPower(speed);
    }

    // Launcher functions
    public void launcherVelocity(double power) {
        RobotLog.vv("Rockin' Robots", "launcherVelocity(%.2f)", power);
        launcherVelocity = power;
        leftLauncher.setVelocity(launcherVelocity);
        rightLauncher.setVelocity(launcherVelocity);
    }

    public void setpValue(double pValue){
        pidf.p += pValue;
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void waitForLaunchers(double target) {
        runtime.reset();
        RobotLog.vv("Rockin' Robots", "waitForLaunchers start: LauncherVelocity(): " + leftLauncher.getVelocity() + "/" + rightLauncher.getVelocity());
        while((leftLauncher.getVelocity() < target*0.97 || rightLauncher.getVelocity() < target*0.95) && runtime.seconds() < 1) {
            sleep(10);
        }
        RobotLog.vv("Rockin' Robots", "waitForLaunchers end LauncherVelocity(): " + leftLauncher.getVelocity() + "/" + rightLauncher.getVelocity());
    }

    // Lifter functions
    public void lifterPower(double power) {
        if(!lifter.isBusy()) {
            lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lifter.setVelocity(power);
        }
    }

    public void waitForLifter() {
        RobotLog.vv("Rockin' Robots", "waitForLifter LauncherVelocity(): " + leftLauncher.getVelocity() + "/" + rightLauncher.getVelocity());
        runtime.reset();
        while(lifter.isBusy() && runtime.seconds() < 2.5)
        {
            RobotLog.vv("Rockin' Robots", "waitForLifter: lifter is busy:" + lifter.getCurrentPosition());
            sleep(10);
        }
        RobotLog.vv("Rockin' Robots", "waitForLifter: " + lifter.getCurrentPosition());
    }

    public void turnLifterToDegrees(int degrees) {
        turnLifterToDegrees(degrees, 1500);
    }

    public void turnLifterToDegrees(int degrees, int velocity) {
        lifter.setTargetPosition((int) (degrees*3.9));
        lifter.setVelocity(velocity);
        lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RobotLog.vv("Rockin' Robots", "turnLifterToDegrees: " + degrees + " Current position: "+ lifter.getCurrentPosition()/3.9
                + " LauncherVelocity(): " + leftLauncher.getVelocity() + "/" + rightLauncher.getVelocity());
    }

    public void turnLifterByDegrees(int degrees) {
        lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RobotLog.vv("Rockin' Robots", "turnLifterByDegrees: Lifter position: "+ lifter.getCurrentPosition()
                + " LauncherVelocity(): " + leftLauncher.getVelocity() + "/" + rightLauncher.getVelocity());
        int moveToDegrees = (int)(lifter.getCurrentPosition()+(degrees*3.9));
        lifter.setTargetPosition(moveToDegrees);
        lifter.setVelocity(1500);
        lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    // Driving functions
    public boolean getPinpointPosition() {     // Finds robot position
        pinpointTime.reset();

        // Wait until Pinpoint is ready (up to 1s total).
        while (odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.NOT_READY
                && pinpointTime.seconds() < 1) {
            RobotLog.vv("Rockin' Robots", "Pinpoint NOT_READY, status=" + odo.getDeviceStatus());
            stopMoving();
            sleep(10);
        }
        if (odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
            RobotLog.vv("Rockin' Robots", "getPinpointPosition: timed out waiting for READY");
            return false;
        }

        // Read until we get non-NaN values, sharing the same 1s budget.
        do {
            odo.update();
            pos = odo.getPosition();
            xLoc = pos.getX(DistanceUnit.MM);
            yLoc = pos.getY(DistanceUnit.MM);
            hLoc = pos.getHeading(AngleUnit.DEGREES);
            if (!Double.isNaN(xLoc) && !Double.isNaN(yLoc) && !Double.isNaN(hLoc)) {
                RobotLog.vv("Rockin' Robots",
                        "Position: {X: %.1f, Y: %.1f, H: %.1f}", xLoc, yLoc, hLoc);
                return true;
            }
            RobotLog.vv("Rockin' Robots", "Pinpoint returned NaN, status=" + odo.getDeviceStatus());
            stopMoving();
            sleep(10);
        } while (pinpointTime.seconds() < 1);

        RobotLog.vv("Rockin' Robots", "getPinpointPosition: timed out with NaN values");
        return false;
    }

    public void driveToPos(double xTarget, double yTarget, double hTarget, double xyAccuracy, double hAccuracy, double maxDuration) {    // Defaults hAccuracy to 3 if no hAccuracy is given
        driveToPos(xTarget, yTarget, hTarget, xyAccuracy, hAccuracy, maxDuration, false);
    }

    public void driveToPos(double xTarget, double yTarget, double hTarget) {    // Defaults hAccuracy to 3 if no hAccuracy is given
        driveToPos(xTarget, yTarget, hTarget, 15, 3, 5, false);
    }

    public void driveToPos(double xTarget, double yTarget, double hTarget, double xyAccuracy, double hAccuracy, double maxDuration, boolean detect) {   // In millimeters
        if(!getPinpointPosition()) {
            RobotLog.vv("Rockin' Robots", "driveToPos ABORT: initial Pinpoint read failed");
            return;
        }

        boolean SeenBall = false;

        // Calculate distance from target
        double xDistance = xTarget - xLoc;
        double yDistance = yTarget - yLoc;
        double hDistance = hTarget - hLoc;
        // Prevent heading errors
        if (hDistance > 180) hDistance -= 360;
        if (hDistance < -180) hDistance += 360;
        double angleRadians;
        double xRotatedDistance;
        double yRotatedDistance;

        // ENTRY log: one line, all the parameters and the starting pose.
        RobotLog.vv("Rockin' Robots",
                "driveToPos START: target=(%.0f, %.0f, %.1f) accuracy=(xy=%.1f, h=%.1f) maxDur=%.2fs detect=%s | start=(%.0f, %.0f, %.1f) err=(%.0f, %.0f, %.1f)",
                xTarget, yTarget, hTarget, xyAccuracy, hAccuracy, maxDuration, detect,
                xLoc, yLoc, hLoc, xDistance, yDistance, hDistance);

        runtime.reset();
        // Stall detection: if we don't get measurably closer for STALL_TIMEOUT
        // seconds, give up so the rest of autonomous can keep running.
        final double STALL_TIMEOUT = 0.75;       // seconds
        final double STALL_PROGRESS_MM = 5.0;    // must close at least this much to count as progress
        double bestDistSq = xDistance*xDistance + yDistance*yDistance;
        double lastProgressTime = 0;
        // Per-loop log throttling: avoid spamming logs every ~10ms.
        final double LOG_INTERVAL = 0.25;        // seconds between iteration logs
        double lastLogTime = -LOG_INTERVAL;      // force log on first iteration
        int iterations = 0;
        String exitReason = "reached target";
        // While the program is running
        while (o.opModeIsActive()
                && runtime.seconds() < maxDuration
                && (Math.abs(xDistance) > xyAccuracy
                || Math.abs(yDistance) > xyAccuracy
                || Math.abs(hDistance) > hAccuracy)) {
            iterations++;

            angleRadians = Math.toRadians(hLoc);
            xRotatedDistance = xDistance * Math.cos(angleRadians) + yDistance * Math.sin(angleRadians);
            yRotatedDistance = -xDistance * Math.sin(angleRadians) + yDistance * Math.cos(angleRadians);

            // Proportional control: power = error * gain.
            // KP_XY:  1.0 power at 200 mm of translation error.
            // KP_H:   1.0 power at ~33 degrees of heading error.
            // Tune these (raise = more aggressive, lower = gentler/less overshoot).
            final double KP_XY = 0.005; // if it overshoots, lower this value. If it is too slow, raise this value.
            final double KP_H  = 0.03;
            double xCmd = xRotatedDistance * KP_XY;
            double yCmd = yRotatedDistance * KP_XY;
            double hCmd = hDistance        * KP_H;

            // Mecanum mixing
            leftFrontPower  = yCmd + xCmd - hCmd;
            rightFrontPower = yCmd - xCmd + hCmd;
            leftBackPower   = yCmd - xCmd - hCmd;
            rightBackPower  = yCmd + xCmd + hCmd;

            // Normalize the values so wheel power does not exceed 100%
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // If the largest commanded wheel power is below the motor's
            // stiction threshold, scale all four wheels up proportionally
            // so the largest equals MIN_DRIVE_POWER. This preserves the
            // direction/ratio of motion while making sure the robot
            // actually moves.
            final double MIN_DRIVE_POWER = 0.15; // if the robot is slow, raise this
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 0.001 && max < MIN_DRIVE_POWER) {
                double boost = MIN_DRIVE_POWER / max;
                leftFrontPower *= boost;
                rightFrontPower *= boost;
                leftBackPower *= boost;
                rightBackPower *= boost;
            }

            // Move wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Recalibrate position
            if(!getPinpointPosition()) {
                RobotLog.vv("Rockin' Robots", "driveToPos ABORT: Pinpoint read failed mid-move at t=%.2fs iter=%d",
                        runtime.seconds(), iterations);
                stopMoving();
                return;
            }
            xDistance = xTarget - xLoc;
            yDistance = yTarget - yLoc;
            hDistance = hTarget - hLoc;
            if (hDistance > 180) hDistance -= 360;
            if (hDistance < -180) hDistance += 360;

            // Throttled per-iteration log (~4 Hz). One line with everything
            // useful for debugging: time, current pose, remaining error, wheel powers.
            if (runtime.seconds() - lastLogTime >= LOG_INTERVAL) {
                lastLogTime = runtime.seconds();
                RobotLog.vv("Rockin' Robots",
                        "driveToPos t=%.2fs pose=(%.0f, %.0f, %.1f) err=(%.0f, %.0f, %.1f) pwr=(LF=%.2f RF=%.2f LB=%.2f RB=%.2f)",
                        runtime.seconds(), xLoc, yLoc, hLoc, xDistance, yDistance, hDistance,
                        leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }

            // Stall check: did we get meaningfully closer to the target?
            double distSq = xDistance*xDistance + yDistance*yDistance;
            double progressThresholdSq = bestDistSq - STALL_PROGRESS_MM*STALL_PROGRESS_MM;
            if (distSq < progressThresholdSq) {
                bestDistSq = distSq;
                lastProgressTime = runtime.seconds();
            } else if (runtime.seconds() - lastProgressTime > STALL_TIMEOUT) {
                exitReason = String.format("STALLED (no progress for %.2fs)", runtime.seconds() - lastProgressTime);
                break;
            }

            if (detect && hasBall() && !SeenBall) {
                SeenBall = true;
                RobotLog.vv("Rockin' Robots", "driveToPos: ball detected at t=%.2fs pose=(%.0f, %.0f, %.1f)",
                        runtime.seconds(), xLoc, yLoc, hLoc);
                turnLifterToDegrees(0, 700);
            }
            sleep(10); // Don't hog the CPU
        }
        // Finish up
        stopMoving();
        getPinpointPosition();
        if (!o.opModeIsActive()) {
            exitReason = "opMode stopped";
        } else if (runtime.seconds() >= maxDuration && exitReason.equals("reached target")) {
            exitReason = "TIMEOUT (maxDuration exceeded)";
        }
        // EXIT log: outcome, duration, iterations, final pose vs target.
        RobotLog.vv("Rockin' Robots",
                "driveToPos END: %s | duration=%.2fs iters=%d | final=(%.0f, %.0f, %.1f) target=(%.0f, %.0f, %.1f) err=(%.0f, %.0f, %.1f)",
                exitReason, runtime.seconds(), iterations,
                xLoc, yLoc, hLoc, xTarget, yTarget, hTarget,
                xTarget - xLoc, yTarget - yLoc, hTarget - hLoc);
    }

    // Ball detection functions
    public boolean hasBall() {
        double volts = laserAnalog.getVoltage();
        distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;

        if (distanceMM >= 200) {
            hasBall = true;
            if (!lightsGreen) {
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, green);
                lightsGreen = true;
            }
        }
        else {
            hasBall = false;
            if (lightsGreen) {
                //prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, TRANSPARENT);
                prism.clearAllAnimations();
                lightsGreen = false;
            }
        }
        return hasBall;
    }

    // Turns off the Prism LED strip
    public void lightsOff() {
        if (prism != null) {
            prism.clearAllAnimations();
        }
        lightsGreen = false;
    }

    // Log all (relevant) info about the robot on the hub.
    public void printDataOnScreen() {
        leftLauncherVelocity = leftLauncher.getVelocity();
        rightLauncherVelocity = rightLauncher.getVelocity();
        leftLauncherPower = leftLauncher.getCurrent(CurrentUnit.MILLIAMPS);
        rightLauncherPower = rightLauncher.getCurrent(CurrentUnit.MILLIAMPS);
        intakePower = intake.getCurrent(CurrentUnit.MILLIAMPS);
        lifterVelocity = lifter.getVelocity();
        PIDFCoefficients pidfActual = leftLauncher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        o.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        o.telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        o.telemetry.addData("Goal Launcher Velocity", "%.2f", launcherVelocity);
        o.telemetry.addData("Current Left Launcher Velocity and Power", "%.2f", leftLauncherVelocity, leftLauncherPower);
        o.telemetry.addData("Current Right Launcher Velocity and Power", "%.2f", rightLauncherVelocity, rightLauncherPower);
        o.telemetry.addData("Current Lifter Velocity", "%.2f", lifterVelocity);
        o.telemetry.addData("Intake Speed and Power", "%.2f", intakeSpeed, intakePower);
        o.telemetry.addData("P value: ", "%.2f", pidfActual.p);
        o.telemetry.addData("Has Ball:", "%B", hasBall());
        o.telemetry.addData("Ball Distance:", "%.2f", distanceMM);
        o.telemetry.update();

        RobotLog.vv("Rockin' Robots", "Launcher Velocity (l/r): %.2f, %.2f", leftLauncherVelocity, rightLauncherVelocity);

        /*dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("Left Launcher:", leftLauncherVelocity);
        dashboardTelemetry.addData("Right Launcher:", rightLauncherVelocity);
        dashboardTelemetry.addData("Intake", intakePower);
        dashboardTelemetry.addData("Left Front Wheel", leftFrontPower);
        dashboardTelemetry.addData("Right Front Wheel", rightFrontPower);
        dashboardTelemetry.addData("Left Back Wheel", leftBackPower);
        dashboardTelemetry.addData("Right Back Wheel", rightBackPower);
        dashboardTelemetry.update();*/
    }
}
