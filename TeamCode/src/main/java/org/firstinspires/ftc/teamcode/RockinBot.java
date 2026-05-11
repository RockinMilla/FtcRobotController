package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;

// All the things that we use and borrow
import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import static org.firstinspires.ftc.teamcode.Prism.Direction.Forward;

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
    public static double ORIGIN_ZEROHEADING = Math.PI / 2;
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
    PrismAnimations.Solid red;
    PrismAnimations.Solid green;
    PrismAnimations.Solid TRANSPARENT;
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
        else if(robotType.equals("Driver"))
            initializeDriverVar();
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

        // Robot orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    }

    // Allow driving and braking
    public void initializeDriverVar() {
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

        RobotLog.vv("Rockin' Robots", "Hardware Initialized");

        // Initializes the pinpoint
        odo = o.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.resetPosAndIMU();
        odo.update();
        RobotLog.vv("Rockin' Robots", "Device Status: " + odo.getDeviceStatus());

        // Robot orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    }

    public void setAllToSame(double power) {
    // Send calculated power to wheels
        leftFrontDrive.setPower(-1*power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-1*power);
    }

    public void setWheelPower(double left_y, double left_x, double right_x, boolean park){        // Wheel power and speed
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
        rightBackDrive.setPower(-0.5); // clocksetVelocity
        sleep(ms);
        stopMoving();
    }

    public void launcherVelocity(double power) {
        launcherVelocity = power;
        leftLauncher.setVelocity(launcherVelocity);
        rightLauncher.setVelocity(launcherVelocity);
    }

    public void setpValue(double pValue){
        pidf.p += pValue;
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void intakePower(double speed) {
        intakeSpeed = speed;
        intake.setPower(speed);
    }

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

    public void waitForLaunchers(double target) {
        runtime.reset();
        RobotLog.vv("Rockin' Robots", "waitForLaunchers start: LauncherVelocity(): " + leftLauncher.getVelocity() + "/" + rightLauncher.getVelocity());
        while((leftLauncher.getVelocity() < target*0.97 || rightLauncher.getVelocity() < target*0.95) && runtime.seconds() < 1) {
            sleep(10);
        }
        RobotLog.vv("Rockin' Robots", "waitForLaunchers end LauncherVelocity(): " + leftLauncher.getVelocity() + "/" + rightLauncher.getVelocity());
    }

    public void turnLifterByDegreesRC(int degrees, int velocity) {
        if(!lifter.isBusy())
        {
            lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            turnLifterByDegrees(degrees, 2000, 5);
        }
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
        turnLifterByDegrees(degrees, 2000, 5);
    }

    public void turnLifterByDegrees(int degrees, int velocity) {
        turnLifterByDegrees(degrees, velocity, 5);
    }

    public void turnLifterByDegrees(int degrees, int velocity, int maxDuration) {
        lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RobotLog.vv("Rockin' Robots", "turnLifterByDegrees: Lifter position: "+ lifter.getCurrentPosition()
                + " LauncherVelocity(): " + leftLauncher.getVelocity() + "/" + rightLauncher.getVelocity());
        int moveToDegrees = (int)(lifter.getCurrentPosition()+(degrees*3.9));
        lifter.setTargetPosition(moveToDegrees);
        lifter.setVelocity(velocity);
        lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public boolean getPinpointPosition() {     // Finds robot position
        pinpointTime.reset();
        while(odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.NOT_READY && pinpointTime.seconds() < 1) {
            RobotLog.vv("Rockin' Robots", "Device Status: " + odo.getDeviceStatus());
            stopMoving();
            sleep(10);
        }
        if(pinpointTime.seconds() > 1)
        {
            return false;
        }
        odo.update();
        pos = odo.getPosition();
        xLoc = pos.getX(DistanceUnit.MM);
        yLoc = pos.getY(DistanceUnit.MM);
        hLoc = pos.getHeading(AngleUnit.DEGREES);

        while((Double.isNaN(xLoc) || Double.isNaN(yLoc) || Double.isNaN(hLoc)) && pinpointTime.seconds() < 1) {
            RobotLog.vv("Rockin' Robots", "Device Status: " + odo.getDeviceStatus());
            stopMoving();
            sleep(10);
        }
        String data = String.format("{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        RobotLog.vv("Rockin' Robots", "Position: " + data);
        return true;
    }
    public void driveToPos(double xTarget, double yTarget, double hTarget, double xyAccuracy, double hAccuracy, double maxDuration) {    // Defaults hAccuracy to 3 if no hAccuracy is given
        driveToPos(xTarget, yTarget, hTarget, 15, 3, 5, false);
    }

    public void driveToPos(double xTarget, double yTarget, double hTarget) {    // Defaults hAccuracy to 3 if no hAccuracy is given
        driveToPos(xTarget, yTarget, hTarget, 15, 3, 5, false);
    }

    public void driveToPos(double xTarget, double yTarget, double hTarget, double xyAccuracy, double hAccuracy, double maxDuration, boolean detect) {   // In millimeters
        if(!getPinpointPosition()) {
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
        double angleRadians = Math.toRadians(hLoc);
        double xRotatedDistance = xDistance * Math.cos(angleRadians) + yDistance * Math.sin(angleRadians);
        double yRotatedDistance = xDistance * Math.sin(angleRadians) + yDistance * Math.cos(angleRadians);

        //RobotLog.vv("Rockin' Robots", "driveToPos() xTarget: %.2f, yTarget: %.2f, hTarget: %.2f, xyAccuracy: %.2f, hAccuracy: %.2f",
        //        xTarget, yTarget, hTarget, xyAccuracy, hAccuracy);

        runtime.reset();
        // While the program is running
        while (o.opModeIsActive()
                && runtime.seconds() < maxDuration
                && (Math.abs(xDistance) > xyAccuracy
                || Math.abs(yDistance) > xyAccuracy
                || Math.abs(hDistance) > hAccuracy)) {

            xRotatedDistance = xDistance * Math.cos(angleRadians) + yDistance * Math.sin(angleRadians);
            yRotatedDistance = -xDistance * Math.sin(angleRadians) + yDistance * Math.cos(angleRadians);
            angleRadians = Math.toRadians(hLoc);

            // Set wheel power
            leftFrontPower = (yRotatedDistance + xRotatedDistance - hDistance*3);
            rightFrontPower = (yRotatedDistance - xRotatedDistance + hDistance*3);
            leftBackPower = (yRotatedDistance - xRotatedDistance - hDistance*3);
            rightBackPower = (yRotatedDistance + xRotatedDistance + hDistance*3);

            //RobotLog.vv("Rockin' Robots", "Wheel power: %.2f, %.2f, %.2f, %.2f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
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
            leftFrontPower *= 0.75;
            rightFrontPower *= 0.75;
            leftBackPower *= 0.75;
            rightBackPower *= 0.75;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max < 0.05) {
                leftFrontPower *= 1.5;
                rightFrontPower *= 1.5;
                leftBackPower *= 1.5;
                rightBackPower *= 1.5;
            }

            // Move wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Recalibrate position
            if(getPinpointPosition() == false) {
                stopMoving();
                return;
            }
            xDistance = xTarget - xLoc;
            yDistance = yTarget - yLoc;
            hDistance = hTarget - hLoc;
            if (hDistance > 180) hDistance -= 360;
            if (hDistance < -180) hDistance += 360;

            RobotLog.vv("Rockin' Robots", "Moving: xDist: %.2f, yDist: %.2f, hDist: %.2f",
                    xDistance, yDistance, hDistance);

            if (detect && hasBall() && !SeenBall) {
                SeenBall = true;
                RobotLog.vv("Rockin' Robots", "Seen Ball");
                turnLifterToDegrees(0, 700);
            }
        }
        // Finish up
        stopMoving();
        getPinpointPosition();
        RobotLog.vv("Rockin' Robots", "Done Moving: duration: %.2f, xLoc: %d/%d, yLoc: %d/%d, hLoc:  %d/%d",
                runtime.seconds(), Math.round(xLoc), Math.round(xTarget), Math.round(yLoc), Math.round(yTarget), Math.round(hLoc), Math.round(hTarget));
    }

    public double getXloc()
    {
        return pos.getX(DistanceUnit.INCH);
    }
    public double getYloc()
    {
        return pos.getY(DistanceUnit.INCH);
    }
    public double getHLoc()
    {
        return pos.getHeading(AngleUnit.RADIANS);
    }
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

        dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("Left Launcher:", leftLauncherVelocity);
        dashboardTelemetry.addData("Right Launcher:", rightLauncherVelocity);
        dashboardTelemetry.addData("Intake", intakePower);
        dashboardTelemetry.addData("Left Front Wheel", leftFrontPower);
        dashboardTelemetry.addData("Right Front Wheel", rightFrontPower);
        dashboardTelemetry.addData("Left Back Wheel", leftBackPower);
        dashboardTelemetry.addData("Right Back Wheel", rightBackPower);
        //dashboardTelemetry.update();
    }

}