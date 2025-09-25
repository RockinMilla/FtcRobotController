package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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
    private DcMotor leftLauncher = null;
    private DcMotor rightLauncher = null;
    private DcMotor intake = null;
    private double leftLauncherPower = 0;
    private double rightLauncherPower = 0;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    private double intakePower = 0;
    private double max = 0;
    public GoBildaPinpointDriver odo = null;

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
    public void initializeShooterVar() {
        //Launcher variables
        leftLauncher = o.hardwareMap.get(DcMotor.class, "left_launcher");
        rightLauncher = o.hardwareMap.get(DcMotor.class, "right_launcher");
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        intake = o.hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.REVERSE);

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

        // Robot orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
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

    public void setWheelPower(double left_y, double left_x, double right_x){        // Wheel power and speed
        leftFrontPower = (left_y + left_x + right_x) * 0.75;
        rightFrontPower = (left_y - left_x - right_x) * 0.75;
        leftBackPower = (left_y - left_x + right_x) * 0.75;
        rightBackPower = (left_y + left_x - right_x) * 0.75;

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

    public void launcherPower(double power) {
        leftLauncher.setPower(power);
        rightLauncher.setPower(power);
    }

    public void intakePower(double speed) {
        intake.setPower(speed);
    }

    public void getPinpointPosition() {     // Finds robot position
        RobotLog.vv("Rockin' Robots", "Device Status: " + odo.getDeviceStatus());
        odo.update();
        pos = odo.getPosition();
        xLoc = pos.getX(DistanceUnit.MM);
        yLoc = pos.getY(DistanceUnit.MM);
        hLoc = pos.getHeading(AngleUnit.DEGREES);
        String data = String.format("{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        RobotLog.vv("Rockin' Robots", "Position: " + data);
    }

    public void driveToPos(double xTarget, double yTarget, double hTarget) {    // Defaults hAccuracy to 3 if no hAccuracy is given
        driveToPos(xTarget, yTarget, hTarget, 15, 3); //todo: should hAccuracy be 3? What unit is it?
    }

    public void driveToPos(double xTarget, double yTarget, double hTarget, double xyAccuracy, double hAccuracy) {   // In millimeters
        getPinpointPosition();      // Get initial position

        // Calculate distance from target
        double xDistance = xTarget - xLoc;
        double yDistance = yTarget - yLoc;
        double hDistance = hTarget - hLoc;
        // Prevent heading errors
        if (hDistance > 180) hDistance -= 360;
        if (hDistance < -180) hDistance += 360;
        double angleRadians = Math.toRadians(hLoc);
        double xRotatedDistance = xDistance * Math.cos(angleRadians) + yDistance * Math.sin(angleRadians);
        double yRotatedDistance = -xDistance * Math.sin(angleRadians) + yDistance * Math.cos(angleRadians);

        RobotLog.vv("Rockin' Robots", "driveToPos() xTarget: %.2f, yTarget: %.2f, hTarget: %.2f, xyAccuracy: %.2f, hAccuracy: %.2f",
                xTarget, yTarget, hTarget, xyAccuracy, hAccuracy);

        // While the program is running
        while (o.opModeIsActive() && Math.abs(xDistance) > xyAccuracy
                || Math.abs(yDistance) > xyAccuracy
                || Math.abs(hDistance) > hAccuracy) {

            // Set wheel power
            leftFrontPower = (yRotatedDistance + xRotatedDistance - hDistance);
            rightFrontPower = (yRotatedDistance - xRotatedDistance + hDistance);
            leftBackPower = (yRotatedDistance - xRotatedDistance - hDistance);
            rightBackPower = (yRotatedDistance + xRotatedDistance + hDistance);
            RobotLog.vv("Rockin' Robots", "Wheel power: %.2f, %.2f, %.2f, %.2f");
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
            leftFrontPower *= 0.5;
            rightFrontPower *= 0.5;
            leftBackPower *= 0.5;
            rightBackPower *= 0.5;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max < 0.2) {
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
            getPinpointPosition();
            xDistance = xTarget - xLoc;
            yDistance = yTarget - yLoc;
            hDistance = hTarget - hLoc;
            if (hDistance > 180) hDistance -= 360;
            if (hDistance < -180) hDistance += 360;

            angleRadians = Math.toRadians(hLoc);
            xRotatedDistance = xDistance * Math.cos(angleRadians) + yDistance * Math.sin(angleRadians);
            yRotatedDistance = -xDistance * Math.sin(angleRadians) + yDistance * Math.cos(angleRadians);
            RobotLog.vv("Rockin' Robots", "Moving: xDist: %.2f, yDist: %.2f, hDist: %.2f",
                    xDistance, yDistance, hDistance);
        }
        // Finish up
        stopMoving();
        getPinpointPosition();
        RobotLog.vv("Rockin' Robots", "Done Moving: xLoc: %.2f, yLoc: %.2f, hLoc:  %.2f, leftFrontPower: %.2f, rightFrontPower: %.2f, leftBackPower: %.2f, rightBackPower: %.2f",
                xLoc, yLoc, hLoc, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    // Log all (relevant) info about the robot on the hub.
    public void printDataOnScreen() {
        o.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        o.telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

        o.telemetry.update();
    }
}