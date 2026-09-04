package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;

// All the things that we use and borrow
import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;

public class RockinBot {
    // Motors and sensors
    private LinearOpMode o;
    private DcMotorEx intake = null;
    private Pose2D pos;
    private Pose3D botPose;
    private double xLoc = 0;
    private double yLoc = 0;
    private double hLoc = 0;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Limelight3A limelight;
    public GoBildaPinpointDriver odo = null;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    private double intakePower = 0;
    private double max = 0;
    double intakeSpeed = 1.0;
    // These do NOT affect anything, but leave them as is! See notes in RemoteControlShooter for more information
    // These should be affecting RC, but they do not, and we fear that if we change them, everything will explode
    final ElapsedTime runtime = new ElapsedTime();
    final ElapsedTime pinpointTime = new ElapsedTime();

    // During runtime

    public RockinBot(LinearOpMode opMode, String robotType) {
        o = opMode;
        o.telemetry.addData("This code was last updated", "8/18/2025, 2:45 pm"); // Todo: Update this date when the code is updated
        o.telemetry.update();

        if(robotType.equals("Shooter"))
        {
            initializeDrivingVar();
        }
    }

    // Allow driving and braking
    public void initializeDrivingVar() {
        // Wheel variables
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

        // Initializes the pinpoint
        odo = o.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.resetPosAndIMU();
        odo.update();

        limelight = o.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        intake = o.hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        RobotLog.vv("Rockin' Robots", "Hardware Initialized");
    }

    public void start()
    {
        limelight.start();
    }

    public void loop(){
        odo.update();
        limelight.updateRobotOrientation(odo.getHeading());
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid())
        {
            o.telemetry.addData("X", result.getTx());
            o.telemetry.addData("Y", result.getTy());
            o.telemetry.addData("A", result.getTa());
            o.telemetry.addData("BotPose", botPose);
            o.telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            getPinpointPosition();
        }

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
        } else if (max <= 0.05) {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        RobotLog.vv("Rockin' Robots", "Wheel power: %.2f, %.2f, %.2f, %.2f",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

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

    public void intakePower(double speed) {
        RobotLog.vv("Rockin' Robots", "intakePower(%.2f)", speed);
        intakeSpeed = speed;
        intake.setPower(speed);
    }

    private boolean inBand(double actual, double lower, double upper) {
        return actual >= lower && actual <= upper;
    }

    // Driving functions

    // noStop=true skips the stopMoving() call at the end so the robot rolls
    // straight into the next driveToPos without braking (saves time when
    // chaining moves). Heading/accuracy still need to be reached (or maxDuration/stall).
    // Log all (relevant) info about the robot on the hub.
    public void printDataOnScreen() {

        intakePower = intake.getCurrent(CurrentUnit.MILLIAMPS);

        o.telemetry.addData("Intake Speed and Power", "%.2f", intakeSpeed, intakePower);
        o.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        o.telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        loop();

        o.telemetry.update();

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
