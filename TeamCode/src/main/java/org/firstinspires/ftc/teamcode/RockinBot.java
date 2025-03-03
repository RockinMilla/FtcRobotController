package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RockinBot {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    private double max = 0;
    public boolean wheelClimb = false;

    // This chunk controls our vertical
    private DcMotor vertical = null;
    public final int VERTICAL_MIN = 0;
    public final int VERTICAL_MAX = 2155;
    public final int VERTICAL_MAX_VIPER = 1200;
    public final int VERTICAL_CLIMB_POSITION = 2300;
    public final int VERTICAL_DRIVE_POSITION = 400;
    public final int VERTICAL_DEFAULT_SPEED = 2000;
    public int verticalAdjustedMin = 0;
    private int verticalPosition = VERTICAL_MIN;

    // This chunk controls our viper slide
    private DcMotor viperSlide = null;
    public final int VIPER_MAX_WIDE = 1800;
    public final int VIPER_MAX_TALL = 2637;
    public final int VIPER_MIN = 0;
    private int viperSlidePosition = VIPER_MIN;

    // This chunk controls our claw
    private Servo claw = null;
    public final double CLAW_MIN = 0.2;        // Claw is closed
    public final double CLAW_MAX = 0.36;       // Claw is open - Og non-wrist value was 0.8

    // This chunk controls our wrist
    private Servo wrist = null;
    public final double WRIST_PICKUP = 0.23;       // Wrist is in intake position (picking up)
    public final double WRIST_DROPOFF = 0.89;      // Wrist is in outtake position (dropping in basket)

    // This chunk controls our nose picker (ascent stick)
    private Servo ascentStick = null;
    public final double ASCENT_MIN = 0.2;          // Stick is down
    public final double ASCENT_MAX = 0.49;         // Stick is up


    RockinBot() {
        telemetry.addData("This code was last updated", "1/17/2024, 11:47 am"); // Todo: Update this date when the code is updated
        telemetry.update();
        initializeHardwareVariables();
    }

    public void initializeHardwareVariables() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        vertical = hardwareMap.get(DcMotor.class, "vertical");
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        viperSlide = hardwareMap.get(DcMotor.class, "viper_slide");
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlide.setDirection(DcMotor.Direction.REVERSE);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_MAX);

        // todo: check this
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        setWrist(WRIST_DROPOFF);

        ascentStick = hardwareMap.get(Servo.class, "ascentStick");
        ascentStick.setDirection(Servo.Direction.REVERSE);
    }

    public void setWheelPower(double left_y, double left_x, double right_x){
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

        if(!wheelClimb) {
            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }
    }

    public void activeClimb() {
        if (getVertical() > 100) {
            wheelClimb = true;
            vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vertical.setPower(-0.8);
            leftBackDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
        }
        else {
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            wheelClimb = false;
            setVertical(VERTICAL_MIN, 1000);
        }
    }

    public void tScoringPosition() {
        RobotLog.vv("Rockin' Robots", "Get in scoring position");
        while (getVertical() < 1000 && getViper() > 500) {
            setViper(VIPER_MIN, 4000);
        }
        while (getVertical() < 1700) {
            setVertical(VERTICAL_MAX, 3000);
        }
        setViper(VIPER_MAX_TALL, 4000);
        setWrist(WRIST_DROPOFF);
    }

    public void oPickupPosition() {
        RobotLog.vv("Rockin' Robots", "Get in pickup position");
        while (getViper() > 500) {
            setViper(VIPER_MIN, 4000);
        }
        setVertical(170, 3000);
        while (getVertical() > 500) {
            setVertical(200, 3000);
        }
        setViper(VIPER_MAX_WIDE, 4000);
        setWrist(WRIST_PICKUP);
        setClaw(CLAW_MAX);
    }

    public void sDrivingPosition() {
        RobotLog.vv("Rockin' Robots", "sDrivingPosition()");
        setWrist(WRIST_DROPOFF);
        if (getVertical() < 500)
        {
            RobotLog.vv("Rockin' Robots", "sDrivingPosition(): Starting from low position");
            setVertical(VERTICAL_DRIVE_POSITION, 3000);
            setViper(VIPER_MIN, 4000);
            while (getViper() > 500) {
                RobotLog.vv("Rockin' Robots", "sDrivingPosition(): while waiting for viper from low position");
                setViper(VIPER_MIN, 4000);
            }
        }
        else {
            RobotLog.vv("Rockin' Robots", "sDrivingPosition(): Starting from high position");
            setViper(VIPER_MIN, 4000);
            while (getViper() > 500) {
                RobotLog.vv("Rockin' Robots", "sDrivingPosition(): while waiting for viper from high position");
                setViper(VIPER_MIN, 4000);
            }
            setVertical(VERTICAL_DRIVE_POSITION, 3000);
        }
    }

    public void xClosedPosition() {
        RobotLog.vv("Rockin' Robots", "Get in closed position");
        setWrist(WRIST_DROPOFF);
        setViper(VIPER_MIN, 4000);
        while (getViper() > 900) {
            setViper(VIPER_MIN, 4000);
        }
        setVertical(VERTICAL_MIN, 3000);
        while (getVertical() > 500) {
            setVertical(VERTICAL_MIN, 3000);
        }
        setClaw(CLAW_MIN);
    }

    public void setAscentStick(double target) {
        RobotLog.vv("Rockin' Robots", "Set Ascent Stick to: %4.2f, Current: %4.2f", target, ascentStick.getPosition());
        ascentStick.setPosition(target);
        //sleep(1000);
        RobotLog.vv("Rockin' Robots", "Target: %4.2f, Current: %4.2f", target, ascentStick.getPosition());
    }

    public double getAscentStick() {
        return ascentStick.getPosition();
    }

    public void setClaw(double target) {
        RobotLog.vv("Rockin' Robots", "Set Claw to: %4.2f, Current: %4.2f", target, claw.getPosition());
        claw.setPosition(target);
        RobotLog.vv("Rockin' Robots", "Target: %4.2f, Current: %4.2f", target, claw.getPosition());
    }

    public void disableClaw() {
        claw.close();
    }

    public double getClaw() {
        return claw.getPosition();
    }

    public void setWrist(double target) { // todo: check this method
        RobotLog.vv("Rockin' Robots", "Set Wrist to: %4.2f, Current: %4.2f", target, wrist.getPosition());
        wrist.setPosition(target);
        //sleep(1000);
        RobotLog.vv("Rockin' Robots", "Target: %4.2f, Current: %4.2f", target, wrist.getPosition());
    }

    public double getWrist() {
        return wrist.getPosition();
    }

    public void setVertical(int height){
        setVertical(height, VERTICAL_DEFAULT_SPEED);
    }

    public void setVertical(int height, int speed){
        vertical.setTargetPosition(height);
        ((DcMotorEx) vertical).setVelocity(speed);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getVertical() {
        return verticalPosition;
    }

    public void updateVertical() {
        verticalPosition = vertical.getCurrentPosition();
    }

    public void setViper(int length, int speed){
        viperSlide.setTargetPosition(length);
        ((DcMotorEx) viperSlide).setVelocity(speed);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.vv("Rockin' Robots", "Viper set to %d", viperSlide.getCurrentPosition());
    }

    public double getViper() {
        return viperSlidePosition;
    }

    public void updateViper() {
        viperSlidePosition = viperSlide.getCurrentPosition();
    }

    // Log all (relevant) info about the robot on the hub.
    public void printDataOnScreen() {
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Claw position", "%4.2f", claw.getPosition());
        telemetry.addData("Viper Slide Velocity", "%4.2f", ((DcMotorEx) viperSlide).getVelocity());
        telemetry.addData("Viper power consumption", "%.1f", ((DcMotorEx) viperSlide).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Viper Slide Position", "%d", viperSlidePosition);
        telemetry.addData("Vertical Power", "%.1f", ((DcMotorEx) vertical).getVelocity());
        telemetry.addData("Vertical power consumption", "%.1f", ((DcMotorEx) vertical).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Vertical Position", "%d", vertical.getCurrentPosition());
        telemetry.addData("Vertical Adjusted Min", "%d", verticalAdjustedMin);
        telemetry.addData("wrist position", "%4.2f", wrist.getPosition());

        telemetry.update();
    }
}