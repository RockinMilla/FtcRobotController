package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RockinBot {

    private DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;
    private double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftBackPower = 0;
    double rightBackPower = 0;
    double max = 0;
    boolean wheelClimb = false;

    // Collect joystick position data
    double axial = 0;
    double lateral = 0;
    double yaw = 0;

    // This chunk controls our vertical
    DcMotor vertical = null;
    final int VERTICAL_MIN = 0;
    final int VERTICAL_MAX = 2155;
    final int VERTICAL_MAX_VIPER = 1200;
    final int VERTICAL_CLIMB_POSITION = 2300;
    final int VERTICAL_DRIVE_POSITION = 400;
    final int VERTICAL_DEFAULT_SPEED = 2000;
    int verticalAdjustedMin = vertAdjMin;
    int verticalPosition = vertPos;

    // This chunk controls our viper slide
    DcMotor viperSlide = null;
    final int VIPER_MAX_WIDE = 1800;
    final int VIPER_MAX_TALL = 2637;
    final int VIPER_MIN = 0;
    int viperSlidePosition = vipPos;

    // This chunk controls our claw
    Servo claw = null;
    final double CLAW_MIN = 0.2;        // Claw is closed
    final double CLAW_MAX = 0.36;       // Claw is open - Og non-wrist value was 0.8
    double claw_position = clawPos;

    // This chunk controls our wrist
    Servo wrist = null;
    final double WRIST_PICKUP = 0.23;       // Wrist is in intake position (picking up)
    final double WRIST_DROPOFF = 0.89;      // Wrist is in outtake position (dropping in basket)
    double wrist_position = wristPos;

    // This chunk controls our nose picker (ascent stick)
    Servo ascentStick = null;
    final double ASCENT_MIN = 0.2;          // Stick is down
    final double ASCENT_MAX = 0.49;         // Stick is up


    RockinBot(int vertAdjMin, int vertPos, int vipPos, double clawPos, double wristPos) {

        final ElapsedTime runtime = new ElapsedTime();

        telemetry.addData("This code was last updated", "1/17/2024, 11:47 am"); // Todo: Update this date when the code is updated
        telemetry.update();
        initializeHardwareVariables();
    }

    setAscentStick(ASCENT_MIN);
    //claw.setPosition(CLAW_MAX);
    runtime.reset();

    private void initializeHardwareVariables() {
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

    public void setWheelPower(){
        leftFrontPower = (axial + lateral + yaw) * 0.75;
        rightFrontPower = (axial - lateral - yaw) * 0.75;
        leftBackPower = (axial - lateral + yaw) * 0.75;
        rightBackPower = (axial + lateral - yaw) * 0.75;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
    }

    private double getWheelPower() {
        return "leftFront " + leftFrontPower + " rightFront " + rightFrontPower + " leftBack " + leftBackPower + " rightBack " + rightBackPower;
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
        return vertical.getPosition();
    }

    public void setViper(int length, int speed){
        viperSlide.setTargetPosition(length);
        ((DcMotorEx) viperSlide).setVelocity(speed);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.vv("Rockin' Robots", "Viper set to %d", viperSlide.getCurrentPosition());
    }

    public double getViper() {
        return viperSlide.getCurrentPosition();
    }

    // Log all (relevant) info about the robot on the hub.
    private void printDataOnScreen() {
        telemetry.addData("Run Time", "%.1f", runtime.seconds());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        //RobotLog.vv("RockinRobots", "%4.2f, %4.2f, %4.2f, %4.2f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("Joystick Axial", "%4.2f", axial);
        telemetry.addData("Joystick Lateral", "%4.2f", lateral);
        telemetry.addData("Joystick Yaw", "%4.2f", yaw);
        telemetry.addData("Target claw position", "%4.2f", claw_position);
        telemetry.addData("Claw position", "%4.2f", claw.getPosition());
        telemetry.addData("Viper Slide Velocity", "%4.2f", ((DcMotorEx) viperSlide).getVelocity());
        telemetry.addData("Viper power consumption", "%.1f", ((DcMotorEx) viperSlide).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Viper Slide Position", "%d", viperSlidePosition);
        telemetry.addData("Vertical Power", "%.1f", ((DcMotorEx) vertical).getVelocity());
        telemetry.addData("Vertical power consumption", "%.1f", ((DcMotorEx) vertical).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Vertical Position", "%d", vertical.getCurrentPosition());
        telemetry.addData("Vertical Adjusted Min", "%d", verticalAdjustedMin);
        telemetry.addData("wrist position", "%4.2f", wrist.getPosition());
        //RobotLog.vv("Rockin", "Vert Velocity: %.1f, Vert Power: %.1f, Vert Power Consumption: %.1f, Vert Position: %d",
        //        ((DcMotorEx) vertical).getVelocity(),  vertical.getPower(), ((DcMotorEx)vertical).getCurrent(CurrentUnit.AMPS), vertical.getCurrentPosition());

        telemetry.update();
    }
}