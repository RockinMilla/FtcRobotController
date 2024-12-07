package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Vertical Tester", group="Linear OpMode")
public class VerticalTester extends LinearOpMode {

    // This chunk controls our vertical
    DcMotor vertical = null;
    final int VERTICAL_MIN = 0;
    final int VERTICAL_MAX = 1700;
    int verticalAdjustedMin = 0;
    int verticalPosition = VERTICAL_MIN;

    // This chunk controls our viper slide
    DcMotor viperSlide = null;
    final int VIPER_MAX_WIDE = 2100;
    final int VIPER_MAX_TALL = 3100;
    final int VIPER_MIN = 0;
    int viperSlidePosition = 0;

    final ElapsedTime runtime = new ElapsedTime();

    @Override
    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {

        initializeHardwareVariables();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            verticalPosition = vertical.getCurrentPosition();

            // Control the viper slide - how much it extends
            viperSlidePosition = viperSlide.getCurrentPosition();
            // Extend the viper
            if (gamepad1.right_trigger > 0 && viperSlidePosition < VIPER_MAX_TALL) {
                viperSlide.setTargetPosition(viperSlidePosition + 200);
                ((DcMotorEx) viperSlide).setVelocity(gamepad1.right_trigger*4000);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // Retract the viper
            else if (gamepad1.left_trigger > 0) {
                viperSlide.setTargetPosition(viperSlidePosition - 200);
                ((DcMotorEx) viperSlide).setVelocity(gamepad1.left_trigger*4000);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // Y/Triangle: Go to max vertical
            if (gamepad1.y) {
                setVertical(VERTICAL_MAX, 3000);
            }

            // A/X button: Go to min vertical
            if (gamepad1.a) {
                setVertical(VERTICAL_MIN, 3000);
            }

            // Show the elapsed game time and wheel power.
            printDataOnScreen();
        }
    }

    private void initializeHardwareVariables() {
        vertical = hardwareMap.get(DcMotor.class, "vertical");
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        viperSlide = hardwareMap.get(DcMotor.class, "viper_slide");
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlide.setDirection(DcMotor.Direction.REVERSE);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setVertical(int height, int speed){
        verticalPosition = vertical.getCurrentPosition();
        double maxSpeed = 0;
        RobotLog.vv("Rockin' Robots", "Vertical set to %d, target is %d", verticalPosition, height);
        vertical.setTargetPosition(height);
        ((DcMotorEx) vertical).setVelocity(speed);
        runtime.reset();
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (verticalPosition < height - 50 || verticalPosition > height + 50) {
            verticalPosition = vertical.getCurrentPosition();
            if(((DcMotorEx) vertical).getVelocity() > maxSpeed){
                maxSpeed = ((DcMotorEx) vertical).getVelocity();
            }
        }
        telemetry.addData("Vertical set to", "%d, in %4.2f ms, at speed %4.2f", verticalPosition, runtime.milliseconds(), maxSpeed);
    }
    public void setViper(int length, int speed){
        viperSlide.setTargetPosition(length);
        ((DcMotorEx) viperSlide).setVelocity(speed);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.vv("Rockin' Robots", "Viper set to %d", viperSlide.getCurrentPosition());
    }
    // Log all (relevant) info about the robot on the hub.
    private void printDataOnScreen() {
        telemetry.addData("Run Time", "%.1f", runtime.seconds());
        telemetry.addData("Viper Slide Velocity", "%4.2f", ((DcMotorEx) viperSlide).getVelocity());
        telemetry.addData("Viper power consumption", "%.1f", ((DcMotorEx) viperSlide).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Viper Slide Position", "%d", viperSlidePosition);
        telemetry.addData("Vertical Power", "%.1f", ((DcMotorEx) vertical).getVelocity());
        telemetry.addData("Vertical power consumption", "%.1f", ((DcMotorEx) vertical).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Vertical Position", "%d", vertical.getCurrentPosition());
        telemetry.addData("Vertical Adjusted Min", "%d", verticalAdjustedMin);
        RobotLog.vv("Rockin", "Vert Velocity: %.1f, Vert Power: %.1f, Vert Power Consumption: %.1f, Vert Position: %d",
                ((DcMotorEx) vertical).getVelocity(),  vertical.getPower(), ((DcMotorEx)vertical).getCurrent(CurrentUnit.AMPS), vertical.getCurrentPosition());

        telemetry.update();
    }
}