package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Auto Drive", group="Robot")
public class AutoDrive extends LinearOpMode {

    // Create a LinearOpMode variable so you can pass it to the RockinBot constructor
    RockinBot r = new RockinBot();

    // Initialize all variables for the program
    // Hardware variables
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    double max = 0;

    // This chunk controls our vertical
    DcMotor vertical = null;
    final int VERTICAL_MIN = 0;
    final int VERTICAL_MAX = 2150;
    final int VERTICAL_DEFAULT_SPEED = 2000;

    // This chunk controls our viper slide
    DcMotor viperSlide = null;
    final int VIPER_MIN = 0;
    final int VIPER_MAX = 2540;
    final int VIPER_GROUND = 1000;
    final int VIPER_DEFAULT_SPEED = 3000;

    // This chunk controls our claw
    //Callie
    Servo claw = null;
    final double CLAW_MIN = 0.17;           // Claw is closed
    final double CLAW_MAX = 0.5;          // Claw is open - Og non-wrist value was 0.8

    // This chunk controls our wrist
    Servo wrist = null;
    final double WRIST_PICKUP = 0.23;           // Wrist is in intake position (picking up)
    final double WRIST_MID = 0.4;              // Wrist is out of the way
    final double WRIST_DROPOFF = 0.89;          // Wrist is in outtake position (dropping in basket)

    Servo ascentStick = null;
    final double ASCENT_MIN = 0.2;          // Stick is down
    final double ASCENT_MAX = 0.43;         // Stick is up

    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        RockinBot r = new RockinBot();
        r.initializeHardwareVariables();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous Ready", "You can press start");
        telemetry.addData("This code was last updated", "01/31/2025, 11:55 am"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        r.configureOtos();

        // First Sample ///////////////////////////////////////////////////////////////
        RobotLog.vv("Rockin' Robots", "Quick Test Code");
        r.driveToLoc(3, 5, 90, 3);
        r.setWrist(WRIST_MID);
        r.setVertical(VERTICAL_MAX);
        r.setViper(VIPER_MAX, VIPER_DEFAULT_SPEED);
        sleep(1000);
        r.setViper(VIPER_MIN);
        r.setVertical(VERTICAL_MIN);
        sleep(3000);
        RobotLog.vv("Rockin' Robots", "Test Done");
    }
}