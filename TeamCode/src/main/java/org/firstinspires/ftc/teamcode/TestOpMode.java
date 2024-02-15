package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Custom
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Core.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.Robot;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class TestOpMode extends LinearOpMode
{
    Robot robot;
    public DcMotor frontLeftDriveMotor;
    public DcMotor frontRightDriveMotor;
    public DcMotor backLeftDriveMotor;
    public DcMotor backRightDriveMotor;

    // INTAKE
    // Spin motors (1150RPMs)
    public DcMotor leftIntakeSpinMotor;
    public DcMotor rightIntakeSpinMotor;
    // Position motors
    public Servo leftIntakeRaise;
    public Servo rightIntakeRaise;
    public ControllerInput controllerInput;

    public IMU imu; //define IMU (for GYRO).
    public double targetYaw;
    public double initalYaw;
    public double lastError = 0.0;
    // YAW position
    // This will be modified by the user with joysticks, and the robot will use a
    // PID controller to align itself with the target YAW.

    public double frontLeftDrive;
    public double frontRightDrive;
    public double backLeftDrive;
    public double backRightDrive;

    @Override
    public void runOpMode() throws InterruptedException
    {

        // Runs when "init" is pressed

        // Initialising motors
        // MOVEMENT
        // Drive motors
        frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "frontLeftDriveMotor");
        frontRightDriveMotor = hardwareMap.get(DcMotor.class, "frontRightDriveMotor");
        backLeftDriveMotor = hardwareMap.get(DcMotor.class, "backLeftDriveMotor");
        backRightDriveMotor = hardwareMap.get(DcMotor.class, "backRightDriveMotor");

        frontLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        // INTAKE
        // Spin motors (1150RPMs)
        //leftIntakeSpinMotor = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        //rightIntakeSpinMotor = hardwareMap.get(DcMotor.class, "rightIntakeMotor");
        // Raise servos
        //leftIntakeRaise = hardwareMap.get(Servo.class, "leftIntakeRaise");
        //rightIntakeRaise = hardwareMap.get(Servo.class, "rightIntakeRaise");

        //Reset and initialise the IMU.
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );

        imu = hardwareMap.get(IMU.class, "imu");


        // This resets it
        imu.initialize(parameters);

        if (isStopRequested()) return;
        waitForStart();
        //////////////////////////////////////////
        // Runs when the play button is pressed //
        //////////////////////////////////////////





        // Main loop
        while (opModeIsActive())
        {
            //controllerInput.mainloop(gamepad1);
            moveTele();
        }
    }

    //public void mainloop(Gamepad g)
    //{
      //  refresh(g);
    //}

    public double getYawDegrees() {
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.normalize(angles.firstAngle);
    }

    public class DesiredMovements {
        public double frontLeftDrive;
        public double frontRightDrive;
        public double backLeftDrive;
        public double backRightDrive;
    } public DesiredMovements desiredMovements;

    public double calculateRotationPower(double targetRotation, double threshold) {
        double currentRotation = getYawDegrees();
        double error = targetRotation - currentRotation;
        // Such that if the robot is at 15d and the targetRotation = 0d,
        // this will be equal to -15d. For now I'll use a > or < but once
        // it works, I will switch to PID.

        double correction;

        if (Math.abs(error) > threshold) {
            correction = ((error * 0.2 ) + (0.5 * (error - lastError))) * -0.1;
            if (initalYaw == targetYaw) {
                initalYaw = currentRotation;
            }
        }
        else { correction = 0.0; initalYaw = targetYaw; };

        double speed;

        if (correction < -1) { correction = -1;}
        if (correction > 1) { correction = 1; }

        lastError = error;

        if (Math.abs(error) > 160) { speed = 0.2;}
        else if (Math.abs(error) > 120) { speed = 0.3; }
        else if (Math.abs(error) > 100) { speed = 0.5; }
        else if (Math.abs(error) > 80) { speed = 0.8; }
        else if (Math.abs(error) > 45) { speed = 1.0; }

        else { speed = 0.0; }


        return (correction + (Math.abs(correction) / correction) * 0.04) * speed;
    }

    void calculateMovementTele() {
        double mx = 0.0;// movement_x;
        double my = 0.0;// movement_y;
        //double r = controllerInput.rotation; <- Leave commented out for now
        double r = calculateRotationPower(0.0, 3.0);

        // For now disregard R and focus on making YAW = 0

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(my) + Math.abs(mx) + Math.abs(r), 1);

        //If we are correcting yaw, slow down the motors so that it makes a difference
        // (If they are all 100% speed then decrease to 80% so that the motors can speed up
        // more to rotate). Learnt this last year.
        double movementMultiplier = 1.0 - Math.abs(r);

        frontLeftDrive = ((mx + my) * movementMultiplier + r) / denominator;
        frontRightDrive = ((mx - my) * movementMultiplier - r) / denominator;
        backLeftDrive = ((mx - my) * movementMultiplier + r) / denominator;
        backRightDrive = ((mx + my) * movementMultiplier - r) / denominator;
    }

    public void setMotorPowers()
    {
        frontLeftDriveMotor.setPower(frontLeftDrive);
        frontRightDriveMotor.setPower(frontRightDrive);
        backLeftDriveMotor.setPower(backLeftDrive);
        backRightDriveMotor.setPower(backRightDrive);
    }

    public void moveTele()
    {
        calculateMovementTele();
        setMotorPowers();
    }
    public class Settings {
        boolean movementStick = true;
        // Left = True | Right = False
        String rotationAxis = "rx";
        double strafeCorrection = 1.1;
    } Settings settings;

    public class ControllerInput {

        // Left stick
        double lx;
        double ly;
        // Right stick
        double rx;
        double ry;
        // Whatever stick movement is assigned to
        public double movement_x;
        public double movement_y;

        // Rotation
        public double rotation;

        public void refresh(Gamepad gamepad)
        {
            // Stick inputs
            lx =  gamepad.left_stick_x;
            ly = -gamepad.left_stick_y;
            rx =  gamepad.right_stick_x;
            ry = -gamepad.right_stick_y;

            // Movement input (facilitates stick selection within settings)
            movement_x = settings.movementStick ? lx : rx;
            movement_y = settings.movementStick ? ly : ry;

            movement_x *= settings.strafeCorrection;

            // Rotation
            if (settings.rotationAxis.equals("lx")) { rotation = lx; }
            if (settings.rotationAxis.equals("rx")) { rotation = rx; }
        }


    }
}
