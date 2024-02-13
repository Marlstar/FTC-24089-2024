package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class BaseOpMode extends LinearOpMode
{
    class ControllerInput extends Gamepad {
        class Settings {
            boolean movementStick = true;
            // Left = True | Right = False
            String rotationAxis = "rx";
            double strafeCorrection = 1.1;
        } Settings settings;

        // Left stick
        double lx;
        double ly;
        // Right stick
        double rx;
        double ry;
        // Whatever stick movement is assigned to
        double movement_x;
        double movement_y;

        // Rotation
        double rotation;

        void refresh()
        {
            // Stick inputs
            lx =  gamepad1.left_stick_x;
            ly = -gamepad1.left_stick_y;
            rx =  gamepad1.right_stick_x;
            ry = -gamepad1.right_stick_y;

            // Movement input (facilitates stick selection within settings)
            movement_x = settings.movementStick ? lx : rx;
            movement_y = settings.movementStick ? ly : ry;

            movement_x *= settings.strafeCorrection;

            // Rotation
            if (settings.rotationAxis.equals("lx")) { rotation = lx; }
            if (settings.rotationAxis.equals("rx")) { rotation = rx; }
        }
    }

    class Robot {
        ControllerInput controllerInput;

        public class Drive {
            public class Motors {
                // MOVEMENT
                // Drive motors
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

                public void init() {
                    // Initialising motors
                    // MOVEMENT
                    // Drive motors
                    frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "frontLeftDriveMotor");
                    frontRightDriveMotor = hardwareMap.get(DcMotor.class, "frontRightDriveMotor");
                    backLeftDriveMotor = hardwareMap.get(DcMotor.class, "backLeftDriveMotor");
                    backRightDriveMotor = hardwareMap.get(DcMotor.class, "backRightDriveMotor");
                    // INTAKE
                    // Spin motors (1150RPMs)
                    leftIntakeSpinMotor = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
                    rightIntakeSpinMotor = hardwareMap.get(DcMotor.class, "rightIntakeMotor");
                    // Raise servos
                    leftIntakeRaise = hardwareMap.get(Servo.class, "leftIntakeRaise");
                    rightIntakeRaise = hardwareMap.get(Servo.class, "rightIntakeRaise");
                }
            }

            Motors motors;

            public class Movement {
                public class DesiredMovements {
                    double frontLeftDrive;
                    double frontRightDrive;
                    double backLeftDrive;
                    double backRightDrive;
                } DesiredMovements desiredMovements;

                void calculateMovementTele() {
                    double mx = controllerInput.movement_x;
                    double my = controllerInput.movement_y;
                    double r = controllerInput.rotation;

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio, but only when
                    // at least one is out of the range [-1, 1]
                    double denominator = Math.max(Math.abs(my) + Math.abs(mx) + Math.abs(r), 1);

                    desiredMovements.frontLeftDrive = (mx + my + r) / denominator;
                    desiredMovements.frontRightDrive = (mx - my - r) / denominator;
                    desiredMovements.backLeftDrive = (mx - my + r) / denominator;
                    desiredMovements.backRightDrive = (mx + my - r) / denominator;
                }

                void setMotorPowers()
                {
                    motors.frontLeftDriveMotor.setPower(desiredMovements.frontLeftDrive);
                    motors.frontRightDriveMotor.setPower(desiredMovements.frontRightDrive);
                    motors.backLeftDriveMotor.setPower(desiredMovements.backLeftDrive);
                    motors.backRightDriveMotor.setPower(desiredMovements.backRightDrive);
                }

                void moveTele()
                {
                    calculateMovementTele();
                    setMotorPowers();
                }
            } Movement movement;
        } Drive drive;
    } Robot robot;


    @Override
    public void runOpMode() throws InterruptedException
    {
        // Runs when "init" is pressed
        robot.drive.motors.init();

        if (isStopRequested()) return;
        waitForStart();
        // Runs when the play button is pressed


        // Main loop
        while (opModeIsActive())
        {
            robot.controllerInput.refresh();

            robot.drive.movement.moveTele();
        }
    }
}
