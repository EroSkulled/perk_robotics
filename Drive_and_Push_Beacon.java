
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

// set name to show on phone
@Autonomous(name="TEST RUN 1", group="Autonomous")
public class Drive_and_Push_Beacon extends LinearOpMode {
    final private static int ENCODER_CPR = 2;  // Encoder Counters per Revolution
    final private static double GEAR_RATIO = 1.0;   // Gear Ratio - 1:1 - Direct Drive
    final private static double WHEEL_CIRCUMFERENCE = 1.26; // in meters
    final private static double STRAFE_SLIPPAGE_FACTOR = 1.00;


    DcMotor motor[];


    final private static int MOTOR_COUNT = 2;

    final private static String[] MOTOR_NAMES = {
            "left_drive", "right_drive"
    };
    final private static DcMotorSimple.Direction MOTOR_DIRECTIONS[] = {
        DcMotor.Direction.FORWARD, // mFL
        DcMotor.Direction.REVERSE, // mFR
    };

    final private static int DRIVE_FORWARD  = 0;
    final private static int DRIVE_BACKWARD = 1;
    final private static int TURN_LEFT      = 2;
    final private static int TURN_RIGHT     = 3;
    final private static int STRAFE_LEFT    = 4;
    final private static int STRAFE_RIGHT   = 5;

    final private static double DRIVE_DIRECTIONS[][] = {
              //L     R
            {+1.00, +1.00}, // DRIVE_FORWARD
            {-1.00, -1.00}, // DRIVE_BACKWARD
            {-1.00, +1.00}, // TURN_LEFT
            {-1.00, +1.00}, // TURN_RIGHT
            {-0.00, +1.00}, // STRAFE_LEFT
            {+1.00, -0.00}, // STRAFE_RIGHT
    };

    private void stop_all_motors() {
        for(int i=0; i < MOTOR_COUNT; i++) {
            motor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    private int distance_to_count(double distance, double slippage) {
        return((int)(slippage * ENCODER_CPR * (distance / WHEEL_CIRCUMFERENCE) * GEAR_RATIO));
    }

    private void wait_for_one_stop() {
        while(true) {
            for(int i=0; i < MOTOR_COUNT; i++) {
                if(motor[i].isBusy() != true)
                    return;
            }
        }
    }

    private void wait_for_all_stop() {
        while(true) {
            for(int i=0; i < MOTOR_COUNT; i++) {
                if(motor[i].isBusy() == true)
                    continue;
                return;
            }
        }
    }

    private void set_motor_power(int motor_index, int direction, double power) {
        double motor_power = power;

        motor[motor_index].setPower(motor_power);
        telemetry.addData(MOTOR_NAMES[motor_index] + "_power", motor_power);
    }

    private void drive_to_position(int direction, int count, double speed) {
        try {
            for(int i=0; i < MOTOR_COUNT; i++) {
                motor[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor[i].setTargetPosition((int)((double)count * DRIVE_DIRECTIONS[direction][i]));
                set_motor_power(i, direction, 0.25 * speed * DRIVE_DIRECTIONS[direction][i]);
            }
            telemetry.update();

            Thread.sleep(50);
            for(int i=0; i < MOTOR_COUNT; i++) {
                set_motor_power(i, direction, 0.50 * speed * DRIVE_DIRECTIONS[direction][i]);
            }
            telemetry.update();

            Thread.sleep(10);
            for(int i=0; i < MOTOR_COUNT; i++) {
                set_motor_power(i, direction, 1.00 * speed * DRIVE_DIRECTIONS[direction][i]);
            }
            telemetry.update();

        } catch(InterruptedException e) {
        };
    }

    private void drive_distance(int direction, double distance, double speed) {
        double slippage = 1.0;
        if(direction == STRAFE_LEFT || direction == STRAFE_RIGHT)
            slippage = STRAFE_SLIPPAGE_FACTOR;
        drive_to_position(direction, distance_to_count(distance, slippage), speed);
        wait_for_one_stop();
        stop_all_motors();
    }

    public void robotInit() {
        motor = new DcMotor[MOTOR_COUNT];

        for(int i=0; i < MOTOR_COUNT; i++) {
            motor[i] = hardwareMap.dcMotor.get(MOTOR_NAMES[i]);
            motor[i].setDirection(MOTOR_DIRECTIONS[i]);

        }
        stop_all_motors();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robotInit();
        waitForStart();
        if (!isStarted())
            return;

        // Roughly align to the beacon before checking the color.
        drive_distance(DRIVE_FORWARD, 10.0, 0.6);
        drive_distance(DRIVE_BACKWARD, 10.0, 0.4);

        hardwareMap.dcMotor.wait(1000);
        stop_all_motors();


        hardwareMap.dcMotor.setpo

    }
}
