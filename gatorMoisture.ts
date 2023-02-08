
const enum registers {
    walk_forward = 0x01,
    walk_backward = 0x02,
    walk_left = 0x03,
    walk_right = 0x04,
    turn_left = 0x05,
    turn_right = 0x06
}

const enum directions {
    forward = 0x01,
    backward = 0x02,
    left = 0x03,
    right = 0x04
}

const enum leg_joint_id {
    shoulder = 0xA0,
    upper_leg = 0xB0,
    lower_leg = 0xC0
}

class PCA9685 {
    protected static I2C_adress = 0x40;	// i2c address of PCA9685 servo controller
    protected static I2C_Starting_adress = 0x06; // first servo address for start byte low
    private static initialized: boolean = false;
    constructor() {
        if (!PCA9685.initialized) {
            let I2C_Buffer = pins.createBuffer(2);

            I2C_Buffer[0] = 0;		// Mode 1 register
            I2C_Buffer[1] = 0x10;	// Set sleep
            pins.i2cWriteBuffer(PCA9685.I2C_adress, I2C_Buffer, false);

            I2C_Buffer[0] = 0xFE;	// Prescale register
            I2C_Buffer[1] = 117;	// set to 50 Hz
            pins.i2cWriteBuffer(PCA9685.I2C_adress, I2C_Buffer, false);

            I2C_Buffer[0] = 0;		// Mode 1 register
            I2C_Buffer[1] = 0x81;	// Wake up
            pins.i2cWriteBuffer(PCA9685.I2C_adress, I2C_Buffer, false);

            PCA9685.initialized = true;
        }
    }
}

class servo extends PCA9685 {
    private ServoID: number | null;
    private MinimumPosition: number | null;
    private MaximumPosition: number | null;

    private CurrentPosition: number | null;

    init(id: number, min: number, max: number): void {
        this.ServoID = Math.max(Math.min(15, id), 0);
        this.MinimumPosition = Math.max(Math.min(180, min), 0);
        this.MaximumPosition = Math.max(Math.min(180, max), 0);

        let I2C_Buffer = pins.createBuffer(2);

        I2C_Buffer[0] = PCA9685.I2C_Starting_adress + this.ServoID * 4 + 0;    // Servo register
        I2C_Buffer[1] = 0x00;   // low-byte start - always 0
        pins.i2cWriteBuffer(PCA9685.I2C_adress, I2C_Buffer, false);

        I2C_Buffer[0] = PCA9685.I2C_Starting_adress + this.ServoID * 4 + 1;	// Servo register
        I2C_Buffer[1] = 0x00;   // high-byte start - always 0
        pins.i2cWriteBuffer(PCA9685.I2C_adress, I2C_Buffer, false);

        this.CurrentPosition = 90;

        this.moveServo(this.ServoID, ((this.MaximumPosition - this.MinimumPosition) / 2));
    }
    moveServo(servo: number, angle: number): void {
        angle = Math.max(Math.min(this.MaximumPosition, angle), this.MinimumPosition);

        let i2cData = pins.createBuffer(2);
        let stop = 102 + (angle) * ((512 - 102) / 180);

        i2cData[0] = PCA9685.I2C_Starting_adress + servo * 4 + 2;	// Servo register
        i2cData[1] = (stop & 0xff);		                    // low byte stop
        pins.i2cWriteBuffer(PCA9685.I2C_adress, i2cData, false);

        i2cData[0] = PCA9685.I2C_Starting_adress + servo * 4 + 3;	// Servo register
        i2cData[1] = (stop >> 8);		                    // high byte stop
        pins.i2cWriteBuffer(PCA9685.I2C_adress, i2cData, false);

        this.CurrentPosition = angle;
    }
    setMinMax(min: number, max: number): void {
        this.MinimumPosition = Math.max(Math.min(180, min), 0);
        this.MaximumPosition = Math.max(Math.min(180, max), 0);
    }
    get_id(): number { return this.ServoID; }
    get_pos(): number { return this.CurrentPosition; }
    get_min_pos(): number { return this.MinimumPosition; }
    get_max_pos(): number { return this.MaximumPosition; }
}

class leg {
    private _id: number | null;
    private _currentX: number | null;
    private _currentY: number | null;
    private _currentZ: number | null;
    private _currentPitch: number | null;
    private _currentRoll: number | null;
    private _currentYaw: number | null;

    private _spot_shoulder = new servo;
    private _spot_upper_leg = new servo;
    private _spot_lower_leg = new servo;

    constructor(id: number) {
        this._id = id;
        this._currentX = 0.0;
        this._currentY = 0.0;
        this._currentZ = 0.0;
        this._currentPitch = 0.0;
        this._currentRoll = 0.0;
        this._currentYaw = 0.0;
    }
    init_servo(joint_id: number, servo_id: number, servo_pos: number, servo_min: number, servo_max: number): void {
        if (joint_id == leg_joint_id.shoulder) { this._spot_shoulder.init(servo_id, servo_min, servo_max) }
        if (joint_id == leg_joint_id.upper_leg) { this._spot_upper_leg.init(servo_id, servo_min, servo_max) }
        if (joint_id == leg_joint_id.lower_leg) { this._spot_lower_leg.init(servo_id, servo_min, servo_max) }
    }
    setNewPosition(X: number, Y: number, Z: number, Pitch: number, Roll: number, Yaw: number): void {
        this._currentX = X;
        this._currentY = Y;
        this._currentZ = Z;
        this._currentPitch = Pitch;
        this._currentRoll = Roll;
        this._currentYaw = Yaw;
    }
    outputFromInverseKinamatics(ShoulderX: number, ShoulderY: number, Knee: number): void {
        this._spot_shoulder.moveServo(this._spot_shoulder.get_id(), ShoulderX);
        this._spot_upper_leg.moveServo(this._spot_upper_leg.get_id(), ShoulderY);
        this._spot_lower_leg.moveServo(this._spot_lower_leg.get_id(), Knee);
    }
    getServoCurrentAngles(): Array<number> {
        return [this._spot_shoulder.get_pos(), this._spot_upper_leg.get_pos(), this._spot_lower_leg.get_pos()]
    }
}
// namespace gatorMoisture {

// }
const front_left = new leg(0x10);
const front_right = new leg(0x20);
const back_left = new leg(0x30);
const back_right = new leg(0x40);

enum gatorMoistureType {
    moisture = 1,
    adcVal = 2,
}

//% weight=100 color=#0fbc11 icon="\uf04b"
//% groups='["Startup", "Walking", "Turning"]'
namespace gatorMoisture {

    /**
    * Reads the number
    */
    //% weight=30 blockId="gatorMoisture_moisture" block="Get moisture on pin %pin | in %gatorMoistureType"
    export function moisture(pin: AnalogPin, type: gatorMoistureType): number {
        let ADCVal = pins.analogReadPin(pin)
        switch (type) {
            case gatorMoistureType.moisture: 
            {
                    getMoisture(ADCVal)
                return 123456789
            }
            case gatorMoistureType.adcVal: return ADCVal
            default: return -11111111
        }
    }

    //%
    function DataProcessorKinamatics(speed: number, front_left_leg: number[], front_right_leg: number[], back_left_leg: number[], back_right_leg: number[]): void {
        const filter_steps = 50;
        let wait_value_us = 1000;//((speed + 1) - 10) * 10;

        let n = 0;
        let leg_sel: number[] = [];

        if (front_left_leg != null) { leg_sel[n] = 1; n++; }
        if (front_right_leg != null) { leg_sel[n] = 2; n++; }
        if (back_left_leg != null) { leg_sel[n] = 3; n++; }
        if (back_right_leg != null) { leg_sel[n] = 4; n++; }

        let kinamaticOutput_front_left: number[][] = [];
        let kinamaticOutput_front_right: number[][] = [];
        let kinamaticOutput_back_left: number[][] = [];
        let kinamaticOutput_back_right: number[][] = [];
        
        for (let l = 0; l < n; l++) {
            if (leg_sel[l] == 1) {    //  front left
                front_left.setNewPosition(front_left_leg[0], front_left_leg[1], front_left_leg[2], front_left_leg[3], front_left_leg[4], front_left_leg[5]);
                let array: number[] = [90, 90, 90];
                InverseKinamaticModel();
                kinamaticOutput_front_left =
                    [
                        front_left.getServoCurrentAngles(),
                        array,
                        [0, 0, 0]
                    ];
            }
            
            if (leg_sel[l] == 2) {    //  front right
                front_right.setNewPosition(front_right_leg[0], front_right_leg[1], front_right_leg[2], front_right_leg[3], front_right_leg[4], front_right_leg[5]);
                let array: number[] = [90, 90, 90];
                InverseKinamaticModel();
                kinamaticOutput_front_right =
                    [
                        front_right.getServoCurrentAngles(),
                        array,
                        [0, 0, 0]
                    ];
            }
            if (leg_sel[l] == 3) {    //  back left
                back_left.setNewPosition(back_left_leg[0], back_left_leg[1], back_left_leg[2], back_left_leg[3], back_left_leg[4], back_left_leg[5]);
                let array: number[] = [90, 90, 90];
                InverseKinamaticModel();
                kinamaticOutput_back_left =
                    [
                        back_left.getServoCurrentAngles(),
                        array,
                        [0, 0, 0]
                    ];
            }
            if (leg_sel[l] == 4) {    //  back right
                back_right.setNewPosition(back_right_leg[0], back_right_leg[1], back_right_leg[2], back_right_leg[3], back_right_leg[4], back_right_leg[5]);
                let array: number[] = [90, 90, 90];
                InverseKinamaticModel();
                kinamaticOutput_back_right =
                    [
                        back_right.getServoCurrentAngles(),
                        array,
                        [0, 0, 0]
                    ];
            }
        }

        for (let i = 0; i < filter_steps; i++) {
            for (let j = 0; j < 3; j++) {
                for (let l = 0; l < n; l++) {
                    if (leg_sel[l] == 1) {
                        kinamaticOutput_front_left[2][j] =
                            i * ((kinamaticOutput_front_left[1][j] - kinamaticOutput_front_left[0][j]) /
                                (filter_steps - 1)) + kinamaticOutput_front_left[0][j];
                    } else if (leg_sel[l] == 2) {
                        kinamaticOutput_front_right[2][j] =
                            i * ((kinamaticOutput_front_right[1][j] - kinamaticOutput_front_right[0][j]) /
                                (filter_steps - 1)) + kinamaticOutput_front_right[0][j];
                    } else if (leg_sel[l] == 3) {
                        kinamaticOutput_back_left[2][j] =
                            i * ((kinamaticOutput_back_left[1][j] - kinamaticOutput_back_left[0][j]) /
                                (filter_steps - 1)) + kinamaticOutput_back_left[0][j];
                    } else if (leg_sel[l] == 4) {
                        kinamaticOutput_back_right[2][j] =
                            i * ((kinamaticOutput_back_right[1][j] - kinamaticOutput_back_right[0][j]) /
                                (filter_steps - 1)) + kinamaticOutput_back_right[0][j];
                    }
                }
            }
            for (let l = 0; l < n; l++) {
                if (leg_sel[l] == 1) {
                    front_left.outputFromInverseKinamatics(
                        kinamaticOutput_front_left[2][0],
                        kinamaticOutput_front_left[2][1],
                        kinamaticOutput_front_left[2][2]);
                } else if (leg_sel[l] == 2) {
                    front_right.outputFromInverseKinamatics(
                        kinamaticOutput_front_right[2][0],
                        kinamaticOutput_front_right[2][1],
                        kinamaticOutput_front_right[2][2]);
                } else if (leg_sel[l] == 3) {
                    back_left.outputFromInverseKinamatics(
                        kinamaticOutput_back_left[2][0],
                        kinamaticOutput_back_left[2][1],
                        kinamaticOutput_back_left[2][2]);
                } else if (leg_sel[l] == 4) {
                    back_right.outputFromInverseKinamatics(
                        kinamaticOutput_back_right[2][0],
                        kinamaticOutput_back_right[2][1],
                        kinamaticOutput_back_right[2][2]);
                }
            }
            if (wait_value_us) control.waitMicros(wait_value_us);
        }
    }


    /**
     * Place this in Start block
     */
    //% group="Startup"
    //% block="Start Spot"
    //% weight=10
    export function Init_Spot(): void {
        front_left.init_servo(leg_joint_id.shoulder, 0x00, 90, 0, 180);
        front_left.init_servo(leg_joint_id.upper_leg, 0x01, 90, 0, 180);
        front_left.init_servo(leg_joint_id.lower_leg, 0x02, 90, 0, 180);

        front_right.init_servo(leg_joint_id.shoulder, 0x03, 90, 0, 180);
        front_right.init_servo(leg_joint_id.upper_leg, 0x04, 90, 0, 180);
        front_right.init_servo(leg_joint_id.lower_leg, 0x05, 90, 0, 180);

        back_left.init_servo(leg_joint_id.shoulder, 0x06, 90, 0, 180);
        back_left.init_servo(leg_joint_id.upper_leg, 0x07, 90, 0, 180);
        back_left.init_servo(leg_joint_id.lower_leg, 0x08, 90, 0, 180);

        back_right.init_servo(leg_joint_id.shoulder, 0x09, 90, 0, 180);
        back_right.init_servo(leg_joint_id.upper_leg, 0x0A, 90, 0, 180);
        back_right.init_servo(leg_joint_id.lower_leg, 0x0B, 90, 0, 180);
    }
    /**
     * Do a step forward
     */
    //% group="Walking"
    //% block="Step forward"
    //% weight=70
    export function Step_Forward(): void {
        Walk_1(directions.forward, 1, 5);
        // DataProcessorKinamatics(
        //     [], // front left
        //     [], // front right
        //     [], // back left
        //     []  // back right
        // );
        // DataProcessorKinamatics([], [], [], []);
    }

    /**
     * Do a step backward
     */
    //% group="Walking"
    //% block="Step backward"
    //% weight=60
    export function Step_Backward(): void {
        Walk_1(directions.backward, 1, 5);

    }

    /**
    * Do a step to the left
    */
    //% group="Walking"
    //% block="Step to the left"
    //% weight=50
    export function Step_Left(): void {
        Walk_1(directions.left, 1, 5);

    }

    /**
    * Do a step to the right
    */
    //% group="Walking"
    //% block="Step to the right"
    //% weight=40
    export function Step_Right(): void {
        Walk_1(directions.right, 1, 5);

    }

    /**
    * Turn around
    */
    //% group="Turning"
    //% block="Turn around"
    //% weight=30
    export function Turn_Arount(): void {

    }

    /**
    * Turn a bit to the left
    */
    //% group="Turning"
    //% block="Turn left"
    //% weight=20
    export function Turn_Left(): void {

    }

    /**
    * Turn a bit to the right
    */
    //% group="Turning"
    //% block="Turn right"
    //% weight=10
    export function Turn_Right(): void {

    }

    /**
     * Walk for a defined distance with a defined speed and direction
     * @param direction direction of walking, eg: forward
     * @param amount_of_steps Amound of steps in the certain direction, eg: 5
     * @param speed The speed spot going to walk at
     */
    //% speed.min=1 speed.max=10 speed.defl=5
    //% amount_of_steps.min=1 amount_of_steps.max=10 amount_of_steps.defl=5
    //% group="Walking"
    //% weight=50
    //% block="Walk %direction for %amount_of_steps steps with a speed of %speed"
    //% advanced=true
    export function Walk_1(direction: directions, amount_of_steps: number, speed: number): void {
        Walk_3(direction, amount_of_steps, speed, false);
        // if (direction == directions.forward)  // Forward
        // {
        //     // Do someting
        //     return
        // }

        // if (direction == directions.backward)  // Backward
        // {
        //     // Do someting
        //     return
        // }

        // if (direction == directions.left)  // Left
        // {
        //     // Do someting
        //     return
        // }
        // if (direction == directions.right)  // Right
        // {
        //     // Do someting
        //     return
        // }
    }

    /**
     * Walk for a undefined distance until condition is met with a defined speed and direction
     * @param direction direction of walking, eg: forward
     * @param amount_of_steps Amound of steps in the certain direction, eg: 5
     * @param speed The speed spot going to walk at
     */
    //% speed.min=1 speed.max=10 speed.defl=5
    //% amount_of_steps.min=1 amount_of_steps.max=10 amount_of_steps.defl=5
    //% group="Walking"
    //% weight=40
    //% block="Walk %direction with a speed of %speed until %condition is reached"
    //% advanced=true
    export function Walk_2(direction: directions, speed: number, condition: boolean): void {
        Walk_3(direction, 0, speed, condition);
        // if (direction == directions.forward)  // Forward
        // {
        //     // Walk_3(direction, 0, speed, condition);
        //     return
        // }
        // if (direction == directions.backward)  // Backward
        // {
        //     // Do someting
        //     return
        // }
        // if (direction == directions.left)  // Left
        // {
        //     // Do someting
        //     return
        // }
        // if (direction == directions.right)  // Right
        // {
        //     // Do someting
        //     return
        // }
    }

    /**
     * Walk for a defined distance or do something when condition is met with defined speed and direction
     * @param direction direction of walking, eg: forward
     * @param speed The speed spot going to walk at, eg: 5
     * @param amount_of_steps Amound of steps in the certain direction, eg: 5
     */
    //% speed.min=1 speed.max=10 speed.defl=5
    //% amount_of_steps.min=1 amount_of_steps.max=10 amount_of_steps.defl=5
    //% group="Walking"
    //% weight=30
    //% block="Walk %direction with a speed of %speed for %amount_of_steps steps or until %condition is reached"
    //% advanced=true
    export function Walk_3(direction: directions, speed: number, amount_of_steps: number, condition: boolean): void {
        if (!(amount_of_steps == 0)) amount_of_steps = Math.max(Math.min(10, amount_of_steps), 1);

        speed = Math.max(Math.min(10, speed), 1);

        if (direction == directions.forward)  // Forward
        {
            /*---------------------------------Start sequence step forward begin-----------------------------------*/
            DataProcessorKinamatics(speed, [0, 0, -30, 0, 0, 0], null, null, [0, 0, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 30, -30, 0, 0, 0], null, null, [0, 30, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 30, 0, 0, 0, 0], null, null, [0, 30, 0, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 0, 0, 0, 0, 0], [0, -30, 0, 0, 0, 0], [0, -30, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]);
            /*----------------------------------Start sequence step forward end------------------------------------*/

            /*----------------------------------Full step forward sequence begin-----------------------------------*/
            let i = 0;
            do {
                DataProcessorKinamatics(speed, null, [0, -30, -30, 0, 0, 0], [0, -30, -30, 0, 0, 0], null);
                DataProcessorKinamatics(speed, null, [0, 30, -30, 0, 0, 0], [0, 30, -30, 0, 0, 0], null);
                DataProcessorKinamatics(speed, null, [0, 30, 0, 0, 0, 0], [0, 30, 0, 0, 0, 0], null);
                DataProcessorKinamatics(speed, [0, -30, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, -30, 0, 0, 0, 0]);

                DataProcessorKinamatics(speed, [0, -30, -30, 0, 0, 0], null, null, [0, -30, -30, 0, 0, 0]);
                DataProcessorKinamatics(speed, [0, 30, -30, 0, 0, 0], null, null, [0, 30, -30, 0, 0, 0]);
                DataProcessorKinamatics(speed, [0, 30, 0, 0, 0, 0], null, null, [0, 30, 0, 0, 0, 0]);
                DataProcessorKinamatics(speed, [0, -30, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, -30, 0, 0, 0, 0]);

                if (!(amount_of_steps == 0) && (i == amount_of_steps)) break;
                if (!(amount_of_steps == 0)) i++;
            } while (!condition)
            /*-----------------------------------Full step forward sequence end------------------------------------*/

            /*----------------------------------End sequence step forward begin------------------------------------*/
            DataProcessorKinamatics(speed, [0, -30, -30, 0, 0, 0], null, null, [0, -30, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 0, -30, 0, 0, 0], null, null, [0, 0, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]);
            /*-----------------------------------End sequence step forward end-------------------------------------*/
            return
        }

        if (direction == directions.backward)  // Backward
        {
            /*---------------------------------Start sequence step backward begin----------------------------------*/
            DataProcessorKinamatics(speed, [0, 0, -30, 0, 0, 0], null, null, [0, 0, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, -30, -30, 0, 0, 0], null, null, [0, -30, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, -30, 0, 0, 0, 0], null, null, [0, -30, 0, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 0, 0, 0, 0, 0], [0, 30, 0, 0, 0, 0], [0, 30, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]);
            /*----------------------------------Start sequence step backward end-----------------------------------*/

            /*---------------------------------Full step backward sequence begin-----------------------------------*/
            let i = 0;
            do {
                DataProcessorKinamatics(speed, null, [0, 30, -30, 0, 0, 0], [0, 30, -30, 0, 0, 0], null);
                DataProcessorKinamatics(speed, null, [0, -30, -30, 0, 0, 0], [0, -30, -30, 0, 0, 0], null);
                DataProcessorKinamatics(speed, null, [0, -30, 0, 0, 0, 0], [0, -30, 0, 0, 0, 0], []);
                DataProcessorKinamatics(speed, [0, 30, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 30, 0, 0, 0, 0]);

                DataProcessorKinamatics(speed, [0, 30, -30, 0, 0, 0], null, null, [0, 30, -30, 0, 0, 0]);
                DataProcessorKinamatics(speed, [0, -30, -30, 0, 0, 0], null, null, [0, -30, -30, 0, 0, 0]);
                DataProcessorKinamatics(speed, [0, -30, 0, 0, 0, 0], null, null, [0, -30, 0, 0, 0, 0]);
                DataProcessorKinamatics(speed, [0, 30, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 30, 0, 0, 0, 0]);

                if (!(amount_of_steps == 0) && (i == amount_of_steps)) break;
                i++;
            } while (!condition)
            /*----------------------------------Full step backward sequence end------------------------------------*/

            /*----------------------------------End sequence step backward begin-----------------------------------*/
            DataProcessorKinamatics(speed, [0, 30, -30, 0, 0, 0], null, null, [0, 30, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 0, -30, 0, 0, 0], null, null, [0, 0, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]);
            /*-----------------------------------End sequence step backward end------------------------------------*/
            return
        }

        if (direction == directions.left)  // Left
        {
            /*---------------------------------Start sequence step forward begin-----------------------------------*/
            DataProcessorKinamatics(speed, [0, 0, -30, 0, 0, 0], null, null, [0, 0, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 30, -30, 0, 0, 0], null, null, [0, 30, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 30, 0, 0, 0, 0], null, null, [0, 30, 0, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 0, 0, 0, 0, 0], [0, -30, 0, 0, 0, 0], [0, -30, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]);
            /*----------------------------------Start sequence step forward end------------------------------------*/

            /*----------------------------------Full step forward sequence begin-----------------------------------*/
            let i = 0;
            do {
                DataProcessorKinamatics(speed, null, [0, -30, -30, 0, 0, 0], [0, -30, -30, 0, 0, 0], null);
                DataProcessorKinamatics(speed, null, [0, 30, -30, 0, 0, 0], [0, 30, -30, 0, 0, 0], null);
                DataProcessorKinamatics(speed, null, [0, 30, 0, 0, 0, 0], [0, 30, 0, 0, 0, 0], null);
                DataProcessorKinamatics(speed, [0, -30, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, -30, 0, 0, 0, 0]);

                DataProcessorKinamatics(speed, [0, -30, -30, 0, 0, 0], null, null, [0, -30, -30, 0, 0, 0]);
                DataProcessorKinamatics(speed, [0, 30, -30, 0, 0, 0], null, null, [0, 30, -30, 0, 0, 0]);
                DataProcessorKinamatics(speed, [0, 30, 0, 0, 0, 0], null, null, [0, 30, 0, 0, 0, 0]);
                DataProcessorKinamatics(speed, [0, -30, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, -30, 0, 0, 0, 0]);

                if (!(amount_of_steps == 0) && (i == amount_of_steps)) break;
                if (!(amount_of_steps == 0)) i++;
            } while (!condition)
            /*-----------------------------------Full step forward sequence end------------------------------------*/

            /*----------------------------------End sequence step forward begin------------------------------------*/
            DataProcessorKinamatics(speed, [0, -30, -30, 0, 0, 0], null, null, [0, -30, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 0, -30, 0, 0, 0], null, null, [0, 0, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]);
            /*-----------------------------------End sequence step forward end-------------------------------------*/
            return
        }


        if (direction == directions.right)  // Right
        {
            DataProcessorKinamatics(speed, [0, 0, -30, 0, 0, 0], null, null, [0, 0, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, -30, -30, 0, 0, 0], null, null, [0, -30, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, -30, 0, 0, 0, 0], null, null, [0, -30, 0, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 0, 0, 0, 0, 0], [0, 30, 0, 0, 0, 0], [0, 30, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]);
            /*----------------------------------Start sequence step backward end-----------------------------------*/

            /*---------------------------------Full step backward sequence begin-----------------------------------*/
            let i = 0;
            do {
                DataProcessorKinamatics(speed, null, [0, 30, -30, 0, 0, 0], [0, 30, -30, 0, 0, 0], null);
                DataProcessorKinamatics(speed, null, [0, -30, -30, 0, 0, 0], [0, -30, -30, 0, 0, 0], null);
                DataProcessorKinamatics(speed, null, [0, -30, 0, 0, 0, 0], [0, -30, 0, 0, 0, 0], []);
                DataProcessorKinamatics(speed, [0, 30, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 30, 0, 0, 0, 0]);

                DataProcessorKinamatics(speed, [0, 30, -30, 0, 0, 0], null, null, [0, 30, -30, 0, 0, 0]);
                DataProcessorKinamatics(speed, [0, -30, -30, 0, 0, 0], null, null, [0, -30, -30, 0, 0, 0]);
                DataProcessorKinamatics(speed, [0, -30, 0, 0, 0, 0], null, null, [0, -30, 0, 0, 0, 0]);
                DataProcessorKinamatics(speed, [0, 30, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 30, 0, 0, 0, 0]);

                if (!(amount_of_steps == 0) && (i == amount_of_steps)) break;
                i++;
            } while (!condition)
            /*----------------------------------Full step backward sequence end------------------------------------*/

            /*----------------------------------End sequence step backward begin-----------------------------------*/
            DataProcessorKinamatics(speed, [0, 30, -30, 0, 0, 0], null, null, [0, 30, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 0, -30, 0, 0, 0], null, null, [0, 0, -30, 0, 0, 0]);
            DataProcessorKinamatics(speed, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]);
            /*-----------------------------------End sequence step backward end------------------------------------*/

            return
        }
    }

    /**
     * Turn for a defined amount of time with a defined speed and direction
     */
    //% group="Turning"
    //% weight=20
    //% block
    //% advanced=true
    export function Turn(): void {

    }
    /**
     * Function used for simulator, actual implementation is in gatormoisture.cpp
    */
    //% shim=gatorMoisture::getMoisture
    function getMoisture(ADCVal: number) {
        // Fake function for simulator
        return 0
    }
    //% shim=gatorMoisture::InverseKinamaticModel
    function InverseKinamaticModel()
    {
        // Fake function for simulator
    }
    /**
    * Function used for simulator, actual implementation is in gatormoisture.cpp
    */
    ////% shim=gatorMoisture::InverseKinamaticModel
    //function InverseKinamaticModel(output_array: number[], x: number, y: number, z: number, pitch: number, roll: number, yaw: number, leg_ID: number){
        // Fake function for simulator
    //}
}
