"""
Main script that moves the motors and collects data to generate a calibration
value for the loadcell.

loadcell scale is in kg / (V/V)
The output of bridge sensor is a Volts measured / Volts input to remove the need
to reference the input voltage.
The scale can then be multiplied by the bridge sensor output to get a weight in
kgs.

Loadcell nomenclature:
REF: reference that is calibrated
DUT: device under test, to be calibrated

0 - Drive Forward M1
Drive motor 1 forward. Valid data range is 0 - 127. A value of 127 = full speed forward, 64 =
about half speed forward and 0 = full stop.
Send: [Address, 0, Value, CRC(2 bytes)]
Receive: [0xFF]


52 - Drive M1 With Signed Duty And Acceleration
Drive M1 with a signed duty and acceleration value. The sign indicates which direction the motor
will run. The acceleration values are not signed. This command is used to drive the motor by
PWM and using an acceleration value for ramping. Accel is the rate per second at which the duty
changes from the current duty to the specified duty.
Send: [Address, 52, Duty(2 bytes), Accel(2 Bytes), CRC(2 bytes)]
Receive: [0xFF]
The duty value is signed and the range is -32768 to +32767(eg. +-100% duty). The accel value
range is 0 to 655359(eg maximum acceleration rate is -100% to 100% in 100ms)


"""
import statistics
import time
from pathlib import Path
from datetime import datetime
import numpy as np
# import argparse

from tqdm import tqdm
import serial.tools.list_ports
from Phidget22.Devices import VoltageRatioInput
from Phidget22.BridgeGain import BridgeGain

from roboclaw_3 import Roboclaw
from parse_calibration_curve import parse_calibration_curve

def generate_calibration_curve(
        motor_com: str,
        motor_baud: int,
        motor_addr: int,
        loadcell_ref_scale: float,
        loadcell_ref_ch: int,
        loadcell_dut_ch: int
) -> tuple[list[float], list[float]]:
    """Return the readings of both loadcells.
    """
    ## motor controller setup
    print("setting up motor connection")
    motor = Roboclaw(motor_com, motor_baud)
    motor.Open()

    ## Bridge adc setup
    print("setting up bridge adc")
    ch_ref = VoltageRatioInput.VoltageRatioInput()
    ch_dut = VoltageRatioInput.VoltageRatioInput()

    print("REF channel", loadcell_ref_ch)
    print("DUT channel", loadcell_dut_ch)
    ch_ref.setChannel(loadcell_ref_ch)
    ch_dut.setChannel(loadcell_dut_ch)

    print("opening attachement to bridge")
    ch_ref.openWaitForAttachment(1000)
    ch_dut.openWaitForAttachment(1000)


    # Setting the VoltageRatioChangeTrigger to 0 will result in the channel
    # firing events every DataInterval
    ch_ref.setVoltageRatioChangeTrigger(0)
    ch_dut.setVoltageRatioChangeTrigger(0)

    # The phidgets bridge has a lot of noise if two loadcells are sampling at
    # different frequencies. Pick either 100ms or 50ms period for best results.
    # Anything lower is much worse.
    loadcell_sample_interval = 50  # ms
    ch_ref.setDataInterval(loadcell_sample_interval)
    ch_dut.setDataInterval(loadcell_sample_interval)

    # verify that the rate has been changed TODO progromatic
    print("Loadcell rates")
    print("REF:", ch_ref.getDataRate(), "Hz")
    print("DUT:", ch_dut.getDataRate(), "Hz")

    # TODO figure out best gain for calibration
    ch_ref.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
    ch_dut.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)

    ## Ask if motor needs to move back, if so move it back
    # TODO
    ## Tare both loadcells

    loadcell_values = {loadcell_ref_ch: [], loadcell_dut_ch: []}
    def save_bridge_value(vri, value):
        loadcell_values[vri.getChannel()].append(value)

    ch_ref.setOnVoltageRatioChangeHandler(save_bridge_value)
    ch_dut.setOnVoltageRatioChangeHandler(save_bridge_value)

    print("Getting values to tare loadcell")
    tare_num_vals = 15
    with tqdm(total=tare_num_vals, desc="Taring", unit="samples") as pbar:
        while (num_vals := len(loadcell_values[loadcell_ref_ch])) < tare_num_vals:
            pbar.update(num_vals - pbar.n)
            time.sleep(loadcell_sample_interval/1000)
        pbar.update(num_vals-pbar.n)

    ch_ref.setOnVoltageRatioChangeHandler(None)
    ch_dut.setOnVoltageRatioChangeHandler(None)

    tare_ref = statistics.fmean(loadcell_values[loadcell_ref_ch])
    tare_dut = statistics.fmean(loadcell_values[loadcell_dut_ch])

    print("tare ref:", tare_ref, "len of sample", len(loadcell_values[loadcell_ref_ch]))
    print("tare dut:", tare_dut, "len of sample", len(loadcell_values[loadcell_dut_ch]))

    def convert_ref(value: float) -> float:
        return (value - tare_ref) * loadcell_ref_scale


    print("tare * scale", tare_ref * loadcell_ref_scale, "kg")

    ## Assume positive is compression
    # TODO - should just take the absolute value instead
    ## Calibration curve
    # The linear motor can be back driven, so if it is pushing on the loadcell
    # and not being driven, it will slowly back off.
    # To generate the curve, push the loadcell to the test load, and then back
    # off slowly. The data from backing off should be used for 

    print("recording the values")

    # TODO - consider keeping the tare values for better post processing.
    # clear the value
    loadcell_values = {loadcell_ref_ch: [], loadcell_dut_ch: []}

    # Start recodring both loadcells
    ch_ref.setOnVoltageRatioChangeHandler(save_bridge_value)
    ch_dut.setOnVoltageRatioChangeHandler(save_bridge_value)
    
    # Wait for the phidgets to start recording
    while len(loadcell_values[loadcell_ref_ch]) == 0:
        time.sleep(loadcell_sample_interval/1000)

    # Push to test load
    # TODO - make the push smoother
    test_load = 150.0
    print("Pushing on loadcell until test load")
    motor.ForwardM1(motor_addr, 100)
    with tqdm(total=test_load, unit="kg") as pbar:
        while (cur_load := convert_ref(loadcell_values[loadcell_ref_ch][-1])) < test_load:
            pbar.n = cur_load
            pbar.refresh()
            time.sleep(loadcell_sample_interval/1000)
        pbar.n = test_load  # technically it could be larger, but tqdm doesn't like it
        pbar.refresh()

    # Peak test load
    motor.ForwardM1(motor_addr, 10)  # figure out a number that holds the load constantish
    # sample_count_at_peak = len(loadcell_values[loadcell_ref_ch])
    time.sleep(0)

    # Reverse slowly to generate the data for the curve
    motor.BackwardM1(motor_addr, 20)
    print("test load reached, slowly reversing")
    with tqdm(total=cur_load, unit="kg") as pbar:
        while (cur_load := convert_ref(loadcell_values[loadcell_ref_ch][-1])) > 0.1:
            pbar.n = cur_load
            pbar.refresh()
            time.sleep(loadcell_sample_interval/1000)
        pbar.n = cur_load
        pbar.refresh()
    
    print("moving back for one second")
    motor.BackwardM1(motor_addr, 127)
    time.sleep(1)
    motor.BackwardM1(motor_addr, 0)

    # print(loadcell_values)
    # print(sample_count_at_peak)
    return loadcell_values[loadcell_ref_ch], loadcell_values[loadcell_dut_ch]


if __name__ == "__main__":
    # A bunch of hard coded variables for now
    # The motor controller shows up as stm virtual com port.
    # pick the first match.
    MOTOR_COM = next(serial.tools.list_ports.grep("STMicroelectronics Virtual COM Port")).device
    MOTOR_BAUD = 115200  # This preconfigured for the controller
    MOTOR_ADDR = 0x80  # This is preconfigured
    # The motor controller library kinda sucks cause it needs the address passed
    # every time. TODO: just write a better one later.


    LOADCELL_REF_FSO = 3.0049 / 1000  # Full Scale output in V/V
    LOADCELL_REF_CAPACITY = 500 * 0.45359237  # in kg converted from lbs

    LOADCELL_REF_SCALE = LOADCELL_REF_CAPACITY / LOADCELL_REF_FSO  # kg / V/V

    LOADCELL_REF_CH = 3  # Wired in the 3rd channel

    LOADCELL_DUT_CH = 1  # TODO make this an input
    # TODO add logging to the save directory

    loadcell_dut_serial = input("Serial no of DUT: ")

    loadcell_values_ref, loadcell_values_dut = generate_calibration_curve(
        MOTOR_COM, 
        MOTOR_BAUD, 
        MOTOR_ADDR, 
        LOADCELL_REF_SCALE, 
        LOADCELL_REF_CH, 
        LOADCELL_DUT_CH
    )

    scale_dut, resid = parse_calibration_curve(
        loadcell_values_ref, 
        loadcell_values_dut, 
        LOADCELL_REF_SCALE
    )

    # TODO Save loadcell values somewhere 
    # save 5 values - the 3 inputs to the function (to reporduce) and the output
    date_str = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    dir_data = Path("../loadcell_calibration_data/data/")
    dir_cal = dir_data / loadcell_dut_serial / date_str
    print("Saving calibration curves to:", dir_cal)
    dir_cal.mkdir(parents= True, exist_ok=True)
    np.savetxt(dir_cal / "loadcell_values_ref.txt", loadcell_values_ref)
    np.savetxt(dir_cal / "loadcell_values_dut.txt", loadcell_values_dut)
    np.savetxt(dir_cal / "loadcell_ref_scale.txt", [LOADCELL_REF_SCALE])

    file_cal_logs = dir_data / "calibration_logs.csv"
    file_cal_logs.touch(exist_ok=True)
    print("Writing calivration data to log:", file_cal_logs)
    # Adhoc csv file, TODO make this nicer
    with open(file_cal_logs, mode="a") as f:
        line = ", ".join((date_str, loadcell_dut_serial, str(scale_dut), str(resid)))
        print(line, file=f)
        # f.write(line)
        # f.write("\n")


    # Note: the mavin loadcell should be 2 mV / V for 200kg, which works out to
    # a scale of 200 kg / 2mV/V = 100 kg / mv/V or 100'000 kg / V/V
