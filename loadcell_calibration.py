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

"""
import statistics
import time
# import argparse

from Phidget22.Devices import VoltageRatioInput
from Phidget22.BridgeGain import BridgeGain

from roboclaw_3 import Roboclaw

def generate_calibration_curve(
        motor_com: str,
        motor_baud: int,
        motor_addr: int,
        loadcell_ref_scale: float,
        loadcell_ref_ch: int,
        loacell_dut_ch: int
):
    """Return the readings of both loadcells.
    """
    ## motor controller setup
    print("setting up motor connection")
    motor = Roboclaw(motor_com, motor_baud)

    ## Bridge adc setup
    print("setting up bridge adc")
    ch_ref = VoltageRatioInput.VoltageRatioInput()
    ch_dut = VoltageRatioInput.VoltageRatioInput()

    print("REF channel", loadcell_ref_ch)
    print("DUT channel", loacell_dut_ch)
    ch_ref.setChannel(loadcell_ref_ch)
    ch_dut.setChannel(loacell_dut_ch)

    # Setting the VoltageRatioChangeTrigger to 0 will result in the channel
    # firing events every DataInterval
    ch_ref.setVoltageRatioChangeTrigger(0)
    ch_dut.setVoltageRatioChangeTrigger(0)

    # The phidgets bridge has a lot of noise if two loadcells are sampling at
    # different frequencies. Pick either 100ms or 50ms period for best results.
    # Anything lower is much worse.
    loadcell_sample_interval = 100  # ms
    ch_ref.setDataInterval(loadcell_sample_interval)
    ch_dut.setDataInterval(loadcell_sample_interval)

    # verify that the rate has been changed TODO progromatic
    print("Loadcell rates")
    print("REF:", ch_ref.getDataRate())
    print("DUT:", ch_dut.getDataRate())

    # TODO figure out best gain for calibration
    ch_ref.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
    ch_dut.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)

    ## Ask if motor needs to move back, if so move it back
    # TODO
    ## Tare both loadcells

    loadcell_values = {ch_ref: [], ch_dut: []}
    def save_bridge_value(vri, value):
        loadcell_values[vri.self.getChannel()].append(value)

    ch_ref.setOnVoltageRatioChangeHandler(save_bridge_value)
    ch_dut.setOnVoltageRatioChangeHandler(save_bridge_value)

    print("opening attachement to bridge")
    ch_ref.openWaitForAttachment(1000)
    ch_dut.openWaitForAttachment(1000)

    time.sleep(1)

    ch_ref.close()
    ch_dut.close()

    tare_ref = statistics.fmean(loadcell_values[ch_ref])
    tare_dut = statistics.fmean(loadcell_values[ch_dut])

    print("tare ref:", tare_ref, "len of sample", len(loadcell_values[ch_ref]))
    print("tare dut:", tare_dut, "len of sample", len(loadcell_values[ch_dut]))

    ## Assume positive is compression
    ## Start recodring both loadcells




if __name__ == "__main__":
    # A bunch of hard coded variables for now
    MOTOR_COM = "COM3"  # TODO progromatic it in the future
    MOTOR_BAUD = 115200  # This preconfigured for the controller
    MOTOR_ADDR = 0x80  # This is preconfigured
    # The motor controller library kinda sucks cause it needs the address passed
    # every time. TODO: just write a better one later.


    LOADCELL_REF_FSO = 3.0049 / 1000  # Full Scale output in V/V
    LOADCELL_REF_CAPACITY = 500 * 0.45359237  # in kg converted from lbs

    LOADCELL_REF_SCALE = LOADCELL_REF_CAPACITY / LOADCELL_REF_FSO  # kg / V/V

    LOADCELL_REF_CH = 3  # Wired in the 3rd channel

    LOADCELL_DUT_CH = 0

    generate_calibration_curve()

    # Note: the mavin loadcell should be 2 mV / V for 200kg, which works out to
    # a scale of 200 kg / 2mV/V = 100 kg / mv/V or 100'000 kg / V/V