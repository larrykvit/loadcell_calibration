"""
Uses the lodacell calibration curves and generates a calibration value.

Requires 2 curves - one from a calibrated loadcell, and one from the loadcell
under test.

Mavin NA128 is 2mv/V at 200kg
2/1000 V/V / 200 kg = 0.00001 V/V / kg = 1e-05
1/ 1e-05 V/V / kg = 100'000 kg / V/V

"""

from pathlib import Path
import argparse

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


def interpolate_2x(vals):
    """Cubic interpolation in between each value"""
    xp = np.arange(0, len(vals), 1)  # indexes
    x = np.arange(0, len(vals) - 0.5, 0.5)  # half step indexes
    assert len(xp) * 2 - 1 == len(x)

    # There are some peak loads when the motor stops, so a cubic spline fits
    # better than just linear interpolation. And the acceleration is smoother.
    vals_spline = CubicSpline(xp, vals)

    # vals_linear = np.interp(x, xp, vals)

    # plt.plot(x, vals_linear, label="linear")
    # plt.plot(x, vals_spline(x), label="cubic")
    # plt.plot(xp, vals, 'o', label='data')
    # plt.legend(loc='best')
    # plt.show()
    return vals_spline(x)


def line_fit_residual(x, y):
    """Calculate how close the points fit to a line"""
    assert len(x) == len(y)
    assert x.ndim == 1
    assert y.ndim == 1
    # lol, this is the new np.polyfit
    _, (resid, _, _, _) = np.polynomial.polynomial.Polynomial.fit(x, y, 1, full=True)
    return resid[0]


def parse_calibration_curve(
    values_ref: list[float], values_dut: list[float], scale_ref: float, to_plot=False
):
    # Tare isn't needed - curve fitting a line with offset is more accurate.
    # Its only needed if you are just looking at the ratio of the two inputs.
    # TODO - figure out if the tare is really needed, or if you should just
    # curve fit a line with an offset
    values_ref = np.array(values_ref)
    values_dut = np.array(values_dut)

    # The ADC muxes between the 2 values. So they are offest by half a sample.
    # Also, the sampling could have started and stopped on either channel.
    # A A A A  vs  A A A A
    #  B B B B    B B B B
    # Upscale the data by 2x, so that the offset is now 1 sample
    # When plotting dut vs ref, the data should be ideally a straight line;
    # this means that they both are reading the same values. If it is a straight
    # line offset in either direction (due to moving forwards and backwards)
    # that means the data is offset.
    # Offset the data in either direction and see which one has less error.
    # This assumes the testing data has both increasing and decreasing values.
    # Since that will produce a shift in both directions.

    interp_ref = interpolate_2x(values_ref)
    interp_dut = interpolate_2x(values_dut)

    # The shift should make the plot of ref vs dut be a line. When it isn't
    # there will be a shift in both directions from this line.
    min_len = min(len(interp_ref), len(interp_dut))
    slice_shift_back = slice(1, min_len)
    slice_no_shift = slice(0, min_len - 1)

    cut_dut, cut_ref = min(
        (interp_dut[slice_shift_back], interp_ref[slice_no_shift]),  # DUT first
        (interp_dut[slice_no_shift], interp_ref[slice_shift_back]),  # REF first
        key=lambda x: line_fit_residual(*x),
    )

    if to_plot:
        plt.plot(cut_dut)
        plt.plot(cut_ref)
        plt.show()

        vel_dut = np.gradient(cut_dut)
        vel_ref = np.gradient(cut_ref)
        accel_dut = np.gradient(vel_dut)
        accel_ref = np.gradient(vel_ref)

        plt.plot(accel_dut)
        plt.plot(accel_ref)
        plt.show()

        plt.scatter(np.diff(cut_dut, n=2), np.diff(cut_ref, n=2))
        plt.show()

    ref_val_max = np.max(cut_ref)
    ref_val_min = np.min(cut_ref)
    ref_val_delta = ref_val_max - ref_val_min

    # the acceleration starts after a hardcoded minimal value
    # should cut out all load that doesn't use smooth moving.
    # TODO see if this can be hardcoded even less
    min_load_fraction = 0.1
    start, end = np.where(cut_ref > min_load_fraction * ref_val_delta + ref_val_min)[0][
        [0, -1]
    ]
    print(100 * min_load_fraction, "% load starts, end", start, end)
    slice_to_fit = slice(start, end)

    # How the math works out:
    # scale_ref * value_ref = scale_dut * value_dut (same weight)
    # scale_ref * value_ref/ value_dut = scale_dut

    print("fitline")
    fit_line, (resid, _, _, _) = np.polynomial.polynomial.Polynomial.fit(
        cut_dut[slice_to_fit], cut_ref[slice_to_fit], 1, full=True
    )
    fit_coef = fit_line.convert().coef

    fit_ref = fit_line(cut_dut)
    error = fit_ref - cut_ref

    print("Error in the fit region, min, max:")
    print(np.min(error[slice_to_fit]))
    print(np.max(error[slice_to_fit]))

    # figure out where the error is coming from
    if to_plot:

        plt.plot(cut_ref)
        plt.plot(cut_dut)
        plt.plot(error * 1000)
        plt.show()

    scale_dut = fit_coef[1] * scale_ref
    print("scale", scale_dut)
    print("error from  spec:", (1 - scale_dut / 100000) * 100, "%")
    print("error at 200kg (if using 2mv/V):", 200 - scale_dut * 0.002)
    print("offset", fit_coef[0])
    print("residual", resid[0])

    return scale_dut, resid[0]


if __name__ == "__main__":
    # TODO make the description the doc of this file
    parser = argparse.ArgumentParser(
        description="Uses the lodacell calibration curves and generates a calibration value."
    )
    parser.add_argument("calibration_data_path", type=Path)
    # TODO make the path the latest calibration for default
    args = parser.parse_args()

    dir_calibration = args.calibration_data_path

    # Read the 3 inputs needed
    values_ref = np.loadtxt(dir_calibration / "loadcell_values_ref.txt")
    values_dut = np.loadtxt(dir_calibration / "loadcell_values_dut.txt")
    loadcell_ref_scale = np.loadtxt(dir_calibration / "loadcell_ref_scale.txt")

    parse_calibration_curve(
        values_ref, values_dut, float(loadcell_ref_scale), to_plot=True
    )
