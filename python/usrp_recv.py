#!/usr/bin/python3

#
# Copyright 2022 Yago Lizarribar
# SPDX-License-Identifier: GPL-3.0-or-later
#

import argparse
import datetime
import time
import sys

import numpy as np
import uhd


def parse_args():
    """Parse the command line arguments"""
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", "--args", default="", type=str)
    parser.add_argument("-c", "--channels", default=[0], nargs="+", type=int)
    parser.add_argument("-d", "--duration", default=20.0, type=float)
    parser.add_argument(
        "-n",
        "--no-numpy",
        default=False,
        action="store_true",
        help="Save output file in NumPy format (default: Yes)",
    )
    parser.add_argument("-f", "--freq", type=float, default=868e6)
    parser.add_argument("-g", "--gain", type=int, default=50)
    parser.add_argument(
        "-n",
        "--no-numpy",
        default=False,
        action="store_true",
        help="Save output file in NumPy format (default: Yes)",
    )
    parser.add_argument("-o", "--output-file", type=str)
    parser.add_argument("-r", "--rate", default=2e6, type=float)
    parser.add_argument(
        "-s",
        "--clock-source",
        type=str,
        default="gpsdo",
        choices=["internal", "external", "gpsdo"],
    )
    parser.add_argument(
        "-t",
        "--device_type",
        type=str,
        default="b200",
        choices=["b200", "n200", "x300"],
    )
    parser.add_argument("--antenna-source", default="TX/RX", choices=["TX/RX", "RX"])
    parser.add_argument("--device-addr", type=str, default=None)
    parser.add_argument("--frames", type=int, default=1000)
    parser.add_argument("--sleep-timer", type=float, default=1.0)
    parser.add_argument("--wait-minute", type=int, default=None)
    parser.add_argument("--init-delay", type=float, default=0.05)
    parser.add_argument("--lock-timeout", type=int, default=300)

    return parser.parse_args()


def main():
    # Get our precious arguments
    args = parse_args()

    # Set the buffer size (so that we avoid overflows)
    device_type = args.device_type
    device_addr = args.device_addr
    frames = args.frames
    if device_type == "b200":
        usrp = uhd.usrp.MultiUSRP(f"type={device_type},num_recv_frames={frames}")
    elif device_type != "b210" and device_addr == None:
        print(f"Error. No IP address specified for USRP type: {device_type}")
        sys.exit(-1)
    elif device_type == "n200" and device_addr != None:
        usrp = uhd.usrp.MultiUSRP(f"addr={device_addr}")
    elif device_type == "x300" and device_addr != None:
        usrp = uhd.usrp.MultiUSRP(f"addr={device_addr}")
    else:
        print("Error. Unknown device.")
        sys.exit(-1)

    # Sampling rate and center frequency
    fs = args.rate
    fc = args.freq
    gain = args.gain
    usrp.set_rx_rate(fs, 0)
    usrp.set_rx_freq(uhd.libpyuhd.types.tune_request(fc), 0)
    usrp.set_rx_gain(gain, 0)

    # Wait for 1 PPS to happen, then set the time at next PPS to 0.0
    time_at_last_pps = usrp.get_time_last_pps().get_real_secs()
    while time_at_last_pps == usrp.get_time_last_pps().get_real_secs():
        time.sleep(
            0.1
        )  # keep waiting till it happens- if this while loop never finishes then the PPS signal isn't there

    clock_source = args.clock_source
    time_at_last_pps = usrp.get_time_last_pps().get_full_secs()
    if clock_source == "gpsdo":

        # Fancy waiting bars
        status_bars = ["-", "\\", "|", "/"]
        i = 0

        # Check if GPSDO is locked
        print("Checking whether GPS is locked...")
        if not (usrp.get_mboard_sensor("gps_locked", 0).to_bool()):
            print("GPS not locked. Waiting until lock is achieved...")
            timeout = args.lock_timeout
            lock_wait_start = time.time()
            while not (usrp.get_mboard_sensor("gps_locked", 0).to_bool()):
                print(f"Waiting... {status_bars[i]}", end="\r")
                i = (i + 1) % 4
                time.sleep(0.2)
                lock_wait_current = time.time()
                if (lock_wait_current - lock_wait_start) > timeout:
                    print("Error. Timeout reached while waiting GPS lock. Exiting...")
                    sys.exit(-1)

            print("")

        print("Locked!")

        # Important to set the time source (especially if we want timed examples)
        usrp.set_clock_source(clock_source)
        usrp.set_time_source(clock_source)

        time_at_last_pps = usrp.get_time_last_pps().get_full_secs()

        # Getting gps time can sometimes be very slow, so we have to be careful
        gps_time = usrp.get_mboard_sensor("gps_time").to_int()

        full_secs = usrp.get_time_now().get_full_secs()
        frac_secs = usrp.get_time_now().get_frac_secs()
        if frac_secs > 0.7 and full_secs == time_at_last_pps:
            time.sleep(0.5)
            usrp.set_time_next_pps(uhd.libpyuhd.types.time_spec(gps_time + 1.0))
        elif full_secs > time_at_last_pps:
            usrp.set_time_next_pps(uhd.libpyuhd.types.time_spec(gps_time + 1.0))
        else:
            usrp.set_time_next_pps(uhd.libpyuhd.types.time_spec(gps_time))
    else:
        usrp.set_clock_source(clock_source)
        usrp.set_time_source(clock_source)

        usrp.set_time_next_pps(uhd.libpyuhd.types.time_spec(0.0))

    sleep_timer = args.sleep_timer
    time.sleep(sleep_timer)  # We have to sleep for the usrp timer to reset

    # Other USRP parameters
    antenna_source = args.antenna_source
    usrp.set_rx_antenna(antenna_source, 0)

    # Initial delay to start sampling
    INIT_DELAY = args.init_delay

    # Data Stream arguments
    channels = args.channels
    st_args = uhd.usrp.StreamArgs("fc32", "sc16")
    st_args.channels = channels
    metadata = uhd.types.RXMetadata()
    rx_streamer = usrp.get_rx_stream(st_args)

    # Check whether we have to wait for some minutes
    if args.wait_minute != None and args.wait_minute >= 1:
        wait_minute = args.wait_minute
        dt = datetime.timedelta(minutes=wait_minute)

        current_time = datetime.datetime.utcnow()
        sleep_until_dt = (current_time + dt).replace(second=0)
        print(
            f"Sampling process will start at (UTC): {sleep_until_dt.strftime('%H:%M:%S.0')}"
        )

        diff = sleep_until_dt.timestamp() - current_time.timestamp()
        while diff > 1e-3:
            time.sleep(diff / 2)  # Don't sleep the whole time
            current_time = datetime.datetime.utcnow()
            diff = (sleep_until_dt - current_time).total_seconds()

    initial_time_dt = datetime.datetime.utcnow()
    start_time = initial_time_dt.timestamp()
    print(
        f"Sampling process will start immediately. (Current Time (UTC): {initial_time_dt.strftime('%H:%M:%S.%f')}"
    )

    # Make a receive buffer
    num_channels = rx_streamer.get_num_channels()
    max_samps_per_packet = frames
    recv_buffer = np.empty((num_channels, max_samps_per_packet), dtype=np.complex64)
    metadata = uhd.types.RXMetadata()

    # Craft and send the Stream Command
    stream_cmd = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
    stream_cmd.stream_now = True
    stream_cmd.time_spec = uhd.types.TimeSpec(
        usrp.get_time_now().get_real_secs() + INIT_DELAY
    )
    rx_streamer.issue_stream_cmd(stream_cmd)

    # To estimate the number of dropped samples in an overflow situation, we need the following
    # On the first overflow, set had_an_overflow and record the time
    # On the next ERROR_CODE_NONE, calculate how long its been since the recorded time, and use the
    # tick rate to estimate the number of dropped samples. Also, reset the tracking variables
    had_an_overflow = False
    last_overflow = uhd.types.TimeSpec(0)

    # Setup the statistic counters
    num_rx_samps = 0
    num_rx_dropped = 0
    num_rx_overruns = 0
    num_rx_seqerr = 0
    num_rx_timeouts = 0
    num_rx_late = 0

    rate = usrp.get_rx_rate()

    # Receive until we get the signal to stop
    duration = args.duration
    total_samps = int(rate * duration)

    # Store data and timestamps
    samples = np.empty((num_channels, total_samps), dtype=np.complex64)
    seconds = np.empty((1, total_samps // max_samps_per_packet), dtype=np.uint64)
    nanoseconds = np.empty((1, total_samps // max_samps_per_packet), dtype=np.float64)

    index = 0
    while num_rx_samps < total_samps * num_channels:
        try:
            rx_samps = rx_streamer.recv(recv_buffer, metadata) * num_channels
            if rx_samps != 0:
                samples[
                    :, (index) * (rx_samps) : (index + 1) * (rx_samps)
                ] = recv_buffer
                seconds[0, index] = metadata.time_spec.get_full_secs()
                nanoseconds[0, index] = metadata.time_spec.get_frac_secs()

                index += 1
                num_rx_samps += rx_samps

        except RuntimeError as ex:
            print("Runtime error while reading samples")
            break

        # Handle the error codes
        if metadata.error_code == uhd.types.RXMetadataErrorCode.none:
            # Reset the overflow flag
            if had_an_overflow:
                had_an_overflow = False
                num_rx_dropped += (metadata.time_spec - last_overflow).to_ticks(rate)
        elif metadata.error_code == uhd.types.RXMetadataErrorCode.overflow:
            print("Overflow!")
            had_an_overflow = True
            # Need to make sure that last_overflow is a new TimeSpec object, not
            # a reference to metadata.time_spec, or it would not be useful
            # further up.
            last_overflow = uhd.types.TimeSpec(
                metadata.time_spec.get_full_secs(), metadata.time_spec.get_frac_secs()
            )
            # If we had a sequence error, record it
            if metadata.out_of_sequence:
                num_rx_seqerr += 1
            # Otherwise just count the overrun
            else:
                num_rx_overruns += 1
        elif metadata.error_code == uhd.types.RXMetadataErrorCode.late:
            print(f"Receiver error: {metadata.strerror()}, restarting streaming...")
            num_rx_late += 1
            # Radio core will be in the idle state. Issue stream command to restart streaming.
            stream_cmd.time_spec = uhd.types.TimeSpec(
                usrp.get_time_now().get_real_secs() + INIT_DELAY
            )
            stream_cmd.stream_now = True
            rx_streamer.issue_stream_cmd(stream_cmd)
        elif metadata.error_code == uhd.types.RXMetadataErrorCode.timeout:
            print(f"Receiver error: {metadata.strerror()}, continuing...")
            num_rx_timeouts += 1
        else:
            print(f"Receiver error: {metadata.strerror()}")
            print("Unexpected error on receive, continuing...")

    print(f"Read: {samples.shape[1]} samples in total.")
    print(f"Dropped samples: {num_rx_dropped}")
    print(f"Overflows: {num_rx_overruns}")
    print(f"Sequence errors: {num_rx_seqerr}")
    print(f"Number of timeouts: {num_rx_timeouts}")

    # Check that system time and usrp time are similar
    usrp_current_time = usrp.get_time_last_pps().get_full_secs()
    system_current_time = int(time.time())

    print(f"USRP time: \t{usrp_current_time} seconds.")
    print(f"System time: \t{system_current_time} seconds")

    # Saving our precious results
    if not args.no_numpy:
        if args.output_file is None:
            filename = f"measurement_{start_time}.npz"
        else:
            filename = f"{args.output_file}.npz"

        np.savez_compressed(
            filename,
            data=samples,
            seconds=seconds,
            nanoseconds=nanoseconds,
            start_time=start_time,
            fs=fs,
            fc=fc,
            gain=gain,
        )


if __name__ == "__main__":
    main()
