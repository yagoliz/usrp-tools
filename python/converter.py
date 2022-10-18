#!/usr/bin/python3

#
# Copyright 2022 Yago Lizarribar
# SPDX-License-Identifier: GPL-3.0-or-later
#

import argparse
import os

import numpy as np
import scipy.io


def parse_args():

    """Parse the command line arguments"""
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--filename", type=str, required=True)
    parser.add_argument("-o", "--output-filename", type=str, default="converted")
    parser.add_argument("-t", "--output-filetype", type=str, default=None, choices=["mat", "npz"])

    return parser.parse_args()


def converter(filename, ofilename, otype="mat"):
    # Header obtention
    file_size = os.path.getsize(filename)

    # Let's define the header
    header_t = np.dtype(
        [
            ("start_time", np.uint64),
            ("center_freq", np.double),
            ("gain", np.double),
            ("sample_freq", np.double),
        ]
    )
    header = np.fromfile(filename, dtype=header_t, count=1, sep="", offset=0)
    header_size = header.itemsize

    # Saving with our beloved variable names
    start_time = header[0][0]
    fc = header[0][1]
    gain = header[0][2]
    fs = header[0][3]

    # Get the approximate number of chunks by reading the first
    bytes64 = 8  # number of bytes in a 64 bit type
    chunk_length = np.fromfile(
        filename, dtype=np.uint64, count=1, offset=header_size + 16
    )
    chunk_size = 3 * bytes64 + 2 * (bytes64 // 2) * chunk_length

    nchunks = (file_size - header_size) // chunk_size

    # Reserve arrays
    seconds = np.empty(nchunks, dtype=int)
    nanoseconds = np.empty(nchunks, dtype=np.float64)
    samples = np.empty(((chunk_length * nchunks)[0], 2), dtype=np.float64)

    offset = header_size

    i = 0
    while offset < file_size:
        # subheaders of the chunk
        sec = np.fromfile(filename, dtype=np.uint64, count=1, offset=offset)
        nsec = np.fromfile(filename, dtype=np.float64, count=1, offset=offset + bytes64)
        nsamp = np.fromfile(
            filename, dtype=np.uint64, count=1, offset=offset + 2 * bytes64
        )

        # flat array with the i/q samples of the chunk
        samps_flat = np.fromfile(
            filename,
            dtype=np.float32,
            count=int(nsamp * 2),
            offset=offset + 3 * bytes64,
        )
        samps_i = samps_flat[0::2]
        samps_q = samps_flat[1::2]

        # store our precious data on our result vector
        seconds[i] = sec
        nanoseconds[i] = nsec
        samples[int(i * nsamp) : int((i + 1) * nsamp), 0] = samps_i
        samples[int(i * nsamp) : int((i + 1) * nsamp), 1] = samps_q

        i += 1
        offset += int(3 * bytes64 + 2 * (bytes64 // 2) * nsamp)

    # File saving
    if otype == "mat":
        scipy.io.savemat(
            f"{ofilename}.mat",
            {
                "fs": fs,
                "fc": fc,
                "gain": gain,
                "start_time": start_time,
                "chunk_length": chunk_length,
                "seconds": seconds,
                "nanoseconds": nanoseconds,
                "samples": samples,
            },
        )

    elif otype == "npz":
        np.savez_compressed(
            f"{ofilename}.npz",
            fs=fs,
            fc=fc,
            gain=gain,
            data=samples,
            seconds=seconds,
            nanoseconds=nanoseconds,
            start_time=start_time,
        )

    else:
        raise ValueError("Invalid output file type. Supported ones are: mat|npz")


if __name__ == "__main__":
    # Get our precious arguments
    args = parse_args()

    # Check the filename & also the output filetype
    filename = args.filename
    ofilename = args.output_filename
    otype = args.output_filetype

    outputx = ofilename.split(".")[-1]
    if outputx == "mat" or outputx == "npz":
        if outputx != otype and otype is not None:
            print(f"Warning: Filename has .{outputx} extension, but requested format is .{otype}")
            outputn = ofilename
        else:
            otype = outputx
            outputn = ".".join(ofilename.split(".")[0:-1])
    else:
        outputn = ofilename
        if otype is None:
            otype = "mat"

    # Run the conversion
    converter(filename, ofilename=outputn, otype=otype)
