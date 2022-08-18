//
// Copyright 2010-2011 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Modified by: Yago Lizarribar
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <chrono>
#include <complex>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>

#include "utils.hh"

namespace po = boost::program_options;

// Main block
int UHD_SAFE_MAIN(int argc, char *argv[]) {
    // variables to be set by po
    std::string args;
    std::string wire;
    double seconds_in_future;
    size_t total_num_samps;
    double rate;
    std::string channel_list;
    unsigned int gps_timeout;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("wire", po::value<std::string>(&wire)->default_value(""), "the over the wire type, sc16, sc8, etc")
        ("secs", po::value<double>(&seconds_in_future)->default_value(1.5), "number of seconds in the future to receive")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(10000), "total number of samples to receive")
        ("rate", po::value<double>(&rate)->default_value(100e6/16), "rate of incoming samples")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("with-gps", "whether to sync USRP time to GPSDO")
        ("gps-timeout",po::value<unsigned int>(&gps_timeout)->default_value(300), "amount of time to wait for GPS lock")
        ("dilv", "specify to enable inner-loop verbose")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD RX Timed Samples %s") % desc << std::endl;
        return ~0;
    }

    bool verbose = vm.count("dilv") == 1;

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    std::vector<std::string> sensor_names = usrp->get_mboard_sensor_names(0);

    // detect whether gpsdo is present (if not, it's pointless to sample if
    // use-gps option is there)
    bool use_gps = vm.count("with-gps") == 1;
    if (use_gps) {
        if (std::find(sensor_names.begin(), sensor_names.end(), "gps_locked") ==
            sensor_names.end()) {
            std::cout << "\ngps_locked sensor not found. This could mean that GPSDO "
                         "is not installed correctly.\n\n";
            return EXIT_FAILURE;
        }
        std::cout << "GPSDO detected! USRP will use GPS time to synchronize" << std::endl;
    }

    // detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < channel_strings.size(); ch++) {
        size_t chan = std::stoi(channel_strings[ch]);
        if (chan >= usrp->get_tx_num_channels() or chan >= usrp->get_rx_num_channels()) {
            throw std::runtime_error("Invalid channel(s) specified.");
        } else
            channel_nums.push_back(std::stoi(channel_strings[ch]));
    }

    // set the rx sample rate
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate / 1e6) << std::endl;
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate() / 1e6)
              << std::endl
              << std::endl;

    if (use_gps) {
        // now set the usrp's time to the GPS time
        try {
            set_usrp_clock_gpsdo(usrp);
            set_usrp_time_gpsdo(usrp, gps_timeout);
        } catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
            return -1;
        }
    } else {
        std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
        usrp->set_time_now(uhd::time_spec_t(0.0));
    }

    // create a receive streamer
    uhd::stream_args_t stream_args("fc32", wire); // complex floats
    stream_args.channels = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    // setup streaming
    std::cout << std::endl;
    std::cout << boost::format("Begin streaming %u samples, %f seconds in the future...") %
                     total_num_samps % seconds_in_future
              << std::endl;
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = total_num_samps;
    stream_cmd.stream_now = false;

    if (use_gps) {
        auto current_time = usrp->get_time_last_pps().get_real_secs();
        stream_cmd.time_spec = uhd::time_spec_t(current_time + seconds_in_future);
    } else {
        stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
    }
    rx_stream->issue_stream_cmd(stream_cmd);

    // meta-data will be filled in by recv()
    uhd::rx_metadata_t md;

    // allocate buffer to receive with samples
    std::vector<std::complex<float>> buff(rx_stream->get_max_num_samps());
    std::vector<void *> buffs;
    for (size_t ch = 0; ch < rx_stream->get_num_channels(); ch++)
        buffs.push_back(&buff.front()); // same buffer for each channel

    // allocate the vectors to store the time
    std::vector<uint64_t> chunk_full_secs(total_num_samps / rx_stream->get_max_num_samps());
    std::vector<float> chunk_frac_secs(total_num_samps / rx_stream->get_max_num_samps());

    // the first call to recv() will block this many seconds before receiving
    double timeout = seconds_in_future + 0.1; // timeout (delay before receive + padding)

    size_t num_acc_samps = 0; // number of accumulated samples
    while (num_acc_samps < total_num_samps) {
        // receive a single packet
        size_t num_rx_samps = rx_stream->recv(buffs, buff.size(), md, timeout, true);

        // use a small timeout for subsequent packets
        timeout = 0.1;

        // handle the error code
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
            break;
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            throw std::runtime_error(str(boost::format("Receiver error %s") % md.strerror()));
        }

        if (verbose)
            std::cout << boost::format("Received packet: %u samples, %u full "
                                       "secs, %f frac secs") %
                             num_rx_samps % md.time_spec.get_full_secs() %
                             md.time_spec.get_frac_secs()
                      << std::endl;

        chunk_full_secs.emplace_back(md.time_spec.get_full_secs());
        chunk_frac_secs.emplace_back(md.time_spec.get_frac_secs());

        num_acc_samps += num_rx_samps;
    }

    if (num_acc_samps < total_num_samps)
        std::cerr << "Receive timeout before all samples received..." << std::endl;

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
