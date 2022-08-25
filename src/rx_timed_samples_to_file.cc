//
// Copyright 2010-2011,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
// Modifications: Yago Lizarribar
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <chrono>
#include <complex>
#include <csignal>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>

#include "utils.hh"

// library shortcuts
namespace po = boost::program_options;
using json = nlohmann::json;

static bool stop_signal_called = false;
void sig_int_handler(int) { stop_signal_called = true; }

template <typename samp_type>
// clang-format off
void recv_to_file(uhd::usrp::multi_usrp::sptr usrp, 
                  const std::string &cpu_format,
                  const std::string &wire_format, 
                  const size_t &channel, 
                  const std::string &file,
                  size_t samps_per_buff, 
                  uint64_t num_requested_samples,
                  bool use_binary,
                  const std::string &binfmt,
                  double time_requested = 0.0, 
                  bool bw_summary = false, 
                  bool stats = false,
                  bool null = false, 
                  bool enable_size_map = false, 
                  bool throw_on_overflow = true,
                  bool continue_on_bad_packet = false)
// clang-format on
{
    // json data
    json data;

    // setting up the streamerd
    uint64_t num_total_samps = 0;
    // create a receive streamer
    uhd::stream_args_t stream_args(cpu_format, wire_format);
    std::vector<size_t> channel_nums;
    channel_nums.push_back(channel);
    stream_args.channels = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    uhd::rx_metadata_t md;
    const uint64_t bw = static_cast<uint64_t>(usrp->get_rx_rate());
    const uint64_t num_buffs = (num_requested_samples == 0)
                                   ? 60 * bw / samps_per_buff
                                   : num_requested_samples / samps_per_buff;
    std::vector<samp_type> buff(samps_per_buff);
    std::vector<std::vector<samp_type>> buffs;
    buffs.reserve(num_buffs);
    std::vector<uint64_t> full_sec_buff;
    full_sec_buff.reserve(num_buffs);
    std::vector<double> frac_sec_buff;
    frac_sec_buff.reserve(num_buffs);
    std::vector<size_t> rx_samps_buff;
    rx_samps_buff.reserve(num_buffs);
    std::ofstream outfile;
    if (not null) {
        if (use_binary)
            outfile.open(file.c_str(), std::ofstream::binary);
        else
            outfile.open(file.c_str());
    }
    bool overflow_message = true;

    // setup streaming
    uhd::stream_cmd_t stream_cmd((num_requested_samples == 0)
                                     ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS
                                     : uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = size_t(num_requested_samples);
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t();
    rx_stream->issue_stream_cmd(stream_cmd);

    typedef std::map<size_t, size_t> SizeMap;
    SizeMap mapSizes;
    const auto start_time = std::chrono::steady_clock::now();
    const auto stop_time = start_time + std::chrono::milliseconds(int64_t(1000 * time_requested));
    // Track time and samps between updating the BW summary
    auto last_update = start_time;
    uint64_t last_update_samps = 0;

    // insert the metadata of rx stream
    const auto time = std::time(nullptr);
    std::asctime(std::localtime(&time));
    const auto center_freq = usrp->get_rx_freq();
    const auto gain = usrp->get_rx_gain();
    const auto sample_freq = usrp->get_rx_rate();
    if (!use_binary || binfmt == "msgpack") {
        data["start_time"] = time;
        data["center_freq"] = center_freq;
        data["gain"] = gain;
        data["bandwidth"] = sample_freq;
    } else {
        if (outfile.is_open()) {
            outfile.write((const char *)&time, sizeof(long));
            outfile.write((const char *)&center_freq, sizeof(double));
            outfile.write((const char *)&gain, sizeof(double));
            outfile.write((const char *)&sample_freq, sizeof(double));
        }
    }

    // Run this loop until either time expired (if a duration was given), until
    // the requested number of samples were collected (if such a number was
    // given), or until Ctrl-C was pressed.
    std::cout << "Sampling process will start now." << std::endl;
    uint64_t seconds;
    double nanoseconds;
    while (not stop_signal_called and
           (num_requested_samples != num_total_samps or num_requested_samples == 0) and
           (time_requested == 0.0 or std::chrono::steady_clock::now() <= stop_time)) {
        const auto now = std::chrono::steady_clock::now();
        json chunk;
        size_t num_rx_samps = rx_stream->recv(&buff.front(), buff.size(), md, 3.0, enable_size_map);

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << boost::format("Timeout while streaming") << std::endl;
            break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            if (overflow_message) {
                overflow_message = false;
                std::cerr << boost::format("Got an overflow indication. Please consider the "
                                           "following:\n"
                                           "  Your write medium must sustain a rate of "
                                           "%fMB/s.\n"
                                           "  Dropped samples will not be written to the "
                                           "file.\n"
                                           "  This message will not appear again.\n") %
                                 (usrp->get_rx_rate(channel) * sizeof(samp_type) / 1e6);

                if (throw_on_overflow)
                    throw std::runtime_error("Overflow!");
            }
            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            std::string error = str(boost::format("Receiver error: %s") % md.strerror());
            if (continue_on_bad_packet) {
                std::cerr << error << std::endl;
                continue;
            } else
                throw std::runtime_error(error);
        }

        if (enable_size_map) {
            SizeMap::iterator it = mapSizes.find(num_rx_samps);
            if (it == mapSizes.end())
                mapSizes[num_rx_samps] = 0;
            mapSizes[num_rx_samps] += 1;
        }

        num_total_samps += num_rx_samps;

        if (outfile.is_open()) {
            if (use_binary and binfmt == "raw") {
                seconds = md.time_spec.get_full_secs();
                nanoseconds = md.time_spec.get_frac_secs();
                outfile.write((const char *)&num_rx_samps, sizeof(size_t));
                outfile.write((const char *)&seconds, sizeof(int64_t));
                outfile.write((const char *)&nanoseconds, sizeof(double));
                outfile.write((const char *)&buff.front(), num_rx_samps * sizeof(samp_type));
            } else {
            // create our object
            rx_samps_buff.emplace_back(num_rx_samps);
            full_sec_buff.emplace_back(md.time_spec.get_full_secs());
            frac_sec_buff.emplace_back(md.time_spec.get_frac_secs());
            buffs.emplace_back(buff);
            }
        }

        if (bw_summary) {
            last_update_samps += num_rx_samps;
            const auto time_since_last_update = now - last_update;
            if (time_since_last_update > std::chrono::seconds(1)) {
                const double time_since_last_update_s =
                    std::chrono::duration<double>(time_since_last_update).count();
                const double rate = double(last_update_samps) / time_since_last_update_s;
                std::cout << "\t" << (rate / 1e6) << " Msps" << std::endl;
                last_update_samps = 0;
                last_update = now;
            }
        }
    }
    const auto actual_stop_time = std::chrono::steady_clock::now();

    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    rx_stream->issue_stream_cmd(stream_cmd);

    std::cout << boost::format("Stream finished. Saving data to %s...") % file << std::endl;
    if (outfile.is_open()) {
        if (use_binary and binfmt == "raw") {
            outfile.close();
        } else {
            // emplace all data on our json stream
            for (int i = 0; i < rx_samps_buff.size(); i++) {
                json chunk;
                chunk["seconds"] = full_sec_buff[i];
                chunk["nanoseconds"] = frac_sec_buff[i];
                chunk["size"] = rx_samps_buff[i];
                chunk["samples"] = buffs[i];

                data["data"].emplace_back(chunk);
            }

            if (!use_binary)
                outfile << data << std::endl;
            else if (binfmt == "msgpack") {
                std::vector<uint8_t> message = json::to_msgpack(data);
                outfile.write((const char *)&message.front(), message.size() * sizeof(uint8_t));
            } else {
                std::cerr << "What???" << std::endl;
            }
            outfile.close();
        }
    }

    if (stats) {
        std::cout << std::endl;
        const double actual_duration_seconds =
            std::chrono::duration<float>(actual_stop_time - start_time).count();

        std::cout << boost::format("Received %d samples in %f seconds") % num_total_samps %
                         actual_duration_seconds
                  << std::endl;
        const double rate = (double)num_total_samps / actual_duration_seconds;
        std::cout << (rate / 1e6) << " Msps" << std::endl;

        if (enable_size_map) {
            std::cout << std::endl;
            std::cout << "Packet size map (bytes: count)" << std::endl;
            for (SizeMap::iterator it = mapSizes.begin(); it != mapSizes.end(); it++)
                std::cout << it->first << ":\t" << it->second << std::endl;
        }
    }
}

typedef std::function<uhd::sensor_value_t(const std::string &)> get_sensor_fn_t;

bool check_locked_sensor(std::vector<std::string> sensor_names, const char *sensor_name,
                         get_sensor_fn_t get_sensor_fn, double setup_time) {
    if (std::find(sensor_names.begin(), sensor_names.end(), sensor_name) == sensor_names.end())
        return false;

    auto setup_timeout =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(int64_t(setup_time * 1000));
    bool lock_detected = false;

    std::cout << boost::format("Waiting for \"%s\": ") % sensor_name;
    std::cout.flush();

    while (true) {
        if (lock_detected and (std::chrono::steady_clock::now() > setup_timeout)) {
            std::cout << " locked." << std::endl;
            break;
        }
        if (get_sensor_fn(sensor_name).to_bool()) {
            std::cout << "+";
            std::cout.flush();
            lock_detected = true;
        } else {
            if (std::chrono::steady_clock::now() > setup_timeout) {
                std::cout << std::endl;
                throw std::runtime_error(
                    str(boost::format("timed out waiting for consecutive locks "
                                      "on sensor \"%s\"") %
                        sensor_name));
            }
            std::cout << "_";
            std::cout.flush();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << std::endl;
    return true;
}

int UHD_SAFE_MAIN(int argc, char *argv[]) {
    // variables to be set by po
    std::string args, file, file_ts, type, ant, subdev, ref, wirefmt, binfmt;
    size_t channel, total_num_samps, spb, gps_timeout;
    double rate, freq, gain, bw, total_time, setup_time, lo_offset;
    bool use_binary;

    // setup the program options
    po::options_description desc("Allowed options");

    // clang-format off
    desc.add_options()
        ("help,h", "Show this help message")
        ("args", po::value<std::string>(&args)->default_value(""), "Multi uhd device address args")
        ("file", po::value<std::string>(&file)->default_value(""), "Name of the file to write binary samples to")
        ("type,t", po::value<std::string>(&type)->default_value("float"), "Sample type: double, float, or short")
        ("nsamps,n", po::value<size_t>(&total_num_samps)->default_value(0), "Total number of samples to receive")
        ("duration,d", po::value<double>(&total_time)->default_value(0), "Total number of seconds to receive")
        ("spb,s", po::value<size_t>(&spb)->default_value(2000), "Samples per buffer")
        ("rate,r", po::value<double>(&rate)->default_value(1e6), "Rate of incoming samples")
        ("freq,f", po::value<double>(&freq), "RF center frequency in Hz")
        ("lo-offset", po::value<double>(&lo_offset)->default_value(0.0), "Offset for frontend LO in Hz (optional)")
        ("gain,g", po::value<double>(&gain), "Gain for the RF chain")
        ("ant", po::value<std::string>(&ant), "Antenna selection")
        ("subdev", po::value<std::string>(&subdev), "Subdevice specification")
        ("channel", po::value<size_t>(&channel)->default_value(0), "Which channel to use")
        ("bw", po::value<double>(&bw), "Analog frontend filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("gpsdo"), "Reference source (internal, external, mimo, gpsdo)")
        ("wirefmt", po::value<std::string>(&wirefmt)->default_value("sc16"), "Wire format (sc8 or sc16)")
        ("setup", po::value<double>(&setup_time)->default_value(1.0), "Seconds of setup time")
        ("timeout-gps", po::value<size_t>(&gps_timeout)->default_value(300), "Timeout to wait for GPSDO lock")
        ("use-binary", "Save data in binary format")
        ("binary-format", po::value<std::string>(&binfmt)->default_value("msgpack"), "Which format to save to (msgpack, raw IQ)")
        ("progress", "Periodically display short-term bandwidth")
        ("stats", "Show average bandwidth on exit")
        ("sizemap", "Track packet size and display breakdown on exit")
        ("null", "Run without writing to file")
        ("continue", "Don't abort on a bad packet")
        ("skip-lo", "Skip checking LO lock status")
        ("int-n", "Tune USRP with integer-N tuning")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD RX samples to file %s") % desc << std::endl;
        std::cout << std::endl
                  << "This application streams data from a single channel of a USRP "
                     "device to a file.\n"
                  << std::endl;
        return EXIT_FAILURE;
    }

    // check if frequency was set (otherwise we exit)
    if (vm.count("freq") == 0) {
        std::cerr << "Error: Need to specify a center frequency" << std::endl;
        return EXIT_FAILURE;
    }

    bool bw_summary = vm.count("progress") > 0;
    bool stats = vm.count("stats") > 0;
    bool null = vm.count("null") > 0;
    bool enable_size_map = vm.count("sizemap") > 0;
    bool continue_on_bad_packet = vm.count("continue") > 0;

    if (enable_size_map)
        std::cout << "Packet size tracking enabled - will only recv one packet "
                     "at a time!"
                  << std::endl;

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    // lock mboard clocks
    if (vm.count("ref")) {
        if (ref == "gpsdo") {
            try {
                set_usrp_clock_gpsdo(usrp);
            } catch (const std::exception &e) {
                std::cerr << e.what() << std::endl;
                return EXIT_FAILURE;
            }
        } else {
            usrp->set_clock_source(ref);
            std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
            usrp->set_time_now(uhd::time_spec_t(0.0));
        }
    }

    // always select the subdevice first, the channel mapping affects the other
    // settings
    if (vm.count("subdev"))
        usrp->set_rx_subdev_spec(subdev);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    // set the sample rate
    if (rate <= 0.0) {
        std::cerr << "Please specify a valid sample rate" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate / 1e6) << std::endl;
    usrp->set_rx_rate(rate, channel);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate(channel) / 1e6)
              << std::endl
              << std::endl;

    // set the center frequency
    if (vm.count("freq")) { // with default of 0.0 this will always be true
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq / 1e6) << std::endl;
        std::cout << boost::format("Setting RX LO Offset: %f MHz...") % (lo_offset / 1e6)
                  << std::endl;
        uhd::tune_request_t tune_request(freq, lo_offset);
        if (vm.count("int-n"))
            tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_rx_freq(tune_request, channel);
        std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq(channel) / 1e6)
                  << std::endl
                  << std::endl;
    }

    // set the rf gain
    if (vm.count("gain")) {
        std::cout << boost::format("Setting RX Gain: %f dB...") % gain << std::endl;
        usrp->set_rx_gain(gain, channel);
        std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain(channel)
                  << std::endl
                  << std::endl;
    }

    // set the IF filter bandwidth
    if (vm.count("bw")) {
        std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (bw / 1e6) << std::endl;
        usrp->set_rx_bandwidth(bw, channel);
        std::cout << boost::format("Actual RX Bandwidth: %f MHz...") %
                         (usrp->get_rx_bandwidth(channel) / 1e6)
                  << std::endl
                  << std::endl;
    }

    // set the antenna
    if (vm.count("ant"))
        usrp->set_rx_antenna(ant, channel);

    std::this_thread::sleep_for(std::chrono::milliseconds(int64_t(1000 * setup_time)));

    // check Ref and LO Lock detect
    if (not vm.count("skip-lo")) {
        check_locked_sensor(
            usrp->get_rx_sensor_names(channel), "lo_locked",
            [usrp, channel](const std::string &sensor_name) {
                return usrp->get_rx_sensor(sensor_name, channel);
            },
            setup_time);
        if (ref == "mimo") {
            check_locked_sensor(
                usrp->get_mboard_sensor_names(0), "mimo_locked",
                [usrp](const std::string &sensor_name) {
                    return usrp->get_mboard_sensor(sensor_name);
                },
                setup_time);
        }
        if (ref == "external") {
            check_locked_sensor(
                usrp->get_mboard_sensor_names(0), "ref_locked",
                [usrp](const std::string &sensor_name) {
                    return usrp->get_mboard_sensor(sensor_name);
                },
                setup_time);
        }
    }

    // check filename and format if we want timestamp we'll have json
    use_binary = vm.count("use-binary") > 0;
    const auto time = std::time(nullptr);
    std::asctime(std::localtime(&time));
    if (file == "") {
        if (use_binary)
            if (binfmt == "raw")
                file = str(boost::format("measurement_%i.raw") % time);
            else if (binfmt == "msgpack")
                file = str(boost::format("measurement_%i.msgpack") % time);
            else {
                std::cerr << boost::format("Error. Unsupported binary file format. Only "
                                           "raw|msgpack are allowed; You requested: %s") %
                                 binfmt
                          << std::endl;
                return EXIT_FAILURE;
            }
        else
            file = str(boost::format("measurement_%i.json") % time);
    }

    // before sampling we must set the usrp time to the gps time
    // we have to do it after setting all usrp parameters
    // otherwise we loose time sync
    if (ref == "gpsdo") {
        try {
            set_usrp_time_gpsdo(usrp, gps_timeout);
        } catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }

    if (total_num_samps == 0) {
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    // clang-format off
#define recv_to_file_args(format) (usrp,                   \
                                   format,                 \
                                   wirefmt,                \
                                   channel,                \
                                   file,                   \
                                   spb,                    \
                                   total_num_samps,        \
                                   use_binary,             \
                                   binfmt,                 \
                                   total_time,             \
                                   bw_summary,             \
                                   stats,                  \
                                   null,                   \
                                   enable_size_map,        \
                                   continue_on_bad_packet  )
    // clang-format on

    // recv to file
    if (wirefmt == "s16") {
        if (type == "double")
            recv_to_file<double> recv_to_file_args("f64");
        else if (type == "float")
            recv_to_file<float> recv_to_file_args("f32");
        else if (type == "short")
            recv_to_file<short> recv_to_file_args("s16");
        else
            throw std::runtime_error("Unknown type " + type);
    } else {
        if (type == "double")
            recv_to_file<std::complex<double>> recv_to_file_args("fc64");
        else if (type == "float")
            recv_to_file<std::complex<float>> recv_to_file_args("fc32");
        else if (type == "short")
            recv_to_file<std::complex<short>> recv_to_file_args("sc16");
        else
            throw std::runtime_error("Unknown type " + type);
    }

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
