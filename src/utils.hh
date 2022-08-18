#pragma once

#include <complex>
#include <csignal>
#include <fstream>
#include <iostream>

#include <boost/format.hpp>
#include <nlohmann/json.hpp>
#include <uhd/usrp/multi_usrp.hpp>

// Defining json ser/des for std::complex
using json = nlohmann::json;
namespace std {

template <class T> void to_json(json &j, const std::complex<T> &p) { j = json{p.real(), p.imag()}; }

template <class T> void from_json(const json &j, std::complex<T> &p) {
    p.real(j.at(0));
    p.imag(j.at(1));
}
} // namespace std

// Setting of the USRP to the GPSDO source
void set_usrp_clock_gpsdo(uhd::usrp::multi_usrp::sptr usrp) {
    bool ref_set_to_gpsdo = false;

    // Set clock source to gpsdo if supported
    if (uhd::has(usrp->get_clock_sources(0), "gpsdo")) {
        std::cout << "Setting the reference clock source to \"gpsdo\"..." << std::endl;
        usrp->set_clock_source("gpsdo");
        ref_set_to_gpsdo = true;
    }
    std::cout << "Clock source is now " << usrp->get_clock_source(0) << std::endl;

    // Set time source to gpsdo if supported
    if (uhd::has(usrp->get_time_sources(0), "gpsdo")) {
        std::cout << "Setting the reference clock source to \"gpsdo\"..." << std::endl;
        usrp->set_time_source("gpsdo");
        ref_set_to_gpsdo = true;
    }
    std::cout << "Time source is now " << usrp->get_time_source(0) << std::endl;

    if (not ref_set_to_gpsdo) {
        std::string message = "ERROR: Unable to set clock or time reference to \"gpsdo\"\n";
        throw std::runtime_error(message);
    }

    // Check for ref lock
    std::vector<std::string> sensor_names = usrp->get_mboard_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end()) {
        std::cout << "Waiting for ref_locked..." << std::flush;
        uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked", 0);
        auto end = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        while (!ref_locked.to_bool() && std::chrono::steady_clock::now() < end) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            ref_locked = usrp->get_mboard_sensor("ref_locked", 0);
            std::cout << "." << std::flush;
        }
        if (not ref_locked.to_bool()) {
            std::string message = "USRP NOT Locked to Reference.\n";
            throw std::runtime_error(message);
        } else {
            std::cout << "USRP Locked to Reference.\n";
        }
    }
}

// Setting the time of the USRP to the GPSDO
void set_usrp_time_gpsdo(uhd::usrp::multi_usrp::sptr usrp, unsigned int gps_timeout) {
    // The TCXO has a long warm up time, so wait up to 30 seconds for sensor
    // data to show up
    std::cout << "Waiting for the GPSDO to warm up..." << std::flush;
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (std::chrono::steady_clock::now() < end) {
        try {
            usrp->get_mboard_sensor("gps_locked", 0);
            break;
        } catch (std::exception &) {
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        std::cout << "." << std::flush;
    }
    std::cout << std::endl;
    try {
        usrp->get_mboard_sensor("gps_locked", 0);
    } catch (std::exception &) {
        std::string message = "No response from GPSDO in 30 seconds\n";
        throw std::runtime_error(message);
    }
    std::cout << "The GPSDO is warmed up and talking." << std::endl;

    // Check for GPS lock
    uhd::sensor_value_t gps_locked = usrp->get_mboard_sensor("gps_locked", 0);

    if (not gps_locked.to_bool()) {
        std::cout << boost::format("\nGPS does not have lock. Will wait for %i seconds.\n") %
                         gps_timeout
                  << std::endl;

        bool got_lock = false;
        auto wait_start = std::chrono::steady_clock::now();
        auto wait_end = wait_start;
        while ((wait_end - wait_start).count() < gps_timeout) {
            gps_locked = usrp->get_mboard_sensor("gps_locked", 0);

            if (gps_locked.to_bool()) {
                got_lock = true;
                break;
            }

            std::this_thread::sleep_for(std::chrono::seconds(5));
        }

        if (!got_lock) {
            std::string message = "Timeout reached without getting lock on GPS. Exiting...\n";
            throw std::runtime_error(message);
        }
    }

    std::cout << "GPS Locked!" << std::endl;

    // Check PPS and compare UHD device time to GPS time
    uhd::sensor_value_t gps_time = usrp->get_mboard_sensor("gps_time");
    uhd::time_spec_t last_pps_time = usrp->get_time_last_pps();

    // we only care about the full seconds
    signed gps_seconds = gps_time.to_int();
    long long pps_seconds = last_pps_time.to_ticks(1.0);

    if (pps_seconds != gps_seconds) {
        std::cout << "\nTrying to align the device time to GPS time..." << std::endl;

        gps_time = usrp->get_mboard_sensor("gps_time");

        // set the device time to the GPS time
        // getting the GPS time returns just after the PPS edge, so just add a
        // second and set the device time at the next PPS edge
        usrp->set_time_next_pps(uhd::time_spec_t(gps_time.to_int() + 1.0));
        // allow some time to make sure the PPS has come…
        std::this_thread::sleep_for(std::chrono::milliseconds(1100));
        //…then ask
        gps_seconds = usrp->get_mboard_sensor("gps_time").to_int();
        pps_seconds = usrp->get_time_last_pps().to_ticks(1.0);
    }

    if (pps_seconds == gps_seconds) {
        std::cout << "GPS and UHD Device time are aligned.\n";
    } else {
        std::string message = "Could not align UHD Device time to GPS time. Giving up.\n";
        throw std::runtime_error(message);
    }
    std::cout << boost::format("last_pps: %ld vs gps: %ld.") % pps_seconds % gps_seconds
              << std::endl;
}