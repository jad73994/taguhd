//
// Copyright 2010-2011 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <iostream>
#include <fstream>
#include <csignal>
#include <complex>

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

template<typename samp_type> void recv_to_file(
    uhd::usrp::multi_usrp::sptr usrp,
    const uhd::io_type_t &io_type,
    const std::string &file,
    size_t samps_per_buff,
    size_t total_num_samps
){
    uhd::rx_metadata_t md;
    std::vector<samp_type> buff(samps_per_buff);
    std::ofstream outfile(file.c_str(), std::ofstream::binary);
    bool overflow_message = true;

    //setup streaming
    uhd::stream_cmd_t stream_cmd((total_num_samps == 0)?
        uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
        uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
    );
    stream_cmd.num_samps = total_num_samps;
    stream_cmd.stream_now = true;
    usrp->issue_stream_cmd(stream_cmd);

    size_t tot_rx_samps = 0;

    while(not stop_signal_called){
        size_t num_rx_samps = usrp->get_device()->recv(
            &buff.front(), buff.size(), md, io_type,
            uhd::device::RECV_MODE_FULL_BUFF
        );

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
            if (overflow_message){
                overflow_message = false;
                std::cerr << boost::format(
                    "Got an overflow indication. Please consider the following:\n"
                    "  Your write medium must sustain a rate of %fMB/s.\n"
                    "  Dropped samples will not be written to the file.\n"
                    "  Please modify this example for your purposes.\n"
                    "  This message will not appear again.\n"
                ) % (usrp->get_rx_rate()*sizeof(samp_type)/1e6);
            }
            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            throw std::runtime_error(str(boost::format(
                "Unexpected error code 0x%x"
            ) % md.error_code));
        }

        outfile.write((const char*)&buff.front(), num_rx_samps*sizeof(samp_type));

	tot_rx_samps += num_rx_samps;
	if ((total_num_samps != 0) && (tot_rx_samps >= (total_num_samps - 5e3))) {
	  break;
	}
    }

    outfile.close();
}

int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, file, type, ant, subdev, ref;
    size_t total_num_samps1, total_num_samps2, spb;
    double rate, freq, gain, bw;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "help message")
      ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
      ("file", po::value<std::string>(&file)->default_value("usrp_samples.dat"), "name of the file to write binary samples to")
      //("type", po::value<std::string>(&type)->default_value("float"), "sample type: double, float, or short")
      ("nsamps1", po::value<size_t>(&total_num_samps1)->default_value(0), "total number of samples to receive")
      ("nsamps2", po::value<size_t>(&total_num_samps2)->default_value(0), "total number of samples to receive")
      ("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer")
      ("rate", po::value<double>(&rate)->default_value(5e6), "rate of incoming samples")
      ("freq", po::value<double>(&freq)->default_value(2.45e9), "RF center frequency in Hz")
      ("gain", po::value<double>(&gain)->default_value(35), "gain for the RF chain")
      ("ant", po::value<std::string>(&ant)->default_value("TX/RX"), "daughterboard antenna selection")
      ("subdev", po::value<std::string>(&subdev), "daughterboard subdevice specification")
      ("bw", po::value<double>(&bw), "daughterboard IF filter bandwidth in Hz")
      ("ref", po::value<std::string>(&ref)->default_value("EXTERNAL"), "waveform type (INTERNAL, EXTERNAL, MIMO)")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD RX samples to file %s") % desc << std::endl;
        return ~0;
    }


    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    //Lock mboard clocks
    if (ref == "MIMO") {
        uhd::clock_config_t clock_config;
        clock_config.ref_source = uhd::clock_config_t::REF_MIMO;
        clock_config.pps_source = uhd::clock_config_t::PPS_MIMO;
        usrp->set_clock_config(clock_config, 0);
    }
    else if (ref == "EXTERNAL") {
        usrp->set_clock_config(uhd::clock_config_t::external(), 0);
    }
    else if (ref == "INTERNAL") {
        usrp->set_clock_config(uhd::clock_config_t::internal(), 0);
    }

    //always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev")) usrp->set_rx_subdev_spec(subdev);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //set the sample rate
    if (not vm.count("rate")){
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl << std::endl;

    //set the center frequency
    if (not vm.count("freq")){
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq/1e6) << std::endl;
    usrp->set_rx_freq(freq);
    std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq()/1e6) << std::endl << std::endl;

    //set the rf gain
    if (vm.count("gain")){
        std::cout << boost::format("Setting RX Gain: %f dB...") % gain << std::endl;
        usrp->set_rx_gain(gain);
        std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain() << std::endl << std::endl;
    }

    //set the IF filter bandwidth
    if (vm.count("bw")){
        std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % bw << std::endl;
        usrp->set_rx_bandwidth(bw);
        std::cout << boost::format("Actual RX Bandwidth: %f MHz...") % usrp->get_rx_bandwidth() << std::endl << std::endl;
    }

    //set the antenna
    if (vm.count("ant")) usrp->set_rx_antenna(ant);

    boost::this_thread::sleep(boost::posix_time::seconds(1)); //allow for some setup time

    //Check Ref and LO Lock detect
    std::vector<std::string> sensor_names;
    sensor_names = usrp->get_rx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp->get_rx_sensor("lo_locked",0);
        std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    sensor_names = usrp->get_mboard_sensor_names(0);
    if ((ref == "MIMO") and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked") != sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked",0);
        std::cout << boost::format("Checking RX: %s ...") % mimo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "EXTERNAL") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end())) {
        uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked",0);
        std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    usrp->set_time_unknown_pps(0.0);
    boost::this_thread::sleep(boost::posix_time::seconds(1)); //allow for some setup time

    // wait till tx is ready
    {
      const std::string wakeup_dir = "/tmp/wakeup_rx1";
      const std::string wakeup_file = "wakeup.txt";
      
      bool is_created = false;
      
      const std::string fullpath = wakeup_dir + "/" + wakeup_file;
      int fd = inotify_init();
      int watch = inotify_add_watch(fd, wakeup_dir.c_str(), IN_MODIFY|IN_CREATE|IN_MOVED_TO);

      if (access(fullpath.c_str(), F_OK) == 0) {
	is_created = true;
      }

      char buf[1024*(sizeof(inotify_event)+16)];
      ssize_t len;

      while (!is_created) {
	len = read(fd, buf, sizeof(buf));
	if (len < 0) {
	  break;
	}
	inotify_event *event;
	for (size_t i = 0; i < static_cast<size_t>(len); i += sizeof(inotify_event)+event->len) {
	  event = reinterpret_cast<inotify_event *>(&buf[i]);
	  if ((event->len > 0) && (wakeup_file == event->name)) {
	    is_created = true;
	    break;
	  }
	}
      }

      inotify_rm_watch(fd, watch);
      close(fd);

      std::cout << boost::format("%s: %s triggered") 
	% boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::universal_time())
	% fullpath.c_str() << std::endl;
    }


    // tell tx that rx is ready
    {
      const std::string wakeup_dir = "/tmp/wakeup_tx1";
      const std::string wakeup_file = "wakeup.txt";
      const std::string fullpath = wakeup_dir + "/" + wakeup_file;
      
      int fd = open(fullpath.c_str(), O_CREAT|O_WRONLY, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
      close(fd);

      std::cout << boost::format("%s: wrote %s") 
	% boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::universal_time())
	% fullpath.c_str() << std::endl;
    }
    
    if (total_num_samps1 == 0){
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    //recv to file
    recv_to_file<std::complex<float> >(usrp, uhd::io_type_t::COMPLEX_FLOAT32, file, spb, total_num_samps1);

    // now signal to matlab
    // touch fullpath.c_str()
    {
      const std::string wakeup_dir = "/tmp/wakeup_matlab1";
      const std::string wakeup_file = "wakeup.txt";
      const std::string fullpath = wakeup_dir + "/" + wakeup_file;
      
      int fd = open(fullpath.c_str(), O_CREAT|O_WRONLY, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
      close(fd);

      std::cout << boost::format("%s: wrote %s") 
	% boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::universal_time())
	% fullpath.c_str() << std::endl;
    }
    
    // now matlab code will run
    // it will wake up tx, which will in turn wake up rx

    // wait for wake up file from tx before proceeding
    {
      const std::string wakeup_dir = "/tmp/wakeup_rx2";
      const std::string wakeup_file = "wakeup.txt";
      
      bool is_created = false;
      
      const std::string fullpath = wakeup_dir + "/" + wakeup_file;
      int fd = inotify_init();
      int watch = inotify_add_watch(fd, wakeup_dir.c_str(), IN_MODIFY|IN_CREATE|IN_MOVED_TO);

      if (access(fullpath.c_str(), F_OK) == 0) {
	is_created = true;
      }

      char buf[1024*(sizeof(inotify_event)+16)];
      ssize_t len;

      while (!is_created) {
	len = read(fd, buf, sizeof(buf));
	if (len < 0) {
	  break;
	}
	inotify_event *event;
	for (size_t i = 0; i < static_cast<size_t>(len); i += sizeof(inotify_event)+event->len) {
	  event = reinterpret_cast<inotify_event *>(&buf[i]);
	  if ((event->len > 0) && (wakeup_file == event->name)) {
	    is_created = true;
	    break;
	  }
	}
      }

      inotify_rm_watch(fd, watch);
      close(fd);

      std::cout << boost::format("%s: %s triggered") 
	% boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::universal_time())
	% fullpath.c_str() << std::endl;

    }

    // tell tx that rx is ready
    // {
    //   const std::string wakeup_dir = "/tmp/wakeup_tx3";
    //   const std::string wakeup_file = "wakeup.txt";
    //   const std::string fullpath = wakeup_dir + "/" + wakeup_file;
      
    //   int fd = open(fullpath.c_str(), O_CREAT|O_WRONLY, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
    //   close(fd);

    //   std::cout << boost::format("%s: wrote %s") 
    // 	% boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::universal_time())
    // 	% fullpath.c_str() << std::endl;
    // }
    
    if (total_num_samps2 == 0){
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    //recv to file
    recv_to_file<std::complex<float> >(usrp, uhd::io_type_t::COMPLEX_FLOAT32, file, spb, total_num_samps2);

    // now signal to matlab
    // touch fullpath.c_str()
    {
      const std::string wakeup_dir = "/tmp/wakeup_matlab2";
      const std::string wakeup_file = "wakeup.txt";
      const std::string fullpath = wakeup_dir + "/" + wakeup_file;
      
      int fd = open(fullpath.c_str(), O_CREAT|O_WRONLY, S_IRUSR|S_IWUSR);
      close(fd);

    }
    
    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return 0;
}
