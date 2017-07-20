//
// Copyright 2011 Ettus Research LLC
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
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <csignal>
#include <iostream>
#include <fstream>
#include <complex>

#include <fcntl.h>
#include <unistd.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

void send_from_file(uhd::usrp::multi_usrp::sptr usrp,
		    std::vector <std::vector < std::complex< float > > > &buffs,
		    size_t samps_per_buff,
		    double rate,
		    const uhd::time_spec_t &send_time
){
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst = false;
    md.has_time_spec = true;
    md.time_spec = send_time;

    size_t nleft = buffs[0].size();
    size_t buff_index = 0;

    //loop until the entire file has been read
    while(not md.end_of_burst and not stop_signal_called){

      size_t num_tx_samps = samps_per_buff;
      if (nleft < num_tx_samps) {
	num_tx_samps = nleft;
      }
      
      std::vector<std::complex<float> *> buff_ptrs;
      for (size_t i = 0; i < buffs.size(); ++i) {
	buff_ptrs.push_back((&buffs[i].front()) + buff_index);
      }

      buff_index += num_tx_samps;
      nleft -= num_tx_samps;

      md.end_of_burst = (nleft == 0);

      unsigned int nsent = usrp->get_device()->send(buff_ptrs, num_tx_samps, md, 
						    uhd::io_type_t::COMPLEX_FLOAT32,
						    uhd::device::SEND_MODE_FULL_BUFF, 
						    3.0);

      if (nsent != num_tx_samps) {
	std::cerr << boost::format("nsent %u is less than num_tx_samps %u") % nsent % num_tx_samps << std::endl;
	exit(1);
      }

      md.start_of_burst = false;
      md.has_time_spec = false;

      //md.time_spec += (num_tx_samps*1.0/rate);
    }
}

int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, file, type, ant, subdev, ref;
    size_t spb, nrepeat1, nrepeat2;
    double rate, freq, gain, bw, delay1, delay2, wait1, wait2;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "help message")
      ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
      ("file", po::value<std::string>(&file)->default_value("usrp_samples"), "name of the file to read binary samples from")
      //("type", po::value<std::string>(&type)->default_value("float"), "sample type: double, float, or short")
      ("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer")
      ("rate", po::value<double>(&rate)->default_value(5e6), "rate of outgoing samples")
      ("freq", po::value<double>(&freq)->default_value(2.45e9), "RF center frequency in Hz")
      ("gain", po::value<double>(&gain)->default_value(1.0), "gain for the RF chain")
      ("ant", po::value<std::string>(&ant)->default_value("TX/RX"), "daughterboard antenna selection")
      ("subdev", po::value<std::string>(&subdev), "daughterboard subdevice specification")
      ("bw", po::value<double>(&bw), "daughterboard IF filter bandwidth in Hz")
      ("ref", po::value<std::string>(&ref)->default_value("EXTERNAL"), "waveform type (INTERNAL, EXTERNAL, MIMO)")
      ("wait1", po::value<double>(&wait1)->default_value(1.0), "specify wait 1 (in sec) before first transmission of file")
      ("wait2", po::value<double>(&wait2)->default_value(1.0), "specify wait 2 (in sec) before first transmission of file")
      ("delay1", po::value<double>(&delay1)->default_value(1000), "specify delay 1 (in msec) between repeated transmission of file")
      ("delay2", po::value<double>(&delay2)->default_value(1000), "specify delay 2 (in msec) between repeated transmission of file")
      ("nrepeat1", po::value<size_t>(&nrepeat1)->default_value(1), "nrepeats1 (0 is infinite)")
      ("nrepeat2", po::value<size_t>(&nrepeat2)->default_value(1), "nrepeats2 (0 is infinite)")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD TX samples from file %s") % desc << std::endl;
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
	for (size_t i = 0; i < usrp->get_num_mboards(); ++i) {
	  usrp->set_clock_config(clock_config, i);
	}
    }
    else if (ref == "EXTERNAL") {
	for (size_t i = 0; i < usrp->get_num_mboards(); ++i) {
	  usrp->set_clock_config(uhd::clock_config_t::external(), i);
	}
    }
    else if (ref == "INTERNAL") {
	for (size_t i = 0; i < usrp->get_num_mboards(); ++i) {
	  usrp->set_clock_config(uhd::clock_config_t::internal(), i);
	}
    }

    //always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev")) {
      for (size_t i = 0; i < usrp->get_num_mboards(); ++i) {
	usrp->set_tx_subdev_spec(subdev, i);
      }
    }

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //set the sample rate
    if (not vm.count("rate")){
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }
    for (size_t i = 0; i < usrp->get_tx_num_channels(); ++i) {
      std::cout << boost::format("Setting TX Rate %u: %f Msps...") % i % (rate/1e6) << std::endl;
      usrp->set_tx_rate(rate, i);
      std::cout << boost::format("Actual TX Rate %u: %f Msps...") % i % (usrp->get_tx_rate(i)/1e6) << std::endl << std::endl;
    }

    //set the center frequency
    if (not vm.count("freq")){
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return ~0;
    }
    for (size_t i = 0; i < usrp->get_tx_num_channels(); ++i) {
      std::cout << boost::format("Setting TX Freq %u: %f MHz...") % i % (freq/1e6) << std::endl;
      usrp->set_tx_freq(freq, i);
      std::cout << boost::format("Actual TX Freq %u: %f MHz...") % i % (usrp->get_tx_freq(i)/1e6) << std::endl << std::endl;
    }

    //set the rf gain
    if (vm.count("gain")){
      for (size_t i = 0; i < usrp->get_tx_num_channels(); ++i) {
        std::cout << boost::format("Setting TX Gain %u: %f dB...") % i % gain << std::endl;
        usrp->set_tx_gain(gain, i);
        std::cout << boost::format("Actual TX Gain %u: %f dB...") % i % usrp->get_tx_gain(i) << std::endl << std::endl;
      }
    }

    //set the IF filter bandwidth
    if (vm.count("bw")){
      for (size_t i = 0; i < usrp->get_tx_num_channels(); ++i) {
        std::cout << boost::format("Setting TX Bandwidth %u: %f MHz...") % i % bw << std::endl;
        usrp->set_tx_bandwidth(bw, i);
        std::cout << boost::format("Actual TX Bandwidth %u: %f MHz...") % usrp->get_tx_bandwidth(i) << std::endl << std::endl;
      }
    }

    //set the antenna
    if (vm.count("ant")) {
      for (size_t i = 0; i < usrp->get_tx_num_channels(); ++i) {
	usrp->set_tx_antenna(ant, i);
      }
    }

    boost::this_thread::sleep(boost::posix_time::seconds(1)); //allow for some setup time

    //Check Ref and LO Lock detect
    for (size_t i = 0; i < usrp->get_tx_num_channels(); ++i) {
      std::vector<std::string> sensor_names;
      sensor_names = usrp->get_tx_sensor_names(i);
      if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked", i);
        std::cout << boost::format("Checking TX %u: %s ...") % i % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
      }
    }

    for (size_t i = 0; i < usrp->get_num_mboards(); ++i) {
      std::vector<std::string> sensor_names;
      sensor_names = usrp->get_mboard_sensor_names(i);
      if ((ref == "EXTERNAL") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end())) {
        uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked", i);
        std::cout << boost::format("Checking TX %u: %s ...") % i % ref_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
      }
    }

    usrp->set_time_unknown_pps(0.0);
    boost::this_thread::sleep(boost::posix_time::seconds(1)); //allow for some setup time

    // touch fullpath.c_str()
    // signal to rx that tx is ready
    {
      const std::string wakeup_dir = "/tmp/wakeup_rx1";
      const std::string wakeup_file = "wakeup.txt";
      const std::string fullpath = wakeup_dir + "/" + wakeup_file;
      
      int fd = open(fullpath.c_str(), O_CREAT|O_WRONLY, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
      close(fd);

      std::cout << boost::format("%s: wrote %s") 
	% boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::universal_time())
	% fullpath.c_str() << std::endl;
    }

    // wait for rx to write the file to wake tx up, at this point rx is ready
    {
      const std::string wakeup_dir = "/tmp/wakeup_tx1";
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



    //set sigint if user wants to receive
    if (nrepeat1 == 0) {
      std::signal(SIGINT, &sig_int_handler);
      std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    size_t tx_num_channels = usrp->get_tx_num_channels();

    {
      double txtime = 0;
      std::vector <std::vector < std::complex< float > > > buffs(tx_num_channels);

      std::vector<std::complex<float> > tmpbuff(spb);

      for (size_t i = 0; i < buffs.size(); ++i) {
	std::string fname = file + "_" + boost::lexical_cast<std::string>(i) + ".dat";
	std::ifstream infile;
	infile.open(fname.c_str(), std::ifstream::binary);
	if (infile.fail()) {
	  std::cerr << "Could not open file " << fname.c_str() << std::endl;
	  exit(1);
	}

	float max_abs = -1;
	do  {
	  infile.read((char *)&tmpbuff.front(), tmpbuff.size()*sizeof(std::complex<float>));
	  size_t nread = infile.gcount()/sizeof(std::complex<float>);

	  for (size_t j = 0; j < nread; ++j) {
	    tmpbuff[j] *= float(gain);

	    if (max_abs < std::abs(tmpbuff[j].real())) {
	      max_abs = std::abs(tmpbuff[j].real());
	    }
	    if (max_abs < std::abs(tmpbuff[j].imag())) {
	      max_abs = std::abs(tmpbuff[j].imag());
	    }
	  }

	  buffs[i].insert(buffs[i].end(), tmpbuff.begin(), tmpbuff.begin() + nread);
	} while (!infile.eof());

	infile.close();

	std::cout << boost::format("Max abs in file %s is %e") % fname.c_str() % max_abs << std::endl;
      }

      txtime = buffs[0].size()*1.0/rate;
      std::cout << boost::format("Total samples per file is %u, txtime is %e") % buffs[0].size() % txtime << std::endl;

      if (delay1 < txtime) {
	std::cerr << boost::format("Delay %e is less than packet txtime %e") % delay1 % txtime << std::endl;
	exit(1);
      }

      //send from file
      // uhd::time_spec_t t0 = usrp->get_time_now(0);
      // uhd::time_spec_t t1 = usrp->get_time_now(1);
      // double diff = t0.get_real_secs() - t1.get_real_secs();
      // std::cout << boost::format("t0=%e diff = %e") % t0.get_real_secs() % diff << std::endl;
      
      uhd::time_spec_t send_time = usrp->get_time_now() + wait1;

      unsigned int count = 0;
      do {
	send_from_file(usrp, buffs, spb, rate, send_time);
	send_time = send_time + delay1/1000;
	++count;

	// uhd::time_spec_t t0 = usrp->get_time_now(0);
	// uhd::time_spec_t t1 = usrp->get_time_now(1);
	// double diff = t0.get_real_secs() - t1.get_real_secs();
	// std::cout << boost::format("t0=%e diff = %e") % t0.get_real_secs() % diff << std::endl;
      
	// double sleep_time = delay-200;
	  
	// if (sleep_time > 0) {
	//   boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_time)); // just sleep a bit to prevent from getting too far ahead
	// }

	if ((count % ((unsigned int)1e1)) == 0) {
	  std::cout << boost::format("%d... ") % (count) << std::flush;
	}

      } while ((count != nrepeat1) && (!stop_signal_called));
    }

    std::cout << boost::format("%s: first tx done") 
      % boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::universal_time()) << std::endl;

    if (nrepeat2 == 0) {
      std::signal(SIGINT, &sig_int_handler);
      std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    // {
    //   uhd::async_metadata_t async_md;
    //   std::cout << boost::format("async md: ");
    //   while (usrp->get_device()->recv_async_msg(async_md, 3.0)) {
    // 	std::cout << boost::format("%x... ") % async_md.event_code;
    //   }
    //   std::cout << std::endl;
    // }

    // now wait for matlab to write the receiver files
    {
      const std::string wakeup_dir = "/tmp/wakeup_tx2";
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

    // wake up rx
    {
      const std::string wakeup_dir = "/tmp/wakeup_rx2";
      const std::string wakeup_file = "wakeup.txt";
      const std::string fullpath = wakeup_dir + "/" + wakeup_file;
      
      int fd = open(fullpath.c_str(), O_CREAT|O_WRONLY, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
      close(fd);

      std::cout << boost::format("%s: wrote %s") 
	% boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::universal_time())
	% fullpath.c_str() << std::endl;
    }

    // {
    //   const std::string wakeup_dir = "/tmp/wakeup_tx3";
    //   const std::string wakeup_file = "wakeup.txt";
      
    //   bool is_created = false;
      
    //   const std::string fullpath = wakeup_dir + "/" + wakeup_file;
    //   int fd = inotify_init();
    //   int watch = inotify_add_watch(fd, wakeup_dir.c_str(), IN_MODIFY|IN_CREATE|IN_MOVED_TO);

    //   if (access(fullpath.c_str(), F_OK) == 0) {
    // 	is_created = true;
    //   }

    //   char buf[1024*(sizeof(inotify_event)+16)];
    //   ssize_t len;

    //   while (!is_created) {
    // 	len = read(fd, buf, sizeof(buf));
    // 	if (len < 0) {
    // 	  break;
    // 	}
    // 	inotify_event *event;
    // 	for (size_t i = 0; i < static_cast<size_t>(len); i += sizeof(inotify_event)+event->len) {
    // 	  event = reinterpret_cast<inotify_event *>(&buf[i]);
    // 	  if ((event->len > 0) && (wakeup_file == event->name)) {
    // 	    is_created = true;
    // 	    break;
    // 	  }
    // 	}
    //   }

    //   inotify_rm_watch(fd, watch);
    //   close(fd);

    //   std::cout << boost::format("%s: %s triggered") 
    // 	% boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::universal_time())
    // 	% fullpath.c_str() << std::endl;
    // }

    {
      double txtime = 0;
      std::vector <std::vector < std::complex< float > > > buffs(tx_num_channels);

      std::vector<std::complex<float> > tmpbuff(spb);

      for (size_t i = 0; i < buffs.size(); ++i) {
	std::string fname = file + "_" + boost::lexical_cast<std::string>(i) + ".dat";
	std::ifstream infile;
	infile.open(fname.c_str(), std::ifstream::binary);
	if (infile.fail()) {
	  std::cerr << "Could not open file " << fname.c_str() << std::endl;
	  exit(1);
	}

	float max_abs = -1;
	do  {
	  infile.read((char *)&tmpbuff.front(), tmpbuff.size()*sizeof(std::complex<float>));
	  size_t nread = infile.gcount()/sizeof(std::complex<float>);

	  for (size_t j = 0; j < nread; ++j) {
	    tmpbuff[j] *= float(gain);

	    if (max_abs < std::abs(tmpbuff[j].real())) {
	      max_abs = std::abs(tmpbuff[j].real());
	    }
	    if (max_abs < std::abs(tmpbuff[j].imag())) {
	      max_abs = std::abs(tmpbuff[j].imag());
	    }
	  }

	  buffs[i].insert(buffs[i].end(), tmpbuff.begin(), tmpbuff.begin() + nread);
	} while (!infile.eof());

	infile.close();

	std::cout << boost::format("Max abs in file %s is %e") % fname.c_str() % max_abs << std::endl;
      }

      txtime = buffs[0].size()*1.0/rate;
      std::cout << boost::format("Total samples per file is %u, txtime is %e") % buffs[0].size() % txtime << std::endl;

      if (delay2 < txtime) {
	std::cerr << boost::format("Delay %e is less than packet txtime %e") % delay2 % txtime << std::endl;
	exit(1);
      }

      //send from file
      // uhd::time_spec_t t0 = usrp->get_time_now(0);
      // uhd::time_spec_t t1 = usrp->get_time_now(1);
      // double diff = t0.get_real_secs() - t1.get_real_secs();
      // std::cout << boost::format("t0=%e diff = %e") % t0.get_real_secs() % diff << std::endl;

      // omid: comment line below when things are working
      //boost::this_thread::sleep(boost::posix_time::milliseconds(700)); //allow for some sleep to ensure rx wakes up
      
      uhd::time_spec_t send_time = usrp->get_time_now() + wait2; //can play with this 0.1 number

      unsigned int count = 0;
      do {
	send_from_file(usrp, buffs, spb, rate, send_time);
	send_time = send_time + delay2/1000;
	++count;

	// uhd::time_spec_t t0 = usrp->get_time_now(0);
	// uhd::time_spec_t t1 = usrp->get_time_now(1);
	// double diff = t0.get_real_secs() - t1.get_real_secs();
	// std::cout << boost::format("t0=%e diff = %e") % t0.get_real_secs() % diff << std::endl;
      
	// double sleep_time = delay-200;
	  
	// if (sleep_time > 0) {
	//   boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_time)); // just sleep a bit to prevent from getting too far ahead
	// }

	if ((count % ((unsigned int)1e1)) == 0) {
	  std::cout << boost::format("%d... ") % (count) << std::flush;
	}

      } while ((count != nrepeat2) && (!stop_signal_called));
    }

    {
      uhd::async_metadata_t async_md;
      std::cout << boost::format("async md: ");
      while (usrp->get_device()->recv_async_msg(async_md, 3.0)) {
      	std::cout << boost::format("%x... ") % async_md.event_code;
      }
      std::cout << std::endl;
    }

    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return 0;
}
