#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/tokenizer.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <stdlib.h>

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <cmath>
#include <csignal>

namespace po = boost::program_options;

static bool stop_signal_called = false;
static const unsigned int SAMPLES_PER_BUFF = 10000;

void sig_int_handler(int){stop_signal_called = true;}

// send_from_file(usrp, tx_fname_prefix, tx_gain, nrepeat, delay) 

void send_from_file(uhd::usrp::multi_usrp::sptr usrp,
		    const std::string &tx_fname_prefix, 
		    double tx_gain,
		    unsigned int nrepeat,
		    double delay) {
  
  size_t num_tx_channels = usrp->get_tx_num_channels();
  std::vector<std::vector<std::complex<float> > > buffs(num_tx_channels);

  std::vector<std::complex<float> > tmpbuff(SAMPLES_PER_BUFF);

  for (size_t i = 0; i < buffs.size(); ++i) {
    std::string fname = tx_fname_prefix + "_" + boost::lexical_cast<std::string>(i) + ".dat";
    std::ifstream infile;
    infile.open(fname.c_str(), std::ifstream::binary);
    if (infile.fail()) {
      std::cerr << "Could not open file " << fname.c_str() << std::endl;
      exit(1);
    }

    float max_abs = -1;
    do {
      infile.read((char *)(&tmpbuff.front()), tmpbuff.size()*sizeof(std::complex<float>));
      size_t nread = infile.gcount()/sizeof(std::complex<float>);

      for (size_t j = 0; j < nread; ++j) {
	//tmpbuff[j] *= float(tx_gain);

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

  std::vector<std::complex<float> *> buff_ptrs;
  for (size_t i = 0; i < buffs.size(); ++i) {
    buff_ptrs.push_back(&buffs[i].front());
  }

  uhd::time_spec_t send_time = usrp->get_time_now() + 0.1;
  uhd::tx_metadata_t md;
  md.start_of_burst = md.end_of_burst = true;
  md.has_time_spec = true;

  unsigned int count = 0;
  do {

    md.time_spec = send_time;
    unsigned int nsent = usrp->get_device()->send(buff_ptrs, buffs[0].size(), md,
						  uhd::io_type_t::COMPLEX_FLOAT32,
						  uhd::device::SEND_MODE_FULL_BUFF,
						  3.0);
    if (nsent != buffs[0].size()) {
      std::cerr << boost::format("nsent %u is less than num_tx_samps %u") % nsent % buffs[0].size() << std::endl;
      exit(1);
    }

    send_time = send_time + delay/1000;
    ++count;

    if ((count % ((unsigned int)1e1)) == 0) {
      std::cout << boost::format("%d... ") % (count) << std::flush;
    }

  } while ((count != nrepeat) && (!stop_signal_called));

}

// recv_to_file(usrp, rx_fname_prefix, nsamples);

void recv_to_file(uhd::usrp::multi_usrp::sptr usrp, 
		  const std::string &rx_fname_prefix, 
		  unsigned int nsamples) {
  uhd::rx_metadata_t md;
  std::vector<std::complex<float> > buff(SAMPLES_PER_BUFF);

  size_t num_rx_channels = usrp->get_rx_num_channels();
  std::vector<std::vector<std::complex<float> > > buffs(num_rx_channels, std::vector<std::complex<float> >(SAMPLES_PER_BUFF));

  std::vector<std::ofstream *> ofstreams(buffs.size());

  std::string mdname = rx_fname_prefix + "_md.dat";
  std::ofstream mdstream;
  mdstream.open(mdname.c_str());

  std::vector<std::complex<float> *> buff_ptrs;
  for (unsigned int i = 0; i < buffs.size(); ++i) {
    buff_ptrs.push_back(&buffs[i].front());
    std::string ofname = rx_fname_prefix + "_" + boost::lexical_cast<std::string>(i) + ".dat";
    ofstreams[i] = new std::ofstream;
    ofstreams[i]->open(ofname.c_str(), std::ofstream::binary);
  }

  //setup streaming
  uhd::stream_cmd_t stream_cmd((nsamples == 0)?
			       uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
			       uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
			       );
  stream_cmd.num_samps = nsamples;
  stream_cmd.stream_now = false;
  uhd::time_spec_t start_time_md = usrp->get_time_now() + 0.1;
  
  stream_cmd.time_spec = start_time_md;
  mdstream << boost::lexical_cast<std::string>(start_time_md.get_full_secs());
  mdstream << "\n";
  mdstream << boost::lexical_cast<std::string>(start_time_md.get_frac_secs());
  mdstream.close();

  usrp->issue_stream_cmd(stream_cmd);

  size_t samps_left = nsamples;
  
  bool overflow_message = true;
  while (not stop_signal_called) {

    size_t req_num_samps = SAMPLES_PER_BUFF;
    if (nsamples != 0) {
      req_num_samps = ((samps_left > SAMPLES_PER_BUFF) ? SAMPLES_PER_BUFF : samps_left);
    }

    //std::cout << boost::format("calling recv req_num_samps=%u") % req_num_samps << std::endl;
    size_t num_rx_samps = usrp->get_device()->recv(buff_ptrs, req_num_samps, md, uhd::io_type_t::COMPLEX_FLOAT32, uhd::device::RECV_MODE_FULL_BUFF, 3.0);
    //std::cout << boost::format("recv num_rx_samps=%u, md error code = %u") % num_rx_samps % md.error_code << std::endl;

    samps_left -= num_rx_samps;

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
				   ) % (usrp->get_rx_rate()*sizeof(std::complex<float>)/1e6);
      }
      continue;
    }
    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
      throw std::runtime_error(str(boost::format(
						 "Unexpected error code 0x%x"
						 ) % md.error_code));
    }
    for (unsigned int i = 0; i < ofstreams.size(); ++i) {
      ofstreams[i]->write((const char *)(&buffs[i].front()), num_rx_samps*sizeof(std::complex<float>));
    }

    if ((nsamples != 0) && (samps_left == 0)) {
      break;
    }
  }

  for (unsigned int i = 0; i < ofstreams.size(); ++i) {
    ofstreams[i]->close();
  }
}

void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

int UHD_SAFE_MAIN(int argc, char *argv[]) {
  
  uhd::set_thread_priority_safe();

  //variables to be set by po
  // std::string args, file, type, ant, subdev, ref, time_diff_file, multifreq;
  // size_t spb, nrepeat1, nrepeat2;
  // double rate, freq, gain, bw, delay1, delay2, wait1, wait2;

  std::string args, ant, ref, multifreq;
  double rate, freq, tx_gain, rx_gain;

  //setup the program options
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "help message")
    ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args files")
    ("rate", po::value<double>(&rate)->default_value(1e6), "rate of outgoing samples")
    ("freq", po::value<double>(&freq)->default_value(2.45e9), "RF center frequency in Hz")
    ("multifreq", po::value<std::string>(&multifreq)->default_value(""), "frequencies in args order")
    ("tx_gain", po::value<double>(&tx_gain)->default_value(1.0), "TX gain for the RF chain")
    ("rx_gain", po::value<double>(&rx_gain)->default_value(35.0), "RX gain for the RF chain")
    ("ant", po::value<std::string>(&ant)->default_value("TX/RX"), "daughterboard antenna selection")
    ("ref", po::value<std::string>(&ref)->default_value("EXTERNAL"), "waveform type (INTERNAL, EXTERNAL, MIMO)")
    ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  //print the help message
  if (vm.count("help")){
    std::cout << boost::format("UHD TX samples from file %s") % desc << std::endl;
    return ~0;
  }

  std::signal(SIGINT, &sig_int_handler);
  std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

  //create a usrp device
  std::cout << std::endl;
  std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
  uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

  unsigned int tx_num_channels = usrp->get_tx_num_channels();
  unsigned int rx_num_channels = usrp->get_rx_num_channels();

  //parse multifreq
  std::vector<std::string> vectstr_multifreq = split(multifreq, ',');
  bool use_multifreq = (vectstr_multifreq.size() != 1);
  std::vector<double> vect_multifreq;

  for(int mfi = 0; mfi < vectstr_multifreq.size(); mfi++) {
    vect_multifreq.push_back(atof(vectstr_multifreq[mfi].c_str()));
  }

  // read h files
  // std::vector<std::vector<std::complex<float> > > h_freq(
  
  //Lock mboard clocks
  if (ref == "MIMO") {
    uhd::clock_config_t clock_config;
    clock_config.ref_source = uhd::clock_config_t::REF_MIMO;
    clock_config.pps_source = uhd::clock_config_t::PPS_MIMO;
    for (size_t i = 0; i < usrp->get_num_mboards(); ++i) {
      usrp->set_clock_config(clock_config, i);
    }
  } else if (ref == "EXTERNAL") {
    for (size_t i = 0; i < usrp->get_num_mboards(); ++i) {
      usrp->set_clock_config(uhd::clock_config_t::external(), i);
    }
  } else if (ref == "INTERNAL") {
    for (size_t i = 0; i < usrp->get_num_mboards(); ++i) {
      usrp->set_clock_config(uhd::clock_config_t::internal(), i);
    }
  }

  // //always select the subdevice first, the channel mapping affects the other settings
  // if (vm.count("subdev")) {
  //   for (size_t i = 0; i < usrp->get_num_mboards(); ++i) {
  // 	usrp->set_tx_subdev_spec(subdev, i);
  //   }
  // }

  std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

  //set the sample rate
  if (not vm.count("rate")){
    std::cerr << "Please specify the sample rate with --rate" << std::endl;
    return ~0;
  }
  for (size_t i = 0; i < tx_num_channels; ++i) {
    std::cout << boost::format("Setting TX Rate %u: %f Msps...") % i % (rate/1e6) << std::endl;
    usrp->set_tx_rate(rate, i);
    std::cout << boost::format("Actual TX Rate %u: %f Msps...") % i % (usrp->get_tx_rate(i)/1e6) << std::endl << std::endl;
  }
  for (size_t i = 0; i < rx_num_channels; ++i) {
    std::cout << boost::format("Setting RX Rate %u: %f Msps...") % i % (rate/1e6) << std::endl;
    usrp->set_rx_rate(rate, i);
    std::cout << boost::format("Actual RX Rate %u: %f Msps...") % i % (usrp->get_rx_rate(i)/1e6) << std::endl << std::endl;
  }

  //set the center frequency
  if (not vm.count("freq")){
    std::cerr << "Please specify the center frequency with --freq" << std::endl;
    return ~0;
  }
  for (size_t i = 0; i < tx_num_channels; ++i) {
    if (use_multifreq) {
	std::cout << boost::format("Setting TX Freq %u: %f MHz...") % i % (vect_multifreq[i]/1e6) << std::endl;
	usrp->set_tx_freq(vect_multifreq[i], i);
	std::cout << boost::format("Actual TX Freq %u: %f MHz...") % i % (usrp->get_tx_freq(i)/1e6) << std::endl << std::endl;
    }
    else {
	std::cout << boost::format("Setting TX Freq %u: %f MHz...") % i % (freq/1e6) << std::endl;
	usrp->set_tx_freq(freq, i);
	std::cout << boost::format("Actual TX Freq %u: %f MHz...") % i % (usrp->get_tx_freq(i)/1e6) << std::endl << std::endl;
    }
  }
  for (size_t i = 0; i < rx_num_channels; ++i) {
    if (use_multifreq) {
        std::cout << boost::format("Setting RX Freq %u: %f MHz...") % i % (vect_multifreq[i]/1e6) << std::endl;
        usrp->set_rx_freq(vect_multifreq[i], i);
        std::cout << boost::format("Actual RX Freq %u: %f MHz...") % i % (usrp->get_rx_freq(i)/1e6) << std::endl << std::endl;
    }
    else {
        std::cout << boost::format("Setting RX Freq %u: %f MHz...") % i % (freq/1e6) << std::endl;
        usrp->set_rx_freq(freq, i);
        std::cout << boost::format("Actual RX Freq %u: %f MHz...") % i % (usrp->get_rx_freq(i)/1e6) << std::endl << std::endl;
    }
  }

  //set the rf gain
  if (vm.count("tx_gain")){
    for (size_t i = 0; i < tx_num_channels; ++i) {
      std::cout << boost::format("Setting TX Gain %u: %f dB...") % i % tx_gain << std::endl;
      usrp->set_tx_gain(tx_gain, i);
      std::cout << boost::format("Actual TX Gain %u: %f dB...") % i % usrp->get_tx_gain(i) << std::endl << std::endl;
    }
  }

  if (vm.count("rx_gain")){
    for (size_t i = 0; i < rx_num_channels; ++i) {
      std::cout << boost::format("Setting RX Gain %u: %f dB...") % i % rx_gain << std::endl;
      usrp->set_rx_gain(rx_gain, i);
      std::cout << boost::format("Actual RX Gain %u: %f dB...") % i % usrp->get_rx_gain(i) << std::endl << std::endl;
    }
  }

  //if (!vm.count("tx_gain")) {
    //tx_gain = 1;
  //}


  //set the antenna
  if (vm.count("ant")) {
    for (size_t i = 0; i < tx_num_channels; ++i) {
      usrp->set_tx_antenna(ant, i);
    }

    for (size_t i = 0; i < rx_num_channels; ++i) {
      usrp->set_rx_antenna(ant, i);
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

  for (size_t i = 0; i < usrp->get_rx_num_channels(); ++i) {
    std::vector<std::string> sensor_names;
    sensor_names = usrp->get_rx_sensor_names(i);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
      uhd::sensor_value_t lo_locked = usrp->get_rx_sensor("lo_locked", i);
      std::cout << boost::format("Checking RX %u: %s ...") % i % lo_locked.to_pp_string() << std::endl;
      UHD_ASSERT_THROW(lo_locked.to_bool());
    }
  }

  for (size_t i = 0; i < usrp->get_num_mboards(); ++i) {
    std::vector<std::string> sensor_names;
    sensor_names = usrp->get_mboard_sensor_names(i);
    if ((ref == "EXTERNAL") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end())) {
      uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked", i);
      std::cout << boost::format("Checking TX/RX %u: %s ...") % i % ref_locked.to_pp_string() << std::endl;
      UHD_ASSERT_THROW(ref_locked.to_bool());
    }
  }

  usrp->set_time_unknown_pps(0.0);
  boost::this_thread::sleep(boost::posix_time::seconds(1)); //allow for some setup time

  while (true) {
    std::string cmd;

    stop_signal_called = false;
    std::cout << boost::format("%s> ") % boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::universal_time());
    std::getline(std::cin, cmd);

    // 4 types of commands:
    // tx <tx_file_name_prefix> <nrepeats> <delay>
    // rx <rx_file_name_prefix> <nsamples>
    // sleep <sleep_time>
    // quit

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep(" ");
    tokenizer tokens(cmd, sep);
    tokenizer::iterator tok_it = tokens.begin();
    std::string opcode = *tok_it;
    
    if (opcode == "tx") {
      ++tok_it; std::string tx_fname_prefix = *tok_it;
      ++tok_it; unsigned int nrepeat = boost::lexical_cast<unsigned int>(*tok_it);
      ++tok_it; double delay = boost::lexical_cast<double>(*tok_it);

      send_from_file(usrp, tx_fname_prefix, tx_gain, nrepeat, delay);
    } else if (opcode == "rx") {
      ++tok_it; std::string rx_fname_prefix = *tok_it;
      ++tok_it; unsigned int nsamples = boost::lexical_cast<unsigned int>(*tok_it);

      recv_to_file(usrp, rx_fname_prefix, nsamples);
    } else if (opcode == "sleep") {
      ++tok_it; unsigned int sleep_time_ms = boost::lexical_cast<unsigned int>(*tok_it);

      boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_time_ms)); //allow for some setup time
    } else if (opcode == "quit") {
      return 0;
    } else {
      std::cerr << boost::format("Unknown command %s") % cmd << std::endl;
    }
  }

  return true;
}

 
