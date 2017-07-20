#include <uhd/usrp/mmimo/packet_rx.hpp>
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <fstream>
#include <csignal>

using namespace std;
using namespace boost;
using namespace uhd;
using namespace uhd::mmimo;

bool packet_rx::_stop_signal_called = false;

inline int packet_rx::normal_to_fft(int index, int nfft) {
  if (index < 0) {
    index = index + nfft;
  }
  return index;
}

packet_rx::packet_rx_request_t::packet_rx_request_t(packet_rx::rx_mode_t this_mode, unsigned int this_num_symbols, bool this_has_start_time, uhd::time_spec_t &this_start_time)
  : mode(this_mode), has_start_time(this_has_start_time), start_time(this_start_time), num_symbols(this_num_symbols), has_precomputed_cfo(false) {
  if ((this_mode != RX_MODE_CFO) && (this_mode != RX_MODE_BEACON)) {
    exit(1);
  }
}


packet_rx::packet_rx_request_t::packet_rx_request_t(packet_rx::rx_mode_t this_mode, unsigned int this_num_symbols, std::string &this_logfile, 
						    bool this_has_start_time, uhd::time_spec_t &this_start_time)
  : mode(this_mode), has_start_time(this_has_start_time), start_time(this_start_time), num_symbols(this_num_symbols), logfile(this_logfile), has_precomputed_cfo(false) {
  if ((this_mode != RX_MODE_LOG_ALL) && (this_mode != RX_MODE_DETECT_AND_LOG)) {
    cout<<"packet_rx_request_t: In wrong mode "<<this_mode<<"!"<<endl;
    exit(1);
  }

}


packet_rx::packet_rx_request_t::packet_rx_request_t(packet_rx::rx_mode_t this_mode, bool this_has_start_time, uhd::time_spec_t &this_start_time, 
						    unsigned int this_num_symbols, uhd::time_spec_t &this_h_measurement_time,
						    bool this_has_precomputed_cfo, double this_precomputed_cfo, 
						    unsigned int this_num_h_chunks, unsigned int this_num_h_syms_per_chunk,
						    double this_h_target_offset)
  : mode(this_mode), has_start_time(this_has_start_time), start_time(this_start_time), num_symbols(this_num_symbols), h_measurement_time(this_h_measurement_time), 
    has_precomputed_cfo(this_has_precomputed_cfo), precomputed_cfo(this_precomputed_cfo), num_h_chunks(this_num_h_chunks), num_h_syms_per_chunk(this_num_h_syms_per_chunk),
    h_target_offset(this_h_target_offset) {
  if ((this_mode == RX_MODE_NULL) || (this_mode == RX_MODE_LOG_ALL) || (this_mode == RX_MODE_DETECT_AND_LOG) || (this_mode == RX_MODE_CFO)) {
    exit(1);
  }
}

packet_rx::packet_rx_mem_t::packet_rx_mem_t(const config_params &conf, const packet_rx_request_t &req) {
  energy_samples.reserve(2*conf.ofdm_config.nfft);

  temp_buff.reserve(2);
  for (unsigned int i = 0; i < 2; i++) {
    temp_buff[i] = new std::vector<std::complex<float> >;
    temp_buff[i]->reserve(conf.ofdm_config.nfft);
  }

  switch (req.mode) {
  case RX_MODE_DETECT_AND_LOG:
    log_buff.reserve(conf.ofdm_config.nfft*req.num_symbols); 
    break;

  case RX_MODE_DETECT_START:
  case RX_MODE_HBASE:
    {
      measure_tx_h.push_back(true);
      uhd::mmimo::fftw::vec_sptr v = uhd::mmimo::fftw::vec_sptr(new uhd::mmimo::fftw::vec(req.num_h_chunks));
      for (unsigned int j = 0; j < v->size(); ++j) {
	(*v)[j] = uhd::mmimo::fftw::sptr(new uhd::mmimo::fftw(conf.ofdm_config.nfft, 1, FFTW_FORWARD, FFTW_MEASURE));
      }
      h_samples.push_back(v);
      packet_rx_mem_t::chunk_sym_sptr w = packet_rx_mem_t::chunk_sym_sptr(new std::vector<std::vector<std::complex<float> > >(req.num_h_chunks));
      chunk_channels.push_back(w);
    }
    break;

  case RX_MODE_HIJ:
    {
      for (unsigned int i = 0; i < conf.network_config.num_txs; i++) {
	measure_tx_h.push_back(true);
	uhd::mmimo::fftw::vec_sptr v = uhd::mmimo::fftw::vec_sptr(new uhd::mmimo::fftw::vec(req.num_h_chunks));
	for (unsigned int j = 0; j < v->size(); ++j) { // num chunks
	  (*v)[j] = uhd::mmimo::fftw::sptr(new uhd::mmimo::fftw(conf.ofdm_config.nfft, 1, FFTW_FORWARD, FFTW_MEASURE));
	}
	h_samples.push_back(v);
	packet_rx_mem_t::chunk_sym_sptr w = packet_rx_mem_t::chunk_sym_sptr(new std::vector<std::vector<std::complex<float> > >(req.num_h_chunks));
	chunk_channels.push_back(w);
	std::cout << boost::format("pushed into chunk_channels") << std::endl;
      }
    }
    break;

  case RX_MODE_HBASE_CURR:
    {
      measure_tx_h.push_back(true);
      uhd::mmimo::fftw::vec_sptr v = uhd::mmimo::fftw::vec_sptr(new uhd::mmimo::fftw::vec(req.num_h_chunks));
      for (unsigned int j = 0; j < v->size(); ++j) {
	(*v)[j] = uhd::mmimo::fftw::sptr(new uhd::mmimo::fftw(conf.ofdm_config.nfft, 1, FFTW_FORWARD, FFTW_MEASURE));
      }
      h_samples.push_back(v);
      packet_rx_mem_t::chunk_sym_sptr w = packet_rx_mem_t::chunk_sym_sptr(new std::vector<std::vector<std::complex<float> > >(req.num_h_chunks));
      chunk_channels.push_back(w);
      for (unsigned int i = 1; i < conf.network_config.num_txs; i++) {
	measure_tx_h.push_back(false);
	uhd::mmimo::fftw::vec_sptr v = uhd::mmimo::fftw::vec_sptr(); // empty vector
	h_samples.push_back(v);
	chunk_channels.push_back(packet_rx_mem_t::chunk_sym_sptr());
      }
    }
    break;

  default:
    break;

  }

}

packet_rx::packet_rx(const config_params &conf, uhd::usrp::multi_usrp *usrp, packet_rx::packet_rx_request_t &req, packet_rx::packet_rx_mem_t &mem)
  : _conf(conf), _usrp(usrp), _req(req), _mem(mem), 
    _sample_buff_size(0), _sample_buff_index(0), 
    _sample_buff_start_timestamp(0.0), 
    _num_streamed_samples(0),
    _num_remaining_samples(0),
    _cfo(0),
    _pkt_recv_time(0)
{
  _num_samples_default = _usrp->get_device()->get_max_recv_samps_per_packet();
  //_sample_buff.reserve(_num_samples_default); // Swarun

  _sample_buff_arr = std::vector<std::vector<std::complex<float> > >(_usrp->get_rx_num_channels(), std::vector<std::complex<float> >(_num_samples_default));
  
  for(unsigned int i=0; i<_usrp->get_rx_num_channels(); ++i) {
    _sample_buff.push_back(&_sample_buff_arr[i].front());
  }

  _pi = boost::math::constants::pi<double>();

  uhd::stream_cmd_t stream_cmd((_req.tot_samples != 0) ? 
			       uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE: 
			       uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
  stream_cmd.num_samps = _req.tot_samples;
  stream_cmd.stream_now = (!_req.has_start_time);
  stream_cmd.time_spec = _req.start_time;
  _num_remaining_samples = _req.tot_samples;

  _sample_buff_recv_timeout = 0.1;
  if (_req.has_start_time) {
    _sample_buff_recv_timeout += _req.start_time.get_real_secs();
  }

  _usrp->issue_stream_cmd(stream_cmd);
}


bool packet_rx::get_next_sample(std::complex<float> &next_sample, uhd::time_spec_t &timestamp) {
  static int count = 0;
  static int count2 = 0;
  
  if ((_sample_buff_size == 0) || (_sample_buff_index == _sample_buff_size)) {
    uhd::rx_metadata_t md;
    size_t num_samples = _num_samples_default;
    if (_req.tot_samples != 0) {
      if (num_samples > _num_remaining_samples) {
	num_samples = _num_remaining_samples;
      }
    }

    if (num_samples == 0) {
      return false;
    }

    _sample_buff_size = _usrp->get_device()->recv(_sample_buff, num_samples, md, uhd::io_type_t::COMPLEX_FLOAT32, 
						  uhd::device::RECV_MODE_FULL_BUFF, _sample_buff_recv_timeout);

    _num_remaining_samples -= _sample_buff_size;
    _num_streamed_samples += _sample_buff_size;

    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
      std::cerr << boost::format("Error code %u (sample_buff_size %u, num_samples %u) [count=%d] [count2=%d]") % (unsigned int)md.error_code % _sample_buff_size % num_samples % count % count2 << std::endl;      
      exit(1);
    }
    //std::cerr << boost::format("sample_buff_size: %u") % _sample_buff_size << std::endl;

    _sample_buff_recv_timeout = 0.1; // future recvs
    _sample_buff_start_timestamp = md.time_spec;
    _sample_buff_index = 0;
    count2++;
  }
  count++;
    
  timestamp = _sample_buff_start_timestamp + uhd::time_spec_t(_sample_buff_index*1.0/_conf.usrp_config.rate);

  next_sample = _sample_buff[0][_sample_buff_index++]; // Only from 1st antenna
  return true;
}

void packet_rx::process() {

  enum _rx_state_t {
    RX_STATE_WAIT = 0, 
    RX_STATE_ENERGY_INIT = 1, 
    RX_STATE_ENERGY = 2, 
    RX_STATE_DELAY_CORRELATE = 3, 
    RX_STATE_LOG = 4,
    RX_STATE_CFO_INIT = 5, 
    RX_STATE_CFO = 6, 
    RX_STATE_SKIP = 7,
    RX_STATE_MEASURE_H = 8,
    RX_STATE_DONE = 9
  };

  _rx_state_t state = RX_STATE_WAIT, state_after_skip = RX_STATE_WAIT;

  unsigned int initial_wait_counter = 0;
  unsigned int counter = 0;
  unsigned int num_samples_to_skip = 0;

  float a=0, b=0;
  std::complex<double> corr = 0.0;

  unsigned int start_index = 0, b_start_index = 0;
  unsigned int num_syms = 0, num_chunks = 0, num_tx = 0;

  double cfo = 0;

  if (_req.mode == RX_MODE_LOG_ALL) {
    if (_req.num_symbols == 0) {
      std::signal(SIGINT, &uhd::mmimo::packet_rx::sig_int_handler);
    }

    cout<<"packet_rx_request_t: In log_all mode "<<endl;

    std::vector<std::vector<std::complex<float> > > buff(_usrp->get_rx_num_channels(), std::vector<std::complex<float> >(10000));

    // std::vector<std::complex<float> > buff(10000); // 1 stream

    uhd::rx_metadata_t md;
    std::ofstream outfile(_req.logfile.c_str(), std::ofstream::binary);

    bool done = false;
    unsigned int tot_rx_samps = 0;
    unsigned int num_rx_samps_since_last_msg = 0;

    while (not done) {
      size_t num_rx_samps = _usrp->get_device()->recv(buff, buff[0].size(), md, uhd::io_type_t::COMPLEX_FLOAT32, 
						      uhd::device::RECV_MODE_FULL_BUFF, _sample_buff_recv_timeout);
      _sample_buff_recv_timeout = 0.1;

      if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
      if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
	throw std::runtime_error(str(boost::format("Unexpected error code 0x%x") % md.error_code));
      }
      tot_rx_samps += num_rx_samps;
      num_rx_samps_since_last_msg += num_rx_samps;

      // Log just one stream for now
      outfile.write((const char*)&buff[0].front(), num_rx_samps*sizeof(std::complex<float>));

      if (num_rx_samps_since_last_msg >= (0.5e6)) {
	std::cerr << boost::format("%0.4fMS... ") % (tot_rx_samps/(1e6));
	num_rx_samps_since_last_msg = 0;
      }

      done = ((_stop_signal_called) || ((_req.num_symbols != 0) && (tot_rx_samps >= (_req.num_symbols * _conf.ofdm_config.nfft))));
    }

    std::cerr << std::endl;
    outfile.close();
    exit(0);
  }
    
  std::ofstream outfile("log.dat", std::ofstream::binary);

  int global_counter = 0;
  
  _rx_state_t prev_state = RX_STATE_WAIT;
  while (state != RX_STATE_DONE) {

    if (state != prev_state) {
      std::cerr << boost::format("Curr State: %u, Prev State: %u") % (unsigned int)state % (unsigned int)prev_state << std::endl;
    }
    prev_state = state;
    
    uhd::time_spec_t next_sample_time;
    std::complex<float> next_sample; 
    if (!get_next_sample(next_sample, next_sample_time)) {
      state = RX_STATE_DONE;
    }

    outfile.write((const char*)&next_sample, sizeof(std::complex<float>));

    // std::cerr << boost::format("state: %u, sample_buff_index: %u, next_sample_time: %0.9e, sample: %e + j%e") % (unsigned int)state % _sample_buff_index % next_sample_time.get_real_secs() % next_sample.real() % next_sample.imag() << std::endl;

    if(global_counter%10000000==0) {
      fprintf(stderr, "%0.9e %0.9e\n", next_sample_time.get_real_secs(), _usrp->get_time_now().get_real_secs());
    }
    global_counter++;

    
    switch (state) {
    case RX_STATE_WAIT:
      {
	if (initial_wait_counter < INITIAL_WAIT_NUM_SAMPLES) {
	  ++initial_wait_counter;
	}
	if (initial_wait_counter == INITIAL_WAIT_NUM_SAMPLES) {
	  state = RX_STATE_ENERGY_INIT;
	  start_index = b_start_index = counter = a = b = 0;
	}
      }
      break;

    case RX_STATE_ENERGY_INIT:
      {
	if (counter < _conf.ofdm_config.nfft) {
	  _mem.energy_samples[counter] = std::norm(next_sample);
	  a += _mem.energy_samples[counter];
	  ++counter;
	} else if (counter < (2*_conf.ofdm_config.nfft)) {
	  _mem.energy_samples[counter] = std::norm(next_sample);
	  b += _mem.energy_samples[counter];
	  ++counter;
	}
	if (counter == (2*_conf.ofdm_config.nfft)) {
	  state = RX_STATE_ENERGY;
	  start_index = 0;
	  b_start_index = _conf.ofdm_config.nfft;
	}
      }
      break;

    case RX_STATE_ENERGY:
      {	
	a -= _mem.energy_samples[start_index];
	a += _mem.energy_samples[b_start_index];

	float next_sample_norm = std::norm(next_sample);
	b -= _mem.energy_samples[b_start_index];
	b += next_sample_norm;

	_mem.energy_samples[start_index] = next_sample_norm;

	++start_index;
	++b_start_index;

	if (start_index == (2*_conf.ofdm_config.nfft)) {
	  start_index = 0;
	}

	if (b_start_index == (2*_conf.ofdm_config.nfft)) {
	  b_start_index = 0;
	}

	if (b >= (_conf.ofdm_config.sliding_window_thresh/2*a)) {
	  std::cerr << boost::format("WARN: Energy Ratio: %0.9e (%0.9e/%0.9e)") % (b/a) % b % a << std::endl;
	  //state = RX_STATE_DELAY_CORRELATE;
	  //counter = a = b = 0;
	}	

	if (b >= (_conf.ofdm_config.sliding_window_thresh*a)) {
	  std::cerr << boost::format("Energy Ratio: %0.9e (%0.9e/%0.9e); global_counter=%d") % (b/a) % b % a % global_counter << std::endl;
	  detection_time = next_sample_time; // Detected at this point in time (perhaps should shift by nfft)
	  state = RX_STATE_DELAY_CORRELATE;
	  counter = a = b = 0;
	}	
      }
      break;

    case RX_STATE_DELAY_CORRELATE:
      {
	if (counter < _conf.ofdm_config.nfft) {
	  (*_mem.temp_buff[0])[counter] = next_sample;
	  ++counter;
	} else if (counter < (2*_conf.ofdm_config.nfft)) {
	  corr += (*_mem.temp_buff[0])[counter-_conf.ofdm_config.nfft]*std::conj(next_sample);
	  b += std::norm(next_sample);
	  ++counter;
	}
	if (counter == (2*_conf.ofdm_config.nfft)) {
	  float delay_corr_denom = b*b;
	  float delay_corr_numer = std::norm(corr);	  
	  //detection_time = _usrp->get_time_now();
	  std::cerr << boost::format("Delay Correlate Threshold: %0.9e (%0.9e/%0.9e) [>= %d] ; global_counter=%d") % (delay_corr_numer/delay_corr_denom) % delay_corr_numer % delay_corr_denom % _conf.ofdm_config.delay_correlate_thresh % global_counter << std::endl;
	  if (delay_corr_numer >= (_conf.ofdm_config.delay_correlate_thresh*delay_corr_denom)) {
	    std::cerr << boost::format("Cond true (mode=%d)\n") % _req.mode;
	    switch (_req.mode) {
	    case RX_MODE_BEACON:
	      std::cerr << boost::format("Done\n");
	      cout<< boost::format("Done at time: det_time = %0.9e, next_sample_time= %0.9e, now = %0.9e sec") % (detection_time.get_real_secs()) % (next_sample_time.get_real_secs()) % (_usrp->get_time_now().get_real_secs())  << std::endl;
	      state = RX_STATE_DONE;
	      break;
	    case RX_MODE_HBASE:
	      {
		// Ignore CFO compensation for now
		counter = 0;
		if (_conf.ofdm_config.detect_jump) {
		  state = RX_STATE_SKIP; 
		  num_samples_to_skip = _conf.ofdm_config.detect_jump;
		  state_after_skip = RX_STATE_MEASURE_H; 
		} else {
		  state = RX_STATE_MEASURE_H;
		}
		num_syms = num_chunks = counter = num_tx = 0;
		cfo = (_req.has_precomputed_cfo ? _req.precomputed_cfo : 0);
	      }
	      break;
	    case RX_MODE_DETECT_START:
	    case RX_MODE_CFO: 
	    case RX_MODE_HIJ:
	    case RX_MODE_HBASE_CURR:
	      // state = RX_STATE_SKIP; 
	      // num_samples_to_skip = 2*_conf.ofdm_config.nfft; // skip the 2 low power buffer symbols
	      // state_after_skip = RX_STATE_CFO_INIT; 
	      state = RX_STATE_CFO_INIT;
	      break;
	    case RX_MODE_DETECT_AND_LOG: 
	      // state = RX_STATE_SKIP;
	      // num_samples_to_skip = 2*_conf.ofdm_config.nfft;
	      // state_after_skip = RX_STATE_LOG;
	      state = RX_STATE_LOG;
	      break;
	    default: 
	      break;
	    }
	  } else {
	    state = RX_STATE_ENERGY_INIT;
	  }
	  counter = start_index = a = b = 0;
	  b_start_index = _conf.ofdm_config.nfft;
	  corr = 0;
	}
      }
      break;

    case RX_STATE_SKIP:
      ++counter;
      if (counter == num_samples_to_skip) {
	counter = num_samples_to_skip = 0;
	state = state_after_skip;
      }
      break;

    case RX_STATE_LOG:
      {
	if (counter < _mem.log_buff.capacity()) {
	  _mem.log_buff.push_back(next_sample);
	  ++counter;
	}
	if (counter == _mem.log_buff.capacity()) {
	  state = RX_STATE_DONE;
	}
      }
      break;
      
    case RX_STATE_CFO_INIT:
      {
	if (counter < _conf.ofdm_config.nfft) {
	  (*_mem.temp_buff[start_index])[counter] = next_sample;
	  ++counter;
	}
	if (counter == _conf.ofdm_config.nfft) {
	  start_index = 1;
	  counter = 0;
	  num_syms = 1;
	  corr = 0;
	  state = RX_STATE_CFO;
	}
      }
      break;

    case RX_STATE_CFO:
      {
	if (counter < _conf.ofdm_config.nfft) {
	  (*_mem.temp_buff[start_index])[counter] = next_sample;
	  corr += ((*_mem.temp_buff[1-start_index])[counter] * std::conj(next_sample));
	  ++counter;
	}
	if (counter == _conf.ofdm_config.nfft) {
	  start_index = 1 - start_index;
	  counter = 0;
	  _cfo += -1/(2*_pi*_conf.ofdm_config.nfft)*std::arg(corr);
	  ++num_syms;
	  // std::cerr << boost::format("num_syms: %d, corr is %e+j%e, cfo is %e") % num_syms % corr.real() % corr.imag() % (_cfo/(num_syms-1)) << std::endl;
	  corr = 0;
	  if (num_syms == _req.num_symbols) {
	    _cfo /= (_req.num_symbols-1);
	    switch (_req.mode) {
	    case RX_MODE_CFO:
	      state = RX_STATE_DONE;
	      break;
	      
	    case RX_MODE_DETECT_START:
	    case RX_MODE_HBASE:
	    case RX_MODE_HIJ:
	    case RX_MODE_HBASE_CURR:
	      {
		counter = 0;
		if (_conf.ofdm_config.detect_jump) {
		  state = RX_STATE_SKIP; 
		  num_samples_to_skip = _conf.ofdm_config.detect_jump;
		  state_after_skip = RX_STATE_MEASURE_H; 
		} else {
		  state = RX_STATE_MEASURE_H;
		}
		num_syms = num_chunks = counter = num_tx = 0;
		cfo = (_req.has_precomputed_cfo ? _req.precomputed_cfo : _cfo);
	      }
	      break;
	      
	    default:
	      break;
	    }
	  }
	}
      }
      break;
      
    case RX_STATE_MEASURE_H:
      {
	if ((num_tx < _mem.h_samples.size()) && (_mem.measure_tx_h[num_tx])) {
	  next_sample = next_sample*std::complex<float>(std::polar(1.0, -2*_pi*cfo*(num_chunks*(_req.num_h_syms_per_chunk*_conf.ofdm_config.nfft + _conf.ofdm_config.ncp)+ num_syms*_conf.ofdm_config.nfft + counter)));
	  uhd::mmimo::fftw::vec_sptr vp = _mem.h_samples[num_tx];
	  uhd::mmimo::fftw::sptr fp =  (*vp)[num_chunks];
	  std::complex<float> *cf = (std::complex<float> *)((*fp).input(0)+counter);
	  if (num_syms == 0) { // first symbol in chunk
	    *cf = next_sample;
	  } else {
	    *cf += next_sample;
	  }
	}
	++counter;
	bool end_of_symbol = false, end_of_tx = false, end_of_chunk = false, end_of_processing = false;
	if (counter == _conf.ofdm_config.nfft) {
	  end_of_symbol = true;
	  counter = 0;
	  ++num_syms; 
	  if (num_syms == _req.num_h_syms_per_chunk) { // end of chunk for this tx
	    end_of_tx = true;
	    ++num_tx;
	    num_syms = 0;
	    if (num_tx == _mem.h_samples.size()) { // end of chunk for all txs
	      end_of_chunk = true;
	      ++num_chunks;
	      num_tx = 0;
	      if (num_chunks == (*_mem.h_samples[0]).size()) {
		end_of_processing = true;
		num_chunks = 0;
		// do the processing
		std::cout << boost::format("computing h") << std::endl;
		compute_all_h();
		// std::vector<float> angles;
		// float slope, intercept;
		// unwrap_angles(angles, NULL);
		// regress_H(angles, slope, intercept, NULL);
		// float computed_offset_f = (slope) * (_conf.ofdm_config.nfft/(2*M_PI));
		// //int computed_offset = fabsf(roundf(computed_offset_f));
		// detection_time = next_sample_time - uhd::time_spec_t(computed_offset_f*1.0/_conf.usrp_config.rate);
		// float offset_time = (-computed_offset_f*1.0/_conf.usrp_config.rate);
		// std::cerr << boost::format("detection_time=%0.9e, next_sample_time=%0.9e, computed_offset_f=%0.9e, computed_offset_time=%0.9e") % detection_time.get_real_secs() % next_sample_time.get_real_secs() % computed_offset_f % offset_time << std::endl;
	      }
	    }
	  }
	}

	// std::cerr << boost::format("counter: %u, num_syms: %u, num_chunks: %u, num_tx: %u, global_counter: %d") % counter % num_syms % num_chunks % num_tx % global_counter<< std::endl;

	if (end_of_processing) {
	  std::cerr << boost::format("global_counter: %d") % global_counter<< std::endl;
	  state = RX_STATE_DONE;
	} else if (end_of_chunk || end_of_tx) {
	  num_samples_to_skip = _conf.ofdm_config.ncp;
	  if (num_samples_to_skip > 0) {
	    state = RX_STATE_SKIP; 
	    state_after_skip = RX_STATE_MEASURE_H; 
	    counter = 0;
	  }
	} else if (end_of_symbol) {
	  num_samples_to_skip = 0;
	  if (num_samples_to_skip > 0) {
	    state = RX_STATE_SKIP; 
	    state_after_skip = RX_STATE_MEASURE_H; 
	    counter = 0;
	  }
	}
      }
      break;

    default:
      break;
      
    }
    
    
  }
  cout<< boost::format("End loop: %0.9e sec") % (_usrp->get_time_now().get_real_secs()) << std::endl;
  outfile.close();

  // drain pending packets
  uhd::rx_metadata_t md;

  unsigned int num_samples = _sample_buff.size();
  if (_req.tot_samples == 0) {
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    stream_cmd.num_samps = 0;
    stream_cmd.stream_now = true;
  } else {
    if (num_samples > _num_remaining_samples) {
      num_samples = _num_remaining_samples;
    }
  }
  

  bool samples_left = true;
  if (_req.tot_samples != 0) {
    if (_num_remaining_samples == 0) {
      samples_left = false;
    }
  }

  while (samples_left) {
    size_t nrecd = _usrp->get_device()->recv(_sample_buff, num_samples, md, uhd::io_type_t::COMPLEX_FLOAT32, uhd::device::RECV_MODE_FULL_BUFF, 0.001);
    _num_remaining_samples -= nrecd;
    _num_streamed_samples += nrecd;

    if (_req.tot_samples == 0) {
      if (nrecd == 0) {
	samples_left = false;
      }
    } else {
      if (_num_remaining_samples == 0) {
	samples_left = false;
      }
    }

    if ((md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) &&
	(md.error_code != uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)) {
      std::cerr << boost::format("Error code %u ") % (unsigned int)md.error_code  << std::endl;      
      exit(1);
    }  
  }

  //cout<< boost::format("End function: %0.9e sec") % (_usrp->get_time_now().get_real_secs()) << std::endl;
  return;
}



void packet_rx::compute_all_h() {

  for (unsigned int i = 0; i < _mem.h_samples.size(); ++i) { // number of tx's
    if (!_mem.measure_tx_h[i]) {
      continue;
    }
    uhd::mmimo::fftw::vec_sptr vp = _mem.h_samples[i];
    //packet_rx_mem_t::chunk_sym_sptr v = packet_rx_mem_t::chunk_sym_sptr(new std::vector<std::vector<std::complex<float> > >(vp->size()));
    //_mem.chunk_channels.push_back(v); // push back a vector for each transmitter with size num chunks
    for (unsigned int j = 0; j < (*vp).size(); ++j) { // iterate across chunks
      uhd::mmimo::fftw::sptr fp = (*vp)[j];
      (*fp).scale(1.0/_req.num_h_syms_per_chunk);
      (*fp).execute();
      (*fp).multiply_output(_conf.data_config.h_freq[i]->input(0));
      
      for (unsigned int k = 0; k < (*fp).num_syms(); k++) { // always 1 so far
	for (unsigned int l = 0; l < (*fp).width(); l++) { // nfft
	  std::complex<float> *cf = (std::complex<float> *)(((*fp).output(k))+l);
	  packet_rx_mem_t::chunk_sym_sptr v = _mem.chunk_channels[i];
	  (*v)[j].push_back(*cf);
	  //std::cerr << boost::format("(%u,%u): %0.6e %0.6e [hdr_syms = %f+j%f]") % k % l % std::abs(*f) % std::arg(*f) % (_conf.network_config.hdr_syms[l][0]) % (_conf.network_config.hdr_syms[l][1]) << std::endl;
	}
      }

    }
  }

}


// Get channel recorderd in HBASE mode
// void packet_rx::get_channel(std::vector<std::complex<float> > &channel) {
//   channel = h_channel;
// }
  
double packet_rx::get_cfo() {
  return _cfo;
}

void packet_rx::sig_int_handler(int) {
  _stop_signal_called = true;
}


// Regress the angle of H
float packet_rx::regress_H(std::vector<float> &y, float &slope, float &intercept, float* weights) {

  unsigned int nfft = _conf.ofdm_config.nfft;
    

  std::cerr << boost::format("Regressing this (unwrapped): ") <<std::endl;
  for(unsigned int i=0; i<y.size(); ++i) {
    std::cerr << boost::format("%0.6e") % y[i] << std::endl;
  }

  double sum_x, sum_y, sum_xy, sumsq_x, sum_w;

  int *x = new int[nfft];

  slope = 0;
  intercept = 0;
  sum_x = sum_y = sum_xy = sumsq_x = sum_w = 0;

  int *array_indices = NULL;
  int n = 0;

  for (int i = -int(nfft)/2; i < int(nfft)/2; i++)  {
    int array_index = normal_to_fft(i, nfft);
    std::complex<float> *this_complex = reinterpret_cast<std::complex<float> *>(_conf.data_config.h_freq[0]->input(0)+array_index);
    if (std::abs(*this_complex) > 1e-6) {
      x[n] = i;
      ++n;
    }
  }

  for (int i = 0; i < n; i++) {
    int array_index = (array_indices ? array_indices[i] : i);
    int this_x = (x ? x[i] : i);
    float w = (weights ? weights[array_index] : 1);

    sum_w += w;
    sum_x += w*this_x;
    sumsq_x += w*this_x*this_x;
    sum_y += w*y[array_index];
    sum_xy += w*this_x*y[array_index];
  }

  if(n <= 1) {
    slope = 0; // Slope is "not defined"                                                                                                                                                                                                    
  } else {
    slope = (sum_x*sum_y - sum_w*sum_xy)/(sum_x*sum_x - sum_w*sumsq_x);
  }
  intercept = (sum_y - (slope)*sum_x)/sum_w;

  float reg_error = 0;
  for (int i = 0; i < n; i++) {
    int ai = (array_indices ? array_indices[i] : i);
    int this_x = (x ? x[i] : i);
    float w = (weights ? weights[ai] : 1);
    float this_error = (intercept) + (slope)*this_x - y[ai];
    reg_error += w*this_error*this_error;
  }
  reg_error /= n;

  std::cerr << boost::format("slope = %0.6e, intercept = %0.6e") % (slope) % (intercept) << std::endl;

  return reg_error;
}

// we assume that the slope is large 
float packet_rx::unwrap_angles(std::vector<float>& angles,  float *weights) {

  std::cerr << boost::format("Regressing this (BEFORE unwrap): ") <<std::endl;
  for(unsigned int i=0; i<h_channel.size(); ++i) {
    std::cerr << boost::format("%0.6e %0.6e") % std::abs(h_channel[i]) % std::arg(h_channel[i]) << std::endl;
  }


  int nfft = _conf.ofdm_config.nfft;

  int *array_indices = new int[nfft];
  int *logical_indices = new int[nfft];
  int n = 0;


  for (int i = -int(nfft)/2; i < int(nfft)/2; i++)  {

    int array_index = normal_to_fft(i, nfft);
    std::complex<float> *this_complex = reinterpret_cast<std::complex<float> *>(_conf.data_config.h_freq[0]->input(0)+array_index);
    if (std::abs(*this_complex) > 1e-6) {
      logical_indices[n] = i;
      array_indices[n] = array_index;
      std::cerr << boost::format("%d) %d) %0.6e %0.6e %0.6e") % i % array_index % std::abs(*this_complex) % (std::abs(h_channel[array_indices[n]])) % (std::arg(h_channel[array_indices[n]])) << std::endl;
      ++n;
    }
  }

  for(int j=0; j<n; ++j) {
    angles.push_back(std::arg(h_channel[array_indices[j]]));
  }
  array_indices = NULL;

  std::cerr << boost::format("Angles before unwrap: ") <<std::endl;
  for(unsigned int i=0; i<angles.size(); ++i) {
    std::cerr << boost::format("%0.6e") % angles[i] << std::endl;
  }

  
  
  float *deltas = new float[n];
  memset(deltas, 0, n*sizeof(float));

  // for (int step = 0; step < n; step++) {
  //   int li = (logical_indices ? logical_indices[step] : step);
  //   int ai = (array_indices ? array_indices[step] : step);
  //   fprintf(stderr, "%d %f\n", li, angles[ai]);
  // }

  float mean_angle_numer = 0;
  float mean_angle_denom = 0;

  int prev_logical_index = (logical_indices ? logical_indices[0] : 0);
  int prev_array_index = (array_indices ? array_indices[0] : 0);

  for (int step = 1; step < n; step++) {
    int logical_index = (logical_indices ? logical_indices[step] : step);
    int array_index = (array_indices ? array_indices[step] : step);
    float weight = (weights ? weights[array_index] : 1);
    
    
    float diff_angle = angles[array_index] - angles[prev_array_index];
    if (diff_angle < 0) {
      diff_angle = diff_angle + 2*M_PI;
    }
    fprintf(stderr, "step=%d, array_index=%d, diff_angle=%f, weight = %f\n", step, array_index, diff_angle, weight); fflush(stderr);
    deltas[step] = diff_angle;
    if ((logical_index - prev_logical_index) == 1) {
      mean_angle_numer += weight*diff_angle;
      mean_angle_denom += weight;
    }

    prev_array_index = array_index;
    prev_logical_index = logical_index;
  }


  prev_logical_index = (logical_indices ? logical_indices[0] : 0);
  prev_array_index = (array_indices ? array_indices[0] : 0);

  float mean_angle = mean_angle_numer/mean_angle_denom;
  fprintf(stderr, "mean_angle = %f\n", mean_angle);

  float sum_error_numer = 0;
  float sum_error_denom = 0;

  for (int step = 1; step < n; step++) {
    int logical_index = (logical_indices ? logical_indices[step] : step);
    int array_index = (array_indices ? array_indices[step] : step);
    float weight = (weights ? weights[array_index] : 1);
    
    //fprintf(stderr, "step=%d, i=%d, array_index=%d\n", step, i, array_index); fflush(stderr);
    int diff_index = logical_index - prev_logical_index;
    float expected_diff_angle = diff_index*mean_angle;
    //while(expected_diff_angle >= 2*M_PI) {
    //  expected_diff_angle -= 2*M_PI;
    // }
    float best_delta_error_estimate = deltas[step] - expected_diff_angle;
    float this_delta_error_estimate = best_delta_error_estimate - 2*M_PI;
    for (int j = 0; j <= diff_index; j++) {
      if (fabsf(this_delta_error_estimate) < fabsf(best_delta_error_estimate)) {
	best_delta_error_estimate = this_delta_error_estimate;
      }
      this_delta_error_estimate = this_delta_error_estimate + 2*M_PI;
    }
    
    sum_error_numer += weight*fabsf(best_delta_error_estimate);
    sum_error_denom += weight;

    // if (fabsf(best_delta_error_estimate) > UNWRAP_NOISE_TOLERANCE) {
    //   fprintf(stderr, "unwrap failed at %d, mean_angle=%f, weight=%f, delta=%f, expected_diff=%f, best_delta_error_estimate=%f, prev_angle=%f, curr_angle=%f\n", logical_index, mean_angle, weight, deltas[step], expected_diff_angle, best_delta_error_estimate, angles[prev_array_index], angles[array_index]);
    //   if (weights) {
    // fprintf(stderr, "weights: ");
    // for (int j = 0; j < n; j++) {
    //   int ai = (array_indices ? array_indices[j] : j);
    //   fprintf(stderr, "%f ", weights[ai]);
    // }
    //   }
    //   fprintf(stderr, "\n");
    //   fprintf(stderr, "deltas: ");
    //   for (int j = 0; j < n; j++) {
    // fprintf(stderr, "%f ", deltas[j]);
    //   }
    //   fprintf(stderr, "\n");
    //   fprintf(stderr, "angles: ");
    //   for (int j = 0; j < n; j++) {
    // int this_li = ((logical_indices) ? logical_indices[j] : j);
    // int this_ai = ((array_indices) ? array_indices[j] : j);
    // fprintf(stderr, "%d : %f ", this_li, angles[this_ai]);
    //   }
    //   fprintf(stderr, "\n");
    //   //exit(1);
    // }
    
    float this_best_delta = best_delta_error_estimate + expected_diff_angle;
    
    angles[array_index] = angles[prev_array_index] + this_best_delta;
    
    prev_array_index = array_index;
    prev_logical_index = logical_index;
  }


  delete[] deltas;
  return (sum_error_numer/sum_error_denom);
}

// when you know the slope is close to zero  
float packet_rx::unwrap_angles(std::vector<float>& angles) {
  int nfft = _conf.ofdm_config.nfft;  

  int *array_indices = NULL;
  int *logical_indices = new int[nfft];
  int n = 0;

  for (int i = -int(nfft)/2; i < int(nfft)/2; i++)  {
    int array_index = normal_to_fft(i, nfft);
    std::complex<float> *this_complex = reinterpret_cast<std::complex<float> *>(_conf.data_config.h_freq[0]->input(0)+array_index);
    if (std::abs(*this_complex) > 1e-6) {
      logical_indices[n] = i;
      ++n;
    }
  }

  
  for(int j=0; j<n; ++j) {
    angles.push_back(std::arg(h_channel[array_indices[j]]));
  }

  float prev_angle = angles[(array_indices ? array_indices[0] : 0)];
  float phase_inc = 0;
  for (int i = 1; i < n; i++) {
    int ai = (array_indices ? array_indices[i] : i);
    float delta1 = angles[ai] - prev_angle;
    prev_angle = angles[ai];
    float delta2 = (delta1+M_PI-(2*M_PI*floor((delta1+M_PI)/(2*M_PI))))-M_PI;
    if ((delta2 == -M_PI) && (delta1 > 0)) {
      delta2 = M_PI;
    }
    float delta_diff = delta2-delta1;
    if (fabsf(delta1) < M_PI) {
      delta_diff = 0;
    }
    phase_inc += delta_diff;
    angles[ai] += phase_inc;
  }

  for(int j=0; j<n; ++j) {
    h_channel[array_indices[j]] = std::polar(std::abs(h_channel[array_indices[j]]), angles[j]);
  }
  return 0;
}
