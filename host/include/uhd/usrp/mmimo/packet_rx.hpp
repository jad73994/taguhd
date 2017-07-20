#ifndef INCLUDED_UHD_USRP_MMIMO_PACKET_RX_HPP
#define INCLUDED_UHD_USRP_MMIMO_PACKET_RX_HPP

#include <vector>
#include <complex>
//#include <fftw3.h>
#include <uhd/config.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/usrp/mmimo/config_params.hpp>
#include <uhd/usrp/mmimo/fftw.hpp>

namespace uhd {
  namespace mmimo {

    class UHD_API packet_rx {

    public:

      enum rx_mode_t {
	RX_MODE_NULL = 0, 
	RX_MODE_LOG_ALL = 1, 
	RX_MODE_DETECT_AND_LOG = 2, 
	RX_MODE_CFO = 3, 
	RX_MODE_DETECT_START = 4,  // sync up slave transmitters
	RX_MODE_HBASE = 5,         // at slave transmitters, during measurement (use h_measurement_time)
	RX_MODE_HIJ = 6,           // at receiver
	RX_MODE_HBASE_CURR = 7,     // at slave transmitters, during data
	RX_MODE_BEACON = 8  // beacon for timing purposes
      };

      struct packet_rx_request_t {
	rx_mode_t mode;

	size_t tot_samples;

	bool has_start_time;
	uhd::time_spec_t start_time;

	// bool has_timeout_spec;
	// uhd::time_spec_t timeout_spec;;

	unsigned int num_symbols; // cfo or log
	std::string logfile;
	uhd::time_spec_t h_measurement_time;
	bool has_precomputed_cfo;
	double precomputed_cfo;
	
	unsigned int num_h_chunks;
	unsigned int num_h_syms_per_chunk;
	double h_target_offset;

	// packet_rx_request_t(rx_mode_t, unsigned int);
	packet_rx_request_t(rx_mode_t, unsigned int, bool, uhd::time_spec_t &);
	packet_rx_request_t(rx_mode_t, unsigned int, std::string &, bool, uhd::time_spec_t &);
	packet_rx_request_t(rx_mode_t, bool, uhd::time_spec_t &, unsigned int, uhd::time_spec_t &Y, bool, double, unsigned int, unsigned int, double);

      };

      struct packet_rx_mem_t {
	std::vector<float> energy_samples; // 2*nfft wide
	std::vector<std::vector<std::complex<float> > *> temp_buff; // nfft wide, array of 2
	std::vector<std::complex<float> > log_buff;

	std::vector<bool> measure_tx_h;
	std::vector<uhd::mmimo::fftw::vec_sptr> h_samples; // vector of vectors, inner vector size num_h_chunks, outer vector size num_tx
	
	typedef boost::shared_ptr<std::vector<std::vector<std::complex<float> > > > chunk_sym_sptr;

	std::vector<chunk_sym_sptr> chunk_channels; // outermost index is tx index, second index is chunk index, third index is subcarrier

	packet_rx_mem_t(const config_params &, const packet_rx_request_t &);
	// void zero();
      };

      packet_rx(const config_params &, uhd::usrp::multi_usrp *, packet_rx_request_t &, packet_rx_mem_t &);
      //~packet_rx();
      void process();
      void compute_all_h();

      double get_cfo();
      static void sig_int_handler(int);
      static bool _stop_signal_called;

      void get_detection_time(uhd::time_spec_t& time) { time = detection_time; }

      //void get_channel(std::vector<std::complex<float> > &channel);

      float regress_H(std::vector<float> &y, float &slope, float &intercept, float* weights);
      float unwrap_angles(std::vector<float>& angles,  float *weights);
      float unwrap_angles(std::vector<float>& angles);

    private:
      static int normal_to_fft(int index, int nfft);
      static const unsigned int INITIAL_WAIT_NUM_SAMPLES = 100;
      static const unsigned int MAX_STREAM_SAMPLES = (unsigned int)(100e6);
      

      const config_params &_conf;
      uhd::usrp::multi_usrp *_usrp;
      packet_rx_request_t &_req;
      packet_rx_mem_t &_mem;
      
      std::vector< std::vector<std::complex<float> > > _sample_buff_arr; 
      std::vector< std::complex<float> * > _sample_buff; 
      size_t _sample_buff_size;
      size_t _sample_buff_index;
      uhd::time_spec_t _sample_buff_start_timestamp;
      double _sample_buff_recv_timeout;
      unsigned int _num_samples_default;

      unsigned int _num_streamed_samples;
      unsigned int _num_remaining_samples;

      std::vector<std::complex<float> > h_channel;

      uhd::time_spec_t detection_time;
      
      double _cfo;
      double _pkt_recv_time;

      double _pi;

      // logging


      bool get_next_sample(std::complex<float>&, uhd::time_spec_t &);
    }; 
  } // namespace mmimo
} // namespace uhd

//bool uhd::mmimo::packet_rx::_stop_signal_called = false;

#endif /* INCLUDED_UHD_USRP_MMIMO_PACKET_RX_HPP */
