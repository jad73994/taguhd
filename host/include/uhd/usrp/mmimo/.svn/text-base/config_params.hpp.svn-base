#ifndef INCLUDED_UHD_USRP_MMIMO_CONFIG_PARAMS_HPP
#define INCLUDED_UHD_USRP_MMIMO_CONFIG_PARAMS_HPP

#include <vector>
#include <complex>
#include <uhd/config.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/usrp/mmimo/fftw.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/asio.hpp>

namespace uhd {
  namespace mmimo {

    struct usrp_config_t {
      double rate;
    };

    struct ofdm_config_t {
      unsigned int nfft;
      unsigned int ncp;
      float sliding_window_thresh;
      float delay_correlate_thresh;
      unsigned int num_h_chunks;
      unsigned int num_h_syms_per_chunk;
      unsigned int detect_jump;
      unsigned int num_tx_zeroes;
    };

    struct network_config_t {
      unsigned int num_txs;
    };

    struct data_config_t {
      //std::vector<fftwf_complex *> h_symbol;
      unsigned int num_symbols;
      std::vector<uhd::mmimo::fftw::sptr> h_freq;
      std::vector<uhd::mmimo::fftw::sptr> data_freq;
    };

    struct UHD_API config_params {
      usrp_config_t usrp_config;
      ofdm_config_t ofdm_config;
      network_config_t network_config;
      data_config_t data_config;
    };

  } // namespace mmimo
} //namespace uhd

#endif /* INCLUDED_UHD_USRP_MMIMO_CONFIG_PARAMS_HPP */
