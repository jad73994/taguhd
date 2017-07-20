#ifndef INCLUDED_UHD_USRP_MMIMO_FFTW_HPP
#define INCLUDED_UHD_USRP_MMIMO_FFTW_HPP

#include <complex>
#include <cstring>
#include <fftw3.h>
#include <iostream>
#include <vector>
#include <uhd/config.hpp>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>

namespace uhd {
  namespace mmimo {

    class UHD_API fftw {
      
    private:
      fftwf_complex *_in, *_out;
      fftwf_plan *_plan;
      unsigned int _nfft;
      unsigned int _nsyms;

    public:

      typedef boost::shared_ptr<fftw> sptr;
      typedef std::vector<sptr> vec;
      typedef boost::shared_ptr<vec> vec_sptr;

      fftw(unsigned int nfft, unsigned int nsyms, int sign, unsigned int flag);
      ~fftw();

      inline fftwf_complex *input(unsigned int i) {
	if (i >= _nsyms) {
	  std::cerr << boost::format("fftw::input: symbol index %u exceeds nsyms %u") % i % _nsyms << std::endl;
	}
	return _in+i*_nfft;
      }

      inline fftwf_complex *output(unsigned int i) {
	if (i >= _nsyms) {
	  std::cerr << boost::format("fftw::output: symbol index %u exceeds nsyms %u") % i % _nsyms << std::endl;
	}
	return _out+i*_nfft;
      }

      inline void assign(unsigned int i, fftwf_complex *in) {
	if (i >= _nsyms) {
	  std::cerr << boost::format("fftw::assign: symbol index %u exceeds nsyms %u") % i % _nsyms << std::endl;
	}
	std::copy(in, in + _nfft, _in+i*_nfft);
      }

      inline void zero() {
	::memset((void *)_in, 0, _nsyms*_nfft*sizeof(fftwf_complex));
      }

      inline unsigned int width() {return _nfft;}

      inline unsigned int num_syms() { return _nsyms;}

      inline unsigned int num_samples() {return _nsyms*_nfft;}

      inline static int fft_to_normal(int i, unsigned int nfft) {  
	if (i > (int(nfft)/2-1)) {
	  i -= nfft;
	}
	return i;
      }
      
      inline static unsigned int normal_to_fft(int i, unsigned int nfft) {
	if (i < 0) {
	  i += nfft;
	}
	return ((unsigned int)i);
      }

      void scale(std::complex<float> c);
      void multiply(const std::vector<std::complex<float> >* v);
      void multiply(fftwf_complex *v);
      void multiply_output(fftwf_complex* v);
      void multiply_output(std::complex<float>* v);

      void execute();
	
      void rotate_output(float theta); // rotate subcarrier i by i*theta for all i
    };
  } //namespace mmimo
} // namespace uhd
      
#endif /* INCLUDED_UHD_USRP_MMIMO_FFTW_HPP */



