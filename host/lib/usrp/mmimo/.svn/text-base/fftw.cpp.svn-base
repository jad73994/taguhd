#include <boost/math/constants/constants.hpp>
#include <uhd/usrp/mmimo/fftw.hpp>
#include <iostream>
#include <boost/format.hpp>

using namespace std;
using namespace boost;
using namespace uhd::mmimo;

fftw::fftw(unsigned int nfft, unsigned int nsyms, int sign, unsigned int flag) : _nfft(nfft), _nsyms(nsyms) {
  _in = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * nfft * nsyms);
  _out = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * nfft * nsyms);
  _plan = new fftwf_plan[nsyms];
  for (unsigned int i = 0; i < nsyms; i++) {
    _plan[i] = fftwf_plan_dft_1d(nfft, _in+i*nfft, _out+i*nfft, sign, flag);
  }
}

fftw::~fftw() {
  for (unsigned int i = 0; i < _nsyms; i++) {
    fftwf_destroy_plan(_plan[i]);
  }
  delete[] _plan;
  fftwf_free(_in); 
  fftwf_free(_out);
}


void fftw::scale(std::complex<float> c) {
  for (unsigned int i = 0; i < _nfft*_nsyms; i++) {
    std::complex<float> *x = reinterpret_cast<std::complex<float> *>(_in + i);
    (*x) *= c;
  }
}


void fftw::multiply(const std::vector<std::complex<float> >* v) {
  for (unsigned int i = 0; i < _nfft*_nsyms; i++) {
    std::complex<float> *x = reinterpret_cast<std::complex<float> *>(_in + i);
    (*x) *= (*v)[i];
  }
}

void fftw::multiply(fftwf_complex* v) {
  for (unsigned int i = 0; i < _nfft*_nsyms; i++) {
    std::complex<float> *x = reinterpret_cast<std::complex<float> *>(_in + i);
    std::complex<float> *y = reinterpret_cast<std::complex<float> *>(v + i);
    (*x) *= (*y);
  }
}

void fftw::multiply_output(fftwf_complex* v) {
  for (unsigned int i = 0; i < _nfft*_nsyms; i++) {
    std::complex<float> *x = reinterpret_cast<std::complex<float> *>(_out + i);
    std::complex<float> *y = reinterpret_cast<std::complex<float> *>(v + i);
    (*x) *= (*y);
  }
}

void fftw::multiply_output(std::complex<float>* v) {
  for (unsigned int i = 0; i < _nfft*_nsyms; i++) {
    std::complex<float> *x = reinterpret_cast<std::complex<float> *>(_out + i);
    (*x) *= (*v);
  }
}


void fftw::execute() {
  for (unsigned int i = 0; i < _nsyms; i++) {
    fftwf_execute(_plan[i]);
  }
}

void fftw::rotate_output(float theta) {
  for (unsigned int i = 0; i < _nsyms; ++i) {
    for (unsigned int j = 0; j < _nfft; ++j) {
      std::complex<float> *cf = reinterpret_cast<std::complex<float> *>(_out+i*_nfft+j);
      *cf *= std::polar(float(1.0), fft_to_normal(j, _nfft)*theta);
    }
  }
}


