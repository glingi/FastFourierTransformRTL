//
// Testbench
//

#include "top_256fft.h"
#include "hls/linear_algebra/utils/x_hls_matrix_tb_utils.h"
using namespace std;

int main()
{
	int nSegment = 32;

	ifix_W10_F9 fft_in_set[NFFT*nSegment];
	ifix_W10_F9  fft_in[NFFT*nSegment];

	cfix_W14_F8  fft_HLS_set[NFFT*nSegment];
	cfix_W14_F8  fft_HLS[NFFT*nSegment];
//	cfix_W22_F13 fft_HLS[NFFT];

	ifstream din_s_real;
	din_s_real.open("./mid_fft_input_ch1.dat");

	if(din_s_real.is_open()){

		for(int c = 0; c < NFFT*nSegment; c++){
			din_s_real >> fft_in_set[c];
		}

	}else{
		cout << "Error!! Missing file : input_real.dat or input_imag.dat " << endl;
	}
	din_s_real.close();

//	std::cout << "[[tb_top_256fft.cpp] --------------- fft input " << std::endl;
//	for(int c = 0; c < NFFT*nSegment; c++)
//		std::cout << fft_in_set[c] << std::endl;

//	s[0].real() = 0.0008544922;	// it doesn't work
//	s[64].real() = 0.0013427734;

	for(int iter=0; iter<nSegment; iter++){
		cout << " ------------------- iteration : " << iter << "---------------" << endl;

		for(int idx=0; idx < NFFT; idx++)
			fft_in[idx] = fft_in_set[NFFT*iter + idx];


		radix4FFT3_FixPtEML(
				fft_in,
				fft_HLS
		);

		for(int idx=0; idx < NFFT; idx++)
			fft_HLS_set[NFFT*iter + idx] = fft_HLS[idx];
	}

	ofstream dout_s_real;
	ofstream dout_s_imag;

	dout_s_real.open("./fft_HLS_out.dat");

	if(dout_s_real.is_open()){

		for(int c=0; c <NFFT*nSegment; c++){
			dout_s_real << setw(32) << fft_HLS_set[c].real() << " + 1i*" <<  fft_HLS_set[c].imag() << endl;
		}

	}else{
		cout << "Error!! Can't open input_real.dat or input_imag.dat " << endl;
	}
	dout_s_real.close();

	return 0;
}
