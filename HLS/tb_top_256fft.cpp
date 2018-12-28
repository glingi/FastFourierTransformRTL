//
// C/RTL cosimulation testbench
//

#include "top_256fft.h"
#include "hls/linear_algebra/utils/x_hls_matrix_tb_utils.h"
using namespace std;

int main()
{
	int nSegment = 32;

	ifix_W10_F9  fft_in_set[NFFT*nSegment];
	ifix_W10_F9  fft_in[NFFT*nSegment];

	cfix_W14_F8  fft_HLS_set[NFFT*nSegment];
	cfix_W14_F8  fft_HLS[NFFT*nSegment];
	
	FILE* din_s_real;
	din_s_real = fopen("./mid_fft_input_ch1.dat", "r");

	float val;
	for(int c = 0; c < NFFT*nSegment; c++){
		fscanf(din_s_real, "%f", &val);
		fft_in_set[c] = ifix_W10_F9(val);
	}

	fclose(din_s_real);

	// Below codes doesn't work		
	// s[0].real() = 0.0008544922;	
	// s[64].real() = 0.0013427734;

	for(int iter=0; iter<nSegment; iter++){
		cout << " ------------------- iteration : " << iter << "---------------" << endl;

		for(int idx=0; idx < NFFT; idx++){
			fft_in[idx] = fft_in_set[NFFT*iter + idx];
		}			

		radix4FFT3_FixPtEML(
			fft_in,
			fft_HLS
		);

		for(int idx=0; idx < NFFT; idx++){
			fft_HLS_set[NFFT*iter + idx] = fft_HLS[idx];
		}
	}

	FILE* dout_s_real;

	dout_s_real = fopen("./fft_HLS_out.dat", "w");

	for(int c=0; c <NFFT*nSegment; c++)
	{
		fprintf(dout_s_real, "%f + 1i*%f \n", fft_HLS_set[c].real(), fft_HLS_set[c].imag());
	}

	fclose(dout_s_real);

	return 0;
}
