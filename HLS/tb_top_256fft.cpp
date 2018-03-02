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
//	dout_s_imag.open("./fft_HLS_out_ranging.dat");

	if(dout_s_real.is_open()){

		for(int c=0; c <NFFT*nSegment; c++){
//			input_HLS[c] = fft_HLS[c]; // type-casting

//			input_HLS[c].real() = (cfix_W14_F10)fft_HLS[c].real().range(27,14);
//			input_HLS[c].imag() = (cfix_W14_F10)fft_HLS[c].imag().range(27,14);

			dout_s_real << setw(32) << fft_HLS_set[c].real() << " + 1i*" <<  fft_HLS_set[c].imag() << endl;
//			cout << setw(32) << fft_HLS_set[c].real() << " + 1i*" <<  fft_HLS[c].imag() << endl;

//			dout_s_imag << setw(32) << fft_HLS_set[c].imag() << " + 1i*" <<  out_fft_test[c].imag() << endl;
			//refer to UG902,  p. 661
//			dout_s_real << setw(32) << fft_HLS[c].real() << fft_HLS[c].real().range(27,14)  <<endl;
//			dout_s_imag << setw(32) << fft_HLS[c].imag() << fft_HLS[c].imag().range(27,14)  <<endl;

//			dout_s_real << setw(32) << input_HLS[c].real() << endl;
//			dout_s_imag << setw(32) << input_HLS[c].imag() << endl;

		}

	}else{
		cout << "Error!! Can't open input_real.dat or input_imag.dat " << endl;
	}
	dout_s_real.close();
//	dout_s_imag.close();


//	result.open("simout_stage.dat");
//
//	for(int j=0;j<NFFT;j++){	//j=2 (cloumn)
//		result << setw(32) << stage_HLS[j] << endl; //  "   " ;
//	}
//	result.close();

//	result.open("simout_FFT.dat");
//	for(int j=0;j<NFFT;j++){	//j=2 (cloumn)
//		result << setw(32) << fft_HLS[j] << endl; //  "   " ;
//	}
//	result.close();


	// compare part
	// Check the results, refer to ug902, p.187
//	int ret_re, ret_im, ret =0;
//
//	ret_re = system("diff --brief -w output_real.dat fft_HLS_real.dat");
//	ret_im = system("diff --brief -w output_imag.dat fft_HLS_imag.dat");
//	if ( (ret_re && ret_im) != 0) {
//		cout << "Test failed !!!  " << ret_re << "  " <<ret_im << endl;
//		ret=1;
//	} else {
//		cout <<"Test passed !" << endl;
//		ret=0;
//	}

	// Misc
//	int retval=0, i, i_trans, tmp;
//	FILE *fp;
//
//	// Load expected output data from files
//	fp=fopen(tb_data/outC.golden.dat,r);
//	for (i=0; i<NUM_TRANS; i++){
//	fscanf(fp, %d, &tmp);
//	c_expected[i] = tmp;
//	}
//	fclose(fp);
//	fp=fopen(tb_data/outD.golden.dat,r);
//	for (i=0; i<NUM_TRANS; i++){
//	fscanf(fp, %d, &tmp);
//	d_expected[i] = tmp;
//	}
//	fclose(fp);
//	// Check outputs against expected
//	for (i = 0; i < NUM_TRANS-1; ++i) {
//		if(c[i] != c_expected[i]){
//			retval = 1;
//		}
//
//		if(d[i] != d_expected[i]){
//			retval = 1;
//		}
//	}
//	// Print Results
//	if(retval == 0){
//		printf( *** *** *** *** \n);
//		printf( Results are good \n);
//		printf( *** *** *** *** \n);
//	} else {
//		printf( *** *** *** *** \n);
//		printf( Mismatch: retval=%d \n, retval);
//		printf( *** *** *** *** \n);
//	}
//	// Return 0 if outputs are corre
//	return retval;



	return 0;
}
