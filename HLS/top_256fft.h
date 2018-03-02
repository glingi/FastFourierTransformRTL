#ifndef FFT256_H
#define FFT256_H

#include "hls_math.h"
#include "hls_dsp.h"	// for cmpy
//#include <stdio.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <complex>		// for cmpy

typedef hls::CmpyThreeMult 					ARCH_CMPY;

typedef ap_fixed<10,1> 			     		ifix_W10_F9;
typedef ap_ufixed<10,1> 			     	ufix_W10_F9;
typedef ap_ufixed<10,0> 			     	ufix_W10_F10;
//typedef ap_fixed<10,1> 						ifix_W10_F9;

typedef ap_fixed<14,1> 			     		ifix_W14_F13;
typedef ap_fixed<14,2> 			     		ifix_W14_F12;
typedef ap_fixed<14,4>						ifix_W14_F10;
typedef ap_fixed<14,7> 			     		ifix_W14_F7;
typedef ap_fixed<14,11>						ifix_W14_F3;
typedef ap_fixed<14,6>  					ifix_W14_F8;
typedef ap_fixed<14,9> 						ifix_W14_F5;

typedef ap_fixed<24,11>  					ifix_W24_F13; // for output

typedef ap_fixed<28,13> 					ifix_W28_F15;  // Bo,Co,Do
typedef ap_fixed<28,1>  					ifix_W28_F27;
//typedef ap_fixed<28,8>  					ifix_W28_F20;
typedef ap_fixed<28,7>  					ifix_W28_F21;
typedef ap_fixed<29,9> 						ifix_W29_F19;
typedef ap_fixed<29,5>  					ifix_W29_F24;

typedef std::complex< ifix_W14_F5 >			cfix_W14_F5;
typedef std::complex< ifix_W14_F13 >		cfix_W14_F13;
typedef std::complex< ifix_W14_F12 >		cfix_W14_F12; //twiddle factor
typedef std::complex< ifix_W14_F10 >		cfix_W14_F10; // test
typedef std::complex< ifix_W14_F3 >	 		cfix_W14_F3;
typedef std::complex< ifix_W14_F7 >			cfix_W14_F7;


typedef std::complex< ifix_W14_F8 >			cfix_W14_F8;
typedef std::complex< ifix_W24_F13 >		cfix_W24_F13;  // output


typedef std::complex<ap_fixed<28,11> >		cfix_W28_F17;  // output
typedef std::complex< ifix_W28_F27 >		cfix_W28_F27; // aa,bb,cc,dd
typedef std::complex< ifix_W28_F21 >		cfix_W28_F21; // Ao
//typedef std::x_complex<ap_fixed<28,8> >		cfix_W28_F20; // H_B, H_C
typedef std::complex<ap_fixed<28,11> >		cfix_W28_F17;  // output
typedef std::complex< ifix_W28_F15 >		cfix_W28_F15;  // Bo,Co,Do
typedef std::complex< ifix_W29_F24 >		cfix_W29_F24;

typedef ap_fixed<29,2>		ifix_W29_F27;
typedef ap_fixed<43,5> 		ifix_W43_F38;
typedef ap_fixed<29,8> 	    ifix_W29_F21;
typedef ap_fixed<43,10> 	ifix_W43_F33;

#define deserialization_factor 	8
#define NFFT 					256
#define NSTAGE 					4


/* Function Declarations */
void radix4FFT3_FixPtEML(
	ifix_W10_F9  in_fft[NFFT],	 // FMC126 ADC only supports unsigned data
	cfix_W14_F8  out_fft[NFFT]   // for expander, return 14 bits from MSB
);

void radix4bfly(
	cfix_W14_F13*   x,
	int 		    segment,
	int 		    stageFlag,
	cfix_W14_F12*   TwiddleFactor,
	cfix_W14_F13*   A,
	cfix_W14_F13*   B,
	cfix_W14_F13*   C,
	cfix_W14_F13*   D
);


// Don't use this method, more resources are needed.
//void radix4bfly_RealInput(
//	ifix_W14_F13*   x_re,
//	int 		    segment,
//	int 		    stageFlag,
//	cfix_W14_F12*   TwiddleFactor,
//	cfix_W28_F21*   A,
//	cfix_W28_F15*   B,
//	cfix_W28_F15*   C,
//	cfix_W28_F15*   D
//);

#endif
