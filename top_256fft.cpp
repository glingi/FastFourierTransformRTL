#include "top_256fft.h"
using namespace std;
void radix4FFT3_FixPtEML(
		ifix_W10_F9 in_fft[NFFT],		// Input is only real
										// FMC126 ADC only supports unsigned data
		cfix_W14_F8 out_fft[NFFT]		// for expander
)
{

//	#pragma HLS PIPELINE //not available


	// Reshape an array from one with many elements to one with
	// greater word-width. Useful for improving block RAM
	// accesses without using more block RAM.
	// Refer to UG902 p. 131
	#pragma HLS INTERFACE ap_vld port=in_fft
	#pragma HLS ARRAY_RESHAPE variable=in_fft
	#pragma HLS ARRAY_RESHAPE variable=out_fft

	//	Twiddle_factor_init_region
	// UG902 p. 646
	//	The ap_[u]fixed types do not support initialization if they are used in an array of
	//std::complex types.
	//typedef ap_fixed<DIN_W, 1, AP_TRN, AP_SAT> coeff_t; // MUST have IW >= 1
	//std::complex<coeff_t> twid_rom[REAL_SZ/2] = {{ 1, -0 },{ 0.9,-0.006 }, etc.}
	//The initialization values must first be case to std:complex:
	//typedef ap_fixed<DIN_W, 1, AP_TRN, AP_SAT> coeff_t; // MUST have IW >= 1
	//std::complex<coeff_t> twid_rom[REAL_SZ/2] = {std::complex<coeff_t>( 1, -0 ),
	//std::complex<coeff_t>(0.9,-0.006 ),etc.}

	cfix_W14_F12 TwiddleFactor[256];
	TwiddleFactor[0].real()=1.000000; TwiddleFactor[1].real()=0.999699;TwiddleFactor[2].real()=0.998795;TwiddleFactor[3].real()=0.997290;TwiddleFactor[4].real()=0.995185;
	TwiddleFactor[5].real()=0.992480; TwiddleFactor[6].real()=0.989177;TwiddleFactor[7].real()=0.985278;TwiddleFactor[8].real()=0.980785;TwiddleFactor[9].real()=0.975702;TwiddleFactor[10].real()=0.970031;
	TwiddleFactor[11].real()=0.963776;TwiddleFactor[12].real()=0.956940;TwiddleFactor[13].real()=0.949528;TwiddleFactor[14].real()=0.941544;TwiddleFactor[15].real()=0.932993;TwiddleFactor[16].real()=0.923880;
	TwiddleFactor[17].real()=0.914210;TwiddleFactor[18].real()=0.903989;TwiddleFactor[19].real()=0.893224;TwiddleFactor[20].real()=0.881921;TwiddleFactor[21].real()=0.870087;TwiddleFactor[22].real()=0.857729;
	TwiddleFactor[23].real()=0.844854;TwiddleFactor[24].real()=0.831470;TwiddleFactor[25].real()=0.817585;TwiddleFactor[26].real()=0.803208;TwiddleFactor[27].real()=0.788346;TwiddleFactor[28].real()=0.773010;
	TwiddleFactor[29].real()=0.757209;TwiddleFactor[30].real()=0.740951;TwiddleFactor[31].real()=0.724247;TwiddleFactor[32].real()=0.707107;TwiddleFactor[33].real()=0.689541;TwiddleFactor[34].real()=0.671559;
	TwiddleFactor[35].real()=0.653173;TwiddleFactor[36].real()=0.634393;TwiddleFactor[37].real()=0.615232;TwiddleFactor[38].real()=0.595699;TwiddleFactor[39].real()=0.575808;TwiddleFactor[40].real()=0.555570;
	TwiddleFactor[41].real()=0.534998;TwiddleFactor[42].real()=0.514103;TwiddleFactor[43].real()=0.492898;TwiddleFactor[44].real()=0.471397;TwiddleFactor[45].real()=0.449611;TwiddleFactor[46].real()=0.427555;
	TwiddleFactor[47].real()=0.405241;TwiddleFactor[48].real()=0.382683;TwiddleFactor[49].real()=0.359895;TwiddleFactor[50].real()=0.336890;TwiddleFactor[51].real()=0.313682;TwiddleFactor[52].real()=0.290285;
	TwiddleFactor[53].real()=0.266713;TwiddleFactor[54].real()=0.242980;TwiddleFactor[55].real()=0.219101;TwiddleFactor[56].real()=0.195090;TwiddleFactor[57].real()=0.170962;TwiddleFactor[58].real()=0.146730;
	TwiddleFactor[59].real()=0.122411;TwiddleFactor[60].real()=0.098017;TwiddleFactor[61].real()=0.073565;TwiddleFactor[62].real()=0.049068;TwiddleFactor[63].real()=0.024541;TwiddleFactor[64].real()=0.000000;
	TwiddleFactor[65].real()=-0.024541;TwiddleFactor[66].real()=-0.049068;TwiddleFactor[67].real()=-0.073565;TwiddleFactor[68].real()=-0.098017;TwiddleFactor[69].real()=-0.122411;TwiddleFactor[70].real()=-0.146730;
	TwiddleFactor[71].real()=-0.170962;TwiddleFactor[72].real()=-0.195090;TwiddleFactor[73].real()=-0.219101;TwiddleFactor[74].real()=-0.242980;TwiddleFactor[75].real()=-0.266713;TwiddleFactor[76].real()=-0.290285;
	TwiddleFactor[77].real()=-0.313682;TwiddleFactor[78].real()=-0.336890;TwiddleFactor[79].real()=-0.359895;TwiddleFactor[80].real()=-0.382683;TwiddleFactor[81].real()=-0.405241;TwiddleFactor[82].real()=-0.427555;
	TwiddleFactor[83].real()=-0.449611;TwiddleFactor[84].real()=-0.471397;TwiddleFactor[85].real()=-0.492898;TwiddleFactor[86].real()=-0.514103;TwiddleFactor[87].real()=-0.534998;TwiddleFactor[88].real()=-0.555570;
	TwiddleFactor[89].real()=-0.575808;TwiddleFactor[90].real()=-0.595699;TwiddleFactor[91].real()=-0.615232;TwiddleFactor[92].real()=-0.634393;TwiddleFactor[93].real()=-0.653173;TwiddleFactor[94].real()=-0.671559;
	TwiddleFactor[95].real()=-0.689541;TwiddleFactor[96].real()=-0.707107;TwiddleFactor[97].real()=-0.724247;TwiddleFactor[98].real()=-0.740951;TwiddleFactor[99].real()=-0.757209;TwiddleFactor[100].real()=-0.773010;
	TwiddleFactor[101].real()=-0.788346;TwiddleFactor[102].real()=-0.803208;TwiddleFactor[103].real()=-0.817585;TwiddleFactor[104].real()=-0.831470;TwiddleFactor[105].real()=-0.844854;TwiddleFactor[106].real()=-0.857729;
	TwiddleFactor[107].real()=-0.870087;TwiddleFactor[108].real()=-0.881921;TwiddleFactor[109].real()=-0.893224;TwiddleFactor[110].real()=-0.903989;TwiddleFactor[111].real()=-0.914210;TwiddleFactor[112].real()=-0.923880;
	TwiddleFactor[113].real()=-0.932993;TwiddleFactor[114].real()=-0.941544;TwiddleFactor[115].real()=-0.949528;TwiddleFactor[116].real()=-0.956940;TwiddleFactor[117].real()=-0.963776;TwiddleFactor[118].real()=-0.970031;
	TwiddleFactor[119].real()=-0.975702;TwiddleFactor[120].real()=-0.980785;TwiddleFactor[121].real()=-0.985278;TwiddleFactor[122].real()=-0.989177;TwiddleFactor[123].real()=-0.992480;TwiddleFactor[124].real()=-0.995185;
	TwiddleFactor[125].real()=-0.997290;TwiddleFactor[126].real()=-0.998795;TwiddleFactor[127].real()=-0.999699;TwiddleFactor[128].real()=-1.000000;TwiddleFactor[129].real()=-0.999699;TwiddleFactor[130].real()=-0.998795;
	TwiddleFactor[131].real()=-0.997290;TwiddleFactor[132].real()=-0.995185;TwiddleFactor[133].real()=-0.992480;TwiddleFactor[134].real()=-0.989177;TwiddleFactor[135].real()=-0.985278;TwiddleFactor[136].real()=-0.980785;
	TwiddleFactor[137].real()=-0.975702;TwiddleFactor[138].real()=-0.970031;TwiddleFactor[139].real()=-0.963776;TwiddleFactor[140].real()=-0.956940;TwiddleFactor[141].real()=-0.949528;TwiddleFactor[142].real()=-0.941544;
	TwiddleFactor[143].real()=-0.932993;TwiddleFactor[144].real()=-0.923880;TwiddleFactor[145].real()=-0.914210;TwiddleFactor[146].real()=-0.903989;TwiddleFactor[147].real()=-0.893224;TwiddleFactor[148].real()=-0.881921;
	TwiddleFactor[149].real()=-0.870087;TwiddleFactor[150].real()=-0.857729;TwiddleFactor[151].real()=-0.844854;TwiddleFactor[152].real()=-0.831470;TwiddleFactor[153].real()=-0.817585;TwiddleFactor[154].real()=-0.803208;
	TwiddleFactor[155].real()=-0.788346;TwiddleFactor[156].real()=-0.773010;TwiddleFactor[157].real()=-0.757209;TwiddleFactor[158].real()=-0.740951;TwiddleFactor[159].real()=-0.724247;TwiddleFactor[160].real()=-0.707107;
	TwiddleFactor[161].real()=-0.689541;TwiddleFactor[162].real()=-0.671559;TwiddleFactor[163].real()=-0.653173;TwiddleFactor[164].real()=-0.634393;TwiddleFactor[165].real()=-0.615232;TwiddleFactor[166].real()=-0.595699;
	TwiddleFactor[167].real()=-0.575808;TwiddleFactor[168].real()=-0.555570;TwiddleFactor[169].real()=-0.534998;TwiddleFactor[170].real()=-0.514103;TwiddleFactor[171].real()=-0.492898;TwiddleFactor[172].real()=-0.471397;
	TwiddleFactor[173].real()=-0.449611;TwiddleFactor[174].real()=-0.427555;TwiddleFactor[175].real()=-0.405241;TwiddleFactor[176].real()=-0.382683;TwiddleFactor[177].real()=-0.359895;TwiddleFactor[178].real()=-0.336890;
	TwiddleFactor[179].real()=-0.313682;TwiddleFactor[180].real()=-0.290285;TwiddleFactor[181].real()=-0.266713;TwiddleFactor[182].real()=-0.242980;TwiddleFactor[183].real()=-0.219101;TwiddleFactor[184].real()=-0.195090;
	TwiddleFactor[185].real()=-0.170962;TwiddleFactor[186].real()=-0.146730;TwiddleFactor[187].real()=-0.122411;TwiddleFactor[188].real()=-0.098017;TwiddleFactor[189].real()=-0.073565;TwiddleFactor[190].real()=-0.049068;
	TwiddleFactor[191].real()=-0.024541;TwiddleFactor[192].real()=-0.000000;TwiddleFactor[193].real()=0.024541;TwiddleFactor[194].real()=0.049068;TwiddleFactor[195].real()=0.073565;TwiddleFactor[196].real()=0.098017;
	TwiddleFactor[197].real()=0.122411;TwiddleFactor[198].real()=0.146730;TwiddleFactor[199].real()=0.170962;TwiddleFactor[200].real()=0.195090;TwiddleFactor[201].real()=0.219101;TwiddleFactor[202].real()=0.242980;
	TwiddleFactor[203].real()=0.266713;TwiddleFactor[204].real()=0.290285;TwiddleFactor[205].real()=0.313682;TwiddleFactor[206].real()=0.336890;TwiddleFactor[207].real()=0.359895;TwiddleFactor[208].real()=0.382683;
	TwiddleFactor[209].real()=0.405241;TwiddleFactor[210].real()=0.427555;TwiddleFactor[211].real()=0.449611;TwiddleFactor[212].real()=0.471397;TwiddleFactor[213].real()=0.492898;TwiddleFactor[214].real()=0.514103;
	TwiddleFactor[215].real()=0.534998;TwiddleFactor[216].real()=0.555570;TwiddleFactor[217].real()=0.575808;TwiddleFactor[218].real()=0.595699;TwiddleFactor[219].real()=0.615232;TwiddleFactor[220].real()=0.634393;
	TwiddleFactor[221].real()=0.653173;TwiddleFactor[222].real()=0.671559;TwiddleFactor[223].real()=0.689541;TwiddleFactor[224].real()=0.707107;TwiddleFactor[225].real()=0.724247;TwiddleFactor[226].real()=0.740951;
	TwiddleFactor[227].real()=0.757209;TwiddleFactor[228].real()=0.773010;TwiddleFactor[229].real()=0.788346;TwiddleFactor[230].real()=0.803208;TwiddleFactor[231].real()=0.817585;TwiddleFactor[232].real()=0.831470;
	TwiddleFactor[233].real()=0.844854;TwiddleFactor[234].real()=0.857729;TwiddleFactor[235].real()=0.870087;TwiddleFactor[236].real()=0.881921;TwiddleFactor[237].real()=0.893224;TwiddleFactor[238].real()=0.903989;
	TwiddleFactor[239].real()=0.914210;TwiddleFactor[240].real()=0.923880;TwiddleFactor[241].real()=0.932993;TwiddleFactor[242].real()=0.941544;TwiddleFactor[243].real()=0.949528;TwiddleFactor[244].real()=0.956940;
	TwiddleFactor[245].real()=0.963776;TwiddleFactor[246].real()=0.970031;TwiddleFactor[247].real()=0.975702;TwiddleFactor[248].real()=0.980785;TwiddleFactor[249].real()=0.985278;TwiddleFactor[250].real()=0.989177;
	TwiddleFactor[251].real()=0.992480;TwiddleFactor[252].real()=0.995185;TwiddleFactor[253].real()=0.997290;TwiddleFactor[254].real()=0.998795;TwiddleFactor[255].real()=0.999699;

	TwiddleFactor[0].imag()=0.000000;TwiddleFactor[1].imag()=-0.024541;TwiddleFactor[2].imag()=-0.049068;TwiddleFactor[3].imag()=-0.073565;TwiddleFactor[4].imag()=-0.098017;
	TwiddleFactor[5].imag()=-0.122411;TwiddleFactor[6].imag()=-0.146730;TwiddleFactor[7].imag()=-0.170962;TwiddleFactor[8].imag()=-0.195090;TwiddleFactor[9].imag()=-0.219101;TwiddleFactor[10].imag()=-0.242980;
	TwiddleFactor[11].imag()=-0.266713;TwiddleFactor[12].imag()=-0.290285;TwiddleFactor[13].imag()=-0.313682;TwiddleFactor[14].imag()=-0.336890;TwiddleFactor[15].imag()=-0.359895;TwiddleFactor[16].imag()=-0.382683;
	TwiddleFactor[17].imag()=-0.405241;TwiddleFactor[18].imag()=-0.427555;TwiddleFactor[19].imag()=-0.449611;TwiddleFactor[20].imag()=-0.471397;TwiddleFactor[21].imag()=-0.492898;TwiddleFactor[22].imag()=-0.514103;
	TwiddleFactor[23].imag()=-0.534998;TwiddleFactor[24].imag()=-0.555570;TwiddleFactor[25].imag()=-0.575808;TwiddleFactor[26].imag()=-0.595699;TwiddleFactor[27].imag()=-0.615232;TwiddleFactor[28].imag()=-0.634393;
	TwiddleFactor[29].imag()=-0.653173;TwiddleFactor[30].imag()=-0.671559;TwiddleFactor[31].imag()=-0.689541;TwiddleFactor[32].imag()=-0.707107;TwiddleFactor[33].imag()=-0.724247;TwiddleFactor[34].imag()=-0.740951;
	TwiddleFactor[35].imag()=-0.757209;TwiddleFactor[36].imag()=-0.773010;TwiddleFactor[37].imag()=-0.788346;TwiddleFactor[38].imag()=-0.803208;TwiddleFactor[39].imag()=-0.817585;TwiddleFactor[40].imag()=-0.831470;
	TwiddleFactor[41].imag()=-0.844854;TwiddleFactor[42].imag()=-0.857729;TwiddleFactor[43].imag()=-0.870087;TwiddleFactor[44].imag()=-0.881921;TwiddleFactor[45].imag()=-0.893224;TwiddleFactor[46].imag()=-0.903989;
	TwiddleFactor[47].imag()=-0.914210;TwiddleFactor[48].imag()=-0.923880;TwiddleFactor[49].imag()=-0.932993;TwiddleFactor[50].imag()=-0.941544;TwiddleFactor[51].imag()=-0.949528;TwiddleFactor[52].imag()=-0.956940;
	TwiddleFactor[53].imag()=-0.963776;TwiddleFactor[54].imag()=-0.970031;TwiddleFactor[55].imag()=-0.975702;TwiddleFactor[56].imag()=-0.980785;TwiddleFactor[57].imag()=-0.985278;TwiddleFactor[58].imag()=-0.989177;
	TwiddleFactor[59].imag()=-0.992480;TwiddleFactor[60].imag()=-0.995185;TwiddleFactor[61].imag()=-0.997290;TwiddleFactor[62].imag()=-0.998795;TwiddleFactor[63].imag()=-0.999699;TwiddleFactor[64].imag()=-1.000000;
	TwiddleFactor[65].imag()=-0.999699;TwiddleFactor[66].imag()=-0.998795;TwiddleFactor[67].imag()=-0.997290;TwiddleFactor[68].imag()=-0.995185;TwiddleFactor[69].imag()=-0.992480;TwiddleFactor[70].imag()=-0.989177;
	TwiddleFactor[71].imag()=-0.985278;TwiddleFactor[72].imag()=-0.980785;TwiddleFactor[73].imag()=-0.975702;TwiddleFactor[74].imag()=-0.970031;TwiddleFactor[75].imag()=-0.963776;TwiddleFactor[76].imag()=-0.956940;
	TwiddleFactor[77].imag()=-0.949528;TwiddleFactor[78].imag()=-0.941544;TwiddleFactor[79].imag()=-0.932993;TwiddleFactor[80].imag()=-0.923880;TwiddleFactor[81].imag()=-0.914210;TwiddleFactor[82].imag()=-0.903989;
	TwiddleFactor[83].imag()=-0.893224;TwiddleFactor[84].imag()=-0.881921;TwiddleFactor[85].imag()=-0.870087;TwiddleFactor[86].imag()=-0.857729;TwiddleFactor[87].imag()=-0.844854;TwiddleFactor[88].imag()=-0.831470;
	TwiddleFactor[89].imag()=-0.817585;TwiddleFactor[90].imag()=-0.803208;TwiddleFactor[91].imag()=-0.788346;TwiddleFactor[92].imag()=-0.773010;TwiddleFactor[93].imag()=-0.757209;TwiddleFactor[94].imag()=-0.740951;
	TwiddleFactor[95].imag()=-0.724247;TwiddleFactor[96].imag()=-0.707107;TwiddleFactor[97].imag()=-0.689541;TwiddleFactor[98].imag()=-0.671559;TwiddleFactor[99].imag()=-0.653173;TwiddleFactor[100].imag()=-0.634393;
	TwiddleFactor[101].imag()=-0.615232;TwiddleFactor[102].imag()=-0.595699;TwiddleFactor[103].imag()=-0.575808;TwiddleFactor[104].imag()=-0.555570;TwiddleFactor[105].imag()=-0.534998;TwiddleFactor[106].imag()=-0.514103;
	TwiddleFactor[107].imag()=-0.492898;TwiddleFactor[108].imag()=-0.471397;TwiddleFactor[109].imag()=-0.449611;TwiddleFactor[110].imag()=-0.427555;TwiddleFactor[111].imag()=-0.405241;TwiddleFactor[112].imag()=-0.382683;
	TwiddleFactor[113].imag()=-0.359895;TwiddleFactor[114].imag()=-0.336890;TwiddleFactor[115].imag()=-0.313682;TwiddleFactor[116].imag()=-0.290285;TwiddleFactor[117].imag()=-0.266713;TwiddleFactor[118].imag()=-0.242980;
	TwiddleFactor[119].imag()=-0.219101;TwiddleFactor[120].imag()=-0.195090;TwiddleFactor[121].imag()=-0.170962;TwiddleFactor[122].imag()=-0.146730;TwiddleFactor[123].imag()=-0.122411;TwiddleFactor[124].imag()=-0.098017;
	TwiddleFactor[125].imag()=-0.073565;TwiddleFactor[126].imag()=-0.049068;TwiddleFactor[127].imag()=-0.024541;TwiddleFactor[128].imag()=-0.000000;TwiddleFactor[129].imag()=0.024541;TwiddleFactor[130].imag()=0.049068;
	TwiddleFactor[131].imag()=0.073565;TwiddleFactor[132].imag()=0.098017;TwiddleFactor[133].imag()=0.122411;TwiddleFactor[134].imag()=0.146730;TwiddleFactor[135].imag()=0.170962;TwiddleFactor[136].imag()=0.195090;
	TwiddleFactor[137].imag()=0.219101;TwiddleFactor[138].imag()=0.242980;TwiddleFactor[139].imag()=0.266713;TwiddleFactor[140].imag()=0.290285;TwiddleFactor[141].imag()=0.313682;TwiddleFactor[142].imag()=0.336890;
	TwiddleFactor[143].imag()=0.359895;TwiddleFactor[144].imag()=0.382683;TwiddleFactor[145].imag()=0.405241;TwiddleFactor[146].imag()=0.427555;TwiddleFactor[147].imag()=0.449611;TwiddleFactor[148].imag()=0.471397;
	TwiddleFactor[149].imag()=0.492898;TwiddleFactor[150].imag()=0.514103;TwiddleFactor[151].imag()=0.534998;TwiddleFactor[152].imag()=0.555570;TwiddleFactor[153].imag()=0.575808;TwiddleFactor[154].imag()=0.595699;
	TwiddleFactor[155].imag()=0.615232;TwiddleFactor[156].imag()=0.634393;TwiddleFactor[157].imag()=0.653173;TwiddleFactor[158].imag()=0.671559;TwiddleFactor[159].imag()=0.689541;TwiddleFactor[160].imag()=0.707107;
	TwiddleFactor[161].imag()=0.724247;TwiddleFactor[162].imag()=0.740951;TwiddleFactor[163].imag()=0.757209;TwiddleFactor[164].imag()=0.773010;TwiddleFactor[165].imag()=0.788346;TwiddleFactor[166].imag()=0.803208;
	TwiddleFactor[167].imag()=0.817585;TwiddleFactor[168].imag()=0.831470;TwiddleFactor[169].imag()=0.844854;TwiddleFactor[170].imag()=0.857729;TwiddleFactor[171].imag()=0.870087;TwiddleFactor[172].imag()=0.881921;
	TwiddleFactor[173].imag()=0.893224;TwiddleFactor[174].imag()=0.903989;TwiddleFactor[175].imag()=0.914210;TwiddleFactor[176].imag()=0.923880;TwiddleFactor[177].imag()=0.932993;TwiddleFactor[178].imag()=0.941544;
	TwiddleFactor[179].imag()=0.949528;TwiddleFactor[180].imag()=0.956940;TwiddleFactor[181].imag()=0.963776;TwiddleFactor[182].imag()=0.970031;TwiddleFactor[183].imag()=0.975702;TwiddleFactor[184].imag()=0.980785;
	TwiddleFactor[185].imag()=0.985278;TwiddleFactor[186].imag()=0.989177;TwiddleFactor[187].imag()=0.992480;TwiddleFactor[188].imag()=0.995185;TwiddleFactor[189].imag()=0.997290;TwiddleFactor[190].imag()=0.998795;
	TwiddleFactor[191].imag()=0.999699;TwiddleFactor[192].imag()=1.000000;TwiddleFactor[193].imag()=0.999699;TwiddleFactor[194].imag()=0.998795;TwiddleFactor[195].imag()=0.997290;TwiddleFactor[196].imag()=0.995185;
	TwiddleFactor[197].imag()=0.992480;TwiddleFactor[198].imag()=0.989177;TwiddleFactor[199].imag()=0.985278;TwiddleFactor[200].imag()=0.980785;TwiddleFactor[201].imag()=0.975702;TwiddleFactor[202].imag()=0.970031;
	TwiddleFactor[203].imag()=0.963776;TwiddleFactor[204].imag()=0.956940;TwiddleFactor[205].imag()=0.949528;TwiddleFactor[206].imag()=0.941544;TwiddleFactor[207].imag()=0.932993;TwiddleFactor[208].imag()=0.923880;
	TwiddleFactor[209].imag()=0.914210;TwiddleFactor[210].imag()=0.903989;TwiddleFactor[211].imag()=0.893224;TwiddleFactor[212].imag()=0.881921;TwiddleFactor[213].imag()=0.870087;TwiddleFactor[214].imag()=0.857729;
	TwiddleFactor[215].imag()=0.844854;TwiddleFactor[216].imag()=0.831470;TwiddleFactor[217].imag()=0.817585;TwiddleFactor[218].imag()=0.803208;TwiddleFactor[219].imag()=0.788346;TwiddleFactor[220].imag()=0.773010;
	TwiddleFactor[221].imag()=0.757209;TwiddleFactor[222].imag()=0.740951;TwiddleFactor[223].imag()=0.724247;TwiddleFactor[224].imag()=0.707107;TwiddleFactor[225].imag()=0.689541;TwiddleFactor[226].imag()=0.671559;
	TwiddleFactor[227].imag()=0.653173;TwiddleFactor[228].imag()=0.634393;TwiddleFactor[229].imag()=0.615232;TwiddleFactor[230].imag()=0.595699;TwiddleFactor[231].imag()=0.575808;TwiddleFactor[232].imag()=0.555570;
	TwiddleFactor[233].imag()=0.534998;TwiddleFactor[234].imag()=0.514103;TwiddleFactor[235].imag()=0.492898;TwiddleFactor[236].imag()=0.471397;TwiddleFactor[237].imag()=0.449611;TwiddleFactor[238].imag()=0.427555;
	TwiddleFactor[239].imag()=0.405241;TwiddleFactor[240].imag()=0.382683;TwiddleFactor[241].imag()=0.359895;TwiddleFactor[242].imag()=0.336890;TwiddleFactor[243].imag()=0.313682;TwiddleFactor[244].imag()=0.290285;
	TwiddleFactor[245].imag()=0.266713;TwiddleFactor[246].imag()=0.242980;TwiddleFactor[247].imag()=0.219101;TwiddleFactor[248].imag()=0.195090;TwiddleFactor[249].imag()=0.170962;TwiddleFactor[250].imag()=0.146730;
	TwiddleFactor[251].imag()=0.122411;TwiddleFactor[252].imag()=0.098017;TwiddleFactor[253].imag()=0.073565;TwiddleFactor[254].imag()=0.049068;TwiddleFactor[255].imag()=0.024541;

	int IdxTable[NSTAGE][NFFT/4]={
		{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63},
		{0,0,0,0,4,4,4,4,8,8,8,8,12,12,12,12,16,16,16,16,20,20,20,20,24,24,24,24,28,28,28,28,32,32,32,32,36,36,36,36,40,40,40,40,44,44,44,44,48,48,48,48,52,52,52,52,56,56,56,56,60,60,60,60},
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
	};

	int output_idx[256] = { 0,128,64,192,32,160,96,224,16,144,80,208,48,176,112,240,8,136,72,200,40,168,104,232,24,152,88,216,56,184,120,248,4,132,68,196,36,164,100,228,20,148,84,212,52,180,116,244,12,140,76,204,44,172,108,236,28,156,92,220,60,188,124,252,2,130,66,194,34,162,98,226,18,146,82,210,50,178,114,242,10,138,74,202,42,170,106,234,26,154,90,218,58,186,122,250,6,134,70,198,38,166,102,230,22,150,86,214,54,182,118,246,14,142,78,206,46,174,110,238,30,158,94,222,62,190,126,254,1,129,65,193,33,161,97,225,17,145,81,209,49,177,113,241,9,137,73,201,41,169,105,233,25,153,89,217,57,185,121,249,5,133,69,197,37,165,101,229,21,149,85,213,53,181,117,245,13,141,77,205,45,173,109,237,29,157,93,221,61,189,125,253,3,131,67,195,35,163,99,227,19,147,83,211,51,179,115,243,11,139,75,203,43,171,107,235,27,155,91,219,59,187,123,251,7,135,71,199,39,167,103,231,23,151,87,215,55,183,119,247,15,143,79,207,47,175,111,239,31,159,95,223,63,191,127,255};

	cfix_W14_F13 x[4];
#pragma HLS ARRAY_PARTITION variable=x dim=0

	/*  This is a radix-4 FFT, using decimation in frequency */
	/*  The input signal is fixed point */
	/*  Works with real or complex input */
	/*  NOTE: The length of the input signal should be a power of 4: 4, 16, 64, 256, etc. */

	cfix_W14_F13   Ao;
	cfix_W14_F13   Bo;
	cfix_W14_F13   Co;
	cfix_W14_F13   Do;

	/////////////////////////////////////////////
	//  Calculate butterflies for first stages //
	/////////////////////////////////////////////
	cfix_W14_F13 stage0[NFFT];
	#pragma HLS ARRAY_PARTITION variable=stage0 complete dim=0

	int b_n;
	ap_uint<4> sh = 2;

   int stage=0;
	Stg0Lv1:for (int  n = 0; n < 64; n++) {
#pragma HLS UNROLL
		Stg0Lv2:for (int m = 0; m < 4; m++) {
#pragma HLS UNROLL
			// Bitwidth conversion
			x[m].real()(13,4) = in_fft[n + m*64](9,0);
			x[m].real()(3,0) = 0;
			x[m].imag() = ifix_W14_F13(0);

#ifndef __SYNTHESIS__
//	std::cout << setw(32) << "signedX : " << signedX << std::endl;
//	std::cout << " x  : " << x[n].real() << "+1i*" << x[n].imag() << std::endl;
#endif
		}

		radix4bfly(
			x,
			IdxTable[stage][n],
			1,
			TwiddleFactor,
			&Ao,
			&Bo,
			&Co,
			&Do
		);
#ifndef __SYNTHESIS__
//		cout << "Ao: " << Ao <<  "  Bo: " << Bo <<  "  Co:" << Co <<  "  Do:" << Do << endl;
#endif
		b_n = n << sh;
		stage0[b_n  ] = Ao;	// bit truncation
		stage0[b_n+1] = Bo;
		stage0[b_n+2] = Co;
		stage0[b_n+3] = Do;
	} // StgLv1:for (int  n = 0; n < 64; n++) {


#ifndef __SYNTHESIS__
	std::cout << "-----------------------Stage0--------------------------- " << std::endl;
//	ofstream st0;
//	st0.open("./stage0.dat");
	for (int  n = 0; n < NFFT; n++) {
//		st0 << setw(32) << stage0[n].real() <<"+1i*"<< stage0[n].imag() << endl;
		std::cout << stage0[n].real() <<"+1i*"<< stage0[n].imag() << std::endl;
	}
	std::cout << std::endl;

//	st0.close();
#endif

	//////////////////////////////////////////////
	//  Calculate butterflies for second stages //
	//////////////////////////////////////////////

	cfix_W14_F13 stage1[NFFT];
#pragma HLS ARRAY_PARTITION variable=stage1 complete dim=0

	stage = 1;
	Stg1Lv1:for (int  n = 0; n < 64; n++) {
#pragma HLS UNROLL
		Stg1Lv2:for (int m = 0; m < 4; m++) {
#pragma HLS UNROLL
			x[m] = stage0[n + m*64];

#ifndef __SYNTHESIS__
//				cout << "x[m]" << x[m] << endl;
#endif
		}

		// Sub-function
		radix4bfly(
			x,
			IdxTable[stage][n],
			1,
			TwiddleFactor,
			&Ao,
			&Bo,
			&Co,
			&Do
		);
#ifndef __SYNTHESIS__
//		cout << "Ao: " << Ao <<  "  Bo: " << Bo <<  "  Co:" << Co <<  "  Do:" << Do << endl;
#endif
		b_n = n << sh;
		stage1[b_n  ] = Ao;	// bit truncation
		stage1[b_n+1] = Bo;
		stage1[b_n+2] = Co;
		stage1[b_n+3] = Do;
	} // StgLv2:for (int  n = 0; n < 64; n++) {

#ifndef __SYNTHESIS__
//	ofstream st1;
//	st1.open("./stage1.dat");
//	for (int  n = 0; n < NFFT; n++) {
//		st1 << setw(32) << stage1[n].real() <<"+1i*"<< stage1[n].imag() << endl;
//	}
//	st1.close();
	std::cout << "-----------------------Stage1--------------------------- " << std::endl;
	for (int  n = 0; n < NFFT; n++) {
		std::cout << stage1[n].real() <<"+1i*"<< stage1[n].imag() << std::endl;
	}
	std::cout << std::endl;

#endif


	/////////////////////////////////////////////
	/*  Calculate butterflies for third stages */
	/////////////////////////////////////////////

	cfix_W14_F13 stage2[NFFT];
#pragma HLS ARRAY_PARTITION variable=stage2 complete dim=0

	stage = 2;
	Stg2Lv1:for (int  n = 0; n < 64; n++) {
#pragma HLS UNROLL
		Stg2Lv2:for (int m = 0; m < 4; m++) {
#pragma HLS UNROLL
			x[m] = stage1[n + m*64];
//				cout << "x[m]" << x[m] << endl;
		}

		// Sub-function
		radix4bfly(
			x,
			IdxTable[stage][n],
			1,
			TwiddleFactor,
			&Ao,
			&Bo,
			&Co,
			&Do
		);
#ifndef __SYNTHESIS__
//		cout << "Ao: " << Ao <<  "  Bo: " << Bo <<  "  Co:" << Co <<  "  Do:" << Do << endl;
#endif
		b_n = n << sh;
		stage2[b_n  ] = Ao;	// bit truncation
		stage2[b_n+1] = Bo;
		stage2[b_n+2] = Co;
		stage2[b_n+3] = Do;

	} // StgLv3:for (int  n = 0; n < 64; n++) {

#ifndef __SYNTHESIS__
//	ofstream st2;
//	st2.open("./stage2.dat");
//	for (int  n = 0; n < NFFT; n++) {
//		st2 << setw(32) << stage2[n].real() <<"+1i*"<< stage2[n].imag() << endl;
//		cout << stage2[n].real() <<"+1i*"<< stage2[n].imag() << endl;
//	}
//	st2.close();

	std::cout << "-----------------------Stage2--------------------------- " << std::endl;
	for (int  n = 0; n < NFFT; n++) {
		std::cout << stage2[n].real() <<"+1i*"<< stage2[n].imag() << std::endl;
	}
	std::cout << std::endl;

#endif


	/////////////////////////////////////////////
	/*  Calculate butterflies for last stage */
	/////////////////////////////////////////////

	cfix_W14_F13 stage3[NFFT];
#pragma HLS ARRAY_PARTITION variable=stage3 complete dim=0

	stage=3;
	Stg3Lv1:for (int n = 0; n < 64; n++) {
#pragma HLS UNROLL
		Stg3Lv2:for (int m = 0; m < 4; m++) {
#pragma HLS UNROLL
			x[m] = stage2[n + m*64];
		}

		radix4bfly(
			x,
			IdxTable[stage][n],
			0,
			TwiddleFactor,
			&Ao,
			&Bo,
			&Co,
			&Do
		);
		b_n = n << sh;
		stage3[b_n  ] = Ao;	// bit truncation
		stage3[b_n+1] = Bo;
		stage3[b_n+2] = Co;
		stage3[b_n+3] = Do;
	} // for (int n = 0; n < NFFT/4; n++)

#ifndef __SYNTHESIS__
//	ofstream st3;
//	st3.open("./stage3.dat");
//	for (int  n = 0; n < NFFT; n++) {
//		st3 << setw(32) << stage3[n].real() <<"+1i*"<< stage3[n].imag() << endl;
//		cout << stage3[n].real() <<"+1i*"<< stage3[n].imag() << endl;
//	}
//	st3.close();

	cout << "-----------------------Stage3--------------------------- " <<endl;
	for (int  n = 0; n < NFFT; n++) {
		std::cout << stage3[n].real() <<"+1i*"<< stage3[n].imag() << std::endl;
	}
	std::cout << std::endl;
#endif


cfix_W14_F5 stage_out[NFFT];
#pragma HLS ARRAY_PARTITION variable=stage_out complete dim=0

//	Scaling:for (int i=0; i<NFFT; i++) {
//#pragma HLS UNROLL
//		/*  Rescale the final output */
//
//		// The shift left operation result type is same as the type of operand
//		// Operand must be cat to result type
//		// refer to p.654, ug902 - 2017
//
//		stage_out[i].real()(13,0)   = stage3[i].real()(13,0); // bit range selection
//		stage_out[i].imag()(13,0)   = stage3[i].imag()(13,0); // bit range selection
//	}
//
//	/* Permute input into bit-reversed order */
//	Revr:for (int i=0; i<NFFT; i++) {
//		// Implict type casting, from W14_F5 to from W14_F8
//		out_fft[i].real()[13] = stage_out[output_idx[i]].real()[13];
//		out_fft[i].imag()[13] = stage_out[output_idx[i]].imag()[13];
//
//		out_fft[i].real()(12,3) = stage_out[output_idx[i]].real()(9,0);
//		out_fft[i].imag()(12,3) = stage_out[output_idx[i]].imag()(9,0);
//
//		out_fft[i].real()(2,0) = 0;
//		out_fft[i].imag()(2,0) = 0;
//
//	}

	SnR:for (int i=0; i<NFFT; i++) {
	#pragma HLS UNROLL
		/*  Rescale and Permute input into bit-reversed order */

		// The shift left operation result type is same as the type of operand
		// Operand must be cat to result type
		// refer to p.654, ug902 - 2017

		out_fft[i].real()[13]   = stage3[output_idx[i]].real()[13];
		out_fft[i].imag()[13]   = stage3[output_idx[i]].imag()[13];

		out_fft[i].real()(12,3) = stage3[output_idx[i]].real()(9,0);
		out_fft[i].imag()(12,3) = stage3[output_idx[i]].imag()(9,0);

		out_fft[i].real()(2,0)  = 0;
		out_fft[i].imag()(2,0)  = 0;

	}
#ifndef __SYNTHESIS__
	std::cout << "----------------------- scaling stage --------------------------- " << std::endl;
//	ofstream stend;
//	stend.open("./stageEnd.dat");
//	for (int  n = 0; n < NFFT; n++) {
//		stend << setw(32) << stage_out[n].real() <<"+1i*"<< stage_out[n].imag() << endl;
//		cout << stage_out[n].real() <<"+1i*"<< stage_out[n].imag() << endl;
//	}
//	stend.close();

	for (int  n = 0; n < NFFT; n++) {
		std::cout << stage_out[n].real() <<"+1i*"<< stage_out[n].imag() << std::endl;
	}
	std::cout << std::endl;


	std::cout << "----------------------- reverse stage --------------------------- " << std::endl;
//	ofstream stend;
//	stend.open("./stageEnd.dat");
//	for (int  n = 0; n < NFFT; n++) {
//		stend << setw(32) << stage_out[n].real() <<"+1i*"<< stage_out[n].imag() << endl;
//		cout << stage_out[n].real() <<"+1i*"<< stage_out[n].imag() << endl;
//	}
//	stend.close();

	for (int  n = 0; n < NFFT; n++) {
		std::cout << out_fft[n].real() <<"+1i*"<< out_fft[n].imag() << std::endl;
	}
	std::cout << std::endl;
#endif


#ifndef __SYNTHESIS__
//	ofstream stout;
//	stout.open("./fftout.dat");
//	for (int  n = 0; n < NFFT; n++) {
//		stout << setw(32) << out_fft[n].real() <<"+1i*"<< out_fft[n].imag() << endl;
//		cout << out_fft[n].real() <<"+1i*"<< out_fft[n].imag() << endl;
//	}
//	stout.close();
#endif
}


/* Function Definitions */
void radix4bfly(
	cfix_W14_F13*   x,
	int 		    segment,
	int 		    stageFlag,
	cfix_W14_F12*   TwiddleFactor,
	cfix_W14_F13*   A,
	cfix_W14_F13*   B,
	cfix_W14_F13*   C,
	cfix_W14_F13*   D
){
	#pragma HLS PIPELINE II=2
//	BU_Lv3:for (unsigned int m = 0; m < NFFT; m++) {
//
//		cout << "TwiddleFactor[" <<m << "] : " << TwiddleFactor[m] << endl;
//	}

	cfix_W14_F13 aa;
	cfix_W14_F13 bb;
	cfix_W14_F13 cc;
	cfix_W14_F13 dd; //	cfix_W28_F27 dd;

	/*  For the last stage of a radix-4 FFT all the ABCD multipliers are 1. */
	/*  Use the stageFlag variable to indicate the last stage */
	/*  stageFlag = 0 indicates last FFT stage, set to 1 otherwise */
	/*  Initialize variables and scale by 1/4 */

	ap_uint<4> shL = 2;

	aa.real() = x[0].real() >> shL;
	aa.imag() = x[0].imag() >> shL;

	bb.real() = x[1].real() >> shL;
	bb.imag() = x[1].imag() >> shL;

	cc.real() = x[2].real() >> shL;
	cc.imag() = x[2].imag() >> shL;

	dd.real() = x[3].real() >> shL;
	dd.imag() = x[3].imag() >> shL;


	/*  Radix-4 Algorithm */
	cfix_W14_F12 TF;

	// A=a+b+c+d;
	A->real() = aa.real() + bb.real() + cc.real() + dd.real();
	A->imag() = aa.imag() + bb.imag() + cc.imag() + dd.imag();

	// B=(a-b+c-d)*W(2*segment*stageFlag + 1);
	cfix_W14_F12 H_B;
	H_B.real() = aa.real()-bb.real() + cc.real() - dd.real();
	H_B.imag() = aa.imag()-bb.imag() + cc.imag() - dd.imag();
	TF = TwiddleFactor[2*segment+stageFlag+1];

	cfix_W29_F24 iB;
	hls::cmpy<ARCH_CMPY>(H_B, TF, iB);
	*B	= (cfix_W14_F13)iB;

	// C=(a-b*1i-c+d*1i)*W(segment*stageFlag + 1);
	cfix_W14_F12 H_C;

	H_C.real() =  aa.real()+bb.imag()-cc.real()-dd.imag();
	H_C.imag() =  aa.imag()-bb.real()-cc.imag()+dd.real();
	TF = TwiddleFactor[segment+stageFlag+1];

	cfix_W29_F24 iC;
	hls::cmpy<ARCH_CMPY>(H_C, TF, iC);
	*C	= (cfix_W14_F13)iC;

	// D=(a+b*1i-c-d*1i)*W(3*segment*stageFlag + 1);
	cfix_W14_F12 H_D;
	H_D.real() =  aa.real()-bb.imag()-cc.real()+dd.imag();
	H_D.imag() =  aa.imag()+bb.real()-cc.imag()-dd.real();
	TF = TwiddleFactor[3*segment+stageFlag+1];

	cfix_W29_F24 iD;
	hls::cmpy<ARCH_CMPY>(H_D, TF, iD);
	*D	= (cfix_W14_F13)iD;

#ifndef __SYNTHESIS__
//		cout << "H_B: " << H_B << endl;
//		cout << "H_D: " << H_D << endl;
//		cout << "H_C: " << H_C << endl;
//
//		cout << "A: " << *A << endl;
//		cout << "B: " << *B << endl;
//		cout << "C: " << *C << endl;
//		cout << "D: " << *D << endl;
	if (stageFlag == 0){
//		cout << "aa: " << aa <<  "  bb: " << bb <<  "  cc:" << cc <<  "  dd:" << dd << endl;
//
//
//		cout << "H_B: " << H_B << endl;
//		cout << "H_D: " << H_D << endl;
//		cout << "H_C: " << H_C << endl;
//
//		cout << "A: " << *A << endl;
//		cout << "B: " << *B << endl;
//		cout << "C: " << *C << endl;
//		cout << "D: " << *D << endl;
	}
#endif
}

