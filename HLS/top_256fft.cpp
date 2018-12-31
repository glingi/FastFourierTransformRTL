#include "top_256fft.h"
using namespace std;
void radix4FFT3_FixPtEML(
		ifix_W10_F9 in_fft[NFFT],	// Input is only real
						// FMC126 ADC only supports unsigned data
		cfix_W14_F8 out_fft[NFFT]	// for expander
)
{

	// #pragma HLS PIPELINE //not available

	// Reshape an array from one with many elements to one with greater word-width. 
	// Useful for improving block RAM accesses without using more block RAM.
	// Refer to UG902 p. 131
#pragma HLS INTERFACE ap_vld port=in_fft
#pragma HLS ARRAY_RESHAPE variable=in_fft
#pragma HLS ARRAY_RESHAPE variable=out_fft

	// Twiddle factor initialization 
	//
	// The ap_[u]fixed types do not support initialization if they are used in an array of std::complex types.
	// typedef ap_fixed<DIN_W, 1, AP_TRN, AP_SAT> coeff_t; // MUST have IW >= 1
	// std::complex<coeff_t> twid_rom[REAL_SZ/2] = {{ 1, -0 },{ 0.9,-0.006 }, etc.}
	//
	// The initialization values must first be case to std:complex:
	// typedef ap_fixed<DIN_W, 1, AP_TRN, AP_SAT> coeff_t; // MUST have IW >= 1
	// std::complex<coeff_t> twid_rom[REAL_SZ/2] = {std::complex<coeff_t>( 1, -0 ),
	// std::complex<coeff_t>(0.9,-0.006 ),etc.}
	// Refer to UG902 p. 646
	
	cfix_W14_F12 TwiddleFactor[256];
	TwiddleFactor[0] = cfix_W14_F12(1.000000, 0.000000); TwiddleFactor[1] = cfix_W14_F12(0.999699, -0.024541); TwiddleFactor[2] = cfix_W14_F12(0.998795, -0.049068); TwiddleFactor[3] = cfix_W14_F12(0.997290, -0.073565); TwiddleFactor[4] = cfix_W14_F12(0.995185, -0.098017);
	TwiddleFactor[5] = cfix_W14_F12(0.992480, -0.122411); TwiddleFactor[6] = cfix_W14_F12(0.989177, -0.146730); TwiddleFactor[7] = cfix_W14_F12(0.985278, -0.170962); TwiddleFactor[8] = cfix_W14_F12(0.980785, -0.195090); TwiddleFactor[9] = cfix_W14_F12(0.975702, -0.219101); TwiddleFactor[10] = cfix_W14_F12(0.970031, -0.242980);
	TwiddleFactor[11] = cfix_W14_F12(0.963776, -0.266713); TwiddleFactor[12] = cfix_W14_F12(0.956940, -0.290285); TwiddleFactor[13] = cfix_W14_F12(0.949528, -0.313682); TwiddleFactor[14] = cfix_W14_F12(0.941544, -0.336890); TwiddleFactor[15] = cfix_W14_F12(0.932993, -0.359895); TwiddleFactor[16] = cfix_W14_F12(0.923880, -0.382683);
	TwiddleFactor[17] = cfix_W14_F12(0.914210, -0.405241); TwiddleFactor[18] = cfix_W14_F12(0.903989, -0.427555); TwiddleFactor[19] = cfix_W14_F12(0.893224, -0.449611); TwiddleFactor[20] = cfix_W14_F12(0.881921, -0.471397); TwiddleFactor[21] = cfix_W14_F12(0.870087, -0.492898); TwiddleFactor[22] = cfix_W14_F12(0.857729, -0.514103);
	TwiddleFactor[23] = cfix_W14_F12(0.844854, -0.534998); TwiddleFactor[24] = cfix_W14_F12(0.831470, -0.555570); TwiddleFactor[25] = cfix_W14_F12(0.817585, -0.575808); TwiddleFactor[26] = cfix_W14_F12(0.803208, -0.595699); TwiddleFactor[27] = cfix_W14_F12(0.788346, -0.615232); TwiddleFactor[28] = cfix_W14_F12(0.773010, -0.634393);
	TwiddleFactor[29] = cfix_W14_F12(0.757209, -0.653173); TwiddleFactor[30] = cfix_W14_F12(0.740951, -0.671559); TwiddleFactor[31] = cfix_W14_F12(0.724247, -0.689541); TwiddleFactor[32] = cfix_W14_F12(0.707107, -0.707107); TwiddleFactor[33] = cfix_W14_F12(0.689541, -0.724247); TwiddleFactor[34] = cfix_W14_F12(0.671559, -0.740951);
	TwiddleFactor[35] = cfix_W14_F12(0.653173, -0.757209); TwiddleFactor[36] = cfix_W14_F12(0.634393, -0.773010); TwiddleFactor[37] = cfix_W14_F12(0.615232, -0.788346); TwiddleFactor[38] = cfix_W14_F12(0.595699, -0.803208); TwiddleFactor[39] = cfix_W14_F12(0.575808, -0.817585); TwiddleFactor[40] = cfix_W14_F12(0.555570, -0.831470);
	TwiddleFactor[41] = cfix_W14_F12(0.534998, -0.844854); TwiddleFactor[42] = cfix_W14_F12(0.514103, -0.857729); TwiddleFactor[43] = cfix_W14_F12(0.492898, -0.870087); TwiddleFactor[44] = cfix_W14_F12(0.471397, -0.881921); TwiddleFactor[45] = cfix_W14_F12(0.449611, -0.893224); TwiddleFactor[46] = cfix_W14_F12(0.427555, -0.903989);
	TwiddleFactor[47] = cfix_W14_F12(0.405241, -0.914210); TwiddleFactor[48] = cfix_W14_F12(0.382683, -0.923880); TwiddleFactor[49] = cfix_W14_F12(0.359895, -0.932993); TwiddleFactor[50] = cfix_W14_F12(0.336890, -0.941544); TwiddleFactor[51] = cfix_W14_F12(0.313682, -0.949528); TwiddleFactor[52] = cfix_W14_F12(0.290285, -0.956940);
	TwiddleFactor[53] = cfix_W14_F12(0.266713, -0.963776); TwiddleFactor[54] = cfix_W14_F12(0.242980, -0.970031); TwiddleFactor[55] = cfix_W14_F12(0.219101, -0.975702); TwiddleFactor[56] = cfix_W14_F12(0.195090, -0.980785); TwiddleFactor[57] = cfix_W14_F12(0.170962, -0.985278); TwiddleFactor[58] = cfix_W14_F12(0.146730, -0.989177);
	TwiddleFactor[59] = cfix_W14_F12(0.122411, -0.992480); TwiddleFactor[60] = cfix_W14_F12(0.098017, -0.995185); TwiddleFactor[61] = cfix_W14_F12(0.073565, -0.997290); TwiddleFactor[62] = cfix_W14_F12(0.049068, -0.998795); TwiddleFactor[63] = cfix_W14_F12(0.024541, -0.999699); TwiddleFactor[64] = cfix_W14_F12(0.000000, -1.000000);
	TwiddleFactor[65] = cfix_W14_F12(-0.024541, -0.999699); TwiddleFactor[66] = cfix_W14_F12(-0.049068, -0.998795); TwiddleFactor[67] = cfix_W14_F12(-0.073565, -0.997290); TwiddleFactor[68] = cfix_W14_F12(-0.098017, -0.995185); TwiddleFactor[69] = cfix_W14_F12(-0.122411, -0.992480); TwiddleFactor[70] = cfix_W14_F12(-0.146730, -0.989177);
	TwiddleFactor[71] = cfix_W14_F12(-0.170962, -0.985278); TwiddleFactor[72] = cfix_W14_F12(-0.195090, -0.980785); TwiddleFactor[73] = cfix_W14_F12(-0.219101, -0.975702); TwiddleFactor[74] = cfix_W14_F12(-0.242980, -0.970031); TwiddleFactor[75] = cfix_W14_F12(-0.266713, -0.963776); TwiddleFactor[76] = cfix_W14_F12(-0.290285, -0.956940);
	TwiddleFactor[77] = cfix_W14_F12(-0.313682, -0.949528); TwiddleFactor[78] = cfix_W14_F12(-0.336890, -0.941544); TwiddleFactor[79] = cfix_W14_F12(-0.359895, -0.932993); TwiddleFactor[80] = cfix_W14_F12(-0.382683, -0.923880); TwiddleFactor[81] = cfix_W14_F12(-0.405241, -0.914210); TwiddleFactor[82] = cfix_W14_F12(-0.427555, -0.903989);
	TwiddleFactor[83] = cfix_W14_F12(-0.449611, -0.893224); TwiddleFactor[84] = cfix_W14_F12(-0.471397, -0.881921); TwiddleFactor[85] = cfix_W14_F12(-0.492898, -0.870087); TwiddleFactor[86] = cfix_W14_F12(-0.514103, -0.857729); TwiddleFactor[87] = cfix_W14_F12(-0.534998, -0.844854); TwiddleFactor[88] = cfix_W14_F12(-0.555570, -0.831470);
	TwiddleFactor[89] = cfix_W14_F12(-0.575808, -0.817585); TwiddleFactor[90] = cfix_W14_F12(-0.595699, -0.803208); TwiddleFactor[91] = cfix_W14_F12(-0.615232, -0.788346); TwiddleFactor[92] = cfix_W14_F12(-0.634393, -0.773010); TwiddleFactor[93] = cfix_W14_F12(-0.653173, -0.757209); TwiddleFactor[94] = cfix_W14_F12(-0.671559, -0.740951);
	TwiddleFactor[95] = cfix_W14_F12(-0.689541, -0.724247); TwiddleFactor[96] = cfix_W14_F12(-0.707107, -0.707107); TwiddleFactor[97] = cfix_W14_F12(-0.724247, -0.689541); TwiddleFactor[98] = cfix_W14_F12(-0.740951, -0.671559); TwiddleFactor[99] = cfix_W14_F12(-0.757209, -0.653173); TwiddleFactor[100] = cfix_W14_F12(-0.773010, -0.634393);
	TwiddleFactor[101] = cfix_W14_F12(-0.788346, -0.615232); TwiddleFactor[102] = cfix_W14_F12(-0.803208, -0.595699); TwiddleFactor[103] = cfix_W14_F12(-0.817585, -0.575808); TwiddleFactor[104] = cfix_W14_F12(-0.831470, -0.555570); TwiddleFactor[105] = cfix_W14_F12(-0.844854, -0.534998); TwiddleFactor[106] = cfix_W14_F12(-0.857729, -0.514103);
	TwiddleFactor[107] = cfix_W14_F12(-0.870087, -0.492898); TwiddleFactor[108] = cfix_W14_F12(-0.881921, -0.471397); TwiddleFactor[109] = cfix_W14_F12(-0.893224, -0.449611); TwiddleFactor[110] = cfix_W14_F12(-0.903989, -0.427555); TwiddleFactor[111] = cfix_W14_F12(-0.914210, -0.405241); TwiddleFactor[112] = cfix_W14_F12(-0.923880, -0.382683);
	TwiddleFactor[113] = cfix_W14_F12(-0.932993, -0.359895); TwiddleFactor[114] = cfix_W14_F12(-0.941544, -0.336890); TwiddleFactor[115] = cfix_W14_F12(-0.949528, -0.313682); TwiddleFactor[116] = cfix_W14_F12(-0.956940, -0.290285); TwiddleFactor[117] = cfix_W14_F12(-0.963776, -0.266713); TwiddleFactor[118] = cfix_W14_F12(-0.970031, -0.242980);
	TwiddleFactor[119] = cfix_W14_F12(-0.975702, -0.219101); TwiddleFactor[120] = cfix_W14_F12(-0.980785, -0.195090); TwiddleFactor[121] = cfix_W14_F12(-0.985278, -0.170962); TwiddleFactor[122] = cfix_W14_F12(-0.989177, -0.146730); TwiddleFactor[123] = cfix_W14_F12(-0.992480, -0.122411); TwiddleFactor[124] = cfix_W14_F12(-0.995185, -0.098017);
	TwiddleFactor[125] = cfix_W14_F12(-0.997290, -0.073565); TwiddleFactor[126] = cfix_W14_F12(-0.998795, -0.049068); TwiddleFactor[127] = cfix_W14_F12(-0.999699, -0.024541); TwiddleFactor[128] = cfix_W14_F12(-1.000000, -0.000000); TwiddleFactor[129] = cfix_W14_F12(-0.999699, 0.024541); TwiddleFactor[130] = cfix_W14_F12(-0.998795, 0.049068);
	TwiddleFactor[131] = cfix_W14_F12(-0.997290, 0.073565); TwiddleFactor[132] = cfix_W14_F12(-0.995185, 0.098017); TwiddleFactor[133] = cfix_W14_F12(-0.992480, 0.122411); TwiddleFactor[134] = cfix_W14_F12(-0.989177, 0.146730); TwiddleFactor[135] = cfix_W14_F12(-0.985278, 0.170962); TwiddleFactor[136] = cfix_W14_F12(-0.980785, 0.195090);
	TwiddleFactor[137] = cfix_W14_F12(-0.975702, 0.219101); TwiddleFactor[138] = cfix_W14_F12(-0.970031, 0.242980); TwiddleFactor[139] = cfix_W14_F12(-0.963776, 0.266713); TwiddleFactor[140] = cfix_W14_F12(-0.956940, 0.290285); TwiddleFactor[141] = cfix_W14_F12(-0.949528, 0.313682); TwiddleFactor[142] = cfix_W14_F12(-0.941544, 0.336890);
	TwiddleFactor[143] = cfix_W14_F12(-0.932993, 0.359895); TwiddleFactor[144] = cfix_W14_F12(-0.923880, 0.382683); TwiddleFactor[145] = cfix_W14_F12(-0.914210, 0.405241); TwiddleFactor[146] = cfix_W14_F12(-0.903989, 0.427555); TwiddleFactor[147] = cfix_W14_F12(-0.893224, 0.449611); TwiddleFactor[148] = cfix_W14_F12(-0.881921, 0.471397);
	TwiddleFactor[149] = cfix_W14_F12(-0.870087, 0.492898); TwiddleFactor[150] = cfix_W14_F12(-0.857729, 0.514103); TwiddleFactor[151] = cfix_W14_F12(-0.844854, 0.534998); TwiddleFactor[152] = cfix_W14_F12(-0.831470, 0.555570); TwiddleFactor[153] = cfix_W14_F12(-0.817585, 0.575808); TwiddleFactor[154] = cfix_W14_F12(-0.803208, 0.595699);
	TwiddleFactor[155] = cfix_W14_F12(-0.788346, 0.615232); TwiddleFactor[156] = cfix_W14_F12(-0.773010, 0.634393); TwiddleFactor[157] = cfix_W14_F12(-0.757209, 0.653173); TwiddleFactor[158] = cfix_W14_F12(-0.740951, 0.671559); TwiddleFactor[159] = cfix_W14_F12(-0.724247, 0.689541); TwiddleFactor[160] = cfix_W14_F12(-0.707107, 0.707107);
	TwiddleFactor[161] = cfix_W14_F12(-0.689541, 0.724247); TwiddleFactor[162] = cfix_W14_F12(-0.671559, 0.740951); TwiddleFactor[163] = cfix_W14_F12(-0.653173, 0.757209); TwiddleFactor[164] = cfix_W14_F12(-0.634393, 0.773010); TwiddleFactor[165] = cfix_W14_F12(-0.615232, 0.788346); TwiddleFactor[166] = cfix_W14_F12(-0.595699, 0.803208);
	TwiddleFactor[167] = cfix_W14_F12(-0.575808, 0.817585); TwiddleFactor[168] = cfix_W14_F12(-0.555570, 0.831470); TwiddleFactor[169] = cfix_W14_F12(-0.534998, 0.844854); TwiddleFactor[170] = cfix_W14_F12(-0.514103, 0.857729); TwiddleFactor[171] = cfix_W14_F12(-0.492898, 0.870087); TwiddleFactor[172] = cfix_W14_F12(-0.471397, 0.881921);
	TwiddleFactor[173] = cfix_W14_F12(-0.449611, 0.893224); TwiddleFactor[174] = cfix_W14_F12(-0.427555, 0.903989); TwiddleFactor[175] = cfix_W14_F12(-0.405241, 0.914210); TwiddleFactor[176] = cfix_W14_F12(-0.382683, 0.923880); TwiddleFactor[177] = cfix_W14_F12(-0.359895, 0.932993); TwiddleFactor[178] = cfix_W14_F12(-0.336890, 0.941544);
	TwiddleFactor[179] = cfix_W14_F12(-0.313682, 0.949528); TwiddleFactor[180] = cfix_W14_F12(-0.290285, 0.956940); TwiddleFactor[181] = cfix_W14_F12(-0.266713, 0.963776); TwiddleFactor[182] = cfix_W14_F12(-0.242980, 0.970031); TwiddleFactor[183] = cfix_W14_F12(-0.219101, 0.975702); TwiddleFactor[184] = cfix_W14_F12(-0.195090, 0.980785);
	TwiddleFactor[185] = cfix_W14_F12(-0.170962, 0.985278); TwiddleFactor[186] = cfix_W14_F12(-0.146730, 0.989177); TwiddleFactor[187] = cfix_W14_F12(-0.122411, 0.992480); TwiddleFactor[188] = cfix_W14_F12(-0.098017, 0.995185); TwiddleFactor[189] = cfix_W14_F12(-0.073565, 0.997290); TwiddleFactor[190] = cfix_W14_F12(-0.049068, 0.998795);
	TwiddleFactor[191] = cfix_W14_F12(-0.024541, 0.999699); TwiddleFactor[192] = cfix_W14_F12(-0.000000, 1.000000); TwiddleFactor[193] = cfix_W14_F12(0.024541, 0.999699); TwiddleFactor[194] = cfix_W14_F12(0.049068, 0.998795); TwiddleFactor[195] = cfix_W14_F12(0.073565, 0.997290); TwiddleFactor[196] = cfix_W14_F12(0.098017, 0.995185);
	TwiddleFactor[197] = cfix_W14_F12(0.122411, 0.992480); TwiddleFactor[198] = cfix_W14_F12(0.146730, 0.989177); TwiddleFactor[199] = cfix_W14_F12(0.170962, 0.985278); TwiddleFactor[200] = cfix_W14_F12(0.195090, 0.980785); TwiddleFactor[201] = cfix_W14_F12(0.219101, 0.975702); TwiddleFactor[202] = cfix_W14_F12(0.242980, 0.970031);
	TwiddleFactor[203] = cfix_W14_F12(0.266713, 0.963776); TwiddleFactor[204] = cfix_W14_F12(0.290285, 0.956940); TwiddleFactor[205] = cfix_W14_F12(0.313682, 0.949528); TwiddleFactor[206] = cfix_W14_F12(0.336890, 0.941544); TwiddleFactor[207] = cfix_W14_F12(0.359895, 0.932993); TwiddleFactor[208] = cfix_W14_F12(0.382683, 0.923880);
	TwiddleFactor[209] = cfix_W14_F12(0.405241, 0.914210); TwiddleFactor[210] = cfix_W14_F12(0.427555, 0.903989); TwiddleFactor[211] = cfix_W14_F12(0.449611, 0.893224); TwiddleFactor[212] = cfix_W14_F12(0.471397, 0.881921); TwiddleFactor[213] = cfix_W14_F12(0.492898, 0.870087); TwiddleFactor[214] = cfix_W14_F12(0.514103, 0.857729);
	TwiddleFactor[215] = cfix_W14_F12(0.534998, 0.844854); TwiddleFactor[216] = cfix_W14_F12(0.555570, 0.831470); TwiddleFactor[217] = cfix_W14_F12(0.575808, 0.817585); TwiddleFactor[218] = cfix_W14_F12(0.595699, 0.803208); TwiddleFactor[219] = cfix_W14_F12(0.615232, 0.788346); TwiddleFactor[220] = cfix_W14_F12(0.634393, 0.773010);
	TwiddleFactor[221] = cfix_W14_F12(0.653173, 0.757209); TwiddleFactor[222] = cfix_W14_F12(0.671559, 0.740951); TwiddleFactor[223] = cfix_W14_F12(0.689541, 0.724247); TwiddleFactor[224] = cfix_W14_F12(0.707107, 0.707107); TwiddleFactor[225] = cfix_W14_F12(0.724247, 0.689541); TwiddleFactor[226] = cfix_W14_F12(0.740951, 0.671559);
	TwiddleFactor[227] = cfix_W14_F12(0.757209, 0.653173); TwiddleFactor[228] = cfix_W14_F12(0.773010, 0.634393); TwiddleFactor[229] = cfix_W14_F12(0.788346, 0.615232); TwiddleFactor[230] = cfix_W14_F12(0.803208, 0.595699); TwiddleFactor[231] = cfix_W14_F12(0.817585, 0.575808); TwiddleFactor[232] = cfix_W14_F12(0.831470, 0.555570);
	TwiddleFactor[233] = cfix_W14_F12(0.844854, 0.534998); TwiddleFactor[234] = cfix_W14_F12(0.857729, 0.514103); TwiddleFactor[235] = cfix_W14_F12(0.870087, 0.492898); TwiddleFactor[236] = cfix_W14_F12(0.881921, 0.471397); TwiddleFactor[237] = cfix_W14_F12(0.893224, 0.449611); TwiddleFactor[238] = cfix_W14_F12(0.903989, 0.427555);
	TwiddleFactor[239] = cfix_W14_F12(0.914210, 0.405241); TwiddleFactor[240] = cfix_W14_F12(0.923880, 0.382683); TwiddleFactor[241] = cfix_W14_F12(0.932993, 0.359895); TwiddleFactor[242] = cfix_W14_F12(0.941544, 0.336890); TwiddleFactor[243] = cfix_W14_F12(0.949528, 0.313682); TwiddleFactor[244] = cfix_W14_F12(0.956940, 0.290285);
	TwiddleFactor[245] = cfix_W14_F12(0.963776, 0.266713); TwiddleFactor[246] = cfix_W14_F12(0.970031, 0.242980); TwiddleFactor[247] = cfix_W14_F12(0.975702, 0.219101); TwiddleFactor[248] = cfix_W14_F12(0.980785, 0.195090); TwiddleFactor[249] = cfix_W14_F12(0.985278, 0.170962); TwiddleFactor[250] = cfix_W14_F12(0.989177, 0.146730);
	TwiddleFactor[251] = cfix_W14_F12(0.992480, 0.122411); TwiddleFactor[252] = cfix_W14_F12(0.995185, 0.098017); TwiddleFactor[253] = cfix_W14_F12(0.997290, 0.073565); TwiddleFactor[254] = cfix_W14_F12(0.998795, 0.049068); TwiddleFactor[255] = cfix_W14_F12(0.999699, 0.024541);

	int IdxTable[NSTAGE][NFFT/4]={
		{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63},
		{0,0,0,0,4,4,4,4,8,8,8,8,12,12,12,12,16,16,16,16,20,20,20,20,24,24,24,24,28,28,28,28,32,32,32,32,36,36,36,36,40,40,40,40,44,44,44,44,48,48,48,48,52,52,52,52,56,56,56,56,60,60,60,60},
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48},
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
	};

	int output_idx[256] = { 0,128,64,192,32,160,96,224,16,144,80,208,48,176,112,240,8,136,72,200,40,168,104,232,24,152,88,216,56,184,120,248,4,132,68,196,36,164,100,228,20,148,84,212,52,180,116,244,12,140,76,204,44,172,108,236,28,156,92,220,60,188,124,252,2,130,66,194,34,162,98,226,18,146,82,210,50,178,114,242,10,138,74,202,42,170,106,234,26,154,90,218,58,186,122,250,6,134,70,198,38,166,102,230,22,150,86,214,54,182,118,246,14,142,78,206,46,174,110,238,30,158,94,222,62,190,126,254,1,129,65,193,33,161,97,225,17,145,81,209,49,177,113,241,9,137,73,201,41,169,105,233,25,153,89,217,57,185,121,249,5,133,69,197,37,165,101,229,21,149,85,213,53,181,117,245,13,141,77,205,45,173,109,237,29,157,93,221,61,189,125,253,3,131,67,195,35,163,99,227,19,147,83,211,51,179,115,243,11,139,75,203,43,171,107,235,27,155,91,219,59,187,123,251,7,135,71,199,39,167,103,231,23,151,87,215,55,183,119,247,15,143,79,207,47,175,111,239,31,159,95,223,63,191,127,255};

	cfix_W14_F13 x[4];
#pragma HLS ARRAY_PARTITION variable=x dim=0

	//  This is a radix-4 FFT, using decimation in frequency 
	//  The input signal is fixed point 
	//  Works with real or complex input 
	//  NOTE: The length of the input signal should be a power of 4: 4, 16, 64, 256, etc. 

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
			x[m].imag(0);
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

		b_n = n << sh;
		stage0[b_n  ] = Ao;	// bit truncation
		stage0[b_n+1] = Bo;
		stage0[b_n+2] = Co;
		stage0[b_n+3] = Do;
	} // StgLv1:for (int  n = 0; n < 64; n++) {


#ifndef __SYNTHESIS__
	std::cout << "-----------------------Stage0--------------------------- " << std::endl;
	for (int  n = 0; n < NFFT; n++) {
		std::cout << stage0[n].real() <<"+1i*"<< stage0[n].imag() << std::endl;
	}
	std::cout << std::endl;
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

		b_n = n << sh;
		stage1[b_n  ] = Ao;	// implict bit truncation
		stage1[b_n+1] = Bo;
		stage1[b_n+2] = Co;
		stage1[b_n+3] = Do;
	} // StgLv2:for (int  n = 0; n < 64; n++) {

#ifndef __SYNTHESIS__
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

		b_n = n << sh;
		stage2[b_n  ] = Ao;	// bit truncation
		stage2[b_n+1] = Bo;
		stage2[b_n+2] = Co;
		stage2[b_n+3] = Do;

	} // end for StgLv3:for (int  n = 0; n < 64; n++) 

#ifndef __SYNTHESIS__
	std::cout << "-----------------------Stage2--------------------------- " << std::endl;
	for (int  n = 0; n < NFFT; n++) {
		std::cout << stage2[n].real() <<"+1i*"<< stage2[n].imag() << std::endl;
	}
	std::cout << std::endl;

#endif

	/////////////////////////////////////////////
	//  Calculate butterflies for last stage   //
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
	} // end for (int n = 0; n < NFFT/4; n++)

#ifndef __SYNTHESIS__
	std::cout << "-----------------------Stage3--------------------------- " << std::endl;
	for (int  n = 0; n < NFFT; n++) {
		std::cout << stage3[n].real() <<"+1i*"<< stage3[n].imag() << std::endl;
	}
	std::cout << std::endl;
#endif

	cfix_W14_F5 stage_out[NFFT];
	#pragma HLS ARRAY_PARTITION variable=stage_out complete dim=0

	SnR:for (int i=0; i<NFFT; i++) {
	#pragma HLS UNROLL
		///  Rescale and Permute input into bit-reversed order 

		// The shift left operation result type is same as the type of operand
		// Operand must be cat to result type
		// refer to p.654, ug902 - 2017
		
		out_fft[i] = cfix_W14_F8(stage3[output_idx[i]].real(), stage3[output_idx[i]].imag());

		out_fft[i].real()(12,3) = stage3[output_idx[i]].real()(9, 0);
		out_fft[i].imag()(12,3) = stage3[output_idx[i]].imag()(9, 0);

		out_fft[i].real()(2,0) = 0;
		out_fft[i].imag()(2,0) = 0;

	}
#ifndef __SYNTHESIS__
	std::cout << "-----------------------scaling and reverse stage--------------------------- " << std::endl;
	for (int  n = 0; n < NFFT; n++) {
		std::cout << out_fft[n].real() <<"+1i*"<< out_fft[n].imag() << std::endl;
	}
	std::cout << std::endl;

	ofstream stout;
	stout.open("./fftout.dat");
	for (int  n = 0; n < NFFT; n++) {
		stout << setw(32) << out_fft[n].real() <<"+1i*"<< out_fft[n].imag() << endl;
		cout << out_fft[n].real() <<"+1i*"<< out_fft[n].imag() << endl;
	}
	stout.close();
#endif
}

void radix4bfly(
	cfix_W14_F13*   x,
	int 		segment,
	int 		stageFlag,
	cfix_W14_F12*   TwiddleFactor,
	cfix_W14_F13*   A,
	cfix_W14_F13*   B,
	cfix_W14_F13*   C,
	cfix_W14_F13*   D
){
	#pragma HLS PIPELINE II=2
	
	cfix_W14_F13 aa;
	cfix_W14_F13 bb;
	cfix_W14_F13 cc;
	cfix_W14_F13 dd; 

	//  For the last stage of a radix-4 FFT all the ABCD multipliers are 1. 
	//  Use the stageFlag variable to indicate the last stage 
	//  stageFlag = 0 indicates last FFT stage, set to 1 otherwise 
	//  Initialize variables and scale by 1/4 

	ap_uint<4> shL = 2;

	aa = cfix_W14_F13(x[0].real() >> shL, x[0].imag() >> shL);
	bb = cfix_W14_F13(x[1].real() >> shL, x[1].imag() >> shL);
	cc = cfix_W14_F13(x[2].real() >> shL, x[2].imag() >> shL);
	dd = cfix_W14_F13(x[3].real() >> shL, x[3].imag() >> shL);

	/*  Radix-4 Algorithm */
	cfix_W14_F12 TF;

	// A=a+b+c+d;
	*A = cfix_W14_F13(
			aa.real() + bb.real() + cc.real() + dd.real(),
			aa.imag() + bb.imag() + cc.imag() + dd.imag());

	// B=(a-b+c-d)*W(2*segment*stageFlag + 1);
	cfix_W14_F12 H_B;
	H_B = cfix_W14_F12(
			aa.real()-bb.real() + cc.real() - dd.real(),
			aa.imag()-bb.imag() + cc.imag() - dd.imag());
	TF = TwiddleFactor[2*segment+stageFlag+1];

	cfix_W29_F24 iB;
	hls::cmpy<ARCH_CMPY>(H_B, TF, iB);
	*B	= (cfix_W14_F13)iB;

	// C=(a-b*1i-c+d*1i)*W(segment*stageFlag + 1);
	cfix_W14_F12 H_C;

	H_C =  cfix_W14_F12(
			aa.real()+bb.imag()-cc.real()-dd.imag(),
			aa.imag()-bb.real()-cc.imag()+dd.real());
	TF = TwiddleFactor[segment+stageFlag+1];

	cfix_W29_F24 iC;
	hls::cmpy<ARCH_CMPY>(H_C, TF, iC);
	*C	= (cfix_W14_F13)iC;

	// D=(a+b*1i-c-d*1i)*W(3*segment*stageFlag + 1);
	cfix_W14_F12 H_D;
	H_D =  cfix_W14_F12(
			aa.real()-bb.imag()-cc.real()+dd.imag(),
			aa.imag()+bb.real()-cc.imag()-dd.real());

	TF = TwiddleFactor[3*segment+stageFlag+1];

	cfix_W29_F24 iD;
	hls::cmpy<ARCH_CMPY>(H_D, TF, iD);
	*D	= (cfix_W14_F13)iD;
}

