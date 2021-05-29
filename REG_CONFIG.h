#define ID_Drycontact 1
#define ID_PowerMeter 2
#define ID_Temp1      3
#define ID_Temp2      4
#define ID_SDM        5    

///////////////////////////////////////////
// Register Power Meter
#define c_A       2999
#define c_B       3001
#define c_C       3003
#define c_N       3005
#define c_G       3007
#define c_Avg     3009
#define v_A_B     3019
#define v_B_C     3021
#define v_C_A     3023
#define v_LL_Avg  3025
#define v_A_N      3027
#define v_B_N      3029
#define v_C_N      3031
#define v_LN_Avg   3035

#define v_Freq     3109

#define ap_Total    3059

/////
#define cu_A  3011 // Current Unbalance A
#define cu_B  3013 // Current Unbalance B
#define cu_C  3015 // Current Unbalance C
#define cu_W  3017 // Current Unbalance Worst

#define vu_AB 3037
#define vu_BC 3039
#define vu_CA 3041
#define vu_LLW 3043 // Voltage Unbalance L-L Worst

#define vu_AN 3045
#define vu_BN 3047
#define vu_CN 3049
#define vu_LNW 3051 // Voltage Unbalance L-N Worst

#define ap_A 3053
#define ap_B 3055
#define ap_C 3057
#define ap_T 3059 // Active Power Total
#define rp_A 3061
#define rp_B 3063
#define rp_C 3065
#define rp_T 3067 // Reactive Power Total
#define app_A 3069
#define app_B 3061
#define app_C 3073
#define app_T 3075 // Apparent Power Total
////


///////////////////////////////////////////
// Register Dry Contact
#define DI1   0
#define DI2   1
#define DI3   2
#define DI4   3

///////////////////////////////////////////
// Register Temp
#define pvTemp  4096

// Register SDM120
#define SDM120_Volt                0x0000      //  0.
#define SDM120_Current             0x0006      //  1.
#define SDM120_ActivePower         0x000C      //  2.          
#define SDM120_PowerFactor         0x001E      //  3.
#define SDM120_Frequency           0x0046      //  4.
#define SDM120_TotalActiveEnergy   0x0156      //  5.
