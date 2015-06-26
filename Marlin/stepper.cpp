/**
 * stepper.cpp - stepper motor driver: executes motion plans using stepper motors
 * Marlin Firmware
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "ultralcd.h"
#include "language.h"
#include "speed_lookuptable.h"
#ifdef LASER
  #include <SPI.h>
#include "laser.h"
#include "avr/pgmspace.h"
#else
#include "temperature.h"
#include "cardreader.h"
#endif
#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif

//===========================================================================
//============================= public variables ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced


//===========================================================================
//============================= private variables ===========================
//===========================================================================
//static makes it impossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits = 0;        // The next stepping-bits to be output
static unsigned int cleaning_buffer_counter;

#ifdef Z_DUAL_ENDSTOPS
  static bool performing_homing = false, 
              locked_z_motor = false, 
              locked_z2_motor = false;
#endif

// Counter variables for the Bresenham line tracer
static long counter_x, counter_y, counter_z, counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block

#ifdef ADVANCE
  static long advance_rate, advance, final_advance = 0;
  static long old_advance = 0;
  static long e_steps[4];
#endif

static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for deceleration start point
static char step_loops;
static unsigned short OCR1A_nominal;
static unsigned short step_loops_nominal;
volatile unsigned long Galvo_WorldXPosition;
volatile unsigned long Galvo_WorldYPosition;

volatile long endstops_trigsteps[3] = { 0 };
volatile long endstops_stepsTotal, endstops_stepsDone;
static volatile char endstop_hit_bits = 0; // use X_MIN, Y_MIN, Z_MIN and Z_PROBE as BIT value

#ifndef Z_DUAL_ENDSTOPS
  static byte
#else
  static uint16_t
#endif
  old_endstop_bits = 0; // use X_MIN, X_MAX... Z_MAX, Z_PROBE, Z2_MIN, Z2_MAX

#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
  bool abort_on_endstop_hit = false;
#endif

#ifdef MOTOR_CURRENT_PWM_XY_PIN
  int motor_current_setting[3] = DEFAULT_PWM_MOTOR_CURRENT;
#endif

static bool check_endstops = true;

volatile long count_position[NUM_AXIS] = { 0 };
volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1 };

#ifdef LASER
/*
  This table contains a mapping from 0 to 2048 for a tangent translated DAC value.
  This corrects for curvature as a result of projection. Note that this table will
  need to be recreated should the distance of laser to resevoir change
*/
const uint16_t PROGMEM dac_table[] = { 0, 17, 35, 52, 70, 87, 104, 122, 139, 157, 174, 192, 209, 226, 244, 261, 279, 296, 313, 331, 348, 366, 383, 401, 418, 435, 453, 470, 488, 505, 522, 540, 557, 575, 592, 610, 627, 644, 662, 679, 697, 714, 731, 749, 766, 784, 801, 819, 836, 853, 871, 888, 906, 923, 940, 958, 975, 993, 1010, 1027, 1045, 1062, 1080, 1097, 1115, 1132, 1149, 1167, 1184, 1202, 1219, 1236, 1254, 1271, 1289, 1306, 1323, 1341, 1358, 1376, 1393, 1410, 1428, 1445, 1463, 1480, 1498, 1515, 1532, 1550, 1567, 1585, 1602, 1619, 1637, 1654, 1672, 1689, 1706, 1724, 1741, 1759, 1776, 1793, 1811, 1828, 1846, 1863, 1880, 1898, 1915, 1933, 1950, 1967, 1985, 2002, 2020, 2037, 2054, 2072, 2089, 2107, 2124, 2141, 2159, 2176, 2194, 2211, 2228, 2246, 2263, 2281, 2298, 2315, 2333, 2350, 2368, 2385, 2402, 2420, 2437, 2455, 2472, 2489, 2507, 2524, 2542, 2559, 2576, 2594, 2611, 2628, 2646, 2663, 2681, 2698, 2715, 2733, 2750, 2768, 2785, 2802, 2820, 2837, 2854, 2872, 2889, 2907, 2924, 2941, 2959, 2976, 2994, 3011, 3028, 3046, 3063, 3080, 3098, 3115, 3133, 3150, 3167, 3185, 3202, 3219, 3237, 3254, 3272, 3289, 3306, 3324, 3341, 3358, 3376, 3393, 3411, 3428, 3445, 3463, 3480, 3497, 3515, 3532, 3550, 3567, 3584, 3602, 3619, 3636, 3654, 3671, 3688, 3706, 3723, 3741, 3758, 3775, 3793, 3810, 3827, 3845, 3862, 3879, 3897, 3914, 3931, 3949, 3966, 3984, 4001, 4018, 4036, 4053, 4070, 4088, 4105, 4122, 4140, 4157, 4174, 4192, 4209, 4226, 4244, 4261, 4278, 4296, 4313, 4331, 4348, 4365, 4383, 4400, 4417, 4435, 4452, 4469, 4487, 4504, 4521, 4539, 4556, 4573, 4591, 4608, 4625, 4643, 4660, 4677, 4695, 4712, 4729, 4747, 4764, 4781, 4799, 4816, 4833, 4851, 4868, 4885, 4902, 4920, 4937, 4954, 4972, 4989, 5006, 5024, 5041, 5058, 5076, 5093, 5110, 5128, 5145, 5162, 5180, 5197, 5214, 5232, 5249, 5266, 5283, 5301, 5318, 5335, 5353, 5370, 5387, 5405, 5422, 5439, 5456, 5474, 5491, 5508, 5526, 5543, 5560, 5578, 5595, 5612, 5629, 5647, 5664, 5681, 5699, 5716, 5733, 5750, 5768, 5785, 5802, 5820, 5837, 5854, 5871, 5889, 5906, 5923, 5941, 5958, 5975, 5992, 6010, 6027, 6044, 6062, 6079, 6096, 6113, 6131, 6148, 6165, 6182, 6200, 6217, 6234, 6251, 6269, 6286, 6303, 6321, 6338, 6355, 6372, 6390, 6407, 6424, 6441, 6459, 6476, 6493, 6510, 6528, 6545, 6562, 6579, 6597, 6614, 6631, 6648, 6666, 6683, 6700, 6717, 6735, 6752, 6769, 6786, 6803, 6821, 6838, 6855, 6872, 6890, 6907, 6924, 6941, 6959, 6976, 6993, 7010, 7027, 7045, 7062, 7079, 7096, 7114, 7131, 7148, 7165, 7182, 7200, 7217, 7234, 7251, 7269, 7286, 7303, 7320, 7337, 7355, 7372, 7389, 7406, 7423, 7441, 7458, 7475, 7492, 7509, 7527, 7544, 7561, 7578, 7595, 7613, 7630, 7647, 7664, 7681, 7698, 7716, 7733, 7750, 7767, 7784, 7802, 7819, 7836, 7853, 7870, 7887, 7905, 7922, 7939, 7956, 7973, 7991, 8008, 8025, 8042, 8059, 8076, 8094, 8111, 8128, 8145, 8162, 8179, 8196, 8214, 8231, 8248, 8265, 8282, 8299, 8317, 8334, 8351, 8368, 8385, 8402, 8419, 8437, 8454, 8471, 8488, 8505, 8522, 8539, 8556, 8574, 8591, 8608, 8625, 8642, 8659, 8676, 8694, 8711, 8728, 8745, 8762, 8779, 8796, 8813, 8830, 8848, 8865, 8882, 8899, 8916, 8933, 8950, 8967, 8984, 9002, 9019, 9036, 9053, 9070, 9087, 9104, 9121, 9138, 9155, 9173, 9190, 9207, 9224, 9241, 9258, 9275, 9292, 9309, 9326, 9343, 9360, 9377, 9395, 9412, 9429, 9446, 9463, 9480, 9497, 9514, 9531, 9548, 9565, 9582, 9599, 9616, 9633, 9651, 9668, 9685, 9702, 9719, 9736, 9753, 9770, 9787, 9804, 9821, 9838, 9855, 9872, 9889, 9906, 9923, 9940, 9957, 9974, 9991, 10008, 10025, 10042, 10060, 10077, 10094, 10111, 10128, 10145, 10162, 10179, 10196, 10213, 10230, 10247, 10264, 10281, 10298, 10315, 10332, 10349, 10366, 10383, 10400, 10417, 10434, 10451, 10468, 10485, 10502, 10519, 10536, 10553, 10570, 10587, 10604, 10621, 10638, 10655, 10672, 10689, 10706, 10722, 10739, 10756, 10773, 10790, 10807, 10824, 10841, 10858, 10875, 10892, 10909, 10926, 10943, 10960, 10977, 10994, 11011, 11028, 11045, 11062, 11079, 11096, 11112, 11129, 11146, 11163, 11180, 11197, 11214, 11231, 11248, 11265, 11282, 11299, 11316, 11333, 11349, 11366, 11383, 11400, 11417, 11434, 11451, 11468, 11485, 11502, 11519, 11535, 11552, 11569, 11586, 11603, 11620, 11637, 11654, 11671, 11688, 11704, 11721, 11738, 11755, 11772, 11789, 11806, 11823, 11840, 11856, 11873, 11890, 11907, 11924, 11941, 11958, 11974, 11991, 12008, 12025, 12042, 12059, 12076, 12092, 12109, 12126, 12143, 12160, 12177, 12194, 12210, 12227, 12244, 12261, 12278, 12295, 12311, 12328, 12345, 12362, 12379, 12396, 12412, 12429, 12446, 12463, 12480, 12496, 12513, 12530, 12547, 12564, 12581, 12597, 12614, 12631, 12648, 12665, 12681, 12698, 12715, 12732, 12749, 12765, 12782, 12799, 12816, 12832, 12849, 12866, 12883, 12900, 12916, 12933, 12950, 12967, 12983, 13000, 13017, 13034, 13050, 13067, 13084, 13101, 13118, 13134, 13151, 13168, 13185, 13201, 13218, 13235, 13252, 13268, 13285, 13302, 13318, 13335, 13352, 13369, 13385, 13402, 13419, 13436, 13452, 13469, 13486, 13502, 13519, 13536, 13553, 13569, 13586, 13603, 13619, 13636, 13653, 13669, 13686, 13703, 13720, 13736, 13753, 13770, 13786, 13803, 13820, 13836, 13853, 13870, 13886, 13903, 13920, 13936, 13953, 13970, 13986, 14003, 14020, 14036, 14053, 14070, 14086, 14103, 14120, 14136, 14153, 14170, 14186, 14203, 14220, 14236, 14253, 14270, 14286, 14303, 14319, 14336, 14353, 14369, 14386, 14403, 14419, 14436, 14452, 14469, 14486, 14502, 14519, 14535, 14552, 14569, 14585, 14602, 14618, 14635, 14652, 14668, 14685, 14701, 14718, 14735, 14751, 14768, 14784, 14801, 14817, 14834, 14851, 14867, 14884, 14900, 14917, 14933, 14950, 14967, 14983, 15000, 15016, 15033, 15049, 15066, 15082, 15099, 15116, 15132, 15149, 15165, 15182, 15198, 15215, 15231, 15248, 15264, 15281, 15297, 15314, 15330, 15347, 15363, 15380, 15396, 15413, 15429, 15446, 15462, 15479, 15495, 15512, 15528, 15545, 15561, 15578, 15594, 15611, 15627, 15644, 15660, 15677, 15693, 15710, 15726, 15743, 15759, 15776, 15792, 15809, 15825, 15841, 15858, 15874, 15891, 15907, 15924, 15940, 15957, 15973, 15990, 16006, 16022, 16039, 16055, 16072, 16088, 16105, 16121, 16137, 16154, 16170, 16187, 16203, 16219, 16236, 16252, 16269, 16285, 16302, 16318, 16334, 16351, 16367, 16384, 16400, 16416, 16433, 16449, 16465, 16482, 16498, 16515, 16531, 16547, 16564, 16580, 16596, 16613, 16629, 16646, 16662, 16678, 16695, 16711, 16727, 16744, 16760, 16776, 16793, 16809, 16825, 16842, 16858, 16874, 16891, 16907, 16923, 16940, 16956, 16972, 16989, 17005, 17021, 17038, 17054, 17070, 17086, 17103, 17119, 17135, 17152, 17168, 17184, 17201, 17217, 17233, 17249, 17266, 17282, 17298, 17315, 17331, 17347, 17363, 17380, 17396, 17412, 17428, 17445, 17461, 17477, 17493, 17510, 17526, 17542, 17558, 17575, 17591, 17607, 17623, 17640, 17656, 17672, 17688, 17704, 17721, 17737, 17753, 17769, 17786, 17802, 17818, 17834, 17850, 17867, 17883, 17899, 17915, 17931, 17948, 17964, 17980, 17996, 18012, 18029, 18045, 18061, 18077, 18093, 18109, 18126, 18142, 18158, 18174, 18190, 18206, 18223, 18239, 18255, 18271, 18287, 18303, 18319, 18336, 18352, 18368, 18384, 18400, 18416, 18432, 18448, 18465, 18481, 18497, 18513, 18529, 18545, 18561, 18577, 18594, 18610, 18626, 18642, 18658, 18674, 18690, 18706, 18722, 18738, 18754, 18771, 18787, 18803, 18819, 18835, 18851, 18867, 18883, 18899, 18915, 18931, 18947, 18963, 18979, 18995, 19011, 19028, 19044, 19060, 19076, 19092, 19108, 19124, 19140, 19156, 19172, 19188, 19204, 19220, 19236, 19252, 19268, 19284, 19300, 19316, 19332, 19348, 19364, 19380, 19396, 19412, 19428, 19444, 19460, 19476, 19492, 19508, 19524, 19540, 19556, 19572, 19588, 19604, 19620, 19636, 19651, 19667, 19683, 19699, 19715, 19731, 19747, 19763, 19779, 19795, 19811, 19827, 19843, 19859, 19875, 19891, 19906, 19922, 19938, 19954, 19970, 19986, 20002, 20018, 20034, 20050, 20066, 20081, 20097, 20113, 20129, 20145, 20161, 20177, 20193, 20208, 20224, 20240, 20256, 20272, 20288, 20304, 20320, 20335, 20351, 20367, 20383, 20399, 20415, 20430, 20446, 20462, 20478, 20494, 20510, 20525, 20541, 20557, 20573, 20589, 20605, 20620, 20636, 20652, 20668, 20684, 20699, 20715, 20731, 20747, 20763, 20778, 20794, 20810, 20826, 20842, 20857, 20873, 20889, 20905, 20920, 20936, 20952, 20968, 20983, 20999, 21015, 21031, 21046, 21062, 21078, 21094, 21109, 21125, 21141, 21157, 21172, 21188, 21204, 21220, 21235, 21251, 21267, 21282, 21298, 21314, 21330, 21345, 21361, 21377, 21392, 21408, 21424, 21439, 21455, 21471, 21486, 21502, 21518, 21533, 21549, 21565, 21580, 21596, 21612, 21627, 21643, 21659, 21674, 21690, 21706, 21721, 21737, 21753, 21768, 21784, 21800, 21815, 21831, 21846, 21862, 21878, 21893, 21909, 21924, 21940, 21956, 21971, 21987, 22003, 22018, 22034, 22049, 22065, 22080, 22096, 22112, 22127, 22143, 22158, 22174, 22190, 22205, 22221, 22236, 22252, 22267, 22283, 22298, 22314, 22330, 22345, 22361, 22376, 22392, 22407, 22423, 22438, 22454, 22469, 22485, 22500, 22516, 22531, 22547, 22562, 22578, 22593, 22609, 22624, 22640, 22655, 22671, 22686, 22702, 22717, 22733, 22748, 22764, 22779, 22795, 22810, 22826, 22841, 22857, 22872, 22888, 22903, 22918, 22934, 22949, 22965, 22980, 22996, 23011, 23027, 23042, 23057, 23073, 23088, 23104, 23119, 23134, 23150, 23165, 23181, 23196, 23212, 23227, 23242, 23258, 23273, 23288, 23304, 23319, 23335, 23350, 23365, 23381, 23396, 23412, 23427, 23442, 23458, 23473, 23488, 23504, 23519, 23534, 23550, 23565, 23580, 23596, 23611, 23626, 23642, 23657, 23672, 23688, 23703, 23718, 23734, 23749, 23764, 23780, 23795, 23810, 23825, 23841, 23856, 23871, 23887, 23902, 23917, 23933, 23948, 23963, 23978, 23994, 24009, 24024, 24039, 24055, 24070, 24085, 24100, 24116, 24131, 24146, 24161, 24177, 24192, 24207, 24222, 24238, 24253, 24268, 24283, 24298, 24314, 24329, 24344, 24359, 24374, 24390, 24405, 24420, 24435, 24450, 24466, 24481, 24496, 24511, 24526, 24542, 24557, 24572, 24587, 24602, 24617, 24633, 24648, 24663, 24678, 24693, 24708, 24723, 24739, 24754, 24769, 24784, 24799, 24814, 24829, 24845, 24860, 24875, 24890, 24905, 24920, 24935, 24950, 24965, 24981, 24996, 25011, 25026, 25041, 25056, 25071, 25086, 25101, 25116, 25131, 25146, 25161, 25177, 25192, 25207, 25222, 25237, 25252, 25267, 25282, 25297, 25312, 25327, 25342, 25357, 25372, 25387, 25402, 25417, 25432, 25447, 25462, 25477, 25492, 25507, 25522, 25537, 25552, 25567, 25582, 25597, 25612, 25627, 25642, 25657, 25672, 25687, 25702, 25717, 25732, 25747, 25762, 25777, 25792, 25807, 25822, 25837, 25852, 25867, 25881, 25896, 25911, 25926, 25941, 25956, 25971, 25986, 26001, 26016, 26031, 26046, 26061, 26075, 26090, 26105, 26120, 26135, 26150, 26165, 26180, 26195, 26209, 26224, 26239, 26254, 26269, 26284, 26299, 26313, 26328, 26343, 26358, 26373, 26388, 26403, 26417, 26432, 26447, 26462, 26477, 26492, 26506, 26521, 26536, 26551, 26566, 26580, 26595, 26610, 26625, 26640, 26654, 26669, 26684, 26699, 26714, 26728, 26743, 26758, 26773, 26787, 26802, 26817, 26832, 26846, 26861, 26876, 26891, 26906, 26920, 26935, 26950, 26964, 26979, 26994, 27009, 27023, 27038, 27053, 27068, 27082, 27097, 27112, 27126, 27141, 27156, 27170, 27185, 27200, 27215, 27229, 27244, 27259, 27273, 27288, 27303, 27317, 27332, 27347, 27361, 27376, 27391, 27405, 27420, 27435, 27449, 27464, 27478, 27493, 27508, 27522, 27537, 27552, 27566, 27581, 27595, 27610, 27625, 27639, 27654, 27669, 27683, 27698, 27712, 27727, 27742, 27756, 27771, 27785, 27800, 27814, 27829, 27844, 27858, 27873, 27887, 27902, 27916, 27931, 27945, 27960, 27975, 27989, 28004, 28018, 28033, 28047, 28062, 28076, 28091, 28105, 28120, 28134, 28149, 28163, 28178, 28192, 28207, 28221, 28236, 28250, 28265, 28279, 28294, 28308, 28323, 28337, 28352, 28366, 28381, 28395, 28410, 28424, 28438, 28453, 28467, 28482, 28496, 28511, 28525, 28540, 28554, 28568, 28583, 28597, 28612, 28626, 28641, 28655, 28669, 28684, 28698, 28713, 28727, 28741, 28756, 28770, 28784, 28799, 28813, 28828, 28842, 28856, 28871, 28885, 28899, 28914, 28928, 28943, 28957, 28971, 28986, 29000, 29014, 29029, 29043, 29057, 29072, 29086, 29100, 29115, 29129, 29143, 29158, 29172, 29186, 29200, 29215, 29229, 29243, 29258, 29272, 29286, 29300, 29315, 29329, 29343, 29358, 29372, 29386, 29400, 29415, 29429, 29443, 29457, 29472, 29486, 29500, 29514, 29529, 29543, 29557, 29571, 29586, 29600, 29614, 29628, 29642, 29657, 29671, 29685, 29699, 29713, 29728, 29742, 29756, 29770, 29784, 29799, 29813, 29827, 29841, 29855, 29869, 29884, 29898, 29912, 29926, 29940, 29954, 29969, 29983, 29997, 30011, 30025, 30039, 30053, 30067, 30082, 30096, 30110, 30124, 30138, 30152, 30166, 30180, 30195, 30209, 30223, 30237, 30251, 30265, 30279, 30293, 30307, 30321, 30335, 30349, 30364, 30378, 30392, 30406, 30420, 30434, 30448, 30462, 30476, 30490, 30504, 30518, 30532, 30546, 30560, 30574, 30588, 30602, 30616, 30630, 30644, 30658, 30672, 30686, 30700, 30714, 30728, 30742, 30756, 30770, 30784, 30798, 30812, 30826, 30840, 30854, 30868, 30882, 30896, 30910, 30924, 30938, 30952, 30966, 30980, 30994, 31007, 31021, 31035, 31049, 31063, 31077, 31091, 31105, 31119, 31133, 31147, 31161, 31174, 31188, 31202, 31216, 31230, 31244, 31258, 31272, 31286, 31299, 31313, 31327, 31341, 31355, 31369, 31383, 31396, 31410, 31424, 31438, 31452, 31466, 31480, 31493, 31507, 31521, 31535, 31549, 31562, 31576, 31590, 31604, 31618, 31632, 31645, 31659, 31673, 31687, 31700, 31714, 31728, 31742, 31756, 31769, 31783, 31797, 31811, 31824, 31838, 31852, 31866, 31879, 31893, 31907, 31921, 31934, 31948, 31962, 31976, 31989, 32003, 32017, 32031, 32044, 32058, 32072, 32085, 32099, 32113, 32126, 32140, 32154, 32168, 32181, 32195, 32209, 32222, 32236, 32250, 32263, 32277, 32291, 32304, 32318, 32332, 32345, 32359, 32373, 32386, 32400, 32413, 32427, 32441, 32454, 32468, 32482, 32495, 32509, 32522, 32536, 32550, 32563, 32577, 32590, 32604, 32618, 32631, 32645, 32658, 32672, 32686, 32699, 32713, 32726, 32740, 32753, 32767 };
const int grid = GRID_SIZE >> 1;
volatile int offset_value;
volatile int scaled_value;
volatile int sign;
#endif

//===========================================================================
//================================ functions ================================
//===========================================================================

#ifdef DUAL_X_CARRIAGE
  #define X_APPLY_DIR(v,ALWAYS) \
    if (extruder_duplication_enabled || ALWAYS) { \
      X_DIR_WRITE(v); \
      X2_DIR_WRITE(v); \
    } \
    else { \
      if (current_block->active_extruder) X2_DIR_WRITE(v); else X_DIR_WRITE(v); \
    }
  #define X_APPLY_STEP(v,ALWAYS) \
    if (extruder_duplication_enabled || ALWAYS) { \
      X_STEP_WRITE(v); \
      X2_STEP_WRITE(v); \
    } \
    else { \
      if (current_block->active_extruder != 0) X2_STEP_WRITE(v); else X_STEP_WRITE(v); \
    }
#else
  #define X_APPLY_DIR(v,Q) X_DIR_WRITE(v)
  #define X_APPLY_STEP(v,Q) X_STEP_WRITE(v)
#endif

#ifdef Y_DUAL_STEPPER_DRIVERS
  #define Y_APPLY_DIR(v,Q) { Y_DIR_WRITE(v); Y2_DIR_WRITE((v) != INVERT_Y2_VS_Y_DIR); }
  #define Y_APPLY_STEP(v,Q) { Y_STEP_WRITE(v); Y2_STEP_WRITE(v); }
#else
  #define Y_APPLY_DIR(v,Q) Y_DIR_WRITE(v)
  #define Y_APPLY_STEP(v,Q) Y_STEP_WRITE(v)
#endif

#ifdef Z_DUAL_STEPPER_DRIVERS
  #define Z_APPLY_DIR(v,Q) { Z_DIR_WRITE(v); Z2_DIR_WRITE(v); }
  #ifdef Z_DUAL_ENDSTOPS
    #define Z_APPLY_STEP(v,Q) \
    if (performing_homing) { \
      if (Z_HOME_DIR > 0) {\
        if (!(TEST(old_endstop_bits, Z_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(old_endstop_bits, Z2_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
      } else {\
        if (!(TEST(old_endstop_bits, Z_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(old_endstop_bits, Z2_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
      } \
    } else { \
      Z_STEP_WRITE(v); \
      Z2_STEP_WRITE(v); \
    }
  #else
    #define Z_APPLY_STEP(v,Q) { Z_STEP_WRITE(v); Z2_STEP_WRITE(v); }
  #endif
#else
  #define Z_APPLY_DIR(v,Q) Z_DIR_WRITE(v)
  #define Z_APPLY_STEP(v,Q) Z_STEP_WRITE(v)
#endif

#define E_APPLY_STEP(v,Q) E_STEP_WRITE(v)

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
  asm volatile ( \
    "clr r26 \n\t" \
    "mul %A1, %B2 \n\t" \
    "movw %A0, r0 \n\t" \
    "mul %A1, %A2 \n\t" \
    "add %A0, r1 \n\t" \
    "adc %B0, r26 \n\t" \
    "lsr r0 \n\t" \
    "adc %A0, r26 \n\t" \
    "adc %B0, r26 \n\t" \
    "clr r1 \n\t" \
    : \
    "=&r" (intRes) \
    : \
    "d" (charIn1), \
    "d" (intIn2) \
    : \
    "r26" \
  )

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
// B0 A0 are bits 24-39 and are the returned value
// C1 B1 A1 is longIn1
// D2 C2 B2 A2 is longIn2
//
#define MultiU24X32toH16(intRes, longIn1, longIn2) \
  asm volatile ( \
    "clr r26 \n\t" \
    "mul %A1, %B2 \n\t" \
    "mov r27, r1 \n\t" \
    "mul %B1, %C2 \n\t" \
    "movw %A0, r0 \n\t" \
    "mul %C1, %C2 \n\t" \
    "add %B0, r0 \n\t" \
    "mul %C1, %B2 \n\t" \
    "add %A0, r0 \n\t" \
    "adc %B0, r1 \n\t" \
    "mul %A1, %C2 \n\t" \
    "add r27, r0 \n\t" \
    "adc %A0, r1 \n\t" \
    "adc %B0, r26 \n\t" \
    "mul %B1, %B2 \n\t" \
    "add r27, r0 \n\t" \
    "adc %A0, r1 \n\t" \
    "adc %B0, r26 \n\t" \
    "mul %C1, %A2 \n\t" \
    "add r27, r0 \n\t" \
    "adc %A0, r1 \n\t" \
    "adc %B0, r26 \n\t" \
    "mul %B1, %A2 \n\t" \
    "add r27, r1 \n\t" \
    "adc %A0, r26 \n\t" \
    "adc %B0, r26 \n\t" \
    "lsr r27 \n\t" \
    "adc %A0, r26 \n\t" \
    "adc %B0, r26 \n\t" \
    "mul %D2, %A1 \n\t" \
    "add %A0, r0 \n\t" \
    "adc %B0, r1 \n\t" \
    "mul %D2, %B1 \n\t" \
    "add %B0, r0 \n\t" \
    "clr r1 \n\t" \
    : \
    "=&r" (intRes) \
    : \
    "d" (longIn1), \
    "d" (longIn2) \
    : \
    "r26" , "r27" \
  )

// Some useful constants

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= BIT(OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~BIT(OCIE1A)

void endstops_hit_on_purpose() {
  endstop_hit_bits = 0;
}

void checkHitEndstops() {
  if (endstop_hit_bits) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_ENDSTOPS_HIT);
    if (endstop_hit_bits & BIT(X_MIN)) {
      SERIAL_ECHOPAIR(" X:", (float)endstops_trigsteps[X_AXIS] / axis_steps_per_unit[X_AXIS]);
      LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "X");
    }
    if (endstop_hit_bits & BIT(Y_MIN)) {
      SERIAL_ECHOPAIR(" Y:", (float)endstops_trigsteps[Y_AXIS] / axis_steps_per_unit[Y_AXIS]);
      LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "Y");
    }
    if (endstop_hit_bits & BIT(Z_MIN)) {
      SERIAL_ECHOPAIR(" Z:", (float)endstops_trigsteps[Z_AXIS] / axis_steps_per_unit[Z_AXIS]);
      LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "Z");
    }
    #ifdef Z_PROBE_ENDSTOP
    if (endstop_hit_bits & BIT(Z_PROBE)) {
      SERIAL_ECHOPAIR(" Z_PROBE:", (float)endstops_trigsteps[Z_AXIS] / axis_steps_per_unit[Z_AXIS]);
      LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "ZP");
    }
    #endif
    SERIAL_EOL;

    endstops_hit_on_purpose();

    #if defined(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED) && defined(SDSUPPORT)
      if (abort_on_endstop_hit) {
        card.sdprinting = false;
        card.closefile();
        quickStop();
        disable_all_heaters(); // switch off all heaters.
      }
    #endif
  }
}

void enable_endstops(bool check) { check_endstops = check; }

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
//
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated using v = u + at where t is the accumulated timer values of the steps so far.

void st_wake_up() {
  //  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
  unsigned short timer;
  if (step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;

  if (step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2) & 0x3fff;
    step_loops = 4;
  }
  else if (step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1) & 0x7fff;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  }

  if (step_rate < (F_CPU / 500000)) step_rate = (F_CPU / 500000);
  step_rate -= (F_CPU / 500000); // Correct for minimal speed
  if (step_rate >= (8 * 256)) { // higher step rate
    unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
    MultiU16X8toH16(timer, tmp_step_rate, gain);
    timer = (unsigned short)pgm_read_word_near(table_address) - timer;
  }
  else { // lower step rates
    unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate)>>1) & 0xfffc;
    timer = (unsigned short)pgm_read_word_near(table_address);
    timer -= (((unsigned short)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
  }
  if (timer < 100) { timer = 100; MYSERIAL.print(MSG_STEPPER_TOO_HIGH); MYSERIAL.println(step_rate); }//(20kHz this should never happen)
  return timer;
}

// set the stepper direction of each axis
void set_stepper_direction() {
  
  // Set the direction bits (X_AXIS=A_AXIS and Y_AXIS=B_AXIS for COREXY)
  if (TEST(out_bits, X_AXIS)) {
#ifndef LASER
    X_APPLY_DIR(INVERT_X_DIR,0);
#endif
    count_direction[X_AXIS] = -1;
  }
  else {
#ifndef LASER
    X_APPLY_DIR(!INVERT_X_DIR,0);
#endif
    count_direction[X_AXIS] = 1;
  }

  if (TEST(out_bits, Y_AXIS)) {
#ifndef LASER
    Y_APPLY_DIR(INVERT_Y_DIR,0);
#endif
    count_direction[Y_AXIS] = -1;
  }
  else {
#ifndef LASER
    Y_APPLY_DIR(!INVERT_Y_DIR,0);
#endif
    count_direction[Y_AXIS] = 1;
  }
  
  if (TEST(out_bits, Z_AXIS)) {
    Z_APPLY_DIR(INVERT_Z_DIR,0);
    count_direction[Z_AXIS] = -1;
  }
  else {
    Z_APPLY_DIR(!INVERT_Z_DIR,0);
    count_direction[Z_AXIS] = 1;
  }
  
  #ifndef ADVANCE
    if (TEST(out_bits, E_AXIS)) {
#ifndef LASER
      REV_E_DIR();
#endif
      count_direction[E_AXIS] = -1;
    }
    else {
#ifndef LASER
      NORM_E_DIR();
#endif
      count_direction[E_AXIS] = 1;
    }
  #endif //!ADVANCE
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
FORCE_INLINE void trapezoid_generator_reset() {

  if (current_block->direction_bits != out_bits) {
    out_bits = current_block->direction_bits;
    set_stepper_direction();
  }
  
  #ifdef ADVANCE
    advance = current_block->initial_advance;
    final_advance = current_block->final_advance;
    // Do E steps + advance steps
    e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
    old_advance = advance >>8;
  #endif
  deceleration_time = 0;
  // step_rate to timer interval
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  OCR1A = acceleration_time;

  // SERIAL_ECHO_START;
  // SERIAL_ECHOPGM("advance :");
  // SERIAL_ECHO(current_block->advance/256.0);
  // SERIAL_ECHOPGM("advance rate :");
  // SERIAL_ECHO(current_block->advance_rate/256.0);
  // SERIAL_ECHOPGM("initial advance :");
  // SERIAL_ECHO(current_block->initial_advance/256.0);
  // SERIAL_ECHOPGM("final advance :");
  // SERIAL_ECHOLN(current_block->final_advance/256.0);
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
ISR(TIMER1_COMPA_vect) {

  if (cleaning_buffer_counter)
  {
    current_block = NULL;
    plan_discard_current_block();
    #ifdef SD_FINISHED_RELEASECOMMAND
      if ((cleaning_buffer_counter == 1) && (SD_FINISHED_STEPPERRELEASE)) enqueuecommands_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    #endif
    cleaning_buffer_counter--;
    OCR1A = 200;
    return;
  }

  // If there is no current block, attempt to pop one from the buffer
  if (!current_block) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block) {
      current_block->busy = true;
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_z = counter_e = counter_x;
      step_events_completed = 0;

      #ifdef Z_LATE_ENABLE
        if (current_block->steps[Z_AXIS] > 0) {
          enable_z();
          OCR1A = 2000; //1ms wait
          return;
        }
      #endif

      // #ifdef ADVANCE
      //   e_steps[current_block->active_extruder] = 0;
      // #endif
    }
    else {
      OCR1A = 2000; // 1kHz.
    }
  }

  if (current_block != NULL) {
#if defined(LASER) && LASER_CONTROL == 1
	  // Laser - Continuous Firing Mode

	  if (current_block->laser_status == LASER_ON) {
		  laser_fire(current_block->laser_intensity);
	  }

	  if (current_block->laser_status == LASER_OFF) {
		  laser_extinguish();
	  }
#endif
    // Check endstops
    if (check_endstops) {
      
      #ifdef Z_DUAL_ENDSTOPS
        uint16_t
      #else
        byte
      #endif
      current_endstop_bits;

      #define _ENDSTOP_PIN(AXIS, MINMAX) AXIS ##_## MINMAX ##_PIN
      #define _ENDSTOP_INVERTING(AXIS, MINMAX) AXIS ##_## MINMAX ##_ENDSTOP_INVERTING
      #define _AXIS(AXIS) AXIS ##_AXIS
      #define _ENDSTOP_HIT(AXIS) endstop_hit_bits |= BIT(_ENDSTOP(AXIS, MIN))
      #define _ENDSTOP(AXIS, MINMAX) AXIS ##_## MINMAX

      // SET_ENDSTOP_BIT: set the current endstop bits for an endstop to its status
      #define SET_ENDSTOP_BIT(AXIS, MINMAX) SET_BIT(current_endstop_bits, _ENDSTOP(AXIS, MINMAX), (READ(_ENDSTOP_PIN(AXIS, MINMAX)) != _ENDSTOP_INVERTING(AXIS, MINMAX)))
      // COPY_BIT: copy the value of COPY_BIT to BIT in bits
      #define COPY_BIT(bits, COPY_BIT, BIT) SET_BIT(bits, BIT, TEST(bits, COPY_BIT))
      // TEST_ENDSTOP: test the old and the current status of an endstop
      #define TEST_ENDSTOP(ENDSTOP) (TEST(current_endstop_bits, ENDSTOP) && TEST(old_endstop_bits, ENDSTOP))

      #define UPDATE_ENDSTOP(AXIS,MINMAX) \
        SET_ENDSTOP_BIT(AXIS, MINMAX); \
        if (TEST_ENDSTOP(_ENDSTOP(AXIS, MINMAX))  && (current_block->steps[_AXIS(AXIS)] > 0)) { \
          endstops_trigsteps[_AXIS(AXIS)] = count_position[_AXIS(AXIS)]; \
          _ENDSTOP_HIT(AXIS); \
          step_events_completed = current_block->step_event_count; \
        }
      
      #ifdef COREXY
        // Head direction in -X axis for CoreXY bots.
        // If DeltaX == -DeltaY, the movement is only in Y axis
        if ((current_block->steps[A_AXIS] != current_block->steps[B_AXIS]) || (TEST(out_bits, A_AXIS) == TEST(out_bits, B_AXIS))) {
          if (TEST(out_bits, X_HEAD))
      #else
          if (TEST(out_bits, X_AXIS))   // stepping along -X axis (regular Cartesian bot)
      #endif
          { // -direction
            #ifdef DUAL_X_CARRIAGE
              // with 2 x-carriages, endstops are only checked in the homing direction for the active extruder
              if ((current_block->active_extruder == 0 && X_HOME_DIR == -1) || (current_block->active_extruder != 0 && X2_HOME_DIR == -1))
            #endif
              {
                #if HAS_X_MIN
                  UPDATE_ENDSTOP(X, MIN);
                #endif
              }
          }
          else { // +direction
            #ifdef DUAL_X_CARRIAGE
              // with 2 x-carriages, endstops are only checked in the homing direction for the active extruder
              if ((current_block->active_extruder == 0 && X_HOME_DIR == 1) || (current_block->active_extruder != 0 && X2_HOME_DIR == 1))
            #endif
              {
                #if HAS_X_MAX
                  UPDATE_ENDSTOP(X, MAX);
                #endif
              }
          }
      #ifdef COREXY
        }
        // Head direction in -Y axis for CoreXY bots.
        // If DeltaX == DeltaY, the movement is only in X axis
        if ((current_block->steps[A_AXIS] != current_block->steps[B_AXIS]) || (TEST(out_bits, A_AXIS) != TEST(out_bits, B_AXIS))) {
          if (TEST(out_bits, Y_HEAD))
      #else
          if (TEST(out_bits, Y_AXIS))   // -direction
      #endif
          { // -direction
            #if HAS_Y_MIN
              UPDATE_ENDSTOP(Y, MIN);
            #endif
          }
          else { // +direction
            #if HAS_Y_MAX
              UPDATE_ENDSTOP(Y, MAX);
            #endif
          }
      #ifdef COREXY
        }
      #endif
      if (TEST(out_bits, Z_AXIS)) { // z -direction
        #if HAS_Z_MIN

          #ifdef Z_DUAL_ENDSTOPS
            SET_ENDSTOP_BIT(Z, MIN);
              #if HAS_Z2_MIN
                SET_ENDSTOP_BIT(Z2, MIN);
              #else

                COPY_BIT(current_endstop_bits, Z_MIN, Z2_MIN);
              #endif

            byte z_test = TEST_ENDSTOP(Z_MIN) << 0 + TEST_ENDSTOP(Z2_MIN) << 1; // bit 0 for Z, bit 1 for Z2

            if (z_test && current_block->steps[Z_AXIS] > 0) { // z_test = Z_MIN || Z2_MIN
              endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
              endstop_hit_bits |= BIT(Z_MIN);
              if (!performing_homing || (z_test == 0x3))  //if not performing home or if both endstops were trigged during homing...
                step_events_completed = current_block->step_event_count;
            }
          #else // !Z_DUAL_ENDSTOPS

            UPDATE_ENDSTOP(Z, MIN);
          #endif // !Z_DUAL_ENDSTOPS
        #endif // Z_MIN_PIN

        #ifdef Z_PROBE_ENDSTOP
          UPDATE_ENDSTOP(Z, PROBE);

          if (TEST_ENDSTOP(Z_PROBE))
          {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_hit_bits |= BIT(Z_PROBE);
          }
        #endif
      }
      else { // z +direction
        #if HAS_Z_MAX

          #ifdef Z_DUAL_ENDSTOPS

            SET_ENDSTOP_BIT(Z, MAX);
              #if HAS_Z2_MAX
                SET_ENDSTOP_BIT(Z2, MAX);
              #else
                COPY_BIT(current_endstop_bits, Z_MAX, Z2_MAX)
              #endif

            byte z_test = TEST_ENDSTOP(Z_MAX) << 0 + TEST_ENDSTOP(Z2_MAX) << 1; // bit 0 for Z, bit 1 for Z2

            if (z_test && current_block->steps[Z_AXIS] > 0) {  // t_test = Z_MAX || Z2_MAX
              endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
              endstop_hit_bits |= BIT(Z_MIN);

              if (!performing_homing || (z_test == 0x3))  //if not performing home or if both endstops were trigged during homing...
                step_events_completed = current_block->step_event_count;
            }

          #else // !Z_DUAL_ENDSTOPS

            UPDATE_ENDSTOP(Z, MAX);

          #endif // !Z_DUAL_ENDSTOPS
        #endif // Z_MAX_PIN
        
        #ifdef Z_PROBE_ENDSTOP
          UPDATE_ENDSTOP(Z, PROBE);

          if (TEST_ENDSTOP(Z_PROBE))
          {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_hit_bits |= BIT(Z_PROBE);
          }
        #endif
      }
      old_endstop_bits = current_endstop_bits;
    }


#define _COUNTER(axis) counter_## axis
#define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP

#ifdef LASER
#define APPLY_GALVO_MOVEMENT(axis, AXIS) \
          _COUNTER(axis) += current_block->steps[_AXIS(AXIS)] * step_loops; \
          if (_COUNTER(axis) > 0) { \
            _COUNTER(axis) -= current_block->step_event_count * step_loops; \
            count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)] * step_loops; \
            AXIS ##_galvo_step(count_direction[_AXIS(AXIS)] * step_loops); \
		            }
	// APPLY_GALVO_MOVEMENT(x, X);
	//  APPLY_GALVO_MOVEMENT(y, Y);
#endif


    // Take multiple steps per interrupt (For high speed moves)
    for (int8_t i = 0; i < step_loops; i++) {
      #ifndef AT90USB
        MSerial.checkRx(); // Check for serial chars.
      #endif

      #ifdef ADVANCE
        counter_e += current_block->steps[E_AXIS];
        if (counter_e > 0) {
          counter_e -= current_block->step_event_count;
          e_steps[current_block->active_extruder] += TEST(out_bits, E_AXIS) ? -1 : 1;
        }
      #endif //ADVANCE

      #define _COUNTER(axis) counter_## axis
      #define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
      #define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN

      #define STEP_ADD(axis, AXIS) \
        _COUNTER(axis) += current_block->steps[_AXIS(AXIS)]; \
        if (_COUNTER(axis) > 0) { _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS),0); }


      #define STEP_IF_COUNTER(axis, AXIS) \
        if (_COUNTER(axis) > 0) { \
          _COUNTER(axis) -= current_block->step_event_count; \
          count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
          _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS),0); \
        }

#if X_STEP_PIN > -1 && X_DIR_PIN > -1
		STEP_ADD(x,X);
		STEP_IF_COUNTER(x, X);
#endif
#if Y_STEP_PIN > -1 && Y_DIR_PIN > -1
		STEP_ADD(y,Y);
		STEP_IF_COUNTER(y, Y);
#endif
#if E0_STEP_PIN > -1 && E0_DIR_PIN > -1 && defined(ADVANCE)
		STEP_ADD(e, E);
		STEP_IF_COUNTER(e, E);
#endif
#if Z_STEP_PIN > -1 && Z_DIR_PIN > -1
		STEP_ADD(z, Z);
		STEP_IF_COUNTER(z, Z);
#endif

      step_events_completed++;
      if (step_events_completed >= current_block->step_event_count) break;
    }
    // Calculate new timer value
    unsigned short timer;
    unsigned short step_rate;
    if (step_events_completed <= (unsigned long)current_block->accelerate_until) {

      MultiU24X32toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;

      // upper limit
      if (acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      OCR1A = timer;
      acceleration_time += timer;
      #ifdef ADVANCE
        for(int8_t i=0; i < step_loops; i++) {
          advance += advance_rate;
        }
        //if (advance > current_block->advance) advance = current_block->advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >>8;

      #endif
    }
    else if (step_events_completed > (unsigned long)current_block->decelerate_after) {
      MultiU24X32toH16(step_rate, deceleration_time, current_block->acceleration_rate);

      if (step_rate > acc_step_rate) { // Check step_rate stays positive
        step_rate = current_block->final_rate;
      }
      else {
        step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
      }

      // lower limit
      if (step_rate < current_block->final_rate)
        step_rate = current_block->final_rate;

      // step_rate to timer interval
      timer = calc_timer(step_rate);
      OCR1A = timer;
      deceleration_time += timer;
      #ifdef ADVANCE
        for(int8_t i=0; i < step_loops; i++) {
          advance -= advance_rate;
        }
        if (advance < final_advance) advance = final_advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >>8;
      #endif //ADVANCE
    }
    else {
      OCR1A = OCR1A_nominal;
      // ensure we're running at the correct step rate, even if we just came off an acceleration
      step_loops = step_loops_nominal;
    }

    // If current block is finished, reset pointer
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
    }
  }
}

#ifdef ADVANCE
  unsigned char old_OCR0A;
  // Timer interrupt for E. e_steps is set in the main routine;
  // Timer 0 is shared with millies
  ISR(TIMER0_COMPA_vect)
  {
    old_OCR0A += 52; // ~10kHz interrupt (250000 / 26 = 9615kHz)
    OCR0A = old_OCR0A;
    // Set E direction (Depends on E direction + advance)
    for(unsigned char i=0; i<4;i++) {
      if (e_steps[0] != 0) {
        E0_STEP_WRITE(INVERT_E_STEP_PIN);
        if (e_steps[0] < 0) {
          E0_DIR_WRITE(INVERT_E0_DIR);
          e_steps[0]++;
          E0_STEP_WRITE(!INVERT_E_STEP_PIN);
        }
        else if (e_steps[0] > 0) {
          E0_DIR_WRITE(!INVERT_E0_DIR);
          e_steps[0]--;
          E0_STEP_WRITE(!INVERT_E_STEP_PIN);
        }
      }
 #if EXTRUDERS > 1
      if (e_steps[1] != 0) {
        E1_STEP_WRITE(INVERT_E_STEP_PIN);
        if (e_steps[1] < 0) {
          E1_DIR_WRITE(INVERT_E1_DIR);
          e_steps[1]++;
          E1_STEP_WRITE(!INVERT_E_STEP_PIN);
        }
        else if (e_steps[1] > 0) {
          E1_DIR_WRITE(!INVERT_E1_DIR);
          e_steps[1]--;
          E1_STEP_WRITE(!INVERT_E_STEP_PIN);
        }
      }
 #endif
 #if EXTRUDERS > 2
      if (e_steps[2] != 0) {
        E2_STEP_WRITE(INVERT_E_STEP_PIN);
        if (e_steps[2] < 0) {
          E2_DIR_WRITE(INVERT_E2_DIR);
          e_steps[2]++;
          E2_STEP_WRITE(!INVERT_E_STEP_PIN);
        }
        else if (e_steps[2] > 0) {
          E2_DIR_WRITE(!INVERT_E2_DIR);
          e_steps[2]--;
          E2_STEP_WRITE(!INVERT_E_STEP_PIN);
        }
      }
 #endif
 #if EXTRUDERS > 3
      if (e_steps[3] != 0) {
        E3_STEP_WRITE(INVERT_E_STEP_PIN);
        if (e_steps[3] < 0) {
          E3_DIR_WRITE(INVERT_E3_DIR);
          e_steps[3]++;
          E3_STEP_WRITE(!INVERT_E_STEP_PIN);
        }
        else if (e_steps[3] > 0) {
          E3_DIR_WRITE(!INVERT_E3_DIR);
          e_steps[3]--;
          E3_STEP_WRITE(!INVERT_E_STEP_PIN);
        }
      }
 #endif

    }
  }
#endif // ADVANCE

void st_init() {
  digipot_init(); //Initialize Digipot Motor Current
  microstep_init(); //Initialize Microstepping Pins

  // initialise TMC Steppers
  #ifdef HAVE_TMCDRIVER
    tmc_init();
  #endif
    // initialise L6470 Steppers
  #ifdef HAVE_L6470DRIVER
    L6470_init();
  #endif

  // Initialize Dir Pins
  #if HAS_X_DIR
    X_DIR_INIT;
  #endif
  #if HAS_X2_DIR
    X2_DIR_INIT;
  #endif
  #if HAS_Y_DIR
    Y_DIR_INIT;
    #if defined(Y_DUAL_STEPPER_DRIVERS) && HAS_Y2_DIR
      Y2_DIR_INIT;
    #endif
  #endif
  #if HAS_Z_DIR
    Z_DIR_INIT;
    #if defined(Z_DUAL_STEPPER_DRIVERS) && HAS_Z2_DIR
      Z2_DIR_INIT;
    #endif
  #endif
  #if HAS_E0_DIR
    E0_DIR_INIT;
  #endif
  #if HAS_E1_DIR
    E1_DIR_INIT;
  #endif
  #if HAS_E2_DIR
    E2_DIR_INIT;
  #endif
  #if HAS_E3_DIR
    E3_DIR_INIT;
  #endif

  //Initialize Enable Pins - steppers default to disabled.

  #if HAS_X_ENABLE
    X_ENABLE_INIT;
    if (!X_ENABLE_ON) X_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_X2_ENABLE
    X2_ENABLE_INIT;
    if (!X_ENABLE_ON) X2_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_Y_ENABLE
    Y_ENABLE_INIT;
    if (!Y_ENABLE_ON) Y_ENABLE_WRITE(HIGH);

  #if defined(Y_DUAL_STEPPER_DRIVERS) && HAS_Y2_ENABLE
    Y2_ENABLE_INIT;
    if (!Y_ENABLE_ON) Y2_ENABLE_WRITE(HIGH);
  #endif
  #endif
  #if HAS_Z_ENABLE
    Z_ENABLE_INIT;
    if (!Z_ENABLE_ON) Z_ENABLE_WRITE(HIGH);

    #if defined(Z_DUAL_STEPPER_DRIVERS) && HAS_Z2_ENABLE
      Z2_ENABLE_INIT;
      if (!Z_ENABLE_ON) Z2_ENABLE_WRITE(HIGH);
    #endif
  #endif
  #if HAS_E0_ENABLE
    E0_ENABLE_INIT;
    if (!E_ENABLE_ON) E0_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E1_ENABLE
    E1_ENABLE_INIT;
    if (!E_ENABLE_ON) E1_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E2_ENABLE
    E2_ENABLE_INIT;
    if (!E_ENABLE_ON) E2_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E3_ENABLE
    E3_ENABLE_INIT;
    if (!E_ENABLE_ON) E3_ENABLE_WRITE(HIGH);
  #endif

  //endstops and pullups

  #if HAS_X_MIN
    SET_INPUT(X_MIN_PIN);
    #ifdef ENDSTOPPULLUP_XMIN
      WRITE(X_MIN_PIN,HIGH);
    #endif
  #endif

  #if HAS_Y_MIN
    SET_INPUT(Y_MIN_PIN);
    #ifdef ENDSTOPPULLUP_YMIN
      WRITE(Y_MIN_PIN,HIGH);
    #endif
  #endif

  #if HAS_Z_MIN
    SET_INPUT(Z_MIN_PIN);
    #ifdef ENDSTOPPULLUP_ZMIN
      WRITE(Z_MIN_PIN,HIGH);
    #endif
  #endif

  #if HAS_X_MAX
    SET_INPUT(X_MAX_PIN);
    #ifdef ENDSTOPPULLUP_XMAX
      WRITE(X_MAX_PIN,HIGH);
    #endif
  #endif

  #if HAS_Y_MAX
    SET_INPUT(Y_MAX_PIN);
    #ifdef ENDSTOPPULLUP_YMAX
      WRITE(Y_MAX_PIN,HIGH);
    #endif
  #endif

  #if HAS_Z_MAX
    SET_INPUT(Z_MAX_PIN);
    #ifdef ENDSTOPPULLUP_ZMAX
      WRITE(Z_MAX_PIN,HIGH);
    #endif
  #endif

  #if HAS_Z2_MAX
    SET_INPUT(Z2_MAX_PIN);
    #ifdef ENDSTOPPULLUP_ZMAX
      WRITE(Z2_MAX_PIN,HIGH);
    #endif
  #endif

  #if (defined(Z_PROBE_PIN) && Z_PROBE_PIN >= 0) && defined(Z_PROBE_ENDSTOP) // Check for Z_PROBE_ENDSTOP so we don't pull a pin high unless it's to be used.
    SET_INPUT(Z_PROBE_PIN);
    #ifdef ENDSTOPPULLUP_ZPROBE
      WRITE(Z_PROBE_PIN,HIGH);
    #endif
  #endif

  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE(axis) disable_## axis()

  #define AXIS_INIT(axis, AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, _INVERT_STEP_PIN(PIN)); \
    _DISABLE(axis)

  #define E_AXIS_INIT(NUM) AXIS_INIT(e## NUM, E## NUM, E)

  // Initialize Step Pins
  #if HAS_X_STEP
    AXIS_INIT(x, X, X);
  #endif
  #if HAS_X2_STEP
    AXIS_INIT(x, X2, X);
  #endif
  #if HAS_Y_STEP
    #if defined(Y_DUAL_STEPPER_DRIVERS) && HAS_Y2_STEP
      Y2_STEP_INIT;
      Y2_STEP_WRITE(INVERT_Y_STEP_PIN);
    #endif
    AXIS_INIT(y, Y, Y);
  #endif
  #if HAS_Z_STEP
    #if defined(Z_DUAL_STEPPER_DRIVERS) && HAS_Z2_STEP
      Z2_STEP_INIT;
      Z2_STEP_WRITE(INVERT_Z_STEP_PIN);
    #endif
    AXIS_INIT(z, Z, Z);
  #endif
  #if HAS_E0_STEP
    E_AXIS_INIT(0);
  #endif
  #if HAS_E1_STEP
    E_AXIS_INIT(1);
  #endif
  #if HAS_E2_STEP
    E_AXIS_INIT(2);
  #endif
  #if HAS_E3_STEP
    E_AXIS_INIT(3);
  #endif

  // waveform generation = 0100 = CTC
  TCCR1B &= ~BIT(WGM13);
  TCCR1B |=  BIT(WGM12);
  TCCR1A &= ~BIT(WGM11);
  TCCR1A &= ~BIT(WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0);
  TCCR1A &= ~(3<<COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  OCR1A = 0x4000;
  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();

  #ifdef ADVANCE
    #if defined(TCCR0A) && defined(WGM01)
      TCCR0A &= ~BIT(WGM01);
      TCCR0A &= ~BIT(WGM00);
    #endif
    e_steps[0] = e_steps[1] = e_steps[2] = e_steps[3] = 0;
    TIMSK0 |= BIT(OCIE0A);
  #endif //ADVANCE

  enable_endstops(true); // Start with endstops active. After homing they can be disabled
  sei();
  
  set_stepper_direction(); // Init directions to out_bits = 0
}


/**
 * Block until all buffered steps are executed
 */
void st_synchronize() { while (blocks_queued()) idle(); }

void st_set_position(const long &x, const long &y, const long &z, const long &e) {
  CRITICAL_SECTION_START;
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void st_set_e_position(const long &e) {
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

long st_get_position(uint8_t axis) {
  long count_pos;
  CRITICAL_SECTION_START;
  count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

#ifdef ENABLE_AUTO_BED_LEVELING

  float st_get_position_mm(AxisEnum axis) {
    return st_get_position(axis) / axis_steps_per_unit[axis];
  }

#endif  // ENABLE_AUTO_BED_LEVELING

void finishAndDisableSteppers() {
  st_synchronize();
  disable_all_steppers();
}

void quickStop() {
  cleaning_buffer_counter = 5000;
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while (blocks_queued()) plan_discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

#ifdef BABYSTEPPING

  // MUST ONLY BE CALLED BY AN ISR,
  // No other ISR should ever interrupt this!
  void babystep(const uint8_t axis, const bool direction) {

    #define _ENABLE(axis) enable_## axis()
    #define _READ_DIR(AXIS) AXIS ##_DIR_READ
    #define _INVERT_DIR(AXIS) INVERT_## AXIS ##_DIR
    #define _APPLY_DIR(AXIS, INVERT) AXIS ##_APPLY_DIR(INVERT, true)

    #define BABYSTEP_AXIS(axis, AXIS, INVERT) { \
        _ENABLE(axis); \
        uint8_t old_pin = _READ_DIR(AXIS); \
        _APPLY_DIR(AXIS, _INVERT_DIR(AXIS)^direction^INVERT); \
        _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS), true); \
        delayMicroseconds(2); \
        _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS), true); \
        _APPLY_DIR(AXIS, old_pin); \
      }

    switch(axis) {

      case X_AXIS:
        BABYSTEP_AXIS(x, X, false);
        break;

      case Y_AXIS:
        BABYSTEP_AXIS(y, Y, false);
        break;
 
      case Z_AXIS: {

        #ifndef DELTA

          BABYSTEP_AXIS(z, Z, BABYSTEP_INVERT_Z);

        #else // DELTA

          bool z_direction = direction ^ BABYSTEP_INVERT_Z;

          enable_x();
          enable_y();
          enable_z();
          uint8_t old_x_dir_pin = X_DIR_READ,
                  old_y_dir_pin = Y_DIR_READ,
                  old_z_dir_pin = Z_DIR_READ;
          //setup new step
          X_DIR_WRITE(INVERT_X_DIR^z_direction);
          Y_DIR_WRITE(INVERT_Y_DIR^z_direction);
          Z_DIR_WRITE(INVERT_Z_DIR^z_direction);
          //perform step 
          X_STEP_WRITE(!INVERT_X_STEP_PIN);
          Y_STEP_WRITE(!INVERT_Y_STEP_PIN);
          Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
          delayMicroseconds(2);
          X_STEP_WRITE(INVERT_X_STEP_PIN); 
          Y_STEP_WRITE(INVERT_Y_STEP_PIN); 
          Z_STEP_WRITE(INVERT_Z_STEP_PIN);
          //get old pin state back.
          X_DIR_WRITE(old_x_dir_pin);
          Y_DIR_WRITE(old_y_dir_pin);
          Z_DIR_WRITE(old_z_dir_pin);

        #endif

      } break;
 
      default: break;
    }
  }

#endif //BABYSTEPPING
#ifdef LASER
  void set_galvo_pos(unsigned long X, unsigned long Y)
  {
	  Galvo_WorldXPosition = X;
	  Galvo_WorldYPosition = Y;
  }

  FORCE_INLINE void move_galvo(unsigned int axis, unsigned short value)
  {
	  /*
	  offset_value = (int) value - grid;
	  sign = offset_value < 0 ? -1 : 1;
	  scaled_value = (int) pgm_read_word_near(dac_table + abs(offset_value)) * sign + 32767;
	  */
	  //scaled_value = value;
	  //relocated galvo transfer here to save a function call
	  WRITE(GALVO_SS_PIN, LOW);
	  SPI.transfer((axis | (3 << 4)));  // Sets the axis and update immediately
	  SPI.transfer((uint8_t)((unsigned short) value >> 8));
	  SPI.transfer((uint8_t)(unsigned short) value);  // Sends position
	  WRITE(GALVO_SS_PIN, HIGH);
  }

  void move_galvos(unsigned long X, unsigned long Y)
  {

	  unsigned short sX = (unsigned short)X;
	  if (X > GRID_SIZE)
	  {
		  sX = GRID_SIZE;
	  }

	  unsigned short sY = (unsigned short)Y;
	  if (Y > GRID_SIZE)
	  {
		  sY = GRID_SIZE;
	  }
	  move_galvo(X_AXIS, sX);
	  move_galvo(Y_AXIS, sY);
  }

  void X_galvo_step(int step_dir)
  {
	  Galvo_WorldXPosition += step_dir;
	  unsigned short s = (unsigned short)Galvo_WorldXPosition;
	  if (Galvo_WorldXPosition > GRID_SIZE)
	  {
		  s = GRID_SIZE;
	  }
	  move_galvo(X_AXIS, s);
  }

  void Y_galvo_step(int step_dir)
  {
	  Galvo_WorldYPosition += step_dir;
	  unsigned short s = (unsigned short)Galvo_WorldYPosition;
	  if (Galvo_WorldYPosition > GRID_SIZE)
	  {
		  s = GRID_SIZE;
	  }

	  move_galvo(Y_AXIS, s);
  }
#endif
// From Arduino DigitalPotControl example
void digitalPotWrite(int address, int value) {
  #if HAS_DIGIPOTSS
    digitalWrite(DIGIPOTSS_PIN,LOW); // take the SS pin low to select the chip
    SPI.transfer(address); //  send in the address and value via SPI:
    SPI.transfer(value);
    digitalWrite(DIGIPOTSS_PIN,HIGH); // take the SS pin high to de-select the chip:
    //delay(10);
  #endif
}

// Initialize Digipot Motor Current
void digipot_init() {
  #if HAS_DIGIPOTSS
    const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;

    SPI.begin();
    pinMode(DIGIPOTSS_PIN, OUTPUT);
    for (int i = 0; i <= 4; i++) {
      //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
      digipot_current(i,digipot_motor_current[i]);
    }
  #endif
  #ifdef MOTOR_CURRENT_PWM_XY_PIN
    pinMode(MOTOR_CURRENT_PWM_XY_PIN, OUTPUT);
    pinMode(MOTOR_CURRENT_PWM_Z_PIN, OUTPUT);
    pinMode(MOTOR_CURRENT_PWM_E_PIN, OUTPUT);
    digipot_current(0, motor_current_setting[0]);
    digipot_current(1, motor_current_setting[1]);
    digipot_current(2, motor_current_setting[2]);
    //Set timer5 to 31khz so the PWM of the motor power is as constant as possible. (removes a buzzing noise)
    TCCR5B = (TCCR5B & ~(_BV(CS50) | _BV(CS51) | _BV(CS52))) | _BV(CS50);
  #endif
}

void digipot_current(uint8_t driver, int current) {
  #if HAS_DIGIPOTSS
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
    digitalPotWrite(digipot_ch[driver], current);
  #endif
  #ifdef MOTOR_CURRENT_PWM_XY_PIN
    switch(driver) {
      case 0: analogWrite(MOTOR_CURRENT_PWM_XY_PIN, 255L * current / MOTOR_CURRENT_PWM_RANGE); break;
      case 1: analogWrite(MOTOR_CURRENT_PWM_Z_PIN, 255L * current / MOTOR_CURRENT_PWM_RANGE); break;
      case 2: analogWrite(MOTOR_CURRENT_PWM_E_PIN, 255L * current / MOTOR_CURRENT_PWM_RANGE); break;
    }
  #endif
}

void microstep_init() {
  #if HAS_MICROSTEPS_E1
    pinMode(E1_MS1_PIN,OUTPUT);
    pinMode(E1_MS2_PIN,OUTPUT);
  #endif

  #if HAS_MICROSTEPS
    pinMode(X_MS1_PIN,OUTPUT);
    pinMode(X_MS2_PIN,OUTPUT);
    pinMode(Y_MS1_PIN,OUTPUT);
    pinMode(Y_MS2_PIN,OUTPUT);
    pinMode(Z_MS1_PIN,OUTPUT);
    pinMode(Z_MS2_PIN,OUTPUT);
    pinMode(E0_MS1_PIN,OUTPUT);
    pinMode(E0_MS2_PIN,OUTPUT);
    const uint8_t microstep_modes[] = MICROSTEP_MODES;
    for (uint16_t i = 0; i < sizeof(microstep_modes) / sizeof(microstep_modes[0]); i++)
      microstep_mode(i, microstep_modes[i]);
  #endif
}

void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2) {
  if (ms1 >= 0) switch(driver) {
    case 0: digitalWrite(X_MS1_PIN, ms1); break;
    case 1: digitalWrite(Y_MS1_PIN, ms1); break;
    case 2: digitalWrite(Z_MS1_PIN, ms1); break;
    case 3: digitalWrite(E0_MS1_PIN, ms1); break;
    #if HAS_MICROSTEPS_E1
      case 4: digitalWrite(E1_MS1_PIN, ms1); break;
    #endif
  }
  if (ms2 >= 0) switch(driver) {
    case 0: digitalWrite(X_MS2_PIN, ms2); break;
    case 1: digitalWrite(Y_MS2_PIN, ms2); break;
    case 2: digitalWrite(Z_MS2_PIN, ms2); break;
    case 3: digitalWrite(E0_MS2_PIN, ms2); break;
    #if defined(E1_MS2_PIN) && E1_MS2_PIN >= 0
      case 4: digitalWrite(E1_MS2_PIN, ms2); break;
    #endif
  }
}

void microstep_mode(uint8_t driver, uint8_t stepping_mode) {
  switch(stepping_mode) {
    case 1: microstep_ms(driver,MICROSTEP1); break;
    case 2: microstep_ms(driver,MICROSTEP2); break;
    case 4: microstep_ms(driver,MICROSTEP4); break;
    case 8: microstep_ms(driver,MICROSTEP8); break;
    case 16: microstep_ms(driver,MICROSTEP16); break;
  }
}

void microstep_readings() {
  SERIAL_PROTOCOLPGM("MS1,MS2 Pins\n");
  SERIAL_PROTOCOLPGM("X: ");
  SERIAL_PROTOCOL(digitalRead(X_MS1_PIN));
  SERIAL_PROTOCOLLN(digitalRead(X_MS2_PIN));
  SERIAL_PROTOCOLPGM("Y: ");
  SERIAL_PROTOCOL(digitalRead(Y_MS1_PIN));
  SERIAL_PROTOCOLLN(digitalRead(Y_MS2_PIN));
  SERIAL_PROTOCOLPGM("Z: ");
  SERIAL_PROTOCOL(digitalRead(Z_MS1_PIN));
  SERIAL_PROTOCOLLN(digitalRead(Z_MS2_PIN));
  SERIAL_PROTOCOLPGM("E0: ");
  SERIAL_PROTOCOL(digitalRead(E0_MS1_PIN));
  SERIAL_PROTOCOLLN(digitalRead(E0_MS2_PIN));
  #if HAS_MICROSTEPS_E1
    SERIAL_PROTOCOLPGM("E1: ");
    SERIAL_PROTOCOL(digitalRead(E1_MS1_PIN));
    SERIAL_PROTOCOLLN(digitalRead(E1_MS2_PIN));
  #endif
}

#ifdef Z_DUAL_ENDSTOPS
  void In_Homing_Process(bool state) { performing_homing = state; }
  void Lock_z_motor(bool state) { locked_z_motor = state; }
  void Lock_z2_motor(bool state) { locked_z2_motor = state; }
#endif
