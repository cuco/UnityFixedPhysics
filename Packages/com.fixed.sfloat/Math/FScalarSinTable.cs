// This file is generated by FScalarTablesGenerator.cs
// DO NOT MODIFY THIS MANUALLY

namespace Morefun.LockStep
{
    partial struct FScalar
    {
        private static readonly short[] c_sinTable = 
        {

               0,    6,   13,   19,   25,   31,   38,   44, 
              50,   57,   63,   69,   75,   82,   88,   94, 
             101,  107,  113,  119,  126,  132,  138,  145, 
             151,  157,  163,  170,  176,  182,  188,  195, 
             201,  207,  214,  220,  226,  232,  239,  245, 
             251,  258,  264,  270,  276,  283,  289,  295, 
             302,  308,  314,  320,  327,  333,  339,  345, 
             352,  358,  364,  371,  377,  383,  389,  396, 
             402,  408,  415,  421,  427,  433,  440,  446, 
             452,  458,  465,  471,  477,  484,  490,  496, 
             502,  509,  515,  521,  527,  534,  540,  546, 
             553,  559,  565,  571,  578,  584,  590,  596, 
             603,  609,  615,  621,  628,  634,  640,  646, 
             653,  659,  665,  672,  678,  684,  690,  697, 
             703,  709,  715,  722,  728,  734,  740,  747, 
             753,  759,  765,  772,  778,  784,  790,  797, 
             803,  809,  815,  822,  828,  834,  840,  847, 
             853,  859,  865,  872,  878,  884,  890,  897, 
             903,  909,  915,  922,  928,  934,  940,  947, 
             953,  959,  965,  972,  978,  984,  990,  997, 
            1003, 1009, 1015, 1021, 1028, 1034, 1040, 1046, 
            1053, 1059, 1065, 1071, 1078, 1084, 1090, 1096, 
            1102, 1109, 1115, 1121, 1127, 1134, 1140, 1146, 
            1152, 1158, 1165, 1171, 1177, 1183, 1190, 1196, 
            1202, 1208, 1214, 1221, 1227, 1233, 1239, 1246, 
            1252, 1258, 1264, 1270, 1277, 1283, 1289, 1295, 
            1301, 1308, 1314, 1320, 1326, 1332, 1339, 1345, 
            1351, 1357, 1363, 1370, 1376, 1382, 1388, 1394, 
            1401, 1407, 1413, 1419, 1425, 1431, 1438, 1444, 
            1450, 1456, 1462, 1469, 1475, 1481, 1487, 1493, 
            1499, 1506, 1512, 1518, 1524, 1530, 1537, 1543, 
            1549, 1555, 1561, 1567, 1574, 1580, 1586, 1592, 
            1598, 1604, 1611, 1617, 1623, 1629, 1635, 1641, 
            1647, 1654, 1660, 1666, 1672, 1678, 1684, 1691, 
            1697, 1703, 1709, 1715, 1721, 1727, 1734, 1740, 
            1746, 1752, 1758, 1764, 1770, 1776, 1783, 1789, 
            1795, 1801, 1807, 1813, 1819, 1826, 1832, 1838, 
            1844, 1850, 1856, 1862, 1868, 1874, 1881, 1887, 
            1893, 1899, 1905, 1911, 1917, 1923, 1929, 1936, 
            1942, 1948, 1954, 1960, 1966, 1972, 1978, 1984, 
            1990, 1997, 2003, 2009, 2015, 2021, 2027, 2033, 
            2039, 2045, 2051, 2057, 2064, 2070, 2076, 2082, 
            2088, 2094, 2100, 2106, 2112, 2118, 2124, 2130, 
            2136, 2142, 2149, 2155, 2161, 2167, 2173, 2179, 
            2185, 2191, 2197, 2203, 2209, 2215, 2221, 2227, 
            2233, 2239, 2245, 2251, 2257, 2264, 2270, 2276, 
            2282, 2288, 2294, 2300, 2306, 2312, 2318, 2324, 
            2330, 2336, 2342, 2348, 2354, 2360, 2366, 2372, 
            2378, 2384, 2390, 2396, 2402, 2408, 2414, 2420, 
            2426, 2432, 2438, 2444, 2450, 2456, 2462, 2468, 
            2474, 2480, 2486, 2492, 2498, 2504, 2510, 2516, 
            2522, 2528, 2534, 2540, 2546, 2552, 2558, 2564, 
            2570, 2576, 2582, 2588, 2594, 2599, 2605, 2611, 
            2617, 2623, 2629, 2635, 2641, 2647, 2653, 2659, 
            2665, 2671, 2677, 2683, 2689, 2695, 2701, 2706, 
            2712, 2718, 2724, 2730, 2736, 2742, 2748, 2754, 
            2760, 2766, 2772, 2778, 2783, 2789, 2795, 2801, 
            2807, 2813, 2819, 2825, 2831, 2837, 2842, 2848, 
            2854, 2860, 2866, 2872, 2878, 2884, 2890, 2895, 
            2901, 2907, 2913, 2919, 2925, 2931, 2937, 2942, 
            2948, 2954, 2960, 2966, 2972, 2978, 2983, 2989, 
            2995, 3001, 3007, 3013, 3018, 3024, 3030, 3036, 
            3042, 3048, 3053, 3059, 3065, 3071, 3077, 3083, 
            3088, 3094, 3100, 3106, 3112, 3118, 3123, 3129, 
            3135, 3141, 3147, 3152, 3158, 3164, 3170, 3176, 
            3181, 3187, 3193, 3199, 3204, 3210, 3216, 3222, 
            3228, 3233, 3239, 3245, 3251, 3256, 3262, 3268, 
            3274, 3279, 3285, 3291, 3297, 3302, 3308, 3314, 
            3320, 3325, 3331, 3337, 3343, 3348, 3354, 3360, 
            3366, 3371, 3377, 3383, 3389, 3394, 3400, 3406, 
            3411, 3417, 3423, 3429, 3434, 3440, 3446, 3451, 
            3457, 3463, 3468, 3474, 3480, 3485, 3491, 3497, 
            3503, 3508, 3514, 3520, 3525, 3531, 3537, 3542, 
            3548, 3554, 3559, 3565, 3571, 3576, 3582, 3587, 
            3593, 3599, 3604, 3610, 3616, 3621, 3627, 3633, 
            3638, 3644, 3650, 3655, 3661, 3666, 3672, 3678, 
            3683, 3689, 3694, 3700, 3706, 3711, 3717, 3722, 
            3728, 3734, 3739, 3745, 3750, 3756, 3762, 3767, 
            3773, 3778, 3784, 3789, 3795, 3801, 3806, 3812, 
            3817, 3823, 3828, 3834, 3839, 3845, 3851, 3856, 
            3862, 3867, 3873, 3878, 3884, 3889, 3895, 3900, 
            3906, 3911, 3917, 3922, 3928, 3934, 3939, 3945, 
            3950, 3956, 3961, 3967, 3972, 3978, 3983, 3989, 
            3994, 3999, 4005, 4010, 4016, 4021, 4027, 4032, 
            4038, 4043, 4049, 4054, 4060, 4065, 4071, 4076, 
            4081, 4087, 4092, 4098, 4103, 4109, 4114, 4120, 
            4125, 4130, 4136, 4141, 4147, 4152, 4158, 4163, 
            4168, 4174, 4179, 4185, 4190, 4195, 4201, 4206, 
            4212, 4217, 4222, 4228, 4233, 4238, 4244, 4249, 
            4255, 4260, 4265, 4271, 4276, 4281, 4287, 4292, 
            4297, 4303, 4308, 4313, 4319, 4324, 4329, 4335, 
            4340, 4345, 4351, 4356, 4361, 4367, 4372, 4377, 
            4383, 4388, 4393, 4399, 4404, 4409, 4415, 4420, 
            4425, 4430, 4436, 4441, 4446, 4451, 4457, 4462, 
            4467, 4473, 4478, 4483, 4488, 4494, 4499, 4504, 
            4509, 4515, 4520, 4525, 4530, 4536, 4541, 4546, 
            4551, 4556, 4562, 4567, 4572, 4577, 4583, 4588, 
            4593, 4598, 4603, 4609, 4614, 4619, 4624, 4629, 
            4634, 4640, 4645, 4650, 4655, 4660, 4666, 4671, 
            4676, 4681, 4686, 4691, 4696, 4702, 4707, 4712, 
            4717, 4722, 4727, 4732, 4738, 4743, 4748, 4753, 
            4758, 4763, 4768, 4773, 4778, 4784, 4789, 4794, 
            4799, 4804, 4809, 4814, 4819, 4824, 4829, 4834, 
            4840, 4845, 4850, 4855, 4860, 4865, 4870, 4875, 
            4880, 4885, 4890, 4895, 4900, 4905, 4910, 4915, 
            4920, 4925, 4930, 4935, 4940, 4945, 4950, 4955, 
            4960, 4965, 4970, 4975, 4980, 4985, 4990, 4995, 
            5000, 5005, 5010, 5015, 5020, 5025, 5030, 5035, 
            5040, 5045, 5050, 5055, 5060, 5065, 5070, 5075, 
            5080, 5084, 5089, 5094, 5099, 5104, 5109, 5114, 
            5119, 5124, 5129, 5134, 5138, 5143, 5148, 5153, 
            5158, 5163, 5168, 5173, 5177, 5182, 5187, 5192, 
            5197, 5202, 5207, 5212, 5216, 5221, 5226, 5231, 
            5236, 5241, 5245, 5250, 5255, 5260, 5265, 5269, 
            5274, 5279, 5284, 5289, 5293, 5298, 5303, 5308, 
            5313, 5317, 5322, 5327, 5332, 5337, 5341, 5346, 
            5351, 5356, 5360, 5365, 5370, 5375, 5379, 5384, 
            5389, 5393, 5398, 5403, 5408, 5412, 5417, 5422, 
            5427, 5431, 5436, 5441, 5445, 5450, 5455, 5459, 
            5464, 5469, 5473, 5478, 5483, 5487, 5492, 5497, 
            5501, 5506, 5511, 5515, 5520, 5525, 5529, 5534, 
            5539, 5543, 5548, 5552, 5557, 5562, 5566, 5571, 
            5575, 5580, 5585, 5589, 5594, 5598, 5603, 5608, 
            5612, 5617, 5621, 5626, 5630, 5635, 5640, 5644, 
            5649, 5653, 5658, 5662, 5667, 5671, 5676, 5680, 
            5685, 5690, 5694, 5699, 5703, 5708, 5712, 5717, 
            5721, 5726, 5730, 5735, 5739, 5744, 5748, 5752, 
            5757, 5761, 5766, 5770, 5775, 5779, 5784, 5788, 
            5793, 5797, 5801, 5806, 5810, 5815, 5819, 5824, 
            5828, 5832, 5837, 5841, 5846, 5850, 5854, 5859, 
            5863, 5868, 5872, 5876, 5881, 5885, 5890, 5894, 
            5898, 5903, 5907, 5911, 5916, 5920, 5924, 5929, 
            5933, 5937, 5942, 5946, 5950, 5955, 5959, 5963, 
            5968, 5972, 5976, 5980, 5985, 5989, 5993, 5998, 
            6002, 6006, 6010, 6015, 6019, 6023, 6027, 6032, 
            6036, 6040, 6044, 6049, 6053, 6057, 6061, 6066, 
            6070, 6074, 6078, 6083, 6087, 6091, 6095, 6099, 
            6104, 6108, 6112, 6116, 6120, 6124, 6129, 6133, 
            6137, 6141, 6145, 6149, 6154, 6158, 6162, 6166, 
            6170, 6174, 6178, 6182, 6187, 6191, 6195, 6199, 
            6203, 6207, 6211, 6215, 6219, 6224, 6228, 6232, 
            6236, 6240, 6244, 6248, 6252, 6256, 6260, 6264, 
            6268, 6272, 6276, 6280, 6284, 6288, 6292, 6296, 
            6300, 6305, 6309, 6313, 6317, 6321, 6325, 6329, 
            6333, 6336, 6340, 6344, 6348, 6352, 6356, 6360, 
            6364, 6368, 6372, 6376, 6380, 6384, 6388, 6392, 
            6396, 6400, 6404, 6408, 6411, 6415, 6419, 6423, 
            6427, 6431, 6435, 6439, 6443, 6447, 6450, 6454, 
            6458, 6462, 6466, 6470, 6474, 6477, 6481, 6485, 
            6489, 6493, 6497, 6500, 6504, 6508, 6512, 6516, 
            6519, 6523, 6527, 6531, 6535, 6538, 6542, 6546, 
            6550, 6554, 6557, 6561, 6565, 6569, 6572, 6576, 
            6580, 6584, 6587, 6591, 6595, 6599, 6602, 6606, 
            6610, 6613, 6617, 6621, 6625, 6628, 6632, 6636, 
            6639, 6643, 6647, 6650, 6654, 6658, 6661, 6665, 
            6669, 6672, 6676, 6680, 6683, 6687, 6690, 6694, 
            6698, 6701, 6705, 6708, 6712, 6716, 6719, 6723, 
            6726, 6730, 6734, 6737, 6741, 6744, 6748, 6751, 
            6755, 6759, 6762, 6766, 6769, 6773, 6776, 6780, 
            6783, 6787, 6790, 6794, 6797, 6801, 6804, 6808, 
            6811, 6815, 6818, 6822, 6825, 6829, 6832, 6836, 
            6839, 6843, 6846, 6850, 6853, 6856, 6860, 6863, 
            6867, 6870, 6874, 6877, 6880, 6884, 6887, 6891, 
            6894, 6897, 6901, 6904, 6908, 6911, 6914, 6918, 
            6921, 6924, 6928, 6931, 6934, 6938, 6941, 6944, 
            6948, 6951, 6954, 6958, 6961, 6964, 6968, 6971, 
            6974, 6978, 6981, 6984, 6987, 6991, 6994, 6997, 
            7001, 7004, 7007, 7010, 7014, 7017, 7020, 7023, 
            7027, 7030, 7033, 7036, 7039, 7043, 7046, 7049, 
            7052, 7055, 7059, 7062, 7065, 7068, 7071, 7074, 
            7078, 7081, 7084, 7087, 7090, 7093, 7097, 7100, 
            7103, 7106, 7109, 7112, 7115, 7118, 7122, 7125, 
            7128, 7131, 7134, 7137, 7140, 7143, 7146, 7149, 
            7152, 7155, 7159, 7162, 7165, 7168, 7171, 7174, 
            7177, 7180, 7183, 7186, 7189, 7192, 7195, 7198, 
            7201, 7204, 7207, 7210, 7213, 7216, 7219, 7222, 
            7225, 7228, 7231, 7234, 7237, 7239, 7242, 7245, 
            7248, 7251, 7254, 7257, 7260, 7263, 7266, 7269, 
            7272, 7274, 7277, 7280, 7283, 7286, 7289, 7292, 
            7295, 7297, 7300, 7303, 7306, 7309, 7312, 7314, 
            7317, 7320, 7323, 7326, 7329, 7331, 7334, 7337, 
            7340, 7343, 7345, 7348, 7351, 7354, 7356, 7359, 
            7362, 7365, 7367, 7370, 7373, 7376, 7378, 7381, 
            7384, 7387, 7389, 7392, 7395, 7397, 7400, 7403, 
            7405, 7408, 7411, 7414, 7416, 7419, 7422, 7424, 
            7427, 7429, 7432, 7435, 7437, 7440, 7443, 7445, 
            7448, 7451, 7453, 7456, 7458, 7461, 7464, 7466, 
            7469, 7471, 7474, 7476, 7479, 7482, 7484, 7487, 
            7489, 7492, 7494, 7497, 7499, 7502, 7504, 7507, 
            7509, 7512, 7514, 7517, 7519, 7522, 7524, 7527, 
            7529, 7532, 7534, 7537, 7539, 7542, 7544, 7547, 
            7549, 7551, 7554, 7556, 7559, 7561, 7564, 7566, 
            7568, 7571, 7573, 7576, 7578, 7580, 7583, 7585, 
            7588, 7590, 7592, 7595, 7597, 7599, 7602, 7604, 
            7606, 7609, 7611, 7613, 7616, 7618, 7620, 7623, 
            7625, 7627, 7629, 7632, 7634, 7636, 7639, 7641, 
            7643, 7645, 7648, 7650, 7652, 7654, 7657, 7659, 
            7661, 7663, 7665, 7668, 7670, 7672, 7674, 7676, 
            7679, 7681, 7683, 7685, 7687, 7690, 7692, 7694, 
            7696, 7698, 7700, 7702, 7705, 7707, 7709, 7711, 
            7713, 7715, 7717, 7719, 7722, 7724, 7726, 7728, 
            7730, 7732, 7734, 7736, 7738, 7740, 7742, 7744, 
            7746, 7748, 7750, 7753, 7755, 7757, 7759, 7761, 
            7763, 7765, 7767, 7769, 7771, 7773, 7775, 7777, 
            7779, 7781, 7782, 7784, 7786, 7788, 7790, 7792, 
            7794, 7796, 7798, 7800, 7802, 7804, 7806, 7808, 
            7809, 7811, 7813, 7815, 7817, 7819, 7821, 7823, 
            7825, 7826, 7828, 7830, 7832, 7834, 7836, 7837, 
            7839, 7841, 7843, 7845, 7847, 7848, 7850, 7852, 
            7854, 7855, 7857, 7859, 7861, 7863, 7864, 7866, 
            7868, 7870, 7871, 7873, 7875, 7877, 7878, 7880, 
            7882, 7883, 7885, 7887, 7889, 7890, 7892, 7894, 
            7895, 7897, 7899, 7900, 7902, 7904, 7905, 7907, 
            7909, 7910, 7912, 7913, 7915, 7917, 7918, 7920, 
            7921, 7923, 7925, 7926, 7928, 7929, 7931, 7933, 
            7934, 7936, 7937, 7939, 7940, 7942, 7943, 7945, 
            7946, 7948, 7950, 7951, 7953, 7954, 7956, 7957, 
            7959, 7960, 7962, 7963, 7964, 7966, 7967, 7969, 
            7970, 7972, 7973, 7975, 7976, 7978, 7979, 7980, 
            7982, 7983, 7985, 7986, 7987, 7989, 7990, 7992, 
            7993, 7994, 7996, 7997, 7998, 8000, 8001, 8002, 
            8004, 8005, 8006, 8008, 8009, 8010, 8012, 8013, 
            8014, 8016, 8017, 8018, 8020, 8021, 8022, 8023, 
            8025, 8026, 8027, 8028, 8030, 8031, 8032, 8033, 
            8035, 8036, 8037, 8038, 8039, 8041, 8042, 8043, 
            8044, 8045, 8047, 8048, 8049, 8050, 8051, 8052, 
            8054, 8055, 8056, 8057, 8058, 8059, 8060, 8062, 
            8063, 8064, 8065, 8066, 8067, 8068, 8069, 8070, 
            8071, 8072, 8074, 8075, 8076, 8077, 8078, 8079, 
            8080, 8081, 8082, 8083, 8084, 8085, 8086, 8087, 
            8088, 8089, 8090, 8091, 8092, 8093, 8094, 8095, 
            8096, 8097, 8098, 8099, 8100, 8101, 8101, 8102, 
            8103, 8104, 8105, 8106, 8107, 8108, 8109, 8110, 
            8111, 8111, 8112, 8113, 8114, 8115, 8116, 8117, 
            8117, 8118, 8119, 8120, 8121, 8122, 8122, 8123, 
            8124, 8125, 8126, 8126, 8127, 8128, 8129, 8130, 
            8130, 8131, 8132, 8133, 8133, 8134, 8135, 8136, 
            8136, 8137, 8138, 8139, 8139, 8140, 8141, 8141, 
            8142, 8143, 8143, 8144, 8145, 8145, 8146, 8147, 
            8147, 8148, 8149, 8149, 8150, 8151, 8151, 8152, 
            8153, 8153, 8154, 8154, 8155, 8156, 8156, 8157, 
            8157, 8158, 8158, 8159, 8160, 8160, 8161, 8161, 
            8162, 8162, 8163, 8163, 8164, 8164, 8165, 8165, 
            8166, 8166, 8167, 8167, 8168, 8168, 8169, 8169, 
            8170, 8170, 8171, 8171, 8172, 8172, 8172, 8173, 
            8173, 8174, 8174, 8175, 8175, 8175, 8176, 8176, 
            8177, 8177, 8177, 8178, 8178, 8178, 8179, 8179, 
            8180, 8180, 8180, 8181, 8181, 8181, 8182, 8182, 
            8182, 8182, 8183, 8183, 8183, 8184, 8184, 8184, 
            8184, 8185, 8185, 8185, 8185, 8186, 8186, 8186, 
            8186, 8187, 8187, 8187, 8187, 8188, 8188, 8188, 
            8188, 8188, 8189, 8189, 8189, 8189, 8189, 8189, 
            8190, 8190, 8190, 8190, 8190, 8190, 8190, 8190, 
            8191, 8191, 8191, 8191, 8191, 8191, 8191, 8191, 
            8191, 8191, 8192, 8192, 8192, 8192, 8192, 8192, 
            8192, 8192, 8192, 8192, 8192, 8192, 8192, 8192, 
            8192, 
        };
    }
}
