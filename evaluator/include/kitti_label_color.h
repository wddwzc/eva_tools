#ifndef _KITTI_LABEL_COLOR_H_
#define _KITTI_LABEL_COLOR_H_

#include <vector>
#include <unordered_map>

// 34 classes, the map for class ID to RGB
// 0 is unclassified
// raw class to rgb
std::unordered_map<int, std::vector<int> > colors_map = {
	{0, {0, 0, 0}},
    {1, {0, 0, 255}},
    {10, {245, 150, 100}},
    {11, {245, 230, 100}},
    {13, {250, 80, 100}},
    {15, {150, 60, 30}},
    {16, {255, 0, 0}},
    {18, {180, 30, 80}},
    {20, {255, 0, 0}},
    {30, {30, 30, 255}},
    {31, {200, 40, 255}},
    {32, {90, 30, 150}},
    {40, {255, 0, 255}},
    {44, {255, 150, 255}},
    {48, {75, 0, 75}},
    {49, {75, 0, 175}},
    {50, {0, 200, 255}},
    {51, {50, 120, 255}},
    {52, {0, 150, 255}},
    {60, {170, 255, 150}},
    {70, {0, 175, 0}},
    {71, {0, 60, 135}},
    {72, {80, 240, 150}},
    {80, {150, 240, 255}},
    {81, {0, 0, 255}},
    {99, {255, 255, 50}},

    // {252, {245, 150, 100}},
    // {253, {200, 40, 255}},
    // {254, {30, 30, 255}},
    // {255, {90, 30, 150}},
    // {256, {255, 0, 0}},
    // {257, {250, 80, 100}},
    // {258, {180, 30, 80}},
    // {259, {255, 0, 0}}
    {250, {245, 150, 100}}
};


// trans 25 classes to color
std::unordered_map<int, std::vector<int> > colors_map_tran = {
	// {0, {0, 0, 0}},
    {0, {40, 40, 40}},
    {1, {245, 150, 100}},
    {2, {245, 230, 100}},
    {3, {150, 60, 30}},
    {4, {180, 30, 80}},
    {5, {250, 80, 100}},
    {6, {30, 30, 255}},
    {7, {200, 40, 255}},
    {8, {90, 30, 150}},
    {9, {255, 0, 255}},
    {10, {255, 150, 255}},
    {11, {75, 0, 75}},
    {12, {75, 0, 175}},
    {13, {0, 200, 255}},
    {14, {50, 120, 255}},
    {15, {0, 175, 0}},
    {16, {0, 60, 135}},
    {17, {80, 240, 150}},
    {18, {150, 240, 255}},
    {19, {0, 0, 255}},
    {20, {245, 150, 100}},
    {21, {200, 40, 255}},
    {22, {30, 30, 255}},
    {23, {90, 30, 150}},
    {24, {255, 0, 0}},
    {25, {180, 30, 80}},
};

std::unordered_map<int, std::vector<int> > colors_map_simple = {
    {0, {40, 40, 40}},
    // {1, {245, 150, 100}},
    {1, {255, 0, 255}},
    {2, {245, 230, 100}},
    {3, {30, 30, 255}},
    // {4, {255, 0, 255}},
    {4, {245, 150, 100}},
    {5, {0, 200, 255}},
    {6, {50, 120, 255}},
    {7, {0, 175, 0}},
    {8, {0, 60, 135}},
    {9, {150, 240, 255}},
    {10, {0, 0, 255}},
    {11, {250, 80, 100}}
};

// 车辆、行人、地面、建筑、栏杆、植被、树木、杆子、交通标识

// 26 classes mapping to  classes
// 0 unlabeled
// 1 car
// 2 person
// 3 ground
// 4 building
// 5 fence
// 6 vegetation
// 7 trunk
// 8 pole
// 9 traffic sign



// 26 valid classes, the map for original id to training id
// 0 is unclassified (total 26 classes with 0)
std::unordered_map<int, int> classes_map = {
	{0 , 0},
	{1 , 0},
	{10, 1},
	{11, 2},
	{13, 5},
	{15, 3},
	{16, 5},
	{18, 4},
	{20, 5},
	{30, 6},
	{31, 7},
	{32, 8},
	{40, 9},
	{44, 10},
	{48, 11},
	{49, 12},
	{50, 13},
	{51, 14},
	{52, 0},
	{60, 9},
	{70, 15},
	{71, 16},
	{72, 17},
	{80, 18},
	{81, 19},
	{99, 0},
	{252, 20},
	{253, 21},
	{254, 22},
	{255, 23},
	{256, 24},
	{257, 24},
	{258, 25},
	{259, 24}
};


/*
0:  0,1,52,99               # "unlabeled"
1:  10,252,18,258           # "car"
2:  11,255,253,31,32,15     # "bicycle"
3:  30,254                  # "person"
4:  40,60,49,48,44          # "road"
5: 50                       # "building"
6: 51                       # "fence"
7: 70,72                    # "vegetation"
8: 71                       # "trunk"
9: 80                       # "pole"
10: 81                      # "traffic-sign"
11:  13,16,20,256,257,259   # "other-vehicle"
*/
std::unordered_map<int, int> simple_classes_map = {
    {0 , 0},
	{1 , 0},
	{10, 1},
	{11, 2},
	{13, 11},
	{15, 2},
	{16, 11},
	{18, 1},
	{20, 11},
	{30, 3},
	{31, 2},
	{32, 2},
	{40, 4},
	{44, 4},
	{48, 4},
	{49, 4},
	{50, 5},
	{51, 6},
	{52, 0},
	{60, 4},
	{70, 7},
	{71, 8},
	{72, 7},
	{80, 9},
	{81, 10},
	{99, 0},
	{252, 1},
	{253, 2},
	{254, 3},
	{255, 2},
	{256, 11},
	{257, 11},
	{258, 1},
	{259, 11}
};



/*
0:  0,1,52,99   # "unlabeled"
1:  10          # "car"
2:  11          # "bicycle"
3:  15          # "motorcycle"
4:  18          # "truck"
5:  13,16,20    # "other-vehicle"
6:  30          # "person"
7:  31          # "bicyclist"
8:  32          # "motorcyclist"
9:  40,60       # "road"
10: 44          # "parking"
11: 48          # "sidewalk"
12: 49          # "other-ground"
13: 50          # "building"
14: 51          # "fence"
15: 70          # "vegetation"
16: 71          # "trunk"
17: 72          # "terrain"
18: 80          # "pole"
19: 81          # "traffic-sign"
20: 252         # "moving-car"
21: 253         # "moving-bicyclist"
22: 254         # "moving-person"
23: 255         # "moving-motorcyclist"
24: 256,257,259 # "moving-other-vehicle"
25: 258         # "moving-truck"
*/


/*
  0 : 0     # "unlabeled"
  1 : 0     # "outlier" mapped to "unlabeled" --------------------------mapped
  10: 1     # "car"
  11: 2     # "bicycle"
  13: 5     # "bus" mapped to "other-vehicle" --------------------------mapped
  15: 3     # "motorcycle"
  16: 5     # "on-rails" mapped to "other-vehicle" ---------------------mapped
  18: 4     # "truck"
  20: 5     # "other-vehicle"
  30: 6     # "person"
  31: 7     # "bicyclist"
  32: 8     # "motorcyclist"
  40: 9     # "road"
  44: 10    # "parking"
  48: 11    # "sidewalk"
  49: 12    # "other-ground"
  50: 13    # "building"
  51: 14    # "fence"
  52: 0     # "other-structure" mapped to "unlabeled" ------------------mapped
  60: 9     # "lane-marking" to "road" ---------------------------------mapped
  70: 15    # "vegetation"
  71: 16    # "trunk"
  72: 17    # "terrain"
  80: 18    # "pole"
  81: 19    # "traffic-sign"
  99: 0     # "other-object" to "unlabeled" ----------------------------mapped
  252: 20    # "moving-car"
  253: 21    # "moving-bicyclist"
  254: 22    # "moving-person"
  255: 23    # "moving-motorcyclist"
  256: 24    # "moving-on-rails" mapped to "moving-other-vehicle" ------mapped
  257: 24    # "moving-bus" mapped to "moving-other-vehicle" -----------mapped
  258: 25    # "moving-truck"
  259: 24    # "moving-other-vehicle"
*/


#endif
