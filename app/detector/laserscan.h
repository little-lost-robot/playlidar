#include <string>
#include <vector>
#include <ostream>

class LaserScan{
   public:
   float angle_min;
   float angle_max;
   float angle_increment;
   float time_increment;
   float scan_time;
   float range_min;
   float range_max;
   std::vector<float>  ranges;
   std::vector<float>  intensities;


     LaserScan(){
      angle_min=0.0;
      angle_max=0.0;
      angle_increment=0.0;
      time_increment=0.0;
      scan_time=0.0;
      range_min=0.0;
      range_max=0.0;
     }
};
