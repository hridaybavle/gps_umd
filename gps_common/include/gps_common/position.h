#ifndef _POSITION_H
#define _POSITION_H

 #include <angles/angles.h>

 namespace Position
 {
  class Pose3D
  {
   public:
   double x;
   double y;
   double z;
   double roll;
   double pitch;
   double yaw;
 
   Pose3D(void):
   x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0)
   {}
   Pose3D(const Pose3D &copy):
   x(copy.x), y(copy.y), z(copy.z),
   roll(copy.roll), pitch(copy.pitch), yaw(copy.yaw)
   {}
   Pose3D(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw):
         x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw)
   {}
 
   // some useful operators, define more if needed
   bool operator==(const Pose3D &that) const
   {
     return (this->x == that.x
              && this->y == that.y
              && this->z == that.z
              && this->roll == that.roll
              && this->pitch == that.pitch
              && this->yaw == that.yaw);
   }
   bool operator!=(const Pose3D &that) const
   {
     return (this->x != that.x
            || this->y != that.y
            || this->z != that.z
            || this->roll != that.roll
            || this->pitch != that.pitch
            || this->yaw != that.yaw);
   }
   Pose3D operator+(const Pose3D &that) const
   {
    return Pose3D(this->x + that.x,
                  this->y + that.y,
                  this->z + that.z,
                  angles::normalize_angle(this->roll + that.roll),
                  angles::normalize_angle(this->pitch + that.pitch),
                  angles::normalize_angle(this->yaw + that.yaw));
   }
    Pose3D operator-(const Pose3D &that) const
  {
    return Pose3D(this->x - that.x,
                  this->y - that.y,
                  this->z - that.z,
                  angles::normalize_angle(this->roll - that.roll),
                  angles::normalize_angle(this->pitch - that.pitch),
                  angles::normalize_angle(this->yaw - that.yaw));
  }
   };

   class Position3D
  {
   public:
   Pose3D pos;
   Pose3D vel;
  };
 }
 
#endif // _POSITION_H


