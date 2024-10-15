#include "cv1/robot.hpp"
#include "rclcpp/rclcpp.hpp"
int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   Robot robot;

   // Nastavenie terminalu do módu "raw" aby program prijímal klávesy w,a,s,d,q bez potreby stlačíť Enter
   system("stty raw");

   while (1)
   {
      system("clear");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PositionXY: %f %f ", robot.getX(), robot.getY());

      char move = getchar();

      if (move == 'Q' || move == 'q')
      {
         break;
      }

      // Spracovanie pohybu hráča
      switch (move)
      {
      case 'W':
      case 'w':
         robot.move(0, 1.0);
         break;
      case 'S':
      case 's':
         robot.move(0, -1.0);
         break;
      case 'A':
      case 'a':
         robot.move(-1.0, 0);
         break;
      case 'D':
      case 'd':
         robot.move(1.0, 0);
         break;
      }

   }

   // Nastavenie terminalu do módu "cooked" resetuje nastavenie "raw"
   system("stty cooked");

   return 0;
}
