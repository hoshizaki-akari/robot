// Offline signal replay; no ROS node, SDK object, or hardware connection.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "fr_traction/aubo_drag_model.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {std::cerr << "usage: replay_aubo_drag traction.csv\n"; return 2;}
  std::ifstream input(argv[1]);
  if (!input) {return 2;}
  std::string line, value;
  std::getline(input, line);
  std::istringstream header(line);
  std::map<std::string, std::size_t> fields;
  while (std::getline(header, value, ',')) {fields[value] = fields.size();}
  fr_traction::AuboDragModel model;
  double previous = -1;
  std::cout << "elapsed_s,old_vx,old_vy,old_vz,new_vx,new_vy,new_vz,fx,fy,fz\n";
  std::cout << std::setprecision(12);
  while (std::getline(input, line)) {
    std::vector<std::string> row;
    std::istringstream stream(line);
    while (std::getline(stream, value, ',')) {row.push_back(value);}
    const auto number = [&](const std::string & name) {
        return std::stod(row.at(fields.at(name)));
      };
    const double time = number("elapsed_s");
    const double dt = previous < 0 ? 0.01 : time - previous;
    previous = time;
    fr_traction::Vec3 velocity;
    const fr_traction::Vec3 force{number("fx"), number("fy"), number("fz")};
    if (row.at(fields.at("state")) == "DRAGGING") {velocity = model.update(force, dt);} else {
      model.reset();
    }
    std::cout << time << ',' << number("command_vx") << ',' << number("command_vy") << ',' <<
      number("command_vz") << ',' << velocity.x << ',' << velocity.y << ',' << velocity.z <<
      ',' << force.x << ',' << force.y << ',' << force.z << '\n';
  }
  return 0;
}
