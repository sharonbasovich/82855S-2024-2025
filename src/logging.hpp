#include <string>
#include "json.hpp"

using json = nlohmann::json;

struct Message
{
  std::string topic;
  json payload;
};

inline void to_json(json &j, const Message &msg)
{
  j = json{{"topic", msg.topic}, {"payload", msg.payload}};
}

struct Odometry
{
  double x;
  double y;
  double theta;
};

inline void to_json(json &j, const Odometry& msg)
{
  j = json{{"x", msg.x}, {"y", msg.y}, {"theta", msg.theta}};
}

// struct ExampleStruct
// {
//   int x;
// };

// inline void to_json(json &j, const ExampleStruct &msg)
// {
//   j = json{{"x", msg.x}};
// }