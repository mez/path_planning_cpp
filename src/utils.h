#include <math.h>
#include <vector>

using namespace std; //Shouldn't do this in prod.

namespace utils {

  //consts
  const int LANE_WIDTH          = 4;          //in meters
  const double MAX_VELOCITY     = 49.5;
  // For converting back and forth between radians and degrees.
  constexpr double pi() { return M_PI; }

  double rad2deg(double x) { return x * 180 / pi(); }

  double deg2rad(double x) { return x * pi() / 180; }

  double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); i++) {
      double map_x = maps_x[i];
      double map_y = maps_y[i];
      double dist = distance(x, y, map_x, map_y);
      if (dist < closestLen) {
        closestLen = dist;
        closestWaypoint = i;
      }

    }

    return closestWaypoint;

  }


  int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {

    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = abs(theta - heading);

    if (angle > pi() / 4) {
      closestWaypoint++;
    }

    return closestWaypoint;

  }

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
      prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
      frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
      frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};

  }

  // Transform from Frenet s,d coordinates to Cartesian x,y
  vector<double> getXY(double s, double d, std::vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
      prev_wp++;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};

  }


  // Checks if the SocketIO event has JSON data.
  // If there is data the JSON object in string format will be returned,
  // else the empty string "" will be returned.
  string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    } else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 2);
    }
    return "";
  }


  /**
   * loads the waypoints from the file.
   * @param map_waypoints_x
   * @param map_waypoints_y
   * @param map_waypoints_s
   * @param map_waypoints_dx
   * @param map_waypoints_dy
   */
  void load_map_waypoints(vector<double> & map_waypoints_x,
                          vector<double> & map_waypoints_y,
                          vector<double> & map_waypoints_s,
                          vector<double> & map_waypoints_dx,
                          vector<double> & map_waypoints_dy) {
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
      istringstream iss(line);
      double x;
      double y;
      float s;
      float d_x;
      float d_y;
      iss >> x;
      iss >> y;
      iss >> s;
      iss >> d_x;
      iss >> d_y;
      map_waypoints_x.push_back(x);
      map_waypoints_y.push_back(y);
      map_waypoints_s.push_back(s);
      map_waypoints_dx.push_back(d_x);
      map_waypoints_dy.push_back(d_y);
    }

  }

  /**
   * Changes given points into car' local space
   * @param ref_x
   * @param ref_y
   * @param ref_yaw
   * @param ptsx
   * @param ptsy
   */
  void globaltToCarSpace(double ref_x,
                         double ref_y,
                         double ref_yaw,
                         vector<double> & ptsx, vector<double> & ptsy) {

    //Let's convert the global pts to local car space
    for (int i = 0; i < ptsx.size(); ++i) {
      double shift_x = ptsx[i]-ref_x;
      double shift_y = ptsy[i]-ref_y;
      const double shift_yaw = 0-ref_yaw;

      ptsx[i] = shift_x*cos(shift_yaw) - shift_y*sin(shift_yaw);
      ptsy[i] = shift_x*sin(shift_yaw) + shift_y*cos(shift_yaw);
    }
  }
}