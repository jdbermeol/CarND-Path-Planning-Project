#include"utils.h"

using namespace std;

double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2){
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y){

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y){

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y){
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y){
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int compute_car_lane(double d){
    int car_lane = -1;
    if ( d > 0 && d < 4 ) {
        car_lane = 0;
    } else if ( d > 4 && d < 8 ) {
        car_lane = 1;
    } else if ( d > 8 && d < 12 ) {
        car_lane = 2;
    }
	return car_lane;
}

Vehicle build_vehicle_from_sensor_fusion(const vector<double>& sensor_fusion){
	Vehicle vehicle;
	vehicle.id = sensor_fusion[0];
	vehicle.x = sensor_fusion[1];
	vehicle.y = sensor_fusion[2];
	vehicle.s = sensor_fusion[5];
	vehicle.d = sensor_fusion[6];
	vehicle.v = sqrt((sensor_fusion[3]*sensor_fusion[3]) + (sensor_fusion[4]*sensor_fusion[4]));
	vehicle.yaw = atan2(sensor_fusion[4], sensor_fusion[3]);
	vehicle.lane = compute_car_lane(vehicle.d);
	return vehicle;
}

Vehicle predict_sensor_future_position(const Vehicle& sensor_fusion, int N, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	// Lets assume cars are not going to change lanes
	Vehicle vehicle;
	vehicle.id = sensor_fusion.id;
	vehicle.s = sensor_fusion.s + (N + 0.0) * 0.02 * sensor_fusion.v;
	vehicle.d = sensor_fusion.d;
	vehicle.v = sensor_fusion.v;
	vehicle.lane = compute_car_lane(vehicle.d);
	vector<double> xy = getXY(vehicle.s, vehicle.d, maps_s, maps_x, maps_y);
	vehicle.x = xy[0];
	vehicle.y = xy[1];
	return vehicle;
}

vector<Vehicle> get_vehicles_ahead(const vector<Vehicle>& vehicles, double lane, double width, double s){
    vector<Vehicle> vehicles_ahead;
    copy_if(
        vehicles.begin(), vehicles.end(),
        back_inserter(vehicles_ahead),
        [&s](const Vehicle& vehicle){return vehicle.s > s;} );
    vehicles_ahead.erase(
	    remove_if(
		    vehicles_ahead.begin(), 
            vehicles_ahead.end(),
            [&lane, &width](const Vehicle& vehicle){return vehicle.d <= lane + width && vehicle.d >= lane - width;}),
        vehicles_ahead.end());
    sort(
        vehicles_ahead.begin(), vehicles_ahead.end(),
        [](const Vehicle& left, const Vehicle& right){return left.s < right.s;});   
    return vehicles_ahead;
}