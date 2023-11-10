#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
#include <list>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose2D_t& pose,
                        OccupancyGrid& map)
{
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;

    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO: Update the map's log odds using the movingScan  
    //
    // Hint: Consider both the cells the laser hit and the cells it passed through.
    //define a ray that can be used incalc
    int n = scan.size();
    std::vector<adjusted_ray_t> ray;
    for (int i=0;n;i++)
    {
        ray = scan[i];
        scoreEndpoint(ray, map);
        scoreRay(ray,map);

    }
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits  
    
    int x = ray.range*cos(ray.theta);
    int y = ray.range*sin(ray.theta);
    int currentOdds = map.logOdds(x, y);
    int updatedOdds = currentOdds + kHitOdds_;
    map.setLogOdds(x,y,updatedOdds);  
    
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through  
    //takes in ray from maping update
    std::vector<Point<int>> rayPoints = bresenham(ray, map);
    Point<int> currentPoint; 
    int x;
    int y;
    int currentOdds;
    int updatedOdds;

    for (int i; rayPoints.size(); i++)
    {
        currentPoint = rayPoints[i];
        x = currentPoint.x;
        y = currentPoint.y;
        currentOdds = map.logOdds(x, y);
        updatedOdds = currentOdds - kMissOdds_;
        map.setLogOdds(x,y,updatedOdds);
    }  
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    Point<int> gridCellPoint = global_position_to_grid_cell(ray.origin, map);
    int x_o = gridCellPoint.x;
    int x_i = static_cast<int>(ray.range*cos(ray.theta));
    int y_o = gridCellPoint.y;
    int y_i = static_cast<int>(ray.range*sin(ray.theta));
    int sx;
    int sy;

    int dx = abs(x_i - x_o);
    int dy = abs(y_i - y_o);
    if((x_i - x_o) != 0)
    {
        sx = (x_i - x_o)/abs(x_i - x_o);
    }
    else
    {
        sx = 1;
    }
    if((x_i - x_o) != 0)
    {
        sy = (y_i - y_o)/abs(y_i - y_o);
    }
    else
    {
        sy = 1;
    }    

    int err = dx-dy;
    int x = x_o;
    int y = y_o;
    int e2 = 0;
    std::vector<Point<int>> points;
    points.push_back({x,y});
    while (x != x_i || y != y_i)
    {
        e2 = 2*err;
        if (e2 >= -dy) // this checks for x step
        {
            err = err - dy;
            sx = x + sx; 
        }
        if (e2 <= dx) // this checks for y step
        {
            err = err + dx;
            sy = y + sy;
        }        
        points.push_back({x,y});
    }
    return points; // is a vector of points that were touched by the ray. divide and step along gets the same answer but more geometry based logic
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray. 
    return {};
    
}
