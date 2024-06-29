#ifndef GUI_H
#define GUI_H
#include <iostream>
#include <map>
#include "ros/ros.h"
#include <QtCore/QTimer>
#include <QtGui/QKeyEvent>
#include <QtWidgets/QWidget>
#include <QtOpenGL/QGLWidget>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point.h"
#include "perception/Observations.h"
#include "perception/Observation.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "perception/Landmarks.h"
#include "navigation/Trajectories.h"
#include "navigation/Trajectory.h"
class GUI : public QGLWidget
{
    Q_OBJECT
public:
    GUI(QWidget *parent = NULL);
    virtual ~GUI();
    void handleLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg);
    void handleOdom(const nav_msgs::Odometry::ConstPtr &msg);
    void handleObservations(const perception::Observations::ConstPtr &msg);
    void handleEstimatedOdom(const nav_msgs::Odometry::ConstPtr &msg);
    void handleGoal(const geometry_msgs::Pose::ConstPtr &msg);
    void handleLandmarks(const perception::Landmarks::ConstPtr &msg);
    void handleObservedLandmarks(const perception::Landmarks::ConstPtr &msg);
    void handleMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void handleSimulatedObstacles(const geometry_msgs::Polygon::ConstPtr &msg);
    void handleTrajectories(const navigation::Trajectories::ConstPtr &msg);
protected slots:
    void timer_callback(void);

protected:
    virtual void initializeGL();
    virtual void resizeGL(int width, int height);
    virtual void paintGL();
    virtual void drawObstacles(const geometry_msgs::Polygon &obstacles,
                               const double &r,
                               const double &g,
                               const double &b);

    void drawCoordinateSystem(void);
    void drawGrid();
    void drawObservations(const geometry_msgs::Pose &pose,
                          const perception::Observations &observations,
                          const double &red = 0.0,
                          const double &green = 0.0,
                          const double &blue = 1.0,
                          const double &alpha = 1.0,
                          const double &radius = 0.05);
    void drawLaserScan(const geometry_msgs::Pose& pose,
                       const sensor_msgs::LaserScan &laserscan,
                       const double &red,
                       const double &green,
                       const double &blue,
                       const double &alpha);
    virtual void keyPressEvent(QKeyEvent *event);
    void drawRobot(const geometry_msgs::Pose &pose,
                   const double &red = 0.0,
                   const double &green = 0.0,
                   const double &blue = 0.0,
                   const double &radius = 0.1225);
    void drawRobotSensorHorizon(const geometry_msgs::Pose &pose,
		    const double &red = 0.0,
		    const double &green = 0.0,
		    const double &blue = 0.0,
		    const double &minAngle = -M_PI / 4.0,
		    const double &maxAngle = M_PI / 4.0,
		    const double &maxRange = 5.0);

    void drawLandmarks(const perception::Landmarks &polygon,
                       const double &red = 0.0,
                       const double &green = 0.0,
                       const double &blue = 0.0,
                       const double &size = 1.0);
    void drawMap(const nav_msgs::OccupancyGrid &map,
                 const double &r,
                 const double &g,
                 const double &b);
    void drawTrajectories(const navigation::Trajectories &trajectories,
                            const double &red = 0.0,
                            const double &green = 0.0,
                            const double &blue = 1.0);

    QTimer _timer;
    double _zoom;
    std::pair<double, double> _center;

    sensor_msgs::LaserScan _laserscan;
    nav_msgs::Odometry _odom;
    nav_msgs::Odometry _estimated_odom;
    perception::Observations _observations;
    nav_msgs::OccupancyGrid _map;
    geometry_msgs::Pose _goal;
    nav_msgs::Path _projection;
    perception::Landmarks _landmarks;
    perception::Landmarks _observed_landmarks;
    std::map<int, geometry_msgs::Point> _observed_landmarks_map;
    geometry_msgs::Polygon _simulated_obstacles;
    navigation::Trajectories _trajectories;
};
#endif /* GUI_H */
