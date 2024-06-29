#include <iostream>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "gui/gui.h"
#include <cmath>

double quaternion_to_yaw(const geometry_msgs::Quaternion &quaternion)
{
    // since beta  = 0, -pi/2 < beta < pi/2
    double x = quaternion.x;
    double y = quaternion.y;
    double z = quaternion.z;
    double eta = quaternion.w;

    // alpha = atan2(r21, r11)
    double r11 = 2 * (pow(eta, 2) + pow(x, 2)) - 1;
    double r21 = 2 * (x * y + eta * z);
    double alpha = atan2(r21, r11);
    return alpha;
}

GUI::GUI(QWidget *parent) : QGLWidget(parent),
                            _timer(),
                            _zoom(5.0),
                            _center(0.0, 0.0),
                            _laserscan(),
                            _odom(),
                            _landmarks(),
                            _observed_landmarks(),
                            _observations(),
                            _map(),
                            _estimated_odom(),
                            _goal(),
                            _observed_landmarks_map(),
                            _simulated_obstacles(),
                            _projection()

{
    setMinimumSize(600, 600);
    setFocusPolicy(Qt::StrongFocus);
    connect(&_timer, SIGNAL(timeout()), this, SLOT(timer_callback()));
    _timer.start(10);
}

GUI::
    ~GUI() {}

void GUI::
    handleObservations(const perception::Observations::ConstPtr &msg)
{
    _observations = *msg;
    updateGL();
    return;
}

void GUI::
    handleLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    _laserscan = *msg;
    updateGL();
    return;
}
void GUI::
    handleOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    _odom = *msg;
    updateGL();
    return;
}

void GUI::
    handleMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    _map = *msg;
    return;
}

void GUI::
    handleSimulatedObstacles(const geometry_msgs::Polygon::ConstPtr &msg)
{
    _simulated_obstacles = *msg;
    return;
}

void GUI::handleEstimatedOdom(const nav_msgs::Odometry::ConstPtr &msg) {
    _estimated_odom = *msg;    
    return;
}
void GUI::handleGoal(const geometry_msgs::Pose::ConstPtr &msg) {
    _goal = *msg;   
    return;
}
void GUI::handleLandmarks(const perception::Landmarks::ConstPtr &msg) {
    _landmarks = *msg;    
    return;
}
void GUI::handleObservedLandmarks(const perception::Landmarks::ConstPtr &msg) {
    _observed_landmarks = *msg;    
    return;
}
void GUI::handleTrajectories(const navigation::Trajectories::ConstPtr &msg) {
	  std::cout << "Trajectories received" << std::endl;
    _trajectories = *msg;
    return;
}
void GUI::
    timer_callback(void)
{
    ros::spinOnce();
    return;
}
void GUI::
    initializeGL()
{
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    return;
}
void GUI::
    resizeGL(int width,
             int height)
{
    glViewport(0, 0, width, height);
    return;
}

void GUI::drawLandmarks(const perception::Landmarks &polygon,
                        const double &red,
                        const double &green,
                        const double &blue,
                        const double &size)
{
    glPushMatrix();
    unsigned int discretization = 60;
    glColor4f(red, green, blue, 1.0);
    for (const perception::Landmark &landmark : polygon.landmarks)
    {
        glPushMatrix();
        glLineWidth(2.0);
        double x_landmark = landmark.pos.x;
        double y_landmark = landmark.pos.y;
        int signature = landmark.signature;
        double radius = 0.05;

        glTranslated(x_landmark, y_landmark, 0.0);
        glBegin(GL_LINE_STRIP);
        for (unsigned int i = 0; i < discretization; i++)
        {
            double angle = 2.0 * M_PI * (double)(i) / (double)(discretization - 1);
            glVertex3f(radius * cos(angle), radius * sin(angle), 0.0);
        }
        glEnd();
        glPopMatrix();
    }
    glPopMatrix();
    return;
}

void GUI::drawObstacles(const geometry_msgs::Polygon &obstacles,
                        const double &r,
                        const double &g,
                        const double &b)

{
	glPushMatrix();	
	for (geometry_msgs::Point32 point : obstacles.points) {
		unsigned int discretization = 33;
		glColor4f(r, g, b, 1.0);
		glLineWidth(3.0);
		glBegin(GL_LINE_STRIP);
		for (unsigned int i = 0; i < discretization; i++)
			{
				double angle = 2.0 * M_PI * (double)(i) / (double)(discretization - 1);
				glVertex3f(point.z * cos(angle) + point.x, point.z * sin(angle) + point.y, 0.0);
			}
		glEnd();
	}
	glPopMatrix();
    return;
}

void GUI::drawTrajectories(const navigation::Trajectories &trajectories,
                            const double &red,
                            const double &green,
                            const double &blue) {

    glPushMatrix();
    for(navigation::Trajectory trajectory : _trajectories.trajectories) {
        glColor4f(red, green, blue, 1);
        glLineWidth(2.0);
        glBegin(GL_LINE_STRIP);
        for (geometry_msgs::Point32 point : trajectory.trajectory) {
            glVertex3f(point.x, point.y, 0);
        }
        glEnd();
    }
    glPopMatrix();
}

void GUI::
    paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double ratio = (double)(size().width()) / (double)(size().height());
    gluOrtho2D(-_zoom * ratio + _center.first, _zoom * ratio + _center.first, -_zoom + _center.second, _zoom + _center.second);
    glMatrixMode(GL_MODELVIEW);
    ///////////////////////////////////////////////////////////
    glLoadIdentity();
    drawMap(_map, 1.0, 1.0, 1.0);
    drawGrid();
    drawCoordinateSystem();
    drawLaserScan(_estimated_odom.pose.pose, _laserscan, 0.0, 0.0, 1.0, 0.75);
    drawObservations(_odom.pose.pose, _observations, 1.0, 0.0, 1.0, 1.0, 0.1 );
    drawRobot(_odom.pose.pose, 1.0, 0.0, 0.0, 0.1225);  // robot odom in red
    drawRobot(_estimated_odom.pose.pose, 0.2, 0.2, 0.2, 0.1225); // robot estimated odom in black
    //drawRobotSensorHorizon(_estimated_odom.pose.pose, 0.2, 0.2, 0.2, -M_PI / 4.0,M_PI / 4.0, 5.0);
    drawRobot(_goal, 0.8, 0.1, 0.1, 0.1225);
    drawLandmarks(_landmarks, 1.0, 0.0, 0.0, 14.0);
    drawLandmarks(_observed_landmarks, 0.5, 0.5, 0.0, 15.0);
    drawObstacles(_simulated_obstacles, 0.0, 0.0, 1.0);
    drawTrajectories(_trajectories,0.0, 0.8, 0.8);     
    ///////////////////////////////////////////////////////////   
    return;
}

void GUI::drawObservations(const geometry_msgs::Pose &pose,
                           const perception::Observations &observations,
                           const double &red,
                           const double &green,
                           const double &blue,
                           const double &alpha,
                           const double &radius)
{
    glPushMatrix();
    glTranslated(pose.position.x, pose.position.y, 0.0);
    glRotated(quaternion_to_yaw(pose.orientation) * 180.0 / M_PI, 0.0, 0.0,
              1.0);
    unsigned int discretization = 60;
    glColor4f(red, green, blue, alpha);
    for (const perception::Observation &observation : observations.observations)
    {
        glPushMatrix();
        glLineWidth(2.0);
        double range = observation.range;
        double bearing = observation.bearing;
        int signature = observation.signature;
        glRotated(bearing * 180 / M_PI, 0.0, 0.0, 1.0);
        glTranslated(range, 0.0, 0.0);
        glBegin(GL_LINE_STRIP);
        for (unsigned int i = 0; i < discretization; i++)
        {
            double angle = 2.0 * M_PI * (double)(i) / (double)(discretization - 1);
            glVertex3f(radius * cos(angle), radius * sin(angle), 0.0);
        }
        glEnd();
        glPopMatrix();
    }
    glPopMatrix();
}
void GUI::
    drawCoordinateSystem(void)
{
    glBegin(GL_LINES);
    glColor4f(1.0, 0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(1.0, 0.0, 0.0);
    glColor4f(0.0, 1.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 1.0, 0.0);
    glColor4f(0.0, 0.0, 1.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 1.0);
    glEnd();
    return;
}
void GUI::
    drawGrid(void)
{
    glColor4f(0.8, 0.8, 0.8, 1.0);
    glLineWidth(2.0);
    glBegin(GL_LINES);
    for (int i = -10; i <= 10; i++)
    {
        glVertex3f(-10.0, (double)(i), 0.0);
        glVertex3f(10.0, (double)(i), 0.0);
        glVertex3f((double)(i), -10.0, 0.0);
        glVertex3f((double)(i), 10.0, 0.0);
    }
    glEnd();
    glLineWidth(1.0);
}

void GUI::
drawLaserScan(const geometry_msgs::Pose &pose,
                  const sensor_msgs::LaserScan &laserscan,
                  const double &red,
                  const double &green,
                  const double &blue,
                  const double &alpha)
{
    glPushMatrix();
    glTranslated(pose.position.x, pose.position.y, 0.0);
    glRotated(quaternion_to_yaw(pose.orientation) * 180.0 / M_PI, 0.0, 0.0, 1.0);
    glColor4f(red, green, blue, alpha);
    glLineWidth(2.0);
    glBegin(GL_LINES);
    for (unsigned int i = 0; i < laserscan.ranges.size(); i++)
    {
        double angle = laserscan.angle_min + (double)(i)*laserscan.angle_increment;
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(laserscan.ranges[i] * cos(angle), laserscan.ranges[i] * sin(angle), 0.0);
    }
    glEnd();
    glLineWidth(1.0);
    glPopMatrix();
    return;
}

void GUI::
    drawMap(const nav_msgs::OccupancyGrid &map,
            const double &r,
            const double &g,
            const double &b)
{
    double half_discretization = map.info.resolution / 2.0;
    double min_x = -(double)(map.info.width - 1) * half_discretization;
    double min_y = -(double)(map.info.height - 1) * half_discretization;
    glPushMatrix();
    glBegin(GL_QUADS);
    for (unsigned int i = 0; i < map.info.width; i++)
    {
        double x = min_x + (double)(i)*map.info.resolution;
        for (unsigned int j = 0; j < map.info.height; j++)
        {
            double y = min_y + (double)(j)*map.info.resolution;
            double occ = 1.0 - (1.0 / (1.0 + exp((double)(map.data[i * map.info.height + j]) * 0.05)));
            glColor4f((1.0 - occ) * r, (1.0 - occ) * g, (1.0 - occ) * b, 1.0);
            glVertex3f(x - half_discretization,
                       y - half_discretization,
                       0.0);
            glVertex3f(x + half_discretization,
                       y - half_discretization,
                       0.0);
            glVertex3f(x + half_discretization,
                       y + half_discretization,
                       0.0);
            glVertex3f(x - half_discretization, y + half_discretization, 0.0);
        }
    }
    glEnd();
    glPopMatrix();
    return;
}
void GUI::drawRobotSensorHorizon(const geometry_msgs::Pose &pose,
                                 const double &red,
                                 const double &green,
                                 const double &blue,
                                 const double &minAngle,
                                 const double &maxAngle,
                                 const double &maxRange)
{
    return;
}
void GUI::
    drawRobot(const geometry_msgs::Pose &pose,
              const double &red,
              const double &green,
              const double &blue,
              const double &radius)
{
    glPushMatrix();
    glTranslated(pose.position.x, pose.position.y, 0.0);
    glRotated(quaternion_to_yaw(pose.orientation) * 180.0 / M_PI, 0.0, 0.0,
              1.0);
    unsigned int discretization = 33;
    glColor4f(red, green, blue, 1.0);
    glLineWidth(5.0);
    glBegin(GL_LINE_STRIP);
    for (unsigned int i = 0; i < discretization; i++)
    {
        double angle = 2.0 * M_PI * (double)(i) / (double)(discretization - 1);
        glVertex3f(radius * cos(angle), radius * sin(angle), 0.0);
    }
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(radius, 0.0, 0.0);
    glVertex3f(-radius, 0.0, 0.0);
    glEnd();
    glBegin(GL_TRIANGLES);
    glVertex3f(radius, 0.0, 0.0);
    glVertex3f(radius / 4.0, radius / 2.0, 0.0);
    glVertex3f(radius / 4.0, -radius / 2.0, 0.0);
    glEnd();
    glLineWidth(1.0);
    glPopMatrix();
    return;
}
void GUI::
    keyPressEvent(QKeyEvent *event)
{
    if (event->matches(QKeySequence::Copy))
    {
        close();
        return;
    }
    else
    {
        switch (event->key())
        {
        case Qt::Key_Left:
            _center.first -= 0.5;
            break;
        case Qt::Key_Right:
            _center.first += 0.5;
            break;
        case Qt::Key_Down:
            _center.second -= 0.5;
            break;
        case Qt::Key_Up:
            _center.second += 0.5;
            break;
        case Qt::Key_I:
            if (_zoom > 0.5)
            {
                _zoom -= 0.5;
            }
            break;
        case Qt::Key_O:
            _zoom += 0.5;
            break;
        default:
	    std::cout << "could not handle key " << event->key() << std::endl;
            break;
        }
        updateGL();
    }
    return;
}
