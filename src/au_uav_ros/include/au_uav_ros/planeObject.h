#ifndef _PLANE_OBJECT_H_
#define _PLANE_OBJECT_H_

#include "ros/ros.h"
#include <math.h>

#include "au_uav_ros/standardDefs.h"
#include "au_uav_ros/standardFuncs.h"

//Message Includes
#include "au_uav_ros/Command.h"
#include "au_uav_ros/Telemetry.h"

namespace au_uav_ros {
      class PlaneObject {
      public:
            /* Default constructor. Sets everything to zero. */
            PlaneObject(void);

            PlaneObject(struct waypoint wp);
      
            PlaneObject(int _id);
            
            /* Explicit value constructor: Takes a collision radius and a
            telemetry update and creates a new PlaneObject. */
            PlaneObject(const Telemetry &msg);

            /* Mutator functions */
            void setID(int id);
            void setPreviousLoc(double lat, double lon, double alt);
            void setCurrentLoc(double lat, double lon, double alt);
            void setTargetBearing(double tBearing);         /* set bearing to destination */
            void setCurrentBearing(double cBearing);  /* set current bearing in the air */
            void setSpeed(double speed);
	    void setCurrentDest(double lat, double lon, double alt);
	    void incrementCurrentWaypointIndex(void);
            void updateTime(void);

            /* Update the plane's data members with the information contained within the telemetry update. */
            bool update(const Telemetry &msg, Command &cmd);
	    void populatePlaneInfo(const Telemetry &msg);
	    bool isWaypointHit(const Telemetry &msg, Command &cmd);
            /* Accessor functions */
            int getID(void) const;
            waypoint getPreviousLoc(void) const;
            waypoint getCurrentLoc(void) const;
            double getTargetBearing(void) const;
            double getCurrentBearing(void) const;
            double getSpeed(void) const;
            double getLastUpdateTime(void) const;
	    int getCurrentWaypointIndex(void);
            waypoint getDestination(void) const;
            virtual double getSimSpeed(void) const;

            /* Find distance between this plane and another plane */
            double findDistance(const PlaneObject& plane) const;
            /* Find distance between this plane and another plane's latitude/longitude */
            double findDistance(double lat2, double lon2) const;

            /* Find Cartesian angle between this plane and another plane */
            double findAngle(const PlaneObject& plane) const;
            /* Find Cartesian angle between this plane and another plane's latitude/longitude */
            double findAngle(double lat2, double lon2) const;

            /* Overloaded equality operator */
            PlaneObject& operator=(const PlaneObject& pobj);

            //void setPlannedPath(std::vector<waypoint> path);
            void addAvoidanceWp(struct waypoint wp);
            void addNormalWp(struct waypoint wp);
            void removeAvoidanceWp(void);
            void removeNormalWp(struct waypoint wp);
            au_uav_ros::Command getPriorityCommand(void);


            /* Used for A* - fills vector with all normal waypoints, and avoidWp at the front */
            std::vector<waypoint> getNormalPath(void);

        protected:
            /* Private data members */
            int id;
            double targetBearing;         /* get bearing to destination */
            double currentBearing;        /* get current bearing in the air */
            double speed;
            double lastUpdateTime;
	    double distanceToDestination;
	    int currentWaypointIndex;
	    int MAX_NUMBER_OF_WAYPOINTS;	// used to calc currentWaypointIndex
            waypoint previousLoc;   /*used to calculate currentBearing*/
            waypoint currentLoc;
	    waypoint currentDest;
            std::list<struct waypoint> normalPath;
            //std::list<struct waypoint> plannedPath;
       
      };
}
#endif
