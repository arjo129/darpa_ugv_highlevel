#ifndef _POSE_EST_H_
#define _POSE_EST_H_

#include <rviz/tool.h>
#include <OGRE/Ogre.h>
#include <ros/ros.h>
#include <rviz/ogre_helpers/shape.h>

namespace phy_planner {
    class PoseEstimateTool : public rviz::Tool {
        Q_OBJECT
    public:
        PoseEstimateTool();
        ~PoseEstimateTool();
        virtual void onInitialize();

        virtual void activate();
        virtual void deactivate();
        virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

        virtual void load( const rviz::Config& config );
        virtual void save( rviz::Config config ) const;

    private:
        Ogre::SceneNode* robotScene_;
        std::string flag_resource_;
        ros::ServiceClient client;
        rviz::Shape *shape;
    };
}
#endif