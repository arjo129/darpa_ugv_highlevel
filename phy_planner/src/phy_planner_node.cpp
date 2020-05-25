#include <ros/ros.h>
#include <phy_planner/final_pose_estimate.h>
#include <amapper/elevation_grid.h>
#include <set>
#include <tf/tf.h>
#include <optional>

#define tf_vec_debug(label , v) std::cout <<label << v.x() << ", " << v.y() << ", " <<v.z() <<std::endl;

typedef std::vector<tf::Point> ContactArea;

struct CylindricalCoordinateFrame {
    tf::Vector3 origin;
    tf::Vector3 axis;
    tf::Vector3 planar_axis;
};

struct TumblePoint {
    int  count; //Supported by 1 pivot or more than 1 pivot
    CylindricalCoordinateFrame axis;
};


CylindricalCoordinateFrame getPivotFromSinglePoint(tf::Vector3 center_of_mass, tf::Vector3 first_contact, tf::Vector3 current_normal) {
    auto f = center_of_mass-first_contact;
    auto pivot = f.cross(current_normal);
    CylindricalCoordinateFrame frame;
    frame.origin = first_contact;
    frame.axis = pivot;
    frame.planar_axis = pivot.cross(current_normal);
    return frame;
}
/**
 * Calculates the projection of a vector
 */ 
double projection(tf::Vector3 vec, tf::Vector3 onto) {
    return onto.dot(vec)/onto.length();
}
/**
 * Checks if a point is inside a line segment
 */ 
TumblePoint getTumbleAxis(tf::Vector3 start_lineseg, tf::Vector3 end_lineseg, tf::Vector3 point, tf::Vector3 normal=tf::Vector3(0,0,1)) {
    tf::Vector3 line = end_lineseg - start_lineseg;
    tf::Vector3 point_ref = point - start_lineseg;
    auto length = projection(point_ref, line);
    TumblePoint tp;
    if(length < 0) {
        tp.count = 1;
        tp.axis = getPivotFromSinglePoint(point, start_lineseg, normal);

    }
    else if (length > line.length()) {
        tp.count = 1;
        tp.axis = getPivotFromSinglePoint(point, end_lineseg, normal);
    }
    else {
        tp.count = 2;
        tp.axis = getPivotFromSinglePoint(point, start_lineseg+line.normalize()*length, normal);
    }
    return tp;
}

/**
 * Gets sample points in a rectangle
 */  
ContactArea rectangularSample(float width, float height, float sample_interval, float default_height=0){
    ContactArea points;
    for(float x = -width/2; x <= width/2; x += sample_interval) {
        for(float y = -height/2; y <= height/2; y += sample_interval) {
            tf::Point pt(x, y, default_height);
            points.push_back(pt);
        }
    }
    return points;
}

/**
 * Transforms sample points
 */ 
void transformPoints(ContactArea& sample_points, tf::Transform target_pose) {
    for(int i = 0 ; i < sample_points.size(); i++) {
        sample_points[i] = target_pose*sample_points[i];
    }
}

/**
 * Combines contact areas
 */ 
ContactArea combineContactArea(std::vector<ContactArea> contactAreas) {
    ContactArea final;
    for (auto area: contactAreas) {
        final.insert(final.end(), area.begin(), area.end());
    }
    return final;
}

/**
 * Extracts relevant amapper elevation points. Transforms the amapper point to local coordinates.
 */ 
std::vector<tf::Vector3> transformElevGridToLocalFrame(ContactArea ca, AMapper::ElevationGrid& grid, geometry_msgs::Pose pose) {
     //Transform the points to target position first
    auto pos = pose.position;
    auto _orientation = pose.orientation;
    tf::Vector3 origin(pos.x, pos.y, pos.z);
    tf::Quaternion orientation(_orientation.x, _orientation.y, _orientation.z, _orientation.w);
    tf::Transform target_pose(orientation, origin);

    std::vector<tf::Vector3> result;
    for(auto pt: ca) { // TODO: This is WRONG
        auto transformed = target_pose*pt;
        auto elevation = grid.queryElevation(transformed.x(), transformed.y());
        result.push_back(tf::Vector3(pt.x(), pt.y(), elevation));
    }
    return result;
}


void testTransformElevGridToLocalFrame() {

    ContactArea ca;
    ca.push_back(tf::Vector3(0,0,0));
    AMapper::ElevationGrid g;
    for(double x = 0 ; x < 5 ; x += 0.05) {
        for(double y = 0; y < 5; y += 0.05) {
            amapper::ElevationPoint pt;
            pt.x = x; 
            pt.y = y;
            pt.elevation = 1;
            g.add(pt);
        }
    }
    geometry_msgs::Pose p;
    p.position.x = 1;
    p.position.y = 1;
    p.position.z = 0;
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = 0;
    p.orientation.w = 1;
    ContactArea a= transformElevGridToLocalFrame(ca, g, p);
}
/**
 * Transforms the system to a cylindrical coordinate frame
 * @param a ContactArea is the area to be transformed, 
 * @param rotation axis is the axis of the pivot. 
 * @param origin is the origin of the ha
 */ 
ContactArea transformToCylindrical(ContactArea a, tf::Vector3 rotation_axis, tf::Vector3 planar_axis, tf::Vector3 origin) {
    
    ContactArea transformed;
    auto z_cartesian_axis = rotation_axis.cross(planar_axis).normalize();

    for(auto pt: a) {
        auto transformed_point = pt - origin;
        auto z = transformed_point.dot(rotation_axis.normalize());
        auto z_normal = (transformed_point - z*rotation_axis.normalize());
        auto r = z_normal.length();
        auto theta_proj = (z_normal - planar_axis).dot(z_cartesian_axis);
        auto sign = theta_proj >= 0 ? 1 :-1;
        double theta;
        if(planar_axis.length()*z_normal.length() != 0)
            theta = sign*acos( planar_axis.dot(z_normal)/(planar_axis.length()*z_normal.length()));
        else if(sign > 0) {
            theta = 0;
        }
        else {
            r *= -1;
            theta = 0;
        }

        if(isnan(theta)) {
            //This si due to FLOATING POINT ERROR. FLOATS ARE SHIT
            if(planar_axis.dot(z_normal)/(planar_axis.length()*z_normal.length() > 1))
                theta = sign*acos(1);
            else
            {
                theta = sign*acos(-1);
            }
        }
            
        transformed.push_back(tf::Vector3(z, r, theta));
    }

    return transformed;
}

/**
 * Cylindrical to cartesian
 */ 
tf::Point cylindricalToCartesian(tf::Point cylindrical, tf::Vector3 rotation_axis, tf::Vector3 planar_axis, tf::Vector3 origin) {
    //auto theta = cylindrical.y() < 0 ? M_PI + cylindrical.z(): cylindrical.z();
    auto point_on_axis = origin + rotation_axis*cylindrical.x();
    tf::Quaternion qt(rotation_axis, cylindrical.z());
    tf::Transform t(qt);
    auto offset = t*(planar_axis.normalize()*cylindrical.y());
    return point_on_axis + offset;
}

typedef AMapper::GenericCoordinateStorage<std::vector<float>> CylindricalMap;

void add(CylindricalMap& map, float r, float z, float theta) {
    auto _r = (long)(r*10.0f);
    auto _z = (long)(z*10.0f);
    auto coordinate = std::make_pair(_r,_z);
  // std::cout << "add " << r << "->"<< _r << ":" << z << "-> "<< _z <<"," << theta <<std::endl;
    map[coordinate].push_back(theta);
}

std::vector<float> find(CylindricalMap& map, float r, float z) {
    auto _r = (long)(r*10.0f);
    auto _z = (long)(z*10.0f);
    auto coordinate = std::make_pair(_r,_z);
    //std::cout << "query " << r << "->"<< _r << ":" << z << "-> "<< _z  <<std::endl;
    return map[coordinate];
}
/**
 * Determine first contact point in a cylindrical transformed coordinate system.
 * @returns the cylindrical coordinate system
 */ 
tf::Point getClosestCylindrical(ContactArea cylindrical, ContactArea footprint) {
    
    CylindricalMap map; 

    for(auto pt: cylindrical) {
        add(map, pt.x(), pt.y(), pt.z());
    }

    tf::Vector3 min_pt(0, 0, INFINITY);
    for(auto pt: footprint) {
        //tf_vec_debug("point", pt);
        for(auto t :  find(map, pt.x() , pt.y())){
            auto angle = pt.z() - t;
            if(angle < min_pt.z()) {
                min_pt = tf::Vector3(pt.x(), pt.y(), angle);
            }
        }
    }
    return min_pt;
}

struct TumbleResult{
    bool ok;
    float angle;
    tf::Vector3 intersectionPoint;
};
/**
 * Performs the actual tumble
 */ 
TumbleResult executeTumble(TumblePoint pt, ContactArea terrain, ContactArea footprint) {

    auto cylindricalTerrain = transformToCylindrical(terrain, pt.axis.axis, pt.axis.planar_axis, pt.axis.origin);
    auto cylindricalFootprint = transformToCylindrical(footprint, pt.axis.axis, pt.axis.planar_axis, pt.axis.origin);
    auto result = getClosestCylindrical(cylindricalTerrain, cylindricalFootprint);
    TumbleResult tr;
    tr.angle = result.z();
    tr.ok = false;
    tr.intersectionPoint = cylindricalToCartesian(result, pt.axis.axis, pt.axis.planar_axis, pt.axis.origin);
    if(tr.angle < 1.0) {
        tr.ok = true;
    }
    return tr;
}

void testExecuteTumble() {
    // Creates an aelevation map like so
    // a point at 
    //

    CylindricalCoordinateFrame frame;
    frame.axis = tf::Vector3(0,1,0);
    frame.planar_axis = tf::Vector3(1,0,0);
    frame.origin = tf::Vector3(0,0,10);

    TumblePoint pt;
    pt.axis = frame;
    pt.count = 1;

    ContactArea elevationMap;
    elevationMap = rectangularSample(10, 10, 0.05, 9.5);
    ContactArea robot; 
    robot = rectangularSample(2,2,0.05,10);
    
    auto res = executeTumble(pt, elevationMap, robot);

    std::cout<< "final pose" << res.ok << ", "<< res.angle << std::endl;
    tf_vec_debug("contact point", res.intersectionPoint);
}
void testTransformCylinder() {
    ContactArea ca;
    ca.push_back(tf::Vector3(-1,0,0));
    //
    //  Scenario
    //         axis of rotation
    //    |  /
    //    | /   x <--- le point
    //    |/_________  
    //      planar axis
    //
    //
    auto res = transformToCylindrical(ca, tf::Vector3(0,1,0), tf::Vector3(1,0,0), tf::Vector3(-1,0,0));
    std::cout << "got z:"<< res[0].x() << ", r:" << res[0].y() << ", theta:" << res[0].z() <<std::endl;
    auto inv= cylindricalToCartesian(res[0], tf::Vector3(0,1,0), tf::Vector3(1,0,0), tf::Vector3(-1,0,0));
    std::cout << "got x:"<< inv.x() << ", y:" << inv.y() << ", z:" << inv.z() <<std::endl;
    tf_vec_debug("begin", ca[0]);
    //assert(ca[0] == inv);
}

void testTransformCoplanar() {
    ContactArea ca;
    ca.push_back(tf::Vector3(1,0,1));
    //
    //  Scenario
    //         axis of rotation
    //    |  /
    //    | /   x <--- le point
    //    |/_________  
    //      planar axis
    //
    //
    auto res = transformToCylindrical(ca, tf::Vector3(0,0,1), tf::Vector3(1,0,0), tf::Vector3(-1,0,0));
    std::cout << "got z:"<< res[0].x() << ", r:" << res[0].y() << ", theta:" << res[0].z() <<std::endl;
    auto inv= cylindricalToCartesian(res[0], tf::Vector3(0,0,1), tf::Vector3(1,0,0), tf::Vector3(-1,0,0));
    std::cout << "got x:"<< inv.x() << ", y:" << inv.y() << ", z:" << inv.z() <<std::endl;
    //assert(ca[0] == inv);
}

/**
 * Gets point of first contact
 * @returns a pair containing (first contact in world reference, first contact in body frame of reference)
 */ 
tf::Vector3 getFirstContact(ContactArea ca, AMapper::ElevationGrid grid, geometry_msgs::Pose pose) {
    auto contactArea = transformElevGridToLocalFrame(ca, grid, pose);

    AMapper::ElevationGrid localFrame;
    for(auto pt: contactArea){
        amapper::ElevationPoint e;
        e.x = pt.x();
        e.y = pt.y();
        e.elevation = pt.z();
        localFrame.add(e);
    }    
    //Find first point TODO: reimplement as multiscale grid
    tf::Vector3 first_contact(0, 0, -INFINITY);
    for(auto point: ca) {
        auto elevation = localFrame.queryElevation(point.x(), point.y()) - point.z();
        if(elevation > first_contact.z()) {
            first_contact = tf::Vector3(point.x(), point.y(), elevation);
        }
    }
    return first_contact;
}

ContactArea rotate(TumblePoint tumble1, TumbleResult resultantTumble, ContactArea ca) {
    ContactArea res;
    for(auto a: ca) {
        res.push_back(a);
    }
    return res;
}

tf::Point rotate(TumblePoint tumble1, TumbleResult resultantTumble, tf::Point pt) {
    tf::Transform rotation;
    rotation.setOrigin(tf::Vector3(0,0,0));
    rotation.setRotation(tf::Quaternion(tumble1.axis.axis, resultantTumble.angle));
    return tumble1.axis.origin+rotation*(pt-tumble1.axis.origin);
}

void testRotateTumble() {
    TumblePoint pt;
    pt.axis.axis = tf::Vector3(0,0,1);
    pt.axis.origin =tf::Vector3(2,0,0);
    TumbleResult result;
    result.angle = 1.5708;
    auto final = rotate(pt, result, tf::Vector3(4,0,1));
    tf_vec_debug("final rotation", final);
}

struct FinalPose {
    float z;
    bool ok;
    tf::Quaternion quaternion;
};
/**
 * Builds a robot model 
 * Robot may be made of "Good" (wheels/track) and "Bad" contact points (body)
 * In this example we will model a robot as follows:
 * 
 *   []------[]
 *     |    |
 *     |    |
 *   []------[]
 * 
 * Body will be 0.5*0.5m it is raised 0.1m above the ground.
 * Wheels will be 0.1*0.1m mounted at the ends of the body.
 */ 
class RobotModel {
    void makeWheel(std::string name, double x, double y){
        auto wheel = rectangularSample(0.1, 0.1, 0.05);
        tf::Quaternion qt(0, 0, 0, 1);
        tf::Vector3 vec(x, y, 0);
        tf::Transform lfwheel_pos(qt,vec);
        transformPoints(wheel, lfwheel_pos);
        contactAreas[name] = wheel;
    }
public:
    std::unordered_map<std::string, ContactArea> contactAreas;
    std::unordered_map<std::string, double> elevation;
    std::unordered_map<std::string, tf::Vector3> alreadyMadeContact;

    RobotModel() {
        ContactArea body;
        body = rectangularSample(0.5, 0.5, 0.05, 0.1);
        contactAreas["body"] = body;
        makeWheel("left_back_wheel", -0.25, -0.25);
        makeWheel("right_back_wheel",0.25, -0.25);
        makeWheel("left_front_wheel", -0.25, 0.25);
        makeWheel("right_front_wheel", 0.25, 0.25);
    }
    
    int determineFirstContactPoint(AMapper::ElevationGrid grid, geometry_msgs::Pose pose){
        double elevation = -INFINITY;
        this->alreadyMadeContact.clear();
        std::string body_part;
        tf::Point first_contact(0,0,-INFINITY), first_contact_in_body_frame;
        for(auto area : contactAreas){
            auto vec = getFirstContact(area.second, grid, pose);
            if(vec.z() == -INFINITY) continue;
            auto world_frame  = vec - tf::Vector3(0,0, this->elevation[area.first]); //HERE BE BUGS
            if(world_frame.z() > first_contact.z()) {
                first_contact = world_frame;
                first_contact_in_body_frame = vec;
                body_part  = area.first;
            }
        }

        if(first_contact.z() == -INFINITY) {
            return -1;
        }

        for(auto area : contactAreas){
            auto vec = getFirstContact(area.second, grid, pose);
            if(vec.z() == -INFINITY) continue;
            auto world_frame  = vec - tf::Vector3(0,0, this->elevation[area.first]); //HERE BE BUGS
            if(abs(world_frame.z() - first_contact.z()) < 1e-2) {
                tf::Vector3 contactPoint = vec;
                this->alreadyMadeContact[area.first] = contactPoint;
            }
        }
        return 0;
    }

    ContactArea getRobotFootprint() {
        std::vector<ContactArea> areas;
        for(auto a: contactAreas) {
            if(elevation[a.first] != 0)
                continue;
            areas.push_back(a.second);
        }
        return combineContactArea(areas);
    }

    TumblePoint determineAxisOfTumble(tf::Vector3 currentAxis = tf::Vector3(0,0,1), tf::Vector3 com = tf::Vector3(0,0,0)){
        
        assert(this->alreadyMadeContact.size() > 0 && this->alreadyMadeContact.size() < 3);

        std::vector<std::string> keys;
        for(auto pt: this->alreadyMadeContact){
            keys.push_back(pt.first);
        }
        //robot makes only one contact 
        if(this->alreadyMadeContact.size() == 1) {
            TumblePoint t;
            t.count = 1;
            t.axis = getPivotFromSinglePoint(com, this->alreadyMadeContact[keys[0]], currentAxis);
            return t;
        }
        else if (this->alreadyMadeContact.size() == 2){
            return getTumbleAxis(alreadyMadeContact[keys[0]], alreadyMadeContact[keys[1]], this->getCenterOfMass(), currentAxis);
        }
    }

    TumbleResult secondContactPoint(AMapper::ElevationGrid& grid, TumblePoint firstTumble, geometry_msgs::Pose pose) {
        std::unordered_map<std::string, ContactArea> transformedPoints;
        auto height = firstTumble.axis.origin.z();
        tf::Transform transformHeight;
        transformHeight.setOrigin(tf::Vector3(0,0,height));
        transformHeight.setRotation(tf::Quaternion(0,0,0));
        
        for(auto area: this->contactAreas) { 
            ContactArea part(area.second);
            transformPoints(part, transformHeight);
            transformedPoints[area.first] = part;  
        }
        return tumbleAlong(grid, firstTumble, pose, transformedPoints);
    }

    TumbleResult thirdContactPoint(AMapper::ElevationGrid& grid, TumblePoint firstTumble, TumbleResult amount, TumblePoint finalTumble, geometry_msgs::Pose pose) {
        std::unordered_map<std::string, ContactArea> transformedPoints;
        auto height = firstTumble.axis.origin.z();
        tf::Transform transformHeight;
        transformHeight.setOrigin(tf::Vector3(0,0,height));
        transformHeight.setRotation(tf::Quaternion(0,0,0));
       
        
        for(auto area: this->contactAreas) { 
            ContactArea part(area.second);
            transformPoints(part, transformHeight);
            auto rotated = rotate(firstTumble, amount, part);
            transformedPoints[area.first] = rotated;  
        }
        return tumbleAlong(grid, firstTumble, pose, transformedPoints);
    }

    TumbleResult tumbleAlong(AMapper::ElevationGrid& grid, TumblePoint firstTumble, geometry_msgs::Pose pose, std::unordered_map<std::string, ContactArea>& transformedPoints) {
        ContactArea elevation = transformElevGridToLocalFrame(rectangularSample(2,2,0.05), grid, pose);
        TumbleResult minTumble;
        minTumble.angle = INFINITY;
        minTumble.ok = false;

        std::string contactArea;
        for(auto part: transformedPoints){
            
            if(alreadyMadeContact.count(part.first) || part.first == "body") continue;
            auto result = executeTumble(firstTumble, elevation, part.second);
            if(minTumble.angle > result.angle) {
                contactArea = part.first;
                minTumble = result;
            }    
        }

        this->alreadyMadeContact[contactArea] = minTumble.intersectionPoint;

        for(auto part: transformedPoints){
            
            if(alreadyMadeContact.count(part.first) || part.first == "body") continue;

            auto result = executeTumble(firstTumble, elevation, part.second);
            //std::cout << result.angle << " "<<minTumble.angle - result.angle <<std::endl;
            if(abs(minTumble.angle - result.angle) < 1e-6) {
                this->alreadyMadeContact[part.first] = result.intersectionPoint;
            }
        }

        return minTumble;
    }


    tf::Vector3 getCenterOfMass() {
        return tf::Vector3(0,0,0);
    }

    tf::Quaternion determineRotation() {

        assert(this->alreadyMadeContact.size() >= 3);

        std::vector<tf::Vector3> contacts;
        for(auto contactPoint: this->alreadyMadeContact) {
            auto v = contactPoint.second;
            contacts.push_back(v);
        }

        auto v1 = contacts[1] - contacts[0];
        auto v2 = contacts[2] - contacts[0];
        auto normal = v1.cross(v2);
        tf::Vector3 zaxis(0,0,1);

        auto rotation = normal.cross(zaxis);
        auto w = zaxis.dot(normal) + sqrt(zaxis.length2()*normal.length2());
        tf_vec_debug("rotation", rotation);
        std::cout <<w <<std::endl;
        return tf::Quaternion(rotation.x(), rotation.y(), rotation.z(), w).normalize(); 
    }


    FinalPose tumble(AMapper::ElevationGrid& elevationMap, geometry_msgs::Pose pose) {
        

        this->determineFirstContactPoint(elevationMap, pose);

        tf::Vector3 firstContact;
        if(this->alreadyMadeContact.size() >= 3){
            FinalPose finalPose;
            finalPose.ok = true;
            finalPose.z = this->alreadyMadeContact.begin()->second.z();
            finalPose.quaternion = tf::Quaternion(0,0,0,1);
            return finalPose;
        }

        if(this->alreadyMadeContact.size() == 0){
            FinalPose finalPose;
            finalPose.ok = false;
            ROS_ERROR("No contact point. Is the point cloud empty?");
            return finalPose;
        }
        std::cout << "first drop" << std::endl;
        
        auto tumble1 = this->determineAxisOfTumble();
        auto resultantTumble = this->secondContactPoint(elevationMap, tumble1, pose);

        for(auto pt: this->alreadyMadeContact){
             tf_vec_debug(pt.first, pt.second);
        }
        std::cout << "second drop" << std::endl;
        
        if(!resultantTumble.ok) {
            std::cout << "fell off" << std::endl;
            FinalPose finalPose;
            finalPose.ok = false;
            return finalPose;
        }

        if(this->alreadyMadeContact.size() >= 3){
            std::cout << "landed" << std::endl;
            FinalPose finalPose;
            finalPose.ok = true;
            double sum = 0;
            for(auto contactPoint: this->alreadyMadeContact) {
                sum += contactPoint.second.z();
            }
            finalPose.z = sum/this->alreadyMadeContact.size();
            finalPose.quaternion = determineRotation();
            std::cout << "returning" << std::endl;
            return finalPose;  
        }
        
        auto com = this->getCenterOfMass();
        tf::Transform trans(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,firstContact.z()));
        com = trans*com;
        auto transformedCom = rotate(tumble1, resultantTumble, com);
        auto qt = tf::Quaternion(tumble1.axis.axis, resultantTumble.angle);
        tf::Transform tr(qt);
        auto tumble2 = this->determineAxisOfTumble(tr*tf::Vector3(0,0,1), transformedCom);
        TumbleResult res =  this->thirdContactPoint(elevationMap, tumble1, resultantTumble, tumble2, pose);
         std::cout << "final drop" << std::endl;
        if (!res.ok) {
            FinalPose finalPose;
            finalPose.ok = false;
            return finalPose;
        }

        if(this->alreadyMadeContact.size() >= 3){
            FinalPose finalPose;
            finalPose.ok = true;
            double sum = 0;
            for(auto contactPoint: this->alreadyMadeContact) {
                sum += contactPoint.second.z();
            }
            finalPose.z = sum/this->alreadyMadeContact.size();
            finalPose.quaternion = determineRotation();
            return finalPose;  
        }
        FinalPose finalPose;
        finalPose.ok = false;
        return finalPose;
    }
};


void testElevation() {
    AMapper::ElevationGrid g;
    for(double x = -5 ; x < 5 ; x += 0.05) {
        for(double y = -5; y < 5; y += 0.05) {
            amapper::ElevationPoint pt;
            pt.x = x; 
            pt.y = y;
            if(x >0 && y > 0){
                pt.elevation = 1;
            }
            else {
                pt.elevation = 0.8;
            }
            g.add(pt);
        }
    }

    RobotModel robot;
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    robot.tumble(g, pose);
    
    //std::cout << "got second contact at " << second.first << ". Contact point is " << second.second.x() << ", " <<second.second.y() << ", " << second.second.z() << std::endl;
}

AMapper::ElevationGrid grid;

void onElevMsg(amapper::ElevationGridMsg elevation_grid) {
    grid = AMapper::ElevationGrid(elevation_grid);
}


/**
 * Calculates the pose after vehicle drops onto said location
 * 
 *  - First determine first contact point. (Highest point in elevation map)
 *  - Next determine axis of pitch
 *  - Wait for next contact - repeat 
 *  - Determine location of final contact
 */ 
bool phy_pose_estimate(phy_planner::final_pose_estimate::Request &req,
                       phy_planner::final_pose_estimate::Response &res) {
    
    res.final_pose.position = req.target_pose.position;
    RobotModel robot;
    auto pose = robot.tumble(grid, req.target_pose);
    tf::Quaternion qt(req.target_pose.orientation.x, req.target_pose.orientation.y, req.target_pose.orientation.z, req.target_pose.orientation.w);
    auto final_rotation = pose.quaternion*qt;
    res.final_pose.orientation.x = final_rotation.x();
    res.final_pose.orientation.y = final_rotation.y();
    res.final_pose.orientation.z = final_rotation.z();
    res.final_pose.orientation.w = final_rotation.w(); 
    res.final_pose.position.z = pose.z;
    res.ok = pose.ok;
    return true;
}



int main(int argc, char**argv) {
    
    ros::init(argc, argv, "final_pose_estimation");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("pose_estimate", phy_pose_estimate);
    ros::Subscriber sub = n.subscribe("/plane_segmentation/grid_map", 10, onElevMsg);
    ros::spin();
    //testTransformCylinder();

    //testRotateTumble();
    //testElevation();
    //testExecuteTumble();
    //testTransformElevGridToLocalFrame();
}