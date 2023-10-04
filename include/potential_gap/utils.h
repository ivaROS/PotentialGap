#ifndef GAP_UTILS_H
#define GAP_UTILS_H


#include <ros/ros.h>
#include <math.h>
#include <potential_gap/gap.h>
#include <potential_gap/potentialgap_config.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
#include <potential_gap/robot_geo_parser.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

namespace potential_gap {
namespace utils {
    constexpr auto float_inf = std::numeric_limits<float>::infinity();

    inline float sq(float x) {
        return x * x;
    }

    inline Eigen::Vector2f car2pol(Eigen::Vector2f a) {
        return Eigen::Vector2f(a.norm(), float(std::atan2(a(1), a(0))));
    }

    inline Eigen::Vector2f pol2car(Eigen::Vector2f a) {
        return Eigen::Vector2f(cos(a(1)) * a(0), sin(a(1)) * a(0));
    }

    inline Eigen::Vector2f pTheta(
        float th, float phiB, Eigen::Vector2f pRp, Eigen::Vector2f pLp) {
        return pLp * (th - pRp(1)) / phiB + pRp * (pLp(1) - th) / phiB;
    }

    inline bool is_inf(Eigen::Vector2f vec)
    {
        if(vec(0) == float_inf || vec(1) == float_inf)
        {
            return true;
        }

        return false;
    }

    inline Eigen::Vector2f findScaleLineCirInt(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, float r, bool extend_outside)
    {
        float epsilon = 1e-2;
        epsilon = extend_outside ? epsilon : -epsilon;
        Eigen::Vector2f diff_vec = p2 - p1;

        float a = sq(diff_vec.norm());
        float b = 2 * diff_vec.dot(p1);
        float c = sq(p1.norm()) - sq(r + epsilon);

        float det = sq(b) - 4 * a * c;
        if(det < 0)
        {
            ROS_ERROR_STREAM("b^2-4ac is smaller than 0.");
            return Eigen::Vector2f(float_inf, float_inf);
        }

        float root1 = (-b + sqrt(det)) / (2 * a);
        float root2 = (-b - sqrt(det)) / (2 * a);
        float root;

        if(extend_outside)
        {
            if(root1 >= 1)
            {
                root = root1;
            }
            else if(root1 < 0 && root2 >= 1)
            {
                root = root2;
            }
            else
            {
                ROS_ERROR_STREAM("root1 and root2 are invalid.");
                return Eigen::Vector2f(float_inf, float_inf);
            }
        }
        else
        {
            if(root1 >= 1 && root1 <= root2)
            {
                root = root1;
            }
            else if(root2 >= 1 && root2 < root1)
            {
                root = root2;
            }
            else
            {
                ROS_ERROR_STREAM("root1 and root2 are invalid.");
                return Eigen::Vector2f(float_inf, float_inf);
            }
        }

        Eigen::Vector2f new_p2 = root * diff_vec + p1;

        return new_p2;
    }

    inline Eigen::Vector2f intersectLines(Eigen::Vector2f p1, Eigen::Vector2f p2, Eigen::Vector2f p3, Eigen::Vector2f p4 )
    {
        // Line AB represented as a1x + b1y = c1
        float a1 = p2(1) - p1(1);
        float b1 = p1(0) - p2(0);
        float c1 = a1 * p1(0) + b1 * p1(1);
    
        // Line CD represented as a2x + b2y = c2
        float a2 = p4(1) - p3(1);
        float b2 = p3(0) - p4(0);
        float c2 = a2 * p3(0) + b2 * p3(1);
    
        float determinant = a1 * b2 - a2 * b1;
    
        if (determinant == 0.)
        {
            // The lines are parallel. This is simplified
            // by returning a pair of FLT_MAX
            return Eigen::Vector2f(float_inf, float_inf);
        }
        else
        {
            float x = (b2 * c1 - b1 * c2) / determinant;
            float y = (a1 * c2 - a2 * c1) / determinant;
            return Eigen::Vector2f(x, y);
        }
    }

    inline std::vector<Eigen::Vector2f> intersectLineCir(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &cp, float r, bool segment)
    {
        constexpr auto eps = 1e-3;

        std::vector<Eigen::Vector2f> res;
        auto x0 = cp(0);
        auto y0 = cp(1);
        auto x1 = p1(0);
        auto y1 = p1(1);
        auto x2 = p2(0);
        auto y2 = p2(1);
        auto A = y2 - y1;
        auto B = x1 - x2;
        auto C = x2 * y1 - x1 * y2;
        auto a = A * A + B * B;
        float b, c;
        bool bnz = true;
        if (abs(B) >= 1e-2) {
            b = 2 * (A * C + A * B * y0 - sq(B) * x0);
            c = sq(C) + 2 * B * C * y0 - sq(B) * (sq(r) - sq(x0) - sq(y0));
        } else {
            b = 2 * (B * C + A * B * x0 - sq(A) * y0);
            c = sq(C) + 2 * A * C * x0 - sq(A) * (sq(r) - sq(x0) - sq(y0));
            bnz = false;
        }
        auto d = sq(b) - 4 * a * c; // discriminant
        if (d < 0) {
            return res;
        }

        // checks whether a point is within a segment
        auto within = [x1, y1, x2, y2](float x, float y) {
            auto d1 = sqrt(sq(x2 - x1) + sq(y2 - y1));  // distance between end-points
            auto d2 = sqrt(sq(x - x1) + sq(y - y1));    // distance from point to one end
            auto d3 = sqrt(sq(x2 - x) + sq(y2 - y));    // distance from point to other end
            auto delta = d1 - d2 - d3;
            return abs(delta) < eps;                    // true if delta is less than a small tolerance
        };

        auto fx = [A, B, C](float x) {
            return -(A * x + C) / B;
        };

        auto fy = [A, B, C](float y) {
            return -(B * y + C) / A;
        };

        auto rxy = [segment, &res, within](float x, float y) {
            if (!segment || within(x, y)) {
                res.push_back(Eigen::Vector2f(x, y));
            }
        };

        float x, y;
        if (d == 0.) {
            // line is tangent to circle, so just one intersect at most
            if (bnz) {
                x = -b / (2 * a);
                y = fx(x);
                rxy(x, y);
            } else {
                y = -b / (2 * a);
                x = fy(y);
                rxy(x, y);
            }
        } else {
            // two intersects at most
            d = sqrt(d);
            if (bnz) {
                x = (-b + d) / (2 * a);
                y = fx(x);
                rxy(x, y);
                x = (-b - d) / (2 * a);
                y = fx(x);
                rxy(x, y);
            } else {
                y = (-b + d) / (2 * a);
                x = fy(y);
                rxy(x, y);
                y = (-b - d) / (2 * a);
                x = fy(y);
                rxy(x, y);
            }
        }

        return res;
    }

    inline Eigen::Vector2f circTangPt(const Eigen::Vector2f& circ_orig, const Eigen::Vector2f& pt, float r, bool ccw)
    {   
        float eps = 1e-3;

        Eigen::Vector2f pt_vec = pt - circ_orig;
        float dist = pt_vec.norm();

        if(abs(dist - r) <= eps)
            return pt;

        if((dist - r) < -eps || dist < eps)
            throw("[circTangPt] pt is inside circle or invalid.");

        // assert((dist - r) > 1e-2);
        // assert(dist != 0);

        float theta = acos(r / dist);
        float pt_ang = atan2(pt_vec(1), pt_vec(0));

        float tang_ang = ccw ? (pt_ang + theta) : (pt_ang - theta);
        Eigen::Vector2f tang_pt = circ_orig + r * Eigen::Vector2f(cos(tang_ang), sin(tang_ang));

        return tang_pt;
    }

    inline Eigen::Vector2f getMidPt(const Eigen::Vector2f& orig, const Eigen::Vector2f& p1, const Eigen::Vector2f& p2)
    {
        Eigen::Vector2f vec1 = p1 - orig;
        Eigen::Vector2f vec2 = p2 - orig;

        Eigen::Vector2f mid_vec = (vec1 + vec2) / 2 + orig;

        return mid_vec;
    }

    inline void arrangeAngCCW(const float ang1, const float ang2, float& angl, float& angu)
    {
        angl = ang1 > ang2 ? ang2 : ang1;
        angu = ang1 > ang2 ? ang1 : ang2;

        return;
    }

    inline bool ptInsideCirc(const Eigen::Vector2f& orig, const Eigen::Vector2f& pt, float r)
    {
        Eigen::Vector2f vec = pt - orig;
        // ROS_INFO_STREAM("[ptInsideCirc] " << vec.norm() << " " << r);
        return (vec.norm() - r) <= 0.;
    }

    inline bool isLeftofLine(Eigen::Vector2f l1, Eigen::Vector2f l2, Eigen::Vector2f p)
    {
        return ((l2[0] - l1[0])*(p[1] - l1[1]) - (l2[1] - l1[1])*(p[0] - l1[0])) >= 0;
    }

    inline bool isLargerAngle(Eigen::Vector2f v1, Eigen::Vector2f v2)
    {
        // v1 angle is larger than and equal to v2 angle ccw
        double ang_1 = atan2(v1[1], v1[0]);
        double ang_2 = atan2(v2[1], v2[0]);

        return ang_1 >= ang_2;
    }

    inline bool isLeftVec(Eigen::Vector2f v1, Eigen::Vector2f v2)
    {
        // v1 angle is larger than and equal to v2 angle ccw
        double ang_1 = atan2(v1[1], v1[0]);
        double ang_2 = atan2(v2[1], v2[0]);

        if((ang_1 - ang_2) > M_PI)
        {
            ang_1 -= 2 * M_PI;
        }

        return ang_1 >= ang_2;
    }

    inline Eigen::Vector2f getRotatedVec(Eigen::Vector2f orig_vec, float chord_length, bool ccw)
    {
        float r = orig_vec.norm();
        float rotate_angle = acos((2 * r * r - chord_length * chord_length) / (2 * r * r));

        Eigen::Matrix2f rotate_mat;
        if(ccw)
            rotate_mat << cos(rotate_angle), -sin(rotate_angle), sin(rotate_angle), cos(rotate_angle);
        else
            rotate_mat << cos(rotate_angle), sin(rotate_angle), -sin(rotate_angle), cos(rotate_angle);

        Eigen::Vector2f rotated_vec = rotate_mat * orig_vec;

        return rotated_vec;
    }

    struct Point
    {
        float x, y;
        
        Point()
        {
            x = 0.;
            y = 0.;
        }

        Point(Eigen::Vector2f vec)
        {
            x = vec(0);
            y = vec(1);
        }
    };
    

    // Given three collinear points p, q, r, the function checks if
    // point q lies on line segment 'pr'
    inline bool onSegment(Point p, Point q, Point r)
    {
        if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
            return true;
    
        return false;
    }
    
    // To find orientation of ordered triplet (p, q, r).
    // The function returns following values
    // 0 --> p, q and r are collinear
    // 1 --> Clockwise
    // 2 --> Counterclockwise
    inline int orientation(Point p, Point q, Point r)
    {
        // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
        // for details of below formula.
        float val = (q.y - p.y) * (r.x - q.x) -
                (q.x - p.x) * (r.y - q.y);
    
        if (val == 0) return 0;  // collinear
    
        return (val > 0)? 1: 2; // clock or counterclock wise
    }
    
    // The main function that returns true if line segment 'p1q1'
    // and 'p2q2' intersect.
    inline bool doIntersect(Point p1, Point q1, Point p2, Point q2)
    {
        // Find the four orientations needed for general and
        // special cases
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);
    
        // General case
        if (o1 != o2 && o3 != o4)
            return true;
    
        // Special Cases
        // p1, q1 and p2 are collinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    
        // p1, q1 and q2 are collinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    
        // p2, q2 and p1 are collinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    
        // p2, q2 and q1 are collinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;
    
        return false; // Doesn't fall in any of the above cases
    }

    // Function to return the minimum distance
    // between a line segment AB and a point E
    inline Eigen::Vector2f minDistancePt(Point A, Point B, Point E)
    {
    
        // vector AB
        Point AB;
        AB.x = B.x - A.x;
        AB.y = B.y - A.y;
    
        // vector BP
        Point BE;
        BE.x = E.x - B.x;
        BE.y = E.y - B.y;
    
        // vector AP
        Point AE;
        AE.x = E.x - A.x,
        AE.y = E.y - A.y;
    
        // Variables to store dot product
        float AB_BE, AB_AE;
    
        // Calculating the dot product
        AB_BE = (AB.x * BE.x + AB.y * BE.y);
        AB_AE = (AB.x * AE.x + AB.y * AE.y);
    
        // Minimum distance from
        // point E to the line segment
        Eigen::Vector2f reqAns;
    
        // Case 1
        if (AB_BE > 0) {
    
            // Finding the magnitude
            float y = B.y;
            float x = B.x;
            reqAns = Eigen::Vector2f(x, y);
            // reqAns = sqrt(x * x + y * y);
        }
    
        // Case 2
        else if (AB_AE < 0) {
            float y = A.y;
            float x = A.x;
            reqAns = Eigen::Vector2f(x, y);
            // reqAns = sqrt(x * x + y * y);
        }
    
        // Case 3
        else {
    
            // Finding the perpendicular distance
            float x1 = AB.x;
            float y1 = AB.y;
            float x2 = AE.x;
            float y2 = AE.y;
            float mod = x1 * x1 + y1 * y1;
            if(mod <= 1e-3)
            {
                reqAns = Eigen::Vector2f(A.x, A.y);
            }
            else
            {
                float u = (x1 * x2 + y1 * y2) / mod;
                float p_x = A.x + u * x1;
                float p_y = A.y + u * y1;
                reqAns = Eigen::Vector2f(p_x, p_y);
            }
            // reqAns = abs(x1 * y2 - y1 * x2) / mod;
        }
        return reqAns;
    }

    inline void visualizeStaticInfGap(const potential_gap::StaticInfGap& static_inf_gap, const ros::Publisher& ros_pub)
    {
        visualization_msgs::MarkerArray marker_arr;

        visualization_msgs::Marker marker_l;
        marker_l.header.stamp = ros::Time::now();
        marker_l.header.frame_id = static_inf_gap.getMapFrame();
        marker_l.ns = "static_inf_gap";
        marker_l.id = 0;
        marker_l.type = visualization_msgs::Marker::LINE_STRIP;
        marker_l.action = visualization_msgs::Marker::ADD;
        double thick = 0.03;
        marker_l.scale.x = thick;
        marker_l.scale.y = thick;
        marker_l.scale.z = thick;
        marker_l.color.r = 1;
        marker_l.color.g = 0;
        marker_l.color.b = 1;
        marker_l.color.a = 0.6;
        marker_l.pose.orientation.w = 1;

        Eigen::Vector2d xc = static_inf_gap.getOrigMapPoseVec();
        double r = static_inf_gap.getMinDist();
        Eigen::Vector2d l_int = static_inf_gap.getLIntMapVec();
        Eigen::Vector2d r_int = static_inf_gap.getRIntMapVec();
        Eigen::Vector2d lgap = static_inf_gap.getLMapVec();
        Eigen::Vector2d rgap = static_inf_gap.getRMapVec();

        geometry_msgs::Point l1;
        l1.x = l_int(0);
        l1.y = l_int(1);
        geometry_msgs::Point l2;
        l2.x = lgap(0);
        l2.y = lgap(1);
        marker_l.points.push_back(l1);
        marker_l.points.push_back(l2);

        marker_arr.markers.push_back(marker_l);

        visualization_msgs::Marker marker_r = marker_l;
        marker_r.id += 1;
        geometry_msgs::Point r1;
        r1.x = r_int(0);
        r1.y = r_int(1);
        geometry_msgs::Point r2;
        r2.x = rgap(0);
        r2.y = rgap(1);
        marker_r.points.clear();
        marker_r.points.push_back(r1);
        marker_r.points.push_back(r2);

        marker_arr.markers.push_back(marker_r);

        visualization_msgs::Marker marker_circ = marker_r;
        marker_circ.id += 1;
        marker_circ.points.clear();
        double ang_res = 2 * M_PI / 500;
        for(double ang = -M_PI; ang <= M_PI; ang += ang_res)
        {
            geometry_msgs::Point circ;
            circ.x = r * cos(ang) + xc(0);
            circ.y = r * sin(ang) + xc(1);
            marker_circ.points.push_back(circ);
        }

        marker_arr.markers.push_back(marker_circ);

        ros_pub.publish(marker_arr);
    }

}
}

#endif