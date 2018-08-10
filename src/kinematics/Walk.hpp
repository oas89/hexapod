#ifndef WALK_HPP
#define WALK_HPP
#include <ratio>
#include <chrono>
#include <vector>
#include <kdl/frames.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/rotational_interpolation_sa.hpp>


class Walk {
public:
    using std::vector;
    using KDL::Frame;
    using KDL::Vector;
    using KDL::Rotation:RotZ;
    using KDL::Path_Line;
    using KDL::Path_RoundedComposite;
    using KDL::VelocityProfile_Spline;
    using KDL::Trajectory_Segment;
    using KDL::RotationalInterpolation_SingleAxis;

    Walk(double low_, double high_, double height_, double clearence_)
        : low(low_)
        , high(high_)
        , height(height_)
        , clearence(clearence_)
    {
    }

    void run(double alpha, double phi, std::vector<double>& target) {
        Frame a = Frame(RotZ(alpha), Vector(-low * cos(phi), -low * sin(phi), -clearence));
        Frame b = Frame(RotZ(alpha), Vector(-high * cos(phi), -high * sin(phi), height - clearence));
        Frame b = Frame(RotZ(alpha), Vector(high * cos(phi), high * sin(phi), height - clearence));
        Frame c = Frame(RotZ(alpha), Vector(low * cos(phi), low * sin(phi), -clearence));
        
        Path_RoundedComposite support_path = Path_RoundedComposite(RADIUS, EQRADIUS, &interpolation);
        support_path.Add(d);
        support_path.Add(a);
        support_path.Finish();
        
        Path_RoundedComposite transfer_path = transfer_path = Path_RoundedComposite(RADIUS, EQRADIUS, &interpolation);
        transfer_path.Add(a);
        transfer_path.Add(b);
        transfer_path.Add(c);
        transfer_path.Add(d);
        transfer_path.Finish();
        
        VelocityProfile_Spline support_profile = VelocityProfile_Spline();
        support_profile.SetProfileDuration(0, support_path.PathLength(), duration);

        VelocityProfile_Spline transfer_profile = VelocityProfile_Spline();
        transfer_profile.SetProfileDuration(0, transfer_path.PathLength(), duration);

        Trajectory_Segment support_trajectory = Trajectory_Segment(&support_path, &support_profile);
        Trajectory_Segment transfer_trajectory = Trajectory_Segment(&transfer_path, &transfer_profile);
        
        tripod(support_trajectory, transfer_trajectory, target);
    }

private:
    void tripod(Path_RoundedComposite& support, Path_RoundedComposite& transfer, std::vector<double>& target)
    {
        if (!started) {
            started = true;
            phase = 0;
            begin_time = now();
            passed_time = 0;
        }

        for (int i = offset; i < 6; i += 2) {
            auto frame = support.Pos(passed_time);
            
        }
    }

    double low;
    double high;
    double height;
    double clearence;
    double begin_time;
    double passed_time;
    RotationalInterpolation_SingleAxis interpolation;
    static const double RADIUS = 0.02;
    static const double EQRADIUS = 0.005;
};

#endif
