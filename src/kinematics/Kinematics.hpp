
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "hp_chainiksolvervel_pinv.hpp"
#include "hp_chainiksolverpos_nr_jl.hpp"
#include <iostream>

using KDL::PI;
using KDL::Chain;
using KDL::Frame;
using KDL::Vector;
using KDL::JntArray;
using KDL::ChainFkSolverPos_recursive;
using KDL::ChainIkSolverVel_pinv;
using KDL::ChainIkSolverPos_NR_JL;
using std::vector;


JntArray getLimit(double limit)
{
    JntArray jnt(3);
    jnt(0) = limit;
    jnt(1) = limit;
    jnt(2) = limit;
    return jnt;
}

const auto UPPER_LIMIT = getLimit(PI / 2);
const auto LOWER_LIMIT = getLimit(-PI / 2);


class Solver {
public:


    Solver(const Chain& chain)
        : dof(chain.getNrOfJoints())
        , lower_limits(dof)
        , upper_limits(dof)
        , fk_pos_solver(chain)
        , ik_vel_solver(chain)
        , ik_pos_solver(chain, LOWER_LIMIT, UPPER_LIMIT, fk_pos_solver, ik_vel_solver, 100, 0.01)
    {
        for (int i = 0; i < dof; i++) {
            lower_limits(i) = -PI / 2;
            upper_limits(i) = +PI / 2;
        }
    }

    bool inverse(const double* pos, double* angles)
    {
        Frame pos_in = Frame(Vector(pos[0], pos[1], pos[2]));
        JntArray jnt_in(dof);
        JntArray jnt_out(dof);

        for (int i = 0; i < dof; i++) {
            jnt_in(i) = angles[i];
        }

        int ret = ik_pos_solver.CartToJnt(jnt_in, pos_in, jnt_out);

        if (ret >= 0) {
            for (int i = 0; i < dof; i++) {
                angles[i] = jnt_out(i);
            } 
            return true;
        }
        std::cout << ik_pos_solver.strError(ret);
        return false;
    }

    bool forward(const double* angles, double* pos)
    {
        JntArray jnt_in(dof);
        Frame pos_out;

        for (int i = 0; i < dof; i++) {
            jnt_in(i) = angles[i];
        }

        int ret = fk_pos_solver.JntToCart(jnt_in, pos_out);

        if (ret >= 0) {
            pos[0] = pos_out.p[0];
            pos[1] = pos_out.p[1];
            pos[2] = pos_out.p[2];
            return true;            
        }

        return false;
    }

private:
    size_t dof;
    JntArray lower_limits;
    JntArray upper_limits;
    ChainFkSolverPos_recursive fk_pos_solver;
    KDL::HP_ChainIkSolverVel_pinv ik_vel_solver;
    KDL::HP_ChainIkSolverPos_NR_JL ik_pos_solver;
};


KDL::Chain buildChain(double x, double y, double angle, double axis)
{
    using KDL::PI;
    using KDL::Chain;
    using KDL::Frame;
    using KDL::Joint;
    using KDL::Segment;
    using KDL::Vector;
    using KDL::Rotation;

    Chain chain;

    Segment center = Segment("center", Joint(Joint::None), Frame(Vector(x, y, 0)));

    Rotation coaxRotation = Rotation::RPY(0, 0, angle);
    Vector coaxVector = Vector(0, 0, 0);
    Joint coaxJoint = Joint("coax", coaxVector, coaxRotation * Vector(0, 0, -1), Joint::RotAxis);
    Segment coaxSegment = Segment("coax", coaxJoint, Frame(coaxRotation, coaxVector));

    Rotation femurRotation = Rotation::RPY(-PI / 2, 0, 0);
    Vector femurVector = Vector(0.0294, 0, 0);
    Joint femurJoint = Joint("femur", femurVector, femurRotation * Vector(0, 0, axis), Joint::RotAxis);
    Segment femurSegment = Segment("femur", femurJoint, Frame(femurRotation, femurVector));

    Rotation tibiaRotation = Rotation::RPY(-PI, 0, PI / 2);
    Vector tibiaVector = Vector(0.08, 0, 0);
    Joint tibiaJoint = Joint("tibia", tibiaVector, tibiaRotation * Vector(0, 0, axis), Joint::RotAxis);
    Segment tibiaSegment = Segment("tibia", tibiaJoint, Frame(tibiaRotation, tibiaVector));

    Rotation footRotation = Rotation::RPY(0, 0, 0);
    Vector footVector = Vector(0.17, 0, 0);
    Joint footJoint = Joint("foot", Joint::None);
    Segment footSegment = Segment("foot", footJoint, Frame(footRotation, footVector));

    chain.addSegment(center);
    chain.addSegment(coaxSegment);
    chain.addSegment(femurSegment);
    chain.addSegment(tibiaSegment);
    chain.addSegment(footSegment);

    return chain;
}


// class Kinematics {
// public:
//     Kinematics(double af, double ac, double ar, double lc, double lf, double lt, double ox, double oy, double oyy)
//         : lf(chain(ox, -oy, -af, lc, lf, lt))
//         , lc(chain(0.0, -oyy, -ac, lc, lf, lt))
//         , lr(chain(-ox, -oy, -ar, lc, lf, lt))
//         , rf(chain(ox, oy, af, lc, lf, lt))
//         , rc(chain(0.0, oyy, aс, lc, lf, lt))
//         , rr(chain(-ox, oy, ar, lc, lf, lt))
//         , slf(lf)
//         , slc(lc)
//         , slr(lr)
//         , srf(rf)
//         , src(rc)
//         , srr(rr)
//     {
//     }

//     bool inverse(const double target[], double angles[])
//     {
//     }

// private:
//     KDL::Chain lf; // left front kinematic chain
//     KDL::Chain lc; // left center kinematic chain
//     KDL::Chain lr; // left rear kinematic chain
//     KDL::Chain rf; // right front kinematic chain
//     KDL::Chain rc; // right center kinematic chain
//     KDL::Chain rc; // right rear kinematic chain
//     Solver slf; // left front solver
//     Solver slc; // left center solver
//     Solver slr; // left rear solver
//     Solver srf; // right front solver
//     Solver src; // right center solver
//     Solver srr; // right rear solver
// };
