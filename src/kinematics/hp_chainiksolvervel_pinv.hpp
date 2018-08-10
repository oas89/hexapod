#ifndef KDL_HP_CHAIN_IKSOLVERVEL_PINV_HPP
#define KDL_HP_CHAIN_IKSOLVERVEL_PINV_HPP

#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/utilities/svd_HH.hpp>

namespace KDL {

class HP_ChainIkSolverVel_pinv : public ChainIkSolverVel {
public:
    explicit HP_ChainIkSolverVel_pinv(const Chain& _chain, double _eps=0.00001, int _maxiter=150)
        : chain(_chain)
        , jnt2jac(chain)
        , jac(chain.getNrOfJoints())
        , svd(jac)
        , U(6,JntArray(chain.getNrOfJoints()))
        , S(chain.getNrOfJoints())
        , V(chain.getNrOfJoints(), JntArray(chain.getNrOfJoints()))
        , tmp(chain.getNrOfJoints())
        , eps(_eps)
        , maxiter(_maxiter)
    {
    }
    
    ~HP_ChainIkSolverVel_pinv()
    {

    }

    virtual void updateInternalDataStructures()
    {
    }

    virtual int CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
    {
        double sum;
        unsigned int i,j;

        jnt2jac.JntToJac(q_in,jac);
        for (i=0;i<jac.columns();i++) {
        	for (j=3;j<jac.rows();j++) {
        		jac(j,i) = 0;
        	}
        }

        int ret = svd.calculate(jac,U,S,V,maxiter);

        for (i=0;i<jac.columns();i++) {
            sum = 0.0;
            for (j=0;j<jac.rows();j++) {
                sum+= U[j](i)*v_in(j);
            }
            tmp(i) = sum*(fabs(S(i))<eps?0.0:1.0/S(i));
        }
        for (i=0;i<jac.columns();i++) {
            sum = 0.0;
            for (j=0;j<jac.columns();j++) {
                sum+=V[i](j)*tmp(j);
            }
            qdot_out(i)=sum;
        }
        return ret;
    }

    virtual int CartToJnt(const JntArray& q_init, const FrameVel& v_in, JntArrayVel& q_out)
    {
        return -1;
    }

private:
        const Chain chain;
        ChainJntToJacSolver jnt2jac;
        Jacobian jac;
        SVD_HH svd;
        std::vector<JntArray> U;
        JntArray S;
        std::vector<JntArray> V;
        JntArray tmp;
        double eps;
        int maxiter;
};

}
#endif
