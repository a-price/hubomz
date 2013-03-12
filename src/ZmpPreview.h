#ifndef _ZMPPREVIEW_H_
#define _ZMPPREVIEW_H_

#include <Eigen/Dense>
#include <iostream>

class ZmpPreview {
public:

  double T, h, g;
  size_t nl;

  Eigen::Matrix3d A;
  Eigen::Vector3d B;
  Eigen::RowVector3d C;

  Eigen::Matrix4d AA;
  Eigen::Vector4d BB;
  double RR;
  Eigen::Matrix4d QQ;
  
  Eigen::Matrix4d PP;

  double Ks;
  Eigen::RowVector3d Kx;

  Eigen::RowVectorXd G;

  ZmpPreview(double timestep,
	     double height,
	     size_t numlookahead,
	     double R=1e-6,
	     double Qe=1,
	     double Qdpos=0,
	     double Qdvel=0,
	     double Qdaccel=0,
	     double gaccel=9.8) {


    T = timestep;
    h = height;
    nl = numlookahead;
    g = gaccel;

    A << 
      1, T, T*T/2, 
      0, 1, T,
      0, 0, 1;

    B << T*T*T/6, T*T/2, T;

    C << 1, 0, -h/g;

    AA(0,0) = 1;
    AA.block<1,3>(0,1) = C*A;
    AA.block<3,1>(1,0).setZero();
    AA.block<3,3>(1,1) = A;

    BB(0,0) = C*B;
    BB.block<3,1>(1,0) = B;

    QQ << 
      Qe, 0, 0, 0,
      0, Qdpos, 0, 0, 
      0, 0, Qdvel, 0,
      0, 0, 0, Qdaccel;

    RR = R;
    
    /*
    std::cout << "A=\n" << A << "\n\n";
    std::cout << "B=\n" << B << "\n\n";
    std::cout << "C=\n" << C << "\n\n";
    std::cout << "AA=\n" << AA << "\n\n";
    std::cout << "BB=\n" << BB << "\n\n";
    std::cout << "QQ=\n" << QQ << "\n\n";
    std::cout << "RR=\n" << RR << "\n\n";
    */

    PP.setIdentity();
    bool converged = false;
    
    for (size_t i=0; i<1000; ++i) {
      Eigen::Matrix4d AX = AA.transpose() * PP;
      Eigen::Matrix4d AXA = AX * AA;
      Eigen::Vector4d AXB = AX * BB;
      double M = RR + BB.transpose()*PP*BB;
      Eigen::Matrix4d PPnew = AXA - AXB*(1/M)*AXB.transpose() + QQ;
      double relerr = (PPnew-PP).norm() / PPnew.norm();
      PP = PPnew;
      if (relerr < 1e-10) {
	std::cerr << "DARE solver converged after " << i << " iterations.\n";
	converged = true;
	break;
      }
    }

    if (!converged) {
      std::cerr << "*** WARNING: DARE solver failed to converge in ZmpPreview ***\n";
    }

    double SS = RR + BB.transpose() * PP * BB;
    
    Eigen::RowVector4d KK = SS * BB.transpose()*PP*AA;
    Ks = KK(0,0);
    Kx = KK.block<1,3>(0,1);
    
    Eigen::Matrix4d Ac = AA - BB*KK;
    Eigen::Vector4d CC;
    CC << 1, 0, 0, 0;

    Eigen::Vector4d XX = -Ac.transpose() * PP * CC;
    G = Eigen::RowVectorXd(nl);

    for (size_t i=0; i<nl; ++i) {
      G[i] = (SS * BB.transpose() * XX);
      XX = Ac.transpose() * XX;
    }
    
  }
	     

};

#endif
