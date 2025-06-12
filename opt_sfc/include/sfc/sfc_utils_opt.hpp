#ifndef SFC_UTILS_OPT_HPP
#define SFC_UTILS_OPT_HPP

#include "gcopter/minco.hpp"  // Needed for gcopter::minco to evaluate the trajectory
#include "gcopter/lbfgs.hpp"  // Needed for gcopter::lbfgs optimization
#include "gcopter/sdlp.hpp"   // Needed for gcopter::sdlp optimization

#include "sfc/sfc_utils.hpp"

//The optimization formulation follows the GCOPTER paper.

namespace sfc_utils_opt
{
    typedef Eigen::Matrix3Xd PolyhedronV;
    typedef std::vector<Eigen::Matrix3Xd> PolyhedraV;
    typedef Eigen::MatrixX4d PolyhedronH;
    typedef std::vector<Eigen::MatrixX4d> PolyhedraH;

    struct EllipsoidR
    {
        EllipsoidR() {}
        EllipsoidR(Eigen::Vector3d d, Eigen::Vector3d r, 
                 Eigen::Matrix3d R, Eigen::Matrix3Xd pc, Eigen::Matrix<double, 6, 4> bd)
            : d(d), r(r), R(R), pc(pc), bd(bd) {}

        Eigen::Vector3d d, r;
        Eigen::Matrix3d R;
        Eigen::Matrix3Xd pc;
        Eigen::Matrix<double, 6, 4> bd;

        // const Eigen::Matrix3d Q = R * (r.cwiseProduct(r)).asDiagonal() * R.transpose();
        double dist(const Eigen::Vector3d &p) const
        {    
            Eigen::Matrix3d Q = R * (r.cwiseProduct(r)).asDiagonal() * R.transpose();
            return (p - d).transpose() * Q.inverse() * (p - d);
        }

        bool inside(const Eigen::Vector3d &p,
                    const double epsilon = 1.0e-6) const
        {
            return dist(p) < 1.0 + epsilon;
        }

        double distMax(const Eigen::Vector3d &p,
                       const double &mu = 1e-4) const
        {
            double x = dist(p) - 1.0;
            if (x < 0.0)
            {
                return 0;
            }
            else if (x > mu)
            {
                return x - 0.5 * mu;
            }
            else
            {
                const double xdmu = x / mu;
                const double sqrxdmu = xdmu * xdmu;
                const double mumxd2 = mu - 0.5 * x;
                return mumxd2 * sqrxdmu * xdmu;
            }
        }
    };

    typedef std::vector<EllipsoidR>  EllipsoidsR;



    /*********************************************************************************************************/
    // Function to process the corridor
    inline bool processCorridor(const PolyhedraH &hPs,
                                      PolyhedraV &vPs)
    {
        const int sizeCorridor = hPs.size() - 1;

        vPs.clear();
        vPs.reserve(2 * sizeCorridor + 1);

        int nv;
        PolyhedronH curIH;
        PolyhedronV curIV, curIOB;
        for (int i = 0; i < sizeCorridor; i++)
        {
            if (!geo_utils::enumerateVs(hPs[i], curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
            curIH.topRows(hPs[i].rows()) = hPs[i];
            curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
            if (!geo_utils::enumerateVs(curIH, curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);
        }

        if (!geo_utils::enumerateVs(hPs.back(), curIV))
        {
            return false;
        }
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB.col(0) = curIV.col(0);
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        vPs.push_back(curIOB);

        return true;
    }

    inline double costMVIEP(void *data,
                           const Eigen::VectorXd &x,
                           Eigen::VectorXd &grad)
    {
        const int *pM = (int *)data;
        const double *pSmoothEps = (double *)(pM + 1);
        const double *pPenaltyWt = pSmoothEps + 1;
        const double *pA = pPenaltyWt + 1;

        const int M = *pM;
        const double *pPath = pA + 3 * M;
        const double *pY = pPath + 6;
        const double *prho = pY + 2;

        const double smoothEps = *pSmoothEps;
        const double penaltyWt = *pPenaltyWt;
        const double rho = *prho;

        Eigen::Map<const Eigen::MatrixX3d> A(pA, M, 3);
        Eigen::Map<const Eigen::Vector3d> p(x.data());
        Eigen::Map<const Eigen::Vector3d> rtd(x.data() + 3);
        Eigen::Map<const Eigen::Vector3d> cde(x.data() + 6);
        Eigen::Map<Eigen::Vector3d> gdp(grad.data());
        Eigen::Map<Eigen::Vector3d> gdrtd(grad.data() + 3);
        Eigen::Map<Eigen::Vector3d> gdcde(grad.data() + 6);
        Eigen::Map<const Eigen::Vector3d> path1(pPath);
        Eigen::Map<const Eigen::Vector3d> path2(pPath + 3);
        Eigen::Map<const Eigen::Vector2d> y(pY);

        double cost = 0;
        gdp.setZero(); // gradient towards point p
        gdrtd.setZero(); // gradient towards the rotation vector rtd
        gdcde.setZero(); // gradient towards the center vector cde

        Eigen::Matrix3d L;
        L(0, 0) = rtd(0) * rtd(0) + DBL_EPSILON;
        L(0, 1) = 0.0;
        L(0, 2) = 0.0;
        L(1, 0) = cde(0);
        L(1, 1) = rtd(1) * rtd(1) + DBL_EPSILON;
        L(1, 2) = 0.0;
        L(2, 0) = cde(2);
        L(2, 1) = cde(1);
        L(2, 2) = rtd(2) * rtd(2) + DBL_EPSILON;

        const Eigen::MatrixX3d AL = A * L;
        const Eigen::VectorXd normAL = AL.rowwise().norm();
        const Eigen::Matrix3Xd adjNormAL = (AL.array().colwise() / normAL.array()).transpose();
        const Eigen::VectorXd consViola = (normAL + A * p).array() - 1.0;
         

        // 1. The cost and gradient of the penalty term for the polytope constraints
        double c, dc;
        Eigen::Vector3d vec;
        for (int i = 0; i < M; ++i)
        {
            if (sfc_utils::smoothedL1(smoothEps, consViola(i), c, dc))
            {
                cost += c;
                vec = dc * A.row(i).transpose();
                gdp += vec;
                gdrtd += adjNormAL.col(i).cwiseProduct(vec);
                gdcde(0) += adjNormAL(0, i) * vec(1);
                gdcde(1) += adjNormAL(1, i) * vec(2);
                gdcde(2) += adjNormAL(0, i) * vec(2);
            }
        }
        cost *= penaltyWt;
        gdp *= penaltyWt;
        gdrtd *= penaltyWt;
        gdcde *= penaltyWt;  // for polytope weights

        Eigen::Matrix<double, 3, 2> pathRtd;
        pathRtd.col(0) = path1 - p;
        pathRtd.col(1) = path2 - p;


        Eigen::Matrix3d forward = L.inverse();
        Eigen::Matrix3d gdL;


        for (int i = 0; i < 2; i++)
        {
            Eigen::Vector3d tempd = forward * pathRtd.col(i);
            //std::cout << "tempd is " << tempd.transpose() << std::endl;
            //double dist = tempd.norm() - 1.0;
            double dist = tempd.squaredNorm() - 1.0;
            if (sfc_utils::smoothedL1(smoothEps, dist, c, dc))
            {
                // https://www.matrixcalculus.org/
                cost += 0.5 * rho * (c + y(i)) * (c + y(i));
                //compute the gradient towards point p, rtd, and cde
                double grad_scale = rho * (c + y(i)) * dc * (-2.0);
                gdp  += grad_scale * forward.transpose() * tempd;
                gdL   = grad_scale * forward.transpose() * tempd * tempd.transpose();

                gdrtd(0) += gdL(0, 0);
                gdrtd(1) += gdL(1, 1);
                gdrtd(2) += gdL(2, 2);

                gdcde(0) += gdL(1, 0);
                gdcde(1) += gdL(2, 1);
                gdcde(2) += gdL(2, 0);

            }

        }

        //part 3: maximum volume
        double weight = 10.0;
        double vol_cost = weight * (log(L(0, 0)) + log(L(1, 1)) + log(L(2, 2)));
        cost -= vol_cost;
        gdrtd(0) -= weight / L(0, 0);
        gdrtd(1) -= weight / L(1, 1);
        gdrtd(2) -= weight / L(2, 2);

        //////////////for all 
        gdrtd(0) *= 2.0 * rtd(0);
        gdrtd(1) *= 2.0 * rtd(1);
        gdrtd(2) *= 2.0 * rtd(2);

        return cost;
    }

    
    // Function to find the maximum volume inscribed ellipsoid while the waypoints should be inside the ellipsoid
    inline bool maxVolInsOutEllipsoid(PolyhedronH hPoly,
                                      Eigen::Vector3d point1,
                                      Eigen::Vector3d point2,
                                      Eigen::Vector2d y,
                                      EllipsoidR &rEllip,
                                      double smoothEps = 1.0e-4,
                                      double rho = 1.0e+3)
    {
        Eigen::Matrix3d R = rEllip.R;
        Eigen::Vector3d r = rEllip.r;
        Eigen::Vector3d p = rEllip.d;

        // // Find the deepest interior point
        const int M = hPoly.rows();
        Eigen::MatrixX4d Alp(M, 4);
        Eigen::VectorXd blp(M);
        Eigen::Vector4d clp, xlp;

        const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
        Alp.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm;
        Alp.rightCols<1>().setConstant(1.0);
        blp = -hPoly.rightCols<1>().array() / hNorm;
        clp.setZero();
        clp(3) = -1.0;
        const double maxdepth = -sdlp::linprog<4>(clp, Alp, blp, xlp);
        if (!(maxdepth > 0.0) || std::isinf(maxdepth))
        {
            return false;
        }
        const Eigen::Vector3d interior = xlp.head<3>();

        // Prepare the data for MVIE optimization
        uint8_t *optData = new uint8_t[sizeof(int) + (2 + 3 * M + 2 * 3 + 2 + 1) * sizeof(double)];

        
        int *pM = (int *)optData;
        double *pSmoothEps = (double *)(pM + 1);
        double *pPenaltyWt = pSmoothEps + 1;
        double *pA = pPenaltyWt + 1;
        double *pPath = pA + 3 * M;
        double *pY = pPath + 6;
        double *prho = pY + 2;
        *prho = rho;

        Eigen::Map<Eigen::Vector3d> path1(pPath);
        Eigen::Map<Eigen::Vector3d> path2(pPath + 3);
        path1 = point1 - interior;
        path2 = point2 - interior;
     
        Eigen::Map<Eigen::Vector2d> Y(pY);
        Y = y;

        *pM = M;
        Eigen::Map<Eigen::MatrixX3d> A(pA, M, 3);
        A = Alp.leftCols<3>().array().colwise() /
            (blp - Alp.leftCols<3>() * interior).array();

        Eigen::VectorXd x(9);
        const Eigen::Matrix3d Q = R * (r.cwiseProduct(r)).asDiagonal() * R.transpose();
        Eigen::Matrix3d L;
        sfc_utils::chol3d(Q, L);


        x.head<3>() = p - interior;
        x(3) = sqrt(L(0, 0));
        x(4) = sqrt(L(1, 1));
        x(5) = sqrt(L(2, 2));
        x(6) = L(1, 0);
        x(7) = L(2, 1);
        x(8) = L(2, 0);

        double minCost;
        lbfgs::lbfgs_parameter_t paramsMVIEP;
        paramsMVIEP.mem_size = 24;
        paramsMVIEP.g_epsilon = 1e-6;
        paramsMVIEP.past = 3;
        paramsMVIEP.delta = 1.0e-5;
        *pSmoothEps = smoothEps; // control the precision of point inside the ellipsoid
        *pPenaltyWt = 1.0e+3; // this is the weight of the penalty term for the polytope constraints

        //std::cout << "[maxVolInsOutEllipsoid] start!  ";
        int res = lbfgs::lbfgs_optimize(x,
                                        minCost,
                                        &costMVIEP,
                                        nullptr,
                                        nullptr,
                                        optData,
                                        paramsMVIEP);
        // if (res < 0)
        // {
        //     std::cout << "[maxVolInsOutEllipsoid] Optimization failed: " << lbfgs::lbfgs_strerror(res) << std::endl;

        // }
        // else
        // {
        //     std::cout << "[maxVolInsOutEllipsoid] Optimization succeeded!" << std::endl;
        // }
        p = x.head<3>() + interior;

        L(0, 0) = x(3) * x(3);
        L(0, 1) = 0.0;
        L(0, 2) = 0.0;
        L(1, 0) = x(6);
        L(1, 1) = x(4) * x(4);
        L(1, 2) = 0.0;
        L(2, 0) = x(8);
        L(2, 1) = x(7);
        L(2, 2) = x(5) * x(5);
        Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::FullPivHouseholderQRPreconditioner> svd(L, Eigen::ComputeFullU);
        const Eigen::Matrix3d U = svd.matrixU();
        const Eigen::Vector3d S = svd.singularValues();
        if (U.determinant() < 0.0)
        {
            R.col(0) = U.col(1);
            R.col(1) = U.col(0);
            R.col(2) = U.col(2);
            r(0) = S(1);
            r(1) = S(0);
            r(2) = S(2);
        }
        else
        {
            R = U;
            r = S;
        }

        rEllip.R = R;
        rEllip.d = p;
        rEllip.r = r;

        delete[] optData;

        return res >= 0;

    }

    // not normalized polytopes
    inline bool findOutPolytope(EllipsoidR ellip,
                                PolyhedronH &hPoly,
                                double epsilon = 1.0e-6)
    {
        
        Eigen::MatrixX4d bd = ellip.bd;
        Eigen::Matrix3Xd pc = ellip.pc;
        Eigen::Matrix3d R = ellip.R;
        Eigen::Vector3d d = ellip.d;
        Eigen::Vector3d r = ellip.r;

        //convert the space to ball space
        const Eigen::Matrix3d forward = r.cwiseInverse().asDiagonal() * R.transpose();
        const Eigen::Matrix3d backward = R * r.asDiagonal();

        
        //compute the forward map boundary B, D, PC
        const Eigen::MatrixX3d forwardB = bd.leftCols<3>() * backward;
        const Eigen::VectorXd  forwardD = bd.rightCols<1>() + bd.leftCols<3>() * d;
        const Eigen::Matrix3Xd forwardPC = forward * (pc.colwise() - d);
        const Eigen::VectorXd distDs = forwardD.cwiseAbs().cwiseQuotient(forwardB.rowwise().norm());
        
        const int M = bd.rows();
        const int N = pc.cols();
        Eigen::MatrixX4d forwardH(M + N, 4);
        int nH = 0;

        Eigen::MatrixX4d tangents(N, 4);
        Eigen::VectorXd distRs(N);

        for (int i = 0; i < N; i++)
        {
            distRs(i) = forwardPC.col(i).norm();
            tangents(i, 3) = -distRs(i);
            tangents.block<1, 3>(i, 0) = forwardPC.col(i).transpose() / distRs(i);
        }

        Eigen::Matrix<uint8_t, -1, 1> bdFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(M, 1);
        Eigen::Matrix<uint8_t, -1, 1> pcFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);

        nH = 0;

        bool completed = false;
        int bdMinId = 0, pcMinId = 0;
        double minSqrD = distDs.minCoeff(&bdMinId);
        double minSqrR = INFINITY;
        if (distRs.size() != 0)
        {
            minSqrR = distRs.minCoeff(&pcMinId);
        }
        for (int i = 0; !completed && i < (M + N); ++i)
        {
            if (minSqrD < minSqrR)
            {
                forwardH.block<1, 3>(nH, 0) = forwardB.row(bdMinId);
                forwardH(nH, 3) = forwardD(bdMinId);
                bdFlags(bdMinId) = 0;
            }
            else
            {
                forwardH.row(nH) = tangents.row(pcMinId);
                pcFlags(pcMinId) = 0;
            }

            completed = true;
            minSqrD = INFINITY;
            for (int j = 0; j < M; ++j)
            {
                if (bdFlags(j))
                {
                    completed = false;
                    if (minSqrD > distDs(j))
                    {
                        bdMinId = j;
                        minSqrD = distDs(j);
                    }
                }
            }
            minSqrR = INFINITY;
            for (int j = 0; j < N; ++j)
            {
                if (pcFlags(j))
                {
                    if (forwardH.block<1, 3>(nH, 0).dot(forwardPC.col(j)) + forwardH(nH, 3) > -epsilon)
                    {
                        pcFlags(j) = 0;
                    }
                    else
                    {
                        completed = false;
                        if (minSqrR > distRs(j))
                        {
                            pcMinId = j;
                            minSqrR = distRs(j);
                        }
                    }
                }
            }
            ++nH;
        }

        hPoly.resize(nH, 4);
        for (int i = 0; i < nH; ++i)
        {
            hPoly.block<1, 3>(i, 0) = forwardH.block<1, 3>(i, 0) * forward;
            hPoly(i, 3) = forwardH(i, 3) - hPoly.block<1, 3>(i, 0).dot(d);
        }

        return true;
    }


/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
    static inline double costDistance(void *ptr,
                                        const Eigen::VectorXd &xi,
                                        Eigen::VectorXd &gradXi)
    {
        void **dataPtrs = (void **)ptr;
        const double &dEps = *((const double *)(dataPtrs[0]));
        const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));
        const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));
        const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));
        const Eigen::Matrix2Xd &ys = *((Eigen::Matrix2Xd *)(dataPtrs[4]));
        const EllipsoidsR &rEllips = *((EllipsoidsR *)(dataPtrs[5]));
        const double &rho = *((const double *)(dataPtrs[6]));

        double cost = 0.0;
        const int overlaps = vPolys.size() / 2;   //the overlapped waypoints

        Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
        Eigen::Vector3d a, b, d;
        Eigen::VectorXd r;
        double smoothedDistance;

        //add cost and gradients about ADMM cost
        Eigen::Vector3d pathRtd;
        Eigen::Matrix3d forward;
        double c, dc;
        

        double totalDistance = 0.0;

        for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
        {
            a = i == 0 ? ini : b;
            if (i < overlaps)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                r = q.normalized().head(k - 1);
                b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                    vPolys[2 * i + 1].col(0); // next point
            }
            else
            {
                b = fin;
            }


            d = b - a;
            smoothedDistance = sqrt(d.squaredNorm() + dEps);
            totalDistance += smoothedDistance;
            cost += smoothedDistance;

            if (i < overlaps)
            {
                gradP.col(i) += d / smoothedDistance;
            }
            if (i > 0)
            {
                gradP.col(i - 1) -= d / smoothedDistance;
            }

            if (i > 0 && i < overlaps)
            {
                for (size_t l = 0; l < 2; l++)
                {
                    //std::cout << "i is " << i << "  l is " << l << std::endl;
                    pathRtd = b - rEllips[i + l].d;
                    forward = rEllips[i + l].r.cwiseInverse().asDiagonal() * rEllips[i + l].R.transpose();
                    Eigen::Vector3d tempd = forward * pathRtd;
                    double dist = tempd.norm() - 1.0;
                    if (sfc_utils::smoothedL1(dEps, dist, c, dc))
                    {
                        double y = ys(1 - l, i + l);
                        cost += 0.5 * rho * (c + y) * (c + y);
                        //compute the gradient towards point p, rtd, and cde
                        gradP.col(i) += rho * (c + y) * dc * (2.0) * forward.transpose() * tempd;
                    }
                }
            }
        }

        Eigen::VectorXd unitQ;
        double sqrNormQ, invNormQ, sqrNormViolation;
        for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
        {
            k = vPolys[2 * i + 1].cols();
            Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
            Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
            sqrNormQ = q.squaredNorm();
            invNormQ = 1.0 / std::sqrt(sqrNormQ + 1e-12);
            unitQ = q * invNormQ;
            gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                unitQ.head(k - 1).array() * 2.0;
            gradQ(k - 1) = 0.0;
            gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

            sqrNormViolation = sqrNormQ - 1.0;
            if (sqrNormViolation > 0.0)
            {
                c = sqrNormViolation * sqrNormViolation;
                dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradQ += dc * 2.0 * q;
            }
        }

        return cost;
    }


    static inline double getDistHeuristics(const Eigen::Vector3d &ini,
                                        const Eigen::Vector3d &fin,
                                        const PolyhedraH &hpolys,
                                        const Eigen::Matrix2Xd &ys,
                                        const EllipsoidsR &rEllips,
                                        Eigen::Matrix3Xd &path,
                                        const double &smoothD = 1.0e-2,
                                        const double &rho = 1.0e+3)
    {
        PolyhedraV vPolys;
        PolyhedraH hNormPolys = hpolys;

        for (auto &poly : hNormPolys)
        {
            const Eigen::ArrayXd norms = poly.leftCols<3>().rowwise().norm();
            poly.array().colwise() /= norms;
        }

        if (!processCorridor(hNormPolys, vPolys))
        {
            std::cout << "[getDistHeuristics] Polytope processing failed!" << std::endl;
            return -1.0;
        }

        const int overlaps = vPolys.size() / 2;
        Eigen::VectorXi vSizes(overlaps);
        for (int i = 0; i < overlaps; ++i)
        {
            vSizes(i) = vPolys[2 * i + 1].cols();
        }

        Eigen::VectorXd xi(vSizes.sum());
        for (int i = 0, idx = 0; i < overlaps; ++i)
        {
            xi.segment(idx, vSizes(i)).setConstant(std::sqrt(1.0 / vSizes(i)));
            idx += vSizes(i);
        }

        double cost;
        void *dataPtrs[7] = {
            (void *)&smoothD,
            (void *)&ini,
            (void *)&fin,
            (void *)&vPolys,
            (void *)&ys,
            (void *)&rEllips,
            (void *)&rho
        };

        lbfgs::lbfgs_parameter_t param;
        param.g_epsilon = 1e-4;
        param.delta = 1e-3;
        param.max_iterations = 100;
        param.max_linesearch = 50;



        int res = lbfgs::lbfgs_optimize(xi, cost, costDistance, nullptr, nullptr, dataPtrs, param);
        if (res < 0)
        {
            // std::cout << "[getDistHeuristics] Optimization failed: " << lbfgs::lbfgs_strerror(res)
            // << " cost = " << cost << std::endl;
            return cost;
        }


        path.resize(3, overlaps + 2);
        path.col(0) = ini;
        path.col(overlaps + 1) = fin;

        for (int i = 0, idx = 0; i < overlaps; ++i)
        {
            int k = vPolys[2 * i + 1].cols();
            Eigen::Map<const Eigen::VectorXd> q(xi.data() + idx, k);
            Eigen::VectorXd r = q.normalized().head(k - 1);

            path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) + vPolys[2 * i + 1].col(0);
            idx += k;
        }

        double minDistance = 0.0;
        for (int i = 0; i < overlaps + 1; ++i)
        {
            minDistance += (path.col(i + 1) - path.col(i)).norm();
        }

        return minDistance;
    }

    static inline void forwardP(const Eigen::VectorXd &xi,
                                const Eigen::VectorXi &vIdx,
                                const PolyhedraV &vPolys,
                                Eigen::Matrix3Xd &P)
    {
        const int sizeP = vIdx.size();
        P.resize(3, sizeP);
        Eigen::VectorXd q;
        for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
        {
            l = vIdx(i);
            k = vPolys[l].cols();
            q = xi.segment(j, k).normalized().head(k - 1);
            P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                        vPolys[l].col(0);
        }
        return;
    }

    static inline double costDistanceContorl(void *ptr,
                                             const Eigen::VectorXd &xi,
                                             Eigen::VectorXd &gradXi)
    {
        void **dataPtrs = (void **)ptr;
        
        const PolyhedraH &hpolys = *((PolyhedraH *)(dataPtrs[0]));
        const Eigen::Matrix2Xd &ys = *((Eigen::Matrix2Xd *)(dataPtrs[1]));
        const EllipsoidsR &rEllips = *((EllipsoidsR *)(dataPtrs[2]));
        minco::MINCO_S3NU &minco = *((minco::MINCO_S3NU *)(dataPtrs[3]));
        const double &smoothEps = *((const double *)(dataPtrs[4]));
        const Eigen::VectorXd &times = *((const Eigen::VectorXd *)(dataPtrs[5]));
        const double &rho = *((const double *)(dataPtrs[6]));
        // const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[7]));


        //get points from xi
        Eigen::Map<const Eigen::Matrix3Xd> points(xi.data(), 3, hpolys.size()-1);
        //std::cout << "points is " << points << std::endl;
        double cost = 0.0;
        //minco cost and gradients
        Eigen::MatrixX3d partialGradByCoeffs;
        Eigen::Matrix3Xd gradByPoints = Eigen::Matrix3Xd::Zero(3, hpolys.size()-1);


        minco.setParameters(points, times);
        minco.getEnergy(cost);
        minco.getEnergyPartialGradByCoeffs(partialGradByCoeffs);

        auto coeffs = minco.getCoeffs();

        int res = 20;
        const double integralFrac = 1.0 / res;
        double step;
        Eigen::Vector3d pos, outerNormal;
        double violaPosPena, violaPosPenaD, violaPos;
        double s1, s2, s3, s4, s5;
        Eigen::Matrix<double, 6, 1> beta0;

        for (size_t i = 0; i < hpolys.size(); i++)
        {
            const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);
            step = integralFrac;
            for (int j = 0; j < res; j++)
            { 
                Eigen::Vector3d gradPos;
                gradPos.setZero();
                double pena = 0.0;
                
                s1 = j * step;
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s4 * s1;
                beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
                pos = c.transpose() * beta0;
                int K = hpolys[i].rows();
                for (int k = 0; k < K; k++)
                {
                    outerNormal = hpolys[i].block<1, 3>(k, 0);
                    violaPos = outerNormal.dot(pos) + hpolys[i](k, 3);
                    if (sfc_utils::smoothedL1(smoothEps, violaPos, violaPosPena, violaPosPenaD))
                    {
                        //std::cout << "violaPos is " << violaPos << std::endl;
                        gradPos += 1.0e+3 * violaPosPenaD * outerNormal;
                        pena += 1.0e+3 * violaPosPena;
                    }
                }
                double node = (j == 0 || j == res) ? 0.5 : 1.0;

                cost += pena * step * node;
                partialGradByCoeffs.block<6, 3>(i * 6, 0) += beta0 * gradPos.transpose() * step * node;
            }
        }
        minco.propogateGrad(partialGradByCoeffs, gradByPoints);

        Eigen::Vector3d pathRtd;
        Eigen::Matrix3d forward;
        double c, dc;

        for (size_t i = 0; i < hpolys.size() - 1; i ++)
        { 
            for (size_t j = 0; j < 2; j++)
            {
                pathRtd = points.col(i) - rEllips[i + j].d;
                forward = rEllips[i + j].r.cwiseInverse().asDiagonal() * rEllips[i + j].R.transpose();
                Eigen::Vector3d tempd = forward * pathRtd;
                double dist = tempd.norm() - 1.0;
                if (sfc_utils::smoothedL1(smoothEps, dist, c, dc))
                {
                    //std::cout << "c is " << c << "  dc is " << dc << std::endl;
                    double y = ys(1 - j, i + j);
                    cost += 0.5 * rho * (c + y) * (c + y);
                    //compute the gradient towards point p, rtd, and cde
                    gradByPoints.col(i) += rho * (c + y) * dc * (2.0) * forward.transpose() * tempd;
                }
            }
        }

        gradXi.resize(3 * (hpolys.size() - 1));
        for (size_t i = 0; i < hpolys.size() - 1; i++)
        {
            gradXi.segment(3 * i, 3) = gradByPoints.col(i);
        }

        return cost;

    }

    static inline double getCenterHeuristics(const Eigen::MatrixXd &ini,
                                             const Eigen::MatrixXd &fin,
                                             const PolyhedraH &hpolys,
                                             Eigen::Matrix3Xd &path)
    {
        path.resize(3, hpolys.size() + 1);
        path.leftCols<1>() = ini.col(0);
        path.rightCols<1>() = fin.col(0);

        Eigen::Matrix3Xd inners;
        inners.resize(3, hpolys.size() - 1);
        for (size_t i = 0; i < hpolys.size() - 1; i++)
        {
            Eigen::Vector3d inner;
            PolyhedronH hPoly_inter;
            hPoly_inter.resize(hpolys[i].rows() + hpolys[i + 1].rows(), 4);
            hPoly_inter << hpolys[i], hpolys[i + 1];
            if (geo_utils::findInterior(hPoly_inter, inner))
            {
                inners.col(i) = inner;
            }
            else
            {
                std::cout << "findInterior fails " << std::endl;
                return -1.0;
            }
        }

        path.block(0, 1, 3, hpolys.size() - 1) = inners;
        return 0.0;
    }

    static inline double evalTraj(const Eigen::MatrixXd &ini,
                                  const Eigen::MatrixXd &fin,
                                  const Eigen::VectorXd &times,
                                  const Eigen::Matrix3Xd &path,
                                  Trajectory<5> &traj)
    { 

        minco::MINCO_S3NU minco;
        int pieceN = path.cols() - 1;
        minco.setConditions(ini, fin, pieceN);
        minco.setParameters(path.block(0, 1, 3, pieceN), times);
        minco.getTrajectory(traj);

        //get trajectory cost
        double cost = 0.0;
        minco.getEnergy(cost);
        return cost;
    }

    static inline double evalTraj(const Trajectory<5> &traj)
    {
        return traj.getEnergy(5);
    }
}

#endif
