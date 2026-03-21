/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "triangulation.h"
#include "matrix_algo.h"
#include <easy3d/optimizer/optimizer_lm.h>


using namespace easy3d;


/**
 * TODO: Finish this function for reconstructing 3D geometry from corresponding image points.
 * @return True on success, otherwise false. On success, the reconstructed 3D points must be written to 'points_3d'
 *      and the recovered relative pose must be written to R and t.
 */
bool Triangulation::triangulation(
        double fx, double fy,     /// input: the focal lengths (same for both cameras)
        double cx, double cy,     /// input: the principal point (same for both cameras)
        double s,                 /// input: the skew factor (same for both cameras)
        const std::vector<Vector2D> &points_0,  /// input: 2D image points in the 1st image.
        const std::vector<Vector2D> &points_1,  /// input: 2D image points in the 2nd image.
        std::vector<Vector3D> &points_3d,       /// output: reconstructed 3D points
        Matrix33 &R,   /// output: 3 by 3 matrix, which is the recovered rotation of the 2nd camera
        Vector3D &t    /// output: 3D vector, which is the recovered translation of the 2nd camera
) const
{
    /// NOTE: there might be multiple workflows for reconstructing 3D geometry from corresponding image points.
    ///       This assignment uses the commonly used one explained in our lecture.
    ///       It is advised to define a function for the sub-tasks. This way you have a clean and well-structured
    ///       implementation, which also makes testing and debugging easier. You can put your other functions above
    ///       'triangulation()'.

    std::cout << "\nTODO: implement the 'triangulation()' function in the file 'Triangulation/triangulation_method.cpp'\n\n";

    std::cout << "[Liangliang]:\n"
                 "\tSimilar to the first assignment, basic linear algebra data structures and functions are provided in\n"
                 "\tthe following files:\n"
                 "\t    - Triangulation/matrix.h: handles matrices of arbitrary dimensions and related functions.\n"
                 "\t    - Triangulation/vector.h: manages vectors of arbitrary sizes and related functions.\n"
                 "\t    - Triangulation/matrix_algo.h: contains functions for determinant, inverse, SVD, linear least-squares...\n"
                 "\tFor more details about these data structures and a complete list of related functions, please\n"
                 "\trefer to the header files mentioned above.\n\n"
                 "\tIf you choose to implement the non-linear method for triangulation (optional task). Please\n"
                 "\trefer to 'Tutorial_NonlinearLeastSquares/main.cpp' for an example and some explanations.\n\n"
                 "\tFor your final submission, adhere to the following guidelines:\n"
                 "\t    - submit ONLY the 'Triangulation/triangulation_method.cpp' file.\n"
                 "\t    - remove ALL unrelated test code, debugging code, and comments.\n"
                 "\t    - ensure that your code compiles and can reproduce your results WITHOUT ANY modification.\n\n" << std::flush;

    //--------------------------------------------------------------------------------------------------------------
    // implementation starts ...

    //checking if the input is valid (always good because you never know how others will call your function). - MARTA
    //number of correspondences >= 8, no. of both images' points must match
    if (points_0.size() < 8 || points_0.size() != points_1.size()) {
        return false;
    }

    // STEP 1.
    // Normalization.
    // Initializing Translation matricies.
    Matrix T_1(3, 3, 0.0);
    Matrix T_2(3, 3, 0.0);

    // Extract u, v and compute centroid for image 1.
    double u1_bar = 0.0, v1_bar = 0.0;
    int N = points_0.size();

    for (int i = 0; i < N; i++) {
        u1_bar += points_0[i][0];  // u coordinate
        v1_bar += points_0[i][1];  // v coordinate
    }
    u1_bar /= N;
    v1_bar /= N;

    // Mean distance for image 1.
    double dist = 0.0;
    for (int i = 0; i < N; i++) {
        double du = points_0[i].x() - u1_bar;
        double dv = points_0[i].y() - v1_bar;
        dist += std::sqrt(du*du + dv*dv);
    }
    dist /= N;

    double sc1 = std::sqrt(2.0) / dist;
    // Filling T1.
    T_1(0,0) = sc1;    T_1(0,1) = 0;    T_1(0,2) = -sc1 * u1_bar;
    T_1(1,0) = 0;    T_1(1,1) = sc1;    T_1(1,2) = -sc1 * v1_bar;
    T_1(2,0) = 0;    T_1(2,1) = 0;    T_1(2,2) = 1.0;

    // Extract u, v and compute centroid for image 2.
    double u2_bar = 0.0, v2_bar = 0.0;
    int N2 = points_1.size();

    for (int i = 0; i < N2; i++) {
        u2_bar += points_1[i][0];  // u coordinate
        v2_bar += points_1[i][1];  // v coordinate
    }
    u2_bar /= N2;
    v2_bar /= N2;

    // Mean distance for image 2.
    double dist2 = 0.0;
    for (int i = 0; i < N2; i++) {
        double du2 = points_1[i].x() - u2_bar;
        double dv2 = points_1[i].y() - v2_bar;
        dist2 += std::sqrt(du2*du2 + dv2*dv2);
    }
    dist2 /= N2;

    double sc2 = std::sqrt(2.0) / dist2;
    // Filling T2.
    T_2(0,0) = sc2;    T_2(0,1) = 0;    T_2(0,2) = -sc2 * u2_bar;
    T_2(1,0) = 0;    T_2(1,1) = sc2;    T_2(1,2) = -sc2 * v2_bar;
    T_2(2,0) = 0;    T_2(2,1) = 0;    T_2(2,2) = 1.0;

    // Normalizing the points with T matricies.
    // Normalized points for image 1.
    std::vector<Vector3D> q_0;
    for (int i = 0; i < N; i++) {
        Vector3D q = T_1 * points_0[i].homogeneous();  // we add 1 to the points so now we have (u,v, 1) then multiply by T
        q_0.push_back(q);
    }

    // Normalized points for image 2.
    std::vector<Vector3D> q_1;
    for (int i = 0; i < N; i++) {
        Vector3D q = T_2 * points_1[i].homogeneous();
        q_1.push_back(q);
    }
    // Linear solution (based on SVD).
    // Initializing W matrix with Os.
    Matrix W1(N, 9, 0.0);
    // Now we add values from q_0 and q_1 to matrix W.
    for (int i = 0; i < N; i++) {
        double u  = q_0[i][0];
        double v  = q_0[i][1];
        double u_ = q_1[i][0];
        double v_ = q_1[i][1];

        W1.set_row(i, {u*u_, v*u_, u_, u*v_, v*v_, v_, u, v, 1.0});
    }

    // SVD for W to get f (Wf=0).
    Matrix U1(N, N, 0.0);
    Matrix S1(N, 9, 0.0);
    Matrix V1(9, 9, 0.0);

    // W = U * S * V^T.
    svd_decompose(W1, U1, S1, V1);

    // Last column of V gives f.
    Vector f = V1.get_column(8);

    // Fundamental matrix from f (before constraint enforcement).
    Matrix33 F_initial(f[0], f[1], f[2],
               f[3], f[4], f[5],
               f[6], f[7], f[8]);

    // Constraint enforcement.
    // SVD of F_initial.
    Matrix U_initial(3, 3, 0.0);
    Matrix S_initial(3, 3, 0.0);
    Matrix V_initial(3, 3, 0.0);

    svd_decompose(F_initial, U_initial, S_initial, V_initial);
    // Setting d3 to 0.
    S_initial(2, 2) = 0.0;

    // Recomposing with constraint applied.
    Matrix33 F_q = U_initial * S_initial * transpose(V_initial);

    // Denormalization F = T_1FT_2.
    Matrix33 F = transpose(T_2) * F_q * T_1;

    // STEP 2.
    // Setting up intrinsic camera matrix K.
    Matrix33 K(fx, s, cx,
               0, fy, cy,
               0, 0, 1);

    // Computing the essential matrix E.
    Matrix33 E = transpose(K) * F * K;

    // Matrices W and Z, for later use in decomposition.
    Matrix33 W(0, -1, 0,
               1, 0, 0,
               0, 0, 1);

    Matrix33 Z(0, 1, 0,
               -1, 0, 0,
               0, 0, 0);

    // Define U, D and V, used for SVD decomposition.
    Matrix33 U = Matrix::identity(3, 3, 1.0);
    Matrix33 D = Matrix::identity(3, 3, 1.0);
    Matrix33 V = Matrix::identity(3, 3, 1.0);

    // Compute the SVD decomposition of E.
    svd_decompose(E, U, D, V);

    // making sure U has a positive determinant
    if (determinant(U) < 0) {
        U = -U;
    }
    // making sure V has a positive determinant
    if (determinant(V) < 0) {
        V = -V;
    }

    D(0,0) = 1.0;
    D(1,1) = 1.0;
    D(2,2) = 0.0;

    // Finding the 4 candidate relative poses.
    // Recover possible t values (by getting the last column of matrix U).
    Vector t_pos = U.get_column(U.cols() - 1);
    Vector t_neg = -t_pos;

    // Recover possible R values.
    Matrix33 R_1 = determinant(U * W * transpose(V)) * U * W * transpose(V);
    Matrix33 R_2 = determinant(U * transpose(W) * transpose(V)) * U * transpose(W) * transpose(V);

    // Triangulate image points to find correct relative pose.
    // Create vector of possible R & t pairs.
    std::vector<std::pair<Matrix33, Vector3D>> candidates = {
        {R_1, t_pos}, {R_1, t_neg},
        {R_2, t_pos}, {R_2, t_neg}
    };

    // Keep track of best pair (start with lowest possible count).
    int best_count = -1;
    Matrix33 best_R;
    Vector3D best_t;

    // Loop through all pairs to find the correct one (where most 3D points lie in front of
    // both cameras (positive z in both camera coordinate systems).
    for (int k = 0; k < candidates.size(); k++) {
        Matrix33 R_cand = candidates[k].first;
        Vector3D t_cand = candidates[k].second;
        int count = 0;

        // Build the [R|t] matrix for camera 2 using this candidate pose.
        Matrix34 Rt_cand;
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) Rt_cand(r,c) = R_cand(r,c);
        }
        Rt_cand(0,3) = t_cand.x();
        Rt_cand(1,3) = t_cand.y();
        Rt_cand(2,3) = t_cand.z();

        // Camera 1 projection matrix: K * [I | 0] (camera 1 is at the world origin).
        Matrix34 P0_cand;
        P0_cand(0,0)=fx; P0_cand(0,1)=s;  P0_cand(0,2)=cx; P0_cand(0,3)=0;
        P0_cand(1,0)=0;  P0_cand(1,1)=fy; P0_cand(1,2)=cy; P0_cand(1,3)=0;
        P0_cand(2,0)=0;  P0_cand(2,1)=0;  P0_cand(2,2)=1;  P0_cand(2,3)=0;

        // Camera 2 projection matrix: K * [R | t].
        Matrix34 P1_cand = K * Rt_cand;

        // Triangulate each point pair using the linear (DLT) method
        // and check if the resulting 3D point is in front of both cameras.
        for (int i = 0; i < (int)points_0.size(); i++) {
            double x0 = points_0[i].x(), y0 = points_0[i].y();
            double x1 = points_1[i].x(), y1 = points_1[i].y();

            // Build the 4x4 matrix A such that A * X = 0, where X is the 3D point
            // in homogeneous coordinates. Each image point contributes 2 rows.
            Matrix A(4, 4, 0.0);
            A.set_row(0, x0 * P0_cand.get_row(2) - P0_cand.get_row(0));
            A.set_row(1, y0 * P0_cand.get_row(2) - P0_cand.get_row(1));
            A.set_row(2, x1 * P1_cand.get_row(2) - P1_cand.get_row(0));
            A.set_row(3, y1 * P1_cand.get_row(2) - P1_cand.get_row(1));

            // Solve A * X = 0 using SVD. The solution is the last column of V.
            Matrix Ux(4,4,0.0), Sx(4,4,0.0), Vx(4,4,0.0);
            svd_decompose(A, Ux, Sx, Vx);
            Vector X = Vx.get_column(3);

            // Skip degenerate solutions where the homogeneous coordinate is near zero.
            if (std::abs(X[3]) < 1e-10) continue;
            if (X[3] < 0) X = -1.0 * X;

            // The SVD solution is defined up to scale, so normalize the sign of
            // the homogeneous coordinate before converting to Euclidean coordinates.
            double Xc = X[0]/X[3], Yc = X[1]/X[3], Zc = X[2]/X[3];

            // Transform the point into camera 2's coordinate system to check depth there.
            Vector3D P_cam2 = R_cand * Vector3D(Xc, Yc, Zc) + t_cand;

            // Count the point if it lies in front of both cameras (positive z depth).
            if (Zc > 0 && P_cam2.z() > 0) count++;
        }

        // Keep track of the candidate pair that puts the most points in front of both cameras.
        if (count > best_count) {
            best_count = count;
            best_R = R_cand;
            best_t = t_cand;
        }
    }

    // keep a copy of the recovered pose for triangulation
    Matrix33 R_used = best_R;
    Vector3D t_used = best_t;

    //
    R = best_R;
    t = best_t;

    // STEP 3: triangulation.
    // clear any previous 3D points so we start fresh for this reconstruction.
    points_3d.clear();

    // build projection matrix for camera 1 (we assume this camera is at the origin)
    Matrix34 P0;
    P0(0,0) = fx;   P0(0,1) = s;    P0(0,2) = cx;   P0(0,3) = 0.0;
    P0(1,0) = 0.0;  P0(1,1) = fy;   P0(1,2) = cy;   P0(1,3) = 0.0;
    P0(2,0) = 0.0;  P0(2,1) = 0.0;  P0(2,2) = 1.0;  P0(2,3) = 0.0;

    // build projection matrix for camera 2 using the R and t we found before. This represents how the second camera is positioned relative to the first one
    Matrix34 Rt;
    Rt(0,0) = R_used(0,0);
    Rt(0,1) = R_used(0,1);
    Rt(0,2) = R_used(0,2);
    Rt(0,3) = t_used.x();

    Rt(1,0) = R_used(1,0);
    Rt(1,1) = R_used(1,1);
    Rt(1,2) = R_used(1,2);
    Rt(1,3) = t_used.y();

    Rt(2,0) = R_used(2,0);
    Rt(2,1) = R_used(2,1);
    Rt(2,2) = R_used(2,2);
    Rt(2,3) = t_used.z();

    // multiply with K to get the full projection matrix for camera 2
    Matrix34 P1 = K * Rt;

    // we now loop over all corresponding image points and triangulate them one by one. Each pair of points should give us one 3D point
    for (int i = 0; i < (int)points_0.size(); i++) {
        double x0 = points_0[i].x();
        double y0 = points_0[i].y();
        double x1 = points_1[i].x();
        double y1 = points_1[i].y();

        // here we build matrix A such that A * X = 0, where X is the 3D point. This comes from combining the projection equations of both cameras
        Matrix A(4, 4, 0.0);

        // compute each row separately so it is easier to see what is going on
        Vector row1 = x0 * P0.get_row(2) - P0.get_row(0);
        Vector row2 = y0 * P0.get_row(2) - P0.get_row(1);
        Vector row3 = x1 * P1.get_row(2) - P1.get_row(0);
        Vector row4 = y1 * P1.get_row(2) - P1.get_row(1);

        A.set_row(0, row1);
        A.set_row(1, row2);
        A.set_row(2, row3);
        A.set_row(3, row4);

        // used SVD to solve for X. the result is in the last column of V
        Matrix Ux(4, 4, 0.0);
        Matrix Sx(4, 4, 0.0);
        Matrix Vx(4, 4, 0.0);
        svd_decompose(A, Ux, Sx, Vx);

        // take the last column of V as the solution in homogeneous coordinates
        Vector X = Vx.get_column(3);

        // check if the homogeneous coordinate is valid before dividing
        if (std::abs(X[3]) < 1e-10) {
            continue;
        }

        // convert from homogeneous to normal 3D coordinates
        double Xc = X[0] / X[3];
        double Yc = X[1] / X[3];
        double Zc = X[2] / X[3];

        // make sure the point lies in front of the camera otherwise flip the sign to move it to the correct side
        if (X[3] < 0) {
            Xc = -Xc; Yc = -Yc; Zc = -Zc;
        }
        // Then skip genuinely invalid points:
        if (Zc < 0) continue;

        // store the 3D point so we can visualize it later
        Vector3D point_3d;
        point_3d = Vector3D(Xc, Yc, Zc);
        points_3d.push_back(point_3d);
    }



// STEP 4: Evaluation using reprojection error (RMSE)
    double total_error = 0.0;
    int count_eval = 0;

    for (int i = 0; i < (int)points_3d.size(); i++) {
        Vector4D X(points_3d[i].x(), points_3d[i].y(), points_3d[i].z(), 1.0);

        Vector3D proj0 = P0 * X;
        double u0_proj = proj0[0] / proj0[2];
        double v0_proj = proj0[1] / proj0[2];

        Vector3D proj1 = P1 * X;
        double u1_proj = proj1[0] / proj1[2];
        double v1_proj = proj1[1] / proj1[2];

        // squared errors (not square rooted yet)
        double err0 = (u0_proj - points_0[i].x())*(u0_proj - points_0[i].x()) +
                      (v0_proj - points_0[i].y())*(v0_proj - points_0[i].y());
        double err1 = (u1_proj - points_1[i].x())*(u1_proj - points_1[i].x()) +
                      (v1_proj - points_1[i].y())*(v1_proj - points_1[i].y());

        total_error += err0 + err1;
        count_eval++;
    }


        double rms = std::sqrt(total_error / (2.0 * count_eval));
        std::cout << "RMS reprojection error: " << rms << " pixels" << std::endl << std::flush;

    // return true if at least one 3D point was reconstructed
    return !points_3d.empty();
}