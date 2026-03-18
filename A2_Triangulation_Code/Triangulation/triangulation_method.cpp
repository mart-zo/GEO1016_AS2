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

    /// Below are a few examples showing some useful data structures and APIs.

    /// define a 2D vector/point
    // Vector2D b(1.1, 2.2);

    /// define a 3D vector/point
    // Vector3D a(1.1, 2.2, 3.3);

    /// get the Cartesian coordinates of a (a is treated as Homogeneous coordinates)
    // Vector2D p = a.cartesian();

    /// get the Homogeneous coordinates of p
    // Vector3D q = p.homogeneous();

    /// define a 3 by 3 matrix (and all elements initialized to 0.0)
    // Matrix33 A;

    /// define and initialize a 3 by 3 matrix
    // Matrix33 T(1.1, 2.2, 3.3,
               // 0, 2.2, 3.3,
               // 0, 0, 1);

    /// define and initialize a 3 by 4 matrix
    // Matrix34 M(1.1, 2.2, 3.3, 0,
               // 0, 2.2, 3.3, 1,
               // 0, 0, 1, 1);

    /// set first row by a vector
    // M.set_row(0, Vector4D(1.1, 2.2, 3.3, 4.4));

    // set second column by a vector
    // M.set_column(1, Vector3D(5.5, 5.5, 5.5));

    /// define a 15 by 9 matrix (and all elements initialized to 0.0)
    // Matrix Y(15, 9, 0.0);
    /// set the first row by a 9-dimensional vector
    // Y.set_row(0, {0, 1, 2, 3, 4, 5, 6, 7, 8}); // {....} is equivalent to a std::vector<double>

    /// get the number of rows.
    // int num_rows = Y.rows();

    // get the number of columns.
    // int num_cols = Y.cols();

    /// get the the element at row 1 and column 2
    // double value = Y(1, 2);

    /// get the last column of a matrix
    // Vector last_column = Y.get_column(Y.cols() - 1);

    /// define a 3 by 3 identity matrix
    // Matrix33 I = Matrix::identity(3, 3, 1.0);

    /// matrix-vector product
    // Vector3D v = M * Vector4D(1, 2, 3, 4); // M is 3 by 4


    //Here I just added svd from last assignment :)))
    // Compute the SVD decomposition of A
    //svd_decompose(A, U, S, V);

    // Now let's check if the SVD result is correct

    // Check 1: U is orthogonal, so U * U^T must be identity
    //std::cout << "U*U^T: \n" << U * transpose(U) << std::endl;

    // Check 2: V is orthogonal, so V * V^T must be identity
    //std::cout << "V*V^T: \n" << V * transpose(V) << std::endl;

    // Check 3: S must be a diagonal matrix
    //std::cout << "S: \n" << S << std::endl;

    // Check 4: according to the definition, A = U * S * V^T
    //std::cout << "M - U * S * V^T: \n" << A - U * S * transpose(V) << std::endl;

    ///For more functions of Matrix and Vector, please refer to 'matrix.h' and 'vector.h'

    // TODO: delete all above example code in your final submission

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

    // Finding the 4 candidate relative poses.
    // Recover possible t values (by getting the last column of matrix U).
    Vector t_pos = U.get_column(U.cols() - 1);
    Vector t_neg = -t_pos;

    // Recover possible R values.
    Matrix33 R_1 = determinant(U * W * transpose(V)) * U * W * transpose(V);
    Matrix33 R_2 = determinant(U * transpose(W) * transpose(V)) * U * transpose(W) * transpose(V);

    // Triangulate image points to find correct relative pose.
    // The 4 candidate pairs.
    Matrix33 K_inv = inverse(K);

    // Create vector of possible R & t pairs.
    std::vector<std::pair<Matrix33, Vector3D>> candidates = {
        {R_1, t_pos}, {R_1, t_neg},
        {R_2, t_pos}, {R_2, t_neg}
    };

    // Keep track of best pair (start with lowest possible count).
    int best_count = -1;
    Matrix33 best_R;
    Vector3D best_t;

    // Loop through all pairs.
    for (int k = 0; k < candidates.size(); k++) {
        Matrix R_cand = candidates[k].first;
        Vector3D t_cand = candidates[k].second;
        int count = 0;

        // Camera 2 center in world frame: O2 = -R^T * t.
        Vector3D O1(0, 0, 0);
        Vector3D O2 = -(transpose(R_cand) * t_cand);

        // Loop through image points to test the pairs.
        for (int i = 0; i < points_0.size(); i++) {
            // Ray directions (unproject pixels using K_inv to go from image points to real world coordinates).
            Vector3D d1 = K_inv * points_0[i].homogeneous();
            
            // Convert the image point to a direction vector and rotate it to match the world coordinate system
            Vector temp_d2 = K_inv * points_1[i].homogeneous();
            Vector rotated_d2 = transpose(R_cand) * temp_d2;
            Vector3D d2(rotated_d2[0], rotated_d2[1], rotated_d2[2]);

            // Find intersection of two lines:
            // O1 + s1*d1 = O2 + s2*d2 (l1 = l2).
            // This gives: s1*d1 - s2*d2 = O2 - O1.
            // Solve [d1 | -d2] * [s1, s2]^T = O2 - O1  (3x2 system, least squares).
            Vector3D w = O2 - O1;

            // Least square solution to s1 and s2.
            double a = dot(d1, d1);
            double b = dot(d1, d2);
            double c = dot(d2, d2);
            double d = dot(d1, w);
            double e = dot(d2, w);

            double denom = a*c - b*b;
            if (std::abs(denom) < 1e-10) continue;  // parallel rays, skip

            double s1      = (b*e - c*d) / denom;
            double s2 = (a*e - b*d) / denom;

            // 3D point P = midpoint of closest approach.
            Vector3D P = ((O1 + s1 * d1) + (O2 + s2 * d2)) / 2.0;

            // In front of camera 1: P.z > 0.
            // In front of camera 2: (R*P + t).z > 0.
            Vector3D P_cam2 = R_cand * P + t_cand;

            if (P.z() > 0 && P_cam2.z() > 0) {
                count++;
            }
        }

        if (count > best_count) {
            best_count = count;
            best_R = R_cand;
            best_t = t_cand;
        }
    }

    // best_R and best_t are now the correct relative pose.
    R = best_R;
    t = best_t;


    // TODO: Reconstruct 3D points. The main task is
    //      - triangulate a pair of image points (i.e., compute the 3D coordinates for each corresponding point pair) -HASSAN

    // TODO: Don't forget to
    //          - write your recovered 3D points into 'points_3d' (so the viewer can visualize the 3D points for you);
    //          - write the recovered relative pose into R and t (the view will be updated as seen from the 2nd camera,
    //            which can help you check if R and t are correct).
    //       You must return either 'true' or 'false' to indicate whether the triangulation was successful (so the
    //       viewer will be notified to visualize the 3D points and update the view).
    //       There are a few cases you should return 'false' instead, for example:
    //          - function not implemented yet;
    //          - input not valid (e.g., not enough points, point numbers don't match);
    //          - encountered failure in any step.

    // STEP 3: triangulation
    // clear any previous 3D points so we start fresh for this reconstruction
    points_3d.clear();

    // build projection matrix for camera 1 (we assume this camera is at the origin)
    Matrix34 P0;
    P0(0,0) = fx;   P0(0,1) = s;    P0(0,2) = cx;   P0(0,3) = 0.0;
    P0(1,0) = 0.0;  P0(1,1) = fy;   P0(1,2) = cy;   P0(1,3) = 0.0;
    P0(2,0) = 0.0;  P0(2,1) = 0.0;  P0(2,2) = 1.0;  P0(2,3) = 0.0;

    // build projection matrix for camera 2 using the R and t we found before
    // this represents how the second camera is positioned relative to the first one
    Matrix34 Rt;
    Rt(0,0) = R(0,0);
    Rt(0,1) = R(0,1);
    Rt(0,2) = R(0,2);
    Rt(0,3) = t.x();

    Rt(1,0) = R(1,0);
    Rt(1,1) = R(1,1);
    Rt(1,2) = R(1,2);
    Rt(1,3) = t.y();

    Rt(2,0) = R(2,0);
    Rt(2,1) = R(2,1);
    Rt(2,2) = R(2,2);
    Rt(2,3) = t.z();

    // multiply with K to get the full projection matrix for camera 2
    Matrix34 P1 = K * Rt;

    // we now loop over all corresponding image points and triangulate them one by one
    // each pair of points should give us one 3D point
    for (int i = 0; i < points_0.size(); i++) {
        double x0 = points_0[i].x();
        double y0 = points_0[i].y();
        double x1 = points_1[i].x();
        double y1 = points_1[i].y();

        // here we build matrix A such that A * X = 0, where X is the 3D point
        // this comes from combining the projection equations of both cameras
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

        // used SVD to solve for X
        // the result is in the last column of V
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

        // convert to normal 3D coordinates by dividing by X[3]
        double Xc = X[0] / X[3];
        double Yc = X[1] / X[3];
        double Zc = X[2] / X[3];

        // store the 3D point so we can visualize it later
        Vector3D point_3d(Xc, Yc, Zc);
        points_3d.push_back(point_3d);
    }

    // we return true if we managed to reconstruct at least one point
    return !points_3d.empty();
}