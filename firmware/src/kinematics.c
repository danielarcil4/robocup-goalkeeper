<<<<<<< HEAD
#include "kinematics.h"
#include <math.h>

const float alpha[3] = {ROBOT_WHEEL_0_OFFSET, ROBOT_WHEEL_1_OFFSET, ROBOT_WHEEL_2_OFFSET};  // Wheel angles in radians
=======
/**
 * @file kinematics.c
 * @brief Implementations for forward and inverse kinematics.
 *
 * Contains helpers for computing wheel target speeds and reconstructing robot velocity.
 */
#include "kinematics.h"
#include <math.h>

/// \brief Wheel orientation angles in radians.
/// \details These are fixed offsets for each wheel on the robot.
const float alpha[3] = {
    ROBOT_WHEEL_0_OFFSET,
    ROBOT_WHEEL_1_OFFSET,
    ROBOT_WHEEL_2_OFFSET
};

/// \brief Precomputed pseudo-inverse matrix A⁺ for forward kinematics.
/// \details Aplus = (Aᵀ·A)⁻¹·Aᵀ where A is the Jacobian matrix for the robot.
static float Aplus[3][3];

/// \brief Flag indicating if the Aplus matrix has been initialized.
static int Aplus_initialized = 0;
>>>>>>> Cristian

void compute_inverse_kinematics(Velocity v, WheelSpeeds *w) {
    for (int i = 0; i < 3; ++i) {
        float a = alpha[i];
        w->phi_dot[i] = (-sinf(a) * v.vx + cosf(a) * v.vy + ROBOT_BODY_RADIUS * v.wz) / ROBOT_WHEEL_RADIUS;
    }
}

<<<<<<< HEAD

// Matrix A⁺ = (Aᵀ·A)⁻¹·Aᵀ — hardcoded inverse for this geometry
// You can also calculate it dynamically with matrix ops if needed

void compute_forward_kinematics(WheelSpeeds w, Velocity *v) {
    // Build matrix A (3x3) and pseudo-invert manually:
    // A = [
    //   [-sin(α₁)/r,  cos(α₁)/r, R/r],
    //   [-sin(α₂)/r,  cos(α₂)/r, R/r],
    //   [-sin(α₃)/r,  cos(α₃)/r, R/r]
    // ]

    // We compute A⁺ * φ̇ = [vx, vy, ω]
    // This is done with hardcoded pseudo-inverse for speed (3x3 system)

    float A[3][3];
=======
/**
 * @brief Initializes the pseudo-inverse matrix Aplus used in forward kinematics.
 *
 * @details Computes the matrix A based on wheel geometry and robot constants,
 * then calculates A⁺ = (AᵗA)⁻¹ Aᵗ and caches it for later use.
 *
 * This function is only called once, on first use.
 */
void init_forward_kinematics_matrix() {
    float A[3][3];

    // Construct the Jacobian matrix A
>>>>>>> Cristian
    for (int i = 0; i < 3; ++i) {
        float a = alpha[i];
        A[i][0] = -sinf(a) / ROBOT_WHEEL_RADIUS;
        A[i][1] =  cosf(a) / ROBOT_WHEEL_RADIUS;
        A[i][2] =  ROBOT_BODY_RADIUS / ROBOT_WHEEL_RADIUS;
    }

<<<<<<< HEAD
    // Compute Aᵗ * A
=======
    // Compute AtA = Aᵗ · A
>>>>>>> Cristian
    float AtA[3][3] = {0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                AtA[i][j] += A[k][i] * A[k][j];

<<<<<<< HEAD
    // Invert AtA (3x3 matrix inversion, only done once here)
    // For simplicity, assume symmetric positive definite => use basic inverse
=======
    // Determinant of AtA
>>>>>>> Cristian
    float det = AtA[0][0]*(AtA[1][1]*AtA[2][2] - AtA[1][2]*AtA[2][1])
              - AtA[0][1]*(AtA[1][0]*AtA[2][2] - AtA[1][2]*AtA[2][0])
              + AtA[0][2]*(AtA[1][0]*AtA[2][1] - AtA[1][1]*AtA[2][0]);

<<<<<<< HEAD
    if (fabsf(det) < 1e-6f) {
        v->vx = v->vy = v->wz = 0;
        return;
    }

    float invDet = 1.0f / det;

=======
    if (fabsf(det) < 1e-6f) return; // Singular matrix

    float invDet = 1.0f / det;

    // Compute inverse(AtA)
>>>>>>> Cristian
    float invAtA[3][3];
    invAtA[0][0] =  (AtA[1][1]*AtA[2][2] - AtA[1][2]*AtA[2][1]) * invDet;
    invAtA[0][1] = -(AtA[0][1]*AtA[2][2] - AtA[0][2]*AtA[2][1]) * invDet;
    invAtA[0][2] =  (AtA[0][1]*AtA[1][2] - AtA[0][2]*AtA[1][1]) * invDet;
    invAtA[1][0] = -(AtA[1][0]*AtA[2][2] - AtA[1][2]*AtA[2][0]) * invDet;
    invAtA[1][1] =  (AtA[0][0]*AtA[2][2] - AtA[0][2]*AtA[2][0]) * invDet;
    invAtA[1][2] = -(AtA[0][0]*AtA[1][2] - AtA[0][2]*AtA[1][0]) * invDet;
    invAtA[2][0] =  (AtA[1][0]*AtA[2][1] - AtA[1][1]*AtA[2][0]) * invDet;
    invAtA[2][1] = -(AtA[0][0]*AtA[2][1] - AtA[0][1]*AtA[2][0]) * invDet;
    invAtA[2][2] =  (AtA[0][0]*AtA[1][1] - AtA[0][1]*AtA[1][0]) * invDet;

<<<<<<< HEAD
    // Compute A⁺ = (AᵗA)⁻¹ Aᵗ and multiply with w.phi_dot
    float At[3][3];
    for (int i = 0; i < 3; ++i)      // transpose A
        for (int j = 0; j < 3; ++j)
            At[i][j] = A[j][i];

    float Aplus[3][3] = {0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                Aplus[i][j] += invAtA[i][k] * At[k][j];

    // Compute v = A⁺ * φ̇
=======
    // Transpose A into At
    float At[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            At[i][j] = A[j][i];

    // Compute Aplus = inv(AtA) · At
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            Aplus[i][j] = 0;
            for (int k = 0; k < 3; ++k)
                Aplus[i][j] += invAtA[i][k] * At[k][j];
        }

    Aplus_initialized = 1;
}


void compute_forward_kinematics(WheelSpeeds w, Velocity *v) {
    if (!Aplus_initialized) {
        init_forward_kinematics_matrix();
        if (!Aplus_initialized) {
            v->vx = v->vy = v->wz = 0;
            return;
        }
    }

>>>>>>> Cristian
    v->vx = v->vy = v->wz = 0;
    for (int i = 0; i < 3; ++i) {
        v->vx += Aplus[0][i] * w.phi_dot[i];
        v->vy += Aplus[1][i] * w.phi_dot[i];
        v->wz += Aplus[2][i] * w.phi_dot[i];
    }
}
