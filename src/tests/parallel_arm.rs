use std::fs::read_to_string;
use crate::kinematic_traits::{Joints, Kinematics, JOINTS_AT_ZERO};
use crate::constraints::BY_PREV;
use crate::urdf;
use std::f64::consts::FRAC_PI_2;

const TOLERANCE: f64 = 1e-6;

fn read_parallel_arm() -> crate::urdf::URDFParameters {
    let xml = read_to_string("src/tests/data/test/parallel_arm.urdf")
        .expect("Failed to read parallel_arm.urdf");
    urdf::from_urdf(xml, &None).expect("Failed to parse parallel_arm.urdf")
}

fn read_parallel_arm_rotated_j5() -> crate::urdf::URDFParameters {
    let xml = read_to_string("src/tests/data/test/parallel_arm_rotated_j5.urdf")
        .expect("Failed to read parallel_arm_rotated_j5.urdf");
    urdf::from_urdf(xml, &None).expect("Failed to parse parallel_arm_rotated_j5.urdf")
}

/// Issue 1: When all joint offsets are along Z (no lateral offsets), the library
/// incorrectly assigns joint2's Z offset to a1 (lateral parameter) and joint4's
/// Z offset to a2 (lateral parameter).
///
/// For this arm, a1 and a2 should both be 0.0 because there are no lateral offsets.
#[test]
fn test_parallel_arm_parameter_extraction() {
    let params = read_parallel_arm();

    // The library currently extracts these WRONG:
    // a1 = 0.065 (should be 0.0 — this is a Z offset, not lateral)
    // a2 = -0.24655 (should be 0.0 — this is a Z offset, not lateral)
    //
    // Correct values for a purely vertical (parallel) arm:
    assert_eq!(params.a1, 0.0, "a1 should be 0 — joint2 offset is along Z, not lateral");
    assert_eq!(params.a2, 0.0, "a2 should be 0 — joint4 offset is along Z, not lateral");

    // c1 should absorb both joint1 and joint2 Z offsets (base to shoulder)
    let expected_c1 = 0.036 + 0.065; // 0.101
    assert!((params.c1 - expected_c1).abs() < TOLERANCE,
            "c1 should be {} (joint1 + joint2 Z offsets), got {}", expected_c1, params.c1);

    // c2 is the upper arm length (joint3 Z offset) — this one is correct
    assert!((params.c2 - 0.445).abs() < TOLERANCE,
            "c2 should be 0.445, got {}", params.c2);

    // c3 should absorb both joint4 and joint5 Z offsets (forearm to wrist)
    let expected_c3 = 0.24655 + 0.1965; // 0.44305
    assert!((params.c3 - expected_c3).abs() < TOLERANCE,
            "c3 should be {} (joint4 + joint5 Z offsets), got {}", expected_c3, params.c3);

    // c4 is the TCP distance (joint6 Z offset) — this one is correct
    assert!((params.c4 - 0.1765).abs() < TOLERANCE,
            "c4 should be 0.1765, got {}", params.c4);

    assert_eq!(params.b, 0.0, "b should be 0 — no Y offsets");
}

/// Issue 1 (FK consequence): With wrong parameters, FK at zero joints gives wrong position.
/// The correct TCP position at all-zeros is straight up: (0, 0, sum_of_all_z_offsets).
#[test]
fn test_parallel_arm_fk_at_zero() {
    let params = read_parallel_arm();
    let robot = params.to_robot(BY_PREV, &JOINTS_AT_ZERO);

    let joints: Joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let pose = robot.forward(&joints);
    let pos = pose.translation;

    // All offsets are along Z, so at zero joints the TCP should be straight up
    let expected_z = 0.036 + 0.065 + 0.445 + 0.24655 + 0.1965 + 0.1765; // 1.1660500
    let expected_x = 0.0;
    let expected_y = 0.0;

    assert!((pos.x - expected_x).abs() < TOLERANCE,
            "FK x at zero should be {}, got {} (lateral offset error)", expected_x, pos.x);
    assert!((pos.y - expected_y).abs() < TOLERANCE,
            "FK y at zero should be {}, got {}", expected_y, pos.y);
    assert!((pos.z - expected_z).abs() < TOLERANCE,
            "FK z at zero should be {}, got {}", expected_z, pos.z);
}

/// Issue 1 (FK at non-zero joints): Verify FK produces correct position when joint2 = 90deg.
/// With joint2 rotated 90deg (around Y), the upper arm points horizontally.
#[test]
fn test_parallel_arm_fk_joint2_90deg() {
    let params = read_parallel_arm();
    let robot = params.to_robot(BY_PREV, &JOINTS_AT_ZERO);

    let joints: Joints = [0.0, FRAC_PI_2, 0.0, 0.0, 0.0, 0.0];
    let pose = robot.forward(&joints);
    let pos = pose.translation;

    // joint2 rotates the upper arm 90deg forward (around Y).
    // The arm from joint2 upward now goes along +X instead of +Z.
    // Base height (c1): 0.036 + 0.065 = 0.101 (this stays vertical)
    // Upper arm (c2=0.445) now goes along +X
    // Forearm (c3=0.44305) now goes along +X
    // TCP (c4=0.1765) now goes along +X
    let expected_x = 0.445 + 0.44305 + 0.1765; // 1.06455
    let expected_y = 0.0;
    let expected_z = 0.036 + 0.065; // 0.101

    assert!((pos.x - expected_x).abs() < TOLERANCE,
            "FK x with j2=90deg should be {}, got {}", expected_x, pos.x);
    assert!((pos.y - expected_y).abs() < TOLERANCE,
            "FK y with j2=90deg should be {}, got {}", expected_y, pos.y);
    assert!((pos.z - expected_z).abs() < TOLERANCE,
            "FK z with j2=90deg should be {}, got {}", expected_z, pos.z);
}

/// Issue 2: When joint5 has a rotated origin (rpy="0 -pi/2 0") with axis X instead of
/// the standard axis Z, the library should account for the rpy in the FK chain.
/// Since OPW always uses fixed axes (Y for joint5), the rpy must be composed into
/// the final orientation. Currently the library ignores rpy entirely.
///
/// At zero joints, a correct FK should produce orientation = Ry(-pi/2) relative to
/// what the standard URDF produces, because the flange rotation is baked into joint5's origin.
/// The library currently produces the same orientation for both (ignoring the rpy).
#[test]
fn test_rotated_j5_orientation_includes_rpy() {
    let rotated_params = read_parallel_arm_rotated_j5();
    assert!(rotated_params.has_origin_rpy_correction(),
            "Should detect non-zero rpy on joint5 origin");
    let rotated_robot = rotated_params.to_corrected_robot(BY_PREV, &JOINTS_AT_ZERO);

    let joints: Joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let pose = rotated_robot.forward(&joints);

    // The rotated URDF has rpy="0 -pi/2 0" on joint5's origin.
    // At zero joints, the correct end-effector orientation should include
    // this -90° pitch rotation. The rotation matrix Ry(-pi/2) is:
    //   [ 0  0  -1]
    //   [ 0  1   0]
    //   [ 1  0   0]
    //
    // The Z-column (tool Z-axis in world frame) = [-1, 0, 0],
    // meaning the tool points along world -X.
    let rot = pose.rotation.to_rotation_matrix();
    let matrix = rot.matrix();

    let tool_z_x = matrix[(0, 2)];
    let tool_z_y = matrix[(1, 2)];
    let tool_z_z = matrix[(2, 2)];

    assert!((tool_z_x - (-1.0)).abs() < TOLERANCE,
            "Tool Z-axis world X component should be -1.0 (from Ry(-pi/2)), got {}", tool_z_x);
    assert!(tool_z_y.abs() < TOLERANCE,
            "Tool Z-axis world Y component should be 0.0, got {}", tool_z_y);
    assert!(tool_z_z.abs() < TOLERANCE,
            "Tool Z-axis world Z component should be 0.0, got {}", tool_z_z);
}

/// Issue 2 (with non-zero joint6): When the flange rpy is absorbed into joint5,
/// joint6 rotation should compose with that rpy. At joint6=90° the tool should rotate
/// around the rpy-adjusted axis. The library ignores the rpy so joint6 rotates
/// around the wrong axis.
#[test]
fn test_rotated_j5_affects_joint6_orientation() {
    let rotated_params = read_parallel_arm_rotated_j5();
    let rotated_robot = rotated_params.to_corrected_robot(BY_PREV, &JOINTS_AT_ZERO);

    let joints: Joints = [0.0, 0.0, 0.0, 0.0, 0.0, FRAC_PI_2];
    let rotated_pose = rotated_robot.forward(&joints);
    let rot = rotated_pose.rotation.to_rotation_matrix();
    let matrix = rot.matrix();

    // With to_corrected_robot(), the OPW FK result is post-multiplied by Ry(-pi/2).
    // OPW FK at j6=pi/2 gives Rz(pi/2), then: Rz(pi/2) * Ry(-pi/2)
    //
    // Rz(pi/2) = [[0,-1,0],[1,0,0],[0,0,1]]
    // Ry(-pi/2) = [[0,0,-1],[0,1,0],[1,0,0]]
    // Product = Rz(pi/2) * Ry(-pi/2):
    //   = [[ 0, -1,  0],
    //      [ 0,  0, -1],
    //      [ 1,  0,  0]]
    let expected = [
        [ 0.0, -1.0,  0.0],
        [ 0.0,  0.0, -1.0],
        [ 1.0,  0.0,  0.0],
    ];

    for i in 0..3 {
        for j in 0..3 {
            assert!((matrix[(i, j)] - expected[i][j]).abs() < TOLERANCE,
                    "Rotation matrix [{},{}] should be {}, got {}. \
                     Library ignores joint5 origin rpy when computing joint6 frame.",
                    i, j, expected[i][j], matrix[(i, j)]);
        }
    }
}

/// RPY values on multiple joint origins should accumulate into origin_rpy_correction.
#[test]
fn test_rpy_accumulation_across_multiple_joints() {
    let xml = read_to_string("src/tests/data/test/multi_rpy.urdf")
        .expect("Failed to read multi_rpy.urdf");
    let params = urdf::from_urdf(xml, &None)
        .expect("Failed to parse multi-rpy URDF");

    assert!(params.has_origin_rpy_correction());
    assert!((params.origin_rpy_correction[0] - 0.1).abs() < TOLERANCE,
            "Roll should accumulate to 0.1, got {}", params.origin_rpy_correction[0]);
    assert!((params.origin_rpy_correction[1] - 0.2).abs() < TOLERANCE,
            "Pitch should accumulate to 0.2, got {}", params.origin_rpy_correction[1]);
    assert!((params.origin_rpy_correction[2] - 0.3).abs() < TOLERANCE,
            "Yaw should accumulate to 0.3, got {}", params.origin_rpy_correction[2]);
}

/// Joint 4 with two non-zero perpendicular offset components should be rejected.
#[test]
fn test_rejects_ambiguous_j4_perpendicular_offsets() {
    let xml = read_to_string("src/tests/data/test/ambiguous_j4.urdf")
        .expect("Failed to read ambiguous_j4.urdf");
    let result = urdf::from_urdf(xml, &None);
    assert!(result.is_err(),
            "Should reject joint4 with two non-zero perpendicular offset components");
}
