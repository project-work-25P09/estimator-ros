import numpy as np
import mscl

write_settings_to_imu_flash = False

def fit_ellipsoid(data):
    # build design matrix D and vector d2
    x, y, z = data[:,0], data[:,1], data[:,2]
    D = np.vstack([
        x*x, y*y, z*z,
        2*x*y, 2*x*z, 2*y*z,
        2*x,   2*y,   2*z
    ]).T
    d2 = np.ones_like(x)
    # solve normal equations
    v, *_ = np.linalg.lstsq(D, d2, rcond=None)
    # unpack algebraic form
    A = np.array([
        [v[0], v[3], v[4], v[6]],
        [v[3], v[1], v[5], v[7]],
        [v[4], v[5], v[2], v[8]],
        [v[6], v[7], v[8], -1.0]
    ])
    # center = -A3x3⁻¹ * A3x1
    center = -np.linalg.solve(A[:3,:3], v[6:9])
    # form the translated A matrix
    T = np.eye(4)
    T[3,:3] = center
    R = T @ A @ T.T
    # extract radii and axes from R
    evals, evecs = np.linalg.eig(R[:3,:3] / -R[3,3])
    radii = np.sqrt(1.0/np.abs(evals))
    # the mapping M that normalizes the ellipsoid to a sphere
    M = evecs @ np.diag(1.0/radii) @ evecs.T
    return center, radii, M

data = np.loadtxt('data/magsweep_dump.csv', delimiter=',', skiprows=1)
center, radii, soft_iron_matrix = fit_ellipsoid(data)
hard_iron = center

print("Hard-iron bias:", hard_iron)
print("Soft-iron matrix:\n", soft_iron_matrix)

if write_settings_to_imu_flash:
    print("Connecting to IMU")
    connection = mscl.Connection.Serial("/dev/imu", 115200)
    node = mscl.InertialNode(connection)
    node.setToIdle()

    node.setMagnetometerHardIronOffset(b)
    node.setMagnetometerSoftIronMatrix(S)

    node.saveSettings()
    print("Magnetometer calibration uploaded and saved!")
else:
    print("Skipped writing to IMU flash")
