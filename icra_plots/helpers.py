from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
# from tf import transformations

def find_optimal_transform(p, q):
    """
    Finds some optimal transform
    For 3D case, input is two matrices w/ shape (n, 3)

    Returns r, t s.t. `R.dot(p[0]) + t ~= q[0]`
    """

    # de-mean the points to calculate the rotation
    p_mean = np.mean(p, axis=0)
    q_mean = np.mean(q, axis=0)

    # SVD of covariance
    cov = np.dot((p - p_mean).T, (q - q_mean))
    U, S, VT = np.linalg.svd(cov, full_matrices=True)

    # negative determinant => reflection => needs fixing
    rectify = np.identity(U.shape[0])
    rectify[-1][-1] = np.linalg.det(np.dot(VT.T, U.T))

    # compute R and t
    R = VT.T.dot(rectify).dot(U.T)
    t = q_mean - np.dot(R, p_mean)

    # compute maximum error
    error = 0.0
    for pi, qi in zip(p, q):
        qi_pred = np.dot(R, pi) + t
        error = max(error, np.linalg.norm(qi - qi_pred))

    return R, t, error
    # R matrix => homogeneous transform => quaternion
    # R = np.hstack((R, np.zeros((3, 1))))
    # R = np.vstack((R, np.zeros((1, 4))))
    # R[-1, -1] = 1.0
    # q = transformations.quaternion_from_matrix(R)
    #
    # return t, q, error

# if __name__ == "__main__":
#     post = np.random.normal(size=(5, 3))
#     print post
#     #post = np.array([[0., 10., 0.],
#     #                    [5., 0., 5.],
#     #                    [0., 0., 10.],
#     #                    [10., 0., 0.]])
#
#     R = np.array([[ -0.1455000, 0.9893582, -0.0000000],\
#         [-0.4196650, -0.0617181, 0.9055784],\
#         [0.8959414, 0.1317617, 0.4241790 ]])
#     t = np.array([2, 4, 5])
#     pre = np.dot(R.T, (post - t).T).T
#
#     t, q, error = find_optimal_transform(pre, post)
#
#     print error
#     print R
#     print transformations.quaternion_matrix(q)
