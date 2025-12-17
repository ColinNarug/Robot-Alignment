import numpy as np

from ur_alignment.flange_pose_estimator.utility_classes import Hole



class QuadraticForm:
    """
    Class to represent a quadratic form Q = [A B/2 D/2; B/2 C E/2; D/2 E/2 F]
    """

    def __init__(self, A: float, B: float, C: float, D: float, E: float, F: float) -> None:
        self.Q = np.array([[A,      B / 2,  D / 2],
                           [B / 2,  C,      E / 2],
                           [D / 2,  E / 2,  F    ]], dtype=float)
        # diagonalization
        self.eigenvalues = None
        self.eigenvectors = None
        # SVD (note: Q is diagonal -> Spectral Thm...)
        self.U = None
        self.Sigma = None
        self.V = None

        self.update_decompositions()


    def update_decompositions(self) -> None:
        """
		Update eigenvalues, eigenvectors, and SVD of the conic matrix.
		"""
        self.eigenvalues, self.eigenvectors = np.linalg.eigh(self.Q)
        U, Sigma, Vt = np.linalg.svd(self.Q)

        # metti controllo su rapporto tra sigma
        self.U = U
        self.Sigma = Sigma
        self.V = Vt.T


    def __mul__(self, val: float) -> 'QuadraticForm':
        """
        Scale the quadratic form by a scalar value.
        """
        self.Q *= val
        self.update_decompositions()
        return self


    def check_eigenvalue_constraints(self) -> bool:
        """
        Check if the conic has two positive and one negative eigenvalue.
        """        
        # Check eigenvalues : 2 positive and 1 negative eigenvalue.
        pos = np.sum(self.eigenvalues > 0)
        neg = np.sum(self.eigenvalues < 0)
        return pos == 2 and neg == 1


    def project(self, P: np.ndarray) -> None:
        """
		Apply the projection matrix to transform the conic.
		"""
        # In general  P: projection matrix
        #             camera_matrix: camera matrix
        #             T: pose change matrix (camera-> desired RF)
        # --> P = T @ camera_matrix
        # Compute the transformed conic matrix: Q' = P^T Q P (in general)
        self.Q = P.T @ self.Q @ P
        self.update_decompositions()





class ReconstructorProjectedCircle():

    @staticmethod
    def get_circles(ellipse: object, camera_matrix: np.ndarray, radius: float = 1.0) -> list[Hole]:
        """
        Estimation of the 3d circle given its projection on the image plane
        this process is ambiguous and generates 2 possible results
        """

        A,B,C,D,E,F = ellipse.get_coefficients()
        QF = QuadraticForm(A,B,C,D,E,F)
        QF.project(camera_matrix)

        if not QF.check_eigenvalue_constraints():
            QF = QF * (-1)
            if not QF.check_eigenvalue_constraints():
                raise ValueError("Eigenvalues do not match ellipse constraints")

        s1 = QF.Sigma[0]
        s2 = QF.Sigma[1]
        s3 = QF.Sigma[2]

        cos_phi_sq = (s2 + s3)/(s1 + s3)
        cos_phiP = np.sqrt(cos_phi_sq)
        cos_phiN = -np.sqrt(cos_phi_sq)
        sin_phiP = np.sqrt(1 - cos_phi_sq)
        sin_phiN = -np.sqrt(1 - cos_phi_sq)

        # normals
        Ns = []
        Ns.append(QF.V @ np.array([-sin_phiP, 0, cos_phiP]))
        Ns.append(QF.V @ np.array([-sin_phiN, 0, cos_phiN]))
        Ns.append(QF.V @ np.array([-sin_phiP, 0, cos_phiN]))
        Ns.append(QF.V @ np.array([-sin_phiN, 0, cos_phiP]))

        # centers
        Ps = []
        Ps.append(radius * QF.V @ np.array([np.sqrt(s3/s1)*sin_phiP, 0, np.sqrt(s1/s3)*cos_phiP]))
        Ps.append(radius * QF.V @ np.array([np.sqrt(s3/s1)*sin_phiN, 0, np.sqrt(s1/s3)*cos_phiN]))
        Ps.append(radius * QF.V @ np.array([np.sqrt(s3/s1)*sin_phiP, 0, np.sqrt(s1/s3)*cos_phiN]))
        Ps.append(radius * QF.V @ np.array([np.sqrt(s3/s1)*sin_phiN, 0, np.sqrt(s1/s3)*cos_phiP]))

        candidates = [(Ps[i], Ns[i]) for i in range(4)]

        # return the estimated circles
        holes = []
        for candidate in candidates:
            P = candidate[0]
            N = candidate[1]
            z = P[2]
            if z > 0:
                hole = Hole()
                hole.radius = radius
                hole.projection_edge = ellipse
                hole.ID = ellipse.ID
                hole["camera"].normal = N
                hole["camera"].center = P

                is_duplicate = any( np.allclose(hole["camera"].center, h["camera"].center) and
                                    np.allclose(hole["camera"].normal, h["camera"].normal)
                                    for h in holes)
                if not is_duplicate:
                    holes.append(hole)

        return holes





