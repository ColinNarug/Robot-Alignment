import numpy as np
import matplotlib.pyplot as plt
import cv2



class Ellipse:

    __slots__ = ( 'ID', 'cx', 'cy', 'angle', 'a', 'b',
                  'A', 'B', 'C', 'D', 'E', 'F',
                  'area', 'e', 'epsilon',
                  'degenerate_points_fitting'
    )

    def __init__(self) -> None:

        self.ID = None # ellipse ID
        # -----------------------------------------
        # ellipse: center-angle-axis parameters
        # -----------------------------------------
        # ellipse center 
        self.cx = None 
        self.cy = None
        # anti-clock-wise rotation angle
        self.angle = None # [rad]
        # semi-axis 
        self.a = None # coincident with x-axis if theta = 0
        self.b = None # coincident with y-axis if theta = 0
        # ------------------------------------------------------
        # coefficients: Ax**2 + Bxy + Cy**2 + Dx + Ey + F = 0
        # ------------------------------------------------------
        self.A = None
        self.B = None
        self.C = None
        self.D = None
        self.E = None
        self.F = None
        # -----------------------------------------------------
        # ellipse features
        # -----------------------------------------------------
        self.area = None
        self.e = None        # eccentricity
        self.epsilon = 1e-9 # upper limit for delta = B**2 - 4AC
        # -----------------------------------------------------
        # flags
        # -----------------------------------------------------
        self.degenerate_points_fitting = False # flag for correct fitting


    def __repr__(self) -> str:
        if self.check_if_parameters_available():
            return f"Ellipse ID = {self.ID}  --->  cx = {self.cx:.3}, cy = {self.cy:.3}, angle = {self.angle:.3}, a = {self.a:.3}, b = {self.b:.3}\n"
        
        return "Ellipse parameters not available"
    


    def print_coefficients(self) -> None:
        """
        Print the ellipse coefficients if available.
        """
        if self.check_if_coefficients_available():
            print(f"A={self.A}, B={self.B}, C={self.C}, D={self.D}, E={self.E}, F={self.F}")
        else: print("Ellipse coefficients not available")


    # --------------------------------------------------------
    # get/set ellipses stuff
    # --------------------------------------------------------

    # --------------- ellipse parameters --------------- #
    def set_parameters(self, cx: float, cy: float, angle: float, a: float, b: float) -> None:
        """
        Set the ellipse parameters: center, angle, and axes.
        """        
        # ellipse center 
        self.cx = cx 
        self.cy = cy
        # ellipse orientation angle
        self.angle = angle
        self.normalize_angle()
        # semi-axis 
        self.a = a
        self.b = b


    def get_parameters(self) -> tuple[float, float, float, float, float]:
        """
        Return ellipse parameters (cx, cy, angle, a, b), computing them from coefficients if necessary.
        """
        if self.check_if_parameters_available():
            # already calculated
            return self.cx, self.cy, self.angle, self.a, self.b 

        self.find_center_from_coefficients()
        self.find_angle_from_coefficients()
        self.find_axis_from_coefficients()
        return self.cx, self.cy, self.angle, self.a, self.b


    # --------------------------------------------- ellipse coefficients --------------------------------------------- #
    def set_coefficients(self, A: float, B: float, C: float, D: float, E: float, F: float) -> None:
        """
        Set the ellipse coefficients and normalize them.
        """   
        self.A = float(A) 
        self.B = float(B)
        self.C = float(C)
        self.D = float(D)
        self.E = float(E)
        self.F = float(F)
        self.normalize_coefficients()



    def get_coefficients(self) -> tuple[float, float, float, float, float, float]:
        """
        Return ellipse coefficients (A, B, C, D, E, F), computing them from parameters if necessary.
        """        
        if self.check_if_coefficients_available():
            # already calculated
            return self.A, self.B, self.C, self.D, self.E, self.F
        
        self.find_coefficients_from_parameters()
        return self.A, self.B, self.C, self.D, self.E, self.F
        

    # ------------------------------ other ellipse features ------------------------------ #
    def get_area(self) -> float:
        """
        Return the ellipse area, calculating it from parameters or coefficients.
        """
        if self.area is not None:
            return self.area
        if self.a is not None and self.b is not None:
            self.area = np.pi*self.a*self.b
            return self.area
        elif(self.A is not None and 
             self.B is not None and 
             self.C is not None):   
            self.area = 2*np.pi/(np.sqrt(4*self.A*self.C - self.B**2))
        else:
            raise RuntimeError("Cannot calculate area without axis parameters or coefficients.")



    def get_eccentricity(self) -> float:
        """
        Calculate and return the ellipse eccentricity.
        """
        if self.e is not None:
            return self.e
        if self.a is not None and self.b is not None:
            max_axis = max(self.b, self.a)
            min_axis = min(self.b, self.a)
            self.e = np.sqrt(1 - (min_axis/max_axis)**2)
            return self.e
        else:
            raise RuntimeError("Cannot calculate eccentricity without axis parameters.")
        


    def get_circumference(self) -> float:
        """
        Calculate the ellipse circumference using the Ramanujan's approximation.
        """
        if self.a is not None and self.b is not None:
            a, b = self.a, self.b
            h = ((a - b)**2) / ((a + b)**2)
            circumference = np.pi * (a + b) * (1 + (3 * h) / (10 + np.sqrt(4 - 3 * h)))
            return circumference
        else:
            raise RuntimeError("Cannot calculate circumference without semi-axis parameters (a, b).")
        


    def calculate_occupied_pixels(self, image_size: tuple[int, int]) -> int:
        """
        Calculate the number of pixels occupied by the ellipse in an image of given size.
        """
        if not self.check_if_parameters_available():
            raise RuntimeError("Ellipse parameters are not set. Cannot calculate pixel occupancy.")
        
        # Parameter convertion to integer
        center = (int(round(self.cx)), int(round(self.cy)))  
        axes = (int(round(self.a)), int(round(self.b)))      
        angle = np.degrees(self.angle)    

        height, width = image_size
        mask = np.zeros((height, width), dtype=np.uint8) 
        cv2.ellipse(mask, center, axes, angle, 0, 360, 255, 1)

        # Conta i pixel occupati dall'ellisse
        ellipse_pixel_count = cv2.countNonZero(mask)

        return ellipse_pixel_count


    # -----------------------------------------------------------------
    # representation change: coefficients --> parameters
    # -----------------------------------------------------------------
    def find_center_from_coefficients(self) -> None:
        """
        Compute the center (cx, cy) from the conic coefficients.
        """

        if self.check_if_coefficients_available():

            self.cx = (2 * self.C * self.D - self.B * self.E) / (self.B**2 - 4 * self.A * self.C)
            self.cy = (2 * self.A * self.E - self.B * self.D) / (self.B**2 - 4 * self.A * self.C)
        else:
            raise ValueError("Tried to find ellipse center, but the the ellipse coefficient are not available.")


    def find_axis_from_coefficients(self) -> None:
        """
        Compute semi-axes (a, b) from the conic coefficients.
        """
        if self.check_if_coefficients_available():

            delta = self.B**2 - 4 * self.A * self.C
            common_root = np.sqrt((self.A - self.C)**2 + self.B**2)
            common_num = 2 * (self.A * self.E**2 + self.C * self.D**2 - self.B * self.D * self.E + delta * self.F)
            self.a = -np.sqrt(common_num * ((self.A + self.C) + common_root)) / delta
            self.b = -np.sqrt(common_num * ((self.A + self.C) - common_root)) / delta
        else:
            raise ValueError("Tried to find ellipse axis, but the the ellipse coefficient are not available.")


    def find_angle_from_coefficients(self) -> None:
        """
        Compute the rotation angle from the conic coefficients.
        """
        if self.check_if_coefficients_available():
            self.angle = 0.5 * np.arctan2(-self.B, self.C - self.A)
            self.normalize_angle()
        else:
            raise ValueError("Tried to find ellipse angle, but the the ellipse coefficient are not available.")


    # -----------------------------------------------------------------
    # representation change: parameters --> coefficients 
    # -----------------------------------------------------------------
    def find_coefficients_from_parameters(self) -> None:
        """
        Compute conic coefficients from ellipse parameters.
        """
        if self.check_if_parameters_available():
            
            self.A = float(self.a**2 * np.sin(self.angle)**2 + self.b**2 * np.cos(self.angle)**2)
            self.B = float(2*(self.b**2 - self.a**2)*np.sin(self.angle)*np.cos(self.angle))
            self.C = float(self.a**2 * np.cos(self.angle)**2 + self.b**2 * np.sin(self.angle)**2)
            self.D = float(-2*self.A*self.cx - self.B*self.cy)
            self.E = float(-self.B*self.cx - 2*self.C*self.cy)
            self.F = float(self.A*self.cx**2  + self.B*self.cx*self.cy + self.C*self.cy**2 - self.a**2 * self.b**2)
            self.normalize_coefficients()
        else:
            raise ValueError("Cannot calculate coefficients (A,B,C,D,E,F) without parameters (cx, cy angle, a, b)")


    # --------------- ellipse coefficients ---------------
    def set_coefficients_from_points(self, points: np.ndarray, wanna_plot: bool = False, debug: bool = False) -> None:
        """
        Fit an ellipse to a set of points using the Direct Least Squares method.
        Optionally, plot the points and the fitted ellipse.
        Radim Hallir, NUMERICALLY STABLE DIRECT LEAST SQUARES FITTING OF ELLIPSES
        """
        
        # ------------------------ plot
        if wanna_plot:
            _, ax = self.plot_fitting_points(points)

        # ------------------------ Extract x and y coordinates
        x = np.round(points[:, 0], decimals=10)
        y = np.round(points[:, 1], decimals=10)

        # -------------------------------- Design matrices
        D1 = np.vstack([x**2, x*y, y**2]).T         # quadratic part of the design matrix
        D2 = np.vstack([x, y, np.ones(len(x))]).T   # linear part of the design matrix

        # -------------------------------- Scatter matrices
        S1 = D1.T @ D1  # quadratic part of the scatter matrix
        S2 = D1.T @ D2  # combined part of the scatter matrix
        S3 = D2.T @ D2  # linear part of the scatter matrix

        # -------------------------------- check 
        cond_number = np.linalg.cond(S3)
        if cond_number > 10**10:
            self.degenerate_points_fitting = True
            if debug:
                print(f"The condition number of the scatter matrix is too high: {cond_number}")
            if wanna_plot:
                plt.show()
            return

        # --------------------------------Constraint matrix
        C = np.zeros((3, 3))
        C[0, 2] = C[2, 0] = 2
        C[1, 1] = -1

        # -------------------------------- Reduced scatter matrix
        T = -np.linalg.inv(S3) @ S2.T
        M = S1 + S2 @ T                  # Reduced scatter matrix

        # -------------------------------- Solve eigensystem
        _, eigvecs = np.linalg.eig(np.linalg.inv(C) @ M)
        cond = 4 * eigvecs[0] * eigvecs[2] - eigvecs[1]**2

        # -------------------------------- extract solution
        a1_candidates = eigvecs[:, cond > 0]
        if a1_candidates.shape[1] != 1:
            self.degenerate_points_fitting = True
            if debug:
                print(f"{a1_candidates.shape[1]=}")
            if wanna_plot:
                plt.show()
            return
        a1 = a1_candidates

        if a1.size != 3:
            self.degenerate_points_fitting = True
            if debug:
                print(f"{a1.size=}")
            if wanna_plot:
                plt.show()
            return

        A, B, C = a1.flatten()
        D, E, F = (T @ a1).flatten()

        delta = B**2 - 4*A*C
        if abs(delta) <= self.epsilon:
            self.degenerate_points_fitting = True
            if debug:
                print(f"{delta}")
            if wanna_plot:
                plt.show()
            return

        coeffs = np.array([A, B, C, D, E, F])
        norm_factor = np.linalg.norm(coeffs)
        if norm_factor == 0:
            self.degenerate_points_fitting = True
            return
        coeffs /= norm_factor

        self.A, self.B, self.C, self.D, self.E, self.F = tuple(coeffs)
        self.normalize_coefficients()
        
        # ------------------------ plot
        if wanna_plot:
            self.plot_fit(ax)
            plt.show()

    
    # ------------------------------------------------------------------------
    # plot
    # ------------------------------------------------------------------------
    def plot(self) -> None:
        # to plot the ellipse
        if not self.check_if_coefficients_available():
            self.find_coefficients_from_parameters()

        # Define grid for visualization
        x = np.linspace(self.cx - 2 * self.a, self.cx + 2 * self.a, 200)
        y = np.linspace(self.cy - 2 * self.b, self.cy + 2 * self.b, 200)
        X, Y = np.meshgrid(x, y)

        # Evaluate the ellipse equation 
        Z = (self.A * X**2 + self.B * X * Y + self.C * Y**2 +
            self.D * X + self.E * Y + self.F)

        # Plot the ellipse
        plt.figure(figsize=(6, 6))
        plt.contour(X, Y, Z, levels=[0], colors='blue', linewidths=2)
        plt.title("Ellipse Visualization")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.axis('equal')
        plt.grid(True)
        plt.show()



    def plot_fitting_points(self, points: np.ndarray) -> tuple[plt.Figure, plt.Axes]:
        """
        Plot the input points used for fitting and return the figure and axis.
        """
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.scatter(points[:, 0], points[:, 1], color='red', label='Given Points', s=10)
        ax.set_title("Ellipse Fit")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.axis('equal')
        ax.grid(True)
        return fig, ax



    def plot_fit(self, ax: plt.Axes) -> None:
        """
        Plot the fitted ellipse on the given Matplotlib axis.
        """
        # Ensure coefficients are available
        if not self.check_if_coefficients_available():
            raise ValueError("Cannot plot fit: coefficients are not available.")
        # Generate the ellipse points for visualization
        theta = np.linspace(0, 2 * np.pi, 500)
        cx, cy, angle, a, b = self.get_parameters()
        ellipse_x = cx + a * np.cos(theta) * np.cos(angle) - b * np.sin(theta) * np.sin(angle)
        ellipse_y = cy + a * np.cos(theta) * np.sin(angle) + b * np.sin(theta) * np.cos(angle)
        # Plot the ellipse
        ax.plot(ellipse_x, ellipse_y, color='blue', label='Fitted Ellipse', linewidth=2)
        ax.legend()


    # ------------------------------------------------------------------------
    # point-ellipse distance calculation
    # ------------------------------------------------------------------------
    def check_points_distance(self, points: np.ndarray, DISTANCE_AREA_INCREASE: float = 1.0, DISTANCE_AREA_DECREASE: float = 1.0) -> np.ndarray:
        """
        Check which points lie within a scaled distance threshold from the ellipse.
        """
        cx, cy, angle, a, b = self.get_parameters()  

        # ideally translate and rotate the ellipse in the origin, with the axis parallel to x,y axis
        # make the same transforation to the point
        x = points[:, 0]
        y = points[:, 1]
        p = np.vstack((x, y, np.ones_like(x)))

        R = np.array([[np.cos(angle), -np.sin(angle), cx],
                      [np.sin(angle),  np.cos(angle), cy],
                      [            0,              0,  1]])
        
        pt = np.linalg.inv(R) @ p # transformed point
        xt = pt[0] # transformed coordinate
        yt = pt[1] # transformed coordinate

        # calculate the distance treshold and the array of results
        k = (xt / a)**2 + (yt / b)**2
        is_inside = (DISTANCE_AREA_DECREASE <= k) & (k <= DISTANCE_AREA_INCREASE)

        return is_inside
    

    # ---------------------------------------------------
    # utilities
    # ---------------------------------------------------
    def check_if_coefficients_available(self) -> bool:
        """
        Check if all ellipse coefficients are available and clean them of complex parts.
        """
        if all(coeff is not None for coeff in [self.A, self.B, self.C, self.D, self.E, self.F]):
            self.A = float(self.A.real) if isinstance(self.A, complex) else float(self.A)
            self.B = float(self.B.real) if isinstance(self.B, complex) else float(self.B)
            self.C = float(self.C.real) if isinstance(self.C, complex) else float(self.C)
            self.D = float(self.D.real) if isinstance(self.D, complex) else float(self.D)
            self.E = float(self.E.real) if isinstance(self.E, complex) else float(self.E)
            self.F = float(self.F.real) if isinstance(self.F, complex) else float(self.F)
            return True
        return False

    

    def check_if_parameters_available(self) -> bool:
        """
        Check if all ellipse parameters are available.
        """
        if (self.cx is not None and
            self.cy is not None and
            self.angle is not None and
            self.a is not None and
            self.b is not None): return True
        else: return False



    def normalize_angle(self) -> None:
        """
        Normalize the ellipse angle to lie within [-π/2, π/2].
        """
        # adjust inside [-pi, pi]
        self.angle = (self.angle + np.pi) % (2 * np.pi) - np.pi
        # adjust inside [-pi/2, pi/2]
        if self.angle > np.pi / 2:
            self.angle -= np.pi
        elif self.angle < -np.pi / 2:
            self.angle += np.pi



    def normalize_coefficients(self) -> None:
        """
        Normalize conic coefficients so that A = 1 and round the rest.
        """
        decimals_digit = 10
        self.B = np.round(self.B/self.A, decimals=decimals_digit)
        self.C = np.round(self.C/self.A, decimals=decimals_digit)
        self.D = np.round(self.D/self.A, decimals=decimals_digit)
        self.E = np.round(self.E/self.A, decimals=decimals_digit)
        self.F = np.round(self.F/self.A, decimals=decimals_digit)
        self.A = 1

    

    def check_if_inside_box(self, height: int, width: int) -> bool:
        """
        Check if the ellipse lies within a box of given height and width.
        """
        # simplified version for speed, ignoring the rotation
        if (self.cx - self.a < 0 or self.cx + self.a > width or  # horizontal axis
            self.cy - self.b < 0 or self.cy + self.b > height):  # vertical axis
            return False
        return True
    


