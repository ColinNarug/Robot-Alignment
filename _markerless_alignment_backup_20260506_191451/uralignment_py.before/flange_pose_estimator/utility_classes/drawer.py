from typing import List, Union
import numpy as np
import cv2
from spatialmath import SE3



class DrawProxy:

    def __init__(self, drawer, image_instance):
        self.drawer = drawer
        self.image_instance = image_instance

    def __getattr__(self, name):
        func = getattr(self.drawer, name)

        def wrapped_func(*args, **kwargs):
            drawn_image = func(*args, **kwargs)
            self.image_instance.images_with_drawings.append(drawn_image)
            return drawn_image

        return wrapped_func



class Drawer():

    def __init__(self, base_image: np.ndarray, params, axis_length: int = 0.025):
        
        self.base_image = base_image
        self.params = params
        self.camera_matrix = self.params.camera_params.CAMERA_MATRIX
        self.axis_length = axis_length      # [mm] axis length drawn on the flange 
        


    def pose_axes(self, T: SE3, image: np.ndarray = None, show: bool = False) -> np.ndarray:
        """
        Draw a pose RF with "rgb = xyz" convention
        """

        if image is None: image = self.base_image
        image = image.copy()

        R = T.R
        rvec, _ = cv2.Rodrigues(R)
        tvec = T.t


        # Define axis points and origin
        axis = np.float32([[self.axis_length, 0,                0],
                          [0,                 self.axis_length, 0],
                          [0,                 0,                self.axis_length]]).reshape(-1, 3)
        origin = np.float32([[0, 0, 0]]).reshape(-1, 3)

        # Project axis points and origin
        imgpts, _ = cv2.projectPoints(np.vstack((origin, axis)), rvec, tvec, self.camera_matrix, None)
        imgpts = np.int32(imgpts).reshape(-1, 2)

        # Draw origin and axes with arrows
        cv2.arrowedLine(image, tuple(imgpts[0]), tuple(imgpts[1]), (0, 0, 255), 3, tipLength=0.2)  # X in red
        cv2.arrowedLine(image, tuple(imgpts[0]), tuple(imgpts[2]), (0, 255, 0), 3, tipLength=0.2)  # Y in green
        cv2.arrowedLine(image, tuple(imgpts[0]), tuple(imgpts[3]), (255, 0, 0), 3, tipLength=0.2)  # Z in blue

        if show:
            cv2.imshow("PnP", image)
            cv2.waitKey(0)
        return image



    def ellipses(self, ellipses: List, image: np.ndarray = None) -> np.ndarray:
        """
        Draw a list of ellipses on the image, including their IDs.
        """
        
        if image is None: image = self.base_image
        image = image.copy()

        for ellipse in ellipses:
            
            cx, cy, angle, a, b = ellipse.get_parameters()
            center = (int(round(cx)), int(round(cy)))
            axes = (int(round(a)), int(round(b)))
            angle_deg = np.degrees(angle)

            # Draw ellipse
            cv2.ellipse(image, center, axes, angle_deg, 0, 360, (255, 0, 0), 1)

        return image




    @staticmethod
    def get_holes_centers(holes: List, reference_frame: str, only_with_camera_repr: bool = False, only_with_projection: bool = False) -> np.ndarray:
        """
        From a list of holes, get the corresponding list of centers
        """

        centers = []
        for hole in holes :

            if only_with_camera_repr and hole["camera"].center is None:
                continue

            if only_with_projection and hole.projection_edge is None:
                continue

            centers.append(hole[reference_frame].center)

        return np.array(centers)
    



    @staticmethod
    def project_3D_points_on_screen(points: Union[np.ndarray, List[np.ndarray]], camera_matrix: np.ndarray, only_with_projection: bool = False) -> np.ndarray:
        """
        Projects 3D points on the screen, described in the camera RF,
        and returns the corresponding 2D projection points.
        Accepts either a single 3D point or an array of 3D points.
        """

        # Convert input to at least 2D and float32
        points = np.asarray(points, dtype=np.float32)
        
        # If a single point is passed, reshape to (1, 3)
        if points.ndim == 1 and points.shape[0] == 3:
            points = points.reshape(1, 3)
        elif points.ndim == 2 and points.shape[1] != 3:
            raise ValueError(f"Expected 3D points with shape (N, 3), got shape {points.shape}")

        # Identity rotation and zero translation (points already in the camera frame)
        rvec = np.zeros((3, 1), dtype=np.float32)
        tvec = np.zeros((3, 1), dtype=np.float32)

        # Project all points
        points_projected, _ = cv2.projectPoints(points, rvec, tvec, camera_matrix, None)

        # Flatten to get 2D points with shape (N, 2)
        return points_projected.reshape(-1, 2)
    



    def PnP(self, T: SE3, holes: List, image: np.ndarray = None, show: bool = False) -> np.ndarray:
        """
        Draw projected 3D model points and 2D ellipse centers.
        """
        if image is None: image = self.base_image
        image = image.copy()

        # 3D points of the holes in flange reference frame (object points)
        objectpoints = self.get_holes_centers(holes, "flange", only_with_projection = True)
        # 2D points of detected holes projected onto the image (image points)
        imagepoints = np.array([self.project_3D_points_on_screen(hole["camera"].center, self.camera_matrix).ravel()
                                for hole in holes if hole["camera"].center is not None])

        # Project objectpoints (3D -> 2D)
        R = T.R
        rvec, _ = cv2.Rodrigues(R)
        tvec = T.t
        
        objectpoints_projected, _ = cv2.projectPoints(objectpoints, rvec, tvec, self.camera_matrix, None)
        objectpoints_projected = np.int32(objectpoints_projected).reshape(-1, 2)

        offset_x = 8   
        offset_y = -8  

        # Plot
        for i in range(len(imagepoints)):

            impt = tuple(np.int32(imagepoints[i]).flatten())  
            cv2.circle(image, impt, 5, (20, 20, 20), -1)

            if holes[i]["camera"].center is None: continue
            obpt = tuple(objectpoints_projected[i])
            cv2.circle(image, obpt, 3, (0, 255, 0), -1)

            text_position = (int(impt[0] + offset_x), int(impt[1] + offset_y))            
            cv2.putText(image, f"{holes[i].ID}", text_position , 1,  2, (0, 255, 0), 2)

        if show:
            cv2.imshow("PnP", image)
            cv2.waitKey(0)
        return image



    def homography_analysis(self, matching_points: List[List[np.ndarray]], image_with_homography_analysis: np.ndarray = None) -> np.ndarray:

        scale = 8

        if image_with_homography_analysis is None:
            image_with_homography_analysis = self.create_base_image_with_homography_analysis(scale)

        img = image_with_homography_analysis.copy()
        img_size = img.shape[0]
        center = (img_size // 2, img_size // 2)

        for match in matching_points:
            if len(match) < 2:
                continue  

            point1, point2 = match[0], match[1]

            x1 = int(center[0] + point1[0] * scale)
            y1 = int(center[1] + point1[1] * scale)

            x2 = int(center[0] + point2[0] * scale)
            y2 = int(center[1] + point2[1] * scale)

            cv2.circle(img, (x1, y1), 3, (255, 200, 0), thickness=-1)  
            cv2.circle(img, (x2, y2), 3, (0, 140, 255), thickness=-1)

        return img



    def create_base_image_with_homography_analysis(self, scale: int = 8) -> np.ndarray:

        img_size = 1500  
        center = (img_size // 2, img_size // 2)

        img = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255

        for crown in self.params.flange_model_params.CROWNS:    
            crown_radius_px = crown.CROWN_RADIUS * scale
            hole_radius_px = crown.HOLE_RADIUS * scale

            for i in range(crown.NUMBER_OF_HOLES):
                if i in crown.SKIP_INDICES:
                    continue

                angle = (2 * np.pi / crown.NUMBER_OF_HOLES) * i + crown.ANGLE_OFFSET
                x = int(center[0] + crown_radius_px * np.cos(angle))
                y = int(center[1] + crown_radius_px * np.sin(angle))

                cv2.circle(img, (x, y), int(hole_radius_px), (0, 0, 0), thickness=3)
                cv2.circle(img, (x, y), 1, (0, 255, 0), thickness=3)

        flange_radius = self.params.flange_model_params.RADIUS * scale

        x = int(center[0])
        y = int(center[1])
        cv2.circle(img, (x, y), flange_radius, (0, 0, 0), thickness=3)
        axis_length = 100

        cv2.arrowedLine(img, center, (center[0], center[1] + axis_length), (0, 255, 0), thickness=4, tipLength=0.05)  # Verde verso il basso (asse Y)
        cv2.arrowedLine(img, center, (center[0] + axis_length, center[1]), (0, 0, 255), thickness=4, tipLength=0.05)  # Rosso verso destra (asse X)
        cv2.circle(img, center, 7, (255, 0, 0), thickness=-1)  # Blu

        return img        
