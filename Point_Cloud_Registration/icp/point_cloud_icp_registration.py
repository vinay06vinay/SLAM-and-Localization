
import open3d as o3d
import numpy as np
import plotly.graph_objects as go
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class PointCloud:
    """
    A class to represent a point cloud and perform operations like ICP (Iterative Closest Point) algorithm.

    Attributes:
    source : open3d.geometry.PointCloud
        The source point cloud which is the rotated pcd
    target : open3d.geometry.PointCloud
        The target point cloud which is the normal pcd
    """
    def __init__(self,source_pcd,target_pcd):
        self.source = source_pcd
        self.target = target_pcd
        # Assign colors for source and target
        self.source.paint_uniform_color([0, 0.651, 0.929])
        self.target.paint_uniform_color([1, 0.706, 0])

    def find_nearest_neighbor_pcd(self,source_pcd,target_pcd):
        """
        Finds the nearest neighbors of points in the target point cloud 
        from the source point cloud using a KD-Tree for efficient searching.

        Returns:
        numpy.ndarray: Array of nearest neighbor points from the source point cloud.
        """

        # A KD tree is built to organize the source point cloud such that it can be queried easily
        point_cloud_tree = o3d.geometry.KDTreeFlann(source_pcd)
        points_arr = []
        # for each point in target point cloud, its nearest neigbhor for source point cloud is add to new array
        for point in target_pcd.points:
            # get the first nearest neigbor
            _, index, _ = point_cloud_tree.search_knn_vector_3d(point, 1)
            # print(a,b)
            points_arr.append(source_pcd.points[index[0]])
        # return the source pcd as numpy array
        return np.asarray(points_arr)
    
    def compute_transformation_matrix(self,source,target):
        """
        Computes the transformation matrix that aligns the source point cloud 
        with the target point cloud using the SVD.

        Returns:
        numpy.ndarray: The transformation matrix (4x4) to align source to target.
        """
        source_mean = np.mean(source, axis=0)
        target_mean = np.mean(target, axis=0)
        # centered source and target points around the mean
        source_centered = source - source_mean
        target_centered = target - target_mean
        # Compute covariance matrix 
        H = np.dot(source_centered.T, target_centered)

        # SVD
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)  # Rotation matrix
        t = target_mean - np.dot(R, source_mean)  # Translation vector

        # Transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = R
        transformation_matrix[:3, 3] = t
        return transformation_matrix

    def icp(self):
        """
        Performs the Iterative Closest Point (ICP) algorithm to align the source 
        point cloud to the target point cloud iteratively.

        The function continues to iterate until convergence is reached or the 
        maximum number of iterations is exceeded. The transformation matrix is 
        printed and the final point clouds are visualized.
        """
        max_iterations = 100
        prev_error = float('inf')
        threshold = 0.0000001
        # Initial transformation matrix set to identity
        transform_matrix = np.eye(4)
        target_pts = np.asarray(self.target.points)
        
        for i in range(max_iterations):
            # Search for nearest neigbors to target point cloud using the source point cloud
            source_close_pts = self.find_nearest_neighbor_pcd(self.source, self.target)
            # Compute transformation matrix
            transformation = self.compute_transformation_matrix(source_close_pts, target_pts)
            # Apply transformation to the source close pts
            source_homogeneous = np.hstack((source_close_pts, np.ones((source_close_pts.shape[0], 1))))
            transformed_source = (transformation @ source_homogeneous.T).T[:, :3]
            # Calculate mean squared error between transformed source and target
            mean_error = np.mean(np.linalg.norm(transformed_source - target_pts, axis=1)**2)
            # Check for convergence
            if np.abs(prev_error - mean_error) < threshold:
                print(f"Converged at iteration {i+1}.")
                break
            # Only apply the transformation if the error improves (error is smaller than the previous error)
            if mean_error < prev_error:
                # Update the cumulative transformation matrix
                transform_matrix = transformation @ transform_matrix
                # Transform the source point cloud
                self.source.transform(transformation)
                # Update previous error
                prev_error = mean_error
                # o3d.visualization.draw_geometries([self.source, self.target])
            else:
                print(f"Error did not improve at iteration {i+1}. Stopping.")
                break
        print("Final Transformation Matrix:\n", transform_matrix)
        print("Final Visualization")
        # Visualize using open3d
        o3d.visualization.draw_geometries([self.source, self.target])


    def visualize_using_plotly(self):
        """
        Visualizes the source and target point clouds using Plotly.

        A 3D scatter plot is created for both point clouds.
        """
        source_pts = np.asarray(self.source.points)
        scatter_source = go.Scatter3d(x=source_pts[:, 0], y=source_pts[:, 1], z=source_pts[:, 2], mode='markers', marker=dict(size=1.0))
        target_pts = np.asarray(self.target.points)
        target_source = go.Scatter3d(x=target_pts[:, 0], y=target_pts[:, 1], z=target_pts[:, 2], mode='markers', marker=dict(size=1.0))
        fig = go.Figure(data=[scatter_source,target_source])
        fig.show()

    def visualize_using_matplotlib(self):
        """
        Visualizes the source and target point clouds using Matplotlib.

        Two subplots are created: one for the source point cloud and one for the target point cloud.
        """
        source_pts = np.asarray(self.source.points)
        target_pts = np.asarray(self.target.points)
        # Create a figure with 1 row and 2 columns for the subplots
        fig = plt.figure(figsize=(16, 8))
        # First subplot for source point cloud
        ax1 = fig.add_subplot(121, projection='3d')
        ax1.scatter(source_pts[:, 0], source_pts[:, 1], source_pts[:, 2], c='b', marker='o', s=0.2, label='Source')
        ax1.set_xlabel('X Label')
        ax1.set_ylabel('Y Label')
        ax1.set_zlabel('Z Label')
        ax1.set_title('Source Point Cloud')
        ax1.legend()
        # Second subplot for target point cloud
        ax2 = fig.add_subplot(122, projection='3d')
        ax2.scatter(target_pts[:, 0], target_pts[:, 1], target_pts[:, 2], c='y', marker='o', s=0.2, label='Target')
        ax2.set_xlabel('X Label')
        ax2.set_ylabel('Y Label')
        ax2.set_zlabel('Z Label')
        ax2.set_title('Target Point Cloud')
        ax2.legend()
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    ply_file_path_read = "bunny-pcd.ply"
    pcd = o3d.io.read_point_cloud(ply_file_path_read)
    # Downsample Sample points uniformly from the point cloud
    pcd_ds = pcd.uniform_down_sample(every_k_points=5)
    # Take another pcd and rotate this point cloud
    rotated_pcd = o3d.io.read_point_cloud(ply_file_path_read)
    rotated_pcd_ds = rotated_pcd.uniform_down_sample(every_k_points=5)
    # Create a rotation matrix for (45, 30, 60) degrees around (X, Y, Z)
    R = pcd.get_rotation_matrix_from_xyz((np.pi/4, np.pi/6, np.pi/3))
    rotated_pcd_ds.rotate(R, center=(0, 0, 0))
    # Visualize the original and rotated point clouds separately
    pcd_ds.paint_uniform_color([1, 0.706, 0])  # Set color for original
    rotated_pcd_ds.paint_uniform_color([0, 0.651, 0.929])  # Set color for rotated
    o3d.visualization.draw_geometries([pcd_ds, rotated_pcd_ds])
    # Apply ICP to align the rotated point cloud back to the original
    pcd_obj = PointCloud(rotated_pcd_ds, pcd_ds)
    pcd_obj.icp()

    # Uncomment below two lines to visualize using different libraries
    # pcd_obj.visualize_using_plotly()
    # pcd_obj.visualize_using_matplotlib()
