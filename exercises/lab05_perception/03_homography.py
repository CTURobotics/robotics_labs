import numpy as np
import json
from matplotlib import pyplot as plt
from pathlib import Path
import cv2
from robotics_toolbox.utils.perception import find_hoop_homography

def main():
    repo_dir = Path(__file__).parent.parent.parent
    data_dir = repo_dir / "exercises" / "lab05_perception" / "hw_data"

    # Load images
    images_paths = [str(data_dir / f"hoop_grid_image_{i}.png") for i in range(17)]
    images = [cv2.imread(pth) for pth in images_paths]

    # Load json with hoop positions
    with open(data_dir / 'hoop_positions.json') as f:
        hoop_positions = json.load(f)

    # HW: Implement the find_hoop_homography function
    H = find_hoop_homography(images, hoop_positions)
    print(np.round(H, 4))

    # Take the hoop points and transform them to image space with with H^-1
    H_inv = np.linalg.inv(H)
    hoop_pts = np.array([x['translation_vector'] for x in hoop_positions])[:, :2]
    hoop_pts = np.pad(hoop_pts, ((0,0),(0,1)), constant_values=1) # Convert to homogeneous coords
    img_pts = (H_inv @ hoop_pts.T).T
    img_pts = img_pts[:,:2]/img_pts[:,2:].reshape(-1,1) # Convert back to cartesian coords
    # Notice the .reshape(-1,1), this makes sure the shapes are compatible,
    # i.e. it transforms the homogeneous component to a column vector


    for img,pt in list(zip(images, img_pts))[::4]: # Show every 4-th image only
        plt.figure(figsize=(9,6))
        plt.imshow(img[:,:,::-1])
        plt.scatter(*pt, c='r', s=20, marker='x')
        plt.tight_layout()
        plt.show()



if __name__ == '__main__':
    main()