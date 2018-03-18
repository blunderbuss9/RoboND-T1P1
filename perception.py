import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 255
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world
	
def region_of_interest(img, vertices):
    """
    Applies an image mask.
    
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)   
    
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

# Define a function to identify rocks in an image
def find_rocks(img, levels=(110,110,50)):
    # Yellow rocks are bright in Red/Green channels, and dim in Blue channel
    rockpix = ((img[:,:,0] > levels[0]) & (img[:,:,1] > levels[1]) & (img[:,:,2] < levels[2]))
    color_select = np.zeros_like(img[:,:,0])
    color_select[rockpix] = 1
    return color_select

# Define a function to perform a perspective transform.	
def perspect_transform(img, src, dst):         
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    return warped

		
"""
# Define function to process images and corresponding rover X/Y positions and yaw angles,
# passed from the Simulator running in Autonomous Mode.
# 
# This function is called by telemetry(), which is defined in drive_rover.py
"""	 
def perception_step(Rover):
    #
    # Perform perception steps to update Rover state data values maintained in object Rover,
    # of Class Rover_State().
    # (Apply the functions defined above, in succession, and update the Rover state accordingly)
	#
    # NOTES: - Current camera image is in Rover.img
    # 		 - I've added the elements Rover.source and Rover.destination to the Rover() class;
	#          the values for thesse elements are initialized at Rover() instantiation to contain
    #          the pre-calibrated points used in the perspect_transform() computation.
    #          (This implementation saves compute time, versus re-computing these same unchanging
    #          values for these points, each time this perception_step() function is called).
	#
	# PSUEDO CODE:
    # 1) Apply perspective transform
    # 2) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 3) Apply a mask to thresholded image, to eliminate sky, and other extreme extraneous detections
    # 4) Update Rover.vision_image (will be displayed on left side of Sim GUI, above camera view)
    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    
    # 1) Apply perspective transform to generate "warped birds-eye view", from original input image
    warped = perspect_transform(Rover.img, Rover.source, Rover.destination)
	
	# 2) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped) # Create warped/thresholded image
    obs_map = np.absolute(np.float32(threshed)-1)
    rock_map = find_rocks(warped)
	
	# 3) Apply a mask to thresholded image, to eliminate sky, and other extreme extraneous detections
    imshape = Rover.img.shape
    # Define a four sided polygon to mask
    vertices = np.array([[(148,imshape[0]),(124, 96), (196, 96), (imshape[1]-148,imshape[0])]], dtype=np.int32)   	
    masked_img = region_of_interest(threshed, vertices) # Create an image with top and side edges masked
    obs_masked_img = region_of_interest(obs_map, vertices) # Again for obstacle map
    rock_masked_img = region_of_interest(rock_map, vertices) # Again for rock image
	
    # 4) Update Rover.vision_image (will be displayed on left side of Sim GUI, above camera view)	
    Rover.vision_image[:,:,2] = masked_img # Put the perceived navigable terrain in the blue channel
    Rover.vision_image[:,:,0] = obs_masked_img
	
    # 5) Convert map image pixel values to rover-centric coords	
    xpix, ypix = rover_coords(masked_img) # Extract navigable terrain pixels
    obs_xpix, obs_ypix = rover_coords(obs_masked_img) # Extract navigable terrain pixels
    if rock_masked_img.any(): # Extract rock pixels
        rock_xpix, rock_ypix = rover_coords(rock_masked_img)
		
    # 6) Convert rover-centric pixel values to world coordinates	
    x_world, y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw,
                                    Rover.worldmap.shape[0], 10)  # 10 = scale = 2*dst_size
    # Get obstacle pixel positions in world coords
    obs_x_world, obs_y_world = pix_to_world(obs_xpix, obs_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)
    if rock_masked_img.any():      
        rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)
			
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[y_world, x_world, 2] += 10     # Add pixel positions to worldmap
    Rover.worldmap[obs_y_world, obs_x_world, 0] += 1     # Add pixel positions to worldmap
    nav_pix = Rover.worldmap[:,:,2] > 0
    Rover.worldmap[nav_pix,0] = 0
    if rock_masked_img.any(): # Add rock pixel positions to worldmap
        Rover.worldmap[rock_y_world, rock_x_world,:] = 255  # Add rock pixel positions (as white) to worldmap
	
	# 8) Convert rover-centric pixel positions to polar coordinates
    if rock_masked_img.any():
        dist, angles = to_polar_coords(rock_xpix, rock_ypix)
        Rover.see_sample = True
    else:
        dist, angles = to_polar_coords(xpix, ypix)
        Rover.see_sample = False
    Rover.nav_angles = angles	
    #Rover.nav_dists = dist
	
    return Rover