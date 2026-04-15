import sys
# sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
# sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages') # append back in order to import rospy
import numpy as np
import os
import os.path
import re  # regular expression


def create_video(dir_path, fps):

    # Count number of files in directory
    img_num = 0
    frame_pattern = re.compile(r'^frame_\d{10}\.png$')  # Regex for frame_0000000001.png format
    for path in os.listdir(dir_path):
        if os.path.isfile(os.path.join(dir_path, path)) and frame_pattern.match(path):
            img_num += 1

    print('Number of images: {}'.format(img_num))

    # Get shape of image
    img = cv2.imread('{0}frame_0000000001.png'.format(dir_path))
    if img is None:
        print(f"Error: Unable to read the first frame '{dir_path}frame_0000000001.png'. Check the file path or integrity.")
        sys.exit(1)

    frameSize = (img.shape[1], img.shape[0])

    # Construct the output video file name
    video_filename = f"{dir_path}output_video_{fps}fps.mp4"
    video_path = os.path.join(dir_path, video_filename)

    # Writer
    out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4V'), fps, frameSize)

    for i in range(1, img_num+1):
        print('Frame: {}'.format(i))
        num_zero = 10 - len(str(i))
        prefix = '0' * num_zero
        filename = '{0}frame_{1}{2}.png'.format(dir_path, prefix, i)
        img = cv2.imread(filename)
        if img is not None:
            out.write(img)
        else:
            print(f"Warning: Unable to read frame '{filename}'. Skipping.")
    print(f'Video saved to {video_path}')

    out.release()


if __name__ == "__main__":

    # Ensure the script is called with the correct number of arguments
    if len(sys.argv) != 2:
        print("Usage: python create_video.py <absolute-image-dir-path>")
        sys.exit(1)

    # dir_path = '/home/genki/GIT/argos-sct/frames/'
    path = os.path.join(os.environ['HOME'], sys.argv[1])
    fps = 10
    create_video(path, fps)