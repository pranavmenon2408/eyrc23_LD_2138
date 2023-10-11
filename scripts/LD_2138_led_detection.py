# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2

# load the image, 
image = cv2.imread('led.jpg', 1)

# convert it to grayscale, and blur it
gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
blur=cv2.blur(gray,(25,25))
# threshold the image to reveal light regions in the blurred image
_,thresh=cv2.threshold(blur,127,255,cv2.THRESH_BINARY)

# perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
#kernel=np.ones((5,5),np.uint8)
thresh=cv2.erode(thresh,None,iterations=2)
thresh=cv2.dilate(thresh,None,iterations=4)
# perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components
labels=measure.label(thresh, connectivity=2, background=0)
mask=np.zeros(thresh.shape,dtype="uint8")
# loop over the unique components
for label in np.unique(labels):
    # if this is the background label, ignore it
    if label==0:
        continue

	# otherwise, construct the label mask and count the number of pixels 
    labelMask=np.zeros(thresh.shape,dtype="uint8")
    labelMask[labels==label]=255
    numPixels=cv2.countNonZero(labelMask)
	# if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"
    if numPixels >300:
        mask=cv2.add(mask,labelMask)
	
# find the contours in the mask, then sort them from left to right
cnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
cnts=imutils.grab_contours(cnts)
cnts=contours.sort_contours(cnts)[0]
# loop over the contours

# Initialize lists to store centroid coordinates and area
centroid=[]
areacnt=[]
# Loop over the contours
for (i,c) in enumerate(cnts):


    # Calculate the area of the contour
    area=cv2.contourArea(c)

    # Draw the bright spot on the image
    (x,y,w,h)=cv2.boundingRect(c)
    ((cX,cY),radius)=cv2.minEnclosingCircle(c)
    cv2.circle(image,(int(cX),int(cY)),int(radius),(0,0,255),3)
    cv2.putText(image, "#{}".format(i + 1), (x, y - 15),
		cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)




    # Append centroid coordinates and area to the respective lists
    centroid.append((cX,cY))
    areacnt.append(area)
# Save the output image as a PNG file
cv2.imwrite("LD_2138_led_detection_results.png", image)
cv2.imshow('led_image',image)

# Open a text file for writing
with open("LD_2138_led_detection_results.txt", "w") as file:
    # Write the number of LEDs detected to the file
    file.write(f"No. of LEDs detected: {len(areacnt)}\n")

    # Loop over the contours
    for i in range(len(areacnt)):
    
        # Write centroid coordinates and area for each LED to the file
        file.write(f"Centroid #{i + 1}: {centroid[i]}\nArea #{i + 1}: {areacnt[i]}\n")
# Close the text file
if cv2.waitKey(0)==ord('q'):
    cv2.destroyAllWindows()
