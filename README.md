# Ball-and-SoccerRegion-detection
The Region detection function file contains a function to detect regions of a soccer field using edge detection and boundedRect function. The input is raw coloe image and output are the points of the rectangle which bound inner Rectangle and D region of the soccer field. The Screen shots shows the regions detected using this function in different colors.


I have also included the src file which includes different functions to detect the ball and different regions of a soccer field.

This project is divided into two tasks; Task1: Detect the ball and Task 2: detect the various regions of football field. All the above tasks are carried out in real-world scenario and not in simulation. Note: Since Darwin-OP were used, the images had poor quality (more noise) and lower resolution (minute/fine details were lost).

For my implementation, I preprocessed the image differently for each of the above tasks. 

Task 1: I started out by creating a Gaussian pyramid and stopped when the resolution reached at most 800 (I down sampled to reduce processing time). I tried down sampling further but that resulted in blurry edges when the ball was further away. Then I used percent of green formula to threshold pixels whose green value in RGB vector is at least 44 percent. By doing this I got a field image with black pixels for all the green color(turf) and white for all the rest, using this I achieved a close to perfect thresholding, differentiating the turf from all the other surroundings. The logic behind this was to get an image(Ip) with borders (different regions/lines of the field) and ball blob inside the black region(turf). Then I applied canny detector to both the greyscale image and the image(Ip), and combined both the edges to get a better edge for the ball. Since I used Hough circle transform, I kept on detecting the ‘D’ region as the ball. Which was the reason behind adding all the edges from different images, so that I get more edges for the ball thereby getting more hits from Hough circle transform. Because I assumed that Hough circle transform worked by getting all the shapes which represented a part of a circle, even if it were a semi-circle or a big arc. 

It should have used the relation ‘2πr’ to detect shapes whose no. of hits from Hough circle transform are close to that value. This would have resolved the issue of detecting semicircles. Since the hits are stored in the accumulator from Hough circle transform, all we could do is to get concentric edges for the ball, so that we get more hits for the ball than the ‘D’ region. 

TODO: Either use ‘2πr’ relation or get better concentric edges for the ball (these edges shouldn’t include the texture of the ball). Since I can’t use the relation I am sometimes detect the semicircle as the ball.

I also added a function called findBallUsingTemplate. This approach is too slow but accurate (accurate in the sense that it will detect a circle and not semi-circles). This function could be used in events where there is no need to keep tracking the ball.

Task 2: I again started out by creating a Gaussian pyramid and stopped when the resolution reached at most 400. Then I used percent of green formula to threshold pixels whose green value in RGB vector is at least 44 percent. Then I applied canny detector and then I found the contours to get the inner rectangle (reference point). 

TODO: Using this logic I sometimes detect the penalty dot (which was rectangle) and the semicircle (which because of poor quality looked like a rectangle from certain perspectives), I could filter the rectangle with the max side as inner rectangles sides is larger than both. This will be true from any perspective or orientation. My logic was to find the inner rectangle then draw all the other regions (as other regions dimensions could be defined relative to the inner rectangles dimension). So, after getting the inner rectangle, apply perspective transformation based on the angle of the inner rectangle then find all the regions and then rotate the whole image back. 
Other approaches:

1>	I tried using corner detection to detect the distinct ‘L’ shaped corner, I tried using Harrison detection. I also tried a couple of filters to detect a specific corner but the filter had to be robust to detect corner from any perspective.

2>	I also used line segment detector to get the lines but didn’t see its use to get a specific reference point (as we can get the same relation for other edges from different perspective as well). Line segment could be used to get the slopes that is more frequent (slopes from parallel lines) than the other slopes (slopes from ball and other noise).

As in any case, there is always a perspective from where we won’t be able to detect any reference point or “edges that have a relation”. It runs faster than 30 frames/second.
