# machine_vision

## Project Goal

Our goal was to make a Neato solve a simple maze by using its camera to detect tape paths on the floor. We were inspired by search-and-rescue robots, which must navigate unknown environments using real-time path-planning to locate an object or a person.

![Neato test maze](/machine_vision/machine_vision/test_maze.jpg)

## Methods

The Neato subscribes to one topic, ‘/camera/image_raw’, and publishes to two topics, ‘cmd_vel’ and ‘/filtered_image_raw’.

To handle navigation, we used the OpenCV library to create a binary image mask which filters out all colors beyond the given threshold values. The Neato then detects where paths are using the state of pixels at specified locations. For example, if a set of pixels in the lower right quadrant of the camera image falls within the color range, the Neato detects that a path exists to its right. If two or more paths exist, the Neato detects an intersection. If no paths exist, the Neato detects a dead end. A bounding box has been placed around the set of pixels for each path detection:

![Bounding boxes](/machine_vision/machine_vision/BoundingBoxes.png)

Using a finite state machine, the Neato switches between five different modes: simple, intersection, investigate, turn back, and retrace steps. This makes the code modular and enables the Neato to separate out the tasks of driving, searching for paths, and solving the maze.

Before continuing, please note that “mode” and “state” are used interchangeably, but both refer to changing states similar to what was done in the “Finite State Machines” project.

### Code Structure Overview

class maze_vision(Node):

Image processing:
- image_callback( )
    - Implement the image mask to filter out every color except the yellow of the tape
    - Set trigger values—these are arrays of pixels that specify a "bounding box” in which we expect to see the tape if a path exists for that direction
    - Count how many pixels fall within the bounds specified by each trigger to determine whether a left, middle, or right path exists
    - Based on the existence of those paths, determine if an intersection or a dead end exists at this location

Helper functions:
- drive( )
    - Command the Neato to move with a specified angular and linear velocity
- go_forward( )
    - Command the Neato to drive forward in a straight line
- turn_right( )
    - Go forward by the buffer amount (since the neato cannot see below it) 
    - Command the Neato to turn right 90 degrees
- turn_left( )
    - Go forward by the buffer amount (since the neato cannot see below it) 
    - Command the Neato to turn left 90 degrees
- add_intersection_stack( )
    - Adds the detected paths to the maze stack.

Maze modes:
(See Maze Stack Logic for full explanation of modes)
- simple_function()
- intersection_function()
- investigate_function()
- turn_back_function()
- retrace_steps_function()

Main loop:
- run_loop( )
    - Switches modes

### Maze Stack Logic

The Logic:
The maze logic works primarily by switching the Neato into 5 different states depending on the camera’s filtered images. Each state manages and processes the maze in a different way. When beginning the maze, the Neato will always start in the “simple” state, which triggers a basic line following function. Below is a summary of each state and what it does. 

The 5 modes:

**Simple**

Every maze always starts by following a straight line (even if it’s very short!) before it reaches the first intersection. This function handles this.
Go forward while the neato is centered on the line
Log any sensed paths
If the neato senses a right path, store it (simple_right = true).
If the neato senses a left path, store it (simple_left = true).
If it’s a dead end and only one path is true, turn into that true direction.
If multiple paths were triggered, we are no longer in a simple line following section. Change the state to intersection, and start solving the maze.

**Intersection**

This mode simply logs an intersection.
First, any sensed paths that the Neato sees is placed into the maze stack as “L”, “M”, or “R”.
If the maze stack already has information this means it’s a sub intersection, so a comma is added to the stack before adding the new values to the stack. 
Then, switch to investigation mode.

**Investigation**

This mode causes the Neato to investigate paths.
FIRST: (and only once), do an investigative turn. Look at the top of the stack and turn in that direction.
In cases where the top of the stack is m, go forward slightly to avoid seeing the current intersection 
Now the neato is facing the direction of the top of the stack and can begin a thorough investigation.
Continue going forward (and turning left and right as needed for non-intersections) until you reach an intersection or a dead end, then react accordingly
Dead end: Trigger the “BACK” mode for backtracking.
Intersection: Trigger the “INTERSECTION” mode.

**Back**

This is a simple mode that simply turns the Neato 180 in any direction, then switches to retrace mode.

**Retrace**

As long as the Neato is not at an intersection, keep going forward.
if it’s not an intersection, go forward.
Follow any simple turns (left or right) that you see.
If it is an intersection, you’ve retraced your steps, and you are back where the neato was before it started investigating.
Look at the top of the stack and then turn 90 degrees in that direction to face the same way you did before investigating.
In cases where the top of the stack is M, turn 180.
If it’s one of the cases above, pop the stack once and go back to investigating (looking at the top of the stack and seeing what that path entails).
If the top of the stack is “,” that means you are done investigating a sub intersection. Pop the maze stack twice (to get rid of the comma and the initial direction you turned to get into that branch. This means that the entire branch in that intersection is popped after it’s all been investigated. 
Switch to investigation mode. 

## Design Decisions

Maze Logic design decisions:

The states are necessary because there is no true multi-threading in python. A single state will run in a loop multiple times. This is why going forward is always nested in an if statement. It will repeatedly get triggered in that state until the next conditional is met.

The Neato will continue switching between these states indefinitely. Since it will end up investigating every potential path, it will eventually find the “dead end” that leads to solving the maze. Since dead ends are determined by having no paths in the left, middle, or right triggers, We could introduce more code into the dead end state that checks whether or not the dead end is actually the solution (say, a yellow flag in the top of the Neato’s camera), and then sends the mode to a new “solved” mode and stops the Neato.

It should be a flaw that the Neato can only see in front of it and cannot see beneath it. While creating some of the modes, we were satisfied when we were able to turn this into an advantage. Dead ends, for example, are pretty much always spot on for this reason.

We decided to store maze path information in a list that we are treating as a stack. This is because using lists enables dynamic sizing, unlike arrays. The stack structure enables the Neato to backtrack easily when it hits a dead end, and it prevents too much information from being stored because paths can easily be deleted after they have been explored. Handling sub-intersections—intersections that occur after other intersections—is much more manageable using stacks.

Maze path information is not stored in the global frame. Instead, it acts like a checklist, dictating the decisions the Neato should make next. We do not need to know how paths are oriented in the global frame since they only become relevant when the Neato is already at that specified location, and we know how the Neato perceives the paths at any given point in time.


Image processing design decisions:

We chose to use the OpenCV library instead of other options because it is well documented, there are lots of resources available online, and it works well with Python. Additionally, OpenCV covers a wide range of applications and gives us the flexibility to try multiple approaches if desired—there are algorithms for image masking, feature extraction, and everything else we are likely to need.

We implemented the color filter in the same node as the maze solver. This eliminates the need for communication between separate nodes, making it easier to access class attributes.

Finally, we chose to define the color range in HSV instead of RGB because that makes it easier to specify a single upper/lower bound; hue is represented by a single number. RGB would likely require more than those two bounds to specify the color range.

## Challenges

We faced several code-related challenges while working on this project. For example, we discovered early on that changes in the lighting—even seemingly small changes, such as the difference between classroom lighting and hallway lighting—cause colors to appear different to the Neato. A color which falls within range in one room may be filtered out in another room. Additionally, HSV colors are represented on different scales in different places, unlike how RGB is always represented as a set of integers ranging from 0 to 255. As we debugged the code, we had to refactor the structure of the code many times to accommodate for unexpected issues, such as the lack of true multithreading in Python. Finally, the Neato does not only have to understand how to navigate normal maze intersections, but also intersections that occur after other intersections—we refer to them as nested intersections or sub-intersections. Their presence makes it challenging to store all available paths because it greatly increases the number of possible trajectories the Neato could follow through the maze.

## Hypothetical Improvements and Limitations

A limitation is that this code only works for Acyclic mazes (mazes that only have one solution). This means no loops, since loops cause multiple solutions. The Neato will get stuck in the loop of creating and removing those intersections. Furthermore, our code only works if the maze paths are a uniform color that is sufficiently distinct from the floor color. If the floor contains any color that is too similar to the tape, the Neato could receive false positive readings for the presence of a path.

If we had more time, we would enable the Neato to solve a maze where there are two tape boundaries which it has to stay between. Currently, it can only solve mazes where the path is the tape itself.

The current iterative method for solving the maze works reliably but requires many lines of code. To improve the Neato’s capacity to solve the maze, we would implement a recursive algorithm so the Neato can handle nested intersections more efficiently.

We would also improve our project by coding for search-and-rescue applications. To do this, we would have the Neato traverse the maze until it finds a specific item which it identifies as its goal using a simple object recognition algorithm.

We would also have the Neato create a map of the maze it has explored using odometry data and info from the stack. This information could be imported into MatPlotLib to create a visual representation of every path the Neato has traversed, and the more it explores, the more of the maze map we will be able to see.

If we were to further extend the stretch goal, we would get the Neato to traverse a 3D maze where it is surrounded by vertical cardboard walls instead of the 2D tape lines we are currently using.

There are so many possibilities for expansions and perfecting logic, sensing data, and tweaking mechanisms in this project. The possibilities are almost endless.

**Short Reflection on Maze Logic:**

The Simple function works perfectly because it doesn’t immediately react when it senses a turn. The other functions should work but react too quickly. Sometimes the camera is tilted, causing the left path to touch the bounding box before the right path does (or vice versa). This causes the Neato to underestimate the number of paths and react incorrectly by changing to the wrong state. Improvements can be made by implementing a “double check” function that nudges forward, logs any new sensed paths, nudges backwards, then outputs the sensed paths. If the functions did this before making a decision, the rest of the functions would work very well.

**Standard and Probabilistic Hough Line Transform:**

A better way to improve the path detection would be to create lines on top of the paths in the image by using Hough Line Transforms, then analyzing which sides were triggered. We already implemented some of the code to accomplish this, but have not integrated it into the maze_vision node.

Hough Line Transforms work by using the polar equation associated with a given line. For any given point, there is an infinite set of lines which go through that point, and the parameters of those lines (the polar radius and angle) define a sinusoidal curve. For collinear points, these sinusoidal curves intersect, meaning that they can be used to locate lines within an image by treating each pixel as a cartesian coordinate.

OpenCV provides a simple way to implement this by calling the method cv2.HoughLines(args). Here is the output generated with a threshold of 200:


Maze intersection with binary mask applied and Hough Lines shown in purple
Arguments: threshold=200

![Hough lines superimposed on filtered image](/machine_vision/machine_vision/IntersectionHoughLines.jpg)

Maze intersection with binary mask applied and Probabilistic Hough Lines shown in purple
Arguments: threshold=50, minLineLength=5, maxLineGap=20

## Lessons Learned

From this project, we learned several lessons about good coding practices and working with machine vision algorithms.

First, official library documentation is a valuable resource. It helps to find relevant methods, example code, and specific applications of that code to reference.

Determine early on how to structure the code. Decide how many nodes to use, what methods each node should have, and what attributes should be accessible to the whole class. This reduces the possibility of scope-related errors in the future.

Sometimes the best way to work out an algorithm is by testing the logic on a real example case. For our project, this meant using a real maze to determine how the stack should be updated. Coding is about deciding how to approach a problem as much as it is about the syntax of any particular language.

Machine vision does not have to require fancy machine learning algorithms or neural networks. There are clever ways to process an image by breaking the task into its simplest components. In our case, using color really helped simplify the problem and achieve greater reliability.

The more complex a program is, the more necessary documentation becomes—in the form of in-line comments, docstrings, and even documentation of the thought processes that came before all of that, such as pseudocode.

## Resources Used

https://docs.opencv.org/4.x/d2/d96/tutorial_py_table_of_contents_imgproc.html 
https://www.geeksforgeeks.org/python/line-detection-python-opencv-houghline-method/
https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
https://www.tutorialspoint.com/how-to-mask-an-image-in-opencv-python 
https://www.geeksforgeeks.org/python/color-spaces-in-opencv-python/
https://www.geeksforgeeks.org/python/simple-thresholdin-using-opencv/ 



