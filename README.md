# Astar on CPP

This is a project used on Drone path planning. Followed by a path smoothing module based on Bezier Curve. Dynamic constraints are considered.

### Dependencies:

1. OpenGL Mathematics (glm)

    This is just a mathematical library for 3D objects. I should have replaced it with a more commonly used library such like `Eigen`. If I have time, I will finish the transformation.

  ```bash
  sudo apt-get libglm-dev
  ```

2. Plotly

    In `/dataScripts` there is a ready to use path visualization script written in R language with a 3D ploting library called `Plotly`.This library provides an interactive 3D data visualization result based on JavaScript. 
    
    You can also generate a html version of the visualization making use of `RStudio` and the feature of `Rmd`.