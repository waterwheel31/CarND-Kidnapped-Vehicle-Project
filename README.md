# Kidnap Vehicle Project


![image](./result.gif)


## Objective 

- Locate current position of own vehicle from following data
    - landmarks on map
    - observations of the landmarks from the vehicle 

## Approach 

- Use "Particle Filter" to locate (used 50 partilces) 


## Results 

- The filter locates the position with limited errors
- See the gif image above 


## How to Run

- This uses a Unity enviornment on Udacity VM. THerefore, to run, first need to prepare the enviornment by doing (TBD)
- Then run following codes 
    - ./install-ubuntu.sh
    - mkdir build & cd build
    - cmake .. & make
    - ./particle_filter 