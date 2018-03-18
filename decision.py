import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            if Rover.near_sample == True:
                print("Forward-Near Sample")
                Rover.brake = .2
                Rover.throttle = 0
            elif Rover.vel < 0.2: # We are "stuck" or "getting started"
                #print("*** WE NEED MORE POWER, Mr. Scott! ***")
                Rover.throttle = .8  # Were giv'n her all she's got Cap'n!
                Rover.brake = 0
            elif Rover.see_sample == True:
                print ((np.mean(Rover.nav_angles * 180/np.pi)))
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                Rover.throttle = 0
			# Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:			
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
				# Check for obstacle straight ahead
                stdDev = np.std(Rover.nav_angles) # Get stanard deviation of nav angles
                #print ("ang sd:",stdDev)
                if (stdDev > .46): # Large Std Dev - big obstacle straight ahead!
                    print("*** HARD RIGHT, Mr. Scott!")
                    Rover.steer = -14.5
                    Rover.throttle = 0
                    Rover.brake = 5
                else:
                    # Set steering to average angle clipped to the range +/- 15
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    # Give the Rover a "left turning bias" to tend to stick more to the left side                     
                    # whenever the Rover is moving "at speed".
                    if ((Rover.vel > .8) and (Rover.steer < 4.2)):
                        Rover.steer = Rover.steer + 4.8
                #print(Rover.steer)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            if Rover.near_sample == True:
                print("Stop-Near Sample")
            #print("stop")
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

