<img width="640" height="670" alt="image" src="https://github.com/user-attachments/assets/86b7701c-d977-45c9-a9be-01efb682812a" />

This project was made using [WPILib's arm simulation example](https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation).
# Features
- Working simulation of an arm
   - You can plot your setpoints, speeds, and gains of an arm, alongside the drops
      - PID values are tunable, being preference keys in the network tables
   - You can simulate it with a visualizer (two arms, one representing the stand and one the actual arm/joint), AScope makes it smooth
- XBoxController binded buttons
   - Raising it up
   - Bringing to a setpoint
   - Letting it drop
   - Homing
# Specs
- One Kraken x60 motor
- CANcoder
- Constraints in Constants.java, from now on I'm going to move them into individual files to make merging easier
# Mechanics
- WPILib PID Controller
   - Feeds setpoint data -> pidController.calculate() which gives the right voltage needed
- WPILib FeedForward
   - FeedForward algorithm keeps it in place
   - Doesn't let it slack like brake mode does when it cant support the arm
# Future goals
- Utilize constraints from our algae intake from last year

# Image gallery
> 75* setpoint.<br>

<img width="640" height="670" alt="image" src="https://github.com/user-attachments/assets/fb7b8ff1-09ef-4e40-9a97-ba96ed9d5a7b" /><br>
> Homed

<img width="640" height="670" alt="image" src="https://github.com/user-attachments/assets/7012c3d0-3c63-4ac7-9111-d5bcc2f2b9f2" />
