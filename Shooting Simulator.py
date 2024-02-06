import pygame
import math

pygame.init()
window = pygame.display.set_mode((1000, 500))
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 50)

# robot variables
xPos = 900
yPos = 400 
robotSize = (50, 50)

#shooter variables
lockedJoint = [xPos + 5, yPos + 5] # center joint thats on robot
shooterAngle = 0

xTranslation = 30 * math.cos((15 + shooterAngle) * math.pi / 180) # Initially shooter is facing straight forward
yTranslation = 30 * math.sin((15 + shooterAngle) * math.pi / 180) # So these translations are simple and equal

upperJoint = [lockedJoint[0] - xTranslation, lockedJoint[1] - yTranslation] # upper part of shooter
lowerJoint = [lockedJoint[0] - xTranslation, lockedJoint[1] + yTranslation] # lower part of shooter

#ring variables
orange = (255, 127, 80)

upperLeft = [lockedJoint[0] -30, lockedJoint[1] -4] # Initially Upper Left Part of Ring
lowerLeft = [lockedJoint[0] -30, lockedJoint[1] +4] # Initially Lower Left Part of Ring
upperRight = [lockedJoint[0] -10, lockedJoint[1] -4] # Initially Upper Right Part of Ring
lowerRight = [lockedJoint[0] -10, lockedJoint[1] +4] # Initially Lower Right Part of Ring

ringVelocity = 80
hasShot = False
lastLockedJoint = lockedJoint
lastShooterAngle = shooterAngle

manualShooting = False
p1X = 100
p1Y = 350

time = 0

def updateJointPositions():
    # For future updates we can just add or subtract from the "shooterAngle" based on whether its upper or lower
    # To get accurate joint locations
    upperXTranslation = 30 * math.cos((15 + shooterAngle) * math.pi / 180)
    upperYTranslation = 30 * math.sin((15 + shooterAngle) * math.pi / 180)
    lowerXTranslation = 30 * math.cos((-15 + shooterAngle) * math.pi / 180)
    lowerYTranslation = 30 * math.sin((-15 + shooterAngle) * math.pi / 180)
    
    # Because of this switch we can just "add" the translations (In reality we subtract since top left is 0,0 for pygame)
    upperJoint = [lockedJoint[0] - upperXTranslation, lockedJoint[1] - upperYTranslation]
    lowerJoint = [lockedJoint[0] - lowerXTranslation, lockedJoint[1] - lowerYTranslation]

    return (upperJoint, lowerJoint)

def ringUpdatePosition():
    # Changing ring position
    upperLeftXTranslation = math.sqrt( 30*30 + 4*4) * math.cos((7.64 + shooterAngle) * math.pi / 180)
    upperLeftYTranslation = math.sqrt( 30*30 + 4*4) * math.sin((7.64 + shooterAngle) * math.pi / 180)
    lowerLeftXTranslation = math.sqrt( 30*30 + 4*4) * math.cos((-7.64 + shooterAngle) * math.pi / 180)
    lowerLeftYTranslation = math.sqrt( 30*30 + 4*4) * math.sin((-7.64 + shooterAngle) * math.pi / 180)
    upperRightXTranslation = math.sqrt( 10*10 + 4*4) * math.cos((21.80 + shooterAngle) * math.pi / 180)
    upperRightYTranslation = math.sqrt( 10*10 + 4*4) * math.sin((21.80 + shooterAngle) * math.pi / 180)
    lowerRightXTranslation = math.sqrt( 10*10 + 4*4) * math.cos((-21.80 + shooterAngle) * math.pi / 180)
    lowerRightYTranslation = math.sqrt( 10*10 + 4*4) * math.sin((-21.80 + shooterAngle) * math.pi / 180)

    upperLeft = [lockedJoint[0] - upperLeftXTranslation, lockedJoint[1] - upperLeftYTranslation]
    lowerLeft = [lockedJoint[0] - lowerLeftXTranslation, lockedJoint[1] - lowerLeftYTranslation]
    upperRight = [lockedJoint[0] - upperRightXTranslation, lockedJoint[1] - upperRightYTranslation]
    lowerRight = [lockedJoint[0] - lowerRightXTranslation, lockedJoint[1] - lowerRightYTranslation]

    return (upperLeft, lowerLeft, upperRight, lowerRight)

def calculateArcLengths(vel, angle):
    distance = vel * vel * math.cos(angle*math.pi/180) * math.sin(angle*math.pi/180) / 4.9
    height = (5/98) * vel * vel * (math.sin(angle*math.pi/180) ** 2)

    return (distance, height)

def projectilePosition(lockedJoint, time, ringVel):

    velX = ringVel * math.cos(lastShooterAngle*math.pi/180)
    velY = ringVel * math.sin(lastShooterAngle*math.pi/180) - 9.8 * time

    theta = math.atan2(velY, velX)

    distance = math.sqrt(15**2 + 4**2)

    thetaInitial = math.atan2(4, 15)

    leftUpperXTranslation = distance * math.cos(theta + thetaInitial)
    leftLowerXTranslation = distance * math.cos(theta - thetaInitial)
    leftLowerYTranslation = distance * math.sin(theta - thetaInitial)
    leftUpperYTranslation = distance * math.sin(theta + thetaInitial)

    rightUpperXTranslation = distance * math.cos(theta - math.pi - thetaInitial)
    rightLowerXTranslation = distance * math.cos(theta - math.pi + thetaInitial)
    rightLowerYTranslation = distance * math.sin(theta - math.pi + thetaInitial)
    rightUpperYTranslation = distance * math.sin(theta - math.pi - thetaInitial)

    deltaY = ringVel*math.sin(lastShooterAngle*math.pi/180) * time - 4.9 * time * time
    deltaX = ringVel*math.cos(lastShooterAngle*math.pi/180) * time

    upperLeft = [lockedJoint[0] - deltaX - leftUpperXTranslation, lockedJoint[1] - deltaY - leftUpperYTranslation]
    lowerLeft = [lockedJoint[0] - deltaX - leftLowerXTranslation, lockedJoint[1] - deltaY - leftLowerYTranslation]
    upperRight = [lockedJoint[0] - deltaX - rightUpperXTranslation, lockedJoint[1] - deltaY - rightUpperYTranslation]
    lowerRight = [lockedJoint[0] - deltaX - rightLowerXTranslation, lockedJoint[1] - deltaY - rightLowerYTranslation]

    return (upperLeft, lowerLeft, upperRight, lowerRight)


def calculateGoalVelocity(length, height, theta):
    vel = math.sqrt( (-4.9 * (length ** 2) / (math.cos(theta * math.pi / 180) ** 2) ) / (height - length * math.tan(theta * math.pi / 180)) )
    return vel
    
    

purple = (128, 0, 128)

run = True
while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

    window.fill((211, 211, 211))

    keys = pygame.key.get_pressed()

    if(keys[pygame.K_a]):
        xPos -= 10
        lockedJoint[0] = lockedJoint[0] - 10
        

    if(keys[pygame.K_d]):
        xPos += 10
        lockedJoint[0] = lockedJoint[0] + 10

    if(keys[pygame.K_w]):
        shooterAngle += 5

    if(keys[pygame.K_s]):
        shooterAngle -= 5

    if(keys[pygame.K_SPACE]):
        lastLockedJoint = lockedJoint.copy()
        lastShooterAngle = shooterAngle
        hasShot = True

    if(keys[pygame.K_r]):
        hasShot = False


    upperJoint, lowerJoint = updateJointPositions()

    if(not hasShot):
        time = 0
        upperLeft, lowerLeft, upperRight, lowerRight = ringUpdatePosition()

    elif(hasShot):
        time += 0.5
        if(manualShooting):
            upperLeft, lowerLeft, upperRight, lowerRight = projectilePosition(lastLockedJoint, time, ringVelocity)
        else:
            goalVel = calculateGoalVelocity( abs(p1X - lastLockedJoint[0]), abs(p1Y - lastLockedJoint[1]) , lastShooterAngle)
            print(goalVel)
            upperLeft, lowerLeft, upperRight, lowerRight = projectilePosition(lastLockedJoint, time, goalVel)

    pygame.draw.line(window, (0, 0, 0), (0, 450), (1000, 450), 3)

    pygame.draw.circle(window, (0, 255, 0), (p1X, p1Y), 5, 3)
    pygame.draw.rect(window, purple, ((xPos, yPos), robotSize), 3)
    pygame.draw.polygon(window, (0, 139, 139), ((lockedJoint), (upperJoint), (lowerJoint)), 4)
    pygame.draw.polygon(window, (255, 0, 0), (upperLeft, lowerLeft, lowerRight, upperRight), 4)


    pygame.display.flip()
    clock.tick(10)

pygame.quit()
exit()