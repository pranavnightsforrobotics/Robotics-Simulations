import pygame
import math

pygame.init()

windowWidth = 1500
windowHeight = 900

window = pygame.display.set_mode((windowWidth, windowHeight))
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 50)

# field variables

sourceLowHeight = 365
sourceHighHeight = 427

ringHeight = 20
ringLength = 140

ampBottom = 260
ampTop = 440
ampDist = 34

# robot variables

shoulderAngle = 90
wristAngle = 90

armLength = 250

shoulderJoint = [windowWidth / 2.0 - 105, windowHeight-210]
sholderJointToGearRadius = 34
wristJoint = [windowWidth / 2.0 - 105, shoulderJoint[1] - armLength]

L2x = 32.8
#L2x = 52.8
L2y = 76.6
L2 = math.sqrt(math.pow(L2x, 2) + math.pow(L2y, 2))

L3x = 131.1
#L3x = 111.1
L3y = 58.6
L3 = math.sqrt(math.pow(L3x, 2) + math.pow(L3y, 2))


thetaL2 = (math.atan2(L2y, L2x) * 180.0 / math.pi)
thetaL3 = (math.atan2(L3y, L3x) * 180.0 / math.pi)

shooterBottom = [wristJoint[0] - L2x, wristJoint[1]]
shooterTop = [wristJoint[0] - L2x, wristJoint[1] - L2y]

intakeBottom = [wristJoint[0] + L3x, wristJoint[1]]
intakeTop = [wristJoint[0] + L3x, wristJoint[1] - L3y]

def calculateIntakeWristAngle(curwristAngle, shChange):
    
    if( (intakeBottom[0] - shoulderJoint[0]) > (intakeTop[0] - shoulderJoint[0]) ):
        intakeEndToShoulderXOffset = intakeBottom[0] - shoulderJoint[0]
    
    elif( (intakeBottom[0] - shoulderJoint[0]) < (intakeTop[0] - shoulderJoint[0]) ):
        intakeEndToShoulderXOffset = intakeTop[0] - shoulderJoint[0]    

    if(intakeEndToShoulderXOffset > 365):
        desiredWristToIntake = windowWidth / 2.0 + 260 - wristJoint[0]

        if((intakeBottom[0] - shoulderJoint[0]) < (intakeTop[0] - shoulderJoint[0])):
            intakeTopWristAngle = -math.acos(desiredWristToIntake / L3) * 180.0 / math.pi
            theta4calc = intakeTopWristAngle - thetaL3 + 180
            wristAnglecalc = 180 - theta4calc + shoulderAngle

        elif((intakeBottom[0] - shoulderJoint[0]) > (intakeTop[0] - shoulderJoint[0])):
            intakeBottomWristAngle = math.acos(desiredWristToIntake / L3x) * 180.0 / math.pi
            theta4calc = intakeBottomWristAngle + 180
            wristAnglecalc = 180 - theta4calc + shoulderAngle
                   
        return wristAnglecalc 


    else:
        return curwristAngle



run = True
while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

    window.fill((211, 211, 211))

    keys = pygame.key.get_pressed()
    shoulderPast = shoulderAngle

    if(keys[pygame.K_a]):
        shoulderAngle += 1
        
    if(keys[pygame.K_d]):
        shoulderAngle -= 1
    
    if(keys[pygame.K_LEFT]):
        wristAngle -= 1
    
    if(keys[pygame.K_RIGHT]):
        wristAngle += 1

    if(keys[pygame.K_SPACE]):
        # check if shoulder is raising or dropping to get viable solution
        shoulderChange = shoulderAngle-shoulderPast
        wristAngle = calculateIntakeWristAngle(wristAngle, shoulderChange)


    if(keys[pygame.K_p]):
        shoulderAngle = 66.58 - 90

    theta4 = 180 + shoulderAngle - wristAngle
    absThetaL2 = theta4 - thetaL2
    absThetaL3 = theta4 - 180 + thetaL3

    wristJoint[0] = (math.cos(shoulderAngle * math.pi / 180.0) * armLength) + shoulderJoint[0] 
    wristJoint[1] = shoulderJoint[1] - (math.sin(shoulderAngle * math.pi / 180.0) * armLength)

    shooterTop[0] = wristJoint[0] + L2 * math.cos(absThetaL2 * math.pi / 180.0)
    shooterTop[1] = wristJoint[1] - L2 * math.sin(absThetaL2 * math.pi / 180.0)
    
    shooterBottom[0] = wristJoint[0] + L2x * math.cos(theta4 * math.pi / 180.0)
    shooterBottom[1] = wristJoint[1] - L2x * math.sin(theta4 * math.pi / 180.0)

    intakeTop[0] = wristJoint[0] + L3 * math.cos(absThetaL3 * math.pi / 180.0)
    intakeTop[1] = wristJoint[1] - L3 * math.sin(absThetaL3 * math.pi / 180.0)

    intakeBottom[0] = wristJoint[0] + L3x * math.cos( (theta4-180) * math.pi / 180.0)      
    intakeBottom[1] = wristJoint[1] - L3x * math.sin( (theta4-180) * math.pi / 180.0)


    # Space Limit
    pygame.draw.rect(window, (0,0,0), ((windowWidth / 2.0 - 260, windowHeight - 480), (520, 480)), 3)

    # Amp
    #pygame.draw.rect(window, (0,200,200), ((windowWidth / 2.0 - 204, windowHeight-ampTop), (ampDist, ampTop-ampBottom)), 3)
    #pygame.draw.rect(window, (0,200,200), ((windowWidth / 2.0 + 170, windowHeight-ampTop), (ampDist, ampTop-ampBottom)), 3)

    # Source
    #pygame.draw.line(window, (0, 200, 200), (windowWidth / 2.0 + 170, windowHeight-sourceHighHeight), (windowWidth / 2.0 + 170, windowHeight-sourceLowHeight), 3)
    #pygame.draw.line(window, (0, 200, 200), (windowWidth / 2.0 + 170, windowHeight-sourceHighHeight), (windowWidth / 2.0 + 290, windowHeight-sourceHighHeight-156), 3)
    #pygame.draw.line(window, (0, 200, 200), (windowWidth / 2.0 + 170, windowHeight-sourceLowHeight), (windowWidth / 2.0 + 290, windowHeight-sourceLowHeight-156), 3)
    #pygame.draw.line(window, (0, 200, 200), (windowWidth / 2.0 + 290, windowHeight-sourceHighHeight-156), (windowWidth / 2.0 + 290, windowHeight-sourceLowHeight-156), 3)

    # Ring
    pygame.draw.rect(window, (255,127,0), ((windowWidth / 2.0 + 200, windowHeight-ringHeight), (ringLength, ringHeight)), 3)  
 
    # Bumpers
    pygame.draw.rect(window, (181,148,15), ((windowWidth / 2.0 - 170, windowHeight - 57.5), (340, 57.5)), 3)

    # base to shoulder
    pygame.draw.line(window, (160, 32, 240), shoulderJoint, (shoulderJoint[0], windowHeight-20), 3)

    # shoulder radius
    pygame.draw.circle(window, (32,32,32), shoulderJoint, sholderJointToGearRadius, 3)

    # shoulder to wrist
    pygame.draw.line(window, (160, 32, 240), shoulderJoint, wristJoint, 3)

    # wrist to shooter bottom / top
    pygame.draw.line(window, (160, 32, 240), wristJoint, shooterBottom, 3)
    pygame.draw.line(window, (160, 32, 240), shooterBottom, shooterTop, 3)

    # wrist to intake bottom / top
    pygame.draw.line(window, (160, 32, 240), wristJoint, intakeBottom, 3)
    pygame.draw.line(window, (160, 32, 240), intakeBottom, intakeTop, 3)

    # projectile straight path
    initialPoint = [ ( shooterTop[0] + shooterBottom[0] ) / 2.0 , ( shooterTop[1] + shooterBottom[1] ) / 2.0]
    finalPoint = [ 1000 * math.cos(theta4 * math.pi / 180.0) + initialPoint[0] , -1000 * math.sin(theta4 * math.pi / 180.0) + initialPoint[1]]

    pygame.draw.line(window, (255, 128, 0), initialPoint, finalPoint, 3)  

    pygame.display.flip()
    clock.tick(50)

pygame.quit()
exit()