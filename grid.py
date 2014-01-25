# Sample Python/Pygame Programs
# Simpson College Computer Science
# http://cs.simpson.edu
 
import pygame
#from quadtree import Node
from bot import Bot
 
# Define some colors
black    = (   0,   0,   0)
white    = ( 255, 255, 255)
green    = (   0, 255,   0)
red      = ( 255,   0,   0)

# Side length of grid (in cells)
cells = 50
# Side length of cell (in pixels)
side = 15

# This sets the margin between each cell
margin=0

# Create grid
grid=[]
for row in range(cells):
    # Add an empty array that will hold each cell
    # in this row
    grid.append([])
    for column in range(cells):
        grid[row].append(0) # Append a cell
 
pygame.init()

bot = Bot(3,3, cells)
botmovecount = 1

# Set the height and width of the screen
size=[cells*side,cells*side]
screen=pygame.display.set_mode(size)
 
# Set title of screen
pygame.display.set_caption("Patching Visualization")
 
# Loop until the user clicks the close button.
done=False

# Keep track of mouse state
mouseDown = False
erasing = False

# Keep track of brush state (i.e. what we're painting)
brush = 1
 
# Used to manage how fast the screen updates
clock=pygame.time.Clock()

# Select the font to use. Default font, 25 pt size.
font = pygame.font.Font(None, 20)
 
# -------- Main Program Loop -----------
while done==False:
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_1: brush = 1
            if event.key == pygame.K_2: brush = 2
            if event.key == pygame.K_3: brush = 3
            if event.key == pygame.K_4: brush = 4
            if event.key == pygame.K_5: brush = 5
            if event.key == pygame.K_6: brush = 6
            if event.key == pygame.K_7: brush = 7
            if event.key == pygame.K_8: brush = 8
            if event.key == pygame.K_9: brush = 9
        if event.type == pygame.MOUSEBUTTONDOWN:
            mouseDown = True
            pos = pygame.mouse.get_pos()
            column = pos[0]//(side+margin)
            row = pos[1]//(side+margin)
            #if grid[row][column] != 0:  # use for brush-agnostic erasing
            if grid[row][column] == brush:
                erasing = True
        if event.type == pygame.MOUSEBUTTONUP:
            mouseDown = False
            erasing = False
        if mouseDown:
            # User clicks the mouse. Get the position
            pos = pygame.mouse.get_pos()
            column = pos[0] // (side+margin)
            row = pos[1] // (side+margin)
            if erasing:
                grid[row][column] = 0
            else:
                grid[row][column] = brush
            #print("Click ",pos,"Grid coordinates: ",row,column)
 
    # Set the screen background
    screen.fill(black)

    # Draw the grid
    for row in range(cells):
        for column in range(cells):
            color = white
            if grid[row][column] == 1:
                color = black
            if grid[row][column] > 1:
                color = red
            pygame.draw.rect(screen,color,[(margin+side)*column+margin,(margin+side)*row+margin,side,side])
          
    # draw room labels
    for row in range(cells):
        for col in range(cells):
            if grid[row][col] > 1:
                text = font.render(chr(65+grid[row][col]-2),True,black)
                screen.blit(text, [col*side,row*side])
   
    # helper things for bot
    bx,by = bot.getLoc()
    flatgrid = [i for l in grid for i in l]
    gridpts = [(x,y) for y in range(cells) for x in range(cells)]
    flatgrid = zip(gridpts,flatgrid)

    # make array of (loc, val) tuples for all goals in grid
    goals = [(l,v) for (l,v) in flatgrid if v > 1]
    bot.setGoals(goals)

    # make array of bot surroundings...'lasers' in cardinal directions
    botRange = 4
    surr = []
    #consider drawing botrange? (showing horizontal, vertical, etc. 'beams')
    # Generate points to analyze in surroundings
    
    for i in range(botRange):
        surr.append((bx+i,by))
        surr.append((bx-i,by))
        surr.append((bx,by+i))
        surr.append((bx,by-i))
        surr.append((bx-i,by-i))
        surr.append((bx+i,by-i))
        surr.append((bx-i,by+i))
        surr.append((bx+i,by+i))
    
    # filter out out of range and duplicate points, then zip in grid values
    surr = list(set(filter(lambda (x,y): 0<=x<cells and 0<=y<cells, surr)))
    sensor = [i for i in surr]  # copy for drawing purposes
    surr = [((x,y),grid[int(x)][int(y)]) for (x,y) in surr]
    # then sort by value, giving priority to 'occupied' cells
    surr.sort(key=lambda tup: tup[1])
    #print surr
    # tell bot what's up
    #bot.update(surr)
    print flatgrid
    bot.update([(l,v) for (l,v) in flatgrid if v > 0]) # cheat! tell bot everything

    s = pygame.Surface((side,side)) 
    s.set_alpha(128)            
    # draw sensor
    s.fill((125,125,125))
    for (x,y) in sensor:
        pygame.display.get_surface().blit(s, (x*side,y*side))
    # draw bot
    s.fill((0,0,255))
    pygame.display.get_surface().blit(s, (bx*side,by*side))

    # move bot for next update
    moveRate = 1
    botmovecount += 1
    if botmovecount%moveRate == 0:
        botmovecount = 1
        #bot.update(surr)  # tell bot what's up
        m = bot.getNextMove()
        if m:
            bot.setLoc((bx+m[0], by+m[1]))
        

    clock.tick(20)
    pygame.display.flip()
     
# Be IDLE friendly. If you forget this line, the program will 'hang'
# on exit.
pygame.quit ()
