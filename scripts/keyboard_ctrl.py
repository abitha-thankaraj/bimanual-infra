# import pygame
# import os
# import sys
# os.environ["SDL_VIDEODRIVER"] = "dummy"


# def handle_arrow_key_events(event, arrow_key_states):
#     if event.type == pygame.KEYDOWN:
#         if event.key == pygame.K_LEFT:
#             arrow_key_states['left'] = True
#             print("Left key pressed")
#         if event.key == pygame.K_RIGHT:
#             arrow_key_states['right'] = True
#             print("Right key pressed")
#         if event.key == pygame.K_UP:
#             arrow_key_states['up'] = True
#             print("Up key pressed")
#         if event.key == pygame.K_DOWN:
#             arrow_key_states['down'] = True
#             print("Down key pressed")
#     elif event.type == pygame.KEYUP:
#         if event.key == pygame.K_LEFT:
#             arrow_key_states['left'] = False
#             print("Left key released")
#         if event.key == pygame.K_RIGHT:
#             arrow_key_states['right'] = False
#             print("Right key released")
#         if event.key == pygame.K_UP:
#             arrow_key_states['up'] = False
#             print("Up key released")
#         if event.key == pygame.K_DOWN:
#             arrow_key_states['down'] = False
#             print("Down key released")

# # Initialize pygame
# pygame.init()

# # # Set up the display
# # screen = pygame.display.set_mode((640, 480))
# # pygame.display.set_caption("Arrow Key Press Detection")

# # Define initial arrow key states
# arrow_key_states = {
#     'left': False,
#     'right': False,
#     'up': False,
#     'down': False,
# }

# # Main game loop
# running = True
# while running:
#     # Handle events
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False

#         handle_arrow_key_events(event, arrow_key_states)

#     # Print arrow key states
#     # print(arrow_key_states)

#     # Clear the screen
#     # screen.fill((0, 0, 0))

#     # Update the display
#     # pygame.display.flip()

# # Clean up and quit
# pygame.quit()

import pygame
import time
import typing
from multiprocessing import Queue
from bimanual.hardware.scratch_robot.xarmrobot import CartesianMoveMessage
def keyboard_control(queue:Queue):
    rel_pose = [0, 0, 0, 0, 0, 0]        
    pygame.init()
    screen = pygame.display.set_mode((100, 100))
    screen.fill("gold")
    pygame.display.update()
    running = True

    latest_ts = time.time()
    send_freq = 50

    while running:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN: 
                print("Keydown")
                if event.key == pygame.K_ESCAPE: 
                    print("Escape key")
                    running = False
                elif event.key == pygame.K_UP:
                    rel_pose[0] = 1
                    print("Left key pressed")
                elif event.key == pygame.K_DOWN:
                    rel_pose[0] = -1
                    print("Left key pressed")
                elif event.key == pygame.K_LEFT:
                    rel_pose[1] = -1
                    print("Left key pressed")
                elif event.key == pygame.K_RIGHT:
                    rel_pose[1] = 1
                    print("Left key pressed")
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_LEFT:
                    rel_pose[1] = 0
                    print("Left key released")
                elif event.key == pygame.K_RIGHT:
                    rel_pose[1] = 0
                    print("Left key released")
                elif event.key == pygame.K_UP:
                    rel_pose[0] = 0
                    print("Left key pressed")
                elif event.key == pygame.K_DOWN:
                    rel_pose[0] = 0
                    print("Left key pressed")
        if time.time() - latest_ts > (1/send_freq):
            print("Sending to queue")
            if sum(rel_pose) ==0:
                continue
            print("Adding to queue")
            queue.put(CartesianMoveMessage(target=rel_pose, wait=False, relative=True))
            latest_ts = time.time()
