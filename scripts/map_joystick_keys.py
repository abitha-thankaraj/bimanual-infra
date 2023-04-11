import pygame
import argparse

def test_joystick(joystick_number):
    print("Initializing joystick... Press and key on keyboard to exit.")
    pygame.init()

    joy = pygame.joystick.Joystick(joystick_number)
    joy.init()

    try:
        while True:
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.JOYAXISMOTION:
                    print(event.dict, event.joy, event.axis, event.value)
                elif event.type == pygame.JOYBALLMOTION:
                    print(event.dict, event.joy, event.ball, event.rel)
                elif event.type == pygame.JOYBUTTONDOWN:
                    print(event.dict, event.joy, event.button, 'pressed')
                elif event.type == pygame.JOYBUTTONUP:
                    print(event.dict, event.joy, event.button, 'released')
                elif event.type == pygame.JOYHATMOTION:
                    print(event.dict, event.joy, event.hat, event.value)


    except KeyboardInterrupt:
        print("Exiting joystick test.")
        joy.quit()
        pygame.quit()

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--joystick_number", type=int, default=0, help="Joystick number to use") 
    args = parser.parse_args()

    test_joystick(args.joystick_number)        
